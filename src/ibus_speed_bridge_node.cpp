#include "solo_usb_controller/dual_track_driver.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <memory>
#include <chrono>

class IBUSSpeedBridgeNode : public rclcpp::Node {
public:
    IBUSSpeedBridgeNode() : Node("ibus_speed_bridge") {
        // Get device paths from parameters or use defaults
        // Default to ttyACM devices if udev symlinks don't exist
        std::string left_device = this->declare_parameter<std::string>("left_device", "/dev/ttyACM0");
        std::string right_device = this->declare_parameter<std::string>("right_device", "/dev/ttyACM1");
        bool invert_left = this->declare_parameter<bool>("invert_left", false);
        bool invert_right = this->declare_parameter<bool>("invert_right", true);
        max_speed_rad_s_ = this->declare_parameter<double>("max_speed_rad_s", 20.94);
        
        // Configure driver
        solo::DualTrackDriver::Config config;
        config.left_device = left_device;
        config.right_device = right_device;
        config.invert_left = invert_left;
        config.invert_right = invert_right;
        config.max_speed_rad_s = static_cast<float>(max_speed_rad_s_);
        
        driver_ = std::make_unique<solo::DualTrackDriver>(config);
        
        // Connect to motors
        RCLCPP_INFO(this->get_logger(), "Connecting to motors...");
        RCLCPP_INFO(this->get_logger(), "  Left:  %s", left_device.c_str());
        RCLCPP_INFO(this->get_logger(), "  Right: %s", right_device.c_str());
        
        if (!driver_->connect()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect: %s", driver_->getLastError().c_str());
            throw std::runtime_error("Failed to connect to motors");
        }
        
        RCLCPP_INFO(this->get_logger(), "Connected successfully!");
        
        // Configure motors (same as TUI does)
        RCLCPP_INFO(this->get_logger(), "Configuring motors...");
        
        // Clear faults first
        for (int i = 0; i < 3; i++) {
            driver_->clearFaults();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        // Configure left motor
        RCLCPP_INFO(this->get_logger(), "Configuring left motor...");
        driver_->left().setMotorType(1);  // BLDC_PMSM
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        driver_->left().setFeedbackControlMode(1);  // ENCODERS
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        driver_->left().setCommandMode(1);  // DIGITAL
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        driver_->left().setControlMode(solo::ControlMode::SPEED);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        
        // Configure right motor
        RCLCPP_INFO(this->get_logger(), "Configuring right motor...");
        driver_->right().setMotorType(1);  // BLDC_PMSM
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        driver_->right().setFeedbackControlMode(1);  // ENCODERS
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        driver_->right().setCommandMode(1);  // DIGITAL
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        driver_->right().setControlMode(solo::ControlMode::SPEED);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        
        // Clear faults again after configuration
        driver_->clearFaults();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // Enable motors
        RCLCPP_INFO(this->get_logger(), "Enabling motors...");
        if (!driver_->enableMotors(true)) {
            RCLCPP_WARN(this->get_logger(), "Failed to enable motors: %s", driver_->getLastError().c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "Motors enabled");
        }
        
        // Subscribe to IBUS channels (using Float32 to match existing publisher)
        speed_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/ibus/ch2",
            10,
            std::bind(&IBUSSpeedBridgeNode::speed_callback, this, std::placeholders::_1)
        );
        
        steering_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/ibus/ch1",
            10,
            std::bind(&IBUSSpeedBridgeNode::steering_callback, this, std::placeholders::_1)
        );
        
        RCLCPP_INFO(this->get_logger(), "Subscribed to:");
        RCLCPP_INFO(this->get_logger(), "  /ibus/ch2 - Speed (-1.0 to 1.0)");
        RCLCPP_INFO(this->get_logger(), "  /ibus/ch1 - Steering (-1.0 to 1.0, 0=straight)");
        RCLCPP_INFO(this->get_logger(), "Ready to receive commands");
        
        // Create timer for periodic updates (SOLO needs periodic commands)
        update_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // 10 Hz
            std::bind(&IBUSSpeedBridgeNode::update_motors, this)
        );
    }
    
    ~IBUSSpeedBridgeNode() {
        if (driver_) {
            driver_->emergencyStop();
            driver_->disconnect();
        }
    }

private:
    void speed_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        // Clamp speed to [-1.0, 1.0] range
        double speed_normalized = std::max(-1.0, std::min(1.0, static_cast<double>(msg->data)));
        
        // Convert to rad/s
        current_speed_rad_s_ = speed_normalized * max_speed_rad_s_;
        
        RCLCPP_DEBUG(this->get_logger(), 
            "Speed command: %.3f (%.1f%%) -> %.2f rad/s", 
            speed_normalized, 
            speed_normalized * 100.0,
            current_speed_rad_s_);
    }
    
    void steering_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        // Clamp steering to [-1.0, 1.0] range
        // -1.0 = full left, 0.0 = straight, 1.0 = full right
        current_turn_ratio_ = std::max(-1.0, std::min(1.0, static_cast<double>(msg->data)));
        
        RCLCPP_DEBUG(this->get_logger(), 
            "Steering command: %.3f", current_turn_ratio_);
    }
    
    void update_motors() {
        // Continuously send speed and steering commands to motors
        if (driver_ && driver_->isConnected()) {
            bool success = false;
            
            // Use arcTurn for differential control (speed + steering)
            // If steering is near zero, use driveStraight for efficiency
            if (std::abs(current_turn_ratio_) < 0.01) {
                // Straight driving
                success = driver_->driveStraight(static_cast<float>(current_speed_rad_s_));
            } else {
                // Turning: use arcTurn with speed and turn ratio
                // Map steering [-1.0, 1.0] to turn_ratio [-1.0, 1.0]
                // Positive turn_ratio = right turn, negative = left turn
                success = driver_->arcTurn(
                    static_cast<float>(current_speed_rad_s_),
                    static_cast<float>(current_turn_ratio_)
                );
            }
            
            if (!success) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "Failed to send command: %s", driver_->getLastError().c_str());
            }
            
            // Check for faults
            if (driver_->hasAnyFault()) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "Fault detected on motors!");
            }
        }
    }
    
    std::unique_ptr<solo::DualTrackDriver> driver_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr steering_sub_;
    rclcpp::TimerBase::SharedPtr update_timer_;
    
    double current_speed_rad_s_ = 0.0;
    double current_turn_ratio_ = 0.0;  // -1.0 (left) to 1.0 (right), 0.0 = straight
    double max_speed_rad_s_ = 20.94;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<IBUSSpeedBridgeNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("ibus_speed_bridge"), "Exception: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}
