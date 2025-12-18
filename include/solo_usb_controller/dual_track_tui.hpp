#ifndef SOLO_USB_CONTROLLER_DUAL_TRACK_TUI_HPP
#define SOLO_USB_CONTROLLER_DUAL_TRACK_TUI_HPP

#include "dual_track_driver.hpp"
#include <ncurses.h>
#include <memory>
#include <string>
#include <fstream>
#include <chrono>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

namespace solo {

/**
 * @brief Terminal User Interface for dual-track vehicle control
 * 
 * Provides interactive control and monitoring of two SOLO motor controllers
 * configured for differential/tank drive.
 * 
 * Features:
 * - Side-by-side telemetry display for left/right motors
 * - Vehicle motion controls (forward, reverse, turn, rotate)
 * - Synchronized enable/disable
 * - Per-motor and combined fault monitoring
 * - CSV data logging
 * - Emergency stop
 * - Optional ROS2 topic control via /ibus/ch2 (speed) and /ibus/ch1 (steering)
 */
class DualTrackTUI {
public:
    explicit DualTrackTUI(const DualTrackDriver::Config& config = DualTrackDriver::Config(), 
                          bool enable_ros = false);
    ~DualTrackTUI();

    // Main application loop
    void run();

private:
    //=========================================================================
    // UI Management
    //=========================================================================
    void initUI();
    void shutdownUI();
    void drawUI();
    void drawHeader();
    void drawConnectionStatus();
    void drawControlPanel();
    void drawTelemetryPanel();
    void drawMotorColumn(int col, const std::string& label, const Telemetry& telemetry);
    void drawFaultPanel();
    void drawHelp();
    void drawStatusBar();
    
    //=========================================================================
    // Input Handling
    //=========================================================================
    void handleInput(int ch);
    
    // Vehicle motion commands
    void cmdForward();
    void cmdReverse();
    void cmdTurnLeft();
    void cmdTurnRight();
    void cmdRotateLeft();
    void cmdRotateRight();
    void cmdStop();
    
    // Speed adjustment
    void increaseSpeed();
    void decreaseSpeed();
    void increaseTurnRate();
    void decreaseTurnRate();
    
    // Motor control
    void toggleMotors();
    void emergencyStop();
    void clearFaults();
    
    // Logging
    void toggleLogging();
    bool openLogFile();
    void closeLogFile();
    void logData();
    
    //=========================================================================
    // ROS2 Topic Handlers
    //=========================================================================
    void initROS2();
    void shutdownROS2();
    void spinROS2();
    void speedCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void steeringCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void updateFromROS2Topics(std::chrono::steady_clock::time_point now);
    
    //=========================================================================
    // State
    //=========================================================================
    std::unique_ptr<DualTrackDriver> driver_;
    DualTrackDriver::Config config_;
    bool running_;
    bool motors_enabled_;
    
    // Current motion state
    float current_speed_;       // Current linear speed setpoint (rad/s)
    float current_turn_ratio_;  // Current turn ratio (-1 to +1)
    float speed_step_;          // Speed increment per keypress
    float turn_step_;           // Turn ratio increment per keypress
    
    // Telemetry cache
    Telemetry left_telemetry_;
    Telemetry right_telemetry_;
    bool telemetry_ok_;
    std::string telemetry_error_;
    
    //=========================================================================
    // UI State
    //=========================================================================
    int screen_height_;
    int screen_width_;
    bool show_help_;
    std::string status_message_;
    std::chrono::steady_clock::time_point last_update_;
    
    //=========================================================================
    // Logging
    //=========================================================================
    bool logging_enabled_;
    std::ofstream log_file_;
    std::chrono::steady_clock::time_point log_start_time_;
    
    //=========================================================================
    // ROS2 State
    //=========================================================================
    bool ros_enabled_;
    std::shared_ptr<rclcpp::Node> ros_node_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr speed_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr steering_sub_;
    std::mutex ros_mutex_;  // Protect ROS callback data
    float ros_speed_value_;      // Latest speed from /ibus/ch2 (-1 to 1)
    float ros_steering_value_;   // Latest steering from /ibus/ch1
    std::chrono::steady_clock::time_point last_ros_speed_time_;
    std::chrono::steady_clock::time_point last_ros_steering_time_;
    bool ros_speed_active_;      // True if we received speed data recently
    bool ros_steering_active_;   // True if we received steering data recently
    
    //=========================================================================
    // Configuration Constants
    //=========================================================================
    static constexpr float DEFAULT_SPEED_STEP = 2.0f;      // rad/s per keypress
    static constexpr float DEFAULT_TURN_STEP = 0.1f;       // turn ratio per keypress
    static constexpr float MAX_SPEED = 20.0f;              // rad/s
    static constexpr float MAX_TURN_RATIO = 1.5f;          // allow slight over-pivot
    static constexpr int UPDATE_RATE_MS = 100;             // 10 Hz display update
    static constexpr int ROS_TIMEOUT_MS = 500;             // Timeout for ROS topic data (ms)
};

} // namespace solo

#endif // SOLO_USB_CONTROLLER_DUAL_TRACK_TUI_HPP
