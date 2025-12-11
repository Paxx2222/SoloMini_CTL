#ifndef SOLO_USB_CONTROLLER_DUAL_TRACK_DRIVER_HPP
#define SOLO_USB_CONTROLLER_DUAL_TRACK_DRIVER_HPP

#include "solo_driver.hpp"
#include <string>
#include <memory>

namespace solo {

/**
 * @brief Dual motor driver for tracked vehicle control
 * 
 * Manages two SOLO motor controllers for differential drive / tank steering.
 * Motors are assumed to be physically mirrored on the chassis:
 * - Left motor: positive velocity = forward track motion
 * - Right motor: positive velocity = backward track motion (inverted in software)
 * 
 * Motion conventions:
 * - Forward: left track forward, right track forward
 * - Reverse: left track backward, right track backward
 * - Rotate CW (right): left forward, right backward (faster left)
 * - Rotate CCW (left): left backward, right forward (faster right)
 */
class DualTrackDriver {
public:
    struct Config {
        std::string left_device;
        std::string right_device;
        bool invert_left;      // Invert left motor direction
        bool invert_right;     // Invert right motor direction (typically true for mirrored mount)
        float max_speed_rad_s; // Maximum wheel speed
        
        Config() 
            : left_device("/dev/solo_left")
            , right_device("/dev/solo_right")
            , invert_left(false)
            , invert_right(true)  // Right motor typically inverted for mirrored mounting
            , max_speed_rad_s(20.94f)  // ~200 RPM
        {}
    };

    explicit DualTrackDriver(const Config& config = Config());
    ~DualTrackDriver();

    // Connection management
    bool connect();
    void disconnect();
    bool isConnected() const;
    
    // Enable/disable motors
    bool enableMotors(bool enable);
    bool areMotorsEnabled() const;

    //=========================================================================
    // Tracked Vehicle Motion Commands
    //=========================================================================
    
    /**
     * @brief Drive straight forward or backward
     * @param velocity_rad_s Wheel velocity in rad/s (positive = forward, negative = reverse)
     * @return true if both motors accepted the command
     */
    bool driveStraight(float velocity_rad_s);
    
    /**
     * @brief Rotate in place (zero-radius turn)
     * @param angular_rad_s Rotation rate (positive = clockwise/right, negative = counter-clockwise/left)
     * @return true if both motors accepted the command
     */
    bool rotateInPlace(float angular_rad_s);
    
    /**
     * @brief Arc turn with differential speeds
     * @param linear_vel Linear velocity component in rad/s
     * @param turn_ratio Turn ratio from -1.0 (hard left) to +1.0 (hard right), 0 = straight
     * @return true if both motors accepted the command
     * 
     * Turn ratio behavior:
     * - 0.0: Both tracks same speed (straight)
     * - 0.5: Inside track at 50% of outside track speed (gentle turn)
     * - 1.0: Inside track stopped (pivot turn)
     * - >1.0: Inside track reverses (tighter than pivot)
     */
    bool arcTurn(float linear_vel, float turn_ratio);
    
    /**
     * @brief Set individual track speeds directly
     * @param left_rad_s Left track velocity (positive = forward)
     * @param right_rad_s Right track velocity (positive = forward)
     * @return true if both motors accepted the command
     */
    bool setTrackSpeeds(float left_rad_s, float right_rad_s);
    
    /**
     * @brief Emergency stop - immediately stop both motors
     * @return true if both motors stopped
     */
    bool emergencyStop();
    
    /**
     * @brief Gradual stop - ramp down to zero
     * @return true if command sent
     */
    bool stop();

    //=========================================================================
    // Telemetry
    //=========================================================================
    
    /**
     * @brief Read telemetry from both motors
     * @param left Output telemetry for left motor
     * @param right Output telemetry for right motor
     * @return true if both reads succeeded
     */
    bool readTelemetry(Telemetry& left, Telemetry& right);
    
    /**
     * @brief Check if either motor has faults
     * @return true if any fault is active
     */
    bool hasAnyFault() const;
    
    /**
     * @brief Clear faults on both motors
     * @return true if both cleared successfully
     */
    bool clearFaults();

    //=========================================================================
    // Direct Motor Access
    //=========================================================================
    
    /** @brief Get reference to left motor driver */
    SoloDriver& left() { return *left_motor_; }
    const SoloDriver& left() const { return *left_motor_; }
    
    /** @brief Get reference to right motor driver */
    SoloDriver& right() { return *right_motor_; }
    const SoloDriver& right() const { return *right_motor_; }

    //=========================================================================
    // Configuration
    //=========================================================================
    
    /** @brief Set control mode for both motors */
    bool setControlMode(ControlMode mode);
    
    /** @brief Get current configuration */
    const Config& getConfig() const { return config_; }
    
    /** @brief Get last error message */
    std::string getLastError() const { return last_error_; }
    
    //=========================================================================
    // Speed Correction (Closed-Loop)
    //=========================================================================
    
    /**
     * @brief Enable/disable automatic speed correction
     * When enabled, compares commanded vs actual speed and applies corrections
     * @param enable True to enable correction
     */
    void enableSpeedCorrection(bool enable) { speed_correction_enabled_ = enable; }
    
    /** @brief Check if speed correction is enabled */
    bool isSpeedCorrectionEnabled() const { return speed_correction_enabled_; }
    
    /**
     * @brief Set proportional gain for speed correction
     * @param kp Proportional gain (default 0.5)
     */
    void setSpeedCorrectionGain(float kp) { speed_correction_kp_ = kp; }
    
    /** @brief Get current correction gain */
    float getSpeedCorrectionGain() const { return speed_correction_kp_; }

private:
    Config config_;
    std::unique_ptr<SoloDriver> left_motor_;
    std::unique_ptr<SoloDriver> right_motor_;
    std::string last_error_;
    
    Telemetry left_telemetry_;
    Telemetry right_telemetry_;
    
    // Speed correction (closed-loop)
    bool speed_correction_enabled_ = false;
    float speed_correction_kp_ = 0.5f;  // Proportional gain
    float left_speed_target_ = 0.0f;    // Target speed for left motor
    float right_speed_target_ = 0.0f;   // Target speed for right motor
    float left_correction_ = 0.0f;      // Accumulated correction for left
    float right_correction_ = 0.0f;     // Accumulated correction for right
    
    // Apply direction inversion based on config
    float applyLeftInversion(float velocity) const;
    float applyRightInversion(float velocity) const;
    
    // Clamp velocity to limits
    float clampVelocity(float velocity) const;
    
    // Apply speed correction based on telemetry feedback
    void applySpeedCorrection(float& left_cmd, float& right_cmd);
};

} // namespace solo

#endif // SOLO_USB_CONTROLLER_DUAL_TRACK_DRIVER_HPP
