#include "solo_usb_controller/dual_track_driver.hpp"
#include <algorithm>
#include <cmath>

namespace solo {

DualTrackDriver::DualTrackDriver(const Config& config)
    : config_(config)
{
    // Create serial configs for each motor
    SoloSerial::Config left_serial_config;
    left_serial_config.device = config_.left_device;
    
    SoloSerial::Config right_serial_config;
    right_serial_config.device = config_.right_device;
    
    left_motor_ = std::make_unique<SoloDriver>(left_serial_config);
    right_motor_ = std::make_unique<SoloDriver>(right_serial_config);
}

DualTrackDriver::~DualTrackDriver() {
    disconnect();
}

bool DualTrackDriver::connect() {
    bool left_ok = left_motor_->connect();
    if (!left_ok) {
        last_error_ = "Left motor: " + left_motor_->getLastError();
        return false;
    }
    
    bool right_ok = right_motor_->connect();
    if (!right_ok) {
        last_error_ = "Right motor: " + right_motor_->getLastError();
        left_motor_->disconnect();
        return false;
    }
    
    // Auto-detect addresses for both motors
    if (!left_motor_->autoDetectAddress()) {
        last_error_ = "Left motor: Failed to detect address";
        disconnect();
        return false;
    }
    
    if (!right_motor_->autoDetectAddress()) {
        last_error_ = "Right motor: Failed to detect address";
        disconnect();
        return false;
    }
    
    return true;
}

void DualTrackDriver::disconnect() {
    if (left_motor_) {
        left_motor_->disconnect();
    }
    if (right_motor_) {
        right_motor_->disconnect();
    }
}

bool DualTrackDriver::isConnected() const {
    return left_motor_ && right_motor_ && 
           left_motor_->isConnected() && right_motor_->isConnected();
}

bool DualTrackDriver::enableMotors(bool enable) {
    bool left_ok = left_motor_->enableMotor(enable);
    bool right_ok = right_motor_->enableMotor(enable);
    
    if (!left_ok) {
        last_error_ = "Left motor enable failed: " + left_motor_->getLastError();
    }
    if (!right_ok) {
        last_error_ = "Right motor enable failed: " + right_motor_->getLastError();
    }
    
    return left_ok && right_ok;
}

bool DualTrackDriver::areMotorsEnabled() const {
    return left_motor_->isMotorEnabled() && right_motor_->isMotorEnabled();
}

//=============================================================================
// Motion Commands
//=============================================================================

bool DualTrackDriver::driveStraight(float velocity_rad_s) {
    // For straight motion, both tracks move in the same direction (vehicle frame)
    // But due to mirrored mounting, the actual motor commands are opposite
    return setTrackSpeeds(velocity_rad_s, velocity_rad_s);
}

bool DualTrackDriver::rotateInPlace(float angular_rad_s) {
    // For in-place rotation:
    // - Positive angular = clockwise (right turn) = left forward, right backward
    // - Negative angular = counter-clockwise (left turn) = left backward, right forward
    float left_vel = angular_rad_s;
    float right_vel = -angular_rad_s;
    
    return setTrackSpeeds(left_vel, right_vel);
}

bool DualTrackDriver::arcTurn(float linear_vel, float turn_ratio) {
    // Clamp turn ratio to reasonable range
    turn_ratio = std::max(-2.0f, std::min(2.0f, turn_ratio));
    
    float left_vel, right_vel;
    
    if (turn_ratio >= 0) {
        // Turning right: slow down right track
        left_vel = linear_vel;
        right_vel = linear_vel * (1.0f - turn_ratio);
    } else {
        // Turning left: slow down left track
        left_vel = linear_vel * (1.0f + turn_ratio);
        right_vel = linear_vel;
    }
    
    return setTrackSpeeds(left_vel, right_vel);
}

bool DualTrackDriver::setTrackSpeeds(float left_rad_s, float right_rad_s) {
    // Clamp to limits
    left_rad_s = clampVelocity(left_rad_s);
    right_rad_s = clampVelocity(right_rad_s);
    
    // Store targets for correction
    left_speed_target_ = left_rad_s;
    right_speed_target_ = right_rad_s;
    
    // Apply motor direction inversions
    float left_cmd = applyLeftInversion(left_rad_s);
    float right_cmd = applyRightInversion(right_rad_s);
    
    // Apply speed correction if enabled
    if (speed_correction_enabled_) {
        applySpeedCorrection(left_cmd, right_cmd);
    }
    
    bool left_ok = left_motor_->setSpeedSetpoint(left_cmd);
    bool right_ok = right_motor_->setSpeedSetpoint(right_cmd);
    
    if (!left_ok) {
        last_error_ = "Left motor: " + left_motor_->getLastError();
    }
    if (!right_ok) {
        last_error_ = "Right motor: " + right_motor_->getLastError();
    }
    
    return left_ok && right_ok;
}

bool DualTrackDriver::emergencyStop() {
    bool left_ok = left_motor_->emergencyStop();
    bool right_ok = right_motor_->emergencyStop();
    
    // Also disable motors for safety
    left_motor_->enableMotor(false);
    right_motor_->enableMotor(false);
    
    return left_ok && right_ok;
}

bool DualTrackDriver::stop() {
    return setTrackSpeeds(0.0f, 0.0f);
}

//=============================================================================
// Telemetry
//=============================================================================

bool DualTrackDriver::readTelemetry(Telemetry& left, Telemetry& right) {
    bool left_ok = left_motor_->readTelemetry(left);
    bool right_ok = right_motor_->readTelemetry(right);
    
    // Cache for fault checking
    left_telemetry_ = left;
    right_telemetry_ = right;
    
    if (!left_ok) {
        last_error_ = "Left telemetry: " + left_motor_->getLastError();
    }
    if (!right_ok) {
        last_error_ = "Right telemetry: " + right_motor_->getLastError();
    }
    
    return left_ok && right_ok;
}

bool DualTrackDriver::hasAnyFault() const {
    return left_telemetry_.faults.hasAnyFault() || 
           right_telemetry_.faults.hasAnyFault();
}

bool DualTrackDriver::clearFaults() {
    bool left_ok = left_motor_->clearFaults();
    bool right_ok = right_motor_->clearFaults();
    return left_ok && right_ok;
}

//=============================================================================
// Configuration
//=============================================================================

bool DualTrackDriver::setControlMode(ControlMode mode) {
    bool left_ok = left_motor_->setControlMode(mode);
    bool right_ok = right_motor_->setControlMode(mode);
    
    if (!left_ok) {
        last_error_ = "Left motor mode: " + left_motor_->getLastError();
    }
    if (!right_ok) {
        last_error_ = "Right motor mode: " + right_motor_->getLastError();
    }
    
    return left_ok && right_ok;
}

//=============================================================================
// Private Helpers
//=============================================================================

float DualTrackDriver::applyLeftInversion(float velocity) const {
    return config_.invert_left ? -velocity : velocity;
}

float DualTrackDriver::applyRightInversion(float velocity) const {
    return config_.invert_right ? -velocity : velocity;
}

float DualTrackDriver::clampVelocity(float velocity) const {
    return std::max(-config_.max_speed_rad_s, 
                    std::min(config_.max_speed_rad_s, velocity));
}

void DualTrackDriver::applySpeedCorrection(float& left_cmd, float& right_cmd) {
    // Get actual speeds from cached telemetry (already inverted in telemetry reading)
    float left_actual = left_telemetry_.speed_rad_s;
    float right_actual = right_telemetry_.speed_rad_s;
    
    // Calculate errors (target vs actual)
    // Note: left_cmd/right_cmd are already inverted for motor direction
    // We compare against the original target speeds
    float left_error = left_speed_target_ - left_actual;
    float right_error = right_speed_target_ - right_actual;
    
    // Apply proportional correction
    left_correction_ = speed_correction_kp_ * left_error;
    right_correction_ = speed_correction_kp_ * right_error;
    
    // Add correction to command (respecting inversion)
    if (config_.invert_left) {
        left_cmd -= left_correction_;
    } else {
        left_cmd += left_correction_;
    }
    
    if (config_.invert_right) {
        right_cmd -= right_correction_;
    } else {
        right_cmd += right_correction_;
    }
    
    // Clamp corrected commands to limits
    left_cmd = std::max(-config_.max_speed_rad_s, 
                        std::min(config_.max_speed_rad_s, left_cmd));
    right_cmd = std::max(-config_.max_speed_rad_s, 
                         std::min(config_.max_speed_rad_s, right_cmd));
}

} // namespace solo
