#ifndef SOLO_USB_CONTROLLER_SOLO_DRIVER_HPP
#define SOLO_USB_CONTROLLER_SOLO_DRIVER_HPP

#include "solo_serial.hpp"
#include <string>
#include <cstdint>
#include <memory>

namespace solo {

// Control modes
enum class ControlMode {
    SPEED = 0,
    TORQUE = 1,
    POSITION = 2
};

// Motor types
enum class MotorType {
    BLDC = 0,
    BRUSHED = 1
};

// Fault flags
struct Fault {
    bool overvoltage = false;
    bool undervoltage = false;
    bool overcurrent = false;
    bool overtemperature = false;
    bool encoder_error = false;
    bool hall_sensor_error = false;
    bool watchdog_timeout = false;
    bool motor_stall = false;
    
    bool hasAnyFault() const {
        return overvoltage || undervoltage || overcurrent || overtemperature ||
               encoder_error || hall_sensor_error || watchdog_timeout || motor_stall;
    }
    
    std::string toString() const;
};

// Telemetry data
struct Telemetry {
    float speed_rad_s = 0.0f;          // rad/s
    float speed_raw = 0.0f;            // Raw value from READ_SPEED_FEEDBACK (0x96)
    float position_rad = 0.0f;         // rad
    float torque_nm = 0.0f;            // N·m
    float current_a = 0.0f;            // A (quadrature current)
    float voltage_v = 0.0f;            // DC bus voltage
    float temperature_c = 0.0f;        // Board temperature
    uint32_t encoder_count = 0;
    Fault faults;
    bool motor_enabled = false;
};

// PID configuration
struct PIDConfig {
    float p = 0.0f;
    float i = 0.0f;
    float d = 0.0f;
    float ff = 0.0f;  // feedforward
};

// Motor parameters
struct MotorConfig {
    MotorType type = MotorType::BLDC;
    uint8_t pole_pairs = 7;
    uint16_t encoder_cpr = 2048;
    bool has_halls = true;
    float kv_rpm_per_volt = 400.0f;
    float gear_ratio = 1.0f;
};

// Limits
struct Limits {
    float max_speed_rad_s = 20.94f;  // 200 RPM = 200 * (2π/60) ≈ 20.94 rad/s
    float max_torque_nm = 2.5f;
    float max_current_a = 20.0f;
    float dc_bus_volt_min = 12.0f;
    float dc_bus_volt_max = 54.0f;
};

/**
 * @brief High-level driver for SOLO motor controller
 * 
 * Implements the SOLO protocol commands for motor control,
 * configuration, and telemetry reading.
 */
class SoloDriver {
public:
    explicit SoloDriver(const SoloSerial::Config& serial_config = SoloSerial::Config());
    ~SoloDriver();

    // Connection
    bool connect();
    void disconnect();
    bool isConnected() const;
    
    // Auto-detect device address (tries 0x00, 0x01, 0x02, etc.)
    bool autoDetectAddress();

    // Motor control
    bool setControlMode(ControlMode mode);
    bool setCommandMode(uint32_t mode);  // 0=ANALOGUE, 1=DIGITAL, 2=ANALOGUE_WITH_DIGITAL_SPEED_GAIN
    bool setSpeedSetpoint(float speed_rad_s);
    bool setTorqueSetpoint(float torque_nm);
    bool setPositionSetpoint(float position_rad, bool relative = false);
    bool emergencyStop();
    
    // Enable/disable
    bool enableMotor(bool enable);
    bool isMotorEnabled();

    // Configuration
    bool setMotorConfig(const MotorConfig& config);
    bool setLimits(const Limits& limits);
    bool setPID(ControlMode mode, const PIDConfig& pid);
    bool setRampRate(float ramp_rate);  // units/s/s
    bool setPWMFrequency(uint32_t frequency_khz);  // PWM frequency in kHz
    bool setMotorType(uint32_t motor_type);  // 0=DC, 1=BLDC_PMSM, 2=ACIM, 3=BLDC_PMSM_ULTRAFAST
    bool setFeedbackControlMode(uint32_t mode);  // 0=SENSORLESS_HSO, 1=ENCODERS, 2=HALL_SENSORS
    bool clearFaults();  // Clear error register
    
    // Telemetry
    bool readTelemetry(Telemetry& telemetry);
    bool readFaults(Fault& faults);
    
    // Flash operations
    bool saveToFlash();
    bool loadDefaults();
    
    // Homing
    bool homePosition();
    
    // Device info
    bool readDeviceAddress(uint8_t& address);
    bool readFirmwareVersion(std::string& version);
    bool readFirmwareVersionRaw(uint32_t& version);
    
    // Error handling
    std::string getLastError() const { return last_error_; }

private:
    std::unique_ptr<SoloSerial> serial_;
    std::string last_error_;
    ControlMode current_mode_;
    
    // Command IDs (SOLO UART Protocol - from SOLO documentation)
    enum class CommandID : uint8_t {
        // Write commands
        WRITE_DEVICE_ADDRESS = 0x01,
        WRITE_COMMAND_MODE = 0x02,
        WRITE_CURRENT_LIMIT = 0x03,
        WRITE_TORQUE_REFERENCE = 0x04,
        WRITE_SPEED_REFERENCE = 0x05,
        WRITE_POWER_REFERENCE = 0x06,
        WRITE_MOTOR_ENABLE = 0x08,
        WRITE_OUTPUT_PWM_FREQUENCY = 0x09,
        WRITE_SPEED_KP = 0x0A,
        WRITE_SPEED_KI = 0x0B,
        WRITE_MOTOR_DIRECTION = 0x0C,
        WRITE_MOTOR_POLES = 0x0F,
        WRITE_ENCODER_LINES = 0x10,
        WRITE_SPEED_LIMIT = 0x11,
        WRITE_FEEDBACK_CONTROL_MODE = 0x13,
        WRITE_MOTOR_TYPE = 0x15,
        WRITE_CONTROL_MODE = 0x16,
        WRITE_CURRENT_KP = 0x17,
        WRITE_CURRENT_KI = 0x18,
        WRITE_POSITION_REFERENCE = 0x1B,
        WRITE_POSITION_KP = 0x1C,
        WRITE_POSITION_KI = 0x1D,
        WRITE_OVERWRITE_ERROR = 0x20,
        
        // Read commands  
        READ_DEVICE_ADDRESS = 0x81,
        READ_PHASEA_VOLTAGE = 0x82,
        READ_PHASEB_VOLTAGE = 0x83,
        READ_PHASEA_CURRENT = 0x84,
        READ_PHASEB_CURRENT = 0x85,
        READ_BUS_VOLTAGE = 0x86,
        READ_DC_MOTOR_CURRENT = 0x87,
        READ_DC_MOTOR_VOLTAGE = 0x88,
        READ_SPEED_KP = 0x89,
        READ_SPEED_KI = 0x8A,
        READ_PWM_FREQUENCY = 0x8B,
        READ_CURRENT_LIMIT = 0x8C,
        READ_CURRENT_IQ = 0x8D,
        READ_CURRENT_ID = 0x8E,
        READ_MOTOR_POLES = 0x8F,
        READ_ENCODER_LINES = 0x90,
        READ_CURRENT_KP = 0x91,
        READ_CURRENT_KI = 0x92,
        READ_TEMPERATURE = 0x93,
        READ_MOTOR_RESISTANCE = 0x94,
        READ_MOTOR_INDUCTANCE = 0x95,
        READ_SPEED_FEEDBACK = 0x96,
        READ_MOTOR_TYPE = 0x97,
        READ_FEEDBACK_MODE = 0x99,
        READ_COMMAND_MODE = 0x9A,
        READ_CONTROL_MODE = 0x9B,
        READ_POSITION_COUNTS_FEEDBACK = 0xA0,
        READ_ERROR_REGISTER = 0xA1,
        READ_FIRMWARE_VERSION = 0xA2,
        READ_MOTOR_ENABLE = 0xB3,  // Drive state
    };
    
    // Helper methods
    bool writeFloat(CommandID cmd, float value);
    bool writeInt32(CommandID cmd, int32_t value);
    bool writeUInt32(CommandID cmd, uint32_t value);
    bool readFloat(CommandID cmd, float& value);
    bool readFloatNoFlush(CommandID cmd, float& value);
    bool readInt32(CommandID cmd, int32_t& value);
    bool readInt32NoFlush(CommandID cmd, int32_t& value);
    bool readUInt32(CommandID cmd, uint32_t& value);
    
    float clampToLimits(float value, float min, float max);
};

} // namespace solo

#endif // SOLO_USB_CONTROLLER_SOLO_DRIVER_HPP




