#include "solo_usb_controller/solo_driver.hpp"
#include <cstring>
#include <sstream>
#include <iomanip>
#include <thread>
#include <chrono>

namespace solo {

// Convert SOLO SFXT format to float
// SFXT is a fixed-point format: value = int32 / 131072.0
// NOTE: Data bytes are already reversed during extraction (little-endian response),
// so we use big-endian interpretation here (matches official library)
static float sfxtToFloat(const std::vector<uint8_t>& data) {
    if (data.size() < 4) return 0.0f;
    
    // Data bytes are already reversed during extraction, so use big-endian interpretation
    // This matches the official SOLO library: ExtractData reverses, then ConvertToFloat uses big-endian
    uint32_t raw = (static_cast<uint32_t>(data[0]) << 24) |
                   (static_cast<uint32_t>(data[1]) << 16) |
                   (static_cast<uint32_t>(data[2]) << 8) |
                   (static_cast<uint32_t>(data[3]));
    
    // Check if positive or negative
    if (raw <= 0x7FFE0000) {
        // Positive number
        return static_cast<float>(raw) / 131072.0f;
    } else {
        // Negative number (two's complement)
        uint32_t abs_val = 0xFFFFFFFF - raw + 1;
        return -static_cast<float>(abs_val) / 131072.0f;
    }
}

// Convert uint32 from bytes
// NOTE: Data bytes are already reversed during extraction (little-endian response),
// so we use big-endian interpretation here (matches official library)
static uint32_t bytesToUInt32(const std::vector<uint8_t>& data) {
    if (data.size() < 4) return 0;
    
    // Data bytes are already reversed during extraction, so use big-endian interpretation
    return (static_cast<uint32_t>(data[0]) << 24) |
           (static_cast<uint32_t>(data[1]) << 16) |
           (static_cast<uint32_t>(data[2]) << 8) |
           (static_cast<uint32_t>(data[3]));
}

// Convert float to SOLO SFXT format bytes
static std::vector<uint8_t> floatToSFXT(float value) {
    int32_t fixed = static_cast<int32_t>(value * 131072.0f);
    uint32_t raw;
    
    if (fixed < 0) {
        raw = 0xFFFFFFFF - static_cast<uint32_t>(std::abs(fixed)) + 1;
    } else {
        raw = static_cast<uint32_t>(fixed);
    }
    
    // Big-endian byte order
    std::vector<uint8_t> bytes(4);
    bytes[0] = (raw >> 24) & 0xFF;
    bytes[1] = (raw >> 16) & 0xFF;
    bytes[2] = (raw >> 8) & 0xFF;
    bytes[3] = raw & 0xFF;
    
    return bytes;
}

std::string Fault::toString() const {
    std::stringstream ss;
    bool first = true;
    
    auto addFault = [&](bool condition, const char* name) {
        if (condition) {
            if (!first) ss << ", ";
            ss << name;
            first = false;
        }
    };
    
    addFault(overvoltage, "OVERVOLTAGE");
    addFault(undervoltage, "UNDERVOLTAGE");
    addFault(overcurrent, "OVERCURRENT");
    addFault(overtemperature, "OVERTEMPERATURE");
    addFault(encoder_error, "ENCODER_ERROR");
    addFault(hall_sensor_error, "HALL_SENSOR_ERROR");
    addFault(watchdog_timeout, "WATCHDOG_TIMEOUT");
    addFault(motor_stall, "MOTOR_STALL");
    
    return first ? "NONE" : ss.str();
}

SoloDriver::SoloDriver(const SoloSerial::Config& serial_config)
    : current_mode_(ControlMode::SPEED)
{
    serial_ = std::make_unique<SoloSerial>(serial_config);
}

SoloDriver::~SoloDriver() {
    disconnect();
}

bool SoloDriver::connect() {
    if (!serial_->connect()) {
        last_error_ = serial_->getLastError();
        return false;
    }
    
    // Small delay for device initialization
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    return true;
}

void SoloDriver::disconnect() {
    serial_->disconnect();
}

bool SoloDriver::isConnected() const {
    return serial_->isConnected();
}

bool SoloDriver::setControlMode(ControlMode mode) {
    uint32_t mode_val = static_cast<uint32_t>(mode);
    if (writeUInt32(CommandID::WRITE_CONTROL_MODE, mode_val)) {
        current_mode_ = mode;
        return true;
    }
    return false;
}

bool SoloDriver::setSpeedSetpoint(float speed_rad_s) {
    return writeFloat(CommandID::WRITE_SPEED_REFERENCE, speed_rad_s);
}

bool SoloDriver::setTorqueSetpoint(float torque_nm) {
    return writeFloat(CommandID::WRITE_TORQUE_REFERENCE, torque_nm);
}

bool SoloDriver::setPositionSetpoint(float position_rad, bool relative) {
    // For relative positioning, read current position and add offset
    if (relative) {
        Telemetry telem;
        if (!readTelemetry(telem)) {
            return false;
        }
        position_rad += telem.position_rad;
    }
    
    return writeFloat(CommandID::WRITE_POSITION_REFERENCE, position_rad);
}

bool SoloDriver::emergencyStop() {
    // Send zero setpoint in current mode and disable motor
    bool success = false;
    
    switch (current_mode_) {
        case ControlMode::SPEED:
            success = setSpeedSetpoint(0.0f);
            break;
        case ControlMode::TORQUE:
            success = setTorqueSetpoint(0.0f);
            break;
        case ControlMode::POSITION:
            // Keep position in position mode
            success = true;
            break;
    }
    
    if (success) {
        success = enableMotor(false);
    }
    
    return success;
}

bool SoloDriver::enableMotor(bool enable) {
    return writeUInt32(CommandID::WRITE_MOTOR_ENABLE, enable ? 1 : 0);
}

bool SoloDriver::isMotorEnabled() {
    uint32_t enabled;
    if (readUInt32(CommandID::READ_MOTOR_ENABLE, enabled)) {
        return enabled != 0;
    }
    return false;
}

bool SoloDriver::setMotorConfig(const MotorConfig& config) {
    bool success = true;
    
    success &= writeUInt32(CommandID::WRITE_MOTOR_POLES, config.pole_pairs);
    success &= writeUInt32(CommandID::WRITE_ENCODER_LINES, config.encoder_cpr);
    
    return success;
}

bool SoloDriver::setLimits(const Limits& limits) {
    bool success = true;
    
    success &= writeFloat(CommandID::WRITE_SPEED_LIMIT, limits.max_speed_rad_s);
    success &= writeFloat(CommandID::WRITE_CURRENT_LIMIT, limits.max_current_a);
    
    return success;
}

bool SoloDriver::setPID(ControlMode mode, const PIDConfig& pid) {
    bool success = true;
    
    switch (mode) {
        case ControlMode::SPEED:
            success &= writeFloat(CommandID::WRITE_SPEED_KP, pid.p);
            success &= writeFloat(CommandID::WRITE_SPEED_KI, pid.i);
            break;
        case ControlMode::TORQUE:
            success &= writeFloat(CommandID::WRITE_CURRENT_KP, pid.p);
            success &= writeFloat(CommandID::WRITE_CURRENT_KI, pid.i);
            break;
        case ControlMode::POSITION:
            success &= writeFloat(CommandID::WRITE_POSITION_KP, pid.p);
            success &= writeFloat(CommandID::WRITE_POSITION_KI, pid.i);
            break;
    }
    
    return success;
}

bool SoloDriver::setRampRate(float ramp_rate) {
    // Implementation depends on SOLO firmware version
    // This is a placeholder - check actual command availability
    (void)ramp_rate;  // Suppress unused parameter warning
    return true;
}

bool SoloDriver::readTelemetry(Telemetry& telemetry) {
    // Flush once before reading all telemetry
    serial_->flush();
    
    bool success = true;
    
    // Read without individual flushes (using internal method)
    success &= readFloatNoFlush(CommandID::READ_SPEED_FEEDBACK, telemetry.speed_rad_s);
    success &= readFloatNoFlush(CommandID::READ_POSITION_FEEDBACK, telemetry.position_rad);
    success &= readFloatNoFlush(CommandID::READ_CURRENT_IQ, telemetry.current_a);
    success &= readFloatNoFlush(CommandID::READ_BUS_VOLTAGE, telemetry.voltage_v);
    success &= readFloatNoFlush(CommandID::READ_TEMPERATURE, telemetry.temperature_c);
    
    // Calculate encoder count from position
    // Assuming 4096 CPR encoder and 15 pole pairs from config
    uint32_t encoder_cpr = 4096;
    const float TWO_PI = 6.28318530718f;
    telemetry.encoder_count = static_cast<uint32_t>(telemetry.position_rad / TWO_PI * encoder_cpr);
    
    // Estimate torque from Iq current
    // For BLDC motor: Torque = Kt * Iq, where Kt ≈ 60/(2*pi*Kv*sqrt(3)) for BLDC
    // With Kv=400 RPM/V: Kt ≈ 0.0218 Nm/A
    float kt = 0.022f;  // Approximate torque constant
    telemetry.torque_nm = kt * telemetry.current_a;
    
    telemetry.motor_enabled = isMotorEnabled();
    success &= readFaults(telemetry.faults);
    
    return success;
}

bool SoloDriver::readFaults(Fault& faults) {
    uint32_t error_reg;
    if (!readUInt32(CommandID::READ_ERROR_REGISTER, error_reg)) {
        return false;
    }
    
    // Parse error register bits (bit positions may vary - check manual)
    faults.overvoltage = (error_reg & 0x01) != 0;
    faults.undervoltage = (error_reg & 0x02) != 0;
    faults.overcurrent = (error_reg & 0x04) != 0;
    faults.overtemperature = (error_reg & 0x08) != 0;
    faults.encoder_error = (error_reg & 0x10) != 0;
    faults.hall_sensor_error = (error_reg & 0x20) != 0;
    faults.watchdog_timeout = (error_reg & 0x40) != 0;
    faults.motor_stall = (error_reg & 0x80) != 0;
    
    return true;
}

bool SoloDriver::clearFaults() {
    return writeUInt32(CommandID::WRITE_OVERWRITE_ERROR, 0);
}

bool SoloDriver::saveToFlash() {
    // SOLO doesn't have a direct save-to-flash command in basic UART protocol
    // Settings are typically saved automatically or via specific configuration sequence
    last_error_ = "Save to flash not supported in current protocol version";
    return false;
}

bool SoloDriver::loadDefaults() {
    // Implementation depends on SOLO firmware - may require special sequence
    return true;
}

bool SoloDriver::homePosition() {
    // Set position to zero
    return setPositionSetpoint(0.0f, false);
}

bool SoloDriver::readDeviceAddress(uint8_t& address) {
    uint32_t addr;
    if (readUInt32(CommandID::READ_DEVICE_ADDRESS, addr)) {
        address = static_cast<uint8_t>(addr);
        return true;
    }
    return false;
}

bool SoloDriver::readFirmwareVersion(std::string& version) {
    uint32_t ver;
    if (readUInt32(CommandID::READ_FIRMWARE_VERSION, ver)) {
        std::stringstream ss;
        ss << ((ver >> 16) & 0xFF) << "." 
           << ((ver >> 8) & 0xFF) << "." 
           << (ver & 0xFF);
        version = ss.str();
        return true;
    }
    return false;
}

bool SoloDriver::readFirmwareVersionRaw(uint32_t& version) {
    return readUInt32(CommandID::READ_FIRMWARE_VERSION, version);
}

// Helper methods
bool SoloDriver::writeFloat(CommandID cmd, float value) {
    // Convert float to SOLO SFXT format
    std::vector<uint8_t> data = floatToSFXT(value);
    
    if (!serial_->writeCommand(static_cast<uint8_t>(cmd), data)) {
        last_error_ = serial_->getLastError();
        return false;
    }
    
    return true;
}

bool SoloDriver::writeInt32(CommandID cmd, int32_t value) {
    // Convert to big-endian bytes
    std::vector<uint8_t> data(4);
    uint32_t raw = static_cast<uint32_t>(value);
    data[0] = (raw >> 24) & 0xFF;
    data[1] = (raw >> 16) & 0xFF;
    data[2] = (raw >> 8) & 0xFF;
    data[3] = raw & 0xFF;
    
    if (!serial_->writeCommand(static_cast<uint8_t>(cmd), data)) {
        last_error_ = serial_->getLastError();
        return false;
    }
    
    return true;
}

bool SoloDriver::writeUInt32(CommandID cmd, uint32_t value) {
    // Convert to big-endian bytes
    std::vector<uint8_t> data(4);
    data[0] = (value >> 24) & 0xFF;
    data[1] = (value >> 16) & 0xFF;
    data[2] = (value >> 8) & 0xFF;
    data[3] = value & 0xFF;
    
    if (!serial_->writeCommand(static_cast<uint8_t>(cmd), data)) {
        last_error_ = serial_->getLastError();
        return false;
    }
    
    return true;
}

bool SoloDriver::readFloat(CommandID cmd, float& value) {
    // Flush before single read
    serial_->flush();
    return readFloatNoFlush(cmd, value);
}

bool SoloDriver::readFloatNoFlush(CommandID cmd, float& value) {
    uint8_t resp_cmd;
    std::vector<uint8_t> data;
    
    // Send read command (empty data for read commands)
    std::vector<uint8_t> empty_data;
    if (!serial_->writeCommand(static_cast<uint8_t>(cmd), empty_data)) {
        last_error_ = serial_->getLastError();
        return false;
    }
    
    // Delay for controller to process and respond
    // Official Python library uses 100ms delay after write (time.sleep(0.1))
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Read response
    if (!serial_->readResponse(resp_cmd, data) || data.size() < 4) {
        last_error_ = "Invalid response: " + serial_->getLastError();
        return false;
    }
    
    // Verify we got the expected command back
    if (resp_cmd != static_cast<uint8_t>(cmd)) {
        last_error_ = "Command mismatch";
        return false;
    }
    
    // Convert from SOLO SFXT format to float
    // Data bytes are already reversed during extraction
    value = sfxtToFloat(data);
    return true;
}

bool SoloDriver::readInt32(CommandID cmd, int32_t& value) {
    uint8_t resp_cmd;
    std::vector<uint8_t> data;
    
    // Flush any stale data first
    serial_->flush();
    
    std::vector<uint8_t> empty_data;
    if (!serial_->writeCommand(static_cast<uint8_t>(cmd), empty_data)) {
        last_error_ = serial_->getLastError();
        return false;
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    if (!serial_->readResponse(resp_cmd, data) || data.size() < 4) {
        last_error_ = "Invalid response: " + serial_->getLastError();
        return false;
    }
    
    // Verify command match
    if (resp_cmd != static_cast<uint8_t>(cmd)) {
        last_error_ = "Command mismatch";
        return false;
    }
    
    // Convert from bytes - data bytes are already reversed during extraction
    uint32_t raw = bytesToUInt32(data);
    value = static_cast<int32_t>(raw);
    return true;
}

bool SoloDriver::readUInt32(CommandID cmd, uint32_t& value) {
    uint8_t resp_cmd;
    std::vector<uint8_t> data;
    
    // Flush any stale data first
    serial_->flush();
    
    std::vector<uint8_t> empty_data;
    if (!serial_->writeCommand(static_cast<uint8_t>(cmd), empty_data)) {
        last_error_ = serial_->getLastError();
        return false;
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    if (!serial_->readResponse(resp_cmd, data) || data.size() < 4) {
        last_error_ = "Invalid response: " + serial_->getLastError();
        return false;
    }
    
    // Verify command match
    if (resp_cmd != static_cast<uint8_t>(cmd)) {
        last_error_ = "Command mismatch";
        return false;
    }
    
    // Convert from bytes - data bytes are already reversed during extraction
    value = bytesToUInt32(data);
    return true;
}

float SoloDriver::clampToLimits(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

} // namespace solo

