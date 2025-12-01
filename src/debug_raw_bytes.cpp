#include "solo_usb_controller/solo_driver.hpp"
#include <iostream>
#include <iomanip>
#include <vector>

int main() {
    solo::SoloSerial::Config config;
    config.device = "/dev/solo_mc_1";
    config.baudrate = 937500;
    config.timeout_ms = 500;
    
    solo::SoloDriver driver(config);
    
    std::cout << "Connecting..." << std::endl;
    if (!driver.connect()) {
        std::cerr << "Failed: " << driver.getLastError() << std::endl;
        return 1;
    }
    std::cout << "Connected!\n" << std::endl;
    
    // Test reading firmware version and show raw bytes
    std::cout << "=== Testing Firmware Version Read ===" << std::endl;
    
    // Use the serial interface directly to see raw bytes
    solo::SoloSerial* serial = const_cast<solo::SoloSerial*>(
        reinterpret_cast<const solo::SoloSerial*>(&driver));
    
    // Actually, let's use a different approach - read via driver but add debug output
    uint32_t fw_raw = 0;
    if (driver.readFirmwareVersionRaw(fw_raw)) {
        std::cout << "Firmware (raw uint32): 0x" << std::hex << std::setfill('0') 
                  << std::setw(8) << fw_raw << std::dec << std::endl;
        std::cout << "Firmware (decimal): " << fw_raw << std::endl;
        
        // Parse as documented format
        uint8_t major = (fw_raw >> 16) & 0xFF;
        uint8_t minor = (fw_raw >> 8) & 0xFF;
        uint8_t patch = fw_raw & 0xFF;
        std::cout << "Parsed as: " << (int)major << "." << (int)minor << "." << (int)patch << std::endl;
        
        // Try little-endian interpretation
        uint8_t major_le = fw_raw & 0xFF;
        uint8_t minor_le = (fw_raw >> 8) & 0xFF;
        uint8_t patch_le = (fw_raw >> 16) & 0xFF;
        std::cout << "Little-endian: " << (int)major_le << "." << (int)minor_le << "." << (int)patch_le << std::endl;
    } else {
        std::cout << "Failed: " << driver.getLastError() << std::endl;
    }
    
    std::cout << "\n=== Testing Bus Voltage Read ===" << std::endl;
    
    solo::Telemetry telem;
    if (driver.readTelemetry(telem)) {
        std::cout << std::fixed << std::setprecision(3);
        std::cout << "Voltage: " << telem.voltage_v << " V" << std::endl;
        std::cout << "Temperature: " << telem.temperature_c << " °C" << std::endl;
        std::cout << "Current: " << telem.current_a << " A" << std::endl;
        
        // Show what we'd expect
        std::cout << "\nExpected ranges:" << std::endl;
        std::cout << "  Voltage: ~16V (12-54V range)" << std::endl;
        std::cout << "  Temperature: ~25°C (ambient)" << std::endl;
        std::cout << "  Current: ~0A (motor not running)" << std::endl;
    } else {
        std::cout << "Failed: " << driver.getLastError() << std::endl;
    }
    
    return 0;
}



