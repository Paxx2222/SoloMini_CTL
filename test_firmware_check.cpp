#include <iostream>
#include <iomanip>
#include "solo_usb_controller/solo_driver.hpp"

int main() {
    std::cout << "SOLO Motor Controller Firmware Check" << std::endl;
    std::cout << "====================================" << std::endl;
    
    solo::SoloDriver driver;
    
    std::cout << "Attempting to connect to motor controller..." << std::endl;
    
    if (!driver.connect()) {
        std::cerr << "Failed to connect: " << driver.getLastError() << std::endl;
        return 1;
    }
    
    std::cout << "✓ Successfully connected to motor controller!" << std::endl;
    
    // Read firmware version as raw hex
    uint32_t raw_version = 0;
    std::string firmware_version;
    
    // Try reading firmware version
    if (driver.readFirmwareVersion(firmware_version)) {
        std::cout << "✓ Firmware Version: " << firmware_version << std::endl;
    } else {
        std::cerr << "✗ Failed to read firmware version: " << driver.getLastError() << std::endl;
    }
    
    // Read temperature and other values via telemetry
    solo::Telemetry telemetry;
    if (driver.readTelemetry(telemetry)) {
        std::cout << "✓ Controller Temperature: " << std::fixed << std::setprecision(1) 
                  << telemetry.temperature_c << " °C" << std::endl;
        std::cout << "✓ Bus Voltage: " << std::fixed << std::setprecision(2) 
                  << telemetry.voltage_v << " V" << std::endl;
        std::cout << "✓ Motor Current: " << std::fixed << std::setprecision(3) 
                  << telemetry.current_a << " A" << std::endl;
        std::cout << "✓ Motor Speed: " << std::fixed << std::setprecision(1) 
                  << telemetry.speed_rad_s << " rad/s" << std::endl;
        std::cout << "✓ Motor Enabled: " << (telemetry.motor_enabled ? "Yes" : "No") << std::endl;
        
        if (telemetry.faults.hasAnyFault()) {
            std::cout << "✗ Faults: " << telemetry.faults.toString() << std::endl;
        } else {
            std::cout << "✓ No faults detected" << std::endl;
        }
    } else {
        std::cerr << "✗ Failed to read telemetry: " << driver.getLastError() << std::endl;
    }
    
    // Read device address
    uint8_t device_address;
    if (driver.readDeviceAddress(device_address)) {
        std::cout << "✓ Device Address: " << static_cast<int>(device_address) << std::endl;
    }
    
    std::cout << std::endl << "Motor controller communication check complete!" << std::endl;
    
    driver.disconnect();
    return 0;
}
