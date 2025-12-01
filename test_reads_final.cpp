#include "solo_usb_controller/solo_driver.hpp"
#include <iostream>
#include <iomanip>

int main() {
    solo::SoloSerial::Config config;
    config.device = "/dev/solo_mc_1";
    config.baudrate = 115200;
    config.timeout_ms = 150;
    
    solo::SoloDriver driver(config);
    
    std::cout << "Connecting..." << std::endl;
    if (!driver.connect()) {
        std::cerr << "Failed: " << driver.getLastError() << std::endl;
        return 1;
    }
    std::cout << "Connected!\n" << std::endl;
    
    // Test individual reads
    std::cout << std::fixed << std::setprecision(2);
    
    std::string fw;
    if (driver.readFirmwareVersion(fw)) {
        std::cout << "Firmware: " << fw << std::endl;
    } else {
        std::cout << "FW read failed: " << driver.getLastError() << std::endl;
    }
    
    solo::Telemetry telem;
    if (driver.readTelemetry(telem)) {
        std::cout << "\nTelemetry:" << std::endl;
        std::cout << "  Voltage:     " << telem.voltage_v << " V" << std::endl;
        std::cout << "  Current:     " << telem.current_a << " A" << std::endl;
        std::cout << "  Temperature: " << telem.temperature_c << " °C" << std::endl;
        std::cout << "  Speed:       " << telem.speed_rad_s << " rad/s" << std::endl;
        std::cout << "  Position:    " << telem.position_rad << " rad" << std::endl;
    } else {
        std::cout << "Telemetry failed: " << driver.getLastError() << std::endl;
    }
    
    return 0;
}




