#include "solo_usb_controller/solo_driver.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <iomanip>

int main() {
    solo::SoloSerial::Config config;
    config.device = "/dev/solo_mc_1";
    config.baudrate = 115200;
    config.timeout_ms = 100;
    
    solo::SoloDriver driver(config);
    
    std::cout << "Connecting..." << std::endl;
    if (!driver.connect()) {
        std::cerr << "Failed: " << driver.getLastError() << std::endl;
        return 1;
    }
    std::cout << "Connected!\n" << std::endl;
    
    // Read telemetry 10 times
    for (int i = 0; i < 10; i++) {
        solo::Telemetry telem;
        
        std::cout << "Read #" << (i+1) << ": ";
        if (driver.readTelemetry(telem)) {
            std::cout << std::fixed << std::setprecision(2);
            std::cout << "V=" << telem.voltage_v << "V, "
                      << "I=" << telem.current_a << "A, "
                      << "T=" << telem.temperature_c << "°C, "
                      << "Speed=" << telem.speed_rad_s << "rad/s"
                      << std::endl;
        } else {
            std::cout << "FAILED - " << driver.getLastError() << std::endl;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    
    return 0;
}




