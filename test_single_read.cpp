#include "solo_usb_controller/solo_serial.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <iomanip>
#include <vector>
#include <cstring>

// Test individual command reads
int main() {
    solo::SoloSerial::Config config;
    config.device = "/dev/solo_mc_1";
    config.baudrate = 115200;
    config.timeout_ms = 150;
    
    solo::SoloSerial serial(config);
    
    std::cout << "Connecting..." << std::endl;
    if (!serial.connect()) {
        std::cerr << "Failed: " << serial.getLastError() << std::endl;
        return 1;
    }
    std::cout << "Connected!\n" << std::endl;
    
    // Test reading bus voltage (command 0x86)
    std::cout << "Reading bus voltage (cmd 0x86)..." << std::endl;
    
    std::vector<uint8_t> empty;
    if (!serial.writeCommand(0x86, empty)) {
        std::cerr << "Write failed: " << serial.getLastError() << std::endl;
        return 1;
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    
    uint8_t resp_cmd;
    std::vector<uint8_t> data;
    
    if (!serial.readResponse(resp_cmd, data)) {
        std::cerr << "Read failed: " << serial.getLastError() << std::endl;
        return 1;
    }
    
    std::cout << "Response cmd: 0x" << std::hex << (int)resp_cmd << std::dec << std::endl;
    std::cout << "Data bytes: ";
    for (auto b : data) {
        printf("%02X ", b);
    }
    std::cout << std::endl;
    
    return 0;
}




