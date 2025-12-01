#include "solo_usb_controller/solo_serial.hpp"
#include <iostream>
#include <iomanip>
#include <vector>

int main() {
    solo::SoloSerial::Config config;
    config.device = "/dev/solo_mc_1";
    config.baudrate = 115200;  // Standard baud rate for Linux compatibility
    config.timeout_ms = 1000;  // Longer timeout for debugging
    
    solo::SoloSerial serial(config);
    
    std::cout << "=== SOLO Protocol Debug Tool ===" << std::endl;
    std::cout << "Device: " << config.device << std::endl;
    std::cout << "Baudrate: " << config.baudrate << std::endl;
    std::cout << "\nConnecting..." << std::endl;
    
    if (!serial.connect()) {
        std::cerr << "Failed to connect: " << serial.getLastError() << std::endl;
        return 1;
    }
    
    std::cout << "Connected!\n" << std::endl;
    
    // Test reading firmware version (command 0xA2)
    std::cout << "=== Test: Read Firmware Version (0xA2) ===" << std::endl;
    
    std::vector<uint8_t> empty_data;
    uint8_t cmd = 0xA2;  // READ_FIRMWARE_VERSION
    
    std::cout << "Sending command frame:" << std::endl;
    std::cout << "  Command: 0x" << std::hex << std::setfill('0') << std::setw(2) 
              << static_cast<int>(cmd) << std::dec << std::endl;
    std::cout << "  Expected frame: [0xFF] [0xFF] [0x00] [0xA2] [0x00] [0x00] [0x00] [0x00] [0x00] [0xFE]" << std::endl;
    std::cout << "  (Device address: 0x00 = default)" << std::endl;
    
    // Try multiple device addresses
    std::vector<uint8_t> addresses_to_try = {0x00, 0x01, 0xFF};
    
    for (uint8_t addr : addresses_to_try) {
        std::cout << "\n--- Trying device address: 0x" << std::hex << std::setfill('0') 
                  << std::setw(2) << static_cast<int>(addr) << std::dec << " ---" << std::endl;
        
        // Note: We'd need to modify writeCommand to accept address, but for now
        // let's just test with the current implementation (0x00)
        if (addr != 0x00) {
            std::cout << "  (Skipping - code currently hardcoded to 0x00)" << std::endl;
            continue;
        }
        
        if (!serial.writeCommand(cmd, empty_data)) {
            std::cerr << "Write failed: " << serial.getLastError() << std::endl;
            continue;
        }
        
        std::cout << "Command sent successfully" << std::endl;
        std::cout << "Waiting for response..." << std::endl;
    
        uint8_t resp_cmd;
        std::vector<uint8_t> data;
        
        if (!serial.readResponse(resp_cmd, data)) {
            std::cerr << "Read failed: " << serial.getLastError() << std::endl;
            if (addr == addresses_to_try.back()) {
                std::cout << "\n=== All addresses tried, no response ===" << std::endl;
                std::cout << "Possible issues:" << std::endl;
                std::cout << "  1. Controller is not powered" << std::endl;
                std::cout << "  2. Controller is in wrong mode (CAN/Modbus instead of UART/USB)" << std::endl;
                std::cout << "  3. Wrong baud rate (try 115200 or check controller settings)" << std::endl;
                std::cout << "  4. Hardware issue (USB cable, port, controller)" << std::endl;
                return 1;
            }
            continue;
        }
        
        std::cout << "\n*** SUCCESS! Response received! ***" << std::endl;
        std::cout << "Response command: 0x" << std::hex << std::setfill('0') << std::setw(2)
                  << static_cast<int>(resp_cmd) << std::dec << std::endl;
        std::cout << "Data bytes (" << data.size() << "): ";
        for (size_t i = 0; i < data.size(); i++) {
            std::cout << "0x" << std::hex << std::setfill('0') << std::setw(2)
                      << static_cast<int>(data[i]) << " ";
        }
        std::cout << std::dec << std::endl;
        
        // Parse firmware version
        if (data.size() >= 4) {
            uint32_t fw = (static_cast<uint32_t>(data[0]) << 24) |
                          (static_cast<uint32_t>(data[1]) << 16) |
                          (static_cast<uint32_t>(data[2]) << 8) |
                          (static_cast<uint32_t>(data[3]));
            
            uint8_t major = (fw >> 16) & 0xFF;
            uint8_t minor = (fw >> 8) & 0xFF;
            uint8_t patch = fw & 0xFF;
            
            std::cout << "\nFirmware version (raw): 0x" << std::hex << std::setfill('0') 
                      << std::setw(8) << fw << std::dec << std::endl;
            std::cout << "Firmware version: " << static_cast<int>(major) << "." 
                      << static_cast<int>(minor) << "." << static_cast<int>(patch) << std::endl;
        }
        
        std::cout << "\n=== Communication successful! ===" << std::endl;
        return 0;
    }
    
    return 1;
}

