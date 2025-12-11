#ifndef SOLO_USB_CONTROLLER_SOLO_SERIAL_HPP
#define SOLO_USB_CONTROLLER_SOLO_SERIAL_HPP

#include <string>
#include <vector>
#include <memory>
#include <cstdint>
#include <boost/asio.hpp>

namespace solo {

/**
 * @brief Serial communication layer for SOLO motor controllers
 * 
 * Handles USB serial communication with SOLO Mini v2 controllers
 * using the UART/USB protocol as described in the communication manual.
 */
class SoloSerial {
public:
    struct Config {
        std::string device;
        unsigned int baudrate;
        unsigned int timeout_ms;
        uint8_t device_address;  // SOLO device address (0x00-0xFE, 0xFF=broadcast)
        
        Config() : device("/dev/solo_mc_1"), baudrate(115200), timeout_ms(1000), device_address(0x00) {}
    };

    explicit SoloSerial(const Config& config = Config());
    ~SoloSerial();

    // Connection management
    bool connect();
    void disconnect();
    bool isConnected() const { return connected_; }

    // Low-level communication
    bool writeCommand(uint8_t cmd, const std::vector<uint8_t>& data);
    bool readResponse(uint8_t& cmd, std::vector<uint8_t>& data, uint32_t timeout_ms = 0);
    
    // Utility
    void flush();
    std::string getLastError() const { return last_error_; }
    Config& getConfig() { return config_; }

private:
    Config config_;
    boost::asio::io_context io_context_;
    std::unique_ptr<boost::asio::serial_port> serial_port_;
    bool connected_;
    std::string last_error_;

    // Protocol helpers
    uint16_t calculateCRC(const std::vector<uint8_t>& data);
    bool validateCRC(const std::vector<uint8_t>& data, uint16_t expected_crc);
};

} // namespace solo

#endif // SOLO_USB_CONTROLLER_SOLO_SERIAL_HPP

