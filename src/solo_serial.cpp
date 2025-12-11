#include "solo_usb_controller/solo_serial.hpp"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <thread>
#ifdef __linux__
#include <sys/select.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/serial.h>
#include <termios.h>
#include <errno.h>
#include <cstring>
#endif

namespace solo {

SoloSerial::SoloSerial(const Config& config)
    : config_(config)
    , serial_port_(nullptr)
    , connected_(false)
{
}

SoloSerial::~SoloSerial() {
    disconnect();
}

bool SoloSerial::connect() {
    try {
        serial_port_ = std::make_unique<boost::asio::serial_port>(io_context_);
        serial_port_->open(config_.device);
        
        // Configure serial port - basic settings first
        serial_port_->set_option(boost::asio::serial_port_base::character_size(8));
        serial_port_->set_option(boost::asio::serial_port_base::parity(
            boost::asio::serial_port_base::parity::none));
        serial_port_->set_option(boost::asio::serial_port_base::stop_bits(
            boost::asio::serial_port_base::stop_bits::one));
        serial_port_->set_option(boost::asio::serial_port_base::flow_control(
            boost::asio::serial_port_base::flow_control::none));
        
        // Set baud rate - use custom rate API for non-standard bauds
        #ifdef __linux__
        if (config_.baudrate == 937500) {
            // Custom baud rate using serial_struct
            int fd = serial_port_->native_handle();
            struct serial_struct ser;
            
            if (ioctl(fd, TIOCGSERIAL, &ser) == 0) {
                ser.custom_divisor = ser.baud_base / config_.baudrate;
                ser.flags &= ~ASYNC_SPD_MASK;
                ser.flags |= ASYNC_SPD_CUST;
                
                if (ioctl(fd, TIOCSSERIAL, &ser) == 0) {
                    // Set to 38400, which will be multiplied by custom_divisor
                    struct termios tio;
                    tcgetattr(fd, &tio);
                    cfsetispeed(&tio, B38400);
                    cfsetospeed(&tio, B38400);
                    tcsetattr(fd, TCSANOW, &tio);
                } else {
                    // Fallback to closest standard rate
                    struct termios tio;
                    tcgetattr(fd, &tio);
                    cfsetispeed(&tio, B921600);
                    cfsetospeed(&tio, B921600);
                    tcsetattr(fd, TCSANOW, &tio);
                }
            }
        } else {
            // Standard baud rate
            serial_port_->set_option(boost::asio::serial_port_base::baud_rate(config_.baudrate));
        }
        #else
        // Non-Linux: use standard Boost.Asio
        serial_port_->set_option(boost::asio::serial_port_base::baud_rate(config_.baudrate));
        #endif
        
        connected_ = true;
        last_error_.clear();
        
        // Additional serial port configuration for Linux
        #ifdef __linux__
        int fd = serial_port_->native_handle();
        struct termios tio;
        if (tcgetattr(fd, &tio) == 0) {
            // Disable canonical mode and echo
            tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
            // Disable input processing
            tio.c_iflag &= ~(IXON | IXOFF | IXANY | INLCR | IGNCR | ICRNL);
            // Raw output
            tio.c_oflag &= ~OPOST;
            // Disable flow control (RTS/CTS) - important for USB serial
            tio.c_cflag &= ~CRTSCTS;
            tio.c_iflag &= ~(IXON | IXOFF);
            // Set VMIN and VTIME for non-blocking reads
            tio.c_cc[VMIN] = 0;
            tio.c_cc[VTIME] = 0;
            // Apply settings
            if (tcsetattr(fd, TCSANOW, &tio) != 0) {
                last_error_ = "Failed to configure serial port termios";
                connected_ = false;
                return false;
            }
        }
        #endif
        
        // Sleep for serial initialization (Python library uses 0.2s)
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        // Flush any stale data
        flush();
        
        return true;
    } catch (const std::exception& e) {
        last_error_ = std::string("Failed to connect: ") + e.what();
        connected_ = false;
        return false;
    }
}

void SoloSerial::disconnect() {
    if (serial_port_ && serial_port_->is_open()) {
        try {
            serial_port_->close();
        } catch (...) {}
    }
    serial_port_.reset();
    connected_ = false;
}

bool SoloSerial::writeCommand(uint8_t cmd, const std::vector<uint8_t>& data) {
    if (!connected_ || !serial_port_ || !serial_port_->is_open()) {
        last_error_ = "Not connected";
        return false;
    }
    
    try {
        // SOLO UART protocol frame: [0xFF] [0xFF] [ADDR] [CMD] [DATA0-3] [CRC] [0xFE]
        std::vector<uint8_t> frame;
        frame.push_back(0xFF);  // Initiator 1
        frame.push_back(0xFF);  // Initiator 2
        // Device address from config (0x00-0xFE for specific device, 0xFF for broadcast)
        frame.push_back(config_.device_address);  // Device address
        frame.push_back(cmd);   // Command ID
        
        // Add 4 bytes of data (pad with zeros if needed)
        for (size_t i = 0; i < 4; i++) {
            frame.push_back(i < data.size() ? data[i] : 0x00);
        }
        
        // CRC is hardcoded to 0x00 in official SOLO library (not calculated)
        // Reference: https://github.com/Solo-FL/SOLO-motor-controllers-CPP-library
        frame.push_back(0x00);  // CRC (hardcoded, not calculated)
        frame.push_back(0xFE);  // Ending byte
        
        // Debug: Print frame being sent (can be enabled for debugging)
        #ifdef DEBUG_SOLO_PROTOCOL
        std::cout << "Sending frame: ";
        for (size_t i = 0; i < frame.size(); i++) {
            std::cout << "0x" << std::hex << std::setfill('0') << std::setw(2)
                      << static_cast<int>(frame[i]) << " ";
        }
        std::cout << std::dec << std::endl;
        #endif
        
        // Write to serial
        // Use low-level write() to avoid Boost.Asio write timeout checks
        // (echo command works, so direct write should work)
        #ifdef __linux__
        int fd = serial_port_->native_handle();
        ssize_t bytes_written = ::write(fd, frame.data(), frame.size());
        
        if (bytes_written < 0) {
            last_error_ = std::string("Write failed: ") + strerror(errno);
            return false;
        }
        
        if (static_cast<size_t>(bytes_written) != frame.size()) {
            last_error_ = "Incomplete write: " + std::to_string(bytes_written) + " of " + std::to_string(frame.size()) + " bytes";
            return false;
        }
        
        // Flush to ensure data is sent
        tcdrain(fd);
        #else
        // Non-Linux: use Boost.Asio
        boost::system::error_code ec;
        size_t bytes_written = boost::asio::write(*serial_port_, boost::asio::buffer(frame), ec);
        
        if (ec) {
            last_error_ = std::string("Write failed: ") + ec.message();
            return false;
        }
        
        if (bytes_written != frame.size()) {
            last_error_ = "Incomplete write: " + std::to_string(bytes_written) + " of " + std::to_string(frame.size()) + " bytes";
            return false;
        }
        #endif
        
        // Small delay after write (controller needs time to process)
        // Official library uses 0.1s, but we'll use shorter delay and start reading immediately
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        
        return true;
    } catch (const std::exception& e) {
        last_error_ = std::string("Write failed: ") + e.what();
        return false;
    }
}

bool SoloSerial::readResponse(uint8_t& cmd, std::vector<uint8_t>& data, uint32_t timeout_ms) {
    if (!connected_ || !serial_port_ || !serial_port_->is_open()) {
        last_error_ = "Not connected";
        return false;
    }
    
    if (timeout_ms == 0) {
        timeout_ms = config_.timeout_ms;
    }
    
    // Don't flush here - we want to read the response!
    // Flush should happen before sending the command, not before reading response
    
    try {
        // Official Python library reads exactly 10 bytes: _read_packet = self._ser.read(10)
        // Let's try that approach - read exactly 10 bytes
        #ifdef __linux__
        int fd = serial_port_->native_handle();
        uint8_t read_buffer[10];
        size_t total_bytes = 0;
        
        auto start_time = std::chrono::steady_clock::now();
        
        // Read exactly 10 bytes (SOLO frame size)
        while (total_bytes < 10) {
            // Check timeout
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - start_time).count();
            if (elapsed >= static_cast<int64_t>(timeout_ms)) {
                std::stringstream ss;
                ss << "Read timeout - got " << total_bytes << " of 10 bytes";
                if (total_bytes > 0) {
                    ss << ": ";
                    for (size_t i = 0; i < total_bytes; i++) {
                        ss << "0x" << std::hex << std::setfill('0') << std::setw(2)
                           << static_cast<int>(read_buffer[i]) << " ";
                    }
                }
                last_error_ = ss.str();
                return false;
            }
            
            // Check if data available
            struct timeval tv;
            tv.tv_sec = 0;
            tv.tv_usec = 100000;  // 100ms per check
            
            fd_set readfds;
            FD_ZERO(&readfds);
            FD_SET(fd, &readfds);
            
            int ret = select(fd + 1, &readfds, NULL, NULL, &tv);
            
            if (ret < 0) {
                last_error_ = "select() failed";
                return false;
            } else if (ret == 0) {
                continue;  // No data yet, keep waiting
            }
            
            // Data available, read it
            ssize_t n = ::read(fd, &read_buffer[total_bytes], 10 - total_bytes);
            if (n < 0) {
                last_error_ = std::string("read() failed: ") + strerror(errno);
                return false;
            }
            if (n == 0) {
                // EOF
                break;
            }
            
            total_bytes += n;
        }
        
        if (total_bytes < 10) {
            std::stringstream ss;
            ss << "Incomplete read: got " << total_bytes << " of 10 bytes";
            if (total_bytes > 0) {
                ss << ": ";
                for (size_t i = 0; i < total_bytes; i++) {
                    ss << "0x" << std::hex << std::setfill('0') << std::setw(2)
                       << static_cast<int>(read_buffer[i]) << " ";
                }
            }
            last_error_ = ss.str();
            return false;
        }
        
        // Validate frame structure: [0xFF] [0xFF] [ADDR] [CMD] [DATA0] [DATA1] [DATA2] [DATA3] [CRC] [0xFE]
        if (read_buffer[0] != 0xFF || read_buffer[1] != 0xFF || read_buffer[9] != 0xFE) {
            std::stringstream ss;
            ss << "Invalid frame structure: ";
            for (size_t i = 0; i < 10; i++) {
                ss << "0x" << std::hex << std::setfill('0') << std::setw(2)
                   << static_cast<int>(read_buffer[i]) << " ";
            }
            last_error_ = ss.str();
            return false;
        }
        
        // Extract command and data
        cmd = read_buffer[3];
        data.clear();
        // Extract data bytes in big-endian order (as they appear in frame)
        // Frame: [0xFF] [0xFF] [ADDR] [CMD] [DATA0] [DATA1] [DATA2] [DATA3] [CRC] [0xFE]
        // DATA0 is MSB, DATA3 is LSB (big-endian)
        for (int j = 0; j < 4; j++) {
            data.push_back(read_buffer[4+j]);
        }
        
        return true;
        
        #else
        // Fallback for non-Linux: simple blocking read
        uint8_t frame[10];
        boost::system::error_code ec;
        size_t bytes_read = boost::asio::read(*serial_port_,
            boost::asio::buffer(frame, 10), ec);
        if (ec) {
            last_error_ = ec.message();
            return false;
        }
        
        if (bytes_read != 10) {
            last_error_ = "Incomplete read: got " + std::to_string(bytes_read) + " bytes";
            return false;
        }
        
        // Validate and extract
        if (frame[0] != 0xFF || frame[1] != 0xFF || frame[9] != 0xFE) {
            last_error_ = "Invalid frame structure";
            return false;
        }
        
        cmd = frame[3];
        data.clear();
        // Extract data bytes in big-endian order (as they appear in frame)
        for (int i = 0; i < 4; i++) {
            data.push_back(frame[4 + i]);
        }
        
        return true;
        #endif
        
    } catch (const std::exception& e) {
        last_error_ = std::string("Read failed: ") + e.what();
        return false;
    }
}

void SoloSerial::flush() {
    if (!serial_port_ || !serial_port_->is_open()) return;
    
    try {
        // Use tcflush to discard input/output buffers (POSIX way)
        // This is non-blocking and works immediately
        #ifdef __linux__
        ::tcflush(serial_port_->native_handle(), TCIOFLUSH);
        #else
        // Alternative: read available data without blocking
        boost::system::error_code ec;
        while (serial_port_->available(ec) > 0 && !ec) {
            uint8_t dummy;
            serial_port_->read_some(boost::asio::buffer(&dummy, 1), ec);
        }
        #endif
    } catch (...) {}
}

uint16_t SoloSerial::calculateCRC(const std::vector<uint8_t>& data) {
    // CRC-16-CCITT (0xFFFF initial value)
    uint16_t crc = 0xFFFF;
    
    for (uint8_t byte : data) {
        crc ^= static_cast<uint16_t>(byte) << 8;
        for (int i = 0; i < 8; i++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc = crc << 1;
            }
        }
    }
    
    return crc;
}

bool SoloSerial::validateCRC(const std::vector<uint8_t>& data, uint16_t expected_crc) {
    return calculateCRC(data) == expected_crc;
}

} // namespace solo

