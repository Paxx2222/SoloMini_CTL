#include "solo_usb_controller/solo_driver.hpp"
#include <ncurses.h>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <thread>

namespace solo {

class SimpleTestTUI {
public:
    explicit SimpleTestTUI(const std::string& device = "/dev/solo_mc_1")
        : device_(device)
        , running_(false)
    {
        SoloSerial::Config config;
        config.device = device_;
        config.baudrate = 115200;  // SOLO USB serial baud rate (standard rate for Linux compatibility)
        driver_ = std::make_unique<SoloDriver>(config);
    }

    ~SimpleTestTUI() {
        if (driver_) {
            driver_->disconnect();
        }
        endwin();
    }

    void run() {
        initUI();
        
        // Try to connect
        mvprintw(2, 2, "Connecting to %s...", device_.c_str());
        refresh();
        
        if (!driver_->connect()) {
            mvprintw(4, 2, "ERROR: Failed to connect: %s", driver_->getLastError().c_str());
            mvprintw(6, 2, "Press any key to exit...");
            refresh();
            getch();
            return;
        }
        
        mvprintw(2, 2, "Connected to %s", device_.c_str());
        refresh();
        
        // Read firmware version once (both formatted and raw hex)
        std::string firmware_version;
        uint32_t firmware_raw = 0;
        if (!driver_->readFirmwareVersion(firmware_version)) {
            firmware_version = "Unknown";
        }
        driver_->readFirmwareVersionRaw(firmware_raw);
        
        running_ = true;
        auto last_update = std::chrono::steady_clock::now();
        
        while (running_) {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - last_update).count();
            
            // Update at 4 Hz
            if (elapsed >= 250) {
                updateDisplay(firmware_version, firmware_raw);
                last_update = now;
            }
            
            // Handle input (non-blocking)
            timeout(50);
            int ch = getch();
            if (ch != ERR) {
                if (ch == 'q' || ch == 'Q' || ch == 27) {  // 'q' or ESC
                    running_ = false;
                }
            }
        }
    }

private:
    void initUI() {
        initscr();
        cbreak();
        noecho();
        keypad(stdscr, TRUE);
        nodelay(stdscr, TRUE);
        curs_set(0);
        
        if (has_colors()) {
            start_color();
            init_pair(1, COLOR_GREEN, COLOR_BLACK);   // Good
            init_pair(2, COLOR_RED, COLOR_BLACK);     // Error
            init_pair(3, COLOR_YELLOW, COLOR_BLACK);  // Warning
            init_pair(4, COLOR_CYAN, COLOR_BLACK);    // Info
        }
    }

    void updateDisplay(const std::string& firmware_version, uint32_t firmware_raw) {
        clear();
        
        // Header - using ASCII characters instead of UTF-8
        attron(COLOR_PAIR(4) | A_BOLD);
        mvprintw(0, 0, "================================================================");
        mvprintw(1, 0, "     SOLO Motor Controller - Communication Test");
        mvprintw(2, 0, "================================================================");
        attroff(COLOR_PAIR(4) | A_BOLD);
        
        int row = 4;
        
        // Connection status
        if (driver_->isConnected()) {
            attron(COLOR_PAIR(1));
            mvprintw(row, 2, "[*] CONNECTED");
            attroff(COLOR_PAIR(1));
        } else {
            attron(COLOR_PAIR(2));
            mvprintw(row, 2, "[X] DISCONNECTED");
            attroff(COLOR_PAIR(2));
        }
        
        mvprintw(row, 20, "Device: %s", device_.c_str());
        mvprintw(row, 50, "FW: %s (0x%08X)", firmware_version.c_str(), firmware_raw);
        
        row += 3;
        
        // Try reading firmware version first as a simple test
        std::string test_fw;
        uint32_t test_fw_raw = 0;
        bool fw_test = driver_->readFirmwareVersionRaw(test_fw_raw);
        if (!fw_test) {
            attron(COLOR_PAIR(3));
            mvprintw(row++, 2, "Firmware read test: FAILED - %s", driver_->getLastError().c_str());
            attroff(COLOR_PAIR(3));
        } else {
            attron(COLOR_PAIR(1));
            mvprintw(row++, 2, "Firmware read test: OK (0x%08X)", test_fw_raw);
            attroff(COLOR_PAIR(1));
        }
        row += 1;
        
        // Read telemetry
        Telemetry telemetry;
        bool success = driver_->readTelemetry(telemetry);
        
        if (!success) {
            attron(COLOR_PAIR(2));
            mvprintw(row++, 2, "ERROR: Failed to read telemetry: %s", driver_->getLastError().c_str());
            mvprintw(row++, 2, "This may indicate:");
            mvprintw(row++, 4, "- Controller not responding");
            mvprintw(row++, 4, "- Communication protocol mismatch");
            mvprintw(row++, 4, "- Serial port configuration issue");
            row++;  // Blank line
            mvprintw(row++, 2, "Troubleshooting:");
            mvprintw(row++, 4, "- Check serial port: %s", device_.c_str());
            mvprintw(row++, 4, "- Verify baud rate (should be: 115200)");
            mvprintw(row++, 4, "- Check controller power and connections");
            attroff(COLOR_PAIR(2));
            row += 1;
        } else {
            // Bus Voltage - prominently displayed
            attron(A_BOLD);
            mvprintw(row++, 2, "BUS VOLTAGE:");
            attroff(A_BOLD);
            
            // Color code voltage
            int voltage_color = COLOR_PAIR(1);  // Green (normal)
            if (telemetry.voltage_v < 12.0f) {
                voltage_color = COLOR_PAIR(2);  // Red (low)
            } else if (telemetry.voltage_v > 54.0f) {
                voltage_color = COLOR_PAIR(2);  // Red (high)
            } else if (telemetry.voltage_v < 14.0f || telemetry.voltage_v > 50.0f) {
                voltage_color = COLOR_PAIR(3);  // Yellow (warning)
            }
            
            attron(voltage_color | A_BOLD);
            mvprintw(row++, 4, "  %.2f V", telemetry.voltage_v);
            attroff(voltage_color | A_BOLD);
            
            row += 2;
            
            // Temperature - prominently displayed
            attron(A_BOLD);
            mvprintw(row++, 2, "TEMPERATURE:");
            attroff(A_BOLD);
            
            // Color code temperature
            int temp_color = COLOR_PAIR(1);  // Green (normal)
            if (telemetry.temperature_c > 70.0f) {
                temp_color = COLOR_PAIR(2);  // Red (hot)
            } else if (telemetry.temperature_c > 50.0f) {
                temp_color = COLOR_PAIR(3);  // Yellow (warm)
            }
            
            attron(temp_color | A_BOLD);
            mvprintw(row++, 4, "  %.2f °C", telemetry.temperature_c);
            attroff(temp_color | A_BOLD);
            
            row += 2;
            
            // Additional info
            attron(COLOR_PAIR(4));
            mvprintw(row++, 2, "Additional Telemetry:");
            attroff(COLOR_PAIR(4));
            mvprintw(row++, 4, "  Current:     %.3f A", telemetry.current_a);
            mvprintw(row++, 4, "  Speed:       %.2f rad/s", telemetry.speed_rad_s);
            mvprintw(row++, 4, "  Position:    %.2f rad", telemetry.position_rad);
            mvprintw(row++, 4, "  Motor Enabled: %s", telemetry.motor_enabled ? "Yes" : "No");
            
            // Faults
            if (telemetry.faults.hasAnyFault()) {
                row++;
                attron(COLOR_PAIR(2) | A_BOLD);
                mvprintw(row++, 2, "FAULTS: %s", telemetry.faults.toString().c_str());
                attroff(COLOR_PAIR(2) | A_BOLD);
            } else {
                row++;
                attron(COLOR_PAIR(1));
                mvprintw(row++, 2, "No faults detected");
                attroff(COLOR_PAIR(1));
            }
        }
        
        row += 2;
        
        // Status bar
        attron(A_REVERSE);
        mvprintw(row, 0, "%-*s", COLS, "Press 'q' or ESC to quit");
        attroff(A_REVERSE);
        
        refresh();
    }

    std::unique_ptr<SoloDriver> driver_;
    std::string device_;
    bool running_;
};

} // namespace solo

int main(int argc, char** argv) {
    std::string device = "/dev/solo_mc_1";
    
    if (argc > 1) {
        device = argv[1];
    }
    
    try {
        solo::SimpleTestTUI tui(device);
        tui.run();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}

