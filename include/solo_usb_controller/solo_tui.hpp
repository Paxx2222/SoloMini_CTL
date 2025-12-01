#ifndef SOLO_USB_CONTROLLER_SOLO_TUI_HPP
#define SOLO_USB_CONTROLLER_SOLO_TUI_HPP

#include "solo_driver.hpp"
#include <ncurses.h>
#include <memory>
#include <string>
#include <fstream>
#include <chrono>

namespace solo {

/**
 * @brief Terminal User Interface for direct SOLO control
 * 
 * Provides interactive control and monitoring of SOLO motor controller
 * Features:
 * - Mode selection (Speed/Torque/Position)
 * - Setpoint adjustment with keyboard
 * - Real-time telemetry display
 * - Fault monitoring and banners
 * - CSV data logging
 * - Emergency stop
 */
class SoloTUI {
public:
    explicit SoloTUI(const std::string& device = "/dev/solo_mc_1");
    ~SoloTUI();

    // Main application loop
    void run();

private:
    // UI Management
    void initUI();
    void shutdownUI();
    void drawUI();
    void drawHeader();
    void drawConnectionStatus();
    void drawControlPanel();
    void drawTelemetry();
    void drawFaults();
    void drawHelp();
    void drawStatusBar();
    
    // Input handling
    void handleInput(int ch);
    void adjustSetpoint(float delta);
    void cycleMode();
    void toggleMotor();
    void emergencyStop();
    void toggleLogging();
    
    // Logging
    bool openLogFile();
    void closeLogFile();
    void logData();
    
    // State
    std::unique_ptr<SoloDriver> driver_;
    std::string device_;
    bool running_;
    bool motor_enabled_;
    ControlMode current_mode_;
    float setpoint_;
    Telemetry telemetry_;
    
    // UI state
    int screen_height_;
    int screen_width_;
    bool show_help_;
    std::string status_message_;
    std::string firmware_version_;  // Cache firmware version
    std::chrono::steady_clock::time_point last_update_;
    
    // Logging
    bool logging_enabled_;
    std::ofstream log_file_;
    std::chrono::steady_clock::time_point log_start_time_;
    
    // Configuration
    static constexpr float SPEED_STEP = 5.0f;      // rad/s
    static constexpr float TORQUE_STEP = 0.1f;     // N·m
    static constexpr float POSITION_STEP = 0.1f;   // rad
    static constexpr int UPDATE_RATE_MS = 250;     // 4 Hz (slower to avoid overwhelming controller)
};

} // namespace solo

#endif // SOLO_USB_CONTROLLER_SOLO_TUI_HPP




