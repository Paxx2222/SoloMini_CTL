#include "solo_usb_controller/solo_tui.hpp"
#include <iomanip>
#include <sstream>
#include <cmath>
#include <thread>
#include <chrono>

namespace solo {

SoloTUI::SoloTUI(const std::string& device)
    : device_(device)
    , running_(false)
    , motor_enabled_(false)
    , current_mode_(ControlMode::SPEED)
    , setpoint_(0.0f)
    , screen_height_(0)
    , screen_width_(0)
    , show_help_(false)
    , telemetry_read_success_(true)
    , telemetry_error_("")
    , logging_enabled_(false)
{
    SoloSerial::Config config;
    config.device = device_;
    driver_ = std::make_unique<SoloDriver>(config);
}

SoloTUI::~SoloTUI() {
    shutdownUI();
    closeLogFile();
}

void SoloTUI::run() {
    initUI();
    
    // Try to connect
    status_message_ = "Connecting to " + device_ + "...";
    drawUI();
    
    if (!driver_->connect()) {
        status_message_ = "ERROR: " + driver_->getLastError();
        drawUI();
        getch();  // Wait for keypress
        return;
    }
    
    status_message_ = "Connected! Using device address 0x00";
    drawUI();
    
    // Skip auto-detection if address is already 0x00 (most common case)
    // Auto-detection can hang if device doesn't respond to broadcast
    // User can manually specify address via command line if needed
    
    // Read firmware version once (cache it)
    if (!driver_->readFirmwareVersion(firmware_version_)) {
        firmware_version_ = "Unknown";
    }
    
    running_ = true;
    last_update_ = std::chrono::steady_clock::now();
    
    // Clear any existing faults first - try multiple times for stubborn faults
    // WATCHDOG_TIMEOUT requires multiple clears
    for (int i = 0; i < 5; i++) {
        driver_->clearFaults();
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
    }
    
    // Read and check for faults before proceeding
    Telemetry test_telem;
    bool has_faults = false;
    if (driver_->readFaults(test_telem.faults)) {
        if (test_telem.faults.hasAnyFault()) {
            has_faults = true;
            status_message_ = "WARNING: Device has faults: " + test_telem.faults.toString();
            drawUI();
            // Try clearing again with longer delay
            for (int i = 0; i < 3; i++) {
                driver_->clearFaults();
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
            }
        }
    }
    
    // If still has faults, try reading firmware version to verify communication
    if (has_faults) {
        std::string fw_test;
        if (!driver_->readFirmwareVersion(fw_test)) {
            status_message_ = "ERROR: Cannot communicate with device. Check connection.";
            drawUI();
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Set motor type: BLDC_PMSM = 1 (required before other config)
    if (!driver_->setMotorType(1)) {  // BLDC_PMSM
        status_message_ = "ERROR: Failed to set motor type: " + driver_->getLastError();
        drawUI();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Set feedback control mode: ENCODERS = 1 (required)
    if (!driver_->setFeedbackControlMode(1)) {  // ENCODERS
        status_message_ = "ERROR: Failed to set feedback mode: " + driver_->getLastError();
        drawUI();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Set command mode to DIGITAL (required for digital speed control)
    if (!driver_->setCommandMode(1)) {  // DIGITAL = 1
        status_message_ = "ERROR: Failed to set command mode: " + driver_->getLastError();
        drawUI();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Set initial control mode
    if (!driver_->setControlMode(current_mode_)) {
        status_message_ = "ERROR: Failed to set control mode: " + driver_->getLastError();
        drawUI();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Configure motor parameters from dual.yaml config
    // poles: 30 means total poles = 30, so pole_pairs = 15
    MotorConfig motor_config;
    motor_config.pole_pairs = 15;  // From dual.yaml: poles=30, so pole_pairs=15
    motor_config.encoder_cpr = 4096;  // From dual.yaml
    if (!driver_->setMotorConfig(motor_config)) {
        status_message_ = "ERROR: Failed to set motor config: " + driver_->getLastError();
        drawUI();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Set speed limit from dual.yaml: max_speed_rad_s: 20
    Limits limits;
    limits.max_speed_rad_s = 20.0f;  // From dual.yaml
    limits.max_current_a = 3.0f;  // From dual.yaml: max_current_a: 3
    if (!driver_->setLimits(limits)) {
        status_message_ = "ERROR: Failed to set limits: " + driver_->getLastError();
        drawUI();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Set PWM frequency to 20 kHz (from dual.yaml requirement)
    if (!driver_->setPWMFrequency(20)) {
        status_message_ = "ERROR: Failed to set PWM frequency: " + driver_->getLastError();
        drawUI();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Set PID gains for speed control from dual.yaml
    PIDConfig speed_pid;
    speed_pid.p = 0.2f;  // From dual.yaml: speed: {p: 0.2, i: 0.01}
    speed_pid.i = 0.01f;
    if (!driver_->setPID(ControlMode::SPEED, speed_pid)) {
        status_message_ = "ERROR: Failed to set PID: " + driver_->getLastError();
        drawUI();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Clear faults again after configuration
    driver_->clearFaults();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    status_message_ = "Configuration complete";
    
    // Main loop
    while (running_) {
        // Handle input FIRST (non-blocking with 50ms timeout)
        int ch = getch();
        if (ch != ERR) {
            handleInput(ch);
        }
        
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_update_).count();
        
        // Update telemetry at fixed rate
        if (elapsed >= UPDATE_RATE_MS) {
            telemetry_read_success_ = driver_->readTelemetry(telemetry_);
            if (!telemetry_read_success_) {
                telemetry_error_ = driver_->getLastError();
            } else {
                telemetry_error_.clear();
            }
            
            // Continuously send setpoint when motor is enabled (SOLO may need periodic updates)
            if (motor_enabled_) {
                switch (current_mode_) {
                    case ControlMode::SPEED:
                        driver_->setSpeedSetpoint(setpoint_);
                        break;
                    case ControlMode::TORQUE:
                        driver_->setTorqueSetpoint(setpoint_);
                        break;
                    case ControlMode::POSITION:
                        driver_->setPositionSetpoint(setpoint_);
                        break;
                }
            }
            
            if (logging_enabled_ && telemetry_read_success_) {
                logData();
            }
            
            last_update_ = now;
        }
        
        // Draw UI every loop
        drawUI();
    }
}

void SoloTUI::initUI() {
    initscr();
    cbreak();     // Line buffering disabled, pass characters immediately
    noecho();     // Don't echo input
    keypad(stdscr, TRUE);  // Enable function keys
    curs_set(0);  // Hide cursor
    timeout(50);  // Set 50ms timeout for getch() globally
    
    // Enable colors if available
    if (has_colors()) {
        start_color();
        init_pair(1, COLOR_GREEN, COLOR_BLACK);   // Normal
        init_pair(2, COLOR_RED, COLOR_BLACK);     // Error/Fault
        init_pair(3, COLOR_YELLOW, COLOR_BLACK);  // Warning
        init_pair(4, COLOR_CYAN, COLOR_BLACK);    // Info
        init_pair(5, COLOR_WHITE, COLOR_RED);     // Emergency
    }
    
    getmaxyx(stdscr, screen_height_, screen_width_);
}

void SoloTUI::shutdownUI() {
    if (motor_enabled_) {
        driver_->emergencyStop();
    }
    endwin();
}

void SoloTUI::drawUI() {
    clear();
    
    drawHeader();
    drawConnectionStatus();
    
    if (telemetry_.faults.hasAnyFault()) {
        drawFaults();
    }
    
    drawControlPanel();
    drawTelemetry();
    
    if (show_help_) {
        drawHelp();
    }
    
    drawStatusBar();
    
    refresh();
}

void SoloTUI::drawHeader() {
    attron(COLOR_PAIR(4) | A_BOLD);
    mvprintw(0, 0, "================================================================================");
    mvprintw(1, 0, "           SOLO Motor Controller - Direct USB Control (Stage 1)        ");
    mvprintw(2, 0, "================================================================================");
    attroff(COLOR_PAIR(4) | A_BOLD);
}

void SoloTUI::drawConnectionStatus() {
    int row = 4;
    
    if (driver_->isConnected()) {
        attron(COLOR_PAIR(1));
        mvprintw(row, 2, "● CONNECTED");
        attroff(COLOR_PAIR(1));
    } else {
        attron(COLOR_PAIR(2));
        mvprintw(row, 2, "● DISCONNECTED");
        attroff(COLOR_PAIR(2));
    }
    
    mvprintw(row, 20, "Device: %s", device_.c_str());
    
    // Display cached firmware version
    if (!firmware_version_.empty()) {
        mvprintw(row, 50, "FW: %s", firmware_version_.c_str());
    }
    
    // Telemetry status indicator
    row++;
    if (telemetry_read_success_) {
        attron(COLOR_PAIR(1));
        mvprintw(row, 2, "● Telemetry: OK");
        attroff(COLOR_PAIR(1));
    } else {
        attron(COLOR_PAIR(2) | A_BOLD);
        mvprintw(row, 2, "● Telemetry: FAILED");
        attroff(COLOR_PAIR(2) | A_BOLD);
    }
}

void SoloTUI::drawControlPanel() {
    int row = 6;
    
    attron(A_BOLD);
    mvprintw(row++, 2, "CONTROL PANEL");
    attroff(A_BOLD);
    
    row++;
    
    // Mode selection
    const char* mode_str = "";
    switch (current_mode_) {
        case ControlMode::SPEED:    mode_str = "SPEED"; break;
        case ControlMode::TORQUE:   mode_str = "TORQUE"; break;
        case ControlMode::POSITION: mode_str = "POSITION"; break;
    }
    
    attron(COLOR_PAIR(4));
    mvprintw(row++, 4, "Mode:     [%s]", mode_str);
    attroff(COLOR_PAIR(4));
    
    // Motor enable status
    if (motor_enabled_) {
        attron(COLOR_PAIR(1) | A_BOLD);
        mvprintw(row++, 4, "Status:   ENABLED");
        attroff(COLOR_PAIR(1) | A_BOLD);
    } else {
        attron(COLOR_PAIR(3));
        mvprintw(row++, 4, "Status:   DISABLED");
        attroff(COLOR_PAIR(3));
    }
    
    row++;
    
    // Setpoint
    const char* unit = "";
    switch (current_mode_) {
        case ControlMode::SPEED:    unit = "rad/s"; break;
        case ControlMode::TORQUE:   unit = "N·m"; break;
        case ControlMode::POSITION: unit = "rad"; break;
    }
    
    mvprintw(row++, 4, "Setpoint: %.2f %s", setpoint_, unit);
    
    // Visual bar for setpoint
    float max_val = 100.0f;
    if (current_mode_ == ControlMode::TORQUE) max_val = 5.0f;
    
    int bar_width = 40;
    int filled = static_cast<int>((std::abs(setpoint_) / max_val) * bar_width);
    if (filled > bar_width) filled = bar_width;
    
    mvprintw(row, 4, "[");
    for (int i = 0; i < bar_width; i++) {
        if (i < filled) {
            addch('#');
        } else {
            addch(' ');
        }
    }
    addch(']');
}

void SoloTUI::drawTelemetry() {
    int row = 15;
    
    attron(A_BOLD);
    mvprintw(row++, 2, "TELEMETRY");
    attroff(A_BOLD);
    
    row++;
    
    // Show error if telemetry read failed
    if (!telemetry_read_success_) {
        attron(COLOR_PAIR(2) | A_BOLD);
        mvprintw(row++, 4, "ERROR: Telemetry read failed!");
        attroff(COLOR_PAIR(2) | A_BOLD);
        if (!telemetry_error_.empty()) {
            attron(COLOR_PAIR(3));
            // Truncate error message if too long
            std::string error_display = telemetry_error_;
            if (error_display.length() > 60) {
                error_display = error_display.substr(0, 57) + "...";
            }
            mvprintw(row++, 4, "%s", error_display.c_str());
            attroff(COLOR_PAIR(3));
        }
        row++;
        mvprintw(row++, 4, "Values shown may be stale");
        row++;
        return;
    }
    
    // Show speed with raw value from 0x96 command
    mvprintw(row++, 4, "Speed:       %10.4f rad/s", telemetry_.speed_rad_s);
    mvprintw(row++, 4, "  Raw 0x96:  %10.4f (%.2f RPM expected)", telemetry_.speed_raw, telemetry_.speed_raw);
    mvprintw(row++, 4, "Position:    %10.4f rad", telemetry_.position_rad);
    mvprintw(row++, 4, "Torque:      %10.4f N·m", telemetry_.torque_nm);
    mvprintw(row++, 4, "Current:     %10.4f A", telemetry_.current_a);
    mvprintw(row++, 4, "Voltage:     %8.2f V", telemetry_.voltage_v);
    
    // Temperature with color coding
    if (telemetry_.temperature_c > 70.0f) {
        attron(COLOR_PAIR(2) | A_BOLD);
    } else if (telemetry_.temperature_c > 50.0f) {
        attron(COLOR_PAIR(3));
    } else {
        attron(COLOR_PAIR(1));
    }
    mvprintw(row++, 4, "Temperature: %8.2f °C", telemetry_.temperature_c);
    attroff(COLOR_PAIR(1) | COLOR_PAIR(2) | COLOR_PAIR(3) | A_BOLD);
    
    row++;
    mvprintw(row++, 4, "Encoder:     %10u counts", telemetry_.encoder_count);
    
    // Logging status
    row++;
    if (logging_enabled_) {
        attron(COLOR_PAIR(1));
        mvprintw(row++, 4, "● LOGGING ACTIVE");
        attroff(COLOR_PAIR(1));
    } else {
        mvprintw(row++, 4, "○ Logging disabled");
    }
}

void SoloTUI::drawFaults() {
    int row = screen_height_ / 2 - 3;
    int col = screen_width_ / 2 - 20;
    
    attron(COLOR_PAIR(5) | A_BOLD | A_BLINK);
    mvprintw(row++, col, "========================================");
    mvprintw(row++, col, "         ! FAULT DETECTED !          ");
    mvprintw(row++, col, "----------------------------------------");
    
    std::string fault_str = telemetry_.faults.toString();
    mvprintw(row++, col, "  %-36s  ", fault_str.c_str());
    mvprintw(row++, col, "========================================");
    attroff(COLOR_PAIR(5) | A_BOLD | A_BLINK);
}

void SoloTUI::drawHelp() {
    int row = screen_height_ / 2 - 8;
    int col = 10;
    
    attron(COLOR_PAIR(4));
    mvprintw(row++, col, "==================================== HELP ====================================");
    mvprintw(row++, col, "                                                                              ");
    mvprintw(row++, col, "  UP/DOWN    - Adjust setpoint                                               ");
    mvprintw(row++, col, "  LEFT/RIGHT - Fine adjust setpoint                                          ");
    mvprintw(row++, col, "  M          - Cycle control mode                                            ");
    mvprintw(row++, col, "  E          - Toggle motor enable                                           ");
    mvprintw(row++, col, "  SPACE      - Emergency stop                                                 ");
    mvprintw(row++, col, "  L          - Toggle CSV logging                                             ");
    mvprintw(row++, col, "  C          - Clear faults                                                   ");
    mvprintw(row++, col, "  H          - Home position (set to 0)                                       ");
    mvprintw(row++, col, "  ?          - Toggle this help                                               ");
    mvprintw(row++, col, "  Q / ESC    - Quit                                                           ");
    mvprintw(row++, col, "                                                                              ");
    mvprintw(row++, col, "==============================================================================");
    attroff(COLOR_PAIR(4));
}

void SoloTUI::drawStatusBar() {
    int row = screen_height_ - 1;
    
    attron(A_REVERSE);
    mvprintw(row, 0, "%-*s", screen_width_, status_message_.c_str());
    attroff(A_REVERSE);
    
    // Add help hint
    std::string hint = "Press ? for help";
    mvprintw(row, screen_width_ - hint.length() - 1, "%s", hint.c_str());
}

void SoloTUI::handleInput(int ch) {
    switch (ch) {
        case KEY_UP:
            adjustSetpoint(1.0f);
            break;
        case KEY_DOWN:
            adjustSetpoint(-1.0f);
            break;
        case KEY_LEFT:
            adjustSetpoint(-0.1f);
            break;
        case KEY_RIGHT:
            adjustSetpoint(0.1f);
            break;
        case 'm':
        case 'M':
            cycleMode();
            break;
        case 'e':
        case 'E':
            toggleMotor();
            break;
        case ' ':
            emergencyStop();
            break;
        case 'l':
        case 'L':
            toggleLogging();
            break;
        case 'c':
        case 'C':
            driver_->clearFaults();
            status_message_ = "Faults cleared";
            break;
        case 'h':
        case 'H':
            driver_->homePosition();
            status_message_ = "Position homed to 0";
            break;
        case '?':
            show_help_ = !show_help_;
            break;
        case 'q':
        case 'Q':
        case 27:  // ESC
            running_ = false;
            status_message_ = "Shutting down...";
            break;
    }
}

void SoloTUI::adjustSetpoint(float delta) {
    float step = 0.0f;
    switch (current_mode_) {
        case ControlMode::SPEED:    step = SPEED_STEP; break;
        case ControlMode::TORQUE:   step = TORQUE_STEP; break;
        case ControlMode::POSITION: step = POSITION_STEP; break;
    }
    
    setpoint_ += delta * step;
    
    // Apply limits
    if (current_mode_ == ControlMode::SPEED) {
        setpoint_ = std::max(-150.0f, std::min(150.0f, setpoint_));
    } else if (current_mode_ == ControlMode::TORQUE) {
        setpoint_ = std::max(-2.5f, std::min(2.5f, setpoint_));
    }
    
    // Send to driver if motor is enabled
    if (motor_enabled_) {
        switch (current_mode_) {
            case ControlMode::SPEED:
                driver_->setSpeedSetpoint(setpoint_);
                break;
            case ControlMode::TORQUE:
                driver_->setTorqueSetpoint(setpoint_);
                break;
            case ControlMode::POSITION:
                driver_->setPositionSetpoint(setpoint_);
                break;
        }
    }
    
    status_message_ = "Setpoint adjusted";
}

void SoloTUI::cycleMode() {
    // Disable motor before changing mode
    if (motor_enabled_) {
        driver_->enableMotor(false);
        motor_enabled_ = false;
    }
    
    // Cycle through modes
    switch (current_mode_) {
        case ControlMode::SPEED:
            current_mode_ = ControlMode::TORQUE;
            break;
        case ControlMode::TORQUE:
            current_mode_ = ControlMode::POSITION;
            break;
        case ControlMode::POSITION:
            current_mode_ = ControlMode::SPEED;
            break;
    }
    
    driver_->setControlMode(current_mode_);
    setpoint_ = 0.0f;
    
    status_message_ = "Mode changed - motor disabled";
}

void SoloTUI::toggleMotor() {
    motor_enabled_ = !motor_enabled_;
    driver_->enableMotor(motor_enabled_);
    
    if (motor_enabled_) {
        // Send current setpoint when enabling motor
        switch (current_mode_) {
            case ControlMode::SPEED:
                driver_->setSpeedSetpoint(setpoint_);
                break;
            case ControlMode::TORQUE:
                driver_->setTorqueSetpoint(setpoint_);
                break;
            case ControlMode::POSITION:
                driver_->setPositionSetpoint(setpoint_);
                break;
        }
        status_message_ = "Motor ENABLED - Use caution!";
    } else {
        status_message_ = "Motor disabled";
    }
}

void SoloTUI::emergencyStop() {
    driver_->emergencyStop();
    motor_enabled_ = false;
    setpoint_ = 0.0f;
    
    status_message_ = "EMERGENCY STOP - Motor disabled";
}

void SoloTUI::toggleLogging() {
    if (logging_enabled_) {
        closeLogFile();
        logging_enabled_ = false;
        status_message_ = "Logging stopped";
    } else {
        if (openLogFile()) {
            logging_enabled_ = true;
            log_start_time_ = std::chrono::steady_clock::now();
            status_message_ = "Logging started";
        } else {
            status_message_ = "Failed to open log file";
        }
    }
}

bool SoloTUI::openLogFile() {
    // Generate filename with timestamp
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::tm tm = *std::localtime(&time_t);
    
    std::stringstream filename;
    filename << "solo_log_" 
             << std::put_time(&tm, "%Y%m%d_%H%M%S")
             << ".csv";
    
    log_file_.open(filename.str());
    if (!log_file_.is_open()) {
        return false;
    }
    
    // Write header
    log_file_ << "timestamp_s,mode,setpoint,speed_rad_s,position_rad,torque_nm,"
              << "current_a,voltage_v,temperature_c,encoder_count,motor_enabled,"
              << "faults\n";
    
    return true;
}

void SoloTUI::closeLogFile() {
    if (log_file_.is_open()) {
        log_file_.close();
    }
}

void SoloTUI::logData() {
    if (!log_file_.is_open()) return;
    
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - log_start_time_).count() / 1000.0;
    
    const char* mode_str = "";
    switch (current_mode_) {
        case ControlMode::SPEED:    mode_str = "SPEED"; break;
        case ControlMode::TORQUE:   mode_str = "TORQUE"; break;
        case ControlMode::POSITION: mode_str = "POSITION"; break;
    }
    
    log_file_ << std::fixed << std::setprecision(3) << elapsed << ","
              << mode_str << ","
              << setpoint_ << ","
              << telemetry_.speed_rad_s << ","
              << telemetry_.position_rad << ","
              << telemetry_.torque_nm << ","
              << telemetry_.current_a << ","
              << telemetry_.voltage_v << ","
              << telemetry_.temperature_c << ","
              << telemetry_.encoder_count << ","
              << (motor_enabled_ ? 1 : 0) << ","
              << telemetry_.faults.toString() << "\n";
    
    log_file_.flush();
}

} // namespace solo

