#include "solo_usb_controller/dual_track_tui.hpp"
#include <iomanip>
#include <sstream>
#include <cmath>
#include <thread>
#include <chrono>

namespace solo {

DualTrackTUI::DualTrackTUI(const DualTrackDriver::Config& config)
    : config_(config)
    , running_(false)
    , motors_enabled_(false)
    , current_speed_(0.0f)
    , current_turn_ratio_(0.0f)
    , speed_step_(DEFAULT_SPEED_STEP)
    , turn_step_(DEFAULT_TURN_STEP)
    , telemetry_ok_(true)
    , screen_height_(0)
    , screen_width_(0)
    , show_help_(false)
    , logging_enabled_(false)
{
    driver_ = std::make_unique<DualTrackDriver>(config_);
}

DualTrackTUI::~DualTrackTUI() {
    shutdownUI();
    closeLogFile();
}

void DualTrackTUI::run() {
    initUI();
    
    // Try to connect
    status_message_ = "Connecting to motors...";
    drawUI();
    
    if (!driver_->connect()) {
        status_message_ = "ERROR: " + driver_->getLastError();
        drawUI();
        getch();
        return;
    }
    
    status_message_ = "Connected to both motors!";
    drawUI();
    
    // Configure both motors for operation
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Clear any existing faults first
    for (int i = 0; i < 3; i++) {
        driver_->clearFaults();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // Configure left motor
    status_message_ = "Configuring left motor...";
    drawUI();
    driver_->left().setMotorType(1);  // BLDC_PMSM
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    driver_->left().setFeedbackControlMode(1);  // ENCODERS
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    driver_->left().setCommandMode(1);  // DIGITAL
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    driver_->left().setControlMode(ControlMode::SPEED);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    // Configure right motor
    status_message_ = "Configuring right motor...";
    drawUI();
    driver_->right().setMotorType(1);  // BLDC_PMSM
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    driver_->right().setFeedbackControlMode(1);  // ENCODERS
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    driver_->right().setCommandMode(1);  // DIGITAL
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    driver_->right().setControlMode(ControlMode::SPEED);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    // Clear faults again after configuration
    driver_->clearFaults();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    status_message_ = "Ready - Press ? for help";
    
    running_ = true;
    last_update_ = std::chrono::steady_clock::now();
    
    // Main loop
    while (running_) {
        // Handle input (non-blocking)
        int ch = getch();
        if (ch != ERR) {
            handleInput(ch);
        }
        
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_update_).count();
        
        // Update telemetry at fixed rate
        if (elapsed >= UPDATE_RATE_MS) {
            telemetry_ok_ = driver_->readTelemetry(left_telemetry_, right_telemetry_);
            if (!telemetry_ok_) {
                telemetry_error_ = driver_->getLastError();
            } else {
                telemetry_error_.clear();
            }
            
            // Check for faults and auto-stop if needed
            if (motors_enabled_ && driver_->hasAnyFault()) {
                driver_->emergencyStop();
                motors_enabled_ = false;
                current_speed_ = 0.0f;
                current_turn_ratio_ = 0.0f;
                status_message_ = "FAULT DETECTED - Motors stopped!";
            }
            
            // Continuously send setpoints when motors enabled (SOLO needs periodic updates)
            if (motors_enabled_) {
                driver_->arcTurn(current_speed_, current_turn_ratio_);
            }
            
            // Log data if enabled
            if (logging_enabled_ && telemetry_ok_) {
                logData();
            }
            
            last_update_ = now;
        }
        
        drawUI();
    }
}

void DualTrackTUI::initUI() {
    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    curs_set(0);
    timeout(50);
    
    if (has_colors()) {
        start_color();
        init_pair(1, COLOR_GREEN, COLOR_BLACK);   // Normal/OK
        init_pair(2, COLOR_RED, COLOR_BLACK);     // Error/Fault
        init_pair(3, COLOR_YELLOW, COLOR_BLACK);  // Warning
        init_pair(4, COLOR_CYAN, COLOR_BLACK);    // Info
        init_pair(5, COLOR_WHITE, COLOR_RED);     // Emergency
        init_pair(6, COLOR_MAGENTA, COLOR_BLACK); // Highlight
    }
    
    getmaxyx(stdscr, screen_height_, screen_width_);
}

void DualTrackTUI::shutdownUI() {
    if (motors_enabled_) {
        driver_->emergencyStop();
    }
    endwin();
}

void DualTrackTUI::drawUI() {
    clear();
    
    drawHeader();
    drawConnectionStatus();
    
    if (driver_->hasAnyFault()) {
        drawFaultPanel();
    }
    
    drawControlPanel();
    drawTelemetryPanel();
    
    if (show_help_) {
        drawHelp();
    }
    
    drawStatusBar();
    
    refresh();
}

void DualTrackTUI::drawHeader() {
    attron(COLOR_PAIR(4) | A_BOLD);
    mvprintw(0, 0, "================================================================================");
    mvprintw(1, 0, "        DUAL TRACK CONTROLLER - Stage 2 - Differential Drive Control          ");
    mvprintw(2, 0, "================================================================================");
    attroff(COLOR_PAIR(4) | A_BOLD);
}

void DualTrackTUI::drawConnectionStatus() {
    int row = 4;
    
    // Left motor status
    if (driver_->left().isConnected()) {
        attron(COLOR_PAIR(1));
        mvprintw(row, 2, "LEFT:  ● CONNECTED");
        attroff(COLOR_PAIR(1));
    } else {
        attron(COLOR_PAIR(2));
        mvprintw(row, 2, "LEFT:  ● DISCONNECTED");
        attroff(COLOR_PAIR(2));
    }
    mvprintw(row, 25, "[%s]", config_.left_device.c_str());
    
    // Right motor status
    if (driver_->right().isConnected()) {
        attron(COLOR_PAIR(1));
        mvprintw(row, 50, "RIGHT: ● CONNECTED");
        attroff(COLOR_PAIR(1));
    } else {
        attron(COLOR_PAIR(2));
        mvprintw(row, 50, "RIGHT: ● DISCONNECTED");
        attroff(COLOR_PAIR(2));
    }
    mvprintw(row, 73, "[%s]", config_.right_device.c_str());
}

void DualTrackTUI::drawControlPanel() {
    int row = 6;
    
    attron(A_BOLD);
    mvprintw(row++, 2, "VEHICLE CONTROL");
    attroff(A_BOLD);
    mvprintw(row++, 2, "----------------------------------------------------------------------------");
    
    // Motor enable status
    if (motors_enabled_) {
        attron(COLOR_PAIR(1) | A_BOLD);
        mvprintw(row, 4, "Motors: ENABLED");
        attroff(COLOR_PAIR(1) | A_BOLD);
    } else {
        attron(COLOR_PAIR(3));
        mvprintw(row, 4, "Motors: DISABLED");
        attroff(COLOR_PAIR(3));
    }
    row++;
    row++;
    
    // Speed display with bar
    mvprintw(row, 4, "Speed:      %+6.1f rad/s", current_speed_);
    
    // Speed bar
    int bar_start = 28;
    int bar_width = 30;
    int center = bar_width / 2;
    int filled = static_cast<int>((current_speed_ / MAX_SPEED) * center);
    
    mvprintw(row, bar_start, "[");
    for (int i = 0; i < bar_width; i++) {
        if (i == center) {
            addch('|');
        } else if (filled > 0 && i > center && i <= center + filled) {
            attron(COLOR_PAIR(1));
            addch('#');
            attroff(COLOR_PAIR(1));
        } else if (filled < 0 && i < center && i >= center + filled) {
            attron(COLOR_PAIR(3));
            addch('#');
            attroff(COLOR_PAIR(3));
        } else {
            addch(' ');
        }
    }
    addch(']');
    row++;
    
    // Turn ratio display with bar
    mvprintw(row, 4, "Turn Ratio: %+6.2f", current_turn_ratio_);
    
    // Turn bar
    int turn_filled = static_cast<int>((current_turn_ratio_ / MAX_TURN_RATIO) * center);
    mvprintw(row, bar_start, "[");
    for (int i = 0; i < bar_width; i++) {
        if (i == center) {
            addch('|');
        } else if (turn_filled > 0 && i > center && i <= center + turn_filled) {
            attron(COLOR_PAIR(6));
            addch('>');
            attroff(COLOR_PAIR(6));
        } else if (turn_filled < 0 && i < center && i >= center + turn_filled) {
            attron(COLOR_PAIR(6));
            addch('<');
            attroff(COLOR_PAIR(6));
        } else {
            addch(' ');
        }
    }
    addch(']');
    row++;
    
    // Track velocities
    row++;
    float left_vel = current_speed_;
    float right_vel = current_speed_;
    if (current_turn_ratio_ >= 0) {
        right_vel = current_speed_ * (1.0f - current_turn_ratio_);
    } else {
        left_vel = current_speed_ * (1.0f + current_turn_ratio_);
    }
    
    mvprintw(row, 4, "Track Speeds:  L: %+6.1f rad/s    R: %+6.1f rad/s", left_vel, right_vel);
    row++;
    
    // Motion diagram
    row++;
    attron(COLOR_PAIR(4));
    if (std::abs(current_speed_) < 0.1f && std::abs(current_turn_ratio_) < 0.05f) {
        mvprintw(row, 4, "Motion: [STOPPED]");
    } else if (std::abs(current_turn_ratio_) > 0.9f && std::abs(current_speed_) < 0.1f) {
        mvprintw(row, 4, "Motion: [ROTATING %s]", current_turn_ratio_ > 0 ? "CW" : "CCW");
    } else if (std::abs(current_turn_ratio_) < 0.05f) {
        mvprintw(row, 4, "Motion: [%s]", current_speed_ > 0 ? "FORWARD" : "REVERSE");
    } else {
        mvprintw(row, 4, "Motion: [%s + %s]", 
                 current_speed_ > 0 ? "FWD" : "REV",
                 current_turn_ratio_ > 0 ? "RIGHT" : "LEFT");
    }
    attroff(COLOR_PAIR(4));
}

void DualTrackTUI::drawTelemetryPanel() {
    int row = 18;
    int left_col = 4;
    int right_col = 42;
    
    attron(A_BOLD);
    mvprintw(row++, 2, "TELEMETRY");
    attroff(A_BOLD);
    mvprintw(row++, 2, "----------------------------------------------------------------------------");
    
    if (!telemetry_ok_) {
        attron(COLOR_PAIR(2) | A_BOLD);
        mvprintw(row++, 4, "ERROR: Telemetry read failed!");
        attroff(COLOR_PAIR(2) | A_BOLD);
        if (!telemetry_error_.empty()) {
            mvprintw(row++, 4, "%s", telemetry_error_.substr(0, 70).c_str());
        }
        return;
    }
    
    // Column headers
    attron(A_BOLD | COLOR_PAIR(4));
    mvprintw(row, left_col, "LEFT MOTOR");
    mvprintw(row, right_col, "RIGHT MOTOR");
    attroff(A_BOLD | COLOR_PAIR(4));
    row++;
    
    // Speed
    mvprintw(row, left_col, "Speed:    %+8.2f rad/s", left_telemetry_.speed_rad_s);
    mvprintw(row, right_col, "Speed:    %+8.2f rad/s", right_telemetry_.speed_rad_s);
    row++;
    
    // Position
    mvprintw(row, left_col, "Position: %+8.2f rad", left_telemetry_.position_rad);
    mvprintw(row, right_col, "Position: %+8.2f rad", right_telemetry_.position_rad);
    row++;
    
    // Current
    mvprintw(row, left_col, "Current:  %8.2f A", left_telemetry_.current_a);
    mvprintw(row, right_col, "Current:  %8.2f A", right_telemetry_.current_a);
    row++;
    
    // Voltage
    mvprintw(row, left_col, "Voltage:  %8.2f V", left_telemetry_.voltage_v);
    mvprintw(row, right_col, "Voltage:  %8.2f V", right_telemetry_.voltage_v);
    row++;
    
    // Temperature with color coding
    auto draw_temp = [&](int col, float temp) {
        if (temp > 70.0f) {
            attron(COLOR_PAIR(2) | A_BOLD);
        } else if (temp > 50.0f) {
            attron(COLOR_PAIR(3));
        } else {
            attron(COLOR_PAIR(1));
        }
        mvprintw(row, col, "Temp:     %8.1f °C", temp);
        attroff(COLOR_PAIR(1) | COLOR_PAIR(2) | COLOR_PAIR(3) | A_BOLD);
    };
    
    draw_temp(left_col, left_telemetry_.temperature_c);
    draw_temp(right_col, right_telemetry_.temperature_c);
    row++;
    
    // Encoder
    mvprintw(row, left_col, "Encoder:  %8u", left_telemetry_.encoder_count);
    mvprintw(row, right_col, "Encoder:  %8u", right_telemetry_.encoder_count);
    row++;
    
    // Fault status
    row++;
    if (left_telemetry_.faults.hasAnyFault()) {
        attron(COLOR_PAIR(2));
        mvprintw(row, left_col, "FAULT: %s", left_telemetry_.faults.toString().c_str());
        attroff(COLOR_PAIR(2));
    } else {
        attron(COLOR_PAIR(1));
        mvprintw(row, left_col, "Status: OK");
        attroff(COLOR_PAIR(1));
    }
    
    if (right_telemetry_.faults.hasAnyFault()) {
        attron(COLOR_PAIR(2));
        mvprintw(row, right_col, "FAULT: %s", right_telemetry_.faults.toString().c_str());
        attroff(COLOR_PAIR(2));
    } else {
        attron(COLOR_PAIR(1));
        mvprintw(row, right_col, "Status: OK");
        attroff(COLOR_PAIR(1));
    }
    row++;
    
    // Logging and correction status
    row++;
    if (logging_enabled_) {
        attron(COLOR_PAIR(1));
        mvprintw(row, 4, "● LOGGING");
        attroff(COLOR_PAIR(1));
    }
    if (driver_->isSpeedCorrectionEnabled()) {
        attron(COLOR_PAIR(6));
        mvprintw(row, logging_enabled_ ? 18 : 4, "● SPEED CORRECTION");
        attroff(COLOR_PAIR(6));
    }
}

void DualTrackTUI::drawMotorColumn(int /*col*/, const std::string& /*label*/, const Telemetry& /*telemetry*/) {
    // Helper function - individual motor column (reserved for future use)
}

void DualTrackTUI::drawFaultPanel() {
    int row = screen_height_ / 2 - 4;
    int col = screen_width_ / 2 - 25;
    
    attron(COLOR_PAIR(5) | A_BOLD);
    mvprintw(row++, col, "====================================================");
    mvprintw(row++, col, "              ! FAULT DETECTED !                    ");
    mvprintw(row++, col, "----------------------------------------------------");
    
    if (left_telemetry_.faults.hasAnyFault()) {
        mvprintw(row++, col, "  LEFT:  %-40s", left_telemetry_.faults.toString().c_str());
    }
    if (right_telemetry_.faults.hasAnyFault()) {
        mvprintw(row++, col, "  RIGHT: %-40s", right_telemetry_.faults.toString().c_str());
    }
    
    mvprintw(row++, col, "====================================================");
    attroff(COLOR_PAIR(5) | A_BOLD);
}

void DualTrackTUI::drawHelp() {
    int row = 6;
    int col = 5;
    
    attron(COLOR_PAIR(4));
    mvprintw(row++, col, "================================= HELP =================================");
    mvprintw(row++, col, "                                                                        ");
    mvprintw(row++, col, "  MOTION CONTROLS:                                                      ");
    mvprintw(row++, col, "    W / UP      - Forward                S / DOWN   - Reverse           ");
    mvprintw(row++, col, "    A / LEFT    - Turn left              D / RIGHT  - Turn right        ");
    mvprintw(row++, col, "    Q           - Rotate CCW (left)      E          - Rotate CW (right) ");
    mvprintw(row++, col, "    X           - Stop (zero speed)                                     ");
    mvprintw(row++, col, "                                                                        ");
    mvprintw(row++, col, "  SPEED ADJUSTMENT:                                                     ");
    mvprintw(row++, col, "    +/=         - Increase speed step    -          - Decrease step     ");
    mvprintw(row++, col, "    [           - Finer turn ratio       ]          - Coarser turn      ");
    mvprintw(row++, col, "                                                                        ");
    mvprintw(row++, col, "  SYSTEM:                                                               ");
    mvprintw(row++, col, "    M           - Toggle motor enable    SPACE      - EMERGENCY STOP    ");
    mvprintw(row++, col, "    C           - Clear faults           L          - Toggle logging    ");
    mvprintw(row++, col, "    P           - Toggle speed correction (closed-loop)                 ");
    mvprintw(row++, col, "    ?           - Toggle this help       ESC / R    - Quit              ");
    mvprintw(row++, col, "                                                                        ");
    mvprintw(row++, col, "========================================================================");
    attroff(COLOR_PAIR(4));
}

void DualTrackTUI::drawStatusBar() {
    int row = screen_height_ - 1;
    
    attron(A_REVERSE);
    mvprintw(row, 0, "%-*s", screen_width_, status_message_.c_str());
    attroff(A_REVERSE);
    
    std::string hint = "Press ? for help | SPACE=E-STOP";
    mvprintw(row, screen_width_ - hint.length() - 1, "%s", hint.c_str());
}

void DualTrackTUI::handleInput(int ch) {
    switch (ch) {
        // Forward/Reverse
        case 'w':
        case 'W':
        case KEY_UP:
            cmdForward();
            break;
        case 's':
        case 'S':
        case KEY_DOWN:
            cmdReverse();
            break;
            
        // Turning
        case 'a':
        case 'A':
        case KEY_LEFT:
            cmdTurnLeft();
            break;
        case 'd':
        case 'D':
        case KEY_RIGHT:
            cmdTurnRight();
            break;
            
        // Rotation in place
        case 'q':
        case 'Q':
            cmdRotateLeft();
            break;
        case 'e':
        case 'E':
            cmdRotateRight();
            break;
            
        // Stop
        case 'x':
        case 'X':
            cmdStop();
            break;
            
        // Speed adjustment
        case '+':
        case '=':
            increaseSpeed();
            break;
        case '-':
        case '_':
            decreaseSpeed();
            break;
        case '[':
            decreaseTurnRate();
            break;
        case ']':
            increaseTurnRate();
            break;
            
        // Motor control
        case 'm':
        case 'M':
            toggleMotors();
            break;
        case ' ':
            emergencyStop();
            break;
        case 'c':
        case 'C':
            clearFaults();
            break;
            
        // Logging
        case 'l':
        case 'L':
            toggleLogging();
            break;
            
        // Speed correction toggle
        case 'p':
        case 'P':
            {
                bool enabled = !driver_->isSpeedCorrectionEnabled();
                driver_->enableSpeedCorrection(enabled);
                status_message_ = enabled ? "Speed correction ENABLED (Kp=0.5)" : "Speed correction DISABLED";
            }
            break;
            
        // Help
        case '?':
            show_help_ = !show_help_;
            break;
            
        // Quit
        case 'r':
        case 'R':
        case 27:  // ESC
            running_ = false;
            status_message_ = "Shutting down...";
            break;
    }
}

//=============================================================================
// Motion Commands
//=============================================================================

void DualTrackTUI::cmdForward() {
    current_speed_ += speed_step_;
    current_speed_ = std::min(current_speed_, MAX_SPEED);
    
    if (motors_enabled_) {
        driver_->arcTurn(current_speed_, current_turn_ratio_);
    }
    status_message_ = "Forward";
}

void DualTrackTUI::cmdReverse() {
    current_speed_ -= speed_step_;
    current_speed_ = std::max(current_speed_, -MAX_SPEED);
    
    if (motors_enabled_) {
        driver_->arcTurn(current_speed_, current_turn_ratio_);
    }
    status_message_ = "Reverse";
}

void DualTrackTUI::cmdTurnLeft() {
    current_turn_ratio_ -= turn_step_;
    current_turn_ratio_ = std::max(current_turn_ratio_, -MAX_TURN_RATIO);
    
    if (motors_enabled_) {
        driver_->arcTurn(current_speed_, current_turn_ratio_);
    }
    status_message_ = "Turn left";
}

void DualTrackTUI::cmdTurnRight() {
    current_turn_ratio_ += turn_step_;
    current_turn_ratio_ = std::min(current_turn_ratio_, MAX_TURN_RATIO);
    
    if (motors_enabled_) {
        driver_->arcTurn(current_speed_, current_turn_ratio_);
    }
    status_message_ = "Turn right";
}

void DualTrackTUI::cmdRotateLeft() {
    // Pure rotation: zero linear, full turn
    current_speed_ = 0.0f;
    current_turn_ratio_ = -1.0f;
    
    if (motors_enabled_) {
        driver_->rotateInPlace(-speed_step_ * 2);  // Use rotation speed
    }
    status_message_ = "Rotate CCW (left)";
}

void DualTrackTUI::cmdRotateRight() {
    // Pure rotation: zero linear, full turn
    current_speed_ = 0.0f;
    current_turn_ratio_ = 1.0f;
    
    if (motors_enabled_) {
        driver_->rotateInPlace(speed_step_ * 2);  // Use rotation speed
    }
    status_message_ = "Rotate CW (right)";
}

void DualTrackTUI::cmdStop() {
    current_speed_ = 0.0f;
    current_turn_ratio_ = 0.0f;
    
    if (motors_enabled_) {
        driver_->stop();
    }
    status_message_ = "Stopped";
}

//=============================================================================
// Speed Adjustment
//=============================================================================

void DualTrackTUI::increaseSpeed() {
    speed_step_ = std::min(speed_step_ + 0.5f, 10.0f);
    status_message_ = "Speed step: " + std::to_string(speed_step_) + " rad/s";
}

void DualTrackTUI::decreaseSpeed() {
    speed_step_ = std::max(speed_step_ - 0.5f, 0.5f);
    status_message_ = "Speed step: " + std::to_string(speed_step_) + " rad/s";
}

void DualTrackTUI::increaseTurnRate() {
    turn_step_ = std::min(turn_step_ + 0.05f, 0.5f);
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2) << "Turn step: " << turn_step_;
    status_message_ = ss.str();
}

void DualTrackTUI::decreaseTurnRate() {
    turn_step_ = std::max(turn_step_ - 0.05f, 0.05f);
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2) << "Turn step: " << turn_step_;
    status_message_ = ss.str();
}

//=============================================================================
// Motor Control
//=============================================================================

void DualTrackTUI::toggleMotors() {
    motors_enabled_ = !motors_enabled_;
    driver_->enableMotors(motors_enabled_);
    
    if (motors_enabled_) {
        // Send current setpoints when enabling
        driver_->arcTurn(current_speed_, current_turn_ratio_);
        status_message_ = "Motors ENABLED - Use caution!";
    } else {
        status_message_ = "Motors disabled";
    }
}

void DualTrackTUI::emergencyStop() {
    driver_->emergencyStop();
    motors_enabled_ = false;
    current_speed_ = 0.0f;
    current_turn_ratio_ = 0.0f;
    
    status_message_ = "!!! EMERGENCY STOP !!! - Motors disabled";
}

void DualTrackTUI::clearFaults() {
    if (driver_->clearFaults()) {
        status_message_ = "Faults cleared";
    } else {
        status_message_ = "Failed to clear faults: " + driver_->getLastError();
    }
}

//=============================================================================
// Logging
//=============================================================================

void DualTrackTUI::toggleLogging() {
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

bool DualTrackTUI::openLogFile() {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::tm tm = *std::localtime(&time_t);
    
    std::stringstream filename;
    filename << "dual_track_log_" 
             << std::put_time(&tm, "%Y%m%d_%H%M%S")
             << ".csv";
    
    log_file_.open(filename.str());
    if (!log_file_.is_open()) {
        return false;
    }
    
    // Write header
    log_file_ << "timestamp_s,speed_cmd,turn_ratio,"
              << "left_speed,left_pos,left_current,left_voltage,left_temp,"
              << "right_speed,right_pos,right_current,right_voltage,right_temp,"
              << "motors_enabled,left_fault,right_fault\n";
    
    return true;
}

void DualTrackTUI::closeLogFile() {
    if (log_file_.is_open()) {
        log_file_.close();
    }
}

void DualTrackTUI::logData() {
    if (!log_file_.is_open()) return;
    
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - log_start_time_).count() / 1000.0;
    
    log_file_ << std::fixed << std::setprecision(3) << elapsed << ","
              << current_speed_ << ","
              << current_turn_ratio_ << ","
              << left_telemetry_.speed_rad_s << ","
              << left_telemetry_.position_rad << ","
              << left_telemetry_.current_a << ","
              << left_telemetry_.voltage_v << ","
              << left_telemetry_.temperature_c << ","
              << right_telemetry_.speed_rad_s << ","
              << right_telemetry_.position_rad << ","
              << right_telemetry_.current_a << ","
              << right_telemetry_.voltage_v << ","
              << right_telemetry_.temperature_c << ","
              << (motors_enabled_ ? 1 : 0) << ","
              << left_telemetry_.faults.toString() << ","
              << right_telemetry_.faults.toString() << "\n";
    
    log_file_.flush();
}

} // namespace solo
