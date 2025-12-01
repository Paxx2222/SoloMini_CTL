# Changelog

All notable changes to the SOLO USB Controller project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.0.0] - 2025-10-22

### Added - Stage 1: Direct USB Control TUI

#### Core Driver Library
- `SoloSerial` class for USB serial communication
  - Boost.Asio-based serial port handling
  - CRC-16-CCITT frame validation
  - Configurable baudrate and timeout
- `SoloDriver` class for motor control
  - Speed, torque, and position control modes
  - Telemetry reading (speed, position, torque, current, voltage, temperature)
  - Fault detection and clearing
  - PID configuration
  - Motor enable/disable
  - Flash operations support
  - Device information queries

#### TUI Application (`solo_tui_direct`)
- Full-featured ncurses-based interface
- Real-time telemetry display
- Control mode cycling (Speed → Torque → Position)
- Keyboard-based setpoint adjustment
  - Large steps: ↑/↓ arrows
  - Fine steps: ←/→ arrows
- Motor enable/disable with safety interlocks
- Emergency stop (SPACE bar)
- CSV data logging with timestamps
- Fault banner display with color coding
- Interactive help display (?)
- Status bar with messages
- Color-coded temperature warnings

#### Safety Features
- Motor disabled by default on startup
- Automatic disable when changing control modes
- Emergency stop functionality
- Fault monitoring and display
- Setpoint limits enforcement
- Watchdog-friendly command structure

#### Configuration & Build
- CMake build system with ROS 2 integration
- Package.xml with all dependencies
- udev rules for stable device names (`/dev/solo_mc_1`)
- Boost and ncurses integration

#### Documentation
- Comprehensive README with usage instructions
- PRD (Product Requirements Document)
- SOLO communication manual reference
- SOLO user manual reference
- Keyboard shortcuts reference
- Troubleshooting guide
- Architecture overview

#### Data Logging
- CSV format with comprehensive telemetry
- Timestamped filenames (solo_log_YYYYMMDD_HHMMSS.csv)
- Columns: timestamp, mode, setpoint, speed, position, torque, current, voltage, temperature, encoder count, motor status, faults
- Toggle on/off during operation

### Protocol Implementation
- UART/USB frame protocol with header (0xAA)
- Command/response model
- Write commands: speed, torque, position, enable, mode, limits, PID
- Read commands: telemetry, faults, device info
- 115200 baud default
- 50ms timeout default

### Exit Criteria Met (Stage 1)
- ✅ Direct USB control working
- ✅ All three control modes (speed/torque/position)
- ✅ Real-time telemetry display
- ✅ Fault detection and clearing
- ✅ CSV logging
- ✅ Emergency stop
- ✅ User-friendly TUI interface
- ✅ Documentation complete

---

## [Unreleased] - Future Stages

### Planned - Stage 2: ROS 2 Node
- `solo_node` ROS 2 wrapper
- Topic interfaces for commands and telemetry
- Service interfaces for configuration
- Parameter server integration
- Diagnostic updater integration

### Planned - Stage 3: ROS TUI
- `solo_tui_ros` - TUI that uses ROS topics
- Multi-motor namespace support
- `/cmd_vel` viewer

### Planned - Stage 4: Dual Motor
- Two `solo_node` instances
- Launch file for dual setup
- Synchronized control

### Planned - Stage 5: Tracked Vehicle
- `tracked_control_node`
- `/cmd_vel` to differential drive conversion
- Odometry publishing
- TF publishing

---

## Version History

- **v1.0.0** (2025-10-22) - Stage 1: Direct USB Control TUI
- **v0.0.0** (Initial) - Project setup







