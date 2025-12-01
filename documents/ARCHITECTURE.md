# Architecture Documentation - Stage 1

## Overview

Stage 1 implements direct USB control of SOLO Mini v2 motor controllers through a Terminal User Interface (TUI). This provides bring-up, testing, and debugging capabilities without requiring ROS 2 topic infrastructure.

## Component Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                    solo_tui_direct (executable)              │
│  ┌───────────────────────────────────────────────────────┐  │
│  │                    SoloTUI                            │  │
│  │  - ncurses UI rendering                               │  │
│  │  - Keyboard input handling                            │  │
│  │  - CSV logging                                        │  │
│  │  - Fault display                                      │  │
│  └───────────────────┬───────────────────────────────────┘  │
│                      │                                       │
│                      │ uses                                  │
│                      ▼                                       │
│  ┌───────────────────────────────────────────────────────┐  │
│  │                   SoloDriver                          │  │
│  │  - Control mode management                            │  │
│  │  - Setpoint commands (speed/torque/position)         │  │
│  │  - Telemetry reading                                  │  │
│  │  - Fault management                                   │  │
│  │  - PID configuration                                  │  │
│  │  - Motor enable/disable                               │  │
│  └───────────────────┬───────────────────────────────────┘  │
│                      │                                       │
│                      │ uses                                  │
│                      ▼                                       │
│  ┌───────────────────────────────────────────────────────┐  │
│  │                  SoloSerial                           │  │
│  │  - USB serial communication (Boost.Asio)              │  │
│  │  - SOLO protocol framing                              │  │
│  │  - CRC-16 validation                                  │  │
│  │  - Command/response handling                          │  │
│  └───────────────────┬───────────────────────────────────┘  │
│                      │                                       │
└──────────────────────┼───────────────────────────────────────┘
                       │
                       ▼
              ┌─────────────────┐
              │   /dev/solo_mc_1 │  (USB serial device)
              │                 │
              │  SOLO Mini v2   │
              │  Controller     │
              └─────────────────┘
                       │
                       ▼
                  ┌─────────┐
                  │  Motor  │
                  └─────────┘
```

## Class Hierarchy

### SoloSerial
**Responsibility:** Low-level USB serial communication  
**Location:** `include/solo_usb_controller/solo_serial.hpp`

**Key Methods:**
- `bool connect()` - Open serial port
- `bool writeCommand(uint8_t cmd, const std::vector<uint8_t>& data)` - Send command frame
- `bool readResponse(uint8_t& cmd, std::vector<uint8_t>& data, uint32_t timeout_ms)` - Receive response
- `uint16_t calculateCRC(const std::vector<uint8_t>& data)` - CRC-16-CCITT

**Dependencies:**
- Boost.Asio for serial port I/O

### SoloDriver
**Responsibility:** Motor control protocol implementation  
**Location:** `include/solo_usb_controller/solo_driver.hpp`

**Key Methods:**
- `bool setControlMode(ControlMode mode)` - Switch speed/torque/position
- `bool setSpeedSetpoint(float speed_rad_s)` - Command speed
- `bool setTorqueSetpoint(float torque_nm)` - Command torque
- `bool setPositionSetpoint(float position_rad, bool relative)` - Command position
- `bool readTelemetry(Telemetry& telemetry)` - Read all sensors
- `bool emergencyStop()` - Immediate motor disable

**Data Structures:**
- `Telemetry` - Speed, position, torque, current, voltage, temperature, faults
- `Fault` - Fault flags (overvoltage, overcurrent, etc.)
- `PIDConfig` - P, I, D, FF gains
- `MotorConfig` - Pole pairs, encoder CPR, motor type
- `Limits` - Speed, torque, current, voltage limits

**Dependencies:**
- `SoloSerial` for communication

### SoloTUI
**Responsibility:** Interactive terminal user interface  
**Location:** `include/solo_usb_controller/solo_tui.hpp`

**Key Methods:**
- `void run()` - Main event loop
- `void drawUI()` - Render all UI panels
- `void handleInput(int ch)` - Process keyboard input
- `void logData()` - Write CSV log entry

**UI Panels:**
1. Header - Title and branding
2. Connection Status - Device, firmware version
3. Control Panel - Mode, enable status, setpoint with bar graph
4. Telemetry - Real-time sensor readings
5. Faults - Blinking fault banner (if faults present)
6. Help - Keyboard shortcuts (toggle with ?)
7. Status Bar - Current action/message

**Dependencies:**
- `SoloDriver` for motor control
- ncurses for terminal rendering

## Protocol Implementation

### SOLO UART/USB Protocol

**Frame Format:**
```
┌────────┬─────────┬────────┬──────────────┬─────────────┐
│ HEADER │ COMMAND │ LENGTH │     DATA     │  CRC-16     │
│  0xAA  │  1 byte │ 1 byte │  0-N bytes   │  2 bytes    │
└────────┴─────────┴────────┴──────────────┴─────────────┘
```

**CRC:** CRC-16-CCITT (polynomial 0x1021, initial value 0xFFFF)

**Command IDs (partial list):**
- `0x01` - Write Speed Reference
- `0x02` - Write Torque Reference
- `0x03` - Write Position Reference
- `0x05` - Write Motor Enable
- `0x06` - Write Control Mode
- `0x81` - Read Speed Feedback
- `0x82` - Read Position Feedback
- `0x83` - Read Current Feedback
- `0x84` - Read Voltage Feedback
- `0x85` - Read Temperature
- `0x86` - Read Error Register

**Data Encoding:**
- Floats: 4 bytes, little-endian IEEE 754
- Integers: 4 bytes, little-endian

## Data Flow

### Command Flow (Setpoint Adjustment)

```
User Keyboard Input (↑/↓)
        ↓
SoloTUI::handleInput()
        ↓
SoloTUI::adjustSetpoint()
        ↓
SoloDriver::setSpeedSetpoint()  (or torque/position)
        ↓
SoloDriver::writeFloat()
        ↓
SoloSerial::writeCommand()
        ↓
[Frame: 0xAA | 0x01 | 0x04 | float_bytes | CRC]
        ↓
    /dev/solo_mc_1 (USB)
        ↓
    SOLO Controller
        ↓
    Motor PWM Output
```

### Telemetry Flow (Periodic Update)

```
Timer (10 Hz in TUI main loop)
        ↓
SoloDriver::readTelemetry()
        ↓
    ├─ readFloat(READ_SPEED_FEEDBACK)
    ├─ readFloat(READ_POSITION_FEEDBACK)
    ├─ readFloat(READ_CURRENT_FEEDBACK)
    ├─ readFloat(READ_VOLTAGE_FEEDBACK)
    ├─ readFloat(READ_TEMPERATURE)
    └─ readUInt32(READ_ERROR_REGISTER)
        ↓
Each: SoloSerial::writeCommand() then readResponse()
        ↓
    /dev/solo_mc_1 (USB)
        ↓
Telemetry struct populated
        ↓
SoloTUI::drawTelemetry() (display)
        │
        └─ SoloTUI::logData() (if logging enabled)
```

## Threading Model (Stage 1)

**Single-threaded:**
- Main thread runs TUI event loop
- Serial I/O is **blocking** with timeouts
- Telemetry polled at 10 Hz (100ms cycle)
- Input handled with 50ms getch() timeout

**Future (Stage 2+):**
- IO thread for non-blocking serial
- ROS executor thread
- Optional logger thread

## File Structure

```
solo_usb_controller/
│
├── include/solo_usb_controller/
│   ├── solo_serial.hpp       # Serial communication layer
│   ├── solo_driver.hpp       # Motor control driver
│   └── solo_tui.hpp          # TUI application
│
├── src/
│   ├── solo_serial.cpp       # Serial implementation
│   ├── solo_driver.cpp       # Driver implementation
│   ├── solo_tui.cpp          # TUI implementation
│   └── solo_tui_direct_main.cpp  # Entry point
│
├── scripts/
│   ├── build.sh              # Build helper
│   ├── install_deps.sh       # Dependency installer
│   ├── run_tui.sh            # Run helper
│   └── setup_udev.sh         # udev rules installer
│
├── config/
│   ├── motor1.yaml           # (Future: ROS node config)
│   └── dual.yaml             # (Future: Dual motor config)
│
├── documents/
│   ├── PRD_Motor_Control_ROS2_SOLO_v1.md  # Requirements
│   ├── ARCHITECTURE.md       # This file
│   ├── BUILDING.md           # Build instructions
│   ├── CHANGELOG.md          # Version history
│   ├── SOLO_MINI_Communication_Manual_UART_USB.pdf
│   └── SOLO_MINI_v2_SLM0322_4020_UserManual.pdf
│
├── rules/
│   └── 99-solo.rules         # udev rules for /dev/solo_mc_*
│
├── CMakeLists.txt            # Build configuration
├── package.xml               # ROS package metadata
├── README.md                 # User documentation
└── QUICKSTART.md             # Quick start guide
```

## Safety Architecture

### Defense in Depth

**Layer 1: Application Logic (SoloTUI)**
- Motor disabled by default on startup
- Emergency stop (SPACE) available at all times
- Mode changes automatically disable motor
- User confirmation for dangerous actions

**Layer 2: Driver Logic (SoloDriver)**
- Setpoint limits enforced in software
- State validation before commands
- Fault checking before enable

**Layer 3: SOLO Firmware**
- Hardware current limiting
- Thermal shutdown
- Voltage monitoring
- Watchdog timer

**Layer 4: Hardware**
- Power supply current limiting
- Motor thermal protection
- Emergency stop circuitry (future)

## Performance Characteristics (Stage 1)

| Metric | Target | Typical |
|--------|--------|---------|
| Command latency | < 20 ms | ~10 ms |
| Telemetry update rate | 10 Hz | 10 Hz |
| UI refresh rate | ~20 Hz | ~20 Hz |
| Serial timeout | 50 ms | 50 ms |
| Log write frequency | 10 Hz | 10 Hz |

## Future Architecture Evolution

### Stage 2: ROS 2 Node
- Add `solo_node` class
- Topic publishers/subscribers
- Service servers
- Parameter server integration
- Threading: IO thread + ROS executor

### Stage 3: ROS TUI
- `solo_tui_ros` uses ROS topics instead of direct driver
- Multi-motor namespace support

### Stage 4: Dual Motor
- Two instances of `solo_node`
- Synchronized control

### Stage 5: Tracked Vehicle
- `tracked_control_node`
- `/cmd_vel` → differential drive
- Odometry publishing

## References

- SOLO Protocol: `documents/SOLO_MINI_Communication_Manual_UART_USB.pdf`
- PRD: `documents/PRD_Motor_Control_ROS2_SOLO_v1.md`
- SOLO C++ Library: https://github.com/Solo-FL/SOLO-motor-controllers-CPP-library







