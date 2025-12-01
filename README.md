# SOLO USB Controller - Stage 1: Direct USB Control

Motor control stack for SOLO Mini v2 controllers via USB on ROS 2 Jazzy (Ubuntu 24.04, Raspberry Pi 4).

**Current Stage:** Stage 1 - `solo_tui_direct` (Direct USB Control TUI)

## Features (Stage 1)

✅ **Direct USB Control**
- Interactive Terminal User Interface (TUI) with ncurses
- Real-time telemetry display
- CSV data logging

✅ **Control Modes**
- Speed control (rad/s)
- Torque control (N·m)
- Position control (rad)

✅ **Safety Features**
- Emergency stop (SPACE)
- Fault detection and display
- Motor enable/disable safeguards
- Fault clearing

✅ **Monitoring**
- Speed, position, torque feedback
- Current and voltage monitoring
- Temperature monitoring with color coding
- Encoder counts
- Fault status

## Hardware Requirements

- **Compute:** Raspberry Pi 4 (4-8 GB RAM)
- **OS:** Ubuntu 24.04 LTS (64-bit)
- **Motor Driver:** SOLO Mini v2 (USB connection)
- **Motor:** BLDC or Brushed (per SOLO support)

## Dependencies

```bash
# System dependencies
sudo apt update
sudo apt install -y \
    libboost-dev \
    libboost-system-dev \
    libncurses-dev \
    ros-jazzy-rclcpp \
    ros-jazzy-std-msgs \
    ros-jazzy-sensor-msgs
```

## Installation

1. **Clone or copy this package to your ROS 2 workspace:**
   ```bash
   cd ~/ros2_ws/src
   # (assuming package is already here)
   ```

2. **Install udev rules for stable device names:**
   ```bash
   sudo cp rules/99-solo.rules /etc/udev/rules.d/
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   ```
   
   After connecting your SOLO controller, it should appear as `/dev/solo_mc_1`.

3. **Build the package:**
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select solo_usb_controller
   source install/setup.bash
   ```

## Usage

### Running the Direct USB TUI

**Default device (`/dev/solo_mc_1`):**
```bash
ros2 run solo_usb_controller solo_tui_direct
```

**Custom device:**
```bash
ros2 run solo_usb_controller solo_tui_direct /dev/ttyUSB0
```

### Keyboard Controls

| Key | Action |
|-----|--------|
| `↑` / `↓` | Adjust setpoint (large steps) |
| `←` / `→` | Fine adjust setpoint |
| `M` | Cycle control mode (Speed → Torque → Position) |
| `E` | Toggle motor enable/disable |
| `SPACE` | **Emergency stop** |
| `L` | Toggle CSV logging |
| `C` | Clear faults |
| `H` | Home position (set to 0) |
| `?` | Toggle help display |
| `Q` / `ESC` | Quit |

### Safety Notes

⚠️ **IMPORTANT:**
- Motor is **disabled** by default on startup
- Motor is **automatically disabled** when changing control modes
- Always test with motor unloaded initially
- Emergency stop (SPACE) immediately disables motor and zeros setpoint
- Review `documents/PRD_Motor_Control_ROS2_SOLO_v1.md` for safety guidelines

## Data Logging

Press `L` to start/stop CSV logging. Log files are saved as `solo_log_YYYYMMDD_HHMMSS.csv` in the current directory.

**CSV Format:**
```
timestamp_s,mode,setpoint,speed_rad_s,position_rad,torque_nm,current_a,voltage_v,temperature_c,encoder_count,motor_enabled,faults
```

## Configuration

Default limits (configurable in code):
- **Speed:** ±150 rad/s
- **Torque:** ±2.5 N·m
- **Current:** 20 A max
- **Voltage:** 12-54 V

Modify these in `solo_driver.hpp` and rebuild if needed.

## Troubleshooting

### Connection Issues

**Problem:** "Failed to connect" error
- Check USB cable connection
- Verify device path: `ls -l /dev/solo_mc_*` or `ls -l /dev/ttyUSB*`
- Check udev rules are installed
- Try specifying device manually: `solo_tui_direct /dev/ttyUSB0`

**Problem:** Permission denied
```bash
sudo usermod -a -G dialout $USER
# Log out and back in for changes to take effect
```

### Faults

If faults appear:
1. Press `C` to clear faults
2. Check power supply voltage (12-54V range)
3. Verify motor connections and encoder/hall sensors
4. Check for overcurrent or overtemperature conditions
5. Review SOLO manual in `documents/` folder

## Architecture

```
solo_usb_controller/
├── include/solo_usb_controller/
│   ├── solo_serial.hpp      # USB serial communication layer
│   ├── solo_driver.hpp      # High-level SOLO driver
│   └── solo_tui.hpp         # TUI application
├── src/
│   ├── solo_serial.cpp      # Serial implementation
│   ├── solo_driver.cpp      # Driver implementation
│   ├── solo_tui.cpp         # TUI implementation
│   └── solo_tui_direct_main.cpp  # Main entry point
├── config/                  # Configuration files (future ROS nodes)
├── documents/               # PRD and datasheets
└── rules/                   # udev rules
```

## Next Stages (Roadmap)

- [ ] **Stage 2:** ROS 2 node (`solo_node`) with topics/services
- [ ] **Stage 3:** ROS-based TUI (`solo_tui_ros`)
- [ ] **Stage 4:** Dual motor support
- [ ] **Stage 5:** Tracked vehicle control (`tracked_control_node`)

## Protocol Reference

The driver implements the SOLO Mini v2 UART/USB protocol as described in:
- `documents/SOLO_MINI_Communication_Manual_UART_USB.pdf`
- `documents/SOLO_MINI_v2_SLM0322_4020_UserManual.pdf`

Protocol features:
- CRC-16-CCITT checksumming
- Command/response model
- 115200 baud default
- 50ms timeout

## References

- **PRD:** `documents/PRD_Motor_Control_ROS2_SOLO_v1.md`
- **SOLO C++ Library Reference:** https://github.com/Solo-FL/SOLO-motor-controllers-CPP-library
- **ROS 2 Jazzy:** https://docs.ros.org/en/jazzy/

## License

Apache-2.0

## Maintainer

jeeves@todo.todo

---

**Version:** 1.0.0 (Stage 1 Complete)  
**Date:** 2025-10-22







