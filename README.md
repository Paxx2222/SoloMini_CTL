# SOLO USB Controller - Dual Track Vehicle Control

Motor control stack for SOLO Mini v2 controllers via USB on ROS 2 Jazzy (Ubuntu 24.04, Raspberry Pi 4).

**Current Stage:** Stage 2 - `dual_track_tui_direct` (Dual Motor Tracked Vehicle Control)

## Features (Stage 2 - Dual Track Control)

✅ **Dual Motor Tracked Vehicle Control**
- Control two SOLO controllers for differential/tank drive
- Coordinated motion: forward, reverse, turn, rotate in place
- Side-by-side telemetry display for both motors
- Fault monitoring for both controllers

✅ **Motion Controls**
- Drive straight (same speed, opposite directions)
- Arc turns (differential speed)
- Rotate in place (pivot turn)
- Emergency stop (both motors)

✅ **Stage 1 Features (Single Motor)**
- Interactive Terminal User Interface (TUI) with ncurses
- Real-time telemetry display
- CSV data logging
- Speed/Torque/Position control modes

✅ **Safety Features**
- Emergency stop (SPACE) - stops both motors
- Fault detection on either motor triggers stop
- Synchronized enable/disable
- Per-motor fault display

## Hardware Requirements

- **Compute:** Raspberry Pi 4 (4-8 GB RAM)
- **OS:** Ubuntu 24.04 LTS (64-bit)
- **Motor Drivers:** 2x SOLO Mini v2 (USB connection)
- **Motors:** 2x BLDC or Brushed (per SOLO support)
- **Configuration:** Tracked/tank drive with motors on opposite sides

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
   
   After connecting your SOLO controllers, they should appear as:
   - `/dev/solo_left` - Left track motor (USB port 1-1.3)
   - `/dev/solo_right` - Right track motor (USB port 1-1.4)
   - `/dev/solo_mc_1` - Legacy single-motor symlink

3. **Build the package:**
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select solo_usb_controller
   source install/setup.bash
   ```

## Usage

### Stage 2: Dual Track Control TUI (Recommended)

**Running the Dual Track TUI:**
```bash
# Using udev symlinks (recommended)
ros2 run solo_usb_controller dual_track_tui_direct

# Or specify devices directly
ros2 run solo_usb_controller dual_track_tui_direct -l /dev/ttyACM1 -r /dev/ttyACM2
```

**Dual Track Keyboard Controls:**

| Key | Action |
|-----|--------|
| `W` / `↑` | Forward (increase speed) |
| `S` / `↓` | Reverse (decrease speed) |
| `A` / `←` | Turn left |
| `D` / `→` | Turn right |
| `Q` | Rotate counter-clockwise (in place) |
| `E` | Rotate clockwise (in place) |
| `X` | Stop (zero speed) |
| `M` | Toggle motors enable/disable |
| `SPACE` | **Emergency stop** (both motors) |
| `+` / `-` | Adjust speed step |
| `[` / `]` | Adjust turn sensitivity |
| `L` | Toggle CSV logging |
| `C` | Clear faults |
| `?` | Toggle help display |
| `ESC` | Quit |

---

### Stage 1: Single Motor TUI

**Running the Direct USB TUI:**

**Important:** The TUI requires an interactive terminal. Make sure you're running it directly (not piped/redirected).

**Option 1: Using udev symlink (recommended)**
```bash
# First, install udev rules (see Installation section above)
ros2 run solo_usb_controller solo_tui_direct
```

**Option 2: Specify device directly**
```bash
# If udev rules not installed, specify the device:
ros2 run solo_usb_controller solo_tui_direct /dev/ttyACM0
# or
ros2 run solo_usb_controller solo_tui_direct /dev/ttyACM1
```

**Finding your device:**
```bash
# List available CDC serial devices
ls -la /dev/ttyACM*

# Check USB devices
lsusb | grep -i solo
```

**Direct executable access (after sourcing workspace):**
```bash
# Option 1: Use the setup script
source ~/ros2_ws/src/SoloMini_CTL/scripts/setup_env.sh
solo_tui_direct

# Option 2: Manually add to PATH
export PATH="$HOME/ros2_ws/install/solo_usb_controller/bin:$PATH"
solo_tui_direct
```

### Stage 1 Keyboard Controls

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

### TUI Display Issues

**Problem:** TUI doesn't appear or shows garbled output
- **Solution:** The TUI requires an interactive terminal. Don't pipe or redirect output
- Run directly: `ros2 run solo_usb_controller solo_tui_direct /dev/ttyACM0`
- Make sure your terminal supports ncurses (most modern terminals do)
- If running over SSH, ensure your terminal is properly sized

### Connection Issues

**Problem:** "Failed to connect" or "No such file or directory" error
- Check USB cable connection
- Verify device path: `ls -l /dev/solo_mc_*` or `ls -l /dev/ttyACM*`
- If `/dev/solo_mc_1` doesn't exist:
  - Install udev rules: `sudo cp rules/99-solo.rules /etc/udev/rules.d/ && sudo udevadm control --reload-rules && sudo udevadm trigger`
  - Or specify device directly: `ros2 run solo_usb_controller solo_tui_direct /dev/ttyACM0`
- Check device permissions: `ls -l /dev/ttyACM*` (should be readable by dialout group)

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
│   ├── solo_serial.hpp         # USB serial communication layer
│   ├── solo_driver.hpp         # High-level SOLO driver (single motor)
│   ├── dual_track_driver.hpp   # Dual motor tracked vehicle driver
│   ├── solo_tui.hpp            # Stage 1: Single motor TUI
│   └── dual_track_tui.hpp      # Stage 2: Dual motor TUI
├── src/
│   ├── solo_serial.cpp         # Serial implementation
│   ├── solo_driver.cpp         # Driver implementation
│   ├── dual_track_driver.cpp   # Dual track driver implementation
│   ├── solo_tui.cpp            # Stage 1 TUI implementation
│   ├── dual_track_tui.cpp      # Stage 2 TUI implementation
│   ├── solo_tui_direct_main.cpp     # Stage 1 entry point
│   └── dual_track_tui_main.cpp      # Stage 2 entry point
├── config/                     # Configuration files
│   ├── motor1.yaml             # Single motor config
│   └── dual.yaml               # Dual motor config
├── documents/                  # PRD and datasheets
└── rules/
    └── 99-solo.rules           # udev rules for device symlinks
```

## Stages (Roadmap)

- [x] **Stage 1:** Direct USB TUI (`solo_tui_direct`) - Single motor control
- [x] **Stage 2:** Dual Track TUI (`dual_track_tui_direct`) - Tracked vehicle control
- [ ] **Stage 3:** ROS 2 node (`solo_node`) with topics/services
- [ ] **Stage 4:** ROS-based TUI (`solo_tui_ros`)
- [ ] **Stage 5:** Tracked vehicle ROS node (`tracked_control_node`) with `/cmd_vel`

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

**Version:** 2.0.0 (Stage 2 Complete)  
**Date:** 2025-12-11







