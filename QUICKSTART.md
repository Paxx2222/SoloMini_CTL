# Quick Start Guide

Get up and running with SOLO motor controller in 5 minutes.

## 1. Install Dependencies (One-time)

```bash
sudo apt update
sudo apt install -y libboost-all-dev libncurses-dev
```

## 2. Build

```bash
cd ~/ros2_ws
colcon build --packages-select solo_usb_controller
source install/setup.bash
```

## 3. Setup USB Device (One-time)

```bash
# Install udev rules
sudo cp ~/ros2_ws/src/solo_usb_controller/rules/99-solo.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

# Add yourself to dialout group
sudo usermod -a -G dialout $USER
# LOG OUT AND BACK IN for this to take effect
```

## 4. Connect Hardware

1. Connect SOLO Mini v2 to Raspberry Pi via USB
2. Connect motor to SOLO (per SOLO manual)
3. Connect power supply to SOLO (12-54V)
4. Verify device appears: `ls -l /dev/solo_mc_1`

## 5. Run the TUI

```bash
ros2 run solo_usb_controller solo_tui_direct
```

## 6. First Test

1. The TUI will open showing **DISCONNECTED** or **CONNECTED**
2. Press `?` to see help
3. Motor is **DISABLED** by default (safe)
4. Press `M` to select control mode (SPEED recommended for first test)
5. Press `E` to **ENABLE** motor ⚠️
6. Press `↑` to increase speed setpoint slowly
7. Motor should start spinning
8. Press `SPACE` for **EMERGENCY STOP** anytime
9. Press `Q` to quit

## Safety First! ⚠️

- **Test with motor unloaded** (disconnected from any mechanical load)
- **Emergency stop**: Press `SPACE` bar anytime
- Motor **auto-disables** when changing modes
- Keep hands clear of moving parts

## Troubleshooting

**Can't connect?**
```bash
# Check device exists
ls -l /dev/solo_mc_* /dev/ttyUSB*

# Try manual device path
ros2 run solo_usb_controller solo_tui_direct /dev/ttyUSB0
```

**Permission denied?**
```bash
# Check you're in dialout group
groups | grep dialout

# If not, add and log out/in
sudo usermod -a -G dialout $USER
```

**Faults shown?**
- Press `C` to clear faults
- Check power supply voltage (12-54V)
- Verify motor and encoder connections

## Next Steps

- Review full `README.md` for all features
- Try CSV logging: Press `L`
- Review `documents/PRD_Motor_Control_ROS2_SOLO_v1.md`
- Experiment with TORQUE and POSITION modes

---

**Need help?** See `README.md` or `documents/BUILDING.md`







