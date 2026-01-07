# Code Migration Issues - Fixed

This document summarizes issues found and fixed after moving the codebase to a new machine.

## Issues Found and Fixed

### 1. ✅ Hardcoded SoloPy Library Path (CRITICAL)
**Files:** `scripts/configure_solo.py`, `scripts/configure_solo_auto.py`

**Problem:** Scripts had hardcoded path `/home/jeeves/programming/solo/SoloPy` which won't exist on a new machine.

**Fix:** 
- Modified scripts to try importing SoloPy from pip installation first
- Added fallback to use `SOLOPY_PATH` environment variable if pip install not available
- Provides clear error message if library not found

**Action Required:**
- Install SoloPy: `pip install SoloPy`
- OR set `SOLOPY_PATH` environment variable if using local installation

### 2. ✅ Missing Launch Directory Reference
**File:** `CMakeLists.txt`

**Problem:** CMakeLists.txt references `launch/` directory for installation, but directory doesn't exist, causing build warnings.

**Fix:** Commented out the launch directory installation (can be uncommented when launch files are added).

### 3. ✅ USB Port Paths in Udev Rules (MACHINE-SPECIFIC)
**File:** `rules/99-solo.rules`

**Problem:** Udev rules contain hardcoded USB port paths (`1-1.3` and `1-1.4`) which are machine-specific and may differ on new hardware.

**Fix:** Added detailed comments explaining:
- USB port paths are machine-specific
- How to find correct paths using `udevadm info`
- Instructions to update rules for new machine

**Action Required:**
1. Plug in both SOLO controllers
2. Run: `udevadm info /dev/ttyACM0 | grep KERNELS` (and ttyACM1)
3. Update KERNELS values in `rules/99-solo.rules` to match your USB topology
4. Reload rules: `sudo udevadm control --reload-rules && sudo udevadm trigger`

### 4. ✅ ROS2 Distribution Path Detection
**Files:** `scripts/build.sh`, `scripts/install_deps.sh`

**Problem:** Scripts hardcoded `/opt/ros/jazzy` path, not flexible for different ROS2 distributions.

**Fix:** 
- Added automatic detection using `ROS_DISTRO` environment variable
- Falls back to `jazzy` if not set
- Provides clear warnings if ROS2 not found

**Action Required:**
- If using non-Jazzy distribution, set: `export ROS_DISTRO=your_distro`
- Scripts will automatically use the correct path

## Additional Notes

### Device Paths
- Default device paths (`/dev/solo_mc_1`, `/dev/solo_left`, `/dev/solo_right`) are created by udev rules
- If udev rules aren't set up, devices will appear as `/dev/ttyACM0`, `/dev/ttyACM1`, etc.
- Most code uses `/dev/solo_mc_1` as default, which should work after udev setup

### Dependencies
- Python: SoloPy library (install via pip or set SOLOPY_PATH)
- System: boost, ncurses, build tools
- ROS2: rclcpp, std_msgs, sensor_msgs, ament_cmake

### Build Process
1. Install dependencies: `./scripts/install_deps.sh`
2. Setup udev rules: `sudo ./scripts/setup_udev.sh` (then update USB port paths if needed)
3. Build: `./scripts/build.sh`
4. Source workspace: `source install/setup.bash`

## Verification Checklist

- [ ] SoloPy library installed or SOLOPY_PATH set
- [ ] ROS2 distribution detected correctly (check build.sh output)
- [ ] Udev rules updated with correct USB port paths
- [ ] User added to dialout group: `sudo usermod -a -G dialout $USER`
- [ ] Logged out and back in (for dialout group to take effect)
- [ ] SOLO devices appear as `/dev/solo_mc_1`, `/dev/solo_left`, `/dev/solo_right`
- [ ] Build completes successfully
- [ ] Test executable runs: `ros2 run solo_usb_controller solo_tui_direct`


