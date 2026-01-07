# Root Cause: Package 'solo_usb_controller' Not Found

## Problem
After sourcing ROS2 and workspace setup, the package cannot be found:
```bash
$ source /opt/ros/jazzy/setup.bash
$ source ~/ros2_ws/install/setup.bash
$ ros2 run solo_usb_controller dual_track_tui_direct
Package 'solo_usb_controller' not found
```

## Root Cause Analysis

### Issue 1: Package Not Built/Installed
The package `solo_usb_controller` doesn't exist in `/home/jeeves/ros2_ws/install/` because the build failed.

### Issue 2: Build Failure - Missing Boost
The build fails with:
```
CMake Error: Could NOT find Boost (missing: Boost_INCLUDE_DIR system)
```

**Root Cause:** Boost development libraries are not installed on the system.

## Solution

### Step 1: Install Boost Development Libraries
```bash
sudo apt update
sudo apt install -y libboost-all-dev libboost-system-dev
```

### Step 2: Rebuild the Package
```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select solo_usb_controller --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Step 3: Source Workspace and Verify
```bash
source ~/ros2_ws/install/setup.bash
ros2 pkg list | grep solo
# Should show: solo_usb_controller

ros2 run solo_usb_controller dual_track_tui_direct
```

## Alternative: Use Install Dependencies Script
The package includes a script to install all dependencies:
```bash
cd ~/ros2_ws/src/SoloMini_CTL
./scripts/install_deps.sh
```

This will install:
- Build tools (cmake, gcc, etc.)
- Boost libraries (`libboost-all-dev`, `libboost-system-dev`)
- ncurses (`libncurses-dev`)
- ROS2 dependencies (if ROS2 is installed)

## Verification Checklist

- [ ] Boost libraries installed: `dpkg -l | grep boost`
- [ ] Package builds successfully: `colcon build --packages-select solo_usb_controller`
- [ ] Package appears in install: `ls ~/ros2_ws/install/solo_usb_controller/`
- [ ] Package listed by ROS2: `ros2 pkg list | grep solo`
- [ ] Executable exists: `ls ~/ros2_ws/install/solo_usb_controller/lib/solo_usb_controller/`
- [ ] Can run: `ros2 run solo_usb_controller dual_track_tui_direct`

## Why This Happened

When code is moved to a new machine, system dependencies need to be reinstalled. The `install_deps.sh` script should be run on the new machine before building.


