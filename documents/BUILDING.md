# Building and Installation Guide

## Prerequisites

### System Requirements

- **OS:** Ubuntu 24.04 LTS (64-bit)
- **ROS:** ROS 2 Jazzy
- **Hardware:** Raspberry Pi 4 (4-8 GB) or compatible x86_64 system

### Dependencies

#### ROS 2 Jazzy

If not already installed:
```bash
# Add ROS 2 repository
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl gnupg lsb-release

# Add ROS 2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
    sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Jazzy
sudo apt update
sudo apt install -y ros-jazzy-desktop
```

#### Build Tools and Dependencies

```bash
# Update package list
sudo apt update

# Install build tools
sudo apt install -y \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-rosdep

# Install library dependencies
sudo apt install -y \
    libboost-all-dev \
    libncurses-dev

# Install ROS dependencies
sudo apt install -y \
    ros-jazzy-rclcpp \
    ros-jazzy-std-msgs \
    ros-jazzy-sensor-msgs \
    ros-jazzy-ament-cmake
```

#### Initialize rosdep (if not done)

```bash
sudo rosdep init
rosdep update
```

## Building from Source

### 1. Create Workspace (if needed)

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 2. Clone or Verify Package

Ensure the `solo_usb_controller` package is in your workspace:
```bash
cd ~/ros2_ws/src
ls solo_usb_controller  # Should show the package directory
```

### 3. Install Dependencies with rosdep

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 4. Build the Package

```bash
cd ~/ros2_ws
colcon build --packages-select solo_usb_controller --cmake-args -DCMAKE_BUILD_TYPE=Release
```

**Build options:**
- Debug build: `-DCMAKE_BUILD_TYPE=Debug`
- Verbose: `--event-handlers console_direct+`
- Parallel jobs: `--parallel-workers 2` (useful on Raspberry Pi)

### 5. Source the Workspace

```bash
source ~/ros2_ws/install/setup.bash
```

Add to `~/.bashrc` for automatic sourcing:
```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

## Installing udev Rules

For stable device names (`/dev/solo_mc_1` instead of `/dev/ttyUSB0`):

```bash
cd ~/ros2_ws/src/solo_usb_controller
sudo cp rules/99-solo.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

**Add user to dialout group** (required for serial access):
```bash
sudo usermod -a -G dialout $USER
```
⚠️ **Log out and back in** for group changes to take effect.

## Verifying Installation

### Check if executable is built

```bash
ros2 run solo_usb_controller solo_tui_direct --help
```

### List installed files

```bash
ros2 pkg prefix solo_usb_controller
ls $(ros2 pkg prefix solo_usb_controller)/lib/solo_usb_controller/
```

Should show: `solo_tui_direct`

### Check device access

```bash
ls -l /dev/solo_mc_*
# or
ls -l /dev/ttyUSB*
```

## Troubleshooting Build Issues

### Issue: Cannot find Boost

```bash
sudo apt install -y libboost-all-dev
```

### Issue: Cannot find ncurses

```bash
sudo apt install -y libncurses-dev
```

### Issue: ROS packages not found

```bash
# Source ROS 2
source /opt/ros/jazzy/setup.bash

# Re-run build
cd ~/ros2_ws
colcon build --packages-select solo_usb_controller
```

### Issue: Permission denied on serial port

```bash
# Check current groups
groups

# Add to dialout if not present
sudo usermod -a -G dialout $USER

# Log out and back in, then verify
groups | grep dialout
```

### Issue: Build fails on Raspberry Pi (low memory)

```bash
# Limit parallel jobs
colcon build --packages-select solo_usb_controller --parallel-workers 1
```

### Issue: CMake version too old

```bash
# Check version
cmake --version

# Upgrade if needed (Ubuntu 24.04 should have 3.22+)
sudo apt update
sudo apt install -y cmake
```

## Clean Build

To rebuild from scratch:

```bash
cd ~/ros2_ws

# Remove build artifacts
rm -rf build/ install/ log/

# Rebuild
colcon build --packages-select solo_usb_controller
source install/setup.bash
```

## Running Tests (Future)

Once tests are implemented:

```bash
cd ~/ros2_ws
colcon test --packages-select solo_usb_controller
colcon test-result --verbose
```

## Uninstalling

```bash
# Remove build artifacts
cd ~/ros2_ws
rm -rf build/solo_usb_controller install/solo_usb_controller

# Remove udev rules
sudo rm /etc/udev/rules.d/99-solo.rules
sudo udevadm control --reload-rules
```

---

**Questions or Issues?**  
Check `README.md` for usage instructions or refer to the PRD in `documents/`.







