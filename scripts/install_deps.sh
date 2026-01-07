#!/bin/bash
# Install dependencies for solo_usb_controller

set -e

echo "Installing dependencies for solo_usb_controller..."

# Update package list
sudo apt update

# Install system dependencies
echo "Installing system dependencies..."
sudo apt install -y \
    build-essential \
    cmake \
    git \
    libboost-all-dev \
    libncurses-dev

# Install ROS dependencies if ROS is available
# Try to detect ROS distribution
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
ROS_INSTALL_PATH="/opt/ros/${ROS_DISTRO}"

if [ -f "${ROS_INSTALL_PATH}/setup.bash" ] || [ -f "/opt/ros/jazzy/setup.bash" ]; then
    echo "Installing ROS dependencies for ${ROS_DISTRO}..."
    sudo apt install -y \
        ros-${ROS_DISTRO}-rclcpp \
        ros-${ROS_DISTRO}-std-msgs \
        ros-${ROS_DISTRO}-sensor-msgs \
        ros-${ROS_DISTRO}-ament-cmake || {
        echo "Warning: Failed to install ROS ${ROS_DISTRO} packages. Trying jazzy..."
        sudo apt install -y \
            ros-jazzy-rclcpp \
            ros-jazzy-std-msgs \
            ros-jazzy-sensor-msgs \
            ros-jazzy-ament-cmake || {
            echo "Warning: Failed to install ROS dependencies. You may need to install manually."
        }
    }
else
    echo "Warning: ROS 2 not found. Skipping ROS dependencies."
    echo "Set ROS_DISTRO environment variable if using a different distribution"
fi

echo ""
echo "Dependencies installed successfully!"
echo ""
echo "Next steps:"
echo "1. Install udev rules: sudo cp rules/99-solo.rules /etc/udev/rules.d/"
echo "2. Add user to dialout: sudo usermod -a -G dialout \$USER"
echo "3. Log out and back in"
echo "4. Build: ./scripts/build.sh"







