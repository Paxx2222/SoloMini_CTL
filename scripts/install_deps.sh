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
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    echo "Installing ROS dependencies..."
    sudo apt install -y \
        ros-jazzy-rclcpp \
        ros-jazzy-std-msgs \
        ros-jazzy-sensor-msgs \
        ros-jazzy-ament-cmake
else
    echo "Warning: ROS 2 Jazzy not found. Skipping ROS dependencies."
fi

echo ""
echo "Dependencies installed successfully!"
echo ""
echo "Next steps:"
echo "1. Install udev rules: sudo cp rules/99-solo.rules /etc/udev/rules.d/"
echo "2. Add user to dialout: sudo usermod -a -G dialout \$USER"
echo "3. Log out and back in"
echo "4. Build: ./scripts/build.sh"







