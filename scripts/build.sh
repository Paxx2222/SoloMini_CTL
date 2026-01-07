#!/bin/bash
# Build script for solo_usb_controller

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"

echo "Building solo_usb_controller..."
echo "Workspace: $WORKSPACE_DIR"

cd "$WORKSPACE_DIR"

# Source ROS 2 - try to detect distribution automatically
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
ROS_INSTALL_PATH="/opt/ros/${ROS_DISTRO}"

if [ -f "${ROS_INSTALL_PATH}/setup.bash" ]; then
    source "${ROS_INSTALL_PATH}/setup.bash"
    echo "Sourced ROS 2 ${ROS_DISTRO} from ${ROS_INSTALL_PATH}"
elif [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
    echo "Sourced ROS 2 Jazzy (fallback)"
else
    echo "Warning: ROS 2 not found at ${ROS_INSTALL_PATH} or /opt/ros/jazzy"
    echo "Set ROS_DISTRO environment variable if using a different distribution"
fi

# Build package
colcon build --packages-select solo_usb_controller \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --event-handlers console_direct+

echo ""
echo "Build complete!"
echo ""
echo "Run: source install/setup.bash"
echo "Then: ros2 run solo_usb_controller solo_tui_direct"







