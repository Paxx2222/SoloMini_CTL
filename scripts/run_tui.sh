#!/bin/bash
# Run solo_tui_direct application

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"

# Source ROS2 base installation first (required for ros2 command)
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
ROS_INSTALL_PATH="/opt/ros/${ROS_DISTRO}"

if [ -f "${ROS_INSTALL_PATH}/setup.bash" ]; then
    source "${ROS_INSTALL_PATH}/setup.bash"
elif [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
fi

# Source workspace
if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    source "$WORKSPACE_DIR/install/setup.bash"
    # Add package bin directory to PATH for direct executable access
    export PATH="$WORKSPACE_DIR/install/solo_usb_controller/bin:$PATH"
else
    echo "Error: Workspace not built. Run ./scripts/build.sh first."
    exit 1
fi

# Default device
DEVICE="${1:-/dev/solo_mc_1}"

echo "Starting SOLO TUI Direct Control..."
echo "Device: $DEVICE"
echo ""
echo "Press ? for help once started"
echo "Press Q to quit"
echo ""

# Run application
ros2 run solo_usb_controller solo_tui_direct "$DEVICE"







