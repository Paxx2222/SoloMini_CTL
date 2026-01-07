#!/bin/bash
# Setup environment for solo_usb_controller
# Source this script to add executables to PATH

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"

# Source ROS2 base installation first (required for ros2 command)
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
ROS_INSTALL_PATH="/opt/ros/${ROS_DISTRO}"

if [ -f "${ROS_INSTALL_PATH}/setup.bash" ]; then
    source "${ROS_INSTALL_PATH}/setup.bash"
elif [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
else
    echo "Warning: ROS 2 base installation not found at ${ROS_INSTALL_PATH} or /opt/ros/jazzy"
    echo "Set ROS_DISTRO environment variable if using a different distribution"
fi

# Source ROS2 workspace
if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    source "$WORKSPACE_DIR/install/setup.bash"
    # Add package bin directory to PATH for direct executable access
    export PATH="$WORKSPACE_DIR/install/solo_usb_controller/bin:$PATH"
    echo "✓ SOLO USB Controller environment setup complete"
    echo "  You can now run: solo_tui_direct"
    echo "  Or use: ros2 run solo_usb_controller solo_tui_direct"
else
    echo "Error: Workspace not built. Run: cd ~/ros2_ws && colcon build --packages-select solo_usb_controller"
    return 1 2>/dev/null || exit 1
fi

