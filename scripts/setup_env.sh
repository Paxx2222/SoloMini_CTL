#!/bin/bash
# Setup environment for solo_usb_controller
# Source this script to add executables to PATH

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"

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

