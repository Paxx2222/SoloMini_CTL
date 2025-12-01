#!/bin/bash
# Build script for solo_usb_controller

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"

echo "Building solo_usb_controller..."
echo "Workspace: $WORKSPACE_DIR"

cd "$WORKSPACE_DIR"

# Source ROS 2
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
else
    echo "Warning: ROS 2 Jazzy not found at /opt/ros/jazzy"
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







