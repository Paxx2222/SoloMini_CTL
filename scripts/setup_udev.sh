#!/bin/bash
# Setup udev rules for SOLO controllers

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "Installing udev rules for SOLO motor controllers..."

# Copy rules
sudo cp "$PACKAGE_DIR/rules/99-solo.rules" /etc/udev/rules.d/

# Reload udev
sudo udevadm control --reload-rules
sudo udevadm trigger

echo ""
echo "udev rules installed successfully!"
echo ""
echo "SOLO devices will now appear as /dev/solo_mc_1, /dev/solo_mc_2, etc."
echo ""
echo "Don't forget to add your user to the dialout group:"
echo "  sudo usermod -a -G dialout \$USER"
echo ""
echo "Then log out and back in for changes to take effect."







