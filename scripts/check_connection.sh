#!/bin/bash
# Check if SOLO controller is connected

echo "======================================"
echo "SOLO Controller Connection Check"
echo "======================================"
echo ""

echo "1. Checking for USB serial devices..."
if ls /dev/ttyACM* 2>/dev/null; then
    echo "   ✓ Found USB serial device(s)"
    for dev in /dev/ttyACM*; do
        echo "      - $dev"
        ls -l $dev
    done
else
    echo "   ✗ No /dev/ttyACM* devices found"
fi

if ls /dev/ttyUSB* 2>/dev/null; then
    echo "   ✓ Found USB serial device(s)"
    for dev in /dev/ttyUSB*; do
        echo "      - $dev"
        ls -l $dev
    done
fi

echo ""
echo "2. Checking for SOLO symlink..."
if [ -e /dev/solo_mc_1 ]; then
    echo "   ✓ /dev/solo_mc_1 exists"
    ls -l /dev/solo_mc_1
else
    echo "   ✗ /dev/solo_mc_1 not found"
    echo "   Run: sudo udevadm control --reload-rules && sudo udevadm trigger"
fi

echo ""
echo "3. USB devices:"
lsusb

echo ""
echo "======================================"
echo "If no devices found:"
echo "  1. Reconnect USB cable to SOLO"
echo "  2. Wait 5 seconds"
echo "  3. Run this script again"
echo "======================================"




