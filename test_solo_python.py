#!/usr/bin/env python3
"""
Test script based on official SoloPy library
Tests communication with SOLO motor controller to read firmware version, bus voltage, and temperature
"""

import sys
import time

try:
    import SoloPy as solo
except ImportError:
    print("ERROR: SoloPy library not installed!")
    print("Install with: pip install SoloPy")
    sys.exit(1)

def main():
    print("=" * 60)
    print("SOLO Motor Controller - Python Communication Test")
    print("=" * 60)
    
    # Configuration
    device = "/dev/solo_mc_1"
    address = 0  # Device address
    baudrate = solo.UartBaudRate.RATE_115200  # Use 115200 for Linux compatibility
    
    print(f"\nConfiguration:")
    print(f"  Device: {device}")
    print(f"  Address: {address}")
    print(f"  Baudrate: 115200")
    
    # Create SOLO controller instance
    print(f"\nConnecting to {device}...")
    try:
        mySolo = solo.SoloMotorControllerUart(
            port=device,
            address=address,
            baudrate=baudrate,
            timeout=1.0,  # 1 second timeout
            autoConnect=True
        )
        print("✓ Connected successfully!")
    except Exception as e:
        print(f"✗ Connection failed: {e}")
        return 1
    
    # Test 1: Read firmware version
    print("\n" + "-" * 60)
    print("Test 1: Reading Firmware Version")
    print("-" * 60)
    try:
        firmware, error = mySolo.get_device_firmware_version()
        if error == solo.Error.NO_ERROR_DETECTED:
            # Parse firmware version (format: major.minor.patch as integer)
            major = (firmware >> 16) & 0xFF
            minor = (firmware >> 8) & 0xFF
            patch = firmware & 0xFF
            print(f"✓ Firmware Version: {major}.{minor}.{patch} (0x{firmware:08X})")
        else:
            print(f"✗ Failed to read firmware: Error code {error}")
    except Exception as e:
        print(f"✗ Exception reading firmware: {e}")
    
    # Test 2: Read bus voltage
    print("\n" + "-" * 60)
    print("Test 2: Reading Bus Voltage")
    print("-" * 60)
    try:
        voltage, error = mySolo.get_bus_voltage()
        if error == solo.Error.NO_ERROR_DETECTED:
            print(f"✓ Bus Voltage: {voltage:.2f} V")
            if 12.0 <= voltage <= 54.0:
                print("  Status: Normal range")
            else:
                print("  Status: Outside normal range (12-54V)")
        else:
            print(f"✗ Failed to read voltage: Error code {error}")
    except Exception as e:
        print(f"✗ Exception reading voltage: {e}")
    
    # Test 3: Read temperature
    print("\n" + "-" * 60)
    print("Test 3: Reading Board Temperature")
    print("-" * 60)
    try:
        temperature, error = mySolo.get_board_temperature()
        if error == solo.Error.NO_ERROR_DETECTED:
            print(f"✓ Board Temperature: {temperature:.2f} °C")
            if temperature < 50:
                print("  Status: Normal")
            elif temperature < 70:
                print("  Status: Warm")
            else:
                print("  Status: Hot!")
        else:
            print(f"✗ Failed to read temperature: Error code {error}")
    except Exception as e:
        print(f"✗ Exception reading temperature: {e}")
    
    # Summary
    print("\n" + "=" * 60)
    print("Test Complete")
    print("=" * 60)
    
    return 0

if __name__ == "__main__":
    sys.exit(main())



