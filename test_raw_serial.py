#!/usr/bin/env python3
"""
Minimal raw serial test to verify port accepts writes
"""

import serial
import time
import sys

device = "/dev/solo_mc_1"
baudrate = 115200

print(f"Testing raw serial write to {device} at {baudrate} baud")
print("=" * 60)

try:
    # Open serial port with minimal configuration
    ser = serial.Serial(
        port=device,
        baudrate=baudrate,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=1.0,
        write_timeout=5.0  # Longer write timeout
    )
    
    print("✓ Port opened")
    time.sleep(0.2)  # Initialization delay
    
    # Flush buffers
    ser.flush()
    ser.flushInput()
    ser.flushOutput()
    print("✓ Buffers flushed")
    
    # Test write - send a simple SOLO command frame
    # Frame: [0xFF] [0xFF] [0x00] [0xA2] [0x00] [0x00] [0x00] [0x00] [0x00] [0xFE]
    test_frame = bytes([0xFF, 0xFF, 0x00, 0xA2, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE])
    
    print(f"\nSending test frame: {[hex(b) for b in test_frame]}")
    
    try:
        bytes_written = ser.write(test_frame)
        print(f"✓ Write successful: {bytes_written} bytes written")
    except serial.SerialTimeoutException as e:
        print(f"✗ Write timeout: {e}")
        print("\nPossible causes:")
        print("  1. USB serial adapter not accepting data")
        print("  2. Controller not powered or not ready")
        print("  3. USB cable issue (try different cable)")
        print("  4. USB port issue (try different port)")
        sys.exit(1)
    except Exception as e:
        print(f"✗ Write failed: {e}")
        sys.exit(1)
    
    # Wait for response
    time.sleep(0.1)
    print("\nWaiting for response...")
    
    # Try to read response
    response = ser.read(10)
    if len(response) > 0:
        print(f"✓ Response received: {len(response)} bytes")
        print(f"  Data: {[hex(b) for b in response]}")
    else:
        print("✗ No response received")
    
    ser.close()
    print("\nTest complete")
    
except serial.SerialException as e:
    print(f"✗ Serial error: {e}")
    sys.exit(1)
except Exception as e:
    print(f"✗ Error: {e}")
    sys.exit(1)



