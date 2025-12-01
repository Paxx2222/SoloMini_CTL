# SOLO Motor Controller Communication Diagnostic Report

**Date:** 2025-10-29  
**Issue:** Application cannot establish communication with SOLO motor controller

## Summary

The `solo_tui_direct` application successfully:
- ✅ Opens the serial device `/dev/solo_mc_1` → `/dev/ttyACM0`
- ✅ Configures serial port (115200 8N1, no flow control)
- ✅ Sends commands to the controller

However:
- ❌ **NO RESPONSE** is received from the SOLO controller
- ❌ All read operations timeout
- ❌ Cannot read firmware version, telemetry, or any data

## Root Cause

**The SOLO motor controller is not responding to UART/USB protocol commands.**

## Test Results

### Serial Port Test
```bash
Device: /dev/solo_mc_1 -> /dev/ttyACM0
Permissions: crw-rw---- (jeeves is in dialout group)
Baud Rate: 115200
Configuration: 8N1, no flow control
Status: Opens successfully
```

### Communication Test
```bash
Command Sent: READ_FIRMWARE_VERSION (0xAA 0x87 0x00 + CRC)
Bytes Written: 5
Response: TIMEOUT (no data received)
```

## Possible Causes (In Order of Likelihood)

### 1. Controller Not Powered / Booted
- Check if SOLO has power LED lit
- Verify voltage supply (12-54V range for SOLO Mini v2)
- Controller may need time to boot after power-on

### 2. Wrong Communication Mode
- SOLO controllers can operate in different modes:
  - **UART/USB mode** (what we need)
  - **CAN mode**
  - **Modbus mode** (some models)
- Controller might be configured for CAN instead of UART/USB
- Check SOLO configuration switches/jumpers (refer to user manual)

### 3. Baud Rate Mismatch
- Current setting: 115200 (protocol default)
- Controller might be configured for:
  - 9600 baud (common default)
  - 57600 baud
  - 921600 baud (high-speed)
- SOLO may allow baud rate configuration via software or DIP switches

### 4. USB Enumeration Issue
- Device shows as `ttyACM0` (CDC ACM device - correct for SOLO)
- Controller might be in USB bootloader mode instead of application mode
- Try power-cycling the controller
- Try different USB port or cable

### 5. Firmware/Protocol Mismatch
- SOLO protocol implementation may not match your controller's firmware version
- Different firmware versions may use different command IDs or frame formats
- Check controller firmware version (if visible on controller or via SOLO software)

### 6. Controller Requires Initialization Sequence
- Some controllers need:
  - Specific "wake-up" command
  - Configuration mode entry sequence
  - Initial handshake or sync bytes
- Check SOLO communication manual for initialization requirements

## Diagnostic Steps

### Step 1: Verify Power and Status
```bash
# Check if controller LEDs are on
# Look for power LED, status LED, communication LED
```

### Step 2: Try Different Baud Rates
Create a test script to try common baud rates:
- 9600
- 19200
- 38400
- 57600
- 115200 (current)
- 230400
- 460800
- 921600

### Step 3: Check USB Enumeration
```bash
# View USB device info
lsusb -v | grep -A 20 "Solo\|ACM"

# Check kernel messages
dmesg | grep -i "ttyACM\|cdc_acm\|usb" | tail -20

# Check device attributes
udevadm info /dev/ttyACM0
```

### Step 4: Monitor Serial Communication
```bash
# Install minicom if not present
sudo apt install minicom

# Open serial connection (read-only monitoring)
minicom -D /dev/solo_mc_1 -b 115200

# Or use screen
screen /dev/solo_mc_1 115200
```

### Step 5: Check Official SOLO Software
If SOLO provides configuration software for Windows/Linux:
- Use it to verify controller is responsive
- Check current configuration (mode, baud rate, firmware version)
- Update firmware if available

### Step 6: Protocol Analysis
Use a logic analyzer or USB sniffer to:
- Verify data is being transmitted on TX line
- Check if controller is sending anything unsolicited
- Compare with official SOLO software communication

## Recommended Actions

### Immediate Actions

1. **Verify power to SOLO controller** - Look for LED indicators

2. **Try official SOLO software/tools** if available:
   - SOLO Motor Controllers Desktop App
   - SOLO Python library (`pip install solomotorcontrollers`)
   - Verify controller responds to official tools

3. **Check SOLO configuration**:
   - Look for DIP switches on controller
   - Check if controller mode selector is set to UART/USB
   - Verify no jumpers are configured for CAN mode

4. **Test with SOLO Python library**:
   ```bash
   pip install solomotorcontrollers
   python3 -c "from solomotorcontrollers import *; solo = SOLOUart('/dev/solo_mc_1'); print(solo.ReadDeviceAddress())"
   ```

### If Still Not Working

5. **Contact SOLO Support**:
   - Provide controller model number: SOLO Mini v2 (SLM0322-4020?)
   - Mention USB connection via CDC ACM (ttyACM0)
   - Ask about default baud rate and initialization sequence

6. **Check Hardware**:
   - Try different USB cable (must be data-capable, not charge-only)
   - Try different USB port on Raspberry Pi
   - Check for physical damage to controller USB connector

## Code Modifications to Try

### 1. Add Multi-Baud Rate Auto-Detection

Modify `SoloSerial::connect()` to try multiple baud rates:

```cpp
bool SoloSerial::connect() {
    std::vector<unsigned int> baud_rates = {115200, 9600, 57600, 19200, 921600};
    
    for (auto baud : baud_rates) {
        try {
            // Try connecting at this baud rate
            // Send ping command and wait for response
            // If successful, break
        } catch (...) {
            continue;
        }
    }
}
```

### 2. Add Communication Logging

Add debug output to see exactly what's being sent/received:

```cpp
// In solo_serial.cpp
bool SoloSerial::writeCommand(...) {
    // Log frame being sent
    std::cout << "TX: ";
    for (auto b : frame) printf("%02X ", b);
    std::cout << std::endl;
    // ...
}
```

### 3. Try Simplified Protocol Test

Send a simple command that might trigger a response:
- Broadcast address query
- Device reset command
- Status request

## References

- SOLO Communication Manual: `documents/SOLO_MINI_Communication_Manual_UART_USB.pdf`
- SOLO User Manual: `documents/SOLO_MINI_v2_SLM0322_4020_UserManual.pdf`
- SOLO GitHub: https://github.com/Solo-FL/SOLO-motor-controllers-CPP-library
- SOLO Website: https://www.solomotorcontrollers.com/

## Next Steps

1. **Verify SOLO controller power and status LEDs**
2. **Test with official SOLO Python library** to confirm controller works
3. **Check controller mode selection** (UART vs CAN vs Modbus)
4. **Try different baud rates** with test program
5. **Contact SOLO support** if hardware is confirmed working with official tools

---

**Diagnostic Tool:** `/home/jeeves/ros2_ws/src/solo_usb_controller/test_serial_comm.cpp`

**Test Command:**
```bash
cd /home/jeeves/ros2_ws/src/solo_usb_controller
./test_serial_comm
```




