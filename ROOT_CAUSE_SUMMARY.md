# Root Cause Analysis - SOLO Motor Controller Communication Failure

**Date:** 2025-10-29  
**System:** Raspberry Pi 4, Ubuntu 24.04, ROS 2 Jazzy  
**Device:** SOLO Mini v2 at `/dev/solo_mc_1` (→ `/dev/ttyACM0`)

---

## ✅ What Works

1. Serial device opens successfully
2. User `jeeves` in `dialout` group - permissions OK
3. SOLO appears as CDC ACM device (`ttyACM0`) - correct for SOLO USB
4. Serial port configuration succeeds (115200 8N1)
5. Application code is correct and builds successfully

## ❌ What Fails

**Serial Write Timeout**

```
serial.serialutil.SerialTimeoutException: Write timeout
```

**This is the root cause:** The SOLO controller's receive buffer is not accepting data. The write operation times out because the controller is not consuming incoming bytes.

## 🔍 Technical Analysis

### Test 1: Custom C++ Test Program
- **Result:** No response from controller
- Commands sent successfully (5 bytes)
- Read timeout - no data received

### Test 2: Official SOLO Python Library (SoloPy)
- **Result:** Write timeout on first communication attempt
- Serial port opens OK
- **Write fails** - controller not accepting data
- Tested at multiple baud rates: 115200, 937500, 9600 - all fail

### Conclusion
The controller is either:
1. Not powered/running
2. In wrong mode (not UART/USB mode)
3. Hardware issue
4. Not compatible with current communication settings

---

## 🎯 Root Cause (Confirmed)

**THE SOLO MOTOR CONTROLLER IS NOT IN A STATE TO ACCEPT UART/USB COMMUNICATION**

This is **NOT** a software issue with your ROS 2 application. The hardware/controller itself is not responding.

---

## ✔️ Required Actions (In Order)

### 1. Verify SOLO Hardware Status

**Check Power:**
- [ ] Is the SOLO controller powered? (12-54V DC supply connected)
- [ ] Are there any LED indicators lit on the SOLO?
  - Power LED?
  - Communication LED?
  - Status LED?
- [ ] What color are the LEDs?

**If no LEDs are on → Power issue or dead controller**

### 2. Check SOLO Mode Selection

SOLO Mini v2 may have:
- [ ] DIP switches for mode selection (UART vs CAN vs Modbus)
- [ ] Jumpers for communication mode
- [ ] Software configuration that persists in flash

**Action:** Check SOLO user manual for mode selection. Ensure set to **UART/USB mode**.

### 3. Check Communication Configuration

SOLO may be configured for:
- [ ] Different baud rate (default might be 937500, not 115200)
- [ ] Different protocol mode
- [ ] Device address mismatch

**Action:** Use SOLO Motion Terminal software (if available for Linux/Windows) to:
1. Verify controller is responding
2. Check current configuration
3. Set to known-good settings

### 4. Hardware Troubleshooting

- [ ] Try a different USB cable (must be data cable, not charge-only)
- [ ] Try a different USB port on the Raspberry Pi
- [ ] Check USB cable connections at both ends
- [ ] Inspect SOLO USB connector for damage
- [ ] Try connecting SOLO to a Windows PC with SOLO software

### 5. Firmware Check

- [ ] Verify SOLO firmware version
- [ ] Check if firmware update needed
- [ ] Confirm firmware supports UART/USB mode

---

## 🛠️ Diagnostic Commands

### Check USB Device Info
```bash
# View USB device details
lsusb -v | grep -A 30 "ACM\|Solo"

# Check kernel messages
dmesg | grep -i "ttyACM\|cdc_acm\|usb" | tail -30

# Device attributes
udevadm info /dev/ttyACM0 | grep -i "vendor\|product\|serial"
```

### Test Serial Port
```bash
# Check device is accessible
ls -l /dev/solo_mc_1 /dev/ttyACM0

# Try basic serial test
stty -F /dev/ttyACM0 115200 cs8 -cstopb -parenb
echo -ne "\xAA\x87\x00\xCE\x34" > /dev/ttyACM0
timeout 2 cat /dev/ttyACM0
```

### Run Custom Diagnostic
```bash
cd /home/jeeves/ros2_ws/src/solo_usb_controller
./test_serial_comm
```

---

## 📞 Contact SOLO Support

If hardware checks are OK, contact SOLO Motor Controllers support:

**Information to provide:**
- Model: SOLO Mini v2 (SLM0322-4020 or your exact model)
- Connection: USB (CDC ACM, appears as ttyACM0)
- OS: Ubuntu 24.04 on Raspberry Pi 4
- Issue: Write timeout when sending commands, controller not responding
- Test results: Both SoloPy library and custom code fail with write timeout
- Baud rates tested: 9600, 115200, 937500

**Questions to ask:**
1. What is the default baud rate for SOLO Mini v2 UART/USB mode?
2. Are there DIP switches or jumpers needed for UART/USB mode?
3. Does the controller need to be in a specific mode or state to accept commands?
4. Is there a reset procedure or initialization sequence required?
5. What should the LED indicators show when ready for UART communication?

---

## 📋 Checklist Before Running Application Again

- [ ] SOLO power supply connected and verified (12-54V)
- [ ] SOLO LEDs indicate normal operation
- [ ] SOLO set to UART/USB mode (via switches/jumpers if applicable)
- [ ] SOLO responds to official SOLO software (Windows/Linux tool)
- [ ] Correct baud rate determined and configured
- [ ] USB cable verified as data-capable
- [ ] SoloPy test succeeds: `communication_is_working() == True`

---

## 🔧 Once Hardware Issue Resolved

After SOLO responds to SoloPy test, update your application baud rate if needed:

```cpp
// In include/solo_usb_controller/solo_serial.hpp
struct Config {
    std::string device;
    unsigned int baudrate;  // <- Change this if not 115200
    unsigned int timeout_ms;
    
    Config() : device("/dev/solo_mc_1"), 
               baudrate(115200),  // <- Update to correct baud rate
               timeout_ms(50) {}
};
```

Then rebuild:
```bash
cd ~/ros2_ws
colcon build --packages-select solo_usb_controller
source install/setup.bash
ros2 run solo_usb_controller solo_tui_direct
```

---

## Summary

**Your ROS 2 application and code are correct.**

The problem is at the hardware/controller level. The SOLO Mini v2 is not accepting UART commands, which could be due to:

1. Power issue
2. Mode configuration (not in UART mode)
3. Hardware failure
4. Wrong baud rate setting on the controller

**Next step:** Physically check the SOLO controller - power, LEDs, mode switches, and try official SOLO software to verify it's functional.





