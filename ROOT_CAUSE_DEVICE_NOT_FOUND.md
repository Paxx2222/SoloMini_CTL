# Root Cause: Device Not Found Error

## Problem
```
ERROR: Left motor: Failed to connect: open: No such file or directory [system:2
```

The application is trying to connect to `/dev/solo_left` and `/dev/solo_right` but these symlinks don't exist.

## Root Cause

**Udev rules are NOT installed on the system.**

The udev rules file exists at `rules/99-solo.rules` but has not been copied to `/etc/udev/rules.d/`.

## USB Port Paths Detected

From the current system:
- `/dev/ttyACM0` → USB path: `1-1.4:1.0` (Right motor)
- `/dev/ttyACM1` → USB path: `1-1.3:1.0` (Left motor)

The udev rules file already has the correct paths (`1-1.3` and `1-1.4`), so no update needed - just installation.

## Solution

### Step 1: Install Udev Rules
```bash
cd ~/ros2_ws/src/SoloMini_CTL
sudo cp rules/99-solo.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### Step 2: Verify Symlinks Created
```bash
ls -la /dev/solo_*
# Should show:
# /dev/solo_left -> ttyACM1
# /dev/solo_right -> ttyACM0
# /dev/solo_mc_1 -> ttyACM0 (or ttyACM1)
```

### Step 3: Verify Permissions
```bash
ls -l /dev/solo_*
# Should show: crw-rw-rw- (readable/writable by all)
```

### Step 4: Test Application
```bash
ros2 run solo_usb_controller dual_track_tui_direct
```

## Alternative: Use Direct Device Paths

If you don't want to install udev rules, you can specify devices directly:
```bash
ros2 run solo_usb_controller dual_track_tui_direct -l /dev/ttyACM1 -r /dev/ttyACM0
```

## Why This Happened

When code is moved to a new machine, udev rules need to be reinstalled. The `setup_udev.sh` script should be run on the new machine.

## Quick Fix Script

```bash
cd ~/ros2_ws/src/SoloMini_CTL
./scripts/setup_udev.sh
```

This will:
1. Copy rules to `/etc/udev/rules.d/`
2. Reload udev rules
3. Trigger udev to create symlinks


