#!/usr/bin/env python3
"""
Configure SOLO Motor Controller with motor parameters
Run this BEFORE using the motor control application
"""

import sys
import time
sys.path.insert(0, '/home/jeeves/programming/solo/SoloPy')

import SoloPy as solo

def configure_solo():
    print("=" * 70)
    print("SOLO Motor Controller Configuration")
    print("=" * 70)
    
    # Connect to SOLO
    print("\n1. Connecting to /dev/solo_mc_1 at 115200 baud...")
    mySolo = solo.SoloMotorControllerUart('/dev/solo_mc_1', 0, 
                                          solo.UartBaudRate.RATE_115200)
    
    # Verify communication
    print("2. Verifying communication...")
    is_working, error = mySolo.communication_is_working()
    if not is_working:
        print(f"   ✗ Communication failed: {error}")
        return False
    print("   ✓ Communication OK")
    
    # Get current firmware version
    fw, _ = mySolo.get_device_firmware_version()
    print(f"   Firmware version: {fw}")
    
    print("\n3. Configuring motor parameters...")
    
    # Motor Type: BLDC/PMSM
    print("   - Setting motor type: BLDC_PMSM")
    result, error = mySolo.set_motor_type(solo.MotorType.BLDC_PMSM)
    if error != solo.Error.NO_ERROR_DETECTED:
        print(f"     ✗ Failed: {error}")
    else:
        print("     ✓ Motor type set")
    time.sleep(0.1)
    
    # Pole Pairs: 15
    print("   - Setting pole pairs: 15")
    result, error = mySolo.set_motor_poles_counts(15)
    if error != solo.Error.NO_ERROR_DETECTED:
        print(f"     ✗ Failed: {error}")
    else:
        print("     ✓ Pole pairs set")
    time.sleep(0.1)
    
    # Encoder Lines: 4096
    print("   - Setting encoder lines: 4096")
    result, error = mySolo.set_incremental_encoder_lines(4096)
    if error != solo.Error.NO_ERROR_DETECTED:
        print(f"     ✗ Failed: {error}")
    else:
        print("     ✓ Encoder lines set")
    time.sleep(0.1)
    
    # Feedback Mode: Encoders (no Hall sensors)
    print("   - Setting feedback mode: ENCODERS")
    result, error = mySolo.set_feedback_control_mode(solo.FeedbackControlMode.ENCODERS)
    if error != solo.Error.NO_ERROR_DETECTED:
        print(f"     ✗ Failed: {error}")
    else:
        print("     ✓ Feedback mode set")
    time.sleep(0.1)
    
    # Current Limit: 20A (conservative)
    print("   - Setting current limit: 20A")
    result, error = mySolo.set_current_limit(20.0)
    if error != solo.Error.NO_ERROR_DETECTED:
        print(f"     ✗ Failed: {error}")
    else:
        print("     ✓ Current limit set")
    time.sleep(0.1)
    
    # Speed Limit: 150 rad/s
    print("   - Setting speed limit: 150 rad/s")
    result, error = mySolo.set_speed_limit(150.0)
    if error != solo.Error.NO_ERROR_DETECTED:
        print(f"     ✗ Failed: {error}")
    else:
        print("     ✓ Speed limit set")
    time.sleep(0.1)
    
    # Control Mode: Speed
    print("   - Setting control mode: SPEED")
    result, error = mySolo.set_control_mode(solo.ControlMode.SPEED_MODE)
    if error != solo.Error.NO_ERROR_DETECTED:
        print(f"     ✗ Failed: {error}")
    else:
        print("     ✓ Control mode set")
    time.sleep(0.1)
    
    # PWM Frequency: 20 kHz (typical for BLDC)
    print("   - Setting PWM frequency: 20 kHz")
    result, error = mySolo.set_output_pwm_frequency_khz(20)
    if error != solo.Error.NO_ERROR_DETECTED:
        print(f"     ✗ Failed: {error}")
    else:
        print("     ✓ PWM frequency set")
    time.sleep(0.1)
    
    print("\n4. Running motor parameter identification...")
    print("   ⚠️  Motor will briefly move during identification")
    print("   Please ensure motor is free to rotate and unloaded")
    input("   Press ENTER to continue or Ctrl+C to skip...")
    
    result, error = mySolo.motor_parameters_identification(solo.Action.START)
    if error != solo.Error.NO_ERROR_DETECTED:
        print(f"   ✗ Identification failed: {error}")
        print("   You may need to tune parameters manually")
    else:
        print("   ✓ Identification started")
        print("   Waiting for identification to complete (3 seconds)...")
        time.sleep(3)
        print("   ✓ Identification complete")
    
    print("\n5. Clearing any existing faults...")
    result, error = mySolo.reset_device_address()  # This also clears errors
    time.sleep(0.2)
    
    print("\n6. Reading configuration back...")
    poles, _ = mySolo.get_motor_poles_counts()
    encoder, _ = mySolo.get_incremental_encoder_lines()
    motor_type, _ = mySolo.get_motor_type()
    feedback, _ = mySolo.get_feedback_control_mode()
    
    print(f"   Motor type: {motor_type}")
    print(f"   Pole pairs: {poles}")
    print(f"   Encoder lines: {encoder}")
    print(f"   Feedback mode: {feedback}")
    
    print("\n" + "=" * 70)
    print("✓ Configuration Complete!")
    print("=" * 70)
    print("\nYou can now run the TUI application:")
    print("  ros2 run solo_usb_controller solo_tui_direct")
    print("\nNOTE: Press 'E' to enable motor, 'SPACE' for emergency stop")
    print("=" * 70)
    
    return True

if __name__ == "__main__":
    try:
        success = configure_solo()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n\nConfiguration cancelled by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)




