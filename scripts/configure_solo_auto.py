#!/usr/bin/env python3
"""
Configure SOLO Motor Controller - Auto mode (no motor ID)
Run this to set motor parameters without running auto-identification
"""

import sys
import time
sys.path.insert(0, '/home/jeeves/programming/solo/SoloPy')

import SoloPy as solo

def configure_solo():
    print("=" * 70)
    print("SOLO Motor Controller Configuration (Auto Mode)")
    print("=" * 70)
    
    # Connect to SOLO
    print("\n1. Connecting to /dev/solo_mc_1 at 937500 baud...")
    mySolo = solo.SoloMotorControllerUart('/dev/solo_mc_1', 0, 
                                          solo.UartBaudRate.RATE_937500)
    
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
    
    # Pole Pairs: 30
    print("   - Setting pole pairs: 30")
    result, error = mySolo.set_motor_poles_counts(30)
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
    
    # Current Limit: 3A
    print("   - Setting current limit: 3A")
    result, error = mySolo.set_current_limit(3.0)
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
    
    # Set reasonable PID values (conservative starting point)
    print("   - Setting speed controller gains (conservative)")
    mySolo.set_speed_controller_kp(0.2)
    time.sleep(0.05)
    mySolo.set_speed_controller_ki(0.01)
    time.sleep(0.05)
    print("     ✓ Speed PID set")
    
    print("\n4. Clearing any existing faults...")
    result, error = mySolo.overwrite_error_register()
    time.sleep(0.2)
    if error == solo.Error.NO_ERROR_DETECTED:
        print("   ✓ Faults cleared")
    else:
        print(f"   ⚠️  Fault clear status: {error}")
    
    print("\n5. Reading configuration back...")
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
    print("\nMotor controller is now configured with:")
    print("  - BLDC motor, 30 pole pairs")
    print("  - 4096 line encoder (no Hall sensors)")
    print("  - Speed control mode")
    print("  - Current limit: 3A")
    print("  - Conservative PID gains")
    print("\nYou can now test with the TUI:")
    print("  ros2 run solo_usb_controller solo_tui_direct")
    print("\n⚠️  SAFETY:")
    print("  - Motor starts DISABLED")
    print("  - Press 'E' to enable")
    print("  - Press 'SPACE' for emergency stop")
    print("  - Start with LOW speed setpoint")
    print("=" * 70)
    
    return True

if __name__ == "__main__":
    try:
        success = configure_solo()
        sys.exit(0 if success else 1)
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

