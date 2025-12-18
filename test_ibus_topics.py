#!/usr/bin/env python3
"""
Test script for IBUS ROS2 topics
Publishes to /ibus/ch2 (speed) and /ibus/ch1 (steering) for dual track controller
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import time
import sys

class IBUSTestPublisher(Node):
    def __init__(self):
        super().__init__('ibus_test_publisher')
        
        # Create publishers for IBUS channels
        self.speed_pub = self.create_publisher(Float64, '/ibus/ch2', 10)
        self.steering_pub = self.create_publisher(Float64, '/ibus/ch1', 10)
        
        self.get_logger().info('IBUS Test Publisher started')
        self.get_logger().info('Publishing to:')
        self.get_logger().info('  /ibus/ch2 - Speed (-1.0 = full reverse, 1.0 = full forward)')
        self.get_logger().info('  /ibus/ch1 - Steering (-1.0 = left, 0.0 = straight, 1.0 = right)')
        
    def publish_speed(self, speed):
        """Publish speed command (-1.0 to 1.0)"""
        msg = Float64()
        msg.data = float(speed)
        self.speed_pub.publish(msg)
        self.get_logger().info(f'Published speed: {speed:.2f}')
        
    def publish_steering(self, steering):
        """Publish steering command"""
        msg = Float64()
        msg.data = float(steering)
        self.steering_pub.publish(msg)
        self.get_logger().info(f'Published steering: {steering:.2f}')
        
    def publish_commands(self, speed, steering):
        """Publish both speed and steering"""
        self.publish_speed(speed)
        self.publish_steering(steering)

def main():
    rclpy.init()
    
    publisher = IBUSTestPublisher()
    
    if len(sys.argv) > 1:
        mode = sys.argv[1]
        
        if mode == 'test':
            # Test sequence: forward, stop, reverse, turn
            print("\n=== Running Test Sequence ===")
            
            print("\n1. Forward straight (speed=0.5, steering=0.0)")
            publisher.publish_commands(0.5, 0.0)
            time.sleep(2)
            
            print("\n2. Turn right (speed=0.3, steering=0.5)")
            publisher.publish_commands(0.3, 0.5)
            time.sleep(2)
            
            print("\n3. Turn left (speed=0.3, steering=-0.5)")
            publisher.publish_commands(0.3, -0.5)
            time.sleep(2)
            
            print("\n4. Forward straight again (speed=0.5, steering=0.0)")
            publisher.publish_commands(0.5, 0.0)
            time.sleep(2)
            
            print("\n5. Reverse (speed=-0.3, steering=0.0)")
            publisher.publish_commands(-0.3, 0.0)
            time.sleep(2)
            
            print("\n6. Stop (speed=0.0, steering=0.0)")
            publisher.publish_commands(0.0, 0.0)
            print("\nTest sequence complete!")
            
        elif mode == 'interactive':
            # Interactive mode
            print("\n=== Interactive Mode ===")
            print("Enter speed (-1.0 to 1.0) and steering (-1.0 to 1.0) values")
            print("Format: speed steering (e.g., '0.5 0.0' for forward, '0.3 0.5' for right turn)")
            print("  Speed: -1.0 = full reverse, 1.0 = full forward")
            print("  Steering: -1.0 = left, 0.0 = straight, 1.0 = right")
            print("Type 'q' to quit")
            
            try:
                while True:
                    user_input = input("\n> ").strip()
                    if user_input.lower() == 'q':
                        break
                    
                    try:
                        parts = user_input.split()
                        if len(parts) >= 1:
                            speed = float(parts[0])
                            steering = float(parts[1]) if len(parts) >= 2 else 0.0
                            publisher.publish_commands(speed, steering)
                        else:
                            print("Invalid input. Use: speed steering")
                    except ValueError as e:
                        print(f"Invalid number: {e}")
            except KeyboardInterrupt:
                pass
                
        elif mode == 'continuous':
            # Continuous publishing at specified rate
            rate_hz = 10.0
            if len(sys.argv) > 2:
                rate_hz = float(sys.argv[2])
            
            print(f"\n=== Continuous Mode (rate: {rate_hz} Hz) ===")
            print("Publishing speed=0.0, steering=0.0 (stop)")
            print("Press Ctrl+C to stop")
            
            rate = publisher.create_rate(rate_hz)
            try:
                while rclpy.ok():
                    publisher.publish_commands(0.0, 0.0)
                    rate.sleep()
            except KeyboardInterrupt:
                pass
        else:
            print(f"Unknown mode: {mode}")
            print("Usage: python3 test_ibus_topics.py [test|interactive|continuous] [rate_hz]")
    else:
        # Default: publish once and exit
        print("\n=== Single Publish ===")
        print("Publishing: speed=0.0, steering=0.0 (stop)")
        publisher.publish_commands(0.0, 0.0)
        time.sleep(0.1)
    
    # Clean shutdown
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
