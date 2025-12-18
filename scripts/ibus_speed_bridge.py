#!/usr/bin/env python3
"""
IBUS Speed Bridge Node
Subscribes to /ibus/ch2 and controls both motors at the same speed
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import subprocess
import sys
import os

class IBUSSpeedBridge(Node):
    def __init__(self):
        super().__init__('ibus_speed_bridge')
        
        # Subscribe to IBUS speed channel
        self.speed_sub = self.create_subscription(
            Float64,
            '/ibus/ch2',
            self.speed_callback,
            10
        )
        
        self.get_logger().info('IBUS Speed Bridge started')
        self.get_logger().info('Subscribed to /ibus/ch2')
        self.get_logger().info('Controlling both motors at same speed')
        
        # Current speed command
        self.current_speed = 0.0
        
    def speed_callback(self, msg):
        """Handle speed command from /ibus/ch2"""
        speed_normalized = max(-1.0, min(1.0, msg.data))
        self.current_speed = speed_normalized
        
        self.get_logger().info(
            f'Speed command: {speed_normalized:.3f} '
            f'({speed_normalized * 100:.1f}%)'
        )
        
        # Note: This script would need to interface with the motor drivers
        # For now, it just logs the command
        # In a full implementation, you'd call the motor control API here

def main():
    rclpy.init()
    
    node = IBUSSpeedBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
