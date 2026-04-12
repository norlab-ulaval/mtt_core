#!/usr/bin/env python3
"""
Test script for MTT-154 multi-mode architecture.
Demonstrates switching between driving modes.
"""

import time
import rclpy
from rclpy.node import Node
from mtt_msgs.msg import MttDrivingMode


class ModeTestNode(Node):
    """Simple test node to demonstrate mode switching."""
    
    def __init__(self):
        super().__init__("mode_test_node")

        self.declare_parameter("mode_topic", "mtt_driving_mode")
        mode_topic = self.get_parameter("mode_topic").value

        self.mode_publisher = self.create_publisher(MttDrivingMode, mode_topic, 10)
        
        # Timer to cycle through modes every 10 seconds
        self.timer = self.create_timer(10.0, self.cycle_modes)
        
        self.current_mode = 0
        self.get_logger().info("Mode test node initialized - will cycle through driving modes")
    
    def cycle_modes(self):
        """Cycle through different driving modes for testing."""
        mode_names = ["Single Trailer", "Dual Differential", "Dual Serpentine"]
        
        # Publish mode change
        mode_msg = MttDrivingMode()
        mode_msg.mode = self.current_mode
        mode_msg.mode_parameters = f"Test mode {self.current_mode}"
        
        self.mode_publisher.publish(mode_msg)
        self.get_logger().info(f"Switched to mode {self.current_mode}: {mode_names[self.current_mode]}")
        
        # Cycle to next mode
        self.current_mode = (self.current_mode + 1) % 3


def main(args=None):
    rclpy.init(args=args)
    node = ModeTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
