#!/usr/bin/env python3
import time
import unittest
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

"""
Contract:
- Publish one TwistStamped on input topic (default: cmd_vel/teleop)
- Expect smoother to emit same direction immediately (within 0.5s)
- After input_timeout, output should decay to ~0 (abs < 1e-3)
Assumptions:
- teleop_cmd_smoother runs with defaults: input_topic=cmd_vel/teleop, output_topic=cmd_vel/teleop_smoothed, input_timeout=0.5
"""

class TestSmootherDecay(unittest.TestCase):
    def setUp(self):
        rclpy.init(args=None)
        self.node = Node('test_cmd_smoother_timeout')
        self.in_pub = self.node.create_publisher(TwistStamped, 'cmd_vel/teleop', 10)
        self.last_out = None
        self.out_sub = self.node.create_subscription(TwistStamped, 'cmd_vel/teleop_smoothed', self._out_cb, 10)

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def _out_cb(self, msg: TwistStamped):
        self.last_out = msg

    def spin_for(self, seconds: float):
        end = time.time() + seconds
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def test_decay_to_zero(self):
        # Send one input
        t = TwistStamped()
        t.twist.linear.x = 0.5
        t.twist.angular.z = 0.0
        self.in_pub.publish(t)
        self.spin_for(0.3)
        # Should have some output soon
        self.assertIsNotNone(self.last_out, 'No output from smoother')
        self.assertGreater(self.last_out.linear.x, 0.0, 'Output not following input sign')
        # Wait past timeout (0.6s)
        self.spin_for(0.7)
        # Expect near zero
        self.assertIsNotNone(self.last_out, 'No output after timeout')
        self.assertLess(abs(self.last_out.linear.x), 1e-3, 'Output did not decay to ~0 after timeout')
        self.assertLess(abs(self.last_out.angular.z), 1e-3, 'Angular output did not decay to ~0 after timeout')

if __name__ == '__main__':
    unittest.main()
