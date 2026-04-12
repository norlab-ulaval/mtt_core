#!/usr/bin/env python3
# Copyright
"""Pytest verifying teleop_cmd_smoother decays to zero after input timeout.

This test starts the smoother as a subprocess (ros2 run ...) to avoid depending
on launch_testing. It then publishes one Twist and checks the output decays to 0.
"""

import os
import signal
import subprocess
import time
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


class _Helper(Node):
    def __init__(self):
        super().__init__('test_smoother_helper')
        self.pub = self.create_publisher(TwistStamped, 'test/smoother/in', 10)
        self.last = None
        self.sub = self.create_subscription(TwistStamped, 'test/smoother/out', self._cb, 10)

    def _cb(self, msg: TwistStamped):
        self.last = msg

    def spin_for(self, sec: float):
        end = time.time() + sec
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_connections(self, timeout: float = 3.0):
        end = time.time() + timeout
        while time.time() < end:
            out_pubs = self.count_publishers('test/smoother/out')
            in_subs = self.count_subscribers('test/smoother/in')
            if out_pubs > 0 and in_subs > 0:
                return True
            self.spin_for(0.05)
        return False


def test_decay_to_zero_after_timeout():
    # Start smoother as a background process
    cmd = [
        'ros2', 'run', 'mtt_driver', 'teleop_cmd_smoother',
        '--ros-args',
        '-p', 'input_topic:=test/smoother/in',
        '-p', 'output_topic:=test/smoother/out',
        '-p', 'input_timeout:=0.3',
        '-p', 'rate_hz:=50.0',
        '-p', 'max_accel_linear:=10.0',
        '-p', 'max_accel_angular:=10.0',
    ]
    proc = subprocess.Popen(cmd, preexec_fn=os.setsid)
    # Give it a moment to start
    time.sleep(0.5)

    rclpy.init()
    node = _Helper()
    try:
        # Wait for ROS graph discovery to connect endpoints
        assert node.wait_connections(3.0), 'Graph did not connect endpoints in time'
        # Send one twist
        t = TwistStamped()
        t.twist.linear.x = 0.5
        # Publish a few times to avoid missing first message before discovery
        for _ in range(5):
            node.pub.publish(t)
            node.spin_for(0.1)
        assert node.last is not None, 'No output from smoother'
        assert node.last.twist.linear.x > 0.0
        # Wait beyond input_timeout
        node.spin_for(1.0)
        assert node.last is not None
        assert math.isclose(node.last.twist.linear.x, 0.0, abs_tol=1e-2)
        assert math.isclose(node.last.twist.angular.z, 0.0, abs_tol=1e-2)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except Exception:
            pass
