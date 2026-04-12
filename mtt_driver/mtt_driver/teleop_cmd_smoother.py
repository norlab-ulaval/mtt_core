#!/usr/bin/env python3

"""Rate-limit teleop TwistStamped commands and decay to zero on timeout."""

from __future__ import annotations

from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node


@dataclass
class _VelocityState:
    linear_x: float = 0.0
    angular_z: float = 0.0


def _step_towards(current: float, target: float, max_delta: float) -> float:
    if target > current + max_delta:
        return current + max_delta
    if target < current - max_delta:
        return current - max_delta
    return target


class TeleopCmdSmoother(Node):
    def __init__(self) -> None:
        super().__init__("teleop_cmd_smoother")

        self.declare_parameter("input_topic", "cmd_vel/teleop")
        self.declare_parameter("output_topic", "cmd_vel/teleop_smoothed")
        self.declare_parameter("input_timeout", 0.5)
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("max_accel_linear", 1.5)
        self.declare_parameter("max_accel_angular", 1.5)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self.input_timeout = float(self.get_parameter("input_timeout").value)
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.max_accel_linear = float(self.get_parameter("max_accel_linear").value)
        self.max_accel_angular = float(self.get_parameter("max_accel_angular").value)

        self._target = _VelocityState()
        self._current = _VelocityState()
        self._last_input = self.get_clock().now()
        self._last_update = self.get_clock().now()

        self._publisher = self.create_publisher(TwistStamped, output_topic, 10)
        self._subscription = self.create_subscription(
            TwistStamped,
            input_topic,
            self._input_callback,
            10,
        )
        self._timer = self.create_timer(1.0 / self.rate_hz, self._publish_smoothed_cmd)

    def _input_callback(self, msg: TwistStamped) -> None:
        self._target.linear_x = float(msg.twist.linear.x)
        self._target.angular_z = float(msg.twist.angular.z)
        self._last_input = self.get_clock().now()

    def _publish_smoothed_cmd(self) -> None:
        now = self.get_clock().now()
        dt = max((now - self._last_update).nanoseconds / 1e9, 1.0 / self.rate_hz)
        self._last_update = now

        age = (now - self._last_input).nanoseconds / 1e9
        if age > self.input_timeout:
            target = _VelocityState()
        else:
            target = self._target

        self._current.linear_x = _step_towards(
            self._current.linear_x,
            target.linear_x,
            self.max_accel_linear * dt,
        )
        self._current.angular_z = _step_towards(
            self._current.angular_z,
            target.angular_z,
            self.max_accel_angular * dt,
        )

        msg = TwistStamped()
        msg.header.stamp = now.to_msg()
        msg.twist.linear.x = self._current.linear_x
        msg.twist.angular.z = self._current.angular_z
        self._publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TeleopCmdSmoother()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
