"""Fail-closed command selector between WILN tracking and guarded Nav2 bypass."""

from __future__ import annotations

import copy
import math
import threading

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class AvoidanceCommandMux(Node):
    def __init__(self) -> None:
        super().__init__("mtt_avoidance_command_mux")
        self.declare_parameter("wiln_cmd_topic", "/mtt_avoidance/wiln_cmd")
        self.declare_parameter("nav_cmd_topic", "/mtt_avoidance/nav_articulated_cmd")
        self.declare_parameter("override_topic", "/mtt_avoidance/override_requested")
        self.declare_parameter("output_topic", "/mtt_avoidance/selected_articulated_cmd")
        self.declare_parameter("input_timeout_s", 0.25)
        self.declare_parameter("switch_guard_s", 0.20)
        self.declare_parameter("publish_rate_hz", 20.0)

        self._wiln_cmd: TwistStamped | None = None
        self._wiln_stamp = None
        self._nav_cmd: TwistStamped | None = None
        self._nav_stamp = None
        self._override = False
        self._last_override = False
        self._last_articulation = 0.0
        self._switch_time = self.get_clock().now()
        self._lock = threading.Lock()
        self.create_subscription(
            TwistStamped,
            str(self.get_parameter("wiln_cmd_topic").value),
            self._on_wiln,
            10,
        )
        self.create_subscription(
            TwistStamped,
            str(self.get_parameter("nav_cmd_topic").value),
            self._on_nav,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("override_topic").value),
            self._on_override,
            10,
        )
        self._publisher = self.create_publisher(
            TwistStamped, str(self.get_parameter("output_topic").value), 10
        )
        rate = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().warning(
            "Canonical articulated mux output is "
            f"{str(self.get_parameter('output_topic').value)}; it must pass through "
            "mtt_articulated_command_dispatcher before any actuator topic."
        )

    def _on_wiln(self, msg: TwistStamped) -> None:
        with self._lock:
            self._wiln_cmd = msg
            self._wiln_stamp = self.get_clock().now()

    def _on_nav(self, msg: TwistStamped) -> None:
        with self._lock:
            self._nav_cmd = msg
            self._nav_stamp = self.get_clock().now()

    def _on_override(self, msg: Bool) -> None:
        with self._lock:
            self._override = bool(msg.data)

    def _fresh(self, now, stamp) -> bool:
        return stamp is not None and (now - stamp).nanoseconds * 1e-9 <= float(
            self.get_parameter("input_timeout_s").value
        )

    def _hold(self, now) -> TwistStamped:
        msg = TwistStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "base_footprint"
        msg.twist.angular.z = self._last_articulation
        return msg

    def _tick(self) -> None:
        now = self.get_clock().now()
        with self._lock:
            override = self._override
            if override != self._last_override:
                self._last_override = override
                self._switch_time = now
            switching = (now - self._switch_time).nanoseconds * 1e-9 < float(
                self.get_parameter("switch_guard_s").value
            )
            if override:
                command = copy.deepcopy(self._nav_cmd)
                stamp = self._nav_stamp
            else:
                command = copy.deepcopy(self._wiln_cmd)
                stamp = self._wiln_stamp
        if switching or command is None or not self._fresh(now, stamp):
            # Stop longitudinal motion but hold the last articulation. Returning
            # angular.z=0 here would unexpectedly recenter the joint on every
            # source transition or stale input.
            command = self._hold(now)
        else:
            if not (
                math.isfinite(command.twist.linear.x)
                and math.isfinite(command.twist.angular.z)
            ):
                command = self._hold(now)
            else:
                command.twist.angular.z = max(
                    -1.0, min(1.0, command.twist.angular.z)
                )
                self._last_articulation = command.twist.angular.z
            command.header.stamp = now.to_msg()
        self._publisher.publish(command)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AvoidanceCommandMux()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
