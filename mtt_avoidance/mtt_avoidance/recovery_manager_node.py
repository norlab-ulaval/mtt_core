"""Strictly gated Nav2 BackUp recovery; disabled until rear perception is ready."""

from __future__ import annotations

import math
import threading

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from nav2_msgs.action import BackUp
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, Float64, String
from std_srvs.srv import SetBool


class RecoveryManager(Node):
    def __init__(self) -> None:
        super().__init__("mtt_recovery_manager")
        self.declare_parameter("reverse_enabled", False)
        self.declare_parameter("allow_runtime_enable", False)
        self.declare_parameter("request_topic", "/mtt_avoidance/recovery_request")
        self.declare_parameter("rear_clearance_topic", "/mtt_obstacle/rear_clearance_m")
        self.declare_parameter("articulation_topic", "/hardware/articulation_angle")
        self.declare_parameter("odom_topic", "/mapping/icp_measurement")
        self.declare_parameter("backup_action", "/mtt_avoidance/backup")
        self.declare_parameter("backup_distance_m", 0.50)
        self.declare_parameter("backup_speed_ms", 0.12)
        self.declare_parameter("clearance_margin_m", 0.40)
        self.declare_parameter("max_articulation_rad", 0.20)
        self.declare_parameter("max_position_sigma_m", 0.25)
        self.declare_parameter("input_timeout_s", 0.5)
        self.declare_parameter("time_allowance_s", 10)
        self.declare_parameter("server_timeout_s", 2.0)

        self._enabled = bool(self.get_parameter("reverse_enabled").value)
        self._rear_clearance = math.nan
        self._rear_stamp = None
        self._articulation = math.nan
        self._articulation_stamp = None
        self._odom: Odometry | None = None
        self._odom_stamp = None
        self._goal_handle = None
        self._lock = threading.Lock()

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("request_topic").value),
            self._on_request,
            10,
        )
        self.create_subscription(
            Float32,
            str(self.get_parameter("rear_clearance_topic").value),
            self._on_rear_clearance,
            10,
        )
        self.create_subscription(
            Float64,
            str(self.get_parameter("articulation_topic").value),
            self._on_articulation,
            10,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._on_odom,
            10,
        )
        self._status_pub = self.create_publisher(
            String, "/mtt_avoidance/recovery_status", latched
        )
        self.create_service(
            SetBool, "/mtt_avoidance/set_reverse_recovery", self._set_enabled
        )
        self._backup = ActionClient(
            self, BackUp, str(self.get_parameter("backup_action").value)
        )
        self._publish("disabled" if not self._enabled else "armed")

    def _publish(self, text: str) -> None:
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)

    def _set_enabled(self, request, response):
        if request.data and not bool(self.get_parameter("allow_runtime_enable").value):
            response.success = False
            response.message = "runtime reverse enable locked by allow_runtime_enable=false"
            return response
        self._enabled = bool(request.data)
        if not self._enabled:
            self._cancel()
        response.success = True
        response.message = (
            "reverse recovery armed" if self._enabled else "reverse recovery disarmed"
        )
        self._publish(response.message)
        return response

    def _on_rear_clearance(self, msg: Float32) -> None:
        with self._lock:
            self._rear_clearance = float(msg.data)
            self._rear_stamp = self.get_clock().now()

    def _on_articulation(self, msg: Float64) -> None:
        with self._lock:
            self._articulation = float(msg.data)
            self._articulation_stamp = self.get_clock().now()

    def _on_odom(self, msg: Odometry) -> None:
        with self._lock:
            self._odom = msg
            self._odom_stamp = self.get_clock().now()

    def _fresh(self, stamp) -> bool:
        return stamp is not None and (
            self.get_clock().now() - stamp
        ).nanoseconds * 1e-9 <= float(self.get_parameter("input_timeout_s").value)

    def _on_request(self, msg: Bool) -> None:
        if not msg.data:
            self._cancel()
            return
        if not self._enabled:
            self._publish("refused: reverse recovery disabled")
            return
        with self._lock:
            clearance = self._rear_clearance
            clearance_stamp = self._rear_stamp
            articulation = self._articulation
            articulation_stamp = self._articulation_stamp
            odom = self._odom
            odom_stamp = self._odom_stamp
            active = self._goal_handle is not None
        if active:
            return
        if not self._fresh(clearance_stamp) or not math.isfinite(clearance):
            self._publish("refused: fresh rear clearance is mandatory")
            return
        required = float(self.get_parameter("backup_distance_m").value) + float(
            self.get_parameter("clearance_margin_m").value
        )
        if clearance < required:
            self._publish(
                f"refused: rear clearance {clearance:.2f}m < required {required:.2f}m"
            )
            return
        if not self._fresh(articulation_stamp) or not math.isfinite(articulation):
            self._publish("refused: fresh articulation is mandatory")
            return
        if abs(articulation) > float(self.get_parameter("max_articulation_rad").value):
            self._publish(f"refused: articulation {articulation:.2f}rad too high")
            return
        if odom is None or not self._fresh(odom_stamp):
            self._publish("refused: fresh localization is mandatory")
            return
        covariance = odom.pose.covariance
        sigma = math.sqrt(max(0.0, covariance[0], covariance[7]))
        if sigma > float(self.get_parameter("max_position_sigma_m").value):
            self._publish(f"refused: localization sigma {sigma:.2f}m too high")
            return
        if not self._backup.wait_for_server(
            timeout_sec=float(self.get_parameter("server_timeout_s").value)
        ):
            self._publish("refused: Nav2 BackUp server unavailable")
            return
        goal = BackUp.Goal()
        goal.target = Point(x=float(self.get_parameter("backup_distance_m").value))
        goal.speed = float(self.get_parameter("backup_speed_ms").value)
        goal.time_allowance = Duration(
            sec=int(self.get_parameter("time_allowance_s").value)
        )
        future = self._backup.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self._publish(f"failed: BackUp exception: {exc}")
            return
        if not handle.accepted:
            self._publish("failed: Nav2 rejected BackUp")
            return
        with self._lock:
            self._goal_handle = handle
        self._publish("recovering")
        result = handle.get_result_async()
        result.add_done_callback(self._on_result)

    def _on_result(self, future) -> None:
        try:
            wrapped = future.result()
            result = wrapped.result
            if wrapped.status == GoalStatus.STATUS_SUCCEEDED and result.error_code == 0:
                detail = "succeeded: bounded reverse recovery completed"
            else:
                detail = f"failed: BackUp {result.error_code}: {result.error_msg}"
        except Exception as exc:  # noqa: BLE001
            detail = f"failed: BackUp result exception: {exc}"
        with self._lock:
            self._goal_handle = None
        self._publish(detail)

    def _cancel(self) -> None:
        with self._lock:
            handle = self._goal_handle
            self._goal_handle = None
        if handle is not None:
            handle.cancel_goal_async()
            self._publish("canceled")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RecoveryManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
