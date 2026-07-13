"""Execute a validated bypass with Nav2 FollowPath behind explicit safety locks."""

from __future__ import annotations

import copy
import threading

from action_msgs.msg import GoalStatus
from mtt_interfaces.srv import ValidateCompositePath
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool

from .geometry import nearest_pose_index


class AvoidanceExecutor(Node):
    def __init__(self) -> None:
        super().__init__("mtt_avoidance_executor")
        self.declare_parameter("shadow_mode", True)
        self.declare_parameter("execution_enabled", False)
        self.declare_parameter("allow_runtime_enable", False)
        self.declare_parameter("selected_path_topic", "/mtt_avoidance/selected_path")
        self.declare_parameter("odom_topic", "/mapping/icp_measurement")
        self.declare_parameter("execute_request_topic", "/mtt_avoidance/execute_request")
        self.declare_parameter("follow_path_action", "/mtt_avoidance/follow_path")
        self.declare_parameter("validation_service", "/mtt_avoidance/validate_path")
        self.declare_parameter("controller_id", "FollowPath")
        self.declare_parameter("goal_checker_id", "general_goal_checker")
        self.declare_parameter("progress_checker_id", "progress_checker")
        self.declare_parameter("path_timeout_s", 1.0)
        self.declare_parameter("server_timeout_s", 2.0)
        self.declare_parameter("revalidation_period_s", 0.5)

        self._shadow_mode = bool(self.get_parameter("shadow_mode").value)
        self._enabled = (
            bool(self.get_parameter("execution_enabled").value)
            and not self._shadow_mode
        )
        self._path: Path | None = None
        self._path_time = None
        self._odom: Odometry | None = None
        self._goal_handle = None
        self._active = False
        self._validation_pending = False
        self._lock = threading.Lock()

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Path,
            str(self.get_parameter("selected_path_topic").value),
            self._on_path,
            latched,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("execute_request_topic").value),
            self._on_request,
            10,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._on_odom,
            10,
        )
        self._status_pub = self.create_publisher(
            String, "/mtt_avoidance/executor_status", latched
        )
        self.create_service(SetBool, "/mtt_avoidance/set_execution", self._set_execution)
        self._follow = ActionClient(
            self,
            FollowPath,
            str(self.get_parameter("follow_path_action").value),
        )
        self._validator = self.create_client(
            ValidateCompositePath,
            str(self.get_parameter("validation_service").value),
        )
        period = max(0.1, float(self.get_parameter("revalidation_period_s").value))
        self.create_timer(period, self._revalidate)
        self._publish_status("disabled" if not self._enabled else "armed")

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)

    def _set_execution(self, request, response):
        if request.data and self._shadow_mode:
            response.success = False
            response.message = "shadow_mode=true is a hard execution lock"
            return response
        if request.data and not bool(self.get_parameter("allow_runtime_enable").value):
            response.success = False
            response.message = "runtime enable is locked by allow_runtime_enable=false"
            return response
        self._enabled = bool(request.data)
        if not self._enabled:
            self._cancel("execution disarmed")
        response.success = True
        response.message = "execution armed" if self._enabled else "execution disarmed"
        self._publish_status(response.message)
        return response

    def _on_path(self, msg: Path) -> None:
        with self._lock:
            self._path = msg
            self._path_time = self.get_clock().now()

    def _on_odom(self, msg: Odometry) -> None:
        with self._lock:
            self._odom = msg

    def _remaining_path(self, path: Path) -> Path:
        """Drop the already-traversed prefix before periodic safety checks.

        A newly appearing obstacle behind the robot must not cancel an otherwise
        safe bypass. One pose of overlap is retained to cover the current body.
        """
        with self._lock:
            odom = copy.deepcopy(self._odom)
        if odom is None or not path.poses:
            return path
        index = nearest_pose_index(
            path.poses,
            odom.pose.pose.position.x,
            odom.pose.pose.position.y,
        )
        remaining = copy.deepcopy(path)
        remaining.poses = remaining.poses[max(0, index - 1):]
        return remaining

    def _on_request(self, msg: Bool) -> None:
        if not msg.data:
            self._cancel("canceled by supervisor")
            return
        with self._lock:
            if self._active:
                return
            path = copy.deepcopy(self._path)
            path_time = self._path_time
        if not self._enabled:
            self._publish_status("refused: execution_enabled=false")
            return
        if path is None or not path.poses or path_time is None:
            self._publish_status("refused: selected path unavailable")
            return
        age = (self.get_clock().now() - path_time).nanoseconds * 1e-9
        if age > float(self.get_parameter("path_timeout_s").value):
            self._publish_status(f"refused: selected path stale ({age:.2f}s)")
            return
        if not self._validator.wait_for_service(
            timeout_sec=float(self.get_parameter("server_timeout_s").value)
        ):
            self._publish_status("refused: composite validator unavailable")
            return
        request = ValidateCompositePath.Request()
        request.path = path
        future = self._validator.call_async(request)
        future.add_done_callback(
            lambda done, candidate=path: self._on_initial_validation(done, candidate)
        )

    def _on_initial_validation(self, future, path: Path) -> None:
        try:
            validation = future.result()
        except Exception as exc:  # noqa: BLE001
            self._publish_status(f"refused: validation exception: {exc}")
            return
        if not validation.valid:
            self._publish_status(f"refused: {validation.message}")
            return
        if not self._follow.wait_for_server(
            timeout_sec=float(self.get_parameter("server_timeout_s").value)
        ):
            self._publish_status("refused: MPPI FollowPath action unavailable")
            return
        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = str(self.get_parameter("controller_id").value)
        goal.goal_checker_id = str(self.get_parameter("goal_checker_id").value)
        goal.progress_checker_id = str(self.get_parameter("progress_checker_id").value)
        future = self._follow.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self._publish_status(f"failed: FollowPath exception: {exc}")
            return
        if not handle.accepted:
            self._publish_status("failed: MPPI rejected FollowPath goal")
            return
        with self._lock:
            self._goal_handle = handle
            self._active = True
        self._publish_status("executing")
        result = handle.get_result_async()
        result.add_done_callback(self._on_result)

    def _on_result(self, future) -> None:
        try:
            wrapped = future.result()
            result = wrapped.result
            if wrapped.status == GoalStatus.STATUS_SUCCEEDED and result.error_code == 0:
                detail = "succeeded: rejoin path completed"
            elif wrapped.status == GoalStatus.STATUS_CANCELED:
                detail = "canceled: FollowPath canceled"
            else:
                detail = f"failed: FollowPath {result.error_code}: {result.error_msg}"
        except Exception as exc:  # noqa: BLE001
            detail = f"failed: FollowPath result exception: {exc}"
        with self._lock:
            self._active = False
            self._goal_handle = None
        self._publish_status(detail)

    def _cancel(self, reason: str) -> None:
        with self._lock:
            handle = self._goal_handle
            active = self._active
            self._active = False
            self._goal_handle = None
        if active and handle is not None:
            handle.cancel_goal_async()
            self._publish_status(f"canceled: {reason}")

    def _revalidate(self) -> None:
        with self._lock:
            if not self._active or self._validation_pending or self._path is None:
                return
            path = copy.deepcopy(self._path)
            self._validation_pending = True
        path = self._remaining_path(path)
        if not self._validator.service_is_ready():
            with self._lock:
                self._validation_pending = False
            self._cancel("validator unavailable during execution")
            return
        request = ValidateCompositePath.Request()
        request.path = path
        future = self._validator.call_async(request)
        future.add_done_callback(self._on_revalidation)

    def _on_revalidation(self, future) -> None:
        with self._lock:
            self._validation_pending = False
        try:
            result = future.result()
        except Exception:  # noqa: BLE001
            self._cancel("validator failure during execution")
            return
        if not result.valid:
            self._cancel(f"path became unsafe: {result.message}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AvoidanceExecutor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
