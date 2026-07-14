"""Select a forward Teach rejoin point and ask Smac for feasible bypass paths."""

from __future__ import annotations

import copy
import math
import threading

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from mtt_interfaces.srv import ValidateCompositePath
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, String

from .geometry import (
    nearest_pose_index,
    path_length_xy,
    rejoin_candidate_indices,
)


class RejoinPlanner(Node):
    def __init__(self) -> None:
        super().__init__("mtt_rejoin_planner")
        self.declare_parameter("reference_path_topic", "/wiln/global_plan")
        self.declare_parameter("odom_topic", "/mapping/icp_measurement")
        self.declare_parameter("request_topic", "/mtt_avoidance/replan_request")
        self.declare_parameter("selected_path_topic", "/mtt_avoidance/selected_path")
        self.declare_parameter("plan_valid_topic", "/mtt_avoidance/plan_valid")
        self.declare_parameter("plan_clearance_topic", "/mtt_avoidance/plan_clearance_m")
        self.declare_parameter("status_topic", "/mtt_avoidance/planner_status")
        self.declare_parameter(
            "compute_path_action", "/mtt_avoidance/compute_path_to_pose"
        )
        self.declare_parameter("validation_service", "/mtt_avoidance/validate_path")
        self.declare_parameter("planner_id", "GridBased")
        self.declare_parameter("rejoin_distances_m", [4.0, 6.0, 8.0, 10.0, 12.0])
        self.declare_parameter("state_timeout_s", 0.75)
        self.declare_parameter("server_timeout_s", 2.0)
        self.declare_parameter("min_required_clearance_m", 0.05)
        self.declare_parameter("route_deviation_weight", 1.5)
        self.declare_parameter("clearance_weight", 2.0)

        self._reference_path: Path | None = None
        self._odom: Odometry | None = None
        self._odom_time = None
        self._planning = False
        self._candidate_indices: list[int] = []
        self._candidate_cursor = 0
        self._accepted: list[tuple[float, Path, float, int]] = []
        self._active_goal_handle = None
        self._lock = threading.Lock()

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Path,
            str(self.get_parameter("reference_path_topic").value),
            self._on_reference,
            latched,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._on_odom,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("request_topic").value),
            self._on_request,
            10,
        )
        self._selected_pub = self.create_publisher(
            Path, str(self.get_parameter("selected_path_topic").value), latched
        )
        self._valid_pub = self.create_publisher(
            Bool, str(self.get_parameter("plan_valid_topic").value), latched
        )
        self._clearance_pub = self.create_publisher(
            Float32, str(self.get_parameter("plan_clearance_topic").value), latched
        )
        self._status_pub = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), latched
        )
        self._planner = ActionClient(
            self,
            ComputePathToPose,
            str(self.get_parameter("compute_path_action").value),
        )
        self._validator = self.create_client(
            ValidateCompositePath,
            str(self.get_parameter("validation_service").value),
        )
        self._publish_status("idle")

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)

    def _publish_valid(self, valid: bool, clearance: float = 0.0) -> None:
        valid_msg = Bool()
        valid_msg.data = valid
        self._valid_pub.publish(valid_msg)
        clearance_msg = Float32()
        clearance_msg.data = float(clearance)
        self._clearance_pub.publish(clearance_msg)

    def _on_reference(self, msg: Path) -> None:
        if msg.poses:
            with self._lock:
                self._reference_path = msg

    def _on_odom(self, msg: Odometry) -> None:
        with self._lock:
            self._odom = msg
            self._odom_time = self.get_clock().now()

    def _on_request(self, msg: Bool) -> None:
        if not msg.data:
            return
        with self._lock:
            if self._planning:
                self._publish_status("planning already in progress")
                return
            reference = copy.deepcopy(self._reference_path)
            odom = copy.deepcopy(self._odom)
            odom_time = self._odom_time
            self._planning = True

        now = self.get_clock().now()
        if reference is None or not reference.poses:
            self._finish_failure("reference Teach path unavailable")
            return
        if odom is None or odom_time is None:
            self._finish_failure("robot pose unavailable")
            return
        age = (now - odom_time).nanoseconds * 1e-9
        if age > float(self.get_parameter("state_timeout_s").value):
            self._finish_failure(f"robot pose stale ({age:.2f}s)")
            return
        if reference.header.frame_id and odom.header.frame_id:
            if reference.header.frame_id != odom.header.frame_id:
                self._finish_failure(
                    "frame mismatch: "
                    f"route={reference.header.frame_id} odom={odom.header.frame_id}"
                )
                return

        current = odom.pose.pose.position
        nearest = nearest_pose_index(reference.poses, current.x, current.y)
        indices = rejoin_candidate_indices(
            reference.poses,
            nearest,
            list(self.get_parameter("rejoin_distances_m").value),
        )
        if not indices:
            self._finish_failure("no forward rejoin point remains on Teach path")
            return
        with self._lock:
            self._reference_path = reference
            self._odom = odom
            self._candidate_indices = indices
            self._candidate_cursor = 0
            self._accepted = []
        self._publish_valid(False)
        self._publish_status(f"planning {len(indices)} forward rejoin candidates")
        self._try_next_candidate()

    def _try_next_candidate(self) -> None:
        with self._lock:
            if self._candidate_cursor >= len(self._candidate_indices):
                self._finish_selection_locked()
                return
            index = self._candidate_indices[self._candidate_cursor]
            self._candidate_cursor += 1
            reference = self._reference_path
            odom = self._odom
        if not self._planner.wait_for_server(
            timeout_sec=float(self.get_parameter("server_timeout_s").value)
        ):
            self._finish_failure("Smac planner action unavailable")
            return

        goal = ComputePathToPose.Goal()
        goal.goal = copy.deepcopy(reference.poses[index])
        goal.start = PoseStamped()
        goal.start.header = copy.deepcopy(odom.header)
        goal.start.pose = copy.deepcopy(odom.pose.pose)
        goal.planner_id = str(self.get_parameter("planner_id").value)
        goal.use_start = True
        future = self._planner.send_goal_async(goal)
        future.add_done_callback(lambda done, idx=index: self._on_goal_response(done, idx))

    def _on_goal_response(self, future, index: int) -> None:
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001 - preserve planner error detail
            self._publish_status(f"candidate {index}: planner exception: {exc}")
            self._try_next_candidate()
            return
        if not handle.accepted:
            self._publish_status(f"candidate {index}: planner rejected goal")
            self._try_next_candidate()
            return
        self._active_goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda done, idx=index: self._on_plan_result(done, idx)
        )

    def _on_plan_result(self, future, index: int) -> None:
        try:
            wrapped = future.result()
            result = wrapped.result
        except Exception as exc:  # noqa: BLE001
            self._publish_status(f"candidate {index}: result exception: {exc}")
            self._try_next_candidate()
            return
        if (
            wrapped.status != GoalStatus.STATUS_SUCCEEDED
            or result.error_code != ComputePathToPose.Result.NONE
            or len(result.path.poses) < 3
        ):
            self._publish_status(
                f"candidate {index}: no Smac path ({result.error_code}: {result.error_msg})"
            )
            self._try_next_candidate()
            return
        if not self._validator.wait_for_service(
            timeout_sec=float(self.get_parameter("server_timeout_s").value)
        ):
            self._finish_failure("composite path validator unavailable")
            return
        request = ValidateCompositePath.Request()
        request.path = result.path
        validation = self._validator.call_async(request)
        validation.add_done_callback(
            lambda done, path=result.path, idx=index: self._on_validation(
                done, path, idx
            )
        )

    def _mean_route_deviation(self, path: Path) -> float:
        reference = self._reference_path
        if not reference or not reference.poses:
            return math.inf
        total = 0.0
        samples = path.poses[:: max(1, len(path.poses) // 30)]
        for pose in samples:
            px, py = pose.pose.position.x, pose.pose.position.y
            total += min(
                math.hypot(px - r.pose.position.x, py - r.pose.position.y)
                for r in reference.poses
            )
        return total / max(1, len(samples))

    def _on_validation(self, future, path: Path, index: int) -> None:
        try:
            result = future.result()
        except Exception as exc:  # noqa: BLE001
            self._publish_status(f"candidate {index}: validator exception: {exc}")
            self._try_next_candidate()
            return
        minimum = float(self.get_parameter("min_required_clearance_m").value)
        if not result.valid or result.min_clearance_m < minimum:
            self._publish_status(
                f"candidate {index}: composite rejection: {result.message}, "
                f"clearance={result.min_clearance_m:.2f}m"
            )
            self._try_next_candidate()
            return
        deviation = self._mean_route_deviation(path)
        score = (
            path_length_xy(path.poses)
            + float(self.get_parameter("route_deviation_weight").value) * deviation
            + float(self.get_parameter("clearance_weight").value)
            / max(result.min_clearance_m, 0.05)
        )
        with self._lock:
            self._accepted.append((score, path, result.min_clearance_m, index))
        self._publish_status(
            f"candidate {index}: accepted score={score:.2f} "
            f"clearance={result.min_clearance_m:.2f}m"
        )
        self._try_next_candidate()

    def _finish_selection_locked(self) -> None:
        if not self._accepted:
            self._planning = False
            self._publish_valid(False)
            self._publish_status("failed: no collision-free composite rejoin path")
            return
        score, path, clearance, index = min(self._accepted, key=lambda item: item[0])
        self._planning = False
        path.header.stamp = self.get_clock().now().to_msg()
        self._selected_pub.publish(path)
        self._publish_valid(True, clearance)
        self._publish_status(
            f"ready: selected rejoin index={index} score={score:.2f} "
            f"clearance={clearance:.2f}m"
        )

    def _finish_failure(self, reason: str) -> None:
        with self._lock:
            self._planning = False
        self._publish_valid(False)
        self._publish_status(f"failed: {reason}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RejoinPlanner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
