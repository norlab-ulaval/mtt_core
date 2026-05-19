#!/usr/bin/env python3

import math
import threading
import time
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry, Path
from norlab_controllers_msgs.action import FollowPath
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

from mtt_bringup.motion_model import (
    MotionModelParams,
    articulation_from_curvature,
    clamp,
    normalized_steer_from_articulation,
    slip_scale,
)


def wrap_to_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(quaternion) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        quaternion.w * quaternion.w
        + quaternion.x * quaternion.x
        - quaternion.y * quaternion.y
        - quaternion.z * quaternion.z,
    )


def distance_xy(pose_a, pose_b) -> float:
    return math.hypot(
        pose_a.position.x - pose_b.position.x,
        pose_a.position.y - pose_b.position.y,
    )


class MttPathFollower(Node):
    RESULT_STATUS_SUCCESS = 3
    RESULT_STATUS_ABORTED = 4
    RESULT_STATUS_INTERNAL_ERROR = 5
    RESULT_STATUS_PATH_LOST = 8

    FEEDBACK_STATUS_MOVING = 0
    FEEDBACK_STATUS_NO_LOCAL_PATH = 1

    def __init__(self) -> None:
        super().__init__("mtt_path_follower")

        self._callback_group = ReentrantCallbackGroup()
        self._state_lock = threading.Lock()
        self._goal_lock = threading.Lock()
        self._local_plan_lock = threading.Lock()
        self._odom_msg: Optional[Odometry] = None
        self._odom_received_time: Optional[float] = None
        self._local_plan_msg: Optional[Path] = None
        self._local_plan_received_time: Optional[float] = None
        self._local_plan_received = False
        self._active_goal = False
        self._path_lost_since: Optional[float] = None

        self.declare_parameter("action_name", "/follow_path")
        self.declare_parameter("odom_topic", "/mapping/icp_odom")
        self.declare_parameter("cmd_vel_topic", "controller/cmd_vel")
        self.declare_parameter("control_rate_hz", 20.0)
        # WILN publishes the live control plan on /wiln/control/local_plan.
        # This is NOT diagnostic-only: it is prioritised over the global path for steering.
        self.declare_parameter("local_plan_topic", "/wiln/control/local_plan")
        self.declare_parameter("local_plan_timeout_s", 0.5)
        self.declare_parameter("default_speed_ms", 0.60)
        self.declare_parameter("max_speed_ms", 0.80)
        self.declare_parameter("min_speed_ms", 0.25)
        self.declare_parameter("slowdown_alpha", 2.0)
        self.declare_parameter("k_y", 0.6)
        self.declare_parameter("k_theta", 1.2)
        self.declare_parameter("l_eq_m", 2.4)
        self.declare_parameter("psi_max_rad", math.radians(60.0))
        self.declare_parameter("psi_dot_max_rad_s", 0.5)
        self.declare_parameter("kappa_max", 0.7)
        self.declare_parameter("advance_distance_m", 0.35)
        self.declare_parameter("waypoint_tolerance_m", 0.35)
        self.declare_parameter("final_heading_tolerance_rad", 0.35)
        self.declare_parameter("odom_timeout_s", 2.5)
        self.declare_parameter("max_start_distance_m", 2.0)
        self.declare_parameter("max_lateral_error_m", 1.25)
        self.declare_parameter("max_heading_error_rad", 1.20)
        self.declare_parameter("max_target_distance_m", 4.0)
        self.declare_parameter("tracking_error_grace_s", 1.0)
        self.declare_parameter("model_use_slip_heuristic", True)
        self.declare_parameter("model_yaw_response_gain", 0.65)
        self.declare_parameter("model_yaw_slip_base", 0.10)
        self.declare_parameter("model_yaw_slip_speed_gain", 0.05)
        self.declare_parameter("model_yaw_slip_articulation_gain", 0.15)
        self.declare_parameter("model_yaw_slip_min_scale", 0.55)

        self._action_name = str(self.get_parameter("action_name").value)
        self._odom_topic = str(self.get_parameter("odom_topic").value)
        self._cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self._local_plan_topic = str(self.get_parameter("local_plan_topic").value)
        self._local_plan_timeout_s = float(self.get_parameter("local_plan_timeout_s").value)
        self._control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self._default_speed_ms = float(self.get_parameter("default_speed_ms").value)
        self._max_speed_ms = float(self.get_parameter("max_speed_ms").value)
        self._min_speed_ms = float(self.get_parameter("min_speed_ms").value)
        self._slowdown_alpha = float(self.get_parameter("slowdown_alpha").value)
        self._k_y = float(self.get_parameter("k_y").value)
        self._k_theta = float(self.get_parameter("k_theta").value)
        self._l_eq_m = float(self.get_parameter("l_eq_m").value)
        self._psi_max_rad = float(self.get_parameter("psi_max_rad").value)
        self._psi_dot_max_rad_s = float(self.get_parameter("psi_dot_max_rad_s").value)
        self._kappa_max = float(self.get_parameter("kappa_max").value)
        self._advance_distance_m = float(self.get_parameter("advance_distance_m").value)
        self._waypoint_tolerance_m = float(self.get_parameter("waypoint_tolerance_m").value)
        self._final_heading_tolerance_rad = float(self.get_parameter("final_heading_tolerance_rad").value)
        self._odom_timeout_s = float(self.get_parameter("odom_timeout_s").value)
        self._max_start_distance_m = float(self.get_parameter("max_start_distance_m").value)
        self._max_lateral_error_m = float(self.get_parameter("max_lateral_error_m").value)
        self._max_heading_error_rad = float(self.get_parameter("max_heading_error_rad").value)
        self._max_target_distance_m = float(self.get_parameter("max_target_distance_m").value)
        self._tracking_error_grace_s = float(self.get_parameter("tracking_error_grace_s").value)
        self._motion_model_params = MotionModelParams(
            wheelbase_m=self._l_eq_m,
            max_articulation_rad=self._psi_max_rad,
            min_turn_speed_ms=self._min_speed_ms,
            use_slip_heuristic=bool(self.get_parameter("model_use_slip_heuristic").value),
            yaw_response_gain=float(self.get_parameter("model_yaw_response_gain").value),
            yaw_slip_base=float(self.get_parameter("model_yaw_slip_base").value),
            yaw_slip_speed_gain=float(self.get_parameter("model_yaw_slip_speed_gain").value),
            yaw_slip_articulation_gain=float(self.get_parameter("model_yaw_slip_articulation_gain").value),
            yaw_slip_min_scale=float(self.get_parameter("model_yaw_slip_min_scale").value),
        )

        transient_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._odom_sub = self.create_subscription(
            Odometry,
            self._odom_topic,
            self._odom_callback,
            20,
            callback_group=self._callback_group,
        )
        self._local_plan_sub = self.create_subscription(
            Path,
            self._local_plan_topic,
            self._local_plan_callback,
            10,
            callback_group=self._callback_group,
        )
        self._cmd_pub = self.create_publisher(TwistStamped, self._cmd_vel_topic, 20)
        self._reference_path_pub = self.create_publisher(Path, "~/reference_path", transient_qos)
        self._target_pose_pub = self.create_publisher(PoseStamped, "~/target_pose", 20)

        self._action_server = ActionServer(
            self,
            FollowPath,
            self._action_name,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )

        self.get_logger().info(
            f"MTT path follower ready — action={self._action_name}  odom={self._odom_topic}  "
            f"cmd_vel={self._cmd_vel_topic}  local_plan={self._local_plan_topic}  "
            f"v_max={self._max_speed_ms:.2f} m/s  psi_max={math.degrees(self._psi_max_rad):.1f}°  "
            f"yaw_response_gain={self._motion_model_params.yaw_response_gain:.2f}  "
            f"path_error_limits=({self._max_lateral_error_m:.2f} m, "
            f"{math.degrees(self._max_heading_error_rad):.1f}°)"
        )

    def destroy_node(self):
        self._publish_zero_command()
        self._action_server.destroy()
        super().destroy_node()

    def _odom_callback(self, msg: Odometry) -> None:
        with self._state_lock:
            self._odom_msg = msg
            self._odom_received_time = time.monotonic()

    def _local_plan_callback(self, msg: Path) -> None:
        with self._local_plan_lock:
            self._local_plan_msg = msg
            self._local_plan_received_time = time.monotonic()

    def _get_fresh_local_plan(self) -> Optional[Path]:
        with self._local_plan_lock:
            local_plan = self._local_plan_msg
            received_time = self._local_plan_received_time
        if local_plan is None or received_time is None:
            return None
        if time.monotonic() - received_time > self._local_plan_timeout_s:
            return None
        return local_plan

    def _goal_callback(self, goal_request: FollowPath.Goal) -> GoalResponse:
        with self._goal_lock:
            if self._active_goal:
                self.get_logger().warning("Rejecting follow_path goal because another goal is active.")
                return GoalResponse.REJECT
            return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        self.get_logger().info("Cancel request received for follow_path goal.")
        return CancelResponse.ACCEPT

    def _get_fresh_odom(self) -> Optional[Odometry]:
        with self._state_lock:
            odom = self._odom_msg
            received_time = self._odom_received_time
        if odom is None:
            return None

        if received_time is None:
            return None

        if time.monotonic() - received_time > self._odom_timeout_s:
            return None
        return odom

    def _publish_zero_command(self) -> None:
        zero = TwistStamped()
        zero.header.stamp = self.get_clock().now().to_msg()
        zero.twist.linear.x = 0.0
        zero.twist.angular.z = 0.0
        self._cmd_pub.publish(zero)

    def _publish_command(self, linear_x: float, steering_normalized: float) -> None:
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.twist.linear.x = linear_x
        cmd.twist.angular.z = steering_normalized
        self._cmd_pub.publish(cmd)

    def _publish_target_pose(self, pose: PoseStamped) -> None:
        target = PoseStamped()
        target.header = pose.header
        target.pose = pose.pose
        self._target_pose_pub.publish(target)

    def _flatten_path_sequence(self, path_sequence) -> Path:
        path = Path()
        path.header = path_sequence.header
        for directional_path in path_sequence.paths:
            for pose in directional_path.poses:
                path.poses.append(pose)
        return path

    def _sanitize_paths(self, path_sequence) -> List:
        return [directional_path for directional_path in path_sequence.paths if directional_path.poses]

    def _find_start_index(self, poses: List[PoseStamped], robot_pose) -> int:
        best_index = 0
        best_distance = float("inf")
        for index, pose in enumerate(poses):
            dist = distance_xy(robot_pose, pose.pose)
            if dist < best_distance:
                best_distance = dist
                best_index = index
        return best_index

    def _advance_waypoint_index(self, poses: List[PoseStamped], current_index: int, robot_pose) -> int:
        index = current_index
        while index < len(poses) - 1 and distance_xy(robot_pose, poses[index].pose) < self._advance_distance_m:
            index += 1
        return index

    def _segment_complete(self, robot_pose, robot_heading_eff: float, target_pose: PoseStamped, forward: bool, final_segment: bool) -> bool:
        if distance_xy(robot_pose, target_pose.pose) > self._waypoint_tolerance_m:
            return False

        if not final_segment:
            return True

        target_heading = yaw_from_quaternion(target_pose.pose.orientation)
        target_heading_eff = target_heading if forward else wrap_to_pi(target_heading + math.pi)
        heading_error = wrap_to_pi(target_heading_eff - robot_heading_eff)
        return abs(heading_error) <= self._final_heading_tolerance_rad

    def _compute_control(
        self,
        robot_pose,
        robot_body_yaw: float,
        target_pose: PoseStamped,
        forward: bool,
        speed_ref_ms: float,
        previous_psi_cmd: float,
        dt: float,
    ) -> Tuple[float, float, float]:
        theta_eff = robot_body_yaw if forward else wrap_to_pi(robot_body_yaw + math.pi)
        target_body_yaw = yaw_from_quaternion(target_pose.pose.orientation)
        target_theta_eff = target_body_yaw if forward else wrap_to_pi(target_body_yaw + math.pi)

        dx = target_pose.pose.position.x - robot_pose.position.x
        dy = target_pose.pose.position.y - robot_pose.position.y
        e_y = -math.sin(theta_eff) * dx + math.cos(theta_eff) * dy
        e_theta = wrap_to_pi(target_theta_eff - theta_eff)

        kappa_eff_desired = self._k_y * e_y + self._k_theta * e_theta
        kappa_eff_desired = clamp(kappa_eff_desired, -self._kappa_max, self._kappa_max)
        kappa_actual = kappa_eff_desired if forward else -kappa_eff_desired

        slip = slip_scale(speed_ref_ms, previous_psi_cmd, self._motion_model_params)
        kappa_command = kappa_actual / max(slip, 1e-6)
        kappa_command = clamp(kappa_command, -self._kappa_max, self._kappa_max)

        psi_raw = articulation_from_curvature(kappa_command, self._motion_model_params)

        max_delta = self._psi_dot_max_rad_s * max(dt, 1e-3)
        psi_cmd = clamp(psi_raw, previous_psi_cmd - max_delta, previous_psi_cmd + max_delta)
        steering_normalized = normalized_steer_from_articulation(psi_cmd, self._motion_model_params)

        speed_mag = min(abs(speed_ref_ms), self._max_speed_ms)
        speed_mag = speed_mag / (1.0 + self._slowdown_alpha * abs(kappa_eff_desired))
        if speed_mag < self._min_speed_ms:
            speed_mag = self._min_speed_ms

        linear_x = speed_mag if forward else -speed_mag
        return linear_x, steering_normalized, psi_cmd

    def _tracking_errors(self, robot_pose, robot_body_yaw: float, target_pose: PoseStamped, forward: bool) -> Tuple[float, float, float]:
        theta_eff = robot_body_yaw if forward else wrap_to_pi(robot_body_yaw + math.pi)
        target_body_yaw = yaw_from_quaternion(target_pose.pose.orientation)
        target_theta_eff = target_body_yaw if forward else wrap_to_pi(target_body_yaw + math.pi)

        dx = target_pose.pose.position.x - robot_pose.position.x
        dy = target_pose.pose.position.y - robot_pose.position.y
        lateral_error = -math.sin(theta_eff) * dx + math.cos(theta_eff) * dy
        heading_error = wrap_to_pi(target_theta_eff - theta_eff)
        target_distance = math.hypot(dx, dy)
        return lateral_error, heading_error, target_distance

    def _tracking_error_exceeded(self, lateral_error: float, heading_error: float, target_distance: float) -> bool:
        return (
            abs(lateral_error) > self._max_lateral_error_m
            or abs(heading_error) > self._max_heading_error_rad
            or target_distance > self._max_target_distance_m
        )

    def _make_result(self, code: int) -> FollowPath.Result:
        result = FollowPath.Result()
        result.result_status.data = code
        return result

    async def _execute_callback(self, goal_handle) -> FollowPath.Result:
        with self._goal_lock:
            self._active_goal = True
            self._local_plan_received = False
            self._path_lost_since = None

        try:
            segments = self._sanitize_paths(goal_handle.request.path)
            if not segments:
                self.get_logger().warning("Received empty PathSequence; aborting follow_path goal.")
                goal_handle.abort()
                return self._make_result(self.RESULT_STATUS_ABORTED)

            speed_ref_ms = float(goal_handle.request.follower_options.velocity.data)
            if speed_ref_ms <= 0.0:
                speed_ref_ms = self._default_speed_ms

            self._reference_path_pub.publish(self._flatten_path_sequence(goal_handle.request.path))
            self.get_logger().info(
                f"Starting follow_path goal with {len(segments)} segment(s) at {speed_ref_ms:.2f} m/s"
            )

            loop_dt = 1.0 / max(self._control_rate_hz, 1.0)
            previous_psi_cmd = 0.0
            previous_time = time.monotonic()

            for segment_index, segment in enumerate(segments):
                forward = bool(segment.forward)
                final_segment = segment_index == len(segments) - 1

                odom = self._get_fresh_odom()
                if odom is None:
                    self.get_logger().error("No fresh odometry available to start trajectory replay.")
                    self._publish_zero_command()
                    goal_handle.abort()
                    return self._make_result(self.RESULT_STATUS_PATH_LOST)

                waypoint_index = self._find_start_index(segment.poses, odom.pose.pose)
                start_distance = distance_xy(odom.pose.pose, segment.poses[waypoint_index].pose)
                if start_distance > self._max_start_distance_m:
                    self.get_logger().error(
                        f"Robot is {start_distance:.2f} m from the closest route pose; "
                        f"limit is {self._max_start_distance_m:.2f} m. Refusing replay."
                    )
                    self._publish_zero_command()
                    goal_handle.abort()
                    return self._make_result(self.RESULT_STATUS_PATH_LOST)

                while rclpy.ok():
                    if goal_handle.is_cancel_requested:
                        self.get_logger().warning("Trajectory replay cancelled.")
                        self._publish_zero_command()
                        goal_handle.canceled()
                        return self._make_result(self.RESULT_STATUS_ABORTED)

                    odom = self._get_fresh_odom()
                    if odom is None:
                        self.get_logger().error("Odometry became stale during trajectory replay.")
                        self._publish_zero_command()
                        goal_handle.abort()
                        return self._make_result(self.RESULT_STATUS_PATH_LOST)

                    robot_pose = odom.pose.pose
                    robot_body_yaw = yaw_from_quaternion(robot_pose.orientation)
                    theta_eff = robot_body_yaw if forward else wrap_to_pi(robot_body_yaw + math.pi)

                    # Track progress on global path
                    waypoint_index = self._advance_waypoint_index(segment.poses, waypoint_index, robot_pose)
                    global_target_pose = segment.poses[waypoint_index]

                    # Prioritize dynamic local plan from WILN
                    local_plan = self._get_fresh_local_plan()
                    if local_plan is not None:
                        self._local_plan_received = True
                        local_idx = self._find_start_index(local_plan.poses, robot_pose)
                        local_idx = self._advance_waypoint_index(local_plan.poses, local_idx, robot_pose)
                        target_pose = local_plan.poses[local_idx]
                    elif self._local_plan_received:
                        # Safety stop if local plan goes stale after being received once
                        self.get_logger().warn("Local plan stale! Stopping for safety.", throttle_duration_sec=2.0)
                        self._publish_zero_command()
                        time.sleep(loop_dt)
                        continue
                    else:
                        # Fallback to static global path
                        target_pose = global_target_pose

                    self._publish_target_pose(target_pose)

                    lateral_error, heading_error, target_distance = self._tracking_errors(
                        robot_pose,
                        robot_body_yaw,
                        target_pose,
                        forward,
                    )
                    if self._tracking_error_exceeded(lateral_error, heading_error, target_distance):
                        now_monotonic = time.monotonic()
                        if self._path_lost_since is None:
                            self._path_lost_since = now_monotonic
                        self.get_logger().warn(
                            "Tracking error high: "
                            f"e_y={lateral_error:.2f} m, "
                            f"e_theta={math.degrees(heading_error):.1f}°, "
                            f"target_dist={target_distance:.2f} m",
                            throttle_duration_sec=1.0,
                        )
                        if now_monotonic - self._path_lost_since > self._tracking_error_grace_s:
                            self.get_logger().error("Path lost for too long; aborting replay.")
                            self._publish_zero_command()
                            goal_handle.abort()
                            return self._make_result(self.RESULT_STATUS_PATH_LOST)
                    else:
                        self._path_lost_since = None

                    if waypoint_index == len(segment.poses) - 1 and self._segment_complete(
                        robot_pose,
                        theta_eff,
                        global_target_pose,
                        forward,
                        final_segment,
                    ):
                        break

                    now = time.monotonic()
                    dt = max(now - previous_time, loop_dt)
                    previous_time = now

                    linear_x, steering_normalized, previous_psi_cmd = self._compute_control(
                        robot_pose,
                        robot_body_yaw,
                        target_pose,
                        forward,
                        speed_ref_ms,
                        previous_psi_cmd,
                        dt,
                    )
                    self._publish_command(linear_x, steering_normalized)

                    feedback = FollowPath.Feedback()
                    feedback.feedback_status.data = self.FEEDBACK_STATUS_MOVING
                    goal_handle.publish_feedback(feedback)

                    time.sleep(loop_dt)

            self._publish_zero_command()
            goal_handle.succeed()
            self.get_logger().info("Trajectory replay completed successfully.")
            return self._make_result(self.RESULT_STATUS_SUCCESS)
        except Exception as exc:  # pragma: no cover - runtime safety path
            self.get_logger().error(f"MTT path follower failed: {exc}")
            self._publish_zero_command()
            goal_handle.abort()
            return self._make_result(self.RESULT_STATUS_INTERNAL_ERROR)
        finally:
            with self._goal_lock:
                self._active_goal = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MttPathFollower()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
