#!/usr/bin/env python3
"""Field route manager for WILN taught routes."""

import math
import os
import threading
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from mtt_interfaces.srv import RouteCommand, RouteList, RouteStatus, RelocalizeRoute
from mtt_msgs.msg import MttHealthState
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker

try:
    from wiln.msg import WilnState as WilnStateMsg
except ImportError:  # pragma: no cover — absent on non-ROS test hosts
    WilnStateMsg = None

try:
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
except ImportError:  # pragma: no cover - only absent on non-ROS test hosts
    PointCloud2 = None
    point_cloud2 = None

try:
    from norlab_controllers_msgs.msg import PathSequence as PathSequenceMsg
except ImportError:  # pragma: no cover - absent on non-ROS test hosts
    PathSequenceMsg = None


@dataclass
class RoutePose:
    x: float
    y: float
    z: float
    yaw: float


@dataclass
class RouteData:
    name: str
    path: Path
    frame_id: str
    poses: List[RoutePose]
    valid: bool
    grade: str
    warnings: List[str]
    stats: dict


def yaw_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
    return math.atan2(
        2.0 * (qw * qz + qx * qy),
        qw * qw + qx * qx - qy * qy - qz * qz,
    )


def quaternion_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def wrap_to_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def read_ltr(path: Path) -> Tuple[str, List[RoutePose]]:
    frame_id = ""
    poses: List[RoutePose] = []
    in_trajectory = False

    with path.open("r", encoding="utf-8") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            if not line:
                continue
            if line.startswith("# vtk"):
                raise ValueError(f"{path} looks like a VTK map, not a WILN .ltr trajectory")
            if line.startswith("#############################"):
                in_trajectory = True
                continue
            if not in_trajectory:
                continue
            if line.startswith("frame_id"):
                frame_id = line.split(":", 1)[1].strip()
                continue
            if line == "changing direction":
                continue
            if line.startswith("direction :") or line.startswith("direction:"):
                continue

            fields = line.split(",")
            if len(fields) != 7:
                raise ValueError(f"invalid pose line in {path}: {line}")
            x, y, z, qx, qy, qz, qw = [float(value) for value in fields]
            poses.append(RoutePose(x=x, y=y, z=z, yaw=yaw_from_quaternion(qx, qy, qz, qw)))

    return frame_id, poses


def route_stats(poses: List[RoutePose]) -> dict:
    steps = [
        math.hypot(current.x - previous.x, current.y - previous.y)
        for previous, current in zip(poses, poses[1:])
    ]
    yaw_steps = [
        abs(wrap_to_pi(current.yaw - previous.yaw))
        for previous, current in zip(poses, poses[1:])
    ]
    z_values = [pose.z for pose in poses]
    path_length = sum(steps)
    return {
        "poses": len(poses),
        "path_length_m": path_length,
        "mean_step_m": path_length / len(steps) if steps else 0.0,
        "max_step_m": max(steps) if steps else 0.0,
        "max_yaw_step_rad": max(yaw_steps) if yaw_steps else 0.0,
        "z_span_m": (max(z_values) - min(z_values)) if z_values else 0.0,
    }


def grade_route(stats: dict, max_step_warn_m: float, max_yaw_step_warn_rad: float) -> Tuple[str, List[str]]:
    warnings: List[str] = []
    if stats["poses"] < 10:
        warnings.append("too_few_poses")
    if stats["path_length_m"] < 1.0:
        warnings.append("route_too_short")
    if stats["max_step_m"] > max_step_warn_m:
        warnings.append("large_xy_jump")
    if stats["max_yaw_step_rad"] > max_yaw_step_warn_rad:
        warnings.append("large_yaw_jump")
    if stats["z_span_m"] > 2.0:
        warnings.append("large_z_span")

    if not warnings:
        return "good", warnings

    # large_z_span is the only hard reject (terrain geometry unsafe to replay).
    # too_few_poses, route_too_short, and single jump warnings are navigable.
    if not any(w == "large_z_span" for w in warnings):
        return "usable_with_caution", warnings
    return "reject_for_replay", warnings


class MttRouteManager(Node):
    def __init__(self) -> None:
        super().__init__("mtt_route_manager")

        self.declare_parameter("routes_dir", "data/wiln_routes")
        self.declare_parameter("icp_odom_topic", "/mapping/icp_odom")
        self.declare_parameter("health_topic", "/mtt_health")
        self.declare_parameter("selected_mode_topic", "mtt_control/selected_mode")
        self.declare_parameter("selected_source_topic", "mtt_control/selected_source")
        self.declare_parameter("auto_enabled_topic", "mtt_control/auto_mode_enabled")
        self.declare_parameter("deadman_topic", "mtt_control/teleop_deadman")
        self.declare_parameter("obstacle_cloud_topic", "/wiln/obstacles")
        self.declare_parameter("wiln_command_topic", "/wiln/command")
        self.declare_parameter("route_state_topic", "/wiln/route/state")
        self.declare_parameter("mark_ready_service", "/mtt_repeat/mark_ready")
        self.declare_parameter("play_line_service", "/mtt_repeat/play_line")
        self.declare_parameter("cancel_service", "/mtt_repeat/cancel")
        self.declare_parameter("request_manual_service", "mtt_control/request_manual")
        self.declare_parameter("route_file_name", "route.ltr")
        self.declare_parameter("icp_timeout_s", 0.75)
        self.declare_parameter("health_timeout_s", 1.0)
        self.declare_parameter("max_start_distance_m", 3.0)
        self.declare_parameter("max_lateral_error_m", 1.25)
        self.declare_parameter("max_heading_error_rad", 1.2)
        self.declare_parameter("route_error_grace_s", 1.0)
        self.declare_parameter("mode_check_grace_s", 3.0)  # grace after replay start for mode latency
        self.declare_parameter("max_step_warn_m", 0.75)
        self.declare_parameter("max_yaw_step_warn_rad", 0.50)
        self.declare_parameter("min_obstacle_clearance_m", 0.75)
        self.declare_parameter("obstacle_corridor_half_width_m", 0.75)
        self.declare_parameter("obstacle_lookahead_m", 4.0)
        self.declare_parameter("require_obstacle_clearance", False)
        self.declare_parameter("cancel_on_obstacle_clearance", False)
        self.declare_parameter("front_obstacle_stop_topic", "/mtt_obstacle/stop_requested")
        self.declare_parameter("front_obstacle_clearance_topic", "/mtt_obstacle/front_clearance_m")
        self.declare_parameter("front_obstacle_status_topic", "/mtt_obstacle/hazard_status")
        self.declare_parameter("front_obstacle_timeout_s", 1.0)
        self.declare_parameter("monitor_rate_hz", 10.0)
        self.declare_parameter("auto_save_on_teach_stop", True)
        self.declare_parameter("teach_state_topic", "/wiln/teach/state")
        self.declare_parameter("auto_relocalize_on_load", True)
        self.declare_parameter("relocalize_service", "/mtt_map_relocalizer/relocalize")
        self.declare_parameter("relocalize_timeout_s", 60.0)  # ICP can be slow
        self.declare_parameter("wiln_trajectory_topic", "/wiln/trajectory")
        self.declare_parameter("icp_convergence_grace_s", 8.0)  # after load: relax distance check while ICP converges
        self.declare_parameter("icp_settle_timeout_s", 2.0)   # max wait for ICP to re-settle before play (Phase 1b)
        self.declare_parameter("map_load_timeout_s", 12.0)
        self.declare_parameter("play_service_timeout_s", 6.0)
        self.declare_parameter("debug", False)                  # enable verbose [MONITOR] logs

        routes_dir = str(self.get_parameter("routes_dir").value)
        self._routes_dir = Path(routes_dir)
        if not self._routes_dir.is_absolute():
            workspace = Path(os.environ.get("WORKSPACE", str(Path.cwd())))
            self._routes_dir = workspace / self._routes_dir

        self._route_file_name = str(self.get_parameter("route_file_name").value)
        self._icp_timeout_s = float(self.get_parameter("icp_timeout_s").value)
        self._health_timeout_s = float(self.get_parameter("health_timeout_s").value)
        self._max_start_distance_m = float(self.get_parameter("max_start_distance_m").value)
        self._max_lateral_error_m = float(self.get_parameter("max_lateral_error_m").value)
        self._max_heading_error_rad = float(self.get_parameter("max_heading_error_rad").value)
        self._route_error_grace_s = float(self.get_parameter("route_error_grace_s").value)
        self._mode_check_grace_s = float(self.get_parameter("mode_check_grace_s").value)
        self._max_step_warn_m = float(self.get_parameter("max_step_warn_m").value)
        self._max_yaw_step_warn_rad = float(self.get_parameter("max_yaw_step_warn_rad").value)
        self._min_obstacle_clearance_m = float(self.get_parameter("min_obstacle_clearance_m").value)
        self._obstacle_corridor_half_width_m = float(
            self.get_parameter("obstacle_corridor_half_width_m").value
        )
        self._obstacle_lookahead_m = float(self.get_parameter("obstacle_lookahead_m").value)
        self._require_obstacle_clearance = bool(self.get_parameter("require_obstacle_clearance").value)
        self._cancel_on_obstacle_clearance = bool(self.get_parameter("cancel_on_obstacle_clearance").value)
        self._front_obstacle_timeout_s = float(self.get_parameter("front_obstacle_timeout_s").value)
        monitor_rate_hz = float(self.get_parameter("monitor_rate_hz").value)
        self._auto_save_on_teach_stop = bool(self.get_parameter("auto_save_on_teach_stop").value)
        self._teach_state_topic = str(self.get_parameter("teach_state_topic").value)
        self._auto_relocalize_on_load = bool(self.get_parameter("auto_relocalize_on_load").value)
        self._relocalize_timeout_s = float(self.get_parameter("relocalize_timeout_s").value)
        self._wiln_trajectory_topic = str(self.get_parameter("wiln_trajectory_topic").value)
        self._icp_convergence_grace_s = float(self.get_parameter("icp_convergence_grace_s").value)
        self._icp_settle_timeout_s = float(self.get_parameter("icp_settle_timeout_s").value)
        self._map_load_timeout_s = float(self.get_parameter("map_load_timeout_s").value)
        self._play_service_timeout_s = float(
            self.get_parameter("play_service_timeout_s").value
        )
        self._debug = bool(self.get_parameter("debug").value)

        self._group = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._active_route: Optional[RouteData] = None
        self._route_loaded = False
        self._replaying = False
        self._teach_was_recording = False  # for auto-save trigger
        self._bad_route_error_since: Optional[float] = None
        self._bad_mode_since: Optional[float] = None
        self._replay_started_at: Optional[float] = None  # wall time when replay started
        self._last_load_time: Optional[float] = None  # set on load; enables ICP convergence grace period
        self._pending_trajectory: Optional[Tuple[List[RoutePose], str]] = None  # (poses, frame_id) from /wiln/trajectory
        self._route_load_event = threading.Event()
        self._route_load_in_progress = False
        self._route_load_detail = ""
        self._icp: Optional[Odometry] = None
        self._icp_received_time: Optional[float] = None
        self._health: Optional[MttHealthState] = None
        self._health_received_time: Optional[float] = None
        self._selected_mode = "stop"
        self._selected_source = "unknown"
        self._auto_enabled = False
        self._deadman = False
        self._deadman_received_time: Optional[float] = None
        self._obstacle_points: List[Tuple[float, float, float]] = []
        self._obstacle_received_time: Optional[float] = None
        self._front_obstacle_stop = False
        self._front_obstacle_clearance = float("nan")
        self._front_obstacle_status = "unavailable"
        self._front_obstacle_received_time: Optional[float] = None
        self._last_status = "no route loaded"
        self._last_allowed = False
        self._last_refusal = "no route loaded"
        self._last_distance = float("nan")
        self._last_heading = float("nan")
        self._last_clearance = float("nan")
        self._last_closest: Optional[RoutePose] = None

        self._wiln_command_pub = self.create_publisher(
            String, str(self.get_parameter("wiln_command_topic").value), 10
        )
        self._mark_ready_client = self.create_client(
            Trigger, str(self.get_parameter("mark_ready_service").value), callback_group=self._group
        )
        self._relocalize_client = self.create_client(
            RelocalizeRoute,
            str(self.get_parameter("relocalize_service").value),
            callback_group=self._group,
        )
        self._play_line_client = self.create_client(
            Trigger, str(self.get_parameter("play_line_service").value), callback_group=self._group
        )
        self._cancel_client = self.create_client(
            Trigger, str(self.get_parameter("cancel_service").value), callback_group=self._group
        )
        self._request_manual_client = self.create_client(
            Trigger, str(self.get_parameter("request_manual_service").value), callback_group=self._group
        )

        self.create_subscription(
            Odometry, str(self.get_parameter("icp_odom_topic").value), self._on_icp, 20
        )
        self.create_subscription(
            MttHealthState, str(self.get_parameter("health_topic").value), self._on_health, 20
        )
        self.create_subscription(
            String, str(self.get_parameter("selected_mode_topic").value), self._on_mode, 20
        )
        self.create_subscription(
            String, str(self.get_parameter("selected_source_topic").value), self._on_source, 20
        )
        self.create_subscription(
            Bool, str(self.get_parameter("auto_enabled_topic").value), self._on_auto_enabled, 20
        )
        self.create_subscription(Bool, str(self.get_parameter("deadman_topic").value), self._on_deadman, 20)
        self.create_subscription(
            Bool,
            str(self.get_parameter("front_obstacle_stop_topic").value),
            self._on_front_obstacle_stop,
            20,
        )
        self.create_subscription(
            Float32,
            str(self.get_parameter("front_obstacle_clearance_topic").value),
            self._on_front_obstacle_clearance,
            20,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("front_obstacle_status_topic").value),
            self._on_front_obstacle_status,
            20,
        )
        if PointCloud2 is not None:
            obstacle_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=5,
                reliability=ReliabilityPolicy.BEST_EFFORT,
            )
            self.create_subscription(
                PointCloud2,
                str(self.get_parameter("obstacle_cloud_topic").value),
                self._on_obstacles,
                obstacle_qos,
            )

        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._path_pub = self.create_publisher(NavPath, "/mtt_route/active_path", latched_qos)
        self._closest_pub = self.create_publisher(PoseStamped, "/mtt_route/closest_pose", latched_qos)
        self._start_pub = self.create_publisher(PoseStamped, "/mtt_route/start_pose", latched_qos)
        self._marker_pub = self.create_publisher(Marker, "/mtt_route/error_marker", latched_qos)
        self._status_pub = self.create_publisher(String, "/mtt_route/status_text", latched_qos)
        self._safety_status_pub = self.create_publisher(String, "/mtt_route/safety_status", latched_qos)
        self._allowed_pub = self.create_publisher(Bool, "/mtt_route/autonomy_allowed", latched_qos)
        self._clearance_pub = self.create_publisher(Float32, "/mtt_route/obstacle_clearance_min", latched_qos)

        self.create_service(RouteList, "/mtt_route/list", self._handle_list, callback_group=self._group)
        self.create_service(RouteCommand, "/mtt_route/load", self._handle_load, callback_group=self._group)
        self.create_service(RouteCommand, "/mtt_route/validate", self._handle_validate, callback_group=self._group)
        self.create_service(RouteCommand, "/mtt_route/preview", self._handle_preview, callback_group=self._group)
        self.create_service(RouteCommand, "/mtt_route/replay", self._handle_replay, callback_group=self._group)
        self.create_service(Trigger, "/mtt_route/stop", self._handle_stop, callback_group=self._group)
        self.create_service(RouteStatus, "/mtt_route/status", self._handle_status, callback_group=self._group)

        # Auto-save: subscribe to teach state to detect teach completion.
        # Uses transient_local so we get the current state even on late subscribe.
        if self._auto_save_on_teach_stop and WilnStateMsg is not None:
            teach_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.create_subscription(
                WilnStateMsg,
                self._teach_state_topic,
                self._on_teach_state,
                teach_qos,
            )
            self.get_logger().info(
                f"Auto-save enabled: listening on {self._teach_state_topic}"
            )

        if WilnStateMsg is not None:
            route_state_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.create_subscription(
                WilnStateMsg,
                str(self.get_parameter("route_state_topic").value),
                self._on_route_state,
                route_state_qos,
            )

        # Track /wiln/trajectory to keep _active_route.poses in sync with the
        # trajectory actually used by WILN (which may be corrected by the relocalizer
        # after a load, or loaded fresh from .ltr by wiln_route_node).
        # Using transient_local so we get the last published trajectory on subscribe.
        if PathSequenceMsg is not None:
            traj_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.create_subscription(
                PathSequenceMsg,
                self._wiln_trajectory_topic,
                self._on_trajectory,
                traj_qos,
            )
            self.get_logger().info(
                f"Tracking trajectory from {self._wiln_trajectory_topic}"
            )

        self.create_timer(1.0 / max(monitor_rate_hz, 1.0), self._monitor)
        self.get_logger().info(f"MTT route manager ready; routes_dir={self._routes_dir}")

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _stamp_to_seconds(self, stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _on_icp(self, msg: Odometry) -> None:
        with self._lock:
            self._icp = msg
            self._icp_received_time = self._now_seconds()

    def _on_health(self, msg: MttHealthState) -> None:
        with self._lock:
            self._health = msg
            self._health_received_time = self._now_seconds()

    def _on_mode(self, msg: String) -> None:
        with self._lock:
            self._selected_mode = str(msg.data).strip().lower()

    def _on_source(self, msg: String) -> None:
        with self._lock:
            self._selected_source = str(msg.data).strip()

    def _on_auto_enabled(self, msg: Bool) -> None:
        with self._lock:
            self._auto_enabled = bool(msg.data)

    def _on_deadman(self, msg: Bool) -> None:
        with self._lock:
            self._deadman = bool(msg.data)
            self._deadman_received_time = self._now_seconds()

    def _on_obstacles(self, msg) -> None:
        if point_cloud2 is None:
            return
        points: List[Tuple[float, float, float]] = []
        for point in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            if len(points) >= 5000:
                break
            points.append((float(point[0]), float(point[1]), float(point[2])))
        with self._lock:
            self._obstacle_points = points
            self._obstacle_received_time = self._now_seconds()

    def _on_front_obstacle_stop(self, msg: Bool) -> None:
        with self._lock:
            self._front_obstacle_stop = bool(msg.data)
            self._front_obstacle_received_time = self._now_seconds()

    def _on_front_obstacle_clearance(self, msg: Float32) -> None:
        with self._lock:
            self._front_obstacle_clearance = float(msg.data) if msg.data >= 0.0 else float("inf")
            self._front_obstacle_received_time = self._now_seconds()

    def _on_front_obstacle_status(self, msg: String) -> None:
        with self._lock:
            self._front_obstacle_status = str(msg.data)
            self._front_obstacle_received_time = self._now_seconds()

    def _on_route_state(self, msg) -> None:
        """Release a pending route load only after the mapper load callback finished."""
        if WilnStateMsg is None or msg.state != WilnStateMsg.IDLE:
            return
        detail = str(msg.detail).strip()
        if detail != "loaded" and not detail.startswith("load failed"):
            return
        with self._lock:
            if not self._route_load_in_progress:
                return
            self._route_load_detail = detail
        self._route_load_event.set()

    def _on_trajectory(self, msg) -> None:
        """Sync _active_route.poses from /wiln/trajectory.

        wiln_route_node publishes the loaded trajectory here; the mtt_map_relocalizer
        republishes a corrected version after SE(3) alignment.  Keeping poses in sync
        means the pre-replay distance check always compares against the same trajectory
        WILN will actually follow — no frame mismatch between the two.

        Sequencing: wiln_route_node now publishes the trajectory only after LoadMap
        returns. This callback can still run before the route-manager service thread
        marks the route active, so keep the pending hand-off.
        """
        poses: List[RoutePose] = []
        for path in msg.paths:
            for stamped_pose in path.poses:
                p = stamped_pose.pose.position
                o = stamped_pose.pose.orientation
                yaw = yaw_from_quaternion(o.x, o.y, o.z, o.w)
                poses.append(RoutePose(x=p.x, y=p.y, z=p.z, yaw=yaw))
        if not poses:
            return
        frame = msg.header.frame_id or ""
        updated = False
        old_count = 0
        with self._lock:
            # Always store as pending so _handle_load can apply it after the route is armed.
            self._pending_trajectory = (poses, frame)
            if self._route_loaded and self._active_route is not None:
                # Route already active: apply immediately (relocalizer correction case).
                old_count = len(self._active_route.poses)
                self._active_route.poses = poses
                if frame:
                    self._active_route.frame_id = frame
                updated = True
        if updated:
            self.get_logger().info(
                f"Trajectory synced from {self._wiln_trajectory_topic}: "
                f"{len(poses)} poses (was {old_count}); "
                f"start=({poses[0].x:.2f},{poses[0].y:.2f}) frame={frame}"
            )

    def _route_path(self, route_name: str) -> Path:
        safe_name = route_name.strip().strip("/")
        return self._routes_dir / safe_name / self._route_file_name

    def _load_route_data(self, route_name: str) -> RouteData:
        path = self._route_path(route_name)
        frame_id, poses = read_ltr(path)
        stats = route_stats(poses)
        grade, warnings = grade_route(stats, self._max_step_warn_m, self._max_yaw_step_warn_rad)
        return RouteData(
            name=route_name,
            path=path,
            frame_id=frame_id or "map",
            poses=poses,
            valid=grade != "reject_for_replay",
            grade=grade,
            warnings=warnings,
            stats=stats,
        )

    def _route_names(self) -> List[str]:
        if not self._routes_dir.exists():
            return []
        names = []
        for child in sorted(self._routes_dir.iterdir()):
            if child.name == "latest" and child.is_symlink():
                continue
            if child.is_dir() and (child / self._route_file_name).exists():
                names.append(child.name)
        return names

    def _icp_fresh(self) -> Tuple[bool, Optional[Odometry]]:
        with self._lock:
            msg = self._icp
            received_time = self._icp_received_time
        if msg is None or received_time is None:
            return False, msg
        now_s = self._now_seconds()
        header_time = self._stamp_to_seconds(msg.header.stamp)
        if header_time > 0.0:
            return (now_s - header_time) <= self._icp_timeout_s, msg
        return (now_s - received_time) <= self._icp_timeout_s, msg

    def _health_fresh(self) -> Tuple[bool, Optional[MttHealthState]]:
        with self._lock:
            msg = self._health
            received_time = self._health_received_time
        if msg is None or received_time is None:
            return False, msg
        return (self._now_seconds() - received_time) <= self._health_timeout_s, msg

    def _deadman_override_active(self) -> bool:
        with self._lock:
            deadman = self._deadman
            received_time = self._deadman_received_time
        return bool(deadman and received_time is not None and self._now_seconds() - received_time < 0.5)

    def _robot_pose(self, odom: Odometry) -> RoutePose:
        pose = odom.pose.pose
        yaw = yaw_from_quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        return RoutePose(x=pose.position.x, y=pose.position.y, z=pose.position.z, yaw=yaw)

    def _closest_pose(self, route: RouteData, robot: RoutePose) -> Tuple[RoutePose, float, float]:
        closest = min(route.poses, key=lambda pose: math.hypot(robot.x - pose.x, robot.y - pose.y))
        distance = math.hypot(robot.x - closest.x, robot.y - closest.y)
        heading = abs(wrap_to_pi(robot.yaw - closest.yaw))
        return closest, distance, heading

    def _obstacle_clearance(self, route: RouteData, robot: RoutePose) -> float:
        with self._lock:
            points = list(self._obstacle_points)
            received_time = self._obstacle_received_time
        if not points or received_time is None or self._now_seconds() - received_time > 1.0:
            return float("nan")

        min_clearance = float("inf")
        for pose in route.poses:
            ahead = math.hypot(pose.x - robot.x, pose.y - robot.y)
            if ahead > self._obstacle_lookahead_m:
                continue
            for x, y, _z in points:
                clearance = math.hypot(x - pose.x, y - pose.y)
                if clearance <= self._obstacle_corridor_half_width_m:
                    min_clearance = min(min_clearance, clearance)
        return min_clearance if math.isfinite(min_clearance) else float("inf")

    def _evaluate(self, route: Optional[RouteData]) -> Tuple[bool, str, float, float, float, Optional[RoutePose]]:
        with self._lock:
            mode = self._selected_mode
            source = self._selected_source
            auto_enabled = self._auto_enabled
            replaying = self._replaying
        control_hint = f"mode={mode} auto_enabled={auto_enabled} source={source}"
        if route is None:
            return False, f"no route loaded; run route_load or teach/save first; {control_hint}", float("nan"), float("nan"), float("nan"), None
        if not route.valid:
            return False, f"route rejected: {route.grade}; validate/preview the route; {control_hint}", float("nan"), float("nan"), float("nan"), None

        icp_ok, icp = self._icp_fresh()
        if not icp_ok or icp is None:
            return False, f"ICP odom stale or absent; mapping/localization not fresh; {control_hint}", float("nan"), float("nan"), float("nan"), None
        health_ok, health = self._health_fresh()
        if not health_ok or health is None:
            return False, f"mtt_health stale or absent; driver health monitor missing/stale; {control_hint}", float("nan"), float("nan"), float("nan"), None
        if not health.security_unlocked:
            return False, f"security_unlocked=false; unlock robot safety; {control_hint}", float("nan"), float("nan"), float("nan"), None
        if health.emergency_stop_active:
            return False, f"emergency stop active; release e-stop; {control_hint}", float("nan"), float("nan"), float("nan"), None
        if health.fallback_active and health.fallback_low_confidence:
            return False, f"health fallback low confidence: {health.fallback_reason}; {control_hint}", float("nan"), float("nan"), float("nan"), None
        if self._deadman_override_active():
            return False, f"manual deadman override active; release RB/deadman/manual stick; {control_hint}", float("nan"), float("nan"), float("nan"), None

        robot = self._robot_pose(icp)
        closest, distance, heading = self._closest_pose(route, robot)
        clearance = self._obstacle_clearance(route, robot)

        if replaying:
            # During replay: enforce lateral tracking error (strict).
            # The monitor calls _evaluate() at 10 Hz; route_error_grace_s provides
            # a 1s buffer before cancel so transient tracking spikes don't abort.
            if distance > self._max_lateral_error_m:
                return False, (
                    f"lateral drift: {distance:.2f} m > {self._max_lateral_error_m:.2f} m; {control_hint}"
                ), distance, heading, clearance, closest
        else:
            # Pre-replay: robot must be on or near the route anywhere (start, middle, end).
            # WILN replay auto-selects direction based on which endpoint is closer.
            start = route.poses[0]
            end = route.poses[-1]
            start_d = math.hypot(robot.x - start.x, robot.y - start.y)
            end_d = math.hypot(robot.x - end.x, robot.y - end.y)
            if distance > self._max_start_distance_m:
                return False, (
                    f"not on route after map/ICP settle: nearest={distance:.2f} m "
                    f"(to_start={start_d:.2f} m, to_end={end_d:.2f} m) "
                    f"> {self._max_start_distance_m:.2f} m; "
                    f"robot=({robot.x:.2f},{robot.y:.2f}) route_start=({start.x:.2f},{start.y:.2f}); "
                    f"position robot on route or check ICP localization; "
                    f"{control_hint}"
                ), distance, heading, clearance, closest

        if heading > self._max_heading_error_rad:
            return False, f"heading error: {heading:.2f} rad > {self._max_heading_error_rad:.2f} rad; align robot with route; {control_hint}", distance, heading, clearance, closest
        if self._require_obstacle_clearance and not math.isfinite(clearance):
            return False, f"obstacle clearance unavailable; /wiln/obstacles missing/stale; {control_hint}", distance, heading, clearance, closest
        if (
            self._cancel_on_obstacle_clearance
            and math.isfinite(clearance)
            and clearance < self._min_obstacle_clearance_m
        ):
            return False, f"obstacle too close: {clearance:.2f} m < {self._min_obstacle_clearance_m:.2f} m; {control_hint}", distance, heading, clearance, closest
        return True, f"ready; {control_hint}", distance, heading, clearance, closest

    def _call_trigger(self, client, timeout_s: float = 2.0) -> Tuple[bool, str]:
        if not client.wait_for_service(timeout_sec=0.0):
            return False, "service unavailable"
        future = client.call_async(Trigger.Request())
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        if not event.wait(timeout_s):
            return False, "service timeout"
        try:
            result = future.result()
        except Exception as exc:  # pragma: no cover - runtime safety
            return False, str(exc)
        if result is None:
            return False, "empty response"
        return bool(result.success), result.message or "ok"

    def _publish_wiln_command(self, command: str) -> Tuple[bool, str]:
        if self._wiln_command_pub.get_subscription_count() <= 0:
            return False, "no WILN command subscribers"
        msg = String()
        msg.data = command
        self._wiln_command_pub.publish(msg)
        self.get_logger().info(f"WILN command published: {command}")
        return True, f"published {command}"

    def _call_load(self, route: RouteData) -> Tuple[bool, str]:
        self._route_load_event.clear()
        with self._lock:
            self._route_load_in_progress = True
            self._route_load_detail = ""
            # A latched trajectory from the previous route must never satisfy
            # the hand-off for this load.
            self._pending_trajectory = None

        ok, detail = self._publish_wiln_command(f"load:{route.path}")
        if not ok:
            with self._lock:
                self._route_load_in_progress = False
            return False, detail

        if not self._route_load_event.wait(self._map_load_timeout_s):
            with self._lock:
                self._route_load_in_progress = False
            return False, (
                f"route/map load did not complete within {self._map_load_timeout_s:.1f}s; "
                "replay was not armed"
            )

        with self._lock:
            load_detail = self._route_load_detail
            self._route_load_in_progress = False
            self._last_load_time = self._now_seconds()
        if load_detail != "loaded":
            return False, load_detail or "route/map load failed"

        ok, detail = self._call_trigger(self._mark_ready_client)
        if not ok:
            return False, f"route/map loaded, but mark_ready failed: {detail}"
        return True, "route and map loaded; trajectory armed"

    def _call_relocalize(self, route: RouteData) -> None:
        """
        Call the mtt_map_relocalizer service to align the old teach map onto the
        current live map and republish the trajectory in the live map frame.

        This is called in a separate thread after a successful load so it does not
        block the load response.  Failures are logged but do not fail the load —
        the replay still works with the original (unaligned) trajectory.
        """
        if not self._auto_relocalize_on_load:
            return

        map_file = str(route.path) + ".vtk"
        if not Path(map_file).exists():
            self.get_logger().warn(
                f"Relocalize: no map file {map_file} — skipping. "
                "The route was taught without a saved map or the .vtk is missing."
            )
            return

        if not self._relocalize_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(
                "Relocalize: mtt_map_relocalizer service not available — "
                "is the relocalizer node running? Skipping relocalization."
            )
            return

        # Give wiln_route_node time to process the load command and publish the
        # trajectory on /wiln/trajectory before the relocalizer reads it.
        import time
        time.sleep(0.5)

        req = RelocalizeRoute.Request()
        req.map_file   = map_file
        req.route_name = route.name

        future = self._relocalize_client.call_async(req)
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        if not event.wait(self._relocalize_timeout_s):
            self.get_logger().error(
                f"Relocalize: service timed out after {self._relocalize_timeout_s:.0f}s"
            )
            return

        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f"Relocalize: service call failed: {exc}")
            return

        if result is None:
            self.get_logger().error("Relocalize: empty response")
            return

        if result.success:
            self.get_logger().info(
                f"Relocalize OK: {result.message}"
            )
        else:
            self.get_logger().warn(
                f"Relocalize FAILED (overlap={result.overlap_ratio:.3f}): {result.message}. "
                "Replay will use the original trajectory (pre-alignment)."
            )

    def _pose_msg(self, route: RouteData, pose: RoutePose) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = route.frame_id
        msg.pose.position.x = pose.x
        msg.pose.position.y = pose.y
        msg.pose.position.z = pose.z
        qx, qy, qz, qw = quaternion_from_yaw(pose.yaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        return msg

    def _publish_route(self, route: RouteData) -> None:
        path = NavPath()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = route.frame_id
        path.poses = [self._pose_msg(route, pose) for pose in route.poses]
        self._path_pub.publish(path)
        if route.poses:
            self._start_pub.publish(self._pose_msg(route, route.poses[0]))

    def _publish_status(self) -> None:
        with self._lock:
            route = self._active_route
            status = self._last_status
            allowed = self._last_allowed
            closest = self._last_closest
            clearance = self._last_clearance
            distance = self._last_distance
        status_msg = String()
        status_msg.data = status
        self._status_pub.publish(status_msg)
        safety_msg = String()
        safety_msg.data = status
        self._safety_status_pub.publish(safety_msg)
        allowed_msg = Bool()
        allowed_msg.data = allowed
        self._allowed_pub.publish(allowed_msg)
        clearance_msg = Float32()
        clearance_msg.data = float(clearance) if math.isfinite(clearance) else -1.0
        self._clearance_pub.publish(clearance_msg)

        if route is None:
            return
        self._publish_route(route)
        if closest is not None:
            self._closest_pub.publish(self._pose_msg(route, closest))

        icp_ok, icp = self._icp_fresh()
        if icp_ok and icp is not None and closest is not None:
            marker = Marker()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = route.frame_id
            marker.ns = "mtt_route"
            marker.id = 1
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.06
            marker.color.a = 1.0
            if allowed:
                marker.color.g = 1.0
            else:
                marker.color.r = 1.0
            robot = icp.pose.pose.position
            marker.points = [Point(x=robot.x, y=robot.y, z=robot.z), Point(x=closest.x, y=closest.y, z=closest.z)]
            self._marker_pub.publish(marker)

    def _set_status(
        self,
        allowed: bool,
        refusal: str,
        distance: float,
        heading: float,
        clearance: float,
        closest: Optional[RoutePose],
    ) -> None:
        with self._lock:
            route_name = self._active_route.name if self._active_route else "none"
            mode = self._selected_mode
            source = self._selected_source
            auto_enabled = self._auto_enabled
            deadman = self._deadman
            front_stop = self._front_obstacle_stop
            front_clearance = self._front_obstacle_clearance
            front_status = self._front_obstacle_status
            front_time = self._front_obstacle_received_time
            self._last_allowed = allowed
            self._last_refusal = "" if allowed else refusal
            self._last_distance = distance
            self._last_heading = heading
            self._last_clearance = clearance
            self._last_closest = closest
            state = "allowed" if allowed else f"blocked: {refusal}"
            clearance_text = f"{clearance:.2f} m" if math.isfinite(clearance) else "unknown"
            if front_time is None or self._now_seconds() - front_time > self._front_obstacle_timeout_s:
                front_text = "front_obstacle=stale/missing"
            else:
                front_clearance_text = (
                    f"{front_clearance:.2f} m"
                    if math.isfinite(front_clearance)
                    else "clear"
                )
                front_text = (
                    f"front_obstacle_stop={front_stop} front_clearance={front_clearance_text} "
                    f"front_status='{front_status}'"
                )
            self._last_status = (
                f"route={route_name} mode={mode} auto_enabled={auto_enabled} source={source} deadman={deadman} auto={state}; "
                f"distance={distance:.2f} m heading={heading:.2f} rad clearance={clearance_text}; "
                f"{front_text}; "
                "buttons: A auto, B stop, Y manual, RB deadman"
            )

    def _handle_list(self, _, response: RouteList.Response) -> RouteList.Response:
        response.route_names = self._route_names()
        with self._lock:
            response.active_route = self._active_route.name if self._active_route else ""
        return response

    def _handle_validate(self, request: RouteCommand.Request, response: RouteCommand.Response) -> RouteCommand.Response:
        try:
            route = self._load_route_data(request.route_name)
        except Exception as exc:
            response.success = False
            response.message = f"invalid route {request.route_name}: {exc}"
            return response
        response.success = route.valid
        warnings = ", ".join(route.warnings) if route.warnings else "none"
        response.message = (
            f"{route.name}: grade={route.grade}, poses={route.stats['poses']}, "
            f"length={route.stats['path_length_m']:.2f} m, warnings={warnings}"
        )
        return response

    def _handle_preview(self, request: RouteCommand.Request, response: RouteCommand.Response) -> RouteCommand.Response:
        try:
            route = self._load_route_data(request.route_name)
        except Exception as exc:
            response.success = False
            response.message = f"preview failed: {exc}"
            return response
        with self._lock:
            self._active_route = route
        self._publish_route(route)
        response.success = route.valid
        response.message = f"published preview for {route.name}; grade={route.grade}"
        return response

    def _handle_load(self, request: RouteCommand.Request, response: RouteCommand.Response) -> RouteCommand.Response:
        try:
            route = self._load_route_data(request.route_name)
        except Exception as exc:
            response.success = False
            response.message = f"load refused: {exc}"
            return response
        if not route.valid:
            response.success = False
            response.message = f"load refused: route grade={route.grade}, warnings={route.warnings}"
            return response
        # Clear any pending trajectory from a previous load so we don't accidentally
        # apply stale data if the new route's trajectory arrives before mark_ready.
        with self._lock:
            self._pending_trajectory = None

        ok, detail = self._call_load(route)
        if ok:
            with self._lock:
                self._active_route = route
                self._route_loaded = True
                self._replaying = False
                # Apply the trajectory that wiln_route_node published during load
                # (it arrives before mark_ready, so _on_trajectory couldn't apply it yet).
                pending = self._pending_trajectory
                if pending is not None:
                    pending_poses, pending_frame = pending
                    if pending_poses:
                        self._active_route.poses = pending_poses
                        if pending_frame:
                            self._active_route.frame_id = pending_frame
                    self._pending_trajectory = None
            self._publish_route(route)
            # Kick off map-to-map SE(3) relocalization in a background thread so
            # the load response is returned immediately.  The corrected trajectory
            # is republished by the relocalizer node on /wiln/trajectory when done.
            if self._auto_relocalize_on_load:
                reloc_thread = threading.Thread(
                    target=self._call_relocalize, args=(route,), daemon=True
                )
                reloc_thread.start()
        response.success = ok
        response.message = detail
        return response

    def _handle_replay(self, request: RouteCommand.Request, response: RouteCommand.Response) -> RouteCommand.Response:
        route_name = request.route_name.strip()
        with self._lock:
            route = self._active_route if self._active_route and self._active_route.name == route_name else None
            loaded = self._route_loaded and route is not None
        if route is None:
            try:
                route = self._load_route_data(route_name)
            except Exception as exc:
                response.success = False
                response.message = f"replay refused: {exc}"
                return response

        load_completed_at: Optional[float] = None
        if not loaded:
            ok, detail = self._call_load(route)
            if not ok:
                response.success = False
                response.message = detail
                return response

            # Apply exactly the trajectory published after the mapper load.
            with self._lock:
                pending = self._pending_trajectory
                if pending is not None:
                    pending_poses, pending_frame = pending
                    if pending_poses:
                        route.poses = pending_poses
                    if pending_frame:
                        route.frame_id = pending_frame
                    self._pending_trajectory = None
                self._active_route = route
                self._route_loaded = True
                self._replaying = False
                load_completed_at = self._last_load_time

        # LoadMap changes the map frame. A merely "fresh" sample may still be
        # the last measurement from before that change, so require an accepted
        # ICP callback received after the map service completed.
        import time as _time
        settle_start = self._now_seconds()
        self.get_logger().info(
            f"[REPLAY] Waiting up to {self._icp_settle_timeout_s:.1f}s for post-load ICP settle..."
        )
        while True:
            icp_ok, _ = self._icp_fresh()
            with self._lock:
                icp_received_time = self._icp_received_time
            post_load = (
                load_completed_at is None
                or (icp_received_time is not None and icp_received_time > load_completed_at)
            )
            if icp_ok and post_load:
                break
            if self._now_seconds() - settle_start >= self._icp_settle_timeout_s:
                response.success = False
                response.message = (
                    f"no fresh post-load ICP after {self._icp_settle_timeout_s:.1f}s; "
                    "replay was not started"
                )
                return response
            _time.sleep(0.1)

        elapsed = self._now_seconds() - settle_start
        self.get_logger().info(f"[REPLAY] Post-load ICP settled after {elapsed:.2f}s.")

        # Evaluate only after map load + ICP settle. The old ordering evaluated
        # in the previous map frame and then optimistically allowed a bad start.
        allowed, reason, distance, heading, clearance, closest = self._evaluate(route)
        self._set_status(allowed, reason, distance, heading, clearance, closest)
        if not allowed:
            response.success = False
            response.message = reason
            return response

        # play_line now waits for both replay-node and follower acceptance. Its
        # service timeout must exceed the supervisor's distributed ack timeout;
        # the previous fixed 2 s timeout could report failure while arming later.
        ok, detail = self._call_trigger(
            self._play_line_client, timeout_s=self._play_service_timeout_s
        )
        if ok:
            with self._lock:
                self._active_route = route
                self._route_loaded = True
                self._replaying = True
                self._bad_route_error_since = None
                self._bad_mode_since = None
                self._replay_started_at = self._now_seconds()
            response.success = True
            response.message = "replay started"
        else:
            response.success = False
            response.message = detail
        return response

    def _handle_stop(self, _, response: Trigger.Response) -> Trigger.Response:
        cancel_ok, cancel_detail = self._call_trigger(self._cancel_client)
        manual_ok, manual_detail = self._call_trigger(self._request_manual_client)
        with self._lock:
            self._replaying = False
        response.success = cancel_ok and manual_ok
        response.message = f"cancel={cancel_detail}; manual={manual_detail}"
        return response

    def _handle_status(self, _, response: RouteStatus.Response) -> RouteStatus.Response:
        with self._lock:
            route = self._active_route
            response.active_route = route.name if route else ""
            response.route_loaded = self._route_loaded
            response.route_valid = bool(route.valid) if route else False
            response.autonomy_allowed = self._last_allowed
            response.refusal_reason = self._last_refusal
            response.route_distance_m = self._last_distance if math.isfinite(self._last_distance) else -1.0
            response.heading_error_rad = self._last_heading if math.isfinite(self._last_heading) else -1.0
            response.obstacle_clearance_min_m = self._last_clearance if math.isfinite(self._last_clearance) else -1.0
            response.success = True
            response.message = self._last_status
        return response

    def _on_teach_state(self, msg) -> None:
        """Track teach state transitions to trigger auto-save."""
        if WilnStateMsg is None:
            return
        with self._lock:
            was_recording = self._teach_was_recording
            if msg.state == WilnStateMsg.RECORDING:
                self._teach_was_recording = True
                return
            if msg.state == WilnStateMsg.IDLE and was_recording:
                self._teach_was_recording = False
                # Defer save to avoid holding the lock during I/O
                save_needed = True
            else:
                save_needed = False

        if save_needed and msg.trajectory_poses > 0:
            self._auto_save_route(msg.trajectory_poses, msg.trajectory_length_m)

    def _auto_save_route(self, n_poses: int, length_m: float) -> None:
        """Save the just-recorded trajectory under a timestamped route name."""
        timestamp = datetime.now().strftime("%Y-%m-%d_%H%M")
        route_name = f"route_{timestamp}"
        route_dir = self._routes_dir / route_name
        ltr_path = route_dir / self._route_file_name

        try:
            route_dir.mkdir(parents=True, exist_ok=True)
        except OSError as exc:
            self.get_logger().error(f"Auto-save: cannot create {route_dir}: {exc}")
            return

        # Publish save command to wiln_route_node (which writes .ltr + async .vtk)
        save_cmd = f"save:{ltr_path}"
        self._publish_wiln_command(save_cmd)
        self.get_logger().info(
            f"Auto-save triggered: {route_name} ({n_poses} poses, {length_m:.1f} m) → {ltr_path}"
        )

        # Update 'latest' symlink for convenience (list already ignores it)
        latest_link = self._routes_dir / "latest"
        try:
            if latest_link.is_symlink() or latest_link.exists():
                latest_link.unlink()
            latest_link.symlink_to(route_dir)
        except OSError as exc:
            self.get_logger().warn(f"Auto-save: could not update 'latest' symlink: {exc}")

    def _monitor(self) -> None:
        with self._lock:
            route = self._active_route
            replaying = self._replaying
            selected_mode = self._selected_mode
            replay_started_at = self._replay_started_at
        allowed, reason, distance, heading, clearance, closest = self._evaluate(route)
        self._set_status(allowed, reason, distance, heading, clearance, closest)

        if replaying:
            # Route manager is read-only during replay: it publishes status and
            # logs diagnostics, but does NOT cancel the replay.
            # mtt_repeat_supervisor is the single cancel authority during replay.
            if self._debug:
                now_s = self._now_seconds()
                elapsed = (now_s - replay_started_at) if replay_started_at else 0.0
                self.get_logger().info(
                    f"[MONITOR] replaying=True mode='{selected_mode}' allowed={allowed} "
                    f"reason='{reason}' dist={distance:.2f}m head={heading:.2f}rad "
                    f"elapsed={elapsed:.1f}s"
                )
        self._publish_status()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MttRouteManager()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
