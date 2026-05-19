#!/usr/bin/env python3
"""Field route manager for WILN taught routes."""

import math
import os
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from mtt_interfaces.srv import RouteCommand, RouteList, RouteStatus
from mtt_msgs.msg import MttHealthState
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker
from wiln.srv import LoadMapTraj

try:
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
except ImportError:  # pragma: no cover - only absent on non-ROS test hosts
    PointCloud2 = None
    point_cloud2 = None


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
    if warnings == ["large_xy_jump"] or warnings == ["large_yaw_jump"]:
        return "usable_with_caution", warnings
    return "reject_for_replay", warnings


class MttRouteManager(Node):
    def __init__(self) -> None:
        super().__init__("mtt_route_manager")

        self.declare_parameter("routes_dir", "data/wiln_routes")
        self.declare_parameter("icp_odom_topic", "/mapping/icp_odom")
        self.declare_parameter("health_topic", "/mtt_health")
        self.declare_parameter("selected_mode_topic", "mtt_control/selected_mode")
        self.declare_parameter("deadman_topic", "teleop_deadman")
        self.declare_parameter("obstacle_cloud_topic", "/mtt_perception/obstacles")
        self.declare_parameter("load_map_traj_service", "/load_map_traj")
        self.declare_parameter("mark_ready_service", "/mtt_repeat/mark_ready")
        self.declare_parameter("play_line_service", "/mtt_repeat/play_line")
        self.declare_parameter("cancel_service", "/mtt_repeat/cancel")
        self.declare_parameter("request_manual_service", "mtt_control/request_manual")
        self.declare_parameter("route_file_name", "route.ltr")
        self.declare_parameter("icp_timeout_s", 0.75)
        self.declare_parameter("health_timeout_s", 1.0)
        self.declare_parameter("max_start_distance_m", 2.0)
        self.declare_parameter("max_lateral_error_m", 1.25)
        self.declare_parameter("max_heading_error_rad", 1.2)
        self.declare_parameter("route_error_grace_s", 1.0)
        self.declare_parameter("max_step_warn_m", 0.75)
        self.declare_parameter("max_yaw_step_warn_rad", 0.50)
        self.declare_parameter("min_obstacle_clearance_m", 0.75)
        self.declare_parameter("obstacle_corridor_half_width_m", 0.75)
        self.declare_parameter("obstacle_lookahead_m", 4.0)
        self.declare_parameter("require_obstacle_clearance", False)
        self.declare_parameter("monitor_rate_hz", 10.0)

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
        self._max_step_warn_m = float(self.get_parameter("max_step_warn_m").value)
        self._max_yaw_step_warn_rad = float(self.get_parameter("max_yaw_step_warn_rad").value)
        self._min_obstacle_clearance_m = float(self.get_parameter("min_obstacle_clearance_m").value)
        self._obstacle_corridor_half_width_m = float(
            self.get_parameter("obstacle_corridor_half_width_m").value
        )
        self._obstacle_lookahead_m = float(self.get_parameter("obstacle_lookahead_m").value)
        self._require_obstacle_clearance = bool(self.get_parameter("require_obstacle_clearance").value)
        monitor_rate_hz = float(self.get_parameter("monitor_rate_hz").value)

        self._group = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._active_route: Optional[RouteData] = None
        self._route_loaded = False
        self._replaying = False
        self._bad_route_error_since: Optional[float] = None
        self._icp: Optional[Odometry] = None
        self._icp_received_time: Optional[float] = None
        self._health: Optional[MttHealthState] = None
        self._health_received_time: Optional[float] = None
        self._selected_mode = "stop"
        self._deadman = False
        self._deadman_received_time: Optional[float] = None
        self._obstacle_points: List[Tuple[float, float, float]] = []
        self._obstacle_received_time: Optional[float] = None
        self._last_status = "no route loaded"
        self._last_allowed = False
        self._last_refusal = "no route loaded"
        self._last_distance = float("nan")
        self._last_heading = float("nan")
        self._last_clearance = float("nan")
        self._last_closest: Optional[RoutePose] = None

        self._load_client = self.create_client(
            LoadMapTraj, str(self.get_parameter("load_map_traj_service").value), callback_group=self._group
        )
        self._mark_ready_client = self.create_client(
            Trigger, str(self.get_parameter("mark_ready_service").value), callback_group=self._group
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
        self.create_subscription(Bool, str(self.get_parameter("deadman_topic").value), self._on_deadman, 20)
        if PointCloud2 is not None:
            self.create_subscription(
                PointCloud2,
                str(self.get_parameter("obstacle_cloud_topic").value),
                self._on_obstacles,
                5,
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
        self._allowed_pub = self.create_publisher(Bool, "/mtt_route/autonomy_allowed", latched_qos)
        self._clearance_pub = self.create_publisher(Float32, "/mtt_route/obstacle_clearance_min", latched_qos)

        self.create_service(RouteList, "/mtt_route/list", self._handle_list, callback_group=self._group)
        self.create_service(RouteCommand, "/mtt_route/load", self._handle_load, callback_group=self._group)
        self.create_service(RouteCommand, "/mtt_route/validate", self._handle_validate, callback_group=self._group)
        self.create_service(RouteCommand, "/mtt_route/preview", self._handle_preview, callback_group=self._group)
        self.create_service(RouteCommand, "/mtt_route/replay", self._handle_replay, callback_group=self._group)
        self.create_service(Trigger, "/mtt_route/stop", self._handle_stop, callback_group=self._group)
        self.create_service(RouteStatus, "/mtt_route/status", self._handle_status, callback_group=self._group)

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
        if route is None:
            return False, "no route loaded", float("nan"), float("nan"), float("nan"), None
        if not route.valid:
            return False, f"route rejected: {route.grade}", float("nan"), float("nan"), float("nan"), None

        icp_ok, icp = self._icp_fresh()
        if not icp_ok or icp is None:
            return False, "ICP odom stale or absent", float("nan"), float("nan"), float("nan"), None
        health_ok, health = self._health_fresh()
        if not health_ok or health is None:
            return False, "mtt_health stale or absent", float("nan"), float("nan"), float("nan"), None
        if not health.security_unlocked:
            return False, "security_unlocked=false", float("nan"), float("nan"), float("nan"), None
        if health.emergency_stop_active:
            return False, "emergency stop active", float("nan"), float("nan"), float("nan"), None
        if health.fallback_active and health.fallback_low_confidence:
            return False, f"health fallback low confidence: {health.fallback_reason}", float("nan"), float("nan"), float("nan"), None
        if self._deadman_override_active():
            return False, "manual deadman override active", float("nan"), float("nan"), float("nan"), None

        robot = self._robot_pose(icp)
        closest, distance, heading = self._closest_pose(route, robot)
        clearance = self._obstacle_clearance(route, robot)
        start = route.poses[0]
        start_distance = math.hypot(robot.x - start.x, robot.y - start.y)
        if start_distance > self._max_start_distance_m:
            return False, f"too far from route start: {start_distance:.2f} m > {self._max_start_distance_m:.2f} m", distance, heading, clearance, closest
        if distance > self._max_lateral_error_m:
            return False, f"too far from route: {distance:.2f} m > {self._max_lateral_error_m:.2f} m", distance, heading, clearance, closest
        if heading > self._max_heading_error_rad:
            return False, f"heading error: {heading:.2f} rad > {self._max_heading_error_rad:.2f} rad", distance, heading, clearance, closest
        if self._require_obstacle_clearance and not math.isfinite(clearance):
            return False, "obstacle clearance unavailable", distance, heading, clearance, closest
        if math.isfinite(clearance) and clearance < self._min_obstacle_clearance_m:
            return False, f"obstacle too close: {clearance:.2f} m < {self._min_obstacle_clearance_m:.2f} m", distance, heading, clearance, closest
        return True, "ready", distance, heading, clearance, closest

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

    def _call_load(self, route: RouteData, timeout_s: float = 3.0) -> Tuple[bool, str]:
        if not self._load_client.wait_for_service(timeout_sec=0.0):
            return False, "/load_map_traj unavailable"
        request = LoadMapTraj.Request()
        request.file_name.data = str(route.path)
        future = self._load_client.call_async(request)
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        if not event.wait(timeout_s):
            return False, "load_map_traj timeout"
        try:
            future.result()
        except Exception as exc:  # pragma: no cover - runtime safety
            return False, str(exc)
        ok, detail = self._call_trigger(self._mark_ready_client)
        if not ok:
            return False, f"route loaded, but mark_ready failed: {detail}"
        return True, "route loaded and armed"

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
            self._last_allowed = allowed
            self._last_refusal = "" if allowed else refusal
            self._last_distance = distance
            self._last_heading = heading
            self._last_clearance = clearance
            self._last_closest = closest
            state = "allowed" if allowed else f"blocked: {refusal}"
            clearance_text = f"{clearance:.2f} m" if math.isfinite(clearance) else "unknown"
            self._last_status = (
                f"route={route_name} mode={mode} auto={state}; "
                f"distance={distance:.2f} m heading={heading:.2f} rad clearance={clearance_text}; "
                "buttons: A auto, B stop, Y manual, RB deadman, LB steering mode"
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
        ok, detail = self._call_load(route)
        if ok:
            with self._lock:
                self._active_route = route
                self._route_loaded = True
                self._replaying = False
            self._publish_route(route)
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

        allowed, reason, distance, heading, clearance, closest = self._evaluate(route)
        self._set_status(allowed, reason, distance, heading, clearance, closest)
        if not allowed:
            response.success = False
            response.message = reason
            return response
        if not loaded:
            ok, detail = self._call_load(route)
            if not ok:
                response.success = False
                response.message = detail
                return response
        ok, detail = self._call_trigger(self._play_line_client)
        if ok:
            with self._lock:
                self._active_route = route
                self._route_loaded = True
                self._replaying = True
                self._bad_route_error_since = None
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

    def _monitor(self) -> None:
        with self._lock:
            route = self._active_route
            replaying = self._replaying
            selected_mode = self._selected_mode
        allowed, reason, distance, heading, clearance, closest = self._evaluate(route)
        self._set_status(allowed, reason, distance, heading, clearance, closest)

        if replaying:
            now_s = self._now_seconds()
            if selected_mode != "auto":
                self._call_trigger(self._cancel_client)
                with self._lock:
                    self._replaying = False
                reason = f"control mode changed to {selected_mode}"
            elif not allowed:
                with self._lock:
                    if self._bad_route_error_since is None:
                        self._bad_route_error_since = now_s
                    bad_since = self._bad_route_error_since
                if bad_since is not None and now_s - bad_since > self._route_error_grace_s:
                    self._call_trigger(self._cancel_client)
                    self._call_trigger(self._request_manual_client)
                    with self._lock:
                        self._replaying = False
                    reason = f"replay cancelled: {reason}"
            else:
                with self._lock:
                    self._bad_route_error_since = None
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
