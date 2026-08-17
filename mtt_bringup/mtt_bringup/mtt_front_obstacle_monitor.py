#!/usr/bin/env python3
"""Front hazard monitor for MTT teach/repeat.

This node is intentionally simple: transform the front LiDAR cloud into the
robot frame, count real points in front of the chassis, and publish a stop or
slowdown request for the WILN follower.
"""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Float32, Header, String
import tf2_ros


Point3 = Tuple[float, float, float]


@dataclass
class HazardResult:
    stop_points: int
    slow_points: int
    min_front_m: float
    slowdown_scale: float
    raw_points: int
    sampled_points: int
    debug_points: List[Point3]


def _quat_to_rotation(qx: float, qy: float, qz: float, qw: float) -> Tuple[Tuple[float, float, float], ...]:
    xx = qx * qx
    yy = qy * qy
    zz = qz * qz
    xy = qx * qy
    xz = qx * qz
    yz = qy * qz
    wx = qw * qx
    wy = qw * qy
    wz = qw * qz
    return (
        (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
        (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
        (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
    )


class MttFrontObstacleMonitor(Node):
    def __init__(self) -> None:
        super().__init__("mtt_front_obstacle_monitor")

        self.declare_parameter("cloud_topic", "/hesai_lidar/points")
        self.declare_parameter("target_frame", "base_footprint")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("cloud_timeout_s", 0.5)
        self.declare_parameter("tf_timeout_s", 0.08)
        self.declare_parameter("min_range_m", 0.55)
        self.declare_parameter("max_range_m", 8.0)
        self.declare_parameter("z_min_m", -0.05)
        self.declare_parameter("z_max_m", 1.50)
        self.declare_parameter("stop_x_min_m", 0.50)
        self.declare_parameter("stop_x_max_m", 2.00)
        self.declare_parameter("stop_half_width_m", 0.85)
        self.declare_parameter("slow_x_min_m", 2.00)
        self.declare_parameter("slow_x_max_m", 5.00)
        self.declare_parameter("slow_half_width_m", 1.25)
        self.declare_parameter("min_stop_points", 20)
        self.declare_parameter("min_slow_points", 12)
        self.declare_parameter("stop_confirm_frames", 2)
        self.declare_parameter("clear_confirm_frames", 5)
        self.declare_parameter("point_stride", 3)
        self.declare_parameter("max_points", 60000)
        self.declare_parameter("publish_debug_cloud", True)

        self._cloud_topic = str(self.get_parameter("cloud_topic").value)
        self._target_frame = str(self.get_parameter("target_frame").value)
        self._publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self._cloud_timeout_s = float(self.get_parameter("cloud_timeout_s").value)
        self._tf_timeout_s = float(self.get_parameter("tf_timeout_s").value)
        self._min_range_m = float(self.get_parameter("min_range_m").value)
        self._max_range_m = float(self.get_parameter("max_range_m").value)
        self._z_min_m = float(self.get_parameter("z_min_m").value)
        self._z_max_m = float(self.get_parameter("z_max_m").value)
        self._stop_x_min_m = float(self.get_parameter("stop_x_min_m").value)
        self._stop_x_max_m = float(self.get_parameter("stop_x_max_m").value)
        self._stop_half_width_m = float(self.get_parameter("stop_half_width_m").value)
        self._slow_x_min_m = float(self.get_parameter("slow_x_min_m").value)
        self._slow_x_max_m = float(self.get_parameter("slow_x_max_m").value)
        self._slow_half_width_m = float(self.get_parameter("slow_half_width_m").value)
        self._min_stop_points = int(self.get_parameter("min_stop_points").value)
        self._min_slow_points = int(self.get_parameter("min_slow_points").value)
        self._stop_confirm_frames = int(self.get_parameter("stop_confirm_frames").value)
        self._clear_confirm_frames = int(self.get_parameter("clear_confirm_frames").value)
        self._point_stride = max(1, int(self.get_parameter("point_stride").value))
        self._max_points = max(1000, int(self.get_parameter("max_points").value))
        self._publish_debug_cloud = bool(self.get_parameter("publish_debug_cloud").value)

        self._lock = threading.Lock()
        self._latest_cloud: Optional[PointCloud2] = None
        self._latest_cloud_time: Optional[float] = None
        self._stop_hits = 0
        self._clear_hits = self._clear_confirm_frames
        self._stop_active = False
        self._last_tf_error = ""

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self._cloud_sub = self.create_subscription(PointCloud2, self._cloud_topic, self._on_cloud, qos)
        self._stop_pub = self.create_publisher(Bool, "/mtt_obstacle/stop_requested", 10)
        self._slowdown_pub = self.create_publisher(Float32, "/mtt_obstacle/slowdown_scale", 10)
        self._clearance_pub = self.create_publisher(Float32, "/mtt_obstacle/front_clearance_m", 10)
        self._status_pub = self.create_publisher(String, "/mtt_obstacle/hazard_status", 10)
        self._debug_pub = self.create_publisher(PointCloud2, "/mtt_obstacle/debug_cloud", qos)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        period = 1.0 / max(self._publish_rate_hz, 1.0)
        self.create_timer(period, self._timer)
        self.get_logger().info(
            f"front obstacle monitor: cloud={self._cloud_topic} target_frame={self._target_frame} "
            f"stop={self._stop_x_min_m:.1f}..{self._stop_x_max_m:.1f}m "
            f"slow={self._slow_x_min_m:.1f}..{self._slow_x_max_m:.1f}m"
        )

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_cloud(self, msg: PointCloud2) -> None:
        with self._lock:
            self._latest_cloud = msg
            self._latest_cloud_time = self._now_seconds()

    def _transform_for_cloud(self, cloud: PointCloud2):
        source_frame = cloud.header.frame_id.strip().lstrip("/")
        target_frame = self._target_frame.strip().lstrip("/")
        if not source_frame:
            raise RuntimeError("cloud header frame_id is empty")
        if source_frame == target_frame:
            return None
        stamp = cloud.header.stamp
        try:
            return self._tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                stamp,
                timeout=Duration(seconds=self._tf_timeout_s),
            )
        except Exception:
            return self._tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self._tf_timeout_s),
            )

    def _apply_transform(self, point: Point3, transform) -> Point3:
        if transform is None:
            return point
        t = transform.transform.translation
        q = transform.transform.rotation
        rot = _quat_to_rotation(q.x, q.y, q.z, q.w)
        x, y, z = point
        return (
            rot[0][0] * x + rot[0][1] * y + rot[0][2] * z + t.x,
            rot[1][0] * x + rot[1][1] * y + rot[1][2] * z + t.y,
            rot[2][0] * x + rot[2][1] * y + rot[2][2] * z + t.z,
        )

    def _analyze(self, cloud: PointCloud2) -> HazardResult:
        transform = self._transform_for_cloud(cloud)
        stop_points = 0
        slow_points = 0
        raw_points = 0
        sampled_points = 0
        min_front_m = math.inf
        debug_points: List[Point3] = []

        for point in point_cloud2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True):
            raw_points += 1
            if raw_points % self._point_stride != 0:
                continue
            if sampled_points >= self._max_points:
                break
            sampled_points += 1
            p = self._apply_transform((float(point[0]), float(point[1]), float(point[2])), transform)
            x, y, z = p
            r = math.sqrt(x * x + y * y + z * z)
            if r < self._min_range_m or r > self._max_range_m:
                continue
            if z < self._z_min_m or z > self._z_max_m:
                continue
            in_stop = (
                self._stop_x_min_m <= x <= self._stop_x_max_m
                and abs(y) <= self._stop_half_width_m
            )
            in_slow = (
                self._slow_x_min_m <= x <= self._slow_x_max_m
                and abs(y) <= self._slow_half_width_m
            )
            if not in_stop and not in_slow:
                continue
            min_front_m = min(min_front_m, x)
            if len(debug_points) < 5000:
                debug_points.append(p)
            if in_stop:
                stop_points += 1
            elif in_slow:
                slow_points += 1

        if slow_points >= self._min_slow_points and math.isfinite(min_front_m):
            span = max(self._slow_x_max_m - self._stop_x_max_m, 0.1)
            slowdown = 0.35 + 0.65 * max(0.0, min(1.0, (min_front_m - self._stop_x_max_m) / span))
        else:
            slowdown = 1.0

        return HazardResult(
            stop_points=stop_points,
            slow_points=slow_points,
            min_front_m=min_front_m,
            slowdown_scale=slowdown,
            raw_points=raw_points,
            sampled_points=sampled_points,
            debug_points=debug_points,
        )

    def _set_stop_state(self, stop_detected: bool) -> None:
        if stop_detected:
            self._stop_hits += 1
            self._clear_hits = 0
        else:
            self._clear_hits += 1
            self._stop_hits = 0
        if self._stop_hits >= self._stop_confirm_frames:
            self._stop_active = True
        if self._clear_hits >= self._clear_confirm_frames:
            self._stop_active = False

    def _publish(self, stop: bool, slowdown: float, clearance: float, status: str, debug_points: List[Point3]) -> None:
        stop_msg = Bool()
        stop_msg.data = stop
        self._stop_pub.publish(stop_msg)
        slowdown_msg = Float32()
        slowdown_msg.data = float(max(0.0, min(1.0, slowdown)))
        self._slowdown_pub.publish(slowdown_msg)
        clearance_msg = Float32()
        clearance_msg.data = float(clearance) if math.isfinite(clearance) else -1.0
        self._clearance_pub.publish(clearance_msg)
        status_msg = String()
        status_msg.data = status
        self._status_pub.publish(status_msg)
        if self._publish_debug_cloud:
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = self._target_frame
            cloud_msg = point_cloud2.create_cloud_xyz32(
                header=header,
                points=debug_points,
            )
            self._debug_pub.publish(cloud_msg)

    def _timer(self) -> None:
        with self._lock:
            cloud = self._latest_cloud
            cloud_time = self._latest_cloud_time
        if cloud is None or cloud_time is None:
            self._stop_active = True
            self._publish(True, 0.0, math.nan, "cloud missing: /hesai_lidar/points not received", [])
            return
        age = self._now_seconds() - cloud_time
        if age > self._cloud_timeout_s:
            self._stop_active = True
            self._publish(True, 0.0, math.nan, f"cloud stale: age={age:.2f}s", [])
            return

        try:
            result = self._analyze(cloud)
            self._last_tf_error = ""
        except Exception as exc:
            self._last_tf_error = str(exc)
            self._stop_active = True
            self._publish(True, 0.0, math.nan, f"tf/filter unavailable: {self._last_tf_error}", [])
            return

        stop_detected = result.stop_points >= self._min_stop_points
        self._set_stop_state(stop_detected)
        slowdown = 0.0 if self._stop_active else result.slowdown_scale
        clearance = result.min_front_m
        clearance_text = f"{clearance:.2f}m" if math.isfinite(clearance) else "clear"
        status = (
            f"stop={self._stop_active} slowdown={slowdown:.2f} clearance={clearance_text} "
            f"stop_pts={result.stop_points}/{self._min_stop_points} "
            f"slow_pts={result.slow_points}/{self._min_slow_points} "
            f"cloud_age={age:.2f}s sampled={result.sampled_points}"
        )
        self._publish(self._stop_active, slowdown, clearance, status, result.debug_points)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MttFrontObstacleMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
