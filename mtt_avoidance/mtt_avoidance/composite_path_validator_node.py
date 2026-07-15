"""Validate tractor and optional trailer swept footprints against a live costmap."""

from __future__ import annotations

import math
import threading

from geometry_msgs.msg import PoseWithCovarianceStamped
from mtt_interfaces.srv import ValidateCompositePath
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from .geometry import (
    OccupancyGridView,
    Pose2,
    propagate_trailer_yaws,
    quaternion_to_yaw,
    validate_composite_path,
)


def _polygon(values: list[float], name: str) -> list[tuple[float, float]]:
    if len(values) < 6 or len(values) % 2:
        raise ValueError(f"{name} must contain at least three x,y pairs")
    return list(zip(values[0::2], values[1::2]))


class CompositePathValidator(Node):
    def __init__(self) -> None:
        super().__init__("mtt_composite_path_validator")
        self.declare_parameter("costmap_topic", "/mtt_avoidance/global_costmap/costmap_raw")
        self.declare_parameter("tractor_odom_topic", "/mapping/icp_measurement")
        self.declare_parameter("trailer_pose_topic", "/trailer/pose_in_map")
        self.declare_parameter("articulation_topic", "/hardware/articulation_angle")
        self.declare_parameter("validation_service", "/mtt_avoidance/validate_path")
        self.declare_parameter("costmap_timeout_s", 0.75)
        self.declare_parameter("state_timeout_s", 0.5)
        self.declare_parameter("lethal_threshold", 65)
        self.declare_parameter("unknown_is_occupied", True)
        self.declare_parameter("include_trailer", True)
        self.declare_parameter("require_trailer_pose", False)
        self.declare_parameter("hitch_offset_x_m", -1.0)
        self.declare_parameter("trailer_axle_distance_m", 2.5)
        self.declare_parameter("base_margin_m", 0.20)
        self.declare_parameter("covariance_margin_scale", 2.5)
        self.declare_parameter("sample_stride", 2)
        self.declare_parameter("clearance_search_m", 4.0)
        self.declare_parameter(
            "tractor_footprint",
            [1.80, 1.25, 1.80, -1.25, -1.00, -1.25, -1.00, 1.25],
        )
        self.declare_parameter(
            "trailer_footprint",
            [0.50, 1.25, 0.50, -1.25, -3.00, -1.25, -3.00, 1.25],
        )

        self._costmap: OccupancyGrid | None = None
        self._costmap_time = None
        self._tractor_odom: Odometry | None = None
        self._tractor_time = None
        self._trailer_pose: PoseWithCovarianceStamped | None = None
        self._trailer_time = None
        self._articulation = 0.0
        self._articulation_time = None
        self._lock = threading.Lock()

        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("costmap_topic").value),
            self._on_costmap,
            1,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("tractor_odom_topic").value),
            self._on_tractor,
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter("trailer_pose_topic").value),
            self._on_trailer,
            10,
        )
        self.create_subscription(
            Float64,
            str(self.get_parameter("articulation_topic").value),
            self._on_articulation,
            10,
        )
        self.create_service(
            ValidateCompositePath,
            str(self.get_parameter("validation_service").value),
            self._validate,
        )
        self.get_logger().info(
            "Composite path validator ready (tractor + configurable trailer footprint)."
        )

    def _on_costmap(self, msg: OccupancyGrid) -> None:
        with self._lock:
            self._costmap = msg
            self._costmap_time = self.get_clock().now()

    def _on_tractor(self, msg: Odometry) -> None:
        with self._lock:
            self._tractor_odom = msg
            self._tractor_time = self.get_clock().now()

    def _on_trailer(self, msg: PoseWithCovarianceStamped) -> None:
        with self._lock:
            self._trailer_pose = msg
            self._trailer_time = self.get_clock().now()

    def _on_articulation(self, msg: Float64) -> None:
        if math.isfinite(msg.data):
            with self._lock:
                self._articulation = float(msg.data)
                self._articulation_time = self.get_clock().now()

    @staticmethod
    def _reject(response, message: str):
        response.valid = False
        response.min_clearance_m = 0.0
        response.first_invalid_pose = -1
        response.message = message
        return response

    def _validate(self, request, response):
        now = self.get_clock().now()
        with self._lock:
            costmap = self._costmap
            costmap_time = self._costmap_time
            tractor_odom = self._tractor_odom
            tractor_time = self._tractor_time
            trailer_pose = self._trailer_pose
            trailer_time = self._trailer_time
            articulation = self._articulation
            articulation_time = self._articulation_time

        if not request.path.poses:
            return self._reject(response, "empty path")
        if costmap is None or costmap_time is None:
            return self._reject(response, "costmap unavailable")
        costmap_age = (now - costmap_time).nanoseconds * 1e-9
        if costmap_age > float(self.get_parameter("costmap_timeout_s").value):
            return self._reject(response, f"costmap stale ({costmap_age:.2f}s)")
        if tractor_odom is None or tractor_time is None:
            return self._reject(response, "tractor state unavailable")
        state_timeout = float(self.get_parameter("state_timeout_s").value)
        if (now - tractor_time).nanoseconds * 1e-9 > state_timeout:
            return self._reject(response, "tractor state stale")
        if request.path.header.frame_id != costmap.header.frame_id:
            return self._reject(
                response,
                "frame mismatch: "
                f"path={request.path.header.frame_id!r} costmap={costmap.header.frame_id!r}",
            )

        poses = [
            Pose2(
                p.pose.position.x,
                p.pose.position.y,
                quaternion_to_yaw(p.pose.orientation),
            )
            for p in request.path.poses
        ]
        include_trailer = bool(self.get_parameter("include_trailer").value)
        trailer_yaws: list[float] = []
        trailer_shape: list[tuple[float, float]] = []
        trailer_sigma = 0.0
        if include_trailer:
            trailer_fresh = (
                trailer_pose is not None
                and trailer_time is not None
                and (now - trailer_time).nanoseconds * 1e-9 <= state_timeout
            )
            if bool(self.get_parameter("require_trailer_pose").value) and not trailer_fresh:
                return self._reject(response, "fresh trailer pose required but unavailable")
            if trailer_fresh:
                initial_trailer_yaw = quaternion_to_yaw(trailer_pose.pose.pose.orientation)
                trailer_cov = trailer_pose.pose.covariance
                trailer_sigma = math.sqrt(max(0.0, trailer_cov[0], trailer_cov[7]))
            else:
                articulation_fresh = (
                    articulation_time is not None
                    and (now - articulation_time).nanoseconds * 1e-9 <= state_timeout
                )
                if not articulation_fresh:
                    return self._reject(
                        response,
                        "neither trailer pose nor articulation is fresh",
                    )
                initial_trailer_yaw = poses[0].yaw - articulation
            trailer_yaws = propagate_trailer_yaws(
                poses,
                initial_trailer_yaw,
                float(self.get_parameter("trailer_axle_distance_m").value),
            )
            trailer_shape = _polygon(
                list(self.get_parameter("trailer_footprint").value),
                "trailer_footprint",
            )

        tractor_cov = tractor_odom.pose.covariance
        tractor_sigma = math.sqrt(max(0.0, tractor_cov[0], tractor_cov[7]))
        margin = float(self.get_parameter("base_margin_m").value) + float(
            self.get_parameter("covariance_margin_scale").value
        ) * max(tractor_sigma, trailer_sigma)

        grid = OccupancyGridView(
            width=costmap.info.width,
            height=costmap.info.height,
            resolution=costmap.info.resolution,
            origin_x=costmap.info.origin.position.x,
            origin_y=costmap.info.origin.position.y,
            data=costmap.data,
            lethal_threshold=int(self.get_parameter("lethal_threshold").value),
            unknown_is_occupied=bool(self.get_parameter("unknown_is_occupied").value),
        )
        result = validate_composite_path(
            grid=grid,
            tractor_poses=poses,
            tractor_footprint=_polygon(
                list(self.get_parameter("tractor_footprint").value),
                "tractor_footprint",
            ),
            trailer_footprint=trailer_shape,
            trailer_yaws=trailer_yaws,
            hitch_offset_x=float(self.get_parameter("hitch_offset_x_m").value),
            margin_m=margin,
            sample_stride=int(self.get_parameter("sample_stride").value),
            clearance_search_m=float(self.get_parameter("clearance_search_m").value),
        )
        response.valid = result.valid
        response.min_clearance_m = float(result.min_clearance_m)
        response.first_invalid_pose = result.first_invalid_pose
        response.message = f"{result.message}; uncertainty margin={margin:.2f}m"
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CompositePathValidator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
