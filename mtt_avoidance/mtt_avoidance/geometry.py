"""Pure geometry and policy helpers shared by the avoidance ROS nodes."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Sequence


@dataclass(frozen=True)
class Pose2:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class ValidationResult:
    valid: bool
    min_clearance_m: float
    first_invalid_pose: int
    message: str


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def path_length_xy(poses: Sequence) -> float:
    return sum(
        math.hypot(
            poses[i].pose.position.x - poses[i - 1].pose.position.x,
            poses[i].pose.position.y - poses[i - 1].pose.position.y,
        )
        for i in range(1, len(poses))
    )


def nearest_pose_index(poses: Sequence, x: float, y: float, start: int = 0) -> int:
    if not poses:
        return -1
    start = max(0, min(start, len(poses) - 1))
    return min(
        range(start, len(poses)),
        key=lambda i: (
            (poses[i].pose.position.x - x) ** 2
            + (poses[i].pose.position.y - y) ** 2,
            i,
        ),
    )


def rejoin_candidate_indices(
    poses: Sequence,
    nearest_index: int,
    distances_m: Iterable[float],
) -> list[int]:
    """Return unique forward route indices at requested arc-length distances."""
    if not poses or nearest_index < 0:
        return []
    targets = sorted(max(0.0, float(v)) for v in distances_m)
    result: list[int] = []
    accumulated = 0.0
    target_pos = 0
    for i in range(nearest_index + 1, len(poses)):
        accumulated += math.hypot(
            poses[i].pose.position.x - poses[i - 1].pose.position.x,
            poses[i].pose.position.y - poses[i - 1].pose.position.y,
        )
        while target_pos < len(targets) and accumulated >= targets[target_pos]:
            if not result or result[-1] != i:
                result.append(i)
            target_pos += 1
        if target_pos >= len(targets):
            break
    if not result and nearest_index < len(poses) - 1:
        result.append(len(poses) - 1)
    return result


def transform_polygon(polygon: Sequence[tuple[float, float]], pose: Pose2):
    c = math.cos(pose.yaw)
    s = math.sin(pose.yaw)
    return [
        (pose.x + c * x - s * y, pose.y + s * x + c * y)
        for x, y in polygon
    ]


def inflate_polygon(
    polygon: Sequence[tuple[float, float]], margin: float
) -> list[tuple[float, float]]:
    """Conservatively inflate a convex footprint about its centroid.

    Scaling by the closest centroid-to-edge distance guarantees every edge
    moves outward by at least ``margin``. Moving each vertex radially by the
    margin would under-inflate the long edges of a rectangle.
    """
    if margin <= 0.0 or not polygon:
        return list(polygon)
    cx = sum(p[0] for p in polygon) / len(polygon)
    cy = sum(p[1] for p in polygon) / len(polygon)
    edge_distances = []
    for first, second in zip(polygon, list(polygon[1:]) + [polygon[0]]):
        dx = second[0] - first[0]
        dy = second[1] - first[1]
        length = math.hypot(dx, dy)
        if length > 1e-9:
            edge_distances.append(
                abs(dx * (first[1] - cy) - (first[0] - cx) * dy) / length
            )
    if not edge_distances or min(edge_distances) <= 1e-9:
        raise ValueError("footprint must be a non-degenerate convex polygon")
    scale = 1.0 + margin / min(edge_distances)
    return [(cx + scale * (x - cx), cy + scale * (y - cy)) for x, y in polygon]


def point_in_polygon(x: float, y: float, polygon: Sequence[tuple[float, float]]) -> bool:
    inside = False
    j = len(polygon) - 1
    for i in range(len(polygon)):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        if (yi > y) != (yj > y):
            crossing_x = (xj - xi) * (y - yi) / (yj - yi) + xi
            if x < crossing_x:
                inside = not inside
        j = i
    return inside


def propagate_trailer_yaws(
    tractor_poses: Sequence[Pose2], initial_yaw: float, axle_distance_m: float
) -> list[float]:
    """Low-speed on-axle trailer prediction for forward and reverse motion."""
    if not tractor_poses:
        return []
    trailer_yaw = initial_yaw
    result = [trailer_yaw]
    length = max(axle_distance_m, 0.1)
    for previous, current in zip(tractor_poses, tractor_poses[1:]):
        dx = current.x - previous.x
        dy = current.y - previous.y
        # Smac path orientations describe the vehicle body. Projection on the
        # body x-axis therefore provides the signed travel direction.
        ds = dx * math.cos(previous.yaw) + dy * math.sin(previous.yaw)
        trailer_yaw = normalize_angle(
            trailer_yaw + ds / length * math.sin(previous.yaw - trailer_yaw)
        )
        result.append(trailer_yaw)
    return result


class OccupancyGridView:
    """Small collision-checking adapter for nav_msgs/OccupancyGrid-like data."""

    def __init__(
        self,
        width: int,
        height: int,
        resolution: float,
        origin_x: float,
        origin_y: float,
        data: Sequence[int],
        lethal_threshold: int = 65,
        unknown_is_occupied: bool = True,
    ):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.data = data
        self.lethal_threshold = lethal_threshold
        self.unknown_is_occupied = unknown_is_occupied

    def _occupied(self, mx: int, my: int) -> bool:
        if mx < 0 or my < 0 or mx >= self.width or my >= self.height:
            return True
        value = self.data[my * self.width + mx]
        if value < 0:
            return self.unknown_is_occupied
        return value >= self.lethal_threshold

    def polygon_collides(self, polygon: Sequence[tuple[float, float]]) -> bool:
        min_x = min(p[0] for p in polygon)
        max_x = max(p[0] for p in polygon)
        min_y = min(p[1] for p in polygon)
        max_y = max(p[1] for p in polygon)
        mx0 = math.floor((min_x - self.origin_x) / self.resolution)
        mx1 = math.floor((max_x - self.origin_x) / self.resolution)
        my0 = math.floor((min_y - self.origin_y) / self.resolution)
        my1 = math.floor((max_y - self.origin_y) / self.resolution)
        for my in range(my0, my1 + 1):
            cy = self.origin_y + (my + 0.5) * self.resolution
            for mx in range(mx0, mx1 + 1):
                if not self._occupied(mx, my):
                    continue
                cx = self.origin_x + (mx + 0.5) * self.resolution
                if point_in_polygon(cx, cy, polygon):
                    return True
        # Vertices outside the map must also be treated as collision.
        return any(
            x < self.origin_x
            or y < self.origin_y
            or x >= self.origin_x + self.width * self.resolution
            or y >= self.origin_y + self.height * self.resolution
            for x, y in polygon
        )

    def approximate_clearance(
        self, x: float, y: float, footprint_radius: float, search_radius: float
    ) -> float:
        cells = max(1, math.ceil(search_radius / self.resolution))
        center_x = math.floor((x - self.origin_x) / self.resolution)
        center_y = math.floor((y - self.origin_y) / self.resolution)
        best = math.inf
        for my in range(center_y - cells, center_y + cells + 1):
            for mx in range(center_x - cells, center_x + cells + 1):
                if not self._occupied(mx, my):
                    continue
                cx = self.origin_x + (mx + 0.5) * self.resolution
                cy = self.origin_y + (my + 0.5) * self.resolution
                best = min(best, math.hypot(cx - x, cy - y) - footprint_radius)
        return max(0.0, best) if math.isfinite(best) else search_radius


def validate_composite_path(
    grid: OccupancyGridView,
    tractor_poses: Sequence[Pose2],
    tractor_footprint: Sequence[tuple[float, float]],
    trailer_footprint: Sequence[tuple[float, float]],
    trailer_yaws: Sequence[float],
    hitch_offset_x: float,
    margin_m: float,
    sample_stride: int = 1,
    clearance_search_m: float = 4.0,
) -> ValidationResult:
    if not tractor_poses:
        return ValidationResult(False, 0.0, -1, "empty path")
    if trailer_footprint and len(trailer_yaws) != len(tractor_poses):
        return ValidationResult(False, 0.0, -1, "trailer prediction size mismatch")

    # OccupancyGrid stores cell occupancy, not point obstacles. Expanding by a
    # cell's circumradius makes the later center-point test conservative for any
    # occupied cell intersecting the physical polygon.
    raster_margin = margin_m + grid.resolution * math.sqrt(2.0) * 0.5
    tractor_shape = inflate_polygon(tractor_footprint, raster_margin)
    trailer_shape = inflate_polygon(trailer_footprint, raster_margin)
    tractor_radius = max(math.hypot(x, y) for x, y in tractor_shape)
    trailer_radius = (
        max(math.hypot(x, y) for x, y in trailer_shape) if trailer_shape else 0.0
    )
    minimum = math.inf
    indices = list(range(0, len(tractor_poses), max(1, sample_stride)))
    if indices[-1] != len(tractor_poses) - 1:
        indices.append(len(tractor_poses) - 1)

    for index in indices:
        tractor = tractor_poses[index]
        tractor_world = transform_polygon(tractor_shape, tractor)
        if grid.polygon_collides(tractor_world):
            return ValidationResult(False, 0.0, index, "tractor footprint collision")
        minimum = min(
            minimum,
            grid.approximate_clearance(
                tractor.x, tractor.y, tractor_radius, clearance_search_m
            ),
        )

        if trailer_shape:
            hitch_x = tractor.x + math.cos(tractor.yaw) * hitch_offset_x
            hitch_y = tractor.y + math.sin(tractor.yaw) * hitch_offset_x
            trailer = Pose2(hitch_x, hitch_y, trailer_yaws[index])
            trailer_world = transform_polygon(trailer_shape, trailer)
            if grid.polygon_collides(trailer_world):
                return ValidationResult(False, 0.0, index, "trailer footprint collision")
            minimum = min(
                minimum,
                grid.approximate_clearance(
                    hitch_x, hitch_y, trailer_radius, clearance_search_m
                ),
            )

    if not math.isfinite(minimum):
        minimum = clearance_search_m
    return ValidationResult(True, minimum, -1, "composite swept path is clear")
