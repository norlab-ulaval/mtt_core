from types import SimpleNamespace

from mtt_avoidance.geometry import (
    inflate_polygon,
    OccupancyGridView,
    Pose2,
    propagate_trailer_yaws,
    rejoin_candidate_indices,
    validate_composite_path,
)


def _pose(x, y=0.0):
    return SimpleNamespace(
        pose=SimpleNamespace(position=SimpleNamespace(x=x, y=y))
    )


def _grid(obstacles=()):
    width = height = 100
    resolution = 0.1
    data = [0] * (width * height)
    for x, y in obstacles:
        mx = int((x + 5.0) / resolution)
        my = int((y + 5.0) / resolution)
        data[my * width + mx] = 100
    return OccupancyGridView(
        width, height, resolution, -5.0, -5.0, data, unknown_is_occupied=True
    )


def test_rejoin_candidates_use_metric_arc_length():
    poses = [_pose(i * 0.1) for i in range(101)]
    assert rejoin_candidate_indices(poses, 10, [1.0, 3.0, 5.0]) == [20, 40, 60]


def test_polygon_inflation_moves_rectangle_edges_by_full_margin():
    inflated = inflate_polygon(
        [(2.0, 1.0), (2.0, -1.0), (-2.0, -1.0), (-2.0, 1.0)],
        0.2,
    )
    assert max(y for _, y in inflated) >= 1.2
    assert min(y for _, y in inflated) <= -1.2
    assert max(x for x, _ in inflated) >= 2.2
    assert min(x for x, _ in inflated) <= -2.2


def test_trailer_prediction_uses_signed_reverse_motion():
    initial_trailer_yaw = -0.20
    forward = propagate_trailer_yaws(
        [Pose2(0.0, 0.0, 0.0), Pose2(0.5, 0.0, 0.0)],
        initial_trailer_yaw,
        2.5,
    )
    reverse = propagate_trailer_yaws(
        [Pose2(0.0, 0.0, 0.0), Pose2(-0.5, 0.0, 0.0)],
        initial_trailer_yaw,
        2.5,
    )
    assert forward[-1] > initial_trailer_yaw
    assert reverse[-1] < initial_trailer_yaw


def test_clear_composite_path_is_valid():
    tractor = [Pose2(0.0, 0.0, 0.0), Pose2(1.0, 0.0, 0.0)]
    trailer_yaws = propagate_trailer_yaws(tractor, 0.0, 2.5)
    result = validate_composite_path(
        _grid(),
        tractor,
        [(0.5, 0.4), (0.5, -0.4), (-0.5, -0.4), (-0.5, 0.4)],
        [(0.2, 0.4), (0.2, -0.4), (-1.5, -0.4), (-1.5, 0.4)],
        trailer_yaws,
        hitch_offset_x=-0.5,
        margin_m=0.1,
    )
    assert result.valid


def test_trailer_collision_rejects_path_even_when_tractor_is_clear():
    tractor = [Pose2(0.0, 0.0, 0.0), Pose2(1.0, 0.0, 0.0)]
    result = validate_composite_path(
        _grid([(-1.5, 0.0)]),
        tractor,
        [(0.5, 0.4), (0.5, -0.4), (-0.5, -0.4), (-0.5, 0.4)],
        [(0.2, 0.4), (0.2, -0.4), (-1.5, -0.4), (-1.5, 0.4)],
        [0.0, 0.0],
        hitch_offset_x=-0.5,
        margin_m=0.0,
    )
    assert not result.valid
    assert "trailer" in result.message


def test_path_outside_costmap_is_rejected_fail_closed():
    result = validate_composite_path(
        _grid(),
        [Pose2(20.0, 0.0, 0.0)],
        [(0.5, 0.4), (0.5, -0.4), (-0.5, -0.4), (-0.5, 0.4)],
        [],
        [],
        hitch_offset_x=0.0,
        margin_m=0.0,
    )
    assert not result.valid
