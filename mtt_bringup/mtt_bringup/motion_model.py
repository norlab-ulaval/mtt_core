import math
from dataclasses import dataclass


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


@dataclass
class MotionModelParams:
    wheelbase_m: float
    max_articulation_rad: float
    min_turn_speed_ms: float
    use_slip_heuristic: bool
    yaw_response_gain: float
    yaw_slip_base: float
    yaw_slip_speed_gain: float
    yaw_slip_articulation_gain: float
    yaw_slip_min_scale: float


def articulation_from_normalized_steer(normalized_steer: float, params: MotionModelParams) -> float:
    return clamp(normalized_steer, -1.0, 1.0) * max(params.max_articulation_rad, 1e-6)


def normalized_steer_from_articulation(articulation_rad: float, params: MotionModelParams) -> float:
    return clamp(articulation_rad / max(params.max_articulation_rad, 1e-6), -1.0, 1.0)


def articulation_from_curvature(curvature_m_inv: float, params: MotionModelParams) -> float:
    if abs(curvature_m_inv) < 1e-12 or params.wheelbase_m <= 1e-9:
        return 0.0
    return clamp(
        math.atan(curvature_m_inv * params.wheelbase_m),
        -params.max_articulation_rad,
        params.max_articulation_rad,
    )


def slip_scale(speed_ms: float, articulation_rad: float, params: MotionModelParams) -> float:
    if not params.use_slip_heuristic:
        return clamp(params.yaw_response_gain, params.yaw_slip_min_scale, 1.0)

    normalized_articulation = abs(articulation_rad) / max(params.max_articulation_rad, 1e-6)
    scale = (
        params.yaw_response_gain
        - params.yaw_slip_base
        - params.yaw_slip_speed_gain * abs(speed_ms)
        - params.yaw_slip_articulation_gain * normalized_articulation
    )
    return clamp(scale, params.yaw_slip_min_scale, 1.0)
