#include "mtt_driver/logic/command_motion_model.hpp"

#include <cmath>

namespace mtt::logic {

namespace {

double wrap_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double clamp01(double value)
{
  return std::clamp(value, 0.0, 1.0);
}

}  // namespace

double CommandMotionModel::normalized_steer_to_articulation_rad(
  double normalized_steer,
  double max_articulation_rad)
{
  return std::clamp(normalized_steer, -1.0, 1.0) * std::max(max_articulation_rad, 1e-6);
}

double CommandMotionModel::articulation_rad_to_normalized_steer(
  double articulation_rad,
  double max_articulation_rad)
{
  return std::clamp(articulation_rad / std::max(max_articulation_rad, 1e-6), -1.0, 1.0);
}

double CommandMotionModel::articulation_from_curvature(
  double curvature_m_inv,
  const CommandMotionParams& params)
{
  if (std::abs(curvature_m_inv) < 1e-12 || params.wheelbase_m <= 1e-9) {
    return 0.0;
  }

  return std::clamp(
    std::atan(curvature_m_inv * params.wheelbase_m),
    -params.max_articulation_rad,
    params.max_articulation_rad);
}

double CommandMotionModel::normalized_steer_from_curvature(
  double curvature_m_inv,
  const CommandMotionParams& params)
{
  return articulation_rad_to_normalized_steer(
    articulation_from_curvature(curvature_m_inv, params),
    params.max_articulation_rad);
}

double CommandMotionModel::normalized_steer_from_yaw_rate(
  double yaw_rate_rad_s,
  double speed_ms,
  const CommandMotionParams& params)
{
  if (std::abs(speed_ms) < std::max(params.min_turn_speed_ms, 1e-6) || params.wheelbase_m <= 1e-9) {
    return 0.0;
  }

  const double articulation = std::atan((yaw_rate_rad_s * params.wheelbase_m) / speed_ms);
  return articulation_rad_to_normalized_steer(articulation, params.max_articulation_rad);
}

double CommandMotionModel::curvature_from_articulation(
  double articulation_rad,
  const CommandMotionParams& params)
{
  if (params.wheelbase_m <= 1e-9) {
    return 0.0;
  }

  const double clamped_phi = std::clamp(
    articulation_rad,
    -params.max_articulation_rad,
    params.max_articulation_rad);
  return std::tan(clamped_phi) / params.wheelbase_m;
}

double CommandMotionModel::curvature_from_steer(
  double normalized_steer,
  const CommandMotionParams& params)
{
  return curvature_from_articulation(
    normalized_steer_to_articulation_rad(normalized_steer, params.max_articulation_rad),
    params);
}

double CommandMotionModel::slip_scale(
  double speed_ms,
  double articulation_rad,
  const CommandMotionParams& params)
{
  if (!params.use_slip_heuristic) {
    return 1.0;
  }

  const double normalized_articulation =
    std::abs(articulation_rad) / std::max(params.max_articulation_rad, 1e-6);
  const double scale =
    1.0
    - params.yaw_slip_base
    - params.yaw_slip_speed_gain * std::abs(speed_ms)
    - params.yaw_slip_articulation_gain * normalized_articulation;
  return std::clamp(scale, params.yaw_slip_min_scale, 1.0);
}

double CommandMotionModel::yaw_rate_from_speed_and_articulation(
  double speed_ms,
  double articulation_rad,
  const CommandMotionParams& params,
  bool apply_slip)
{
  if (std::abs(speed_ms) < std::max(params.min_turn_speed_ms, 1e-6)) {
    return 0.0;
  }

  double curvature = curvature_from_articulation(articulation_rad, params);
  if (apply_slip) {
    curvature *= slip_scale(speed_ms, articulation_rad, params);
  }
  return speed_ms * curvature;
}

double CommandMotionModel::yaw_rate_from_speed_and_steer(
  double normalized_steer,
  double speed_ms,
  const CommandMotionParams& params,
  bool apply_slip)
{
  return yaw_rate_from_speed_and_articulation(
    speed_ms,
    normalized_steer_to_articulation_rad(normalized_steer, params.max_articulation_rad),
    params,
    apply_slip);
}

const CommandMotionState& CommandMotionModel::step(
  const CommandMotionCommand& command,
  bool integrate_pose)
{
  const double dt = std::clamp(command.dt, 0.0, 1.0);
  state_.v_command_ms = command.linear_speed_cmd_ms;
  state_.phi_command_rad = normalized_steer_to_articulation_rad(
    command.normalized_steer_cmd,
    params_.max_articulation_rad);

  const double brake_scale = clamp01(1.0 - params_.brake_gain * clamp01(command.brake_normalized));
  state_.v_target_ms = command.linear_speed_cmd_ms * brake_scale;

  const double speed_alpha = clamp01(params_.speed_response_gain * dt);
  const double articulation_alpha = clamp01(params_.articulation_response_gain * dt);

  state_.v_eff_ms += (state_.v_target_ms - state_.v_eff_ms) * speed_alpha;
  state_.phi_eff_rad += (state_.phi_command_rad - state_.phi_eff_rad) * articulation_alpha;
  state_.phi_eff_rad = std::clamp(
    state_.phi_eff_rad,
    -params_.max_articulation_rad,
    params_.max_articulation_rad);

  if (std::abs(state_.v_eff_ms) < std::max(params_.min_turn_speed_ms, 1e-6)) {
    state_.kappa_nominal_m_inv = 0.0;
    state_.kappa_effective_m_inv = 0.0;
    state_.yaw_rate_nominal_rad_s = 0.0;
    state_.yaw_rate_effective_rad_s = 0.0;
    state_.slip_scale = params_.use_slip_heuristic ? 1.0 : 1.0;
  } else {
    state_.kappa_nominal_m_inv = curvature_from_articulation(state_.phi_eff_rad, params_);
    state_.slip_scale = slip_scale(state_.v_eff_ms, state_.phi_eff_rad, params_);
    state_.kappa_effective_m_inv = state_.kappa_nominal_m_inv * state_.slip_scale;
    state_.yaw_rate_nominal_rad_s = state_.v_eff_ms * state_.kappa_nominal_m_inv;
    state_.yaw_rate_effective_rad_s = state_.v_eff_ms * state_.kappa_effective_m_inv;
  }

  if (integrate_pose && dt > 1e-9) {
    const double dtheta = state_.yaw_rate_effective_rad_s * dt;
    const double ds = state_.v_eff_ms * dt;
    const double heading_mid = state_.heading + 0.5 * dtheta;
    state_.x += ds * std::cos(heading_mid);
    state_.y += ds * std::sin(heading_mid);
    state_.heading = wrap_angle(state_.heading + dtheta);
    state_.cumulative_distance_m += std::abs(ds);
  }

  return state_;
}

}  // namespace mtt::logic
