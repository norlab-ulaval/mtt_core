#include "mtt_gps_teach_repeat/logic/articulated_gps_controller.hpp"

#include <algorithm>
#include <cmath>

namespace mtt_gps_teach_repeat
{
namespace logic
{

ArticulatedGpsController::ArticulatedGpsController(
  ArticulatedGpsControllerParams params,
  mtt_control::logic::ArticulatedCommandModel model)
: params_(params), model_(model)
{
}

ArticulatedGpsControllerOutput ArticulatedGpsController::compute(
  const ArticulatedGpsControllerInput & input) const
{
  const double kappa_fb = params_.k_y * input.lateral_deviation_m +
    params_.k_theta * input.course_deviation_rad;
  const double kappa_desired = std::clamp(
    input.curvature_ff + kappa_fb, -params_.kappa_max, params_.kappa_max);

  double speed = 0.0;
  if (std::abs(input.desired_speed_ms) > 1e-3) {
    const double requested_magnitude = std::abs(input.desired_speed_ms) /
      (1.0 + params_.slowdown_alpha * std::abs(kappa_desired));
    const double magnitude = std::clamp(
      requested_magnitude, params_.min_speed_ms, params_.max_speed_ms);
    speed = std::copysign(magnitude, input.desired_speed_ms);
  }

  // The command model's bisection inversion (articulation_from_yaw_rate)
  // already folds in the calibrated slip heuristic, so the desired curvature
  // is converted through a desired body yaw-rate at the commanded speed
  // rather than through a bare geometric atan(kappa * L).
  const double desired_yaw_rate = kappa_desired * speed;
  const double articulation_raw = std::abs(speed) > 1e-3 ?
    model_.articulation_from_yaw_rate(speed, desired_yaw_rate) :
    input.previous_articulation_rad;
  const double articulation = model_.rate_limited_articulation(
    articulation_raw, input.previous_articulation_rad, input.dt_s);

  return {articulation, speed, kappa_desired};
}

}  // namespace logic
}  // namespace mtt_gps_teach_repeat
