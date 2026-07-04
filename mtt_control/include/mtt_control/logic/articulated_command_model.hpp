#pragma once

#include <algorithm>
#include <cmath>

namespace mtt_control::logic
{

/** Equivalent low-speed model used to map MTT articulation to body yaw-rate.
 *
 * This is deliberately named articulated rather than Ackermann.  The equivalent
 * length and slip coefficients are the same calibrated quantities used by WILN
 * and mtt_driver.  Positive and negative longitudinal speeds are both supported.
 */
struct ArticulatedCommandParams
{
  double equivalent_length_m{2.4};
  double max_articulation_rad{0.733};
  double min_turn_speed_ms{0.05};
  double max_articulation_rate_rad_s{0.50};
  bool use_slip_heuristic{true};
  double yaw_slip_base{0.10};
  double yaw_slip_speed_gain{0.05};
  double yaw_slip_articulation_gain{0.15};
  double yaw_slip_min_scale{0.55};
};

class ArticulatedCommandModel
{
public:
  explicit ArticulatedCommandModel(ArticulatedCommandParams params = {})
  : params_(params)
  {
  }

  [[nodiscard]] double slip_scale(double speed_ms, double articulation_rad) const
  {
    if (!params_.use_slip_heuristic) {
      return 1.0;
    }
    const double normalized = std::abs(articulation_rad) /
      std::max(params_.max_articulation_rad, 1e-6);
    const double scale = 1.0 - params_.yaw_slip_base -
      params_.yaw_slip_speed_gain * std::abs(speed_ms) -
      params_.yaw_slip_articulation_gain * normalized;
    return std::clamp(scale, params_.yaw_slip_min_scale, 1.0);
  }

  [[nodiscard]] double effective_curvature(
    double speed_ms, double articulation_rad) const
  {
    const double angle = std::clamp(
      articulation_rad,
      -params_.max_articulation_rad,
      params_.max_articulation_rad);
    const double nominal = std::tan(angle) /
      std::max(params_.equivalent_length_m, 1e-6);
    return nominal * slip_scale(speed_ms, angle);
  }

  [[nodiscard]] double yaw_rate(double speed_ms, double articulation_rad) const
  {
    if (std::abs(speed_ms) < std::max(params_.min_turn_speed_ms, 1e-6)) {
      return 0.0;
    }
    return speed_ms * effective_curvature(speed_ms, articulation_rad);
  }

  /** Invert desired body yaw-rate into an absolute articulation setpoint.
   *
   * Bisection includes the articulation-dependent slip term.  With reverse
   * speed the articulation sign naturally flips for a fixed world yaw-rate.
   */
  [[nodiscard]] double articulation_from_yaw_rate(
    double speed_ms, double desired_yaw_rate_rad_s) const
  {
    if (!std::isfinite(speed_ms) || !std::isfinite(desired_yaw_rate_rad_s) ||
      std::abs(speed_ms) < std::max(params_.min_turn_speed_ms, 1e-6))
    {
      return 0.0;
    }

    const double target_curvature = desired_yaw_rate_rad_s / speed_ms;
    double low = -params_.max_articulation_rad;
    double high = params_.max_articulation_rad;
    const double min_curvature = effective_curvature(speed_ms, low);
    const double max_curvature = effective_curvature(speed_ms, high);
    if (target_curvature <= min_curvature) {
      return low;
    }
    if (target_curvature >= max_curvature) {
      return high;
    }
    for (int iteration = 0; iteration < 48; ++iteration) {
      const double middle = 0.5 * (low + high);
      if (effective_curvature(speed_ms, middle) < target_curvature) {
        low = middle;
      } else {
        high = middle;
      }
    }
    return 0.5 * (low + high);
  }

  [[nodiscard]] double rate_limited_articulation(
    double desired_rad, double reference_rad, double dt_s) const
  {
    const double desired = std::clamp(
      desired_rad,
      -params_.max_articulation_rad,
      params_.max_articulation_rad);
    const double reference = std::clamp(
      reference_rad,
      -params_.max_articulation_rad,
      params_.max_articulation_rad);
    const double max_delta = params_.max_articulation_rate_rad_s *
      std::clamp(dt_s, 0.0, 1.0);
    return std::clamp(desired, reference - max_delta, reference + max_delta);
  }

  [[nodiscard]] double normalized(double articulation_rad) const
  {
    return std::clamp(
      articulation_rad / std::max(params_.max_articulation_rad, 1e-6),
      -1.0,
      1.0);
  }

  [[nodiscard]] double denormalized(double normalized_articulation) const
  {
    return std::clamp(normalized_articulation, -1.0, 1.0) *
           params_.max_articulation_rad;
  }

private:
  ArticulatedCommandParams params_;
};

}  // namespace mtt_control::logic
