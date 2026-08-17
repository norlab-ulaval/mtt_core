#pragma once

#include "mtt_control/logic/articulated_command_model.hpp"

namespace mtt_gps_teach_repeat
{
namespace logic
{

struct ArticulatedGpsControllerParams
{
  double k_y{0.20};
  double k_theta{0.90};
  double kappa_max{0.35};
  double slowdown_alpha{0.60};
  double min_speed_ms{0.35};
  double max_speed_ms{1.60};
};

struct ArticulatedGpsControllerInput
{
  double lateral_deviation_m{0.0};
  double course_deviation_rad{0.0};
  double curvature_ff{0.0};
  double desired_speed_ms{0.0};
  double previous_articulation_rad{0.0};
  double dt_s{0.0};
};

struct ArticulatedGpsControllerOutput
{
  double articulation_rad{0.0};
  double speed_ms{0.0};
  double kappa_desired{0.0};
};

/** Frenet-error feedback law for an articulated vehicle following a ROMEA
 * matched path point.
 *
 * The curvature-to-articulation inversion and rate limiting are delegated to
 * mtt_control::logic::ArticulatedCommandModel (bisection over the calibrated
 * slip model) so this controller does not duplicate that logic — it only
 * turns Frenet errors into a desired curvature and desired speed.
 */
class ArticulatedGpsController
{
public:
  ArticulatedGpsController(
    ArticulatedGpsControllerParams params,
    mtt_control::logic::ArticulatedCommandModel model);

  [[nodiscard]] ArticulatedGpsControllerOutput compute(
    const ArticulatedGpsControllerInput & input) const;

private:
  ArticulatedGpsControllerParams params_;
  mtt_control::logic::ArticulatedCommandModel model_;
};

}  // namespace logic
}  // namespace mtt_gps_teach_repeat
