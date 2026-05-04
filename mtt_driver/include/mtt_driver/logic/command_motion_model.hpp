// Shared MTT command-motion model.
// Provides one explicit 2D articulated model for:
// - synthetic tachometer / odometry fallback
// - command-only health fallback
// - controller-side geometry helpers

#pragma once

#include <algorithm>

#include "mtt_driver/logic/vehicle_params.hpp"

namespace mtt::logic {

struct CommandMotionParams {
  double wheelbase_m{VehicleParams::total_wheelbase()};
  double max_articulation_rad{VehicleParams::max_articulation_rad};
  double min_turn_speed_ms{VehicleParams::min_speed_for_steering};
  double speed_response_gain{3.0};
  double articulation_response_gain{VehicleParams::articulation_response};
  double brake_gain{1.0};
  bool use_slip_heuristic{true};
  double yaw_slip_base{0.10};
  double yaw_slip_speed_gain{0.05};
  double yaw_slip_articulation_gain{0.15};
  double yaw_slip_min_scale{0.55};
};

struct CommandMotionCommand {
  double linear_speed_cmd_ms{0.0};
  double normalized_steer_cmd{0.0};
  double brake_normalized{0.0};
  double dt{0.02};
};

struct CommandMotionState {
  double x{0.0};
  double y{0.0};
  double heading{0.0};
  double cumulative_distance_m{0.0};

  double v_command_ms{0.0};
  double v_target_ms{0.0};
  double v_eff_ms{0.0};

  double phi_command_rad{0.0};
  double phi_eff_rad{0.0};

  double kappa_nominal_m_inv{0.0};
  double kappa_effective_m_inv{0.0};
  double yaw_rate_nominal_rad_s{0.0};
  double yaw_rate_effective_rad_s{0.0};
  double slip_scale{1.0};
};

class CommandMotionModel {
public:
  CommandMotionModel() = default;
  explicit CommandMotionModel(const CommandMotionParams& params) : params_(params) {}

  void set_params(const CommandMotionParams& params) { params_ = params; }
  const CommandMotionParams& params() const { return params_; }

  void reset() { state_ = CommandMotionState{}; }
  void set_state(const CommandMotionState& state) { state_ = state; }
  const CommandMotionState& state() const { return state_; }

  const CommandMotionState& step(const CommandMotionCommand& command, bool integrate_pose = true);

  static double normalized_steer_to_articulation_rad(
    double normalized_steer,
    double max_articulation_rad = VehicleParams::max_articulation_rad);
  static double articulation_rad_to_normalized_steer(
    double articulation_rad,
    double max_articulation_rad = VehicleParams::max_articulation_rad);
  static double articulation_from_curvature(double curvature_m_inv, const CommandMotionParams& params);
  static double normalized_steer_from_curvature(double curvature_m_inv, const CommandMotionParams& params);
  static double normalized_steer_from_yaw_rate(
    double yaw_rate_rad_s,
    double speed_ms,
    const CommandMotionParams& params);
  static double curvature_from_articulation(double articulation_rad, const CommandMotionParams& params);
  static double curvature_from_steer(double normalized_steer, const CommandMotionParams& params);
  static double yaw_rate_from_speed_and_articulation(
    double speed_ms,
    double articulation_rad,
    const CommandMotionParams& params,
    bool apply_slip = false);
  static double yaw_rate_from_speed_and_steer(
    double normalized_steer,
    double speed_ms,
    const CommandMotionParams& params,
    bool apply_slip = false);
  static double slip_scale(double speed_ms, double articulation_rad, const CommandMotionParams& params);

private:
  CommandMotionParams params_{};
  CommandMotionState state_{};
};

}  // namespace mtt::logic
