// MTT-154 Articulated Vehicle Dynamics Model
// Ported from mtt_articulated_model.py — numpy import dropped (was unused).
// Pure C++17, no ROS dependency.

#pragma once

#include <cmath>
#include <tuple>

#include "mtt_driver/logic/vehicle_params.hpp"

namespace mtt::logic {

// State of the articulated vehicle after each integration step
struct VehicleState {
  double x{0.0};
  double y{0.0};
  double heading{0.0};
  double articulation_angle{0.0};
  double linear_velocity{0.0};
  double angular_velocity{0.0};
  double front_slip_angle{0.0};
  double rear_slip_angle{0.0};
};

class ArticulatedVehicleDynamics {
public:
  ArticulatedVehicleDynamics() = default;

  // Update vehicle state for one time step.
  // throttle_input in [-1, 1], steering_input in [-1, 1], dt in seconds.
  // Returns (x, y, heading).
  std::tuple<double, double, double> update(
    double throttle_input,
    double steering_input,
    double dt,
    double terrain_grip = 1.0);

  // Direct state accessors
  VehicleState state() const { return state_; }
  void set_state(double x, double y, double heading) {
    state_.x = x;
    state_.y = y;
    state_.heading = heading;
  }
  void reset() { state_ = VehicleState{}; }

  // Expose individual fields for odometry node compatibility
  double x()                 const { return state_.x; }
  double y()                 const { return state_.y; }
  double heading()           const { return state_.heading; }
  double articulation_angle() const { return state_.articulation_angle; }
  double linear_velocity()   const { return state_.linear_velocity; }
  double angular_velocity()  const { return state_.angular_velocity; }

  void set_x(double v)       { state_.x = v; }
  void set_y(double v)       { state_.y = v; }
  void set_heading(double v) { state_.heading = v; }
  void set_articulation(double v) { state_.articulation_angle = v; }

private:
  VehicleState state_{};

  // Constants from VehicleParams — avoid repeated lookups
  static constexpr double kMaxArticulation  = VehicleParams::max_articulation_rad;
  static constexpr double kArticResponse    = VehicleParams::articulation_response;
  static constexpr double kMaxSpeed         = VehicleParams::max_speed_ms;
  static constexpr double kGripCoeff        = VehicleParams::track_grip_coeff;
  static constexpr double kSlipCoeff        = VehicleParams::track_slip_coeff;
  static constexpr double kCarvingFactor    = VehicleParams::carving_factor;
  static constexpr double kMinSpeedSteer    = VehicleParams::min_speed_for_steering;
  static constexpr double kLf               = VehicleParams::l_f;
  static constexpr double kLr               = VehicleParams::l_r;
};

}  // namespace mtt::logic
