// MTT-154 Articulated Vehicle Dynamics — implementation
// Direct port of ArticulatedVehicleDynamics.update() from mtt_articulated_model.py

#include "mtt_driver/logic/articulated_model.hpp"

#include <algorithm>
#include <cmath>

namespace mtt::logic {

std::tuple<double, double, double> ArticulatedVehicleDynamics::update(
  double throttle_input,
  double steering_input,
  double dt,
  double terrain_grip)
{
  // 1. Update articulation angle with first-order response
  double target_articulation = steering_input * kMaxArticulation;
  double error = target_articulation - state_.articulation_angle;
  state_.articulation_angle += error * kArticResponse * dt;
  state_.articulation_angle = std::clamp(state_.articulation_angle, -kMaxArticulation, kMaxArticulation);

  // 2. Forward velocity with track-friction deceleration when no throttle
  constexpr double kDecelRate = 3.0;  // m/s² — tracks provide strong braking
  if (std::abs(throttle_input) < 0.01) {
    if (std::abs(state_.linear_velocity) > 0.01) {
      double sign  = (state_.linear_velocity > 0.0) ? -1.0 : 1.0;
      double decel = sign * kDecelRate * dt;
      if (std::abs(decel) >= std::abs(state_.linear_velocity))
        state_.linear_velocity = 0.0;
      else
        state_.linear_velocity += decel;
    } else {
      state_.linear_velocity = 0.0;
    }
  } else {
    double accel_factor = terrain_grip * kGripCoeff;
    double target_vel   = throttle_input * kMaxSpeed;
    double vel_error    = target_vel - state_.linear_velocity;
    state_.linear_velocity += vel_error * accel_factor * 2.0 * dt;
  }

  // 3. Kinematics — tracked vehicles need some forward speed to steer
  if (std::abs(state_.linear_velocity) > kMinSpeedSteer) {
    double wheelbase = kLf + kLr;
    double eff_steer = state_.articulation_angle;
    double slip_factor = kSlipCoeff * 0.3;

    // Articulated joint virtual steering angle
    double virtual_steer = std::atan(
      wheelbase * std::sin(eff_steer) /
      (kLf + kLr * std::cos(eff_steer)));

    double carving = kCarvingFactor * 0.2 * std::abs(virtual_steer);
    double enhanced_steer = virtual_steer * (1.0 + carving);

    state_.angular_velocity =
      (state_.linear_velocity * std::tan(enhanced_steer) / wheelbase)
      * (1.0 - slip_factor);

    if (std::abs(state_.linear_velocity) > 0.01) {
      state_.front_slip_angle = eff_steer - state_.angular_velocity * kLf / state_.linear_velocity;
      state_.rear_slip_angle  = -state_.angular_velocity * kLr / state_.linear_velocity;
    } else {
      state_.front_slip_angle = state_.rear_slip_angle = 0.0;
    }
  } else {
    // Angular velocity decays fast when nearly stopped
    constexpr double kDecayRate = 10.0;  // rad/s²
    if (std::abs(state_.angular_velocity) > 0.01) {
      double sign  = (state_.angular_velocity > 0.0) ? -1.0 : 1.0;
      double decay = sign * kDecayRate * dt;
      if (std::abs(decay) >= std::abs(state_.angular_velocity))
        state_.angular_velocity = 0.0;
      else
        state_.angular_velocity += decay;
    } else {
      state_.angular_velocity = 0.0;
    }
    state_.front_slip_angle = state_.rear_slip_angle = 0.0;
  }

  // 4. Integrate position and heading
  state_.heading += state_.angular_velocity * dt;
  state_.heading = std::atan2(std::sin(state_.heading), std::cos(state_.heading));  // normalize

  state_.x += state_.linear_velocity * std::cos(state_.heading) * dt;
  state_.y += state_.linear_velocity * std::sin(state_.heading) * dt;

  return {state_.x, state_.y, state_.heading};
}

}  // namespace mtt::logic
