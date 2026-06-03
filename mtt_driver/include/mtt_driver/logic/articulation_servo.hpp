// ArticulationServo — Pure C++17 PD articulation position/velocity servo controller
//
// Outer software loop on top of the hardware servo (CAN steer byte / closed_loop mode).
// Corrects for calibration drift, backlash, and nonlinearity in the normalized_steer
// ↔ articulation_rad mapping using the hardware encoder as feedback.
//
// Control law (position mode):
//   e(t) = φ_target - φ_measured
//   u(t) = Kp·e + Kd·ė + Ki·∫e dt
//   steer_cmd = clamp(u, ±max_steer)
//
// Velocity mode:
//   φ_target += vel_cmd · dt   (integrated, clamped to ±max_articulation_rad)
//   then same PD law
//
// Design choices:
//   - No steady-state error for a position hold (pure P is fine since hardware servo
//     already removes most error; Ki disabled by default to avoid windup on sloped terrain)
//   - Derivative on error (not measurement) for bumpless target changes
//   - Anti-windup via integrator clamping

#pragma once

#include <algorithm>
#include <cmath>

namespace mtt::logic
{

struct ArticulationServoParams
{
  double kp{2.0};                    ///< (normalized_steer / rad) proportional gain
  double kd{0.05};                   ///< derivative damping on error
  double ki{0.0};                    ///< integral gain (disabled by default)
  double integrator_limit{0.30};     ///< anti-windup clamp (normalized_steer·s)
  double max_steer{1.0};             ///< output saturation [0..1]
  double max_articulation_rad{1.047};///< ±60° physical limit
  double max_velocity_rad_s{0.50};   ///< velocity mode rate limit
};

struct ArticulationServoDebug
{
  double setpoint_rad{0.0};
  double measured_rad{0.0};
  double error_rad{0.0};
  double error_dot_rad_s{0.0};
  double steer_cmd{0.0};    ///< final clamped output [-1,+1]
  bool saturated{false};    ///< true if output hit max_steer
};

class ArticulationServo
{
public:
  // ── Configuration ─────────────────────────────────────────────────
  void set_params(const ArticulationServoParams & p) { params_ = p; }
  const ArticulationServoParams & params() const { return params_; }

  // ── State reset (call on mode switch or when feedback lost) ────────
  void reset(double initial_setpoint_rad = 0.0)
  {
    setpoint_rad_ = std::clamp(initial_setpoint_rad,
      -params_.max_articulation_rad, params_.max_articulation_rad);
    prev_error_ = 0.0;
    integral_   = 0.0;
    prev_error_initialized_ = false;
  }

  // ── Setpoint generators ────────────────────────────────────────────

  /// Position mode: set absolute target angle (radians, clamped to ±max_articulation_rad)
  void set_position(double target_rad)
  {
    setpoint_rad_ = std::clamp(target_rad,
      -params_.max_articulation_rad, params_.max_articulation_rad);
  }

  /// Velocity mode: integrate angular rate into position setpoint
  void step_velocity(double vel_rad_s, double dt)
  {
    const double vel = std::clamp(vel_rad_s,
      -params_.max_velocity_rad_s, params_.max_velocity_rad_s);
    setpoint_rad_ = std::clamp(setpoint_rad_ + vel * dt,
      -params_.max_articulation_rad, params_.max_articulation_rad);
  }

  // ── Main update — call at fixed rate ──────────────────────────────
  /// Returns clamped normalized steer command [-1,+1] and fills debug.
  double compute(double measured_rad, double dt, ArticulationServoDebug & dbg)
  {
    const double error = setpoint_rad_ - measured_rad;

    double error_dot = 0.0;
    if (prev_error_initialized_ && dt > 1e-6) {
      error_dot = (error - prev_error_) / dt;
    }
    prev_error_ = error;
    prev_error_initialized_ = true;

    // Integral with anti-windup
    integral_ = std::clamp(
      integral_ + error * dt,
      -params_.integrator_limit, params_.integrator_limit);

    const double u_raw = params_.kp * error
                       + params_.kd * error_dot
                       + params_.ki * integral_;

    const double u = std::clamp(u_raw, -params_.max_steer, params_.max_steer);

    dbg.setpoint_rad    = setpoint_rad_;
    dbg.measured_rad    = measured_rad;
    dbg.error_rad       = error;
    dbg.error_dot_rad_s = error_dot;
    dbg.steer_cmd       = u;
    dbg.saturated       = (std::abs(u_raw) > params_.max_steer);

    return u;
  }

  double setpoint_rad() const { return setpoint_rad_; }

private:
  ArticulationServoParams params_;
  double setpoint_rad_{0.0};
  double prev_error_{0.0};
  double integral_{0.0};
  bool   prev_error_initialized_{false};
};

}  // namespace mtt::logic
