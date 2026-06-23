// SpeedServo — Pure C++17 PI speed controller with feedforward LUT
//
// Closed-loop speed tracking using tachometer feedback.
// Designed to sit between the path follower and the CAN node's cmd_vel input.
//
// Control law:
//   rate_limited_setpoint = rate_limit(setpoint, max_accel, max_decel, dt)
//   error  = rate_limited_setpoint - measured_speed
//   p      = kp * error
//   i      = clamp(integrator + ki * error * dt, ±integrator_limit)
//   ff     = feedforward_gain * lut_lookup(rate_limited_setpoint)
//   output = clamp(ff + p + i, 0.0, max_throttle)   [0..1 normalized throttle]
//
// Feedforward LUT:
//   Populated from the "characterize" mode of MttSpeedServoNode.
//   Maps desired speed (m/s) → steady-state throttle fraction.
//   Provides ~90% of the answer immediately; PI corrects for disturbances.
//
// Design choices:
//   - Rate limiter on setpoint: prevents integrator windup on step commands
//   - Separate accel / decel limits: deceleration can be more aggressive
//   - Output is [0, max_throttle] (unsigned) — direction handled by cmd_vel sign
//   - No D term by default: speed control is inherently low-bandwidth

#pragma once

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace mtt::logic
{

struct SpeedServoParams
{
  double kp{1.5};                ///< throttle_normalized / (m/s error)
  double ki{0.8};                ///< integrator — needed for steady-state on slopes/load
  double kd{0.0};                ///< derivative (usually not needed for speed)
  double integrator_limit{0.30}; ///< anti-windup clamp [throttle units]
  double max_throttle{1.0};      ///< output saturation [0, 1]
  double max_speed_ms{2.0};      ///< setpoint clamp (m/s)
  double max_accel_ms2{1.5};     ///< rate limiter: max acceleration (m/s²)
  double max_decel_ms2{2.5};     ///< rate limiter: max deceleration (m/s²)
  double feedforward_gain{0.0};  ///< 0=disabled, 1=full LUT trust
};

struct SpeedServoDebug
{
  double setpoint_ms{0.0};          ///< requested speed (before rate limiter)
  double rate_limited_setpoint_ms{0.0}; ///< after accel/decel limiting
  double measured_ms{0.0};          ///< tachometer feedback
  double error_ms{0.0};             ///< rate_limited_setpoint - measured
  double p_term{0.0};
  double i_term{0.0};
  double d_term{0.0};
  double feedforward{0.0};
  double throttle_normalized{0.0};  ///< final clamped output [0, 1]
  bool saturated{false};
};

class SpeedServo
{
public:
  // ── Configuration ──
  void set_params(const SpeedServoParams & p) { params_ = p; }
  const SpeedServoParams & params() const { return params_; }

  /// Set the feedforward LUT (speed_ms, throttle_normalized) pairs, sorted ascending by speed.
  void set_feedforward_lut(std::vector<std::pair<double, double>> lut)
  {
    // Sort by speed ascending
    std::sort(lut.begin(), lut.end(),
      [](const auto & a, const auto & b) { return a.first < b.first; });
    feedforward_lut_ = std::move(lut);
  }

  // ── State reset ──
  void reset(double initial_speed_ms = 0.0)
  {
    setpoint_ms_ = std::clamp(initial_speed_ms, 0.0, params_.max_speed_ms);
    rate_limited_setpoint_ms_ = setpoint_ms_;
    integrator_ = 0.0;
    prev_error_ = 0.0;
    prev_error_initialized_ = false;
  }

  // ── Setpoint ──
  void set_setpoint(double speed_ms)
  {
    setpoint_ms_ = std::clamp(std::abs(speed_ms), 0.0, params_.max_speed_ms);
  }

  double setpoint_ms() const { return setpoint_ms_; }

  // ── Main update — call at fixed rate ──
  /// Returns normalized throttle [0, 1] and fills debug.
  /// measured_speed_ms: absolute value of tachometer speed (direction handled by caller)
  double compute(double measured_speed_ms, double dt, SpeedServoDebug & dbg)
  {
    // ── Rate-limit the setpoint ──
    const double accel_limit = params_.max_accel_ms2 * dt;
    const double decel_limit = params_.max_decel_ms2 * dt;
    const double delta = setpoint_ms_ - rate_limited_setpoint_ms_;
    if (delta > 0.0) {
      rate_limited_setpoint_ms_ += std::min(delta, accel_limit);
    } else {
      rate_limited_setpoint_ms_ -= std::min(-delta, decel_limit);
    }
    rate_limited_setpoint_ms_ = std::clamp(rate_limited_setpoint_ms_, 0.0, params_.max_speed_ms);

    // ── PI + D ──
    const double error = rate_limited_setpoint_ms_ - measured_speed_ms;

    integrator_ = std::clamp(
      integrator_ + error * dt,
      -params_.integrator_limit, params_.integrator_limit);

    double d_term = 0.0;
    if (prev_error_initialized_ && dt > 1e-6) {
      d_term = params_.kd * (error - prev_error_) / dt;
    }
    prev_error_ = error;
    prev_error_initialized_ = true;

    const double p_term = params_.kp * error;
    const double i_term = params_.ki * integrator_;

    // ── Feedforward ──
    const double ff = params_.feedforward_gain * lookup_feedforward(rate_limited_setpoint_ms_);

    // ── Output ──
    const double u_raw = ff + p_term + i_term + d_term;
    const double u = std::clamp(u_raw, 0.0, params_.max_throttle);

    dbg.setpoint_ms               = setpoint_ms_;
    dbg.rate_limited_setpoint_ms  = rate_limited_setpoint_ms_;
    dbg.measured_ms               = measured_speed_ms;
    dbg.error_ms                  = error;
    dbg.p_term                    = p_term;
    dbg.i_term                    = i_term;
    dbg.d_term                    = d_term;
    dbg.feedforward               = ff;
    dbg.throttle_normalized       = u;
    dbg.saturated                 = (u_raw > params_.max_throttle || u_raw < 0.0);

    return u;
  }

private:
  SpeedServoParams params_;
  double setpoint_ms_{0.0};
  double rate_limited_setpoint_ms_{0.0};
  double integrator_{0.0};
  double prev_error_{0.0};
  bool prev_error_initialized_{false};
  std::vector<std::pair<double, double>> feedforward_lut_;

  /// Linear interpolation in the feedforward LUT.
  double lookup_feedforward(double speed_ms) const
  {
    if (feedforward_lut_.empty() || speed_ms <= 0.0) return 0.0;
    if (speed_ms <= feedforward_lut_.front().first) return feedforward_lut_.front().second;
    if (speed_ms >= feedforward_lut_.back().first)  return feedforward_lut_.back().second;

    // Binary search for the bracket
    for (std::size_t i = 1; i < feedforward_lut_.size(); ++i) {
      if (feedforward_lut_[i].first >= speed_ms) {
        const double s0 = feedforward_lut_[i - 1].first;
        const double t0 = feedforward_lut_[i - 1].second;
        const double s1 = feedforward_lut_[i].first;
        const double t1 = feedforward_lut_[i].second;
        const double alpha = (speed_ms - s0) / std::max(s1 - s0, 1e-9);
        return t0 + alpha * (t1 - t0);
      }
    }
    return feedforward_lut_.back().second;
  }
};

}  // namespace mtt::logic
