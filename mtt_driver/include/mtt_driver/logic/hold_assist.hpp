// Hold-assist controller for low-speed stopping and slope holding.
// Designed to stay deterministic so the same behavior can be validated in
// real hardware, simulation, and bag replay.

#pragma once

#include <algorithm>
#include <cmath>
#include <string>

namespace mtt::logic {

struct HoldAssistParams {
  bool enabled{true};
  double entry_speed_ms{0.03};
  double release_speed_ms{0.015};
  double exit_command_ms{0.08};
  double kp{1.2};
  double ki{0.8};
  double integrator_limit{0.25};
  double output_limit{0.35};
  double deadband_compensation{0.12};
  bool dither_enabled{false};
  double dither_amplitude{0.02};
  double dither_frequency_hz{6.0};

  // ── Active-decel phase (counter-thrust when coasting after release) ──
  // Requires tachometer_mode: real.  Disabled by default.
  // When |measured_speed| > active_decel_entry_speed_ms AND command ≈ 0,
  // applies a pure-P counter-thrust (kp * speed) up to active_decel_output_limit.
  // Integrator is NOT accumulated during this phase.
  // Below active_decel_entry_speed_ms the normal PI hold takes over.
  bool   active_decel_enabled{false};
  double active_decel_entry_speed_ms{0.15};  // m/s — switch to PI hold below this
  double active_decel_output_limit{0.60};    // m/s — max counter-thrust
};

struct HoldAssistInput {
  double dt{0.02};
  double commanded_speed_ms{0.0};
  double measured_speed_ms{0.0};
  bool telemetry_fresh{false};
  bool safety_locked{false};
  double brake_normalized{0.0};
};

struct HoldAssistOutput {
  bool active{false};
  bool dither_active{false};
  double correction_speed_ms{0.0};
  std::string mode{"off"};
};

class HoldAssistController {
public:
  void set_params(const HoldAssistParams& params) { params_ = params; }
  const HoldAssistParams& params() const { return params_; }

  void reset()
  {
    integral_ = 0.0;
    dither_phase_rad_ = 0.0;
    last_output_ = HoldAssistOutput{};
  }

  const HoldAssistOutput& last_output() const { return last_output_; }

  const HoldAssistOutput& update(const HoldAssistInput& input)
  {
    const bool eligible =
      params_.enabled &&
      input.telemetry_fresh &&
      !input.safety_locked &&
      input.brake_normalized < 0.95 &&
      std::abs(input.commanded_speed_ms) <= params_.exit_command_ms;

    if (!eligible) {
      reset();
      return last_output_;
    }

    const double measured_speed_ms = input.measured_speed_ms;
    const double speed_abs_ms = std::abs(measured_speed_ms);
    const bool should_hold =
      speed_abs_ms >= params_.entry_speed_ms ||
      (last_output_.active && speed_abs_ms >= params_.release_speed_ms) ||
      std::abs(integral_) > 1e-4;

    if (!should_hold) {
      integral_ = 0.0;
      dither_phase_rad_ = 0.0;
      last_output_ = HoldAssistOutput{};
      return last_output_;
    }

    // ── Active-decel phase: pure-P counter-thrust at higher speeds ───────
    // Precondition: eligible AND should_hold (already verified above).
    // Uses kp only (no integrator), with a higher output limit, to brake
    // proportionally to measured speed.  Integrator is reset so there is
    // no overshoot once PI hold takes over below active_decel_entry_speed_ms.
    if (params_.active_decel_enabled &&
        speed_abs_ms >= params_.active_decel_entry_speed_ms) {
      integral_ = 0.0;
      dither_phase_rad_ = 0.0;
      double correction_speed_ms = params_.kp * (-measured_speed_ms);
      correction_speed_ms = std::clamp(
        correction_speed_ms,
        -params_.active_decel_output_limit,
        params_.active_decel_output_limit);
      last_output_.active = std::abs(correction_speed_ms) > 1e-6;
      last_output_.dither_active = false;
      last_output_.correction_speed_ms = last_output_.active ? correction_speed_ms : 0.0;
      last_output_.mode = last_output_.active ? "decel" : "off";
      return last_output_;
    }

    // ── Normal PI hold phase ─────────────────────────────────────────────
    const double dt = std::clamp(input.dt, 0.0, 1.0);
    const double error_ms = -measured_speed_ms;
    integral_ += error_ms * params_.ki * dt;
    integral_ = std::clamp(integral_, -params_.integrator_limit, params_.integrator_limit);

    double correction_speed_ms = params_.kp * error_ms + integral_;
    if (speed_abs_ms >= params_.entry_speed_ms &&
        std::abs(correction_speed_ms) < params_.deadband_compensation) {
      const double sign_source = std::abs(error_ms) > 1e-6 ? error_ms : -input.commanded_speed_ms;
      if (std::abs(sign_source) > 1e-6) {
        correction_speed_ms = std::copysign(params_.deadband_compensation, sign_source);
      }
    }

    bool dither_active = false;
    if (params_.dither_enabled && dt > 0.0) {
      dither_phase_rad_ += 2.0 * M_PI * params_.dither_frequency_hz * dt;
      dither_phase_rad_ = std::fmod(dither_phase_rad_, 2.0 * M_PI);
      // Square-wave dither added symmetrically around correction.
      // On level ground (correction ≈ 0) this alternates between
      // +amplitude and -amplitude, physically rocking the vehicle
      // forward/backward to defeat the mechanical deadband.
      // On a slope (large correction) both phases keep the same sign
      // but vary by ±amplitude — still useful against static friction.
      const double dither_sign = std::sin(dither_phase_rad_) >= 0.0 ? 1.0 : -1.0;
      correction_speed_ms += params_.dither_amplitude * dither_sign;
      dither_active = true;
    }

    correction_speed_ms = std::clamp(
      correction_speed_ms,
      -params_.output_limit,
      params_.output_limit);

    last_output_.active = std::abs(correction_speed_ms) > 1e-6;
    last_output_.dither_active = dither_active && last_output_.active;
    last_output_.correction_speed_ms = last_output_.active ? correction_speed_ms : 0.0;
    last_output_.mode = !last_output_.active ? "off" : (last_output_.dither_active ? "dither" : "hold");
    return last_output_;
  }

private:
  HoldAssistParams params_{};
  double integral_{0.0};
  double dither_phase_rad_{0.0};
  HoldAssistOutput last_output_{};
};

}  // namespace mtt::logic
