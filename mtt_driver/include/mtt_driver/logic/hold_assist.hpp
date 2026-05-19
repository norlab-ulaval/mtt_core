// Hold-assist controller for low-speed stopping and slope holding.
// Pure-P controller: no integrator (no windup), no dither, no deadband compensation.
// Designed to stay deterministic so the same behavior can be validated in
// real hardware, simulation, and bag replay.

#pragma once

#include <algorithm>
#include <cmath>
#include <string>

namespace mtt::logic {

struct HoldAssistParams {
  bool enabled{true};
  double entry_speed_ms{0.08};    // enter hold when |tach| < this AND |cmd| ~ 0
  double exit_command_ms{0.08};   // exit hold when |cmd| > this
  double exit_speed_ms{0.15};     // exit hold when overwhelmed (slope too steep)
  double kp{2.5};                  // P gain: correction = -kp * measured_speed
  double output_limit{0.40};      // max hold thrust (m/s equivalent)
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

  void reset() { last_output_ = HoldAssistOutput{}; }

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
      last_output_ = HoldAssistOutput{};
      return last_output_;
    }

    const double speed = input.measured_speed_ms;
    const double speed_abs = std::abs(speed);

    // Overwhelmed — slope too steep, hold assist cannot maintain position
    if (speed_abs > params_.exit_speed_ms) {
      last_output_ = {false, false, 0.0, "overwhelmed"};
      return last_output_;
    }

    // Enter or maintain hold: proportional counter-thrust only, no integrator
    if (speed_abs < params_.entry_speed_ms || last_output_.active) {
      double correction = std::clamp(
        -params_.kp * speed, -params_.output_limit, params_.output_limit);
      // Near-zero speed → holding successfully, suppress correction to avoid hunting
      if (speed_abs < 0.005) { correction = 0.0; }
      last_output_ = {true, false, correction, "hold"};
      return last_output_;
    }

    last_output_ = HoldAssistOutput{};
    return last_output_;
  }

private:
  HoldAssistParams params_{};
  HoldAssistOutput last_output_{};
};

}  // namespace mtt::logic
