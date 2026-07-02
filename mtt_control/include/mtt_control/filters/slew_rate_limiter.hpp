#pragma once

#include <algorithm>
#include <cmath>

namespace mtt_control
{

class SlewRateLimiter
{
public:
  SlewRateLimiter() = default;
  SlewRateLimiter(double rise_rate, double fall_rate)
  : rise_rate_(rise_rate), fall_rate_(fall_rate) {}

  void set_limits(double rise_rate, double fall_rate)
  {
    rise_rate_ = rise_rate;
    fall_rate_ = fall_rate;
  }

  double update(double input, double dt)
  {
    if (!initialized_) {
      value_ = input;
      initialized_ = true;
      return value_;
    }

    const double delta = input - value_;
    // Magnitude-based: rise_rate when speeding up in the same direction,
    // fall_rate when slowing down OR reversing direction.
    // IMPORTANT: direction reversal must use fall_rate (fast decel) throughout
    // the crossing-zero phase, not rise_rate — relay flip under load is dangerous.
    const bool same_dir = (input >= 0.0) == (value_ >= 0.0);
    const bool accelerating = same_dir && std::abs(input) > std::abs(value_);
    const double rate = accelerating ? rise_rate_ : fall_rate_;
    const double max_change = std::max(0.0, rate) * dt;

    if (std::abs(delta) <= max_change) {
      value_ = input;
    } else {
      value_ += std::copysign(max_change, delta);
    }
    return value_;
  }

  void reset(double value = 0.0)
  {
    value_ = value;
    initialized_ = true;
  }

  double value() const { return value_; }

private:
  double rise_rate_{0.0};
  double fall_rate_{0.0};
  double value_{0.0};
  bool initialized_{false};
};

}  // namespace mtt_control
