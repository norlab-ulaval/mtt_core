#pragma once

#include <algorithm>

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

    const double max_up = std::max(0.0, rise_rate_) * dt;
    const double max_down = std::max(0.0, fall_rate_) * dt;
    const double delta = input - value_;

    if (delta > max_up) {
      value_ += max_up;
    } else if (delta < -max_down) {
      value_ -= max_down;
    } else {
      value_ = input;
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
