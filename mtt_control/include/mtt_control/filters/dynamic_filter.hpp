#pragma once

#include <algorithm>
#include <cmath>

namespace mtt_control
{

class DynamicFilter
{
public:
  DynamicFilter() = default;
  DynamicFilter(double omega_n, double zeta)
  : omega_n_(omega_n), zeta_(zeta) {}

  void configure(double omega_n, double zeta)
  {
    omega_n_ = omega_n;
    zeta_ = zeta;
  }

  double update(double input, double dt)
  {
    if (!initialized_) {
      reset(input);
      return output_;
    }

    if (dt <= 1e-6 || omega_n_ <= 1e-6) {
      output_ = input;
      velocity_ = 0.0;
      return output_;
    }

    const double acceleration =
      omega_n_ * omega_n_ * (input - output_) - 2.0 * zeta_ * omega_n_ * velocity_;
    velocity_ += acceleration * dt;
    output_ += velocity_ * dt;
    return output_;
  }

  void reset(double value = 0.0)
  {
    output_ = value;
    velocity_ = 0.0;
    initialized_ = true;
  }

  double output() const { return output_; }

private:
  double omega_n_{0.0};
  double zeta_{1.0};
  double output_{0.0};
  double velocity_{0.0};
  bool initialized_{false};
};

}  // namespace mtt_control
