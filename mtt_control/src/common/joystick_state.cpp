#include "mtt_control/common/joystick_state.hpp"

#include <algorithm>
#include <cmath>

namespace mtt_control
{

void JoystickState::update(const sensor_msgs::msg::Joy & joy)
{
  current_msg_ = joy;
  previous_buttons_ = current_buttons_;
  current_buttons_ = joy.buttons;
  current_axes_ = joy.axes;
}

bool JoystickState::button_pressed(std::size_t index) const
{
  return index < current_buttons_.size() && current_buttons_[index] > 0;
}

bool JoystickState::button_rising(std::size_t index) const
{
  const bool current = button_pressed(index);
  const bool previous = index < previous_buttons_.size() && previous_buttons_[index] > 0;
  return current && !previous;
}

float JoystickState::axis_value(std::size_t index, float default_value) const
{
  if (index < current_axes_.size()) {
    return current_axes_[index];
  }
  return default_value;
}

double JoystickState::shape_axis(double value, double deadband, double expo)
{
  const double clipped = std::clamp(value, -1.0, 1.0);
  const double magnitude = std::abs(clipped);
  if (magnitude <= deadband) {
    return 0.0;
  }

  const double normalized = (magnitude - deadband) / std::max(1.0 - deadband, 1e-6);
  const double curved = expo > 0.0 ? std::pow(normalized, expo) : normalized;
  return std::copysign(std::clamp(curved, 0.0, 1.0), clipped);
}

}  // namespace mtt_control
