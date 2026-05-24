#pragma once

#include <cstddef>
#include <vector>

#include <sensor_msgs/msg/joy.hpp>

namespace mtt_control
{

class JoystickState
{
public:
  void update(const sensor_msgs::msg::Joy & joy);

  bool button_pressed(std::size_t index) const;
  bool button_rising(std::size_t index) const;
  float axis_value(std::size_t index, float default_value = 0.0F) const;

  static double shape_axis(double value, double deadband, double expo);

  const sensor_msgs::msg::Joy & last_msg() const { return current_msg_; }

private:
  sensor_msgs::msg::Joy current_msg_;
  std::vector<int> previous_buttons_;
  std::vector<int> current_buttons_;
  std::vector<float> current_axes_;
};

}  // namespace mtt_control
