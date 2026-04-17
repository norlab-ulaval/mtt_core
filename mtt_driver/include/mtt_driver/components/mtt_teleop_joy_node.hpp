#ifndef MTT_DRIVER__COMPONENTS__MTT_TELEOP_JOY_NODE_HPP_
#define MTT_DRIVER__COMPONENTS__MTT_TELEOP_JOY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "mtt_msgs/msg/mtt_aux_command.hpp"

namespace mtt
{

class MttTeleopJoyNode : public rclcpp::Node
{
public:
  explicit MttTeleopJoyNode(const rclcpp::NodeOptions & options);

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

  bool button_pressed(const sensor_msgs::msg::Joy::SharedPtr msg, size_t index) const;
  float axis_value(const sensor_msgs::msg::Joy::SharedPtr msg, size_t index, float default_val = 0.0) const;

  double max_linear_speed_;
  double max_angular_speed_;
  int deadman_button_index_;
  int light_button_index_;
  int linear_axis_index_;
  int angular_axis_index_;
  int brake_axis_index_;
  bool invert_linear_axis_;
  bool invert_angular_axis_;
  bool enable_brake_axis_;
  double brake_axis_default_;
  
  bool light_state_;
  int prev_light_btn_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<mtt_msgs::msg::MttAuxCommand>::SharedPtr aux_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
};

}  // namespace mtt

#endif  // MTT_DRIVER__COMPONENTS__MTT_TELEOP_JOY_NODE_HPP_
