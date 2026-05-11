#pragma once

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mtt_msgs/msg/mtt_aux_command.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <mtt_interfaces/srv/set_steer_control_mode.hpp>

#include "mtt_control/common/joystick_state.hpp"

namespace mtt_control
{

class MttOperatorInputNode : public rclcpp::Node
{
public:
  explicit MttOperatorInputNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_joy(const sensor_msgs::msg::Joy::SharedPtr msg);
  bool trigger_pressed(const JoystickState & state, int axis_index) const;
  void publish_articulation_hold_state(bool hold_active);
  void toggle_steering_mode();

  double max_linear_speed_{1.0};
  double max_angular_command_{0.6};
  double linear_deadband_{0.05};
  double angular_deadband_{0.08};
  double linear_expo_{1.2};
  double angular_expo_{1.6};
  double manual_activity_linear_threshold_{0.05};
  double manual_activity_angular_threshold_{0.05};
  double estop_trigger_threshold_{-0.10};
  double brake_axis_default_{1.0};
  double articulation_hold_max_speed_ms_{0.25};
  double articulation_hold_release_deadband_{0.08};
  int deadman_button_index_{5};
  int light_button_index_{2};
  int articulation_hold_button_index_{6};
  int steer_mode_switch_button_index_{4};
  int linear_axis_index_{1};
  int angular_axis_index_{3};
  int brake_axis_index_{5};
  int estop_left_trigger_axis_{2};
  int estop_right_trigger_axis_{5};
  bool invert_linear_axis_{true};
  bool invert_angular_axis_{false};
  bool enable_brake_axis_{true};
  bool articulation_hold_enabled_{true};
  bool articulation_hold_reset_on_deadman_release_{true};
  bool enable_steering_mode_switch_{true};

  bool light_state_{false};
  bool previous_deadman_pressed_{false};
  bool articulation_hold_mode_{false};
  bool has_held_angular_command_{false};
  double held_angular_command_{0.0};
  std::string current_steer_mode_{"closed_loop"};

  JoystickState joystick_state_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr manual_raw_pub_;
  rclcpp::Publisher<mtt_msgs::msg::MttAuxCommand>::SharedPtr aux_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr deadman_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr manual_activity_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr articulation_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr articulation_hold_active_pub_;

  rclcpp::Client<mtt_interfaces::srv::SetSteerControlMode>::SharedPtr can_steer_mode_client_;
  rclcpp::Client<mtt_interfaces::srv::SetSteerControlMode>::SharedPtr odom_steer_mode_client_;
};

}  // namespace mtt_control
