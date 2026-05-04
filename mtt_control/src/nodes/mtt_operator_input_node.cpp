#include "mtt_control/nodes/mtt_operator_input_node.hpp"

#include <algorithm>
#include <cmath>

namespace mtt_control
{

MttOperatorInputNode::MttOperatorInputNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("mtt_operator_input_node", options)
{
  max_linear_speed_ = declare_parameter("max_linear_speed", 1.0);
  max_angular_command_ = declare_parameter("max_angular_command", 0.6);
  linear_deadband_ = declare_parameter("linear_deadband", 0.05);
  angular_deadband_ = declare_parameter("angular_deadband", 0.08);
  linear_expo_ = declare_parameter("linear_expo", 1.2);
  angular_expo_ = declare_parameter("angular_expo", 1.6);
  manual_activity_linear_threshold_ = declare_parameter("manual_activity_linear_threshold", 0.05);
  manual_activity_angular_threshold_ = declare_parameter("manual_activity_angular_threshold", 0.05);
  estop_trigger_threshold_ = declare_parameter("estop_trigger_threshold", -0.10);
  brake_axis_default_ = declare_parameter("brake_axis_default", 1.0);
  deadman_button_index_ = declare_parameter("deadman_button_index", 5);
  light_button_index_ = declare_parameter("light_button_index", 2);
  linear_axis_index_ = declare_parameter("linear_axis_index", 1);
  angular_axis_index_ = declare_parameter("angular_axis_index", 3);
  brake_axis_index_ = declare_parameter("brake_axis_index", 5);
  estop_left_trigger_axis_ = declare_parameter("estop_left_trigger_axis", 2);
  estop_right_trigger_axis_ = declare_parameter("estop_right_trigger_axis", 5);
  invert_linear_axis_ = declare_parameter("invert_linear_axis", true);
  invert_angular_axis_ = declare_parameter("invert_angular_axis", false);
  enable_brake_axis_ = declare_parameter("enable_brake_axis", true);

  joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
    "joy",
    rclcpp::SensorDataQoS(),
    std::bind(&MttOperatorInputNode::on_joy, this, std::placeholders::_1));

  manual_raw_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel/manual_raw", 20);
  aux_pub_ = create_publisher<mtt_msgs::msg::MttAuxCommand>("mtt_aux_cmd", 20);
  deadman_pub_ = create_publisher<std_msgs::msg::Bool>("teleop_deadman", 20);
  estop_pub_ = create_publisher<std_msgs::msg::Bool>("teleop_estop", 20);
  manual_activity_pub_ = create_publisher<std_msgs::msg::Bool>("mtt_control/manual_activity", 20);
}

bool MttOperatorInputNode::trigger_pressed(const JoystickState & state, int axis_index) const
{
  return state.axis_value(static_cast<std::size_t>(axis_index), 1.0F) < estop_trigger_threshold_;
}

void MttOperatorInputNode::on_joy(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  joystick_state_.update(*msg);

  const bool deadman_pressed = joystick_state_.button_pressed(static_cast<std::size_t>(deadman_button_index_));
  const bool deadman_released = previous_deadman_pressed_ && !deadman_pressed;
  const bool light_rising = joystick_state_.button_rising(static_cast<std::size_t>(light_button_index_));
  if (light_rising) {
    light_state_ = !light_state_;
  }

  float linear_axis = joystick_state_.axis_value(static_cast<std::size_t>(linear_axis_index_));
  if (invert_linear_axis_) {
    linear_axis = -linear_axis;
  }
  linear_axis = static_cast<float>(JoystickState::shape_axis(linear_axis, linear_deadband_, linear_expo_));

  float angular_axis = joystick_state_.axis_value(static_cast<std::size_t>(angular_axis_index_));
  if (invert_angular_axis_) {
    angular_axis = -angular_axis;
  }
  angular_axis = static_cast<float>(JoystickState::shape_axis(angular_axis, angular_deadband_, angular_expo_));

  const bool estop_active =
    trigger_pressed(joystick_state_, estop_left_trigger_axis_) &&
    trigger_pressed(joystick_state_, estop_right_trigger_axis_);

  const bool manual_activity =
    deadman_pressed &&
    !estop_active &&
    (std::abs(max_linear_speed_ * linear_axis) > manual_activity_linear_threshold_ ||
     std::abs(max_angular_command_ * angular_axis) > manual_activity_angular_threshold_);

  auto deadman_msg = std_msgs::msg::Bool();
  deadman_msg.data = deadman_pressed;
  deadman_pub_->publish(deadman_msg);

  auto estop_msg = std_msgs::msg::Bool();
  estop_msg.data = estop_active;
  estop_pub_->publish(estop_msg);

  auto activity_msg = std_msgs::msg::Bool();
  activity_msg.data = manual_activity;
  manual_activity_pub_->publish(activity_msg);

  mtt_msgs::msg::MttAuxCommand aux_msg;
  aux_msg.light_state = light_state_;
  if (enable_brake_axis_ && !estop_active) {
    const float brake_axis = joystick_state_.axis_value(
      static_cast<std::size_t>(brake_axis_index_),
      static_cast<float>(brake_axis_default_));
    aux_msg.brake = std::clamp((-brake_axis + 1.0F) / 2.0F, 0.0F, 1.0F);
  } else {
    aux_msg.brake = 0.0F;
  }
  aux_pub_->publish(aux_msg);

  geometry_msgs::msg::TwistStamped manual_msg;
  manual_msg.header.stamp = now();
  if (deadman_pressed && !estop_active) {
    manual_msg.twist.linear.x = max_linear_speed_ * linear_axis;
    manual_msg.twist.angular.z = max_angular_command_ * angular_axis;
  }

  if (deadman_pressed || deadman_released || estop_active) {
    manual_raw_pub_->publish(manual_msg);
  }

  previous_deadman_pressed_ = deadman_pressed;
}

}  // namespace mtt_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mtt_control::MttOperatorInputNode>());
  rclcpp::shutdown();
  return 0;
}
