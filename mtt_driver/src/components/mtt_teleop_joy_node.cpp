#include "mtt_driver/components/mtt_teleop_joy_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <algorithm>
#include <memory>
#include <utility>

namespace mtt
{

MttTeleopJoyNode::MttTeleopJoyNode(const rclcpp::NodeOptions & options)
: Node("mtt_teleop_joy_node", options),
  light_state_(false),
  prev_light_btn_(0)
{
  max_linear_speed_ = this->declare_parameter("max_linear_speed", 0.6);
  max_angular_speed_ = this->declare_parameter("max_angular_speed", 1.0);
  deadman_button_index_ = this->declare_parameter("deadman_button_index", 5);
  light_button_index_ = this->declare_parameter("light_button_index", 2);
  linear_axis_index_ = this->declare_parameter("linear_axis_index", 1);
  angular_axis_index_ = this->declare_parameter("angular_axis_index", 3);
  brake_axis_index_ = this->declare_parameter("brake_axis_index", 5);
  invert_linear_axis_ = this->declare_parameter("invert_linear_axis", true);
  invert_angular_axis_ = this->declare_parameter("invert_angular_axis", false);
  enable_brake_axis_ = this->declare_parameter("enable_brake_axis", false);
  brake_axis_default_ = this->declare_parameter("brake_axis_default", 1.0);

  // We publish on cmd_vel_raw typically
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel_raw", 10);
  aux_cmd_pub_ = this->create_publisher<mtt_msgs::msg::MttAuxCommand>("mtt_aux_cmd", 10);
  estop_pub_ = this->create_publisher<std_msgs::msg::Bool>("teleop_estop", 10);

  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "joy", 10, std::bind(&MttTeleopJoyNode::joy_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "MTT Teleop C++ Node started.");
}

bool MttTeleopJoyNode::button_pressed(const sensor_msgs::msg::Joy::SharedPtr msg, size_t index) const
{
  if (index < msg->buttons.size()) {
    return msg->buttons[index] > 0;
  }
  return false;
}

float MttTeleopJoyNode::axis_value(const sensor_msgs::msg::Joy::SharedPtr msg, size_t index, float default_val) const
{
  if (index < msg->axes.size()) {
    return msg->axes[index];
  }
  return default_val;
}

void MttTeleopJoyNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  bool deadman_pressed = button_pressed(msg, static_cast<size_t>(deadman_button_index_));
  bool light_btn = button_pressed(msg, static_cast<size_t>(light_button_index_));

  float linear_axis = axis_value(msg, static_cast<size_t>(linear_axis_index_));
  if (invert_linear_axis_) {
    linear_axis = -linear_axis;
  }

  float angular_axis = axis_value(msg, static_cast<size_t>(angular_axis_index_));
  if (invert_angular_axis_) {
    angular_axis = -angular_axis;
  }

  float brake_axis = axis_value(
    msg,
    static_cast<size_t>(brake_axis_index_),
    static_cast<float>(brake_axis_default_));

  if (light_btn && prev_light_btn_ == 0) {
    light_state_ = !light_state_;
  }
  prev_light_btn_ = light_btn ? 1 : 0;

  auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
  auto aux_msg = std::make_unique<mtt_msgs::msg::MttAuxCommand>();
  auto estop_msg = std::make_unique<std_msgs::msg::Bool>();

  estop_msg->data = !deadman_pressed;
  twist_msg->header.stamp = this->now();
  aux_msg->light_state = light_state_;

  if (deadman_pressed) {
    twist_msg->twist.linear.x = max_linear_speed_ * linear_axis;
    twist_msg->twist.angular.z = max_angular_speed_ * angular_axis;
    // Keep brake released by default so motion is controlled by cmd_vel only.
    // Brake can be re-enabled explicitly if the hardware needs it.
    aux_msg->brake = enable_brake_axis_
      ? std::clamp((-brake_axis + 1.0f) / 2.0f, 0.0f, 1.0f)
      : 0.0;
  } else {
    twist_msg->twist.linear.x = 0.0;
    twist_msg->twist.angular.z = 0.0;
    aux_msg->brake = 0.0;
  }

  estop_pub_->publish(std::move(estop_msg));
  cmd_vel_pub_->publish(std::move(twist_msg));
  aux_cmd_pub_->publish(std::move(aux_msg));
}

}  // namespace mtt

RCLCPP_COMPONENTS_REGISTER_NODE(mtt::MttTeleopJoyNode)
