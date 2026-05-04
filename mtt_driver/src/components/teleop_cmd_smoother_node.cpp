#include "mtt_driver/components/teleop_cmd_smoother_node.hpp"
#include <cmath>
#include <rclcpp_components/register_node_macro.hpp>

namespace mtt
{

TeleopCmdSmootherNode::TeleopCmdSmootherNode(const rclcpp::NodeOptions & options)
: Node("teleop_cmd_smoother_node", options)
{
  input_topic_ = this->declare_parameter("input_topic", "cmd_vel/teleop_raw");
  output_topic_ = this->declare_parameter("output_topic", "cmd_vel/teleop");
  input_timeout_ = this->declare_parameter("input_timeout", 0.5);
  rate_hz_ = this->declare_parameter("rate_hz", 50.0);
  max_accel_linear_ = this->declare_parameter("max_accel_linear", 1.5);
  max_accel_angular_ = this->declare_parameter("max_accel_angular", 1.5);

  last_input_ = this->now();
  last_update_ = this->now();

  pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(output_topic_, 10);
  sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    input_topic_, 10, std::bind(&TeleopCmdSmootherNode::input_callback, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / rate_hz_),
    std::bind(&TeleopCmdSmootherNode::publish_smoothed_cmd, this));
}

double TeleopCmdSmootherNode::step_towards(double current, double target, double max_delta) const
{
  if (target > current + max_delta) {
    return current + max_delta;
  }
  if (target < current - max_delta) {
    return current - max_delta;
  }
  return target;
}

void TeleopCmdSmootherNode::input_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  target_.linear_x = msg->twist.linear.x;
  target_.angular_z = msg->twist.angular.z;
  last_input_ = this->now();
}

void TeleopCmdSmootherNode::publish_smoothed_cmd()
{
  auto now = this->now();
  double dt = (now - last_update_).seconds();
  if (dt <= 1e-6) {
    dt = 1.0 / rate_hz_;
  }
  last_update_ = now;

  double age = (now - last_input_).seconds();
  VelocityState effective_target;
  
  if (age > input_timeout_) {
    // Decay to zero if timeout exceeded
    effective_target.linear_x = 0.0;
    effective_target.angular_z = 0.0;
  } else {
    effective_target = target_;
  }

  current_.linear_x = step_towards(current_.linear_x, effective_target.linear_x, max_accel_linear_ * dt);
  current_.angular_z = step_towards(current_.angular_z, effective_target.angular_z, max_accel_angular_ * dt);

  const bool target_is_zero =
    std::abs(target_.linear_x) < zero_epsilon_ &&
    std::abs(target_.angular_z) < zero_epsilon_;
  const bool current_is_zero =
    std::abs(current_.linear_x) < zero_epsilon_ &&
    std::abs(current_.angular_z) < zero_epsilon_;

  if (target_is_zero && current_is_zero) {
    return;
  }

  auto msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
  msg->header.stamp = now;
  msg->twist.linear.x = current_.linear_x;
  msg->twist.angular.z = current_.angular_z;
  pub_->publish(std::move(msg));
}

}  // namespace mtt

RCLCPP_COMPONENTS_REGISTER_NODE(mtt::TeleopCmdSmootherNode)
