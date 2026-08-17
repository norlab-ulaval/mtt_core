#include "mtt_control/nodes/mtt_cmd_vel_bridge_node.hpp"

#include <algorithm>
#include <cmath>

namespace mtt_control
{

MttCmdVelBridgeNode::MttCmdVelBridgeNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("mtt_cmd_vel_bridge_node", options)
{
  const auto input_topic  = declare_parameter("input_topic",  std::string("nav_cmd_vel"));
  const auto output_topic = declare_parameter("output_topic", std::string("controller/cmd_vel"));
  wheelbase_m_           = declare_parameter("wheelbase_m",           2.4);
  max_articulation_rad_  = declare_parameter("max_articulation_rad",  0.733);
  min_speed_deadband_ms_ = declare_parameter("min_speed_deadband_ms", 0.05);
  timeout_s_             = declare_parameter("timeout_s",             0.5);
  publish_rate_hz_       = declare_parameter("publish_rate_hz",       20.0);

  last_input_time_ = now();

  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    input_topic, 10,
    std::bind(&MttCmdVelBridgeNode::on_cmd_vel, this, std::placeholders::_1));

  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(output_topic, 10);

  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_)),
    std::bind(&MttCmdVelBridgeNode::on_timer, this));

  RCLCPP_INFO(
    get_logger(),
    "MttCmdVelBridgeNode: %s → %s  (L=%.2f m, psi_max=%.4f rad, deadband=%.3f m/s)",
    input_topic.c_str(), output_topic.c_str(),
    wheelbase_m_, max_articulation_rad_, min_speed_deadband_ms_);
}

void MttCmdVelBridgeNode::on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  last_v_     = msg->linear.x;
  last_omega_ = msg->angular.z;
  last_input_time_ = now();
  has_input_  = true;
}

void MttCmdVelBridgeNode::on_timer()
{
  const auto now_stamp = now();

  // No valid input or timeout → safe zero
  const bool timed_out =
    !has_input_ || (now_stamp - last_input_time_).seconds() > timeout_s_;

  if (timed_out) {
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = now_stamp;
    cmd_vel_pub_->publish(msg);
    return;
  }

  const double v     = last_v_;
  const double omega = last_omega_;

  double linear_out  = 0.0;
  double angular_out = 0.0;  // normalized articulation in [-1, 1]

  if (std::abs(v) >= min_speed_deadband_ms_) {
    // psi = atan(L * omega / v)
    // Works for forward (v>0) and reverse (v<0):
    //   forward  + left turn  (omega>0) → psi>0 (left articulation)
    //   reverse  + left turn  (omega>0) → psi<0 (steer right to go left)
    const double psi_rad    = std::atan(wheelbase_m_ * omega / v);
    const double psi_clamped = std::clamp(psi_rad, -max_articulation_rad_, max_articulation_rad_);
    linear_out  = v;
    angular_out = psi_clamped / max_articulation_rad_;
  }
  // else: |v| < deadband → keep linear_out=0, angular_out=0

  geometry_msgs::msg::TwistStamped msg;
  msg.header.stamp    = now_stamp;
  msg.twist.linear.x  = linear_out;
  msg.twist.angular.z = angular_out;
  cmd_vel_pub_->publish(msg);
}

}  // namespace mtt_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mtt_control::MttCmdVelBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
