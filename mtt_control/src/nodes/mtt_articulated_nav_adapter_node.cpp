#include "mtt_control/nodes/mtt_articulated_nav_adapter_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>

namespace mtt_control
{

MttArticulatedNavAdapterNode::MttArticulatedNavAdapterNode(
  const rclcpp::NodeOptions & options)
: rclcpp::Node("mtt_articulated_nav_adapter_node", options),
  model_([this]() {
      logic::ArticulatedCommandParams params;
      params.equivalent_length_m = declare_parameter("equivalent_length_m", 2.40);
      params.max_articulation_rad = declare_parameter("max_articulation_rad", 0.733);
      params.min_turn_speed_ms = declare_parameter("min_turn_speed_ms", 0.05);
      params.max_articulation_rate_rad_s = declare_parameter(
      "max_articulation_rate_rad_s", 0.50);
      params.use_slip_heuristic = declare_parameter("use_slip_heuristic", true);
      params.yaw_slip_base = declare_parameter("yaw_slip_base", 0.10);
      params.yaw_slip_speed_gain = declare_parameter("yaw_slip_speed_gain", 0.05);
      params.yaw_slip_articulation_gain = declare_parameter(
      "yaw_slip_articulation_gain", 0.15);
      params.yaw_slip_min_scale = declare_parameter("yaw_slip_min_scale", 0.55);
      return params;
    }())
{
  input_timeout_s_ = declare_parameter("input_timeout_s", 0.25);
  feedback_timeout_s_ = declare_parameter("feedback_timeout_s", 0.30);
  publish_rate_hz_ = declare_parameter("publish_rate_hz", 20.0);
  require_fresh_articulation_ = declare_parameter("require_fresh_articulation", true);
  const auto input_topic = declare_parameter(
    "input_topic", std::string("/mtt_avoidance/nav_cmd_guarded"));
  const auto output_topic = declare_parameter(
    "output_topic", std::string("/mtt_avoidance/nav_articulated_cmd"));
  const auto articulation_topic = declare_parameter(
    "articulation_feedback_topic", std::string("/hardware/articulation_angle"));

  const auto now_stamp = now();
  input_stamp_ = now_stamp;
  feedback_stamp_ = now_stamp;
  previous_publish_stamp_ = now_stamp;
  nav_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    input_topic, 10,
    std::bind(
      &MttArticulatedNavAdapterNode::on_nav_command, this, std::placeholders::_1));
  articulation_sub_ = create_subscription<std_msgs::msg::Float64>(
    articulation_topic, rclcpp::SensorDataQoS(),
    std::bind(
      &MttArticulatedNavAdapterNode::on_articulation, this, std::placeholders::_1));
  command_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(output_topic, 10);
  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_)),
    std::bind(&MttArticulatedNavAdapterNode::on_timer, this));

  RCLCPP_INFO(
    get_logger(),
    "MTT articulated Nav adapter: %s -> %s; signed forward/reverse, feedback=%s",
    input_topic.c_str(), output_topic.c_str(), articulation_topic.c_str());
}

void MttArticulatedNavAdapterNode::on_nav_command(
  const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (!std::isfinite(msg->linear.x) || !std::isfinite(msg->angular.z)) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  speed_ms_ = msg->linear.x;
  yaw_rate_rad_s_ = msg->angular.z;
  input_stamp_ = now();
  has_input_ = true;
}

void MttArticulatedNavAdapterNode::on_articulation(
  const std_msgs::msg::Float64::SharedPtr msg)
{
  if (!std::isfinite(msg->data)) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  measured_articulation_rad_ = msg->data;
  feedback_stamp_ = now();
  has_feedback_ = true;
}

void MttArticulatedNavAdapterNode::on_timer()
{
  const auto stamp = now();
  geometry_msgs::msg::TwistStamped output;
  output.header.stamp = stamp;
  output.header.frame_id = "base_footprint";

  std::lock_guard<std::mutex> lock(mutex_);
  const bool input_fresh = has_input_ &&
    (stamp - input_stamp_).seconds() <= input_timeout_s_;
  const bool feedback_fresh = has_feedback_ &&
    (stamp - feedback_stamp_).seconds() <= feedback_timeout_s_;
  const double dt_s = std::clamp(
    (stamp - previous_publish_stamp_).seconds(), 0.0, 1.0);
  previous_publish_stamp_ = stamp;

  if (!input_fresh || (require_fresh_articulation_ && !feedback_fresh)) {
    const double hold = feedback_fresh ?
      measured_articulation_rad_ : previous_command_rad_;
    output.twist.angular.z = model_.normalized(hold);
    command_pub_->publish(output);
    return;
  }

  const double desired = model_.articulation_from_yaw_rate(
    speed_ms_, yaw_rate_rad_s_);
  const double reference = feedback_fresh ?
    measured_articulation_rad_ : previous_command_rad_;
  const double limited = model_.rate_limited_articulation(desired, reference, dt_s);
  previous_command_rad_ = limited;
  output.twist.linear.x = speed_ms_;
  output.twist.angular.z = model_.normalized(limited);
  command_pub_->publish(output);
}

}  // namespace mtt_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mtt_control::MttArticulatedNavAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
