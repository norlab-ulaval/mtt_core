#include "mtt_control/nodes/mtt_articulated_command_dispatcher_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>

namespace mtt_control
{

MttArticulatedCommandDispatcherNode::MttArticulatedCommandDispatcherNode(
  const rclcpp::NodeOptions & options)
: rclcpp::Node("mtt_articulated_command_dispatcher_node", options),
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
  const bool enabled = declare_parameter("enabled", false);
  const bool shadow_mode = declare_parameter("shadow_mode", true);
  const bool hardware_output_enabled = declare_parameter("hardware_output_enabled", false);
  input_timeout_s_ = declare_parameter("input_timeout_s", 0.25);
  feedback_timeout_s_ = declare_parameter("feedback_timeout_s", 0.30);
  publish_rate_hz_ = declare_parameter("publish_rate_hz", 20.0);
  const auto input_topic = declare_parameter(
    "input_topic", std::string("/mtt_avoidance/selected_articulated_cmd"));
  const auto speed_topic = declare_parameter(
    "speed_output_topic", std::string("/mtt_avoidance/speed_cmd_preview"));
  const auto articulation_output_topic = declare_parameter(
    "articulation_output_topic",
    std::string("/mtt_avoidance/articulation_setpoint_preview"));
  const auto feedback_topic = declare_parameter(
    "articulation_feedback_topic", std::string("/hardware/articulation_angle"));

  outputs_are_hardware_ = is_hardware_topic(speed_topic) ||
    is_hardware_topic(articulation_output_topic);
  active_ = enabled && !shadow_mode && hardware_output_enabled;
  if (outputs_are_hardware_ && !active_) {
    RCLCPP_ERROR(
      get_logger(),
      "Hardware output topics requested while dispatcher is locked; no command will publish");
  }

  const auto stamp = now();
  command_stamp_ = stamp;
  feedback_stamp_ = stamp;
  previous_publish_stamp_ = stamp;
  command_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    input_topic, 10,
    std::bind(
      &MttArticulatedCommandDispatcherNode::on_command, this,
      std::placeholders::_1));
  articulation_sub_ = create_subscription<std_msgs::msg::Float64>(
    feedback_topic, rclcpp::SensorDataQoS(),
    std::bind(
      &MttArticulatedCommandDispatcherNode::on_articulation, this,
      std::placeholders::_1));
  speed_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(speed_topic, 10);
  articulation_pub_ = create_publisher<std_msgs::msg::Float64>(
    articulation_output_topic, 10);
  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_)),
    std::bind(&MttArticulatedCommandDispatcherNode::on_timer, this));

  RCLCPP_WARN(
    get_logger(),
    "Articulated dispatcher: active=%s shadow=%s hardware_gate=%s outputs=(%s, %s)",
    active_ ? "true" : "false", shadow_mode ? "true" : "false",
    hardware_output_enabled ? "true" : "false", speed_topic.c_str(),
    articulation_output_topic.c_str());
}

bool MttArticulatedCommandDispatcherNode::is_hardware_topic(const std::string & topic)
{
  return topic == "/controller/cmd_vel" || topic == "controller/cmd_vel" ||
         topic == "/mtt_articulation_setpoint" ||
         topic == "mtt_articulation_setpoint";
}

void MttArticulatedCommandDispatcherNode::on_command(
  const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  if (!std::isfinite(msg->twist.linear.x) || !std::isfinite(msg->twist.angular.z)) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  last_command_ = *msg;
  command_stamp_ = now();
  has_command_ = true;
}

void MttArticulatedCommandDispatcherNode::on_articulation(
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

void MttArticulatedCommandDispatcherNode::on_timer()
{
  if (outputs_are_hardware_ && !active_) {
    return;
  }

  const auto stamp = now();
  geometry_msgs::msg::TwistStamped speed_output;
  speed_output.header.stamp = stamp;
  speed_output.header.frame_id = "base_footprint";
  std_msgs::msg::Float64 articulation_output;

  std::lock_guard<std::mutex> lock(mutex_);
  const bool command_fresh = has_command_ &&
    (stamp - command_stamp_).seconds() <= input_timeout_s_;
  const bool feedback_fresh = has_feedback_ &&
    (stamp - feedback_stamp_).seconds() <= feedback_timeout_s_;
  const double dt_s = std::clamp(
    (stamp - previous_publish_stamp_).seconds(), 0.0, 1.0);
  previous_publish_stamp_ = stamp;

  if (active_ && !feedback_fresh) {
    speed_pub_->publish(speed_output);
    return;
  }

  const double reference = feedback_fresh ?
    measured_articulation_rad_ : previous_setpoint_rad_;
  double desired = reference;
  if (command_fresh) {
    desired = model_.denormalized(last_command_.twist.angular.z);
    speed_output.twist.linear.x = last_command_.twist.linear.x;
  }
  const double limited = model_.rate_limited_articulation(desired, reference, dt_s);
  previous_setpoint_rad_ = limited;
  articulation_output.data = limited;
  speed_pub_->publish(speed_output);
  articulation_pub_->publish(articulation_output);
}

}  // namespace mtt_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<mtt_control::MttArticulatedCommandDispatcherNode>());
  rclcpp::shutdown();
  return 0;
}
