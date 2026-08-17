#include "mtt_control/nodes/mtt_autonomy_mux_node.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <functional>

namespace mtt_control
{

MttAutonomyMuxNode::MttAutonomyMuxNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("mtt_autonomy_mux_node", options)
{
  const auto wiln_command_topic = declare_parameter(
    "wiln_cmd_vel_topic", std::string("/mtt_control/auto/wiln/cmd_vel"));
  const auto wiln_articulation_topic = declare_parameter(
    "wiln_articulation_topic",
    std::string("/mtt_control/auto/wiln/articulation_setpoint"));
  const auto gps_command_topic = declare_parameter(
    "gps_cmd_vel_topic", std::string("/mtt_control/auto/gps/cmd_vel"));
  const auto gps_articulation_topic = declare_parameter(
    "gps_articulation_topic",
    std::string("/mtt_control/auto/gps/articulation_setpoint"));
  const auto experiment_command_topic = declare_parameter(
    "experiment_cmd_vel_topic", std::string("/mtt_control/auto/experiment/cmd_vel"));
  const auto experiment_articulation_topic = declare_parameter(
    "experiment_articulation_topic",
    std::string("/mtt_control/auto/experiment/articulation_setpoint"));
  const auto output_command_topic = declare_parameter(
    "output_cmd_vel_topic", std::string("controller/cmd_vel"));
  const auto output_articulation_topic = declare_parameter(
    "output_articulation_topic", std::string("/mtt_articulation_setpoint"));
  const auto request_topic = declare_parameter(
    "request_topic", std::string("/mtt_control/autonomy/request"));
  const auto release_topic = declare_parameter(
    "release_topic", std::string("/mtt_control/autonomy/release"));
  const auto selected_topic = declare_parameter(
    "selected_topic", std::string("/mtt_control/autonomy/selected"));
  input_timeout_s_ = declare_parameter("input_timeout_s", 0.5);
  max_pair_skew_s_ = declare_parameter("max_pair_skew_s", 0.15);
  publish_rate_hz_ = declare_parameter("publish_rate_hz", 50.0);

  const auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  const auto selected_qos = rclcpp::QoS(1).reliable().transient_local();

  wiln_command_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    wiln_command_topic, reliable_qos,
    [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
      on_command(msg, wiln_);
    });
  wiln_articulation_sub_ = create_subscription<std_msgs::msg::Float64>(
    wiln_articulation_topic, reliable_qos,
    [this](const std_msgs::msg::Float64::SharedPtr msg) {
      on_articulation(msg, wiln_);
    });
  gps_command_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    gps_command_topic, reliable_qos,
    [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
      on_command(msg, gps_);
    });
  gps_articulation_sub_ = create_subscription<std_msgs::msg::Float64>(
    gps_articulation_topic, reliable_qos,
    [this](const std_msgs::msg::Float64::SharedPtr msg) {
      on_articulation(msg, gps_);
    });
  experiment_command_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    experiment_command_topic, reliable_qos,
    [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
      on_command(msg, experiment_);
    });
  experiment_articulation_sub_ = create_subscription<std_msgs::msg::Float64>(
    experiment_articulation_topic, reliable_qos,
    [this](const std_msgs::msg::Float64::SharedPtr msg) {
      on_articulation(msg, experiment_);
    });
  request_sub_ = create_subscription<std_msgs::msg::String>(
    request_topic, reliable_qos,
    std::bind(&MttAutonomyMuxNode::on_request, this, std::placeholders::_1));
  release_sub_ = create_subscription<std_msgs::msg::String>(
    release_topic, reliable_qos,
    std::bind(&MttAutonomyMuxNode::on_release, this, std::placeholders::_1));

  command_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
    output_command_topic, reliable_qos);
  articulation_pub_ = create_publisher<std_msgs::msg::Float64>(
    output_articulation_topic, reliable_qos);
  selected_pub_ = create_publisher<std_msgs::msg::String>(selected_topic, selected_qos);
  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_)),
    std::bind(&MttAutonomyMuxNode::on_timer, this));

  std_msgs::msg::String selected;
  selected.data = owner_;
  selected_pub_->publish(selected);
  RCLCPP_INFO(get_logger(), "Autonomy mux ready: selected=%s", owner_.c_str());
}

std::string MttAutonomyMuxNode::normalized_owner(const std::string & owner)
{
  std::string value = owner;
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](const unsigned char c) {return static_cast<char>(std::tolower(c));});
  return value;
}

void MttAutonomyMuxNode::on_request(const std_msgs::msg::String::SharedPtr msg)
{
  const auto requested = normalized_owner(msg->data);
  if (requested != "wiln" && requested != "gps" && requested != "experiment") {
    RCLCPP_WARN(get_logger(), "Ignoring unknown autonomy owner request '%s'", msg->data.c_str());
    return;
  }
  bool changed = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    changed = owner_ != requested;
    select_locked(requested);
  }
  if (changed) {
    publish_zero();
  }
}

void MttAutonomyMuxNode::on_release(const std_msgs::msg::String::SharedPtr msg)
{
  const auto released = normalized_owner(msg->data);
  bool changed = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (released == owner_) {
      select_locked("idle");
      changed = true;
    }
  }
  if (changed) {
    publish_zero();
  }
}

void MttAutonomyMuxNode::select_locked(const std::string & owner)
{
  if (owner_ == owner) {
    return;
  }
  owner_ = owner;
  if (owner_ == "wiln") {
    wiln_.has_command = false;
    wiln_.has_articulation = false;
  } else if (owner_ == "gps") {
    gps_.has_command = false;
    gps_.has_articulation = false;
  } else if (owner_ == "experiment") {
    experiment_.has_command = false;
    experiment_.has_articulation = false;
  }
  std_msgs::msg::String selected;
  selected.data = owner_;
  selected_pub_->publish(selected);
  RCLCPP_INFO(get_logger(), "Autonomy owner -> %s", owner_.c_str());
}

void MttAutonomyMuxNode::on_command(
  const geometry_msgs::msg::TwistStamped::SharedPtr msg, Input & input)
{
  if (!std::isfinite(msg->twist.linear.x) || !std::isfinite(msg->twist.angular.z)) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  input.command = *msg;
  input.command_received = now();
  input.has_command = true;
}

void MttAutonomyMuxNode::on_articulation(
  const std_msgs::msg::Float64::SharedPtr msg, Input & input)
{
  if (!std::isfinite(msg->data)) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  input.articulation_rad = msg->data;
  input.articulation_received = now();
  input.has_articulation = true;
}

void MttAutonomyMuxNode::publish_zero()
{
  geometry_msgs::msg::TwistStamped zero;
  zero.header.stamp = now();
  command_pub_->publish(zero);
}

void MttAutonomyMuxNode::on_timer()
{
  geometry_msgs::msg::TwistStamped command;
  std_msgs::msg::Float64 articulation;
  bool valid = false;
  bool has_owner = false;
  const auto stamp = now();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    Input * input = nullptr;
    if (owner_ == "wiln") {
      input = &wiln_;
    } else if (owner_ == "gps") {
      input = &gps_;
    } else if (owner_ == "experiment") {
      input = &experiment_;
    }
    has_owner = input != nullptr;
    if (input != nullptr && input->has_command && input->has_articulation) {
      const double command_age = (stamp - input->command_received).seconds();
      const double articulation_age = (stamp - input->articulation_received).seconds();
      const double pair_skew = std::abs(
        (input->command_received - input->articulation_received).seconds());
      valid = command_age <= input_timeout_s_ && articulation_age <= input_timeout_s_ &&
        pair_skew <= max_pair_skew_s_;
      if (valid) {
        command = input->command;
        command.header.stamp = stamp;
        articulation.data = input->articulation_rad;
      }
    }
  }

  if (!valid) {
    if (has_owner) {
      publish_zero();
    }
    return;
  }
  command_pub_->publish(command);
  articulation_pub_->publish(articulation);
}

}  // namespace mtt_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mtt_control::MttAutonomyMuxNode>());
  rclcpp::shutdown();
  return 0;
}
