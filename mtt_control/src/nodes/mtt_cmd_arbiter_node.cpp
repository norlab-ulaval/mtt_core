#include "mtt_control/nodes/mtt_cmd_arbiter_node.hpp"

#include <algorithm>

namespace mtt_control
{

MttCmdArbiterNode::MttCmdArbiterNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("mtt_cmd_arbiter_node", options)
{
  manual_cmd_topic_ = declare_parameter("manual_cmd_topic", std::string("cmd_vel/manual"));
  auto_cmd_topic_ = declare_parameter("auto_cmd_topic", std::string("controller/cmd_vel"));
  output_cmd_topic_ = declare_parameter("output_cmd_topic", std::string("cmd_vel"));
  source_topic_ = declare_parameter("source_topic", std::string("mtt_control/selected_source"));
  publish_rate_hz_ = declare_parameter("publish_rate_hz", 50.0);
  manual_timeout_s_ = declare_parameter("manual_timeout_s", 0.5);
  auto_timeout_s_ = declare_parameter("auto_timeout_s", 0.5);
  mode_switch_hold_s_ = declare_parameter("mode_switch_hold_s", 0.15);
  manual_requires_deadman_ = declare_parameter("manual_requires_deadman", true);

  last_mode_change_time_ = now();

  manual_cmd_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    manual_cmd_topic_, 20, std::bind(&MttCmdArbiterNode::on_manual_cmd, this, std::placeholders::_1));
  auto_cmd_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    auto_cmd_topic_, 20, std::bind(&MttCmdArbiterNode::on_auto_cmd, this, std::placeholders::_1));
  mode_sub_ = create_subscription<std_msgs::msg::String>(
    "mtt_control/selected_mode", 20, std::bind(&MttCmdArbiterNode::on_mode, this, std::placeholders::_1));
  auto_enabled_sub_ = create_subscription<std_msgs::msg::Bool>(
    "mtt_control/auto_mode_enabled", 20, std::bind(&MttCmdArbiterNode::on_auto_enabled, this, std::placeholders::_1));
  deadman_sub_ = create_subscription<std_msgs::msg::Bool>(
    "mtt_control/teleop_deadman", 20, std::bind(&MttCmdArbiterNode::on_deadman, this, std::placeholders::_1));
  estop_sub_ = create_subscription<std_msgs::msg::Bool>(
    "mtt_control/teleop_estop", 20, std::bind(&MttCmdArbiterNode::on_estop, this, std::placeholders::_1));

  output_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(output_cmd_topic_, 20);
  source_pub_ = create_publisher<std_msgs::msg::String>(source_topic_, rclcpp::QoS(1).transient_local());
  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_)),
    std::bind(&MttCmdArbiterNode::on_timer, this));
}

void MttCmdArbiterNode::on_manual_cmd(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  last_manual_cmd_ = *msg;
  has_manual_cmd_ = true;
}

void MttCmdArbiterNode::on_auto_cmd(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  last_auto_cmd_ = *msg;
  has_auto_cmd_ = true;
}

void MttCmdArbiterNode::on_mode(const std_msgs::msg::String::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  const auto new_mode = control_mode_from_string(msg->data);
  if (new_mode != current_mode_) {
    current_mode_ = new_mode;
    last_mode_change_time_ = now();
    RCLCPP_INFO(get_logger(), "arbiter mode -> %s", to_string(current_mode_).c_str());
  }
}

void MttCmdArbiterNode::on_auto_enabled(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  auto_enabled_ = msg->data;
}

void MttCmdArbiterNode::on_deadman(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  deadman_active_ = msg->data;
}

void MttCmdArbiterNode::on_estop(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  estop_active_ = msg->data;
}

bool MttCmdArbiterNode::cmd_is_fresh(const rclcpp::Time & stamp, double timeout_s) const
{
  if (stamp.nanoseconds() == 0) {
    return false;
  }
  return (now() - stamp).seconds() <= timeout_s;
}

void MttCmdArbiterNode::publish_source(const std::string & source)
{
  if (source == last_source_) {
    return;
  }
  std_msgs::msg::String msg;
  msg.data = source;
  source_pub_->publish(msg);
  last_source_ = source;
}

void MttCmdArbiterNode::publish_cmd(const geometry_msgs::msg::TwistStamped & msg)
{
  output_pub_->publish(msg);
}

void MttCmdArbiterNode::on_timer()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  geometry_msgs::msg::TwistStamped output;
  output.header.stamp = now();
  std::string source = "STOP";

  if (estop_active_) {
    source = "ESTOP";
    publish_source(source);
    publish_cmd(output);
    return;
  }

  if ((now() - last_mode_change_time_).seconds() < mode_switch_hold_s_) {
    source = "TRANSITION";
    publish_source(source);
    publish_cmd(output);
    return;
  }

  switch (current_mode_) {
    case ControlMode::Stop:
      source = "STOP";
      break;
    case ControlMode::Manual:
      if ((!manual_requires_deadman_ || deadman_active_) &&
          has_manual_cmd_ && cmd_is_fresh(last_manual_cmd_.header.stamp, manual_timeout_s_)) {
        output = last_manual_cmd_;
        output.header.stamp = now();
        source = "MANUAL";
      } else {
        source = deadman_active_ ? "MANUAL_TIMEOUT" : "MANUAL_IDLE";
      }
      break;
    case ControlMode::Auto:
      if (auto_enabled_ &&
          has_auto_cmd_ && cmd_is_fresh(last_auto_cmd_.header.stamp, auto_timeout_s_)) {
        output = last_auto_cmd_;
        output.header.stamp = now();
        source = "AUTO";
      } else {
        source = "AUTO_WAIT";
      }
      break;
  }

  publish_source(source);
  publish_cmd(output);
}

}  // namespace mtt_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mtt_control::MttCmdArbiterNode>());
  rclcpp::shutdown();
  return 0;
}
