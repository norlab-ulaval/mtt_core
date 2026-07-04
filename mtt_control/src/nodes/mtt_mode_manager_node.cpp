#include "mtt_control/nodes/mtt_mode_manager_node.hpp"

#include <algorithm>

namespace mtt_control
{

MttModeManagerNode::MttModeManagerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("mtt_mode_manager_node", options)
{
  button_auto_index_ = declare_parameter("button_auto_index", 0);
  button_manual_index_ = declare_parameter("button_manual_index", 3);
  button_stop_index_ = declare_parameter("button_stop_index", 1);
  startup_hold_s_ = declare_parameter("startup_hold_s", 0.0);
  publish_rate_hz_ = declare_parameter("publish_rate_hz", 20.0);
  request_auto_service_ = declare_parameter(
    "request_auto_service", std::string("mtt_control/request_auto"));
  request_manual_service_ = declare_parameter(
    "request_manual_service", std::string("mtt_control/request_manual"));
  request_stop_service_ = declare_parameter(
    "request_stop_service", std::string("mtt_control/request_stop"));

  startup_time_ = now();

  joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
    "joy",
    rclcpp::SensorDataQoS(),
    std::bind(&MttModeManagerNode::on_joy, this, std::placeholders::_1));
  manual_activity_sub_ = create_subscription<std_msgs::msg::Bool>(
    "mtt_control/manual_activity",
    20,
    std::bind(&MttModeManagerNode::on_manual_activity, this, std::placeholders::_1));
  estop_sub_ = create_subscription<std_msgs::msg::Bool>(
    "mtt_control/teleop_estop",
    20,
    std::bind(&MttModeManagerNode::on_estop, this, std::placeholders::_1));

  auto latched_qos = rclcpp::QoS(1).transient_local();
  mode_pub_ = create_publisher<std_msgs::msg::String>("mtt_control/selected_mode", latched_qos);
  auto_enabled_pub_ = create_publisher<std_msgs::msg::Bool>("mtt_control/auto_mode_enabled", latched_qos);
  request_auto_srv_ = create_service<std_srvs::srv::Trigger>(
    request_auto_service_,
    std::bind(
      &MttModeManagerNode::handle_request_auto, this, std::placeholders::_1,
      std::placeholders::_2));
  request_manual_srv_ = create_service<std_srvs::srv::Trigger>(
    request_manual_service_,
    std::bind(
      &MttModeManagerNode::handle_request_manual, this, std::placeholders::_1,
      std::placeholders::_2));
  request_stop_srv_ = create_service<std_srvs::srv::Trigger>(
    request_stop_service_,
    std::bind(
      &MttModeManagerNode::handle_request_stop, this, std::placeholders::_1,
      std::placeholders::_2));
  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_)),
    std::bind(&MttModeManagerNode::on_timer, this));

  publish_state();
}

void MttModeManagerNode::set_mode(ControlMode mode, const std::string & reason)
{
  if (mode == current_mode_ && reason == mode_reason_) {
    return;
  }
  current_mode_ = mode;
  mode_reason_ = reason;
  RCLCPP_INFO(get_logger(), "control mode -> %s (%s)", to_string(current_mode_).c_str(), mode_reason_.c_str());
  publish_state();
}

void MttModeManagerNode::publish_state()
{
  std_msgs::msg::String mode_msg;
  mode_msg.data = to_string(current_mode_);
  mode_pub_->publish(mode_msg);

  std_msgs::msg::Bool auto_msg;
  auto_msg.data = current_mode_ == ControlMode::Auto;
  auto_enabled_pub_->publish(auto_msg);
}

void MttModeManagerNode::on_joy(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  joystick_state_.update(*msg);

  if ((now() - startup_time_).seconds() < startup_hold_s_) {
    return;
  }

  if (estop_active_) {
    set_mode(ControlMode::Stop, "estop");
    return;
  }

  if (joystick_state_.button_rising(static_cast<std::size_t>(button_stop_index_))) {
    // Explicit stop button: always works, even during replay.
    auto_locked_ = false;
    set_mode(ControlMode::Stop, "stop_button");
    return;
  }
  if (joystick_state_.button_rising(static_cast<std::size_t>(button_manual_index_))) {
    // Explicit manual button: always works, operator takes back control.
    auto_locked_ = false;
    set_mode(ControlMode::Manual, "manual_button");
    return;
  }
  if (joystick_state_.button_rising(static_cast<std::size_t>(button_auto_index_))) {
    // Manual press of auto button: NOT a replay-locked auto, just a direct request.
    auto_locked_ = false;
    set_mode(ControlMode::Auto, "auto_button");
    return;
  }
}

void MttModeManagerNode::on_manual_activity(const std_msgs::msg::Bool::SharedPtr msg)
{
  manual_activity_ = msg->data;
  // Always exit AUTO when the operator moves the stick (deadman held + past threshold).
  // auto_locked_ no longer blocks this: physical presence on the controller is the
  // ultimate safety override, even during a teach-and-repeat replay.
  // Trade-off: stick drift can interrupt a replay — intentional per operator decision.
  if (manual_activity_ && !estop_active_) {
    auto_locked_ = false;
    set_mode(ControlMode::Manual, "manual_activity");
  }
}

void MttModeManagerNode::on_estop(const std_msgs::msg::Bool::SharedPtr msg)
{
  const bool was_active = estop_active_;
  estop_active_ = msg->data;
  if (estop_active_) {
    // Emergency stop always overrides everything, including replay lock.
    auto_locked_ = false;
    set_mode(ControlMode::Stop, "estop");
  } else if (was_active) {
    // Estop cleared (both triggers released): auto-return to Manual.
    // The operator still needs deadman pressed to issue any motion command.
    RCLCPP_INFO(get_logger(), "Estop cleared: returning to Manual mode.");
    set_mode(ControlMode::Manual, "estop_cleared");
  }
}

void MttModeManagerNode::on_timer()
{
  publish_state();
}

void MttModeManagerNode::handle_request_auto(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (estop_active_) {
    response->success = false;
    response->message = "estop active";
    return;
  }
  // Mark as replay-locked: manual_activity events will be ignored until
  // request_manual (or an explicit button press) clears this flag.
  auto_locked_ = true;
  set_mode(ControlMode::Auto, "service_auto");
  response->success = true;
  response->message = "auto mode enabled";
}

void MttModeManagerNode::handle_request_manual(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (estop_active_) {
    response->success = false;
    response->message = "estop active";
    return;
  }
  // Clear the replay lock: supervisor is done with AUTO.
  auto_locked_ = false;
  set_mode(ControlMode::Manual, "service_manual");
  response->success = true;
  response->message = "manual mode enabled";
}

void MttModeManagerNode::handle_request_stop(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  set_mode(ControlMode::Stop, "service_stop");
  response->success = true;
  response->message = "stop mode enabled";
}

}  // namespace mtt_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mtt_control::MttModeManagerNode>());
  rclcpp::shutdown();
  return 0;
}
