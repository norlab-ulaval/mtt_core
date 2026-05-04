#pragma once

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "mtt_control/common/control_mode.hpp"
#include "mtt_control/common/joystick_state.hpp"

namespace mtt_control
{

class MttModeManagerNode : public rclcpp::Node
{
public:
  explicit MttModeManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_joy(const sensor_msgs::msg::Joy::SharedPtr msg);
  void on_manual_activity(const std_msgs::msg::Bool::SharedPtr msg);
  void on_estop(const std_msgs::msg::Bool::SharedPtr msg);
  void on_timer();
  void handle_request_auto(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void handle_request_manual(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void handle_request_stop(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void set_mode(ControlMode mode, const std::string & reason);
  void publish_state();

  JoystickState joystick_state_;
  ControlMode current_mode_{ControlMode::Stop};
  bool estop_active_{false};
  bool manual_activity_{false};
  int button_auto_index_{0};
  int button_manual_index_{3};
  int button_stop_index_{1};
  double startup_hold_s_{0.0};
  double publish_rate_hz_{20.0};
  std::string request_auto_service_{"mtt_control/request_auto"};
  std::string request_manual_service_{"mtt_control/request_manual"};
  std::string request_stop_service_{"mtt_control/request_stop"};
  std::string mode_reason_{"startup"};
  rclcpp::Time startup_time_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr manual_activity_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr auto_enabled_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr request_auto_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr request_manual_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr request_stop_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mtt_control
