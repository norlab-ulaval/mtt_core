#pragma once

#include <mutex>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include "mtt_control/common/control_mode.hpp"

namespace mtt_control
{

class MttCmdArbiterNode : public rclcpp::Node
{
public:
  explicit MttCmdArbiterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_manual_cmd(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void on_auto_cmd(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void on_mode(const std_msgs::msg::String::SharedPtr msg);
  void on_auto_enabled(const std_msgs::msg::Bool::SharedPtr msg);
  void on_deadman(const std_msgs::msg::Bool::SharedPtr msg);
  void on_estop(const std_msgs::msg::Bool::SharedPtr msg);
  void on_timer();
  bool cmd_is_fresh(const rclcpp::Time & stamp, double timeout_s) const;
  void publish_source(const std::string & source);
  void publish_cmd(const geometry_msgs::msg::TwistStamped & msg);

  mutable std::mutex state_mutex_;

  std::string manual_cmd_topic_;
  std::string auto_cmd_topic_;
  std::string output_cmd_topic_;
  std::string source_topic_;
  double publish_rate_hz_{50.0};
  double manual_timeout_s_{0.5};
  double auto_timeout_s_{0.5};
  double mode_switch_hold_s_{0.15};
  bool manual_requires_deadman_{true};

  ControlMode current_mode_{ControlMode::Stop};
  bool auto_enabled_{false};
  bool deadman_active_{false};
  bool estop_active_{false};
  geometry_msgs::msg::TwistStamped last_manual_cmd_;
  geometry_msgs::msg::TwistStamped last_auto_cmd_;
  bool has_manual_cmd_{false};
  bool has_auto_cmd_{false};
  rclcpp::Time last_mode_change_time_;
  std::string last_source_{"STOP"};

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr manual_cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr auto_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr auto_enabled_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr deadman_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr output_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr source_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mtt_control
