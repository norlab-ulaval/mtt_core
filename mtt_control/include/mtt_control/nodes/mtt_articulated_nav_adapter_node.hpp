#pragma once

#include <mutex>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include "mtt_control/logic/articulated_command_model.hpp"

namespace mtt_control
{

/** Convert Nav2 (signed speed, body yaw-rate) into canonical MTT command.
 *
 * Canonical TwistStamped semantics:
 *   linear.x  = signed longitudinal speed [m/s]
 *   angular.z = normalized articulation [-1, 1], never a yaw-rate
 */
class MttArticulatedNavAdapterNode : public rclcpp::Node
{
public:
  explicit MttArticulatedNavAdapterNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_nav_command(const geometry_msgs::msg::Twist::SharedPtr msg);
  void on_articulation(const std_msgs::msg::Float64::SharedPtr msg);
  void on_timer();

  logic::ArticulatedCommandModel model_;
  double input_timeout_s_{0.25};
  double feedback_timeout_s_{0.30};
  double publish_rate_hz_{20.0};
  bool require_fresh_articulation_{true};

  std::mutex mutex_;
  double speed_ms_{0.0};
  double yaw_rate_rad_s_{0.0};
  double measured_articulation_rad_{0.0};
  double previous_command_rad_{0.0};
  bool has_input_{false};
  bool has_feedback_{false};
  rclcpp::Time input_stamp_;
  rclcpp::Time feedback_stamp_;
  rclcpp::Time previous_publish_stamp_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr articulation_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mtt_control
