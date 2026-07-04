#pragma once

#include <mutex>
#include <string>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include "mtt_control/logic/articulated_command_model.hpp"

namespace mtt_control
{

/** Sole adapter from canonical autonomous command to the two MTT actuator APIs.
 *
 * Input TwistStamped is atomic at the arbitration boundary:
 *   linear.x  = signed longitudinal speed [m/s]
 *   angular.z = normalized articulation [-1, 1]
 *
 * Output is split only here:
 *   speed_output_topic        -> TwistStamped with angular.z = 0
 *   articulation_output_topic -> absolute Float64 setpoint [rad]
 */
class MttArticulatedCommandDispatcherNode : public rclcpp::Node
{
public:
  explicit MttArticulatedCommandDispatcherNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_command(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void on_articulation(const std_msgs::msg::Float64::SharedPtr msg);
  void on_timer();
  static bool is_hardware_topic(const std::string & topic);

  logic::ArticulatedCommandModel model_;
  double input_timeout_s_{0.25};
  double feedback_timeout_s_{0.30};
  double publish_rate_hz_{20.0};
  bool active_{false};
  bool outputs_are_hardware_{false};

  std::mutex mutex_;
  geometry_msgs::msg::TwistStamped last_command_;
  double measured_articulation_rad_{0.0};
  double previous_setpoint_rad_{0.0};
  bool has_command_{false};
  bool has_feedback_{false};
  rclcpp::Time command_stamp_;
  rclcpp::Time feedback_stamp_;
  rclcpp::Time previous_publish_stamp_;

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr command_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr articulation_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr speed_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr articulation_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mtt_control
