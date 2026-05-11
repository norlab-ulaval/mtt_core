#pragma once

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

namespace mtt_control
{

/**
 * Converts standard Nav2 cmd_vel (geometry_msgs/Twist, v + omega) into the
 * MTT driver format (geometry_msgs/TwistStamped, v + normalized articulation).
 *
 * Conversion (closed_loop / normalized_steer mode):
 *   psi     = atan(wheelbase_m * omega / v)   [rad]
 *   psi_sat = clamp(psi, -psi_max, +psi_max)
 *   angular_out = psi_sat / psi_max            [-1 .. +1]
 *
 * Safety:
 *   - Timeout: publishes zero if no input received within timeout_s_.
 *   - Deadband: |v| < min_speed_deadband_ms → zero command (no psi estimate).
 *   - Saturation handled by clamping before normalization.
 *   - Reverse (v < 0): atan formula naturally inverts psi sign, matching
 *     articulated-vehicle steering semantics.
 */
class MttCmdVelBridgeNode : public rclcpp::Node
{
public:
  explicit MttCmdVelBridgeNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg);
  void on_timer();

  double wheelbase_m_{2.4};
  double max_articulation_rad_{1.0471975512};
  double min_speed_deadband_ms_{0.05};
  double timeout_s_{0.5};
  double publish_rate_hz_{20.0};

  bool has_input_{false};
  double last_v_{0.0};
  double last_omega_{0.0};
  rclcpp::Time last_input_time_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mtt_control
