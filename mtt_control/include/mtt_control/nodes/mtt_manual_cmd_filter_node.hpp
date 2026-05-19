#pragma once

#include <mutex>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include "mtt_control/common/control_mode.hpp"
#include "mtt_control/filters/dynamic_filter.hpp"
#include "mtt_control/filters/slew_rate_limiter.hpp"

namespace mtt_control
{

class MttManualCmdFilterNode : public rclcpp::Node
{
public:
  explicit MttManualCmdFilterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  struct VelocityState
  {
    double linear_x{0.0};
    double angular_z{0.0};
  };

  void on_input(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void on_mode(const std_msgs::msg::String::SharedPtr msg);
  void on_estop(const std_msgs::msg::Bool::SharedPtr msg);
  void on_timer();
  void reset_filters();
  void publish_zero_once(const rclcpp::Time & stamp);

  std::string input_topic_;
  std::string output_topic_;
  double input_timeout_s_{0.4};
  double publish_rate_hz_{50.0};
  double linear_rise_rate_{0.5};
  double linear_fall_rate_{1.0};
  double angular_rise_rate_{0.8};
  double angular_fall_rate_{1.2};
  bool enable_dynamic_filter_{false};
  double linear_omega_n_{8.0};
  double linear_zeta_{1.0};
  double angular_omega_n_{10.0};
  double angular_zeta_{1.0};
  double zero_epsilon_{1e-3};

  // Feedforward deceleration brake.
  // When output > decel_brake_threshold_ and target drops to 0,
  // the effective target is pushed to -decel_brake_gain_ * output so
  // the rate limiter overshoots zero and commands brief counter-thrust.
  // Self-regulating: as output decays toward 0, boost decays too.
  // Does NOT require tachometer feedback.
  double decel_brake_gain_{0.0};
  double decel_brake_threshold_{0.05};

  mutable std::mutex state_mutex_;

  VelocityState target_;
  ControlMode current_mode_{ControlMode::Stop};
  bool estop_active_{false};
  bool last_publish_was_zero_{true};
  bool has_input_{false};
  bool deadman_active_{false};
  bool prev_deadman_{false};

  SlewRateLimiter linear_limiter_;
  SlewRateLimiter angular_limiter_;
  DynamicFilter linear_filter_;
  DynamicFilter angular_filter_;

  rclcpp::Time last_input_time_;
  rclcpp::Time last_update_time_;

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr input_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr deadman_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr output_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mtt_control
