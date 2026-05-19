#include "mtt_control/nodes/mtt_manual_cmd_filter_node.hpp"

#include <algorithm>
#include <cmath>
#include <mutex>

namespace mtt_control
{

MttManualCmdFilterNode::MttManualCmdFilterNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("mtt_manual_cmd_filter_node", options)
{
  input_topic_ = declare_parameter("input_topic", std::string("cmd_vel/manual_raw"));
  output_topic_ = declare_parameter("output_topic", std::string("cmd_vel/manual"));
  input_timeout_s_ = declare_parameter("input_timeout_s", 0.4);
  publish_rate_hz_ = declare_parameter("publish_rate_hz", 50.0);
  linear_rise_rate_ = declare_parameter("linear_rise_rate", 0.3);
  linear_fall_rate_ = declare_parameter("linear_fall_rate", 1.0);
  angular_rise_rate_ = declare_parameter("angular_rise_rate", 0.5);
  angular_fall_rate_ = declare_parameter("angular_fall_rate", 1.2);
  decel_brake_gain_      = declare_parameter("decel_brake_gain",      0.0);
  decel_brake_threshold_ = declare_parameter("decel_brake_threshold", 0.05);
  enable_dynamic_filter_ = declare_parameter("enable_dynamic_filter", false);
  linear_omega_n_ = declare_parameter("linear_omega_n", 8.0);
  linear_zeta_ = declare_parameter("linear_zeta", 1.0);
  angular_omega_n_ = declare_parameter("angular_omega_n", 10.0);
  angular_zeta_ = declare_parameter("angular_zeta", 1.0);

  linear_limiter_.set_limits(linear_rise_rate_, linear_fall_rate_);
  angular_limiter_.set_limits(angular_rise_rate_, angular_fall_rate_);
  linear_filter_.configure(linear_omega_n_, linear_zeta_);
  angular_filter_.configure(angular_omega_n_, angular_zeta_);

  last_input_time_ = now();
  last_update_time_ = now();

  input_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    input_topic_,
    20,
    std::bind(&MttManualCmdFilterNode::on_input, this, std::placeholders::_1));
  mode_sub_ = create_subscription<std_msgs::msg::String>(
    "mtt_control/selected_mode",
    20,
    std::bind(&MttManualCmdFilterNode::on_mode, this, std::placeholders::_1));
  estop_sub_ = create_subscription<std_msgs::msg::Bool>(
    "mtt_control/teleop_estop",
    20,
    std::bind(&MttManualCmdFilterNode::on_estop, this, std::placeholders::_1));
  deadman_sub_ = create_subscription<std_msgs::msg::Bool>(
    "mtt_control/teleop_deadman",
    20,
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      deadman_active_ = msg->data;
    });
  output_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(output_topic_, 20);
  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_)),
    std::bind(&MttManualCmdFilterNode::on_timer, this));
}

void MttManualCmdFilterNode::on_input(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  target_.linear_x = msg->twist.linear.x;
  target_.angular_z = msg->twist.angular.z;
  last_input_time_ = now();
  has_input_ = true;
}

void MttManualCmdFilterNode::on_mode(const std_msgs::msg::String::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  const auto new_mode = control_mode_from_string(msg->data);
  if (new_mode != current_mode_) {
    current_mode_ = new_mode;
    if (current_mode_ != ControlMode::Manual) {
      reset_filters();
    }
  }
}

void MttManualCmdFilterNode::on_estop(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  estop_active_ = msg->data;
  if (estop_active_) {
    reset_filters();
  }
}

void MttManualCmdFilterNode::reset_filters()
{
  target_ = {};
  linear_limiter_.reset(0.0);
  angular_limiter_.reset(0.0);
  linear_filter_.reset(0.0);
  angular_filter_.reset(0.0);
  has_input_ = false;
}

void MttManualCmdFilterNode::publish_zero_once(const rclcpp::Time & stamp)
{
  if (last_publish_was_zero_) {
    return;
  }

  geometry_msgs::msg::TwistStamped msg;
  msg.header.stamp = stamp;
  output_pub_->publish(msg);
  last_publish_was_zero_ = true;
}

void MttManualCmdFilterNode::on_timer()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  const auto now_stamp = now();
  double dt = (now_stamp - last_update_time_).seconds();
  if (dt <= 1e-6) {
    dt = 1.0 / std::max(1.0, publish_rate_hz_);
  }
  last_update_time_ = now_stamp;

  VelocityState effective_target = target_;
  if (current_mode_ != ControlMode::Manual || estop_active_ || !has_input_ ||
      (now_stamp - last_input_time_).seconds() > input_timeout_s_) {
    effective_target = {};
  }

  // Deadman rising-edge: reset all filter state so the first command after
  // re-press always ramps from zero (no stale velocity).
  if (deadman_active_ && !prev_deadman_) {
    reset_filters();
  }
  prev_deadman_ = deadman_active_;

  // Feedforward deceleration boost: when operator releases stick (target→0)
  // but the output is still significant, push the effective target negative
  // proportionally to the current output.  This makes the rate limiter
  // overshoot zero and issue brief counter-thrust — no tachometer needed.
  // Self-regulating: as output decays toward 0, boost decays to 0 too.
  // Bidirectional: works for both forward and reverse.
  // Disabled when decel_brake_gain_ == 0 (default: off).
  if (decel_brake_gain_ > 0.0) {
    const double prev_linear = linear_limiter_.value();
    if (std::abs(effective_target.linear_x) < zero_epsilon_ &&
        std::abs(prev_linear) > decel_brake_threshold_) {
      effective_target.linear_x = -decel_brake_gain_ * prev_linear;
    }
  }

  double linear = linear_limiter_.update(effective_target.linear_x, dt);
  double angular = angular_limiter_.update(effective_target.angular_z, dt);
  if (enable_dynamic_filter_) {
    linear = linear_filter_.update(linear, dt);
    angular = angular_filter_.update(angular, dt);
  } else {
    linear_filter_.reset(linear);
    angular_filter_.reset(angular);
  }

  const bool target_is_zero =
    std::abs(effective_target.linear_x) < zero_epsilon_ &&
    std::abs(effective_target.angular_z) < zero_epsilon_;
  const bool current_is_zero =
    std::abs(linear) < zero_epsilon_ &&
    std::abs(angular) < zero_epsilon_;

  if (target_is_zero && current_is_zero) {
    publish_zero_once(now_stamp);
    return;
  }

  geometry_msgs::msg::TwistStamped msg;
  msg.header.stamp = now_stamp;
  msg.twist.linear.x = linear;
  msg.twist.angular.z = angular;
  output_pub_->publish(msg);
  last_publish_was_zero_ = false;
}

}  // namespace mtt_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mtt_control::MttManualCmdFilterNode>());
  rclcpp::shutdown();
  return 0;
}
