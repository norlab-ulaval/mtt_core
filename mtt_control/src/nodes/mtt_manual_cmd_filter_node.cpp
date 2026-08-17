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
  decel_brake_gain_          = declare_parameter("decel_brake_gain",          0.0);
  decel_brake_threshold_     = declare_parameter("decel_brake_threshold",     0.05);
  reversal_guard_threshold_  = declare_parameter("reversal_guard_threshold",  0.0);
  enable_dynamic_filter_     = declare_parameter("enable_dynamic_filter",     false);
  linear_omega_n_ = declare_parameter("linear_omega_n", 8.0);
  linear_zeta_ = declare_parameter("linear_zeta", 1.0);
  angular_omega_n_ = declare_parameter("angular_omega_n", 10.0);
  angular_zeta_ = declare_parameter("angular_zeta", 1.0);
  immediate_angular_zero_ = declare_parameter("immediate_angular_zero", true);

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
      const bool was_active = deadman_active_;
      deadman_active_ = msg->data;
      if (was_active && !deadman_active_) {
        reset_filters();
      }
    });
  output_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(output_topic_, 20);
  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_)),
    std::bind(&MttManualCmdFilterNode::on_timer, this));
  param_cb_handle_ = add_on_set_parameters_callback(
    std::bind(&MttManualCmdFilterNode::on_set_parameters, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult MttManualCmdFilterNode::on_set_parameters(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  // Live-tunable slew rates. A rate of 0 would freeze the limiter output at its
  // current value, so the lower bound must stay strictly positive. 20 m/s²
  // reaches the 5.56 m/s top speed in ~0.3 s — effectively "no ramp".
  constexpr double kMinRate = 0.05;
  constexpr double kMaxRate = 20.0;

  for (const auto & param : params) {
    const auto & name = param.get_name();
    if (name == "linear_rise_rate" || name == "linear_fall_rate" ||
        name == "angular_rise_rate" || name == "angular_fall_rate") {
      const double value = param.as_double();
      if (value < kMinRate || value > kMaxRate) {
        result.successful = false;
        result.reason = name + " must be in [" + std::to_string(kMinRate) + ", " +
          std::to_string(kMaxRate) + "]";
        return result;
      }
    }
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  for (const auto & param : params) {
    const auto & name = param.get_name();
    if (name == "linear_rise_rate") {
      linear_rise_rate_ = param.as_double();
    } else if (name == "linear_fall_rate") {
      linear_fall_rate_ = param.as_double();
    } else if (name == "angular_rise_rate") {
      angular_rise_rate_ = param.as_double();
    } else if (name == "angular_fall_rate") {
      angular_fall_rate_ = param.as_double();
    } else {
      continue;
    }
    RCLCPP_INFO(get_logger(), "Live update: %s = %.3f", name.c_str(), param.as_double());
  }
  linear_limiter_.set_limits(linear_rise_rate_, linear_fall_rate_);
  angular_limiter_.set_limits(angular_rise_rate_, angular_fall_rate_);
  return result;
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
  reset_filter_state();
  has_input_ = false;
}

void MttManualCmdFilterNode::reset_filter_state()
{
  linear_limiter_.reset(0.0);
  angular_limiter_.reset(0.0);
  linear_filter_.reset(0.0);
  angular_filter_.reset(0.0);
  last_publish_was_zero_ = true;
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
  if (current_mode_ != ControlMode::Manual || estop_active_ || !deadman_active_ || !has_input_ ||
      (now_stamp - last_input_time_).seconds() > input_timeout_s_) {
    effective_target = {};
  }

  // Deadman rising-edge: reset only the filter/output state so the first
  // command after re-press ramps from zero, but keep the latest joystick target.
  // Joy and deadman arrive on separate topics; clearing target here can erase
  // the current axis value when the operator changes direction while released.
  if (deadman_active_ && !prev_deadman_) {
    reset_filter_state();
  }
  prev_deadman_ = deadman_active_;

  // Direction-reversal guard: block sign-flip commands while the filtered
  // output is above the configured threshold. The relay-direction hardware bug
  // means sending Reverse while still moving Forward causes the motor controller
  // to accelerate forward instead of braking. Requiring speed to drop below the
  // threshold before accepting a direction reversal prevents this.
  if (reversal_guard_threshold_ > 0.0) {
    const double cv = linear_limiter_.value();
    const bool fwd_reversal = effective_target.linear_x < -zero_epsilon_ && cv >  reversal_guard_threshold_;
    const bool rev_reversal = effective_target.linear_x >  zero_epsilon_ && cv < -reversal_guard_threshold_;
    if (fwd_reversal || rev_reversal) {
      effective_target.linear_x = 0.0;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
        "Direction reversal blocked (output=%.2f m/s, guard=%.2f m/s). Reduce speed first.",
        cv, reversal_guard_threshold_);
    }
  }

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
  double angular = 0.0;
  if (immediate_angular_zero_ &&
      std::abs(effective_target.angular_z) < zero_epsilon_) {
    angular_limiter_.reset(0.0);
    angular_filter_.reset(0.0);
  } else {
    angular = angular_limiter_.update(effective_target.angular_z, dt);
  }
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
