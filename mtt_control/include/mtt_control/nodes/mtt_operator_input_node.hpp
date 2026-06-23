#pragma once

#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mtt_msgs/msg/mtt_aux_command.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <mtt_interfaces/srv/set_steer_control_mode.hpp>

#include "mtt_control/common/joystick_state.hpp"

namespace mtt_control
{

class MttOperatorInputNode : public rclcpp::Node
{
public:
  explicit MttOperatorInputNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_joy(const sensor_msgs::msg::Joy::SharedPtr msg);
  void on_watchdog();
  void publish_safe_stop();
  bool trigger_pressed(const JoystickState & state, int axis_index) const;
  void publish_articulation_hold_state(bool hold_active);
  void toggle_steering_mode();

  // COM motor mode toggle
  void publish_com_state(float shaped_angular_axis, bool command_enabled);

  double max_linear_speed_{1.0};
  double max_angular_command_{0.6};
  double linear_deadband_{0.05};
  double angular_deadband_{0.08};
  double linear_expo_{1.2};
  double angular_expo_{1.6};
  double manual_activity_linear_threshold_{0.05};
  double manual_activity_angular_threshold_{0.05};
  double estop_trigger_threshold_{-0.10};
  // Calibrated brake mapping: raw axis value when trigger is fully released / fully pressed.
  // With joy_linux param default_trig_val=1.0 the trigger reads brake_axis_released_ at boot
  // so brake_value = 0 until the operator actually presses RT.
  double brake_axis_released_{1.0};
  double brake_axis_pressed_{-1.0};
  // Fraction of brake_value [0,1] at which linear command is fully attenuated to zero.
  double brake_full_fraction_{0.6};
  double joy_timeout_s_{0.25};
  int deadman_button_index_{5};

  int light_button_index_{2};
  int articulation_hold_button_index_{6};
  int steer_mode_switch_button_index_{4};
  int linear_axis_index_{1};
  int angular_axis_index_{3};
  int brake_axis_index_{5};
  int estop_left_trigger_axis_{2};
  int estop_right_trigger_axis_{5};
  bool invert_linear_axis_{true};
  bool invert_angular_axis_{false};
  bool enable_brake_axis_{true};
  bool enable_steering_mode_switch_{true};

  bool light_state_{false};
  bool previous_deadman_pressed_{false};
  bool movement_inhibited_{false};
  bool parking_brake_mode_{false};
  std::string current_steer_mode_{"closed_loop"};
  bool joy_received_{false};
  bool joy_timeout_reported_{false};
  std::chrono::steady_clock::time_point last_joy_receive_time_{};

  rclcpp::Time rt_press_start_time_;
  bool rt_is_fully_pressed_{false};

  // ── COM motor mode ──
  // When com_mode_active_ is true, the right stick (angular_axis) drives the
  // COM motor via mtt_control/com_steer instead of the articulation servo.
  // Button 7 press durations:
  //   < 1.5 s  → toggle COM ON/OFF
  //   1.5–4.0 s → set_home (save current position as home)
  //   > 4.0 s   → park (return to home)
  bool   enable_com_mode_switch_{true};
  int    com_toggle_button_index_{7};
  bool   invert_com_steer_{false};
  bool   com_mode_active_{false};
  double com_short_press_max_s_{1.5};
  double com_long_press_min_s_{4.0};
  rclcpp::Time       com_btn_press_time_{0, 0, RCL_ROS_TIME};
  bool               com_btn_was_pressed_{false};

  JoystickState joystick_state_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr manual_raw_pub_;
  rclcpp::Publisher<mtt_msgs::msg::MttAuxCommand>::SharedPtr aux_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr deadman_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr manual_activity_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr articulation_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr articulation_hold_active_pub_;

  // COM motor topics (published when enable_com_mode_switch_ is true)
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr    com_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr com_steer_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr   com_park_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr   com_set_home_pub_;

  rclcpp::Client<mtt_interfaces::srv::SetSteerControlMode>::SharedPtr can_steer_mode_client_;
  rclcpp::Client<mtt_interfaces::srv::SetSteerControlMode>::SharedPtr odom_steer_mode_client_;
};

}  // namespace mtt_control
