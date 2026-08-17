#pragma once

#include <chrono>
#include <vector>

#include <rcl_interfaces/msg/set_parameters_result.hpp>
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
  rcl_interfaces::msg::SetParametersResult on_set_parameters(
    const std::vector<rclcpp::Parameter> & params);
  void on_selected_mode(const std_msgs::msg::String::SharedPtr msg);
  void on_watchdog();
  void publish_safe_stop();
  bool trigger_pressed(const JoystickState & state, int axis_index) const;
  void publish_articulation_hold_state(bool hold_active);
  void toggle_steering_mode();
  void on_ice_slip_ratio(const std_msgs::msg::Float64::SharedPtr msg);
  void on_ice_com_shift_arm(const std_msgs::msg::Bool::SharedPtr msg);
  void update_ice_com_shift_experiment(float linear_axis, bool command_enabled, double brake_value);
  void reset_ice_com_shift_experiment(const std::string & phase);
  bool ice_com_shift_arm_fresh();
  bool ice_com_shift_slip_fresh();
  double ice_com_shift_com_axis();
  void request_ice_com_shift_park_home(const std::string & phase);
  void switch_ice_com_shift_phase(const std::string & reason);
  void publish_ice_com_shift_state();
  bool manual_articulation_owned() const;

  // COM motor mode toggle
  void publish_com_state(float shaped_angular_axis, bool command_enabled, bool force_com_mode = false);
  void publish_com_direction_sign();

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
  int articulation_hold_button_index_{10};  // R3 — articulation axis inhibit toggle (default matches YAML)
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
  std::string selected_mode_{"STOP"};
  std::chrono::steady_clock::time_point last_joy_receive_time_{};

  rclcpp::Time rt_press_start_time_;
  bool rt_is_fully_pressed_{false};
  bool rt_auto_engaged_{false};           // true when parking brake was auto-engaged by RT 1s hold

  // R3 articulation inhibit — zeroes right stick for ALL modes (articulation AND COM motor).
  // Toggle on each R3 press. Cleared on deadman release (prevents latent inhibit surprise).
  bool articulation_axis_inhibit_{false};

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
  bool   enable_com_direction_switch_{false};
  int    com_direction_toggle_button_index_{-1};
  double com_direction_sign_{1.0};
  int    com_quick_park_button_index_{-1};
  double com_short_press_max_s_{1.5};
  double com_long_press_min_s_{4.0};
  rclcpp::Time       com_btn_press_time_{0, 0, RCL_ROS_TIME};
  bool               com_btn_was_pressed_{false};

  bool enable_ice_com_shift_experiment_{false};
  std::string ice_com_shift_slip_ratio_topic_{"/ice_slip/slip_ratio"};
  std::string ice_com_shift_arm_topic_{"mtt_control/ice_com_shift_experiment_active"};
  double ice_com_shift_arm_timeout_s_{0.5};
  double ice_com_shift_slip_timeout_s_{0.5};
  double ice_com_shift_speed_ms_{5.56};
  double ice_com_shift_start_axis_threshold_{0.70};
  double ice_com_shift_switch_abs_slip_below_{0.08};
  double ice_com_shift_switch_hold_s_{0.10};
  double ice_com_shift_min_phase_s_{0.50};
  double ice_com_shift_max_phase_s_{0.0};
  bool ice_com_shift_park_home_on_start_{false};
  double ice_com_shift_park_home_s_{0.15};
  bool ice_com_shift_park_home_on_stop_{false};
  bool ice_com_shift_switch_on_brake_{true};
  double ice_com_shift_brake_switch_threshold_{0.20};
  bool ice_com_shift_invert_com_axis_{false};
  bool ice_com_shift_jitter_enabled_{false};
  double ice_com_shift_jitter_amplitude_{0.0};
  double ice_com_shift_jitter_frequency_hz_{0.0};
  bool ice_com_shift_armed_{false};
  bool ice_com_shift_active_{false};
  bool ice_com_shift_parking_home_{false};
  bool ice_com_shift_brake_latched_{false};
  int ice_com_shift_phase_sign_{1};
  double ice_com_shift_last_slip_ratio_{0.0};
  bool ice_com_shift_has_slip_{false};
  rclcpp::Time ice_com_shift_last_arm_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time ice_com_shift_last_slip_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time ice_com_shift_phase_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time ice_com_shift_below_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time ice_com_shift_park_until_{0, 0, RCL_ROS_TIME};
  std::string ice_com_shift_phase_{"idle"};

  // Max values for joystick mapping (read from params)
  double max_joystick_position_rad_{0.785}; // ±45° default
  double max_joystick_velocity_rad_s_{0.50}; // ±29°/s default

  JoystickState joystick_state_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr selected_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr ice_slip_ratio_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ice_com_shift_arm_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr manual_raw_pub_;
  rclcpp::Publisher<mtt_msgs::msg::MttAuxCommand>::SharedPtr aux_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr deadman_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr manual_activity_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr articulation_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr articulation_hold_active_pub_;

  // Publishers for software PID
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr position_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr velocity_cmd_pub_;

  // COM motor topics (published when enable_com_mode_switch_ is true)
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr    com_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr com_steer_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr com_direction_sign_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr   com_park_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr   com_set_home_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ice_com_shift_active_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ice_com_shift_armed_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ice_com_shift_phase_pub_;

  // Software steering mode logic
  bool use_software_pid_{false};
  double last_angular_axis_{0.0};
  enum class SteeringRoutingMode { POSITION_RETURN, VELOCITY_HOLD };
  SteeringRoutingMode current_steering_routing_{SteeringRoutingMode::POSITION_RETURN};

  // Service clients
  rclcpp::Client<mtt_interfaces::srv::SetSteerControlMode>::SharedPtr can_steer_mode_client_;
  rclcpp::Client<mtt_interfaces::srv::SetSteerControlMode>::SharedPtr odom_steer_mode_client_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

}  // namespace mtt_control
