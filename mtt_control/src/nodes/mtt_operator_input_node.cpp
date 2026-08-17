#include "mtt_control/nodes/mtt_operator_input_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace mtt_control
{

MttOperatorInputNode::MttOperatorInputNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("mtt_operator_input_node", options)
{
  max_linear_speed_ = declare_parameter("max_linear_speed", 0.4);
  max_angular_command_ = declare_parameter("max_angular_command", 0.4);
  linear_deadband_ = declare_parameter("linear_deadband", 0.10);
  angular_deadband_ = declare_parameter("angular_deadband", 0.10);
  linear_expo_ = declare_parameter("linear_expo", 1.2);
  angular_expo_ = declare_parameter("angular_expo", 1.6);
  manual_activity_linear_threshold_ = declare_parameter("manual_activity_linear_threshold", 0.05);
  manual_activity_angular_threshold_ = declare_parameter("manual_activity_angular_threshold", 0.05);
  estop_trigger_threshold_ = declare_parameter("estop_trigger_threshold", -0.10);
  brake_axis_released_ = declare_parameter("brake_axis_released", 1.0);
  brake_axis_pressed_  = declare_parameter("brake_axis_pressed",  -1.0);
  brake_full_fraction_ = declare_parameter("brake_full_fraction", 0.6);
  joy_timeout_s_ = declare_parameter("joy_timeout_s", 0.25);
  deadman_button_index_ = declare_parameter("deadman_button_index", 5);
  light_button_index_ = declare_parameter("light_button_index", 2);
  articulation_hold_button_index_ = declare_parameter("articulation_hold_button_index", 10);
  linear_axis_index_ = declare_parameter("linear_axis_index", 1);
  angular_axis_index_ = declare_parameter("angular_axis_index", 3);
  brake_axis_index_ = declare_parameter("brake_axis_index", 5);
  estop_left_trigger_axis_ = declare_parameter("estop_left_trigger_axis", 2);
  estop_right_trigger_axis_ = declare_parameter("estop_right_trigger_axis", 5);
  invert_linear_axis_ = declare_parameter("invert_linear_axis", true);
  invert_angular_axis_ = declare_parameter("invert_angular_axis", false);
  enable_brake_axis_ = declare_parameter("enable_brake_axis", true);
  (void)declare_parameter("articulation_hold_enabled", false);
  (void)declare_parameter("articulation_hold_reset_on_deadman_release", true);
  (void)declare_parameter("articulation_hold_mode_default", false);
  enable_steering_mode_switch_ = declare_parameter("enable_steering_mode_switch", true);
  steer_mode_switch_button_index_ = declare_parameter("steer_mode_switch_button_index", 4);

  // COM motor mode — duration-based actions on button 7
  enable_com_mode_switch_      = declare_parameter("enable_com_mode_switch",      true);
  com_toggle_button_index_     = declare_parameter("com_toggle_button_index",     7);
  invert_com_steer_            = declare_parameter("invert_com_steer",            false);
  enable_com_direction_switch_ = declare_parameter("enable_com_direction_switch", false);
  com_direction_toggle_button_index_ =
    declare_parameter("com_direction_toggle_button_index", -1);
  com_quick_park_button_index_ = declare_parameter("com_quick_park_button_index", -1);
  com_short_press_max_s_       = declare_parameter("com_short_press_max_s",       1.5);
  com_long_press_min_s_        = declare_parameter("com_long_press_min_s",        4.0);

  enable_ice_com_shift_experiment_ =
    declare_parameter("enable_ice_com_shift_experiment", false);
  ice_com_shift_slip_ratio_topic_ = declare_parameter(
    "ice_com_shift_slip_ratio_topic", std::string("/ice_slip/slip_ratio"));
  ice_com_shift_arm_topic_ = declare_parameter(
    "ice_com_shift_arm_topic", std::string("mtt_control/ice_com_shift_experiment_active"));
  ice_com_shift_arm_timeout_s_ = declare_parameter("ice_com_shift_arm_timeout_s", 0.5);
  ice_com_shift_slip_timeout_s_ = declare_parameter("ice_com_shift_slip_timeout_s", 0.5);
  ice_com_shift_speed_ms_ = declare_parameter("ice_com_shift_speed_ms", 5.56);
  ice_com_shift_start_axis_threshold_ =
    declare_parameter("ice_com_shift_start_axis_threshold", 0.70);
  ice_com_shift_switch_abs_slip_below_ =
    declare_parameter("ice_com_shift_switch_abs_slip_below", 0.08);
  ice_com_shift_switch_hold_s_ = declare_parameter("ice_com_shift_switch_hold_s", 0.10);
  ice_com_shift_min_phase_s_ = declare_parameter("ice_com_shift_min_phase_s", 0.50);
  ice_com_shift_max_phase_s_ = declare_parameter("ice_com_shift_max_phase_s", 0.0);
  ice_com_shift_park_home_on_start_ =
    declare_parameter("ice_com_shift_park_home_on_start", false);
  ice_com_shift_park_home_s_ = declare_parameter("ice_com_shift_park_home_s", 0.15);
  ice_com_shift_park_home_on_stop_ =
    declare_parameter("ice_com_shift_park_home_on_stop", false);
  ice_com_shift_switch_on_brake_ =
    declare_parameter("ice_com_shift_switch_on_brake", true);
  ice_com_shift_brake_switch_threshold_ =
    declare_parameter("ice_com_shift_brake_switch_threshold", 0.20);
  ice_com_shift_invert_com_axis_ = declare_parameter("ice_com_shift_invert_com_axis", false);
  ice_com_shift_jitter_enabled_ = declare_parameter("ice_com_shift_jitter_enabled", false);
  ice_com_shift_jitter_amplitude_ = declare_parameter("ice_com_shift_jitter_amplitude", 0.0);
  ice_com_shift_jitter_frequency_hz_ =
    declare_parameter("ice_com_shift_jitter_frequency_hz", 0.0);

  double max_pos_deg = declare_parameter("max_joystick_position_deg", 45.0);
  double max_vel_deg_s = declare_parameter("max_joystick_velocity_deg_s", 29.0);
  max_joystick_position_rad_ = max_pos_deg * M_PI / 180.0;
  max_joystick_velocity_rad_s_ = max_vel_deg_s * M_PI / 180.0;
  use_software_pid_ = declare_parameter("use_software_pid", false);

  joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
    "joy",
    rclcpp::SensorDataQoS(),
    std::bind(&MttOperatorInputNode::on_joy, this, std::placeholders::_1));
  selected_mode_sub_ = create_subscription<std_msgs::msg::String>(
    "mtt_control/selected_mode",
    rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&MttOperatorInputNode::on_selected_mode, this, std::placeholders::_1));
  if (enable_ice_com_shift_experiment_) {
    ice_slip_ratio_sub_ = create_subscription<std_msgs::msg::Float64>(
      ice_com_shift_slip_ratio_topic_, 20,
      std::bind(&MttOperatorInputNode::on_ice_slip_ratio, this, std::placeholders::_1));
    ice_com_shift_arm_sub_ = create_subscription<std_msgs::msg::Bool>(
      ice_com_shift_arm_topic_, 20,
      std::bind(&MttOperatorInputNode::on_ice_com_shift_arm, this, std::placeholders::_1));
  }

  manual_raw_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel/manual_raw", 20);
  aux_pub_ = create_publisher<mtt_msgs::msg::MttAuxCommand>("mtt_aux_cmd", 20);
  deadman_pub_ = create_publisher<std_msgs::msg::Bool>("mtt_control/teleop_deadman", 20);
  estop_pub_ = create_publisher<std_msgs::msg::Bool>("mtt_control/teleop_estop", 20);
  manual_activity_pub_ = create_publisher<std_msgs::msg::Bool>("mtt_control/manual_activity", 20);
  articulation_mode_pub_ =
    create_publisher<std_msgs::msg::String>("mtt_control/articulation_mode", 20);
  articulation_hold_active_pub_ =
    create_publisher<std_msgs::msg::Bool>("mtt_control/articulation_hold_active", rclcpp::QoS(1).transient_local());

  // COM motor publishers (always created so topics appear on the graph even when OFF)
  com_mode_pub_  = create_publisher<std_msgs::msg::Bool>  ("mtt_control/com_mode",  20);
  com_steer_pub_ = create_publisher<std_msgs::msg::Float64>("mtt_control/com_steer", 20);
  com_direction_sign_pub_ =
    create_publisher<std_msgs::msg::Float64>("mtt_control/com_direction_sign", 10);
  com_park_pub_  = create_publisher<std_msgs::msg::Empty> ("mtt_control/com_park",  10);
  com_set_home_pub_ = create_publisher<std_msgs::msg::Empty>("mtt_control/com_set_home", 10);
  ice_com_shift_active_pub_ =
    create_publisher<std_msgs::msg::Bool>("mtt_control/ice_com_shift_active", 10);
  ice_com_shift_armed_pub_ =
    create_publisher<std_msgs::msg::Bool>("mtt_control/ice_com_shift_armed", 10);
  ice_com_shift_phase_pub_ =
    create_publisher<std_msgs::msg::String>("mtt_control/ice_com_shift_phase", 10);

  position_cmd_pub_ = create_publisher<std_msgs::msg::Float64>(
    "/mtt_articulation_setpoint", 10);
  velocity_cmd_pub_ = create_publisher<std_msgs::msg::Float64>(
    "/mtt_articulation_velocity_cmd", 10);

  publish_timer_ = create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&MttOperatorInputNode::on_watchdog, this));

  publish_articulation_hold_state(false);

  can_steer_mode_client_ = create_client<mtt_interfaces::srv::SetSteerControlMode>(
    "mtt/set_steer_control_mode");
  odom_steer_mode_client_ = create_client<mtt_interfaces::srv::SetSteerControlMode>(
    "mtt/odometry/set_steer_control_mode");

  param_cb_handle_ = add_on_set_parameters_callback(
    std::bind(&MttOperatorInputNode::on_set_parameters, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult MttOperatorInputNode::on_set_parameters(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  // Live-tunable software ceilings. 5.56 m/s (20 km/h) matches the physical
  // top speed encoded in mtt_can_node.max_linear_speed_ms — the CAN scaling
  // saturates there anyway, so a higher value cannot be honored.
  constexpr double kMaxLinearCeiling = 5.56;
  constexpr double kMaxAngularCeiling = 1.0;

  for (const auto & param : params) {
    const auto & name = param.get_name();
    if (name == "max_linear_speed") {
      const double value = param.as_double();
      if (value <= 0.0 || value > kMaxLinearCeiling) {
        result.successful = false;
        result.reason = "max_linear_speed must be in (0, " +
          std::to_string(kMaxLinearCeiling) + "]";
        return result;
      }
    } else if (name == "max_angular_command") {
      const double value = param.as_double();
      if (value <= 0.0 || value > kMaxAngularCeiling) {
        result.successful = false;
        result.reason = "max_angular_command must be in (0, " +
          std::to_string(kMaxAngularCeiling) + "]";
        return result;
      }
    }
  }

  for (const auto & param : params) {
    const auto & name = param.get_name();
    if (name == "max_linear_speed") {
      max_linear_speed_ = param.as_double();
    } else if (name == "max_angular_command") {
      max_angular_command_ = param.as_double();
    } else {
      continue;
    }
    RCLCPP_INFO(get_logger(), "Live update: %s = %.3f", name.c_str(), param.as_double());
  }
  return result;
}

void MttOperatorInputNode::on_selected_mode(const std_msgs::msg::String::SharedPtr msg)
{
  selected_mode_ = msg->data;
}

bool MttOperatorInputNode::manual_articulation_owned() const
{
  // The deadman remains a physical presence gate in AUTO, but it must not grant
  // the joystick ownership of steering. WILN owns /mtt_articulation_setpoint in
  // AUTO; the operator owns it in MANUAL/STOP only.
  return selected_mode_ != "AUTO";
}

void MttOperatorInputNode::toggle_steering_mode()
{
  if (use_software_pid_) {
    // Software PID Mode toggle
    if (current_steering_routing_ == SteeringRoutingMode::POSITION_RETURN) {
      current_steering_routing_ = SteeringRoutingMode::VELOCITY_HOLD;
      RCLCPP_INFO(get_logger(), "Software PID Mode: VELOCITY (Maintien de position)");
    } else {
      current_steering_routing_ = SteeringRoutingMode::POSITION_RETURN;
      RCLCPP_INFO(get_logger(), "Software PID Mode: POSITION (Retour à zéro)");
    }
  } else {
    // Hardware CAN Mode toggle (Base behavior)
    const std::string new_mode =
      (current_steer_mode_ == "closed_loop") ? "open_loop" : "closed_loop";

    auto req = std::make_shared<mtt_interfaces::srv::SetSteerControlMode::Request>();
    req->control_mode = new_mode;
    req->max_rate  = 0.0;
    req->max_angle = 0.0;

    if (can_steer_mode_client_->service_is_ready()) {
      can_steer_mode_client_->async_send_request(
        req,
        [this, new_mode](std::shared_future<std::shared_ptr<mtt_interfaces::srv::SetSteerControlMode::Response>> future) {
          auto resp = future.get();
          if (resp->success) {
            current_steer_mode_ = new_mode;
            RCLCPP_INFO(get_logger(), "Steer mode (CAN) → %s", current_steer_mode_.c_str());
          }
        });
    }
    if (odom_steer_mode_client_->service_is_ready()) {
      odom_steer_mode_client_->async_send_request(req);
    }
  }
}

bool MttOperatorInputNode::trigger_pressed(const JoystickState & state, int axis_index) const
{
  return state.axis_value(static_cast<std::size_t>(axis_index), 1.0F) < estop_trigger_threshold_;
}

void MttOperatorInputNode::publish_articulation_hold_state(bool hold_active)
{
  auto mode_msg = std_msgs::msg::String();
  mode_msg.data = "neutral";
  articulation_mode_pub_->publish(mode_msg);

  auto active_msg = std_msgs::msg::Bool();
  active_msg.data = hold_active;
  articulation_hold_active_pub_->publish(active_msg);
}

void MttOperatorInputNode::on_joy(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  last_joy_receive_time_ = std::chrono::steady_clock::now();
  joy_received_ = true;
  if (joy_timeout_reported_) {
    RCLCPP_INFO(get_logger(), "Joystick messages restored.");
    joy_timeout_reported_ = false;
  }

  joystick_state_.update(*msg);

  const bool deadman_pressed = joystick_state_.button_pressed(static_cast<std::size_t>(deadman_button_index_));
  const bool deadman_rising = deadman_pressed && !previous_deadman_pressed_;
  const bool deadman_released = previous_deadman_pressed_ && !deadman_pressed;
  
  const bool light_rising = joystick_state_.button_rising(static_cast<std::size_t>(light_button_index_));
  const bool articulation_hold_rising =
    joystick_state_.button_rising(static_cast<std::size_t>(articulation_hold_button_index_));
  const bool steer_mode_rising =
    enable_steering_mode_switch_ &&
    joystick_state_.button_rising(static_cast<std::size_t>(steer_mode_switch_button_index_));

  if (light_rising) {
    light_state_ = !light_state_;
  }

  // R3 — articulation axis inhibit toggle.
  // Zeroes the right stick for all modes (articulation AND COM motor).
  // Cleared automatically on deadman release to avoid latent surprises.
  if (articulation_hold_rising) {
    articulation_axis_inhibit_ = !articulation_axis_inhibit_;
    RCLCPP_INFO(get_logger(), "Articulation Inhibit (R3): %s",
                articulation_axis_inhibit_ ? "ON (right stick paused)" : "OFF (right stick active)");
  }

  if (steer_mode_rising) {
    toggle_steering_mode();
  }

  // ---- COM motor mode + duration-based actions on button 7 ----
  // Short press (< com_short_press_max_s_) → toggle COM ON/OFF.
  // Medium press (com_short_press_max_s_ – com_long_press_min_s_) → set_home.
  // Long press (> com_long_press_min_s_) → park (return to home).
  if (enable_com_mode_switch_) {
    const bool btn_down = joystick_state_.button_pressed(
      static_cast<std::size_t>(com_toggle_button_index_));

    // Rising edge: record press time
    if (btn_down && !com_btn_was_pressed_) {
      com_btn_press_time_ = now();
    }

    // Falling edge: calculate hold duration and act
    if (!btn_down && com_btn_was_pressed_) {
      const double hold_s = (now() - com_btn_press_time_).seconds();

      if (hold_s >= com_long_press_min_s_) {
        RCLCPP_INFO(get_logger(),
                    "COM btn held %.1fs → PARK (return to home)", hold_s);
        com_park_pub_->publish(std_msgs::msg::Empty());
      } else if (hold_s >= com_short_press_max_s_) {
        RCLCPP_INFO(get_logger(),
                    "COM btn held %.1fs → SET_HOME (save position)", hold_s);
        com_set_home_pub_->publish(std_msgs::msg::Empty());
      } else {
        com_mode_active_ = !com_mode_active_;
        RCLCPP_INFO(get_logger(), "COM motor mode: %s",
                    com_mode_active_ ? "ON (right stick → motor)" : "OFF (right stick → articulation)");
      }
    }

    com_btn_was_pressed_ = btn_down;
  }

  if (
    enable_com_direction_switch_ &&
    com_direction_toggle_button_index_ >= 0 &&
    joystick_state_.button_rising(static_cast<std::size_t>(com_direction_toggle_button_index_)))
  {
    com_direction_sign_ = com_direction_sign_ < 0.0 ? 1.0 : -1.0;
    publish_com_direction_sign();
    RCLCPP_WARN(
      get_logger(),
      "COM direction sign toggled from joystick: %.0f", com_direction_sign_);
  }

  if (
    com_quick_park_button_index_ >= 0 &&
    joystick_state_.button_rising(static_cast<std::size_t>(com_quick_park_button_index_)))
  {
    RCLCPP_INFO(get_logger(), "Quick PARK button pressed → return to home");
    com_park_pub_->publish(std_msgs::msg::Empty());
  }

  float linear_axis = joystick_state_.axis_value(static_cast<std::size_t>(linear_axis_index_));
  if (invert_linear_axis_) {
    linear_axis = -linear_axis;
  }
  linear_axis = static_cast<float>(JoystickState::shape_axis(linear_axis, linear_deadband_, linear_expo_));

  float angular_axis = joystick_state_.axis_value(static_cast<std::size_t>(angular_axis_index_));
  if (invert_angular_axis_) {
    angular_axis = -angular_axis;
  }
  angular_axis = static_cast<float>(JoystickState::shape_axis(angular_axis, angular_deadband_, angular_expo_));

  // R3 articulation inhibit: zero the right stick for ALL modes (articulation AND COM motor).
  // selected_actuator_axis is captured AFTER this so COM steer is also zeroed when inhibited.
  if (articulation_axis_inhibit_) {
    angular_axis = 0.0F;
  }

  float selected_actuator_axis = angular_axis;

  // In COM mode the right stick no longer drives articulation. The COM
  // command itself is published after deadman/e-stop evaluation below.
  if (enable_com_mode_switch_ && com_mode_active_) {
    angular_axis = 0.0F;
  }

  // Fail-safe calibrated brake mapping.
  // The default for a missing axis entry is brake_axis_released_ so that a
  // disconnected controller or a freshly-booted joy_linux node (with
  // default_trig_val=1.0) always reads "brake released" → brake_value = 0.
  const float raw_brake_axis = joystick_state_.axis_value(
    static_cast<std::size_t>(brake_axis_index_),
    static_cast<float>(brake_axis_released_));
  // Linear map: 0.0 = fully released, 1.0 = fully pressed.
  // Works for both normal (released=+1, pressed=-1) and inverted controllers
  // by adjusting the two params; no code change needed.
  const float brake_range = static_cast<float>(brake_axis_released_ - brake_axis_pressed_);
  const float brake_value = (brake_range > 1e-3F)
    ? std::clamp((static_cast<float>(brake_axis_released_) - raw_brake_axis) / brake_range,
                 0.0F, 1.0F)
    : 0.0F;  // degenerate config → safe default (no brake)

  // --- RT Auto-Hold Logic (1s hold) ---
  // Holding RT fully pressed for 1s auto-engages the parking brake.
  // When RT is released, the parking brake is automatically released
  // ONLY if it was auto-engaged by this mechanism (not a manual engage path).
  if (brake_value > 0.95f) {
    if (!rt_is_fully_pressed_) {
      rt_is_fully_pressed_ = true;
      rt_press_start_time_ = now();
    } else if (!parking_brake_mode_) {
      const auto duration = (now() - rt_press_start_time_).seconds();
      if (duration >= 1.0) {
        parking_brake_mode_ = true;
        rt_auto_engaged_ = true;  // remember: auto-engaged, not manual
        RCLCPP_INFO(get_logger(), "RT Held 1s: Parking Brake AUTO-ENGAGED");
      }
    }
  } else {
    rt_is_fully_pressed_ = false;
    // Auto-release parking brake when RT is released (only if auto-engaged by this mechanism).
    if (parking_brake_mode_ && rt_auto_engaged_) {
      parking_brake_mode_ = false;
      rt_auto_engaged_ = false;
      RCLCPP_INFO(get_logger(), "RT released: Parking Brake AUTO-RELEASED");
    }
  }

  const bool estop_active =
    trigger_pressed(joystick_state_, estop_left_trigger_axis_) &&
    trigger_pressed(joystick_state_, estop_right_trigger_axis_);

  // --- Deadman Safety Logic ---
  if (deadman_rising) {
    if (std::abs(linear_axis) > 0.01f || std::abs(selected_actuator_axis) > 0.01f) {
      movement_inhibited_ = true;
      RCLCPP_WARN(get_logger(), "Deadman pressed while joystick not at center! Movement inhibited until centered.");
    } else {
      movement_inhibited_ = false;
    }
  }
  if (!deadman_pressed) {
    movement_inhibited_ = false;
    // Clear articulation inhibit on deadman release to avoid latent surprises after re-press.
    if (articulation_axis_inhibit_) {
      articulation_axis_inhibit_ = false;
      RCLCPP_INFO(get_logger(), "Articulation Inhibit (R3): auto-cleared on deadman release.");
    }
  }
  if (movement_inhibited_) {
    if (std::abs(linear_axis) < 0.001f && std::abs(selected_actuator_axis) < 0.001f) {
      movement_inhibited_ = false;
      RCLCPP_INFO(get_logger(), "Joystick centered. Movement re-enabled.");
    }
  }

  update_ice_com_shift_experiment(
    linear_axis,
    deadman_pressed && !estop_active && !movement_inhibited_,
    brake_value);
  if (ice_com_shift_active_) {
    linear_axis = ice_com_shift_parking_home_
      ? 0.0F
      : static_cast<float>(ice_com_shift_phase_sign_);
    selected_actuator_axis = static_cast<float>(
      ice_com_shift_parking_home_ ? 0.0 : ice_com_shift_com_axis());
    angular_axis = 0.0F;
  }

  double linear_command = max_linear_speed_ * linear_axis;
  double angular_command = max_angular_command_ * angular_axis;
  last_angular_axis_ = angular_axis;
  if (ice_com_shift_active_ && !ice_com_shift_parking_home_) {
    linear_command = static_cast<double>(ice_com_shift_phase_sign_) *
      std::clamp(ice_com_shift_speed_ms_, 0.0, max_linear_speed_);
  } else if (ice_com_shift_active_) {
    linear_command = 0.0;
  }

  // --- Brake Priority Logic (monotonic linear ramp) ---
  // Single curve: attenuates linearly to zero at brake_full_fraction.
  // Brake can only reduce |linear|, never increase it or flip sign.
  // No discontinuity, no un-gating: pressing RT always makes things slower.
  if (enable_brake_axis_ && brake_value > 0.01F) {
    const double attenuation = std::clamp(
      1.0 - static_cast<double>(brake_value) / std::max(brake_full_fraction_, 1e-3),
      0.0,
      1.0);
    linear_command *= attenuation;
  }

  if (movement_inhibited_) {
    linear_command = 0.0;
    angular_command = 0.0;
  }
  if (parking_brake_mode_) {
    linear_command = 0.0;
  }

  if (!deadman_pressed || estop_active) {
    linear_command = 0.0;
    angular_command = 0.0;
  }

  if (enable_com_mode_switch_) {
    publish_com_state(
      selected_actuator_axis,
      deadman_pressed && !estop_active && !movement_inhibited_,
      ice_com_shift_active_);
  }

  const bool manual_activity =
    deadman_pressed &&
    !estop_active &&
    (std::abs(linear_command) > manual_activity_linear_threshold_ ||
     std::abs(angular_command) > manual_activity_angular_threshold_);

  auto deadman_msg = std_msgs::msg::Bool();
  deadman_msg.data = deadman_pressed;
  deadman_pub_->publish(deadman_msg);

  auto estop_msg = std_msgs::msg::Bool();
  estop_msg.data = estop_active;
  estop_pub_->publish(estop_msg);

  auto activity_msg = std_msgs::msg::Bool();
  activity_msg.data = manual_activity;
  manual_activity_pub_->publish(activity_msg);
  publish_articulation_hold_state(false);

  mtt_msgs::msg::MttAuxCommand aux_msg;
  aux_msg.light_state = light_state_;
  if (enable_brake_axis_ && !estop_active) {
    if (parking_brake_mode_) {
      aux_msg.brake = 1.0F;
    } else {
      aux_msg.brake = brake_value;
    }
  } else {
    aux_msg.brake = 0.0F;
  }
  aux_pub_->publish(aux_msg);

  geometry_msgs::msg::TwistStamped manual_msg;
  manual_msg.header.stamp = now();
  if (deadman_pressed && !estop_active) {
    manual_msg.twist.linear.x = linear_command;
    if (use_software_pid_) {
      manual_msg.twist.angular.z = 0.0; // Servo node handles steering
    } else {
      manual_msg.twist.angular.z = angular_command;
    }
  }

  if (deadman_pressed || deadman_released || estop_active) {
    manual_raw_pub_->publish(manual_msg);

    if (use_software_pid_ && manual_articulation_owned()) {
      if (deadman_pressed && !estop_active && !movement_inhibited_) {
        std_msgs::msg::Float64 cmd_msg;
        if (current_steering_routing_ == SteeringRoutingMode::POSITION_RETURN) {
          cmd_msg.data = angular_axis * max_joystick_position_rad_;
          position_cmd_pub_->publish(cmd_msg);
        } else if (current_steering_routing_ == SteeringRoutingMode::VELOCITY_HOLD) {
          cmd_msg.data = angular_axis * max_joystick_velocity_rad_s_;
          velocity_cmd_pub_->publish(cmd_msg);
        }
      } else {
        // If deadman is released, force return to zero in position mode, or zero velocity in hold mode
        std_msgs::msg::Float64 cmd_msg;
        cmd_msg.data = 0.0;
        if (current_steering_routing_ == SteeringRoutingMode::POSITION_RETURN) {
          position_cmd_pub_->publish(cmd_msg);
        } else if (current_steering_routing_ == SteeringRoutingMode::VELOCITY_HOLD) {
          velocity_cmd_pub_->publish(cmd_msg);
        }
      }
    }
  }

  previous_deadman_pressed_ = deadman_pressed;
}

void MttOperatorInputNode::on_watchdog()
{
  if (!joy_received_) {
    publish_safe_stop();
    return;
  }

  const double age_s = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - last_joy_receive_time_).count();

  if (age_s <= joy_timeout_s_) {
    // If the joystick is alive but not moving, joy_node might not publish.
    // We MUST continuously publish the servo commands at 50Hz so the servo node doesn't timeout!
    if (use_software_pid_ && manual_articulation_owned() &&
        previous_deadman_pressed_ && !movement_inhibited_) {
      std_msgs::msg::Float64 cmd_msg;
      if (current_steering_routing_ == SteeringRoutingMode::POSITION_RETURN) {
        cmd_msg.data = last_angular_axis_ * max_joystick_position_rad_;
        position_cmd_pub_->publish(cmd_msg);
      } else if (current_steering_routing_ == SteeringRoutingMode::VELOCITY_HOLD) {
        cmd_msg.data = last_angular_axis_ * max_joystick_velocity_rad_s_;
        velocity_cmd_pub_->publish(cmd_msg);
      }
    }
    return;
  }

  if (!joy_timeout_reported_) {
    RCLCPP_ERROR(
      get_logger(),
      "Joystick timeout after %.3f s. Forcing deadman and all manual commands to zero.",
      age_s);
    joy_timeout_reported_ = true;
  }
  publish_safe_stop();
}

void MttOperatorInputNode::publish_safe_stop()
{
  previous_deadman_pressed_ = false;
  movement_inhibited_ = false;
  rt_is_fully_pressed_ = false;
  reset_ice_com_shift_experiment("safe_stop");

  geometry_msgs::msg::TwistStamped manual_msg;
  manual_msg.header.stamp = now();
  manual_raw_pub_->publish(manual_msg);

  auto false_msg = std_msgs::msg::Bool();
  false_msg.data = false;
  deadman_pub_->publish(false_msg);
  estop_pub_->publish(false_msg);
  manual_activity_pub_->publish(false_msg);
  publish_articulation_hold_state(false);

  if (enable_com_mode_switch_) {
    publish_com_state(0.0F, false);
  }

  if (use_software_pid_ && manual_articulation_owned()) {
    std_msgs::msg::Float64 cmd_msg;
    cmd_msg.data = 0.0;
    if (current_steering_routing_ == SteeringRoutingMode::POSITION_RETURN) {
      position_cmd_pub_->publish(cmd_msg);
    } else if (current_steering_routing_ == SteeringRoutingMode::VELOCITY_HOLD) {
      velocity_cmd_pub_->publish(cmd_msg);
    }
  }
}

void MttOperatorInputNode::publish_com_state(
  float shaped_angular_axis,
  bool command_enabled,
  bool force_com_mode)
{
  const bool com_mode_for_output = com_mode_active_ || force_com_mode;

  // com_mode
  auto mode_msg = std_msgs::msg::Bool();
  mode_msg.data = com_mode_for_output;
  com_mode_pub_->publish(mode_msg);

  // com_steer: forward the already-shaped angular axis value.

  auto steer_msg = std_msgs::msg::Float64();
  steer_msg.data = com_mode_for_output && command_enabled
    ? static_cast<double>(invert_com_steer_ ? -shaped_angular_axis : shaped_angular_axis)
    : 0.0;
  com_steer_pub_->publish(steer_msg);
}

void MttOperatorInputNode::publish_com_direction_sign()
{
  auto sign_msg = std_msgs::msg::Float64();
  sign_msg.data = com_direction_sign_;
  com_direction_sign_pub_->publish(sign_msg);
}

void MttOperatorInputNode::on_ice_slip_ratio(const std_msgs::msg::Float64::SharedPtr msg)
{
  if (!std::isfinite(msg->data)) {
    return;
  }
  ice_com_shift_last_slip_ratio_ = msg->data;
  ice_com_shift_last_slip_time_ = now();
  ice_com_shift_has_slip_ = true;
}

void MttOperatorInputNode::on_ice_com_shift_arm(const std_msgs::msg::Bool::SharedPtr msg)
{
  ice_com_shift_armed_ = msg->data;
  ice_com_shift_last_arm_time_ = now();
}

bool MttOperatorInputNode::ice_com_shift_arm_fresh()
{
  return ice_com_shift_armed_ &&
         ice_com_shift_last_arm_time_.nanoseconds() != 0 &&
         (now() - ice_com_shift_last_arm_time_).seconds() <= ice_com_shift_arm_timeout_s_;
}

bool MttOperatorInputNode::ice_com_shift_slip_fresh()
{
  return ice_com_shift_has_slip_ &&
         ice_com_shift_last_slip_time_.nanoseconds() != 0 &&
         (now() - ice_com_shift_last_slip_time_).seconds() <= ice_com_shift_slip_timeout_s_;
}

double MttOperatorInputNode::ice_com_shift_com_axis()
{
  const double side = static_cast<double>(
    ice_com_shift_invert_com_axis_ ? -ice_com_shift_phase_sign_ : ice_com_shift_phase_sign_);
  if (!ice_com_shift_jitter_enabled_ || ice_com_shift_parking_home_) {
    return side;
  }

  const double amplitude = std::clamp(ice_com_shift_jitter_amplitude_, 0.0, 1.0);
  const double frequency_hz = std::max(0.0, ice_com_shift_jitter_frequency_hz_);
  if (amplitude <= 1e-6 || frequency_hz <= 1e-6 ||
      ice_com_shift_phase_start_time_.nanoseconds() == 0) {
    return side;
  }

  const double phase_age_s = std::max(0.0, (now() - ice_com_shift_phase_start_time_).seconds());
  const double inward_wave = 0.5 * (1.0 - std::cos(2.0 * M_PI * frequency_hz * phase_age_s));
  const double magnitude = std::clamp(1.0 - amplitude * inward_wave, 0.0, 1.0);
  return side * magnitude;
}

void MttOperatorInputNode::reset_ice_com_shift_experiment(const std::string & phase)
{
  if (ice_com_shift_active_) {
    RCLCPP_WARN(get_logger(), "Ice COM-shift experiment stopped: %s", phase.c_str());
    if (ice_com_shift_park_home_on_stop_) {
      com_park_pub_->publish(std_msgs::msg::Empty());
    }
  }
  ice_com_shift_active_ = false;
  ice_com_shift_parking_home_ = false;
  ice_com_shift_brake_latched_ = false;
  ice_com_shift_phase_ = phase;
  ice_com_shift_below_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  publish_ice_com_shift_state();
}

void MttOperatorInputNode::update_ice_com_shift_experiment(
  float linear_axis,
  bool command_enabled,
  double brake_value)
{
  if (!enable_ice_com_shift_experiment_) {
    return;
  }

  const bool armed = ice_com_shift_arm_fresh();
  const bool slip_fresh = ice_com_shift_slip_fresh();
  if (!armed || !command_enabled || !slip_fresh) {
    reset_ice_com_shift_experiment(!armed ? "disarmed" : (!command_enabled ? "deadman_off" : "waiting_slip"));
    return;
  }

  const rclcpp::Time now_stamp = now();
  if (!ice_com_shift_active_) {
    if (std::abs(linear_axis) < ice_com_shift_start_axis_threshold_) {
      ice_com_shift_phase_ = "armed_push_stick_to_start";
      publish_ice_com_shift_state();
      return;
    }
    ice_com_shift_active_ = true;
    ice_com_shift_phase_sign_ = linear_axis >= 0.0F ? 1 : -1;
    ice_com_shift_phase_start_time_ = now_stamp;
    ice_com_shift_below_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    ice_com_shift_phase_ = ice_com_shift_phase_sign_ > 0 ? "forward_limit_pos" : "reverse_limit_neg";
    com_direction_sign_ = 1.0;
    publish_com_direction_sign();
    if (ice_com_shift_park_home_on_start_) {
      request_ice_com_shift_park_home(
        ice_com_shift_phase_sign_ > 0 ? "park_home_before_forward" : "park_home_before_reverse");
    }
    RCLCPP_WARN(
      get_logger(),
      "Ice COM-shift experiment started: speed=%.2f m/s sign=%d slip=%.3f",
      ice_com_shift_speed_ms_, ice_com_shift_phase_sign_, ice_com_shift_last_slip_ratio_);
    publish_ice_com_shift_state();
    return;
  }

  if (ice_com_shift_parking_home_) {
    if (now_stamp < ice_com_shift_park_until_) {
      publish_ice_com_shift_state();
      return;
    }
    ice_com_shift_parking_home_ = false;
    ice_com_shift_phase_start_time_ = now_stamp;
    ice_com_shift_below_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    ice_com_shift_phase_ = ice_com_shift_phase_sign_ > 0 ? "forward_limit_pos" : "reverse_limit_neg";
    RCLCPP_WARN(
      get_logger(),
      "Ice COM-shift park-home window done: drive sign=%d", ice_com_shift_phase_sign_);
  }

  const double phase_age_s = (now_stamp - ice_com_shift_phase_start_time_).seconds();
  const bool min_phase_elapsed = phase_age_s >= ice_com_shift_min_phase_s_;
  const bool max_phase_elapsed =
    ice_com_shift_max_phase_s_ > 0.0 && phase_age_s >= ice_com_shift_max_phase_s_;
  const bool brake_switch_requested =
    ice_com_shift_switch_on_brake_ &&
    brake_value >= ice_com_shift_brake_switch_threshold_;

  if (!brake_switch_requested) {
    ice_com_shift_brake_latched_ = false;
  }
  if (min_phase_elapsed && brake_switch_requested && !ice_com_shift_brake_latched_) {
    ice_com_shift_brake_latched_ = true;
    switch_ice_com_shift_phase("brake_switch");
    publish_ice_com_shift_state();
    return;
  }

  const bool slip_below_threshold =
    std::abs(ice_com_shift_last_slip_ratio_) <= ice_com_shift_switch_abs_slip_below_;

  if (slip_below_threshold) {
    if (ice_com_shift_below_since_.nanoseconds() == 0) {
      ice_com_shift_below_since_ = now_stamp;
    }
  } else {
    ice_com_shift_below_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

  const bool below_hold_elapsed =
    ice_com_shift_below_since_.nanoseconds() != 0 &&
    (now_stamp - ice_com_shift_below_since_).seconds() >= ice_com_shift_switch_hold_s_;

  if (min_phase_elapsed && (below_hold_elapsed || max_phase_elapsed)) {
    switch_ice_com_shift_phase(below_hold_elapsed ? "slip_below_threshold" : "max_phase");
  }

  publish_ice_com_shift_state();
}

void MttOperatorInputNode::request_ice_com_shift_park_home(const std::string & phase)
{
  ice_com_shift_parking_home_ = true;
  ice_com_shift_phase_ = phase;
  ice_com_shift_park_until_ = now() +
    rclcpp::Duration::from_seconds(std::max(0.0, ice_com_shift_park_home_s_));
  com_park_pub_->publish(std_msgs::msg::Empty());
  RCLCPP_WARN(get_logger(), "Ice COM-shift PARK home requested: %s", phase.c_str());
}

void MttOperatorInputNode::switch_ice_com_shift_phase(const std::string & reason)
{
  ice_com_shift_phase_sign_ *= -1;
  ice_com_shift_phase_start_time_ = now();
  ice_com_shift_below_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  ice_com_shift_phase_ = ice_com_shift_phase_sign_ > 0 ? "forward_limit_pos" : "reverse_limit_neg";
  RCLCPP_WARN(
    get_logger(),
    "Ice COM-shift switch: speed sign=%d com limit=%d slip=%.3f reason=%s",
    ice_com_shift_phase_sign_, ice_com_shift_phase_sign_, ice_com_shift_last_slip_ratio_,
    reason.c_str());
}

void MttOperatorInputNode::publish_ice_com_shift_state()
{
  auto active_msg = std_msgs::msg::Bool();
  active_msg.data = ice_com_shift_active_;
  ice_com_shift_active_pub_->publish(active_msg);

  auto armed_msg = std_msgs::msg::Bool();
  armed_msg.data = enable_ice_com_shift_experiment_ && ice_com_shift_arm_fresh();
  ice_com_shift_armed_pub_->publish(armed_msg);

  auto phase_msg = std_msgs::msg::String();
  phase_msg.data = ice_com_shift_phase_;
  ice_com_shift_phase_pub_->publish(phase_msg);
}

}  // namespace mtt_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mtt_control::MttOperatorInputNode>());
  rclcpp::shutdown();
  return 0;
}
