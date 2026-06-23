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
  com_short_press_max_s_       = declare_parameter("com_short_press_max_s",       1.5);
  com_long_press_min_s_        = declare_parameter("com_long_press_min_s",        4.0);

  joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
    "joy",
    rclcpp::SensorDataQoS(),
    std::bind(&MttOperatorInputNode::on_joy, this, std::placeholders::_1));

  manual_raw_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel/manual_raw", 20);
  aux_pub_ = create_publisher<mtt_msgs::msg::MttAuxCommand>("mtt_aux_cmd", 20);
  deadman_pub_ = create_publisher<std_msgs::msg::Bool>("mtt_control/teleop_deadman", 20);
  estop_pub_ = create_publisher<std_msgs::msg::Bool>("mtt_control/teleop_estop", 20);
  manual_activity_pub_ = create_publisher<std_msgs::msg::Bool>("mtt_control/manual_activity", 20);
  articulation_mode_pub_ =
    create_publisher<std_msgs::msg::String>("mtt_control/articulation_mode", 20);
  articulation_hold_active_pub_ =
    create_publisher<std_msgs::msg::Bool>("mtt_control/articulation_hold_active", 20);

  // COM motor publishers (always created so topics appear on the graph even when OFF)
  com_mode_pub_  = create_publisher<std_msgs::msg::Bool>  ("mtt_control/com_mode",  20);
  com_steer_pub_ = create_publisher<std_msgs::msg::Float64>("mtt_control/com_steer", 20);
  com_park_pub_  = create_publisher<std_msgs::msg::Empty> ("mtt_control/com_park",  10);
  com_set_home_pub_ = create_publisher<std_msgs::msg::Empty>("mtt_control/com_set_home", 10);

  publish_timer_ = create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&MttOperatorInputNode::on_watchdog, this));

  publish_articulation_hold_state(false);

  can_steer_mode_client_ = create_client<mtt_interfaces::srv::SetSteerControlMode>(
    "mtt/set_steer_control_mode");
  odom_steer_mode_client_ = create_client<mtt_interfaces::srv::SetSteerControlMode>(
    "mtt/odometry/set_steer_control_mode");
}

void MttOperatorInputNode::toggle_steering_mode()
{
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

  // Parking brake is independent from deadman and articulation.
  if (articulation_hold_rising) {
    parking_brake_mode_ = !parking_brake_mode_;
    RCLCPP_INFO(get_logger(), "Parking Brake: %s", parking_brake_mode_ ? "ENGAGED" : "RELEASED");
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
  const float selected_actuator_axis = angular_axis;

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
  if (brake_value > 0.95f) {
    if (!rt_is_fully_pressed_) {
      rt_is_fully_pressed_ = true;
      rt_press_start_time_ = now();
    } else if (!parking_brake_mode_) {
      const auto duration = (now() - rt_press_start_time_).seconds();
      if (duration >= 1.0) {
        parking_brake_mode_ = true;
        RCLCPP_INFO(get_logger(), "RT Held 1s: Parking Brake AUTO-ENGAGED");
      }
    }
  } else {
    rt_is_fully_pressed_ = false;
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
  }
  if (movement_inhibited_) {
    if (std::abs(linear_axis) < 0.001f && std::abs(selected_actuator_axis) < 0.001f) {
      movement_inhibited_ = false;
      RCLCPP_INFO(get_logger(), "Joystick centered. Movement re-enabled.");
    }
  }

  double linear_command = max_linear_speed_ * linear_axis;
  double angular_command = max_angular_command_ * angular_axis;

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
      deadman_pressed && !estop_active && !movement_inhibited_);
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
    manual_msg.twist.angular.z = angular_command;
  }

  if (deadman_pressed || deadman_released || estop_active) {
    manual_raw_pub_->publish(manual_msg);
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
}

void MttOperatorInputNode::publish_com_state(
  float shaped_angular_axis,
  bool command_enabled)
{
  // com_mode
  auto mode_msg = std_msgs::msg::Bool();
  mode_msg.data = com_mode_active_;
  com_mode_pub_->publish(mode_msg);

  // com_steer: forward the already-shaped angular axis value.

  auto steer_msg = std_msgs::msg::Float64();
  steer_msg.data = com_mode_active_ && command_enabled
    ? static_cast<double>(invert_com_steer_ ? -shaped_angular_axis : shaped_angular_axis)
    : 0.0;
  com_steer_pub_->publish(steer_msg);
}

}  // namespace mtt_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mtt_control::MttOperatorInputNode>());
  rclcpp::shutdown();
  return 0;
}
