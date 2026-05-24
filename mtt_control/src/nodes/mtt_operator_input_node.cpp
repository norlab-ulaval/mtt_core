#include "mtt_control/nodes/mtt_operator_input_node.hpp"

#include <algorithm>
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
  brake_axis_default_ = declare_parameter("brake_axis_default", 1.0);
  articulation_hold_max_speed_ms_ = declare_parameter("articulation_hold_max_speed_ms", 0.75);
  articulation_hold_release_deadband_ = declare_parameter("articulation_hold_release_deadband", 0.08);
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
  articulation_hold_enabled_ = declare_parameter("articulation_hold_enabled", true);
  articulation_hold_reset_on_deadman_release_ =
    declare_parameter("articulation_hold_reset_on_deadman_release", true);
  articulation_hold_mode_default_ = declare_parameter("articulation_hold_mode_default", false);
  articulation_hold_mode_ = articulation_hold_mode_default_;
  enable_steering_mode_switch_ = declare_parameter("enable_steering_mode_switch", true);
  steer_mode_switch_button_index_ = declare_parameter("steer_mode_switch_button_index", 4);

  joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
    "joy",
    rclcpp::SensorDataQoS(),
    std::bind(&MttOperatorInputNode::on_joy, this, std::placeholders::_1));

  // Timer to keep publishing manual_raw at 50Hz (20ms) so the filter doesn't time out
  publish_timer_ = create_wall_timer(
    std::chrono::milliseconds(20),
    [this]() {
      auto msg = std::make_shared<sensor_msgs::msg::Joy>(joystick_state_.last_msg());
      if (msg->header.stamp.sec != 0) { // Only if we have received at least one msg
        on_joy(msg);
      }
    });

  manual_raw_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel/manual_raw", 20);
  aux_pub_ = create_publisher<mtt_msgs::msg::MttAuxCommand>("mtt_aux_cmd", 20);
  deadman_pub_ = create_publisher<std_msgs::msg::Bool>("mtt_control/teleop_deadman", 20);
  estop_pub_ = create_publisher<std_msgs::msg::Bool>("mtt_control/teleop_estop", 20);
  manual_activity_pub_ = create_publisher<std_msgs::msg::Bool>("mtt_control/manual_activity", 20);
  articulation_mode_pub_ =
    create_publisher<std_msgs::msg::String>("mtt_control/articulation_mode", 20);
  articulation_hold_active_pub_ =
    create_publisher<std_msgs::msg::Bool>("mtt_control/articulation_hold_active", 20);

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
  mode_msg.data = articulation_hold_mode_ ? "hold" : "return_to_zero";
  articulation_mode_pub_->publish(mode_msg);

  auto active_msg = std_msgs::msg::Bool();
  active_msg.data = hold_active;
  articulation_hold_active_pub_->publish(active_msg);
}

void MttOperatorInputNode::on_joy(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  joystick_state_.update(*msg);

  const bool deadman_pressed = joystick_state_.button_pressed(static_cast<std::size_t>(deadman_button_index_));
  const bool deadman_rising = deadman_pressed && !previous_deadman_pressed_;
  const bool deadman_released = previous_deadman_pressed_ && !deadman_pressed;
  
  const bool light_rising = joystick_state_.button_rising(static_cast<std::size_t>(light_button_index_));
  const bool articulation_hold_rising =
    articulation_hold_enabled_ &&
    joystick_state_.button_rising(static_cast<std::size_t>(articulation_hold_button_index_));
  const bool steer_mode_rising =
    enable_steering_mode_switch_ &&
    joystick_state_.button_rising(static_cast<std::size_t>(steer_mode_switch_button_index_));

  if (light_rising) {
    light_state_ = !light_state_;
  }

  // --- Parking Brake / Articulation Hold ---
  if (articulation_hold_rising) {
    parking_brake_mode_ = !parking_brake_mode_;
    articulation_hold_mode_ = parking_brake_mode_;
    RCLCPP_INFO(get_logger(), "Parking Brake: %s", parking_brake_mode_ ? "ENGAGED" : "RELEASED");
    if (!parking_brake_mode_) {
      has_held_angular_command_ = false;
      held_angular_command_ = 0.0;
    }
  }

  if (steer_mode_rising) {
    toggle_steering_mode();
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

  const float raw_brake_axis = joystick_state_.axis_value(
    static_cast<std::size_t>(brake_axis_index_),
    static_cast<float>(brake_axis_default_));
  const float brake_value = std::clamp((-raw_brake_axis + 1.0F) / 2.0F, 0.0F, 1.0F);

  // --- RT Auto-Hold Logic (1s hold) ---
  if (brake_value > 0.95f) {
    if (!rt_is_fully_pressed_) {
      rt_is_fully_pressed_ = true;
      rt_press_start_time_ = now();
    } else if (!parking_brake_mode_) {
      const auto duration = (now() - rt_press_start_time_).seconds();
      if (duration >= 1.0) {
        parking_brake_mode_ = true;
        articulation_hold_mode_ = true;
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
    if (std::abs(linear_axis) > 0.01f || std::abs(angular_axis) > 0.01f) {
      movement_inhibited_ = true;
      RCLCPP_WARN(get_logger(), "Deadman pressed while joystick not at center! Movement inhibited until centered.");
    } else {
      movement_inhibited_ = false;
      if (parking_brake_mode_) {
        parking_brake_mode_ = false;
        articulation_hold_mode_ = articulation_hold_mode_default_;
        RCLCPP_INFO(get_logger(), "Movement detected after Deadman: Parking Brake RELEASED");
      }
    }
  }
  if (!deadman_pressed) {
    movement_inhibited_ = false;
  }
  if (movement_inhibited_) {
    if (std::abs(linear_axis) < 0.001f && std::abs(angular_axis) < 0.001f) {
      movement_inhibited_ = false;
      RCLCPP_INFO(get_logger(), "Joystick centered. Movement re-enabled.");
    }
  }

  double linear_command = max_linear_speed_ * linear_axis;
  double angular_command = max_angular_command_ * angular_axis;

  // --- Brake Priority Logic ---
  if (brake_value > 0.05f) {
    if (brake_value > 0.50f) {
      linear_command = 0.0;
    } else {
      linear_command *= (1.0 - (brake_value * 2.0));
    }
  }

  if (movement_inhibited_ || parking_brake_mode_) {
    linear_command = 0.0;
    angular_command = 0.0;
  }

  bool articulation_hold_active = false;
  const bool deadman_active_for_arbiter = deadman_pressed || parking_brake_mode_;

  if (!deadman_active_for_arbiter || estop_active) {
    linear_command = 0.0;
    angular_command = 0.0;
    if (articulation_hold_reset_on_deadman_release_) {
      has_held_angular_command_ = false;
      held_angular_command_ = 0.0;
    }
  } else if (articulation_hold_enabled_ && articulation_hold_mode_) {
    if (std::abs(linear_command) <= articulation_hold_max_speed_ms_) {
      if (std::abs(angular_axis) > articulation_hold_release_deadband_) {
        held_angular_command_ = angular_command;
        has_held_angular_command_ = true;
      } else if (has_held_angular_command_) {
        angular_command = held_angular_command_;
        articulation_hold_active = std::abs(angular_command) > manual_activity_angular_threshold_;
      }
    } else if (has_held_angular_command_) {
      has_held_angular_command_ = false;
      held_angular_command_ = 0.0;
      RCLCPP_WARN(get_logger(), "Speed exceeded threshold! Articulation hold released.");
    }
  }

  const bool manual_activity =
    deadman_pressed &&
    !estop_active &&
    (std::abs(linear_command) > manual_activity_linear_threshold_ ||
     std::abs(angular_command) > manual_activity_angular_threshold_);

  auto deadman_msg = std_msgs::msg::Bool();
  deadman_msg.data = deadman_active_for_arbiter;
  deadman_pub_->publish(deadman_msg);

  auto estop_msg = std_msgs::msg::Bool();
  estop_msg.data = estop_active;
  estop_pub_->publish(estop_msg);

  auto activity_msg = std_msgs::msg::Bool();
  activity_msg.data = manual_activity;
  manual_activity_pub_->publish(activity_msg);
  publish_articulation_hold_state(articulation_hold_active);

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
  if (deadman_active_for_arbiter && !estop_active) {
    manual_msg.twist.linear.x = linear_command;
    manual_msg.twist.angular.z = angular_command;
  }

  if (deadman_active_for_arbiter || deadman_released || estop_active) {
    manual_raw_pub_->publish(manual_msg);
  }

  previous_deadman_pressed_ = deadman_pressed;
}

}  // namespace mtt_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mtt_control::MttOperatorInputNode>());
  rclcpp::shutdown();
  return 0;
}
