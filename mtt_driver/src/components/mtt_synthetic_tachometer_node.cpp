// MTT synthetic tachometer node.
// Mirrors the driver-side cmd_sim contract so simulation, health fallback,
// and bag-based motion-model validation all share the same semantics.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <limits>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <mtt_msgs/msg/mtt_aux_command.hpp>
#include <mtt_msgs/msg/mtt_tachometer_data.hpp>
#include <mtt_msgs/msg/mtt_vehicle_status.hpp>

#include "mtt_driver/logic/can_frame_codec.hpp"
#include "mtt_driver/logic/command_motion_model.hpp"
#include "mtt_driver/logic/hold_assist.hpp"
#include "mtt_driver/logic/vehicle_params.hpp"

namespace mtt {

class MttSyntheticTachometerNode : public rclcpp::Node {
public:
  explicit MttSyntheticTachometerNode(const rclcpp::NodeOptions& options)
  : rclcpp::Node("mtt_synthetic_tachometer", options)
  {
    cmd_vel_topic_ = declare_parameter("cmd_vel_topic", std::string("cmd_vel"));
    tacho_topic_ = declare_parameter("tacho_topic", std::string("mtt_tachometer"));
    publish_rate_hz_ = declare_parameter("publish_rate_hz", 50.0);
    command_timeout_s_ = declare_parameter("command_timeout_seconds", 0.5);
    max_linear_speed_ms_ = declare_parameter("max_linear_speed_ms", 1.0);
    cmd_angular_mode_ = declare_parameter("cmd_angular_mode", std::string("normalized_steer"));
    status_topic_ = declare_parameter("status_topic", std::string("mtt_status"));
    motion_model_params_.wheelbase_m =
      declare_parameter("model_wheelbase_m", VehicleParams::total_wheelbase());
    motion_model_params_.max_articulation_rad =
      declare_parameter("model_max_articulation_deg", VehicleParams::max_articulation_deg) * M_PI / 180.0;
    motion_model_params_.min_turn_speed_ms =
      declare_parameter("model_min_turn_speed_ms", VehicleParams::min_speed_for_steering);
    motion_model_params_.speed_response_gain = declare_parameter("model_speed_response_gain", 3.0);
    motion_model_params_.articulation_response_gain =
      declare_parameter("model_articulation_response_gain", VehicleParams::articulation_response);
    motion_model_params_.brake_gain = declare_parameter("model_brake_gain", 1.0);
    motion_model_params_.use_slip_heuristic = declare_parameter("model_use_slip_heuristic", true);
    motion_model_params_.yaw_slip_base = declare_parameter("model_yaw_slip_base", 0.10);
    motion_model_params_.yaw_slip_speed_gain = declare_parameter("model_yaw_slip_speed_gain", 0.05);
    motion_model_params_.yaw_slip_articulation_gain =
      declare_parameter("model_yaw_slip_articulation_gain", 0.15);
    motion_model_params_.yaw_slip_min_scale =
      declare_parameter("model_yaw_slip_min_scale", 0.55);
    hold_assist_params_.enabled = declare_parameter("hold_assist_enabled", true);
    hold_assist_params_.entry_speed_ms = declare_parameter("hold_assist_entry_speed_ms", 0.08);
    hold_assist_params_.exit_command_ms = declare_parameter("hold_assist_exit_command_ms", 0.08);
    hold_assist_params_.exit_speed_ms = declare_parameter("hold_assist_exit_speed_ms", 0.15);
    hold_assist_params_.kp = declare_parameter("hold_assist_kp", 2.5);
    hold_assist_params_.output_limit = declare_parameter("hold_assist_output_limit", 0.40);

    if (cmd_angular_mode_ != "normalized_steer" && cmd_angular_mode_ != "yaw_rate") {
      RCLCPP_WARN(
        get_logger(),
        "Unknown cmd_angular_mode '%s', falling back to 'normalized_steer'",
        cmd_angular_mode_.c_str());
      cmd_angular_mode_ = "normalized_steer";
    }

    motion_model_.set_params(motion_model_params_);
    hold_assist_controller_.set_params(hold_assist_params_);
    synthetic_frame_.init_defaults();

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      cmd_vel_topic_, 10,
      [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) { on_cmd_vel(msg); });
    aux_cmd_sub_ = create_subscription<mtt_msgs::msg::MttAuxCommand>(
      "mtt_aux_cmd", 10,
      [this](const mtt_msgs::msg::MttAuxCommand::SharedPtr msg) { on_aux_cmd(msg); });

    tacho_pub_ = create_publisher<mtt_msgs::msg::MttTachometerData>(
      tacho_topic_, rclcpp::SensorDataQoS());
    status_pub_ = create_publisher<mtt_msgs::msg::MttVehicleStatus>(status_topic_, 10);
    articulation_cmd_pub_ = create_publisher<std_msgs::msg::Float64>(
      "mtt/articulation_cmd", rclcpp::SensorDataQoS());

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { publish_synthetic_tachometer(); });

    RCLCPP_INFO(
      get_logger(),
      "MttSyntheticTachometerNode started (cmd_vel=%s, cmd_angular_mode=%s, rate=%.1f Hz)",
      cmd_vel_topic_.c_str(),
      cmd_angular_mode_.c_str(),
      publish_rate_hz_);
  }

private:
  std::string cmd_vel_topic_;
  std::string tacho_topic_;
  std::string status_topic_;
  std::string cmd_angular_mode_;
  double publish_rate_hz_{50.0};
  double command_timeout_s_{0.5};
  double max_linear_speed_ms_{1.0};
  logic::CommandMotionParams motion_model_params_{};
  logic::HoldAssistParams hold_assist_params_{};
  logic::CommandMotionModel motion_model_{};
  logic::HoldAssistController hold_assist_controller_{};
  double current_linear_command_ms_{0.0};
  double effective_linear_command_ms_{0.0};
  double current_steer_cmd_{0.0};
  double current_brake_norm_{0.0};
  bool has_cmd_vel_{false};
  std::chrono::steady_clock::time_point last_cmd_vel_time_{};
  std::chrono::steady_clock::time_point last_publish_time_{};
  bool publish_initialized_{false};
  can::CommandFrame synthetic_frame_{};

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<mtt_msgs::msg::MttAuxCommand>::SharedPtr aux_cmd_sub_;
  rclcpp::Publisher<mtt_msgs::msg::MttTachometerData>::SharedPtr tacho_pub_;
  rclcpp::Publisher<mtt_msgs::msg::MttVehicleStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr articulation_cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void on_cmd_vel(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    const double linear_cmd = std::clamp(
      msg->twist.linear.x,
      -max_linear_speed_ms_,
      max_linear_speed_ms_);
    current_linear_command_ms_ = linear_cmd;
    current_steer_cmd_ = command_angular_to_normalized_steer(
      linear_cmd,
      msg->twist.angular.z);
    last_cmd_vel_time_ = std::chrono::steady_clock::now();
    has_cmd_vel_ = true;
  }

  void on_aux_cmd(const mtt_msgs::msg::MttAuxCommand::SharedPtr msg)
  {
    current_brake_norm_ = std::clamp(static_cast<double>(msg->brake), 0.0, 1.0);
  }

  double command_angular_to_normalized_steer(double linear_x, double angular_z) const
  {
    if (cmd_angular_mode_ == "yaw_rate") {
      auto params = motion_model_params_;
      params.min_turn_speed_ms = motion_model_params_.min_turn_speed_ms;
      return logic::CommandMotionModel::normalized_steer_from_yaw_rate(
        angular_z,
        linear_x,
        params);
    }

    return std::clamp(angular_z, -1.0, 1.0);
  }

  bool cmd_vel_is_fresh(const std::chrono::steady_clock::time_point& now_tp) const
  {
    if (!has_cmd_vel_) {
      return false;
    }
    return std::chrono::duration<double>(now_tp - last_cmd_vel_time_).count() <= command_timeout_s_;
  }

  bool stamp_clock_ready()
  {
    bool use_sim_time = false;
    (void)get_parameter("use_sim_time", use_sim_time);
    if (!use_sim_time) {
      return true;
    }
    const auto stamp = now();
    return stamp.nanoseconds() > 0 && stamp.seconds() < 1.0e8;
  }

  void publish_synthetic_tachometer()
  {
    if (!stamp_clock_ready()) {
      return;
    }

    const auto wall_now = std::chrono::steady_clock::now();
    double dt = 1.0 / std::max(1.0, publish_rate_hz_);
    if (publish_initialized_) {
      dt = std::chrono::duration<double>(wall_now - last_publish_time_).count();
    } else {
      publish_initialized_ = true;
    }
    last_publish_time_ = wall_now;
    dt = std::clamp(dt, 0.0, 1.0);

    const bool fresh = cmd_vel_is_fresh(wall_now);
    logic::HoldAssistInput hold_input;
    hold_input.dt = dt;
    hold_input.commanded_speed_ms = fresh ? current_linear_command_ms_ : 0.0;
    hold_input.measured_speed_ms = motion_model_.state().v_eff_ms;
    hold_input.telemetry_fresh = true;
    hold_input.safety_locked = false;
    hold_input.brake_normalized = current_brake_norm_;
    const auto& hold_output = hold_assist_controller_.update(hold_input);
    effective_linear_command_ms_ = std::clamp(
      hold_input.commanded_speed_ms + hold_output.correction_speed_ms,
      -max_linear_speed_ms_,
      max_linear_speed_ms_);

    logic::CommandMotionCommand motion_command;
    motion_command.linear_speed_cmd_ms = effective_linear_command_ms_;
    motion_command.normalized_steer_cmd = fresh ? current_steer_cmd_ : 0.0;
    motion_command.brake_normalized = current_brake_norm_;
    motion_command.dt = dt;
    const auto& state = motion_model_.step(motion_command);

    mtt_msgs::msg::MttTachometerData tacho;
    tacho.header.stamp = now();
    tacho.header.frame_id = "base_footprint";
    tacho.telemetry_can_id = 0x2FFu;
    tacho.telemetry_seen_once = true;
    tacho.telemetry_fresh = true;
    tacho.telemetry_age_ms = 0.0;
    tacho.tachometer_is_synthetic = true;
    tacho.tachometer_source = "cmd_sim";
    tacho.main_sensor_temp_a = 0.0;
    tacho.main_sensor_temp_b = 0.0;

    constexpr double encoder_ratio = VehicleParams::encoder_final_ratio();
    constexpr double track_length_m = VehicleParams::track_length_m;
    const double speed_abs_ms = std::abs(state.v_eff_ms);
    const double instant_rps = speed_abs_ms * encoder_ratio / std::max(track_length_m, 1e-9);
    const double cumulative_ticks = state.cumulative_distance_m * encoder_ratio / std::max(track_length_m, 1e-9);
    tacho.tachometer_instant = static_cast<uint16_t>(std::clamp(
      std::llround(instant_rps),
      0LL,
      static_cast<long long>(std::numeric_limits<uint16_t>::max())));
    tacho.tachometer_cumulative = static_cast<uint32_t>(std::clamp(
      std::llround(cumulative_ticks),
      0LL,
      static_cast<long long>(std::numeric_limits<uint32_t>::max())));

    tacho.speed_ms = state.v_eff_ms;
    tacho.speed_kmh = state.v_eff_ms * 3.6;
    tacho.distance_km = state.cumulative_distance_m / 1000.0;
    if (std::abs(state.v_eff_ms) > 1e-4) {
      tacho.direction = state.v_eff_ms < 0.0 ? "Reverse" : "Forward";
    } else {
      tacho.direction = motion_command.linear_speed_cmd_ms < 0.0 ? "Reverse" : "Forward";
    }
    tacho.steer_cmd = motion_command.normalized_steer_cmd;

    tacho.model_state_valid = true;
    tacho.model_command_linear_speed_ms = state.v_command_ms;
    tacho.model_speed_ms = state.v_eff_ms;
    tacho.model_articulation_command_rad = state.phi_command_rad;
    tacho.model_articulation_effective_rad = state.phi_eff_rad;
    tacho.model_curvature_nominal_m_inv = state.kappa_nominal_m_inv;
    tacho.model_curvature_effective_m_inv = state.kappa_effective_m_inv;
    tacho.model_yaw_rate_nominal_rad_s = state.yaw_rate_nominal_rad_s;
    tacho.model_yaw_rate_effective_rad_s = state.yaw_rate_effective_rad_s;

    tacho_pub_->publish(tacho);

    std_msgs::msg::Float64 artic_cmd_msg;
    artic_cmd_msg.data = state.phi_command_rad;
    articulation_cmd_pub_->publish(artic_cmd_msg);

    const double throttle_norm =
      std::clamp(std::abs(effective_linear_command_ms_) / max_linear_speed_ms_, 0.0, 1.0);
    synthetic_frame_.set_throttle(throttle_norm);
    synthetic_frame_.set_brake(current_brake_norm_);
    synthetic_frame_.set_steer(motion_command.normalized_steer_cmd);
    synthetic_frame_.set_direction(
      effective_linear_command_ms_ >= 0.0 ? can::Direction::Forward : can::Direction::Reverse);

    mtt_msgs::msg::MttVehicleStatus status;
    status.header = tacho.header;
    status.can_interface = "synthetic";
    status.command_can_id = 0x001u;
    status.telemetry_can_id = 0x2FFu;
    status.telemetry_seen_once = true;
    status.telemetry_fresh = true;
    status.telemetry_age_ms = 0.0;
    status.tachometer_is_synthetic = true;
    status.tachometer_source = "cmd_sim";
    status.speed_ms = tacho.speed_ms;
    status.speed_kmh = tacho.speed_kmh;
    status.distance_km = tacho.distance_km;
    status.direction = tacho.direction;
    status.temperature_a = 0.0;
    status.temperature_b = 0.0;
    status.steer_position = synthetic_frame_.steer_raw();
    status.main_sensor_temp_a_raw = 0;
    status.main_sensor_temp_b_raw = 0;
    status.tachometer_instant = tacho.tachometer_instant;
    status.tachometer_cumulative = tacho.tachometer_cumulative;
    status.tachometer_instant_ticks_per_s = tacho.tachometer_instant;
    status.tachometer_cumulative_ticks = tacho.tachometer_cumulative;
    status.vehicle_type_raw = static_cast<uint8_t>(synthetic_frame_.get_vehicle_type());
    status.vehicle_type_label = can::vehicle_type_to_string(synthetic_frame_.get_vehicle_type());
    status.security_unlocked = true;
    status.light_off_estop_patch = false;
    status.direction_reverse = tacho.direction == "Reverse";
    status.throttle_raw = synthetic_frame_.throttle_raw();
    status.brake_raw = synthetic_frame_.brake_raw();
    status.steer_raw = synthetic_frame_.steer_raw();
    status.steer_normalized = motion_command.normalized_steer_cmd;
    status.command_linear_speed_ms = hold_input.commanded_speed_ms;
    status.effective_linear_speed_command_ms = effective_linear_command_ms_;
    status.hold_assist_active = hold_output.active;
    status.hold_assist_mode = hold_output.mode;
    status.hold_assist_output_ms = hold_output.correction_speed_ms;
    status.winch_raw = static_cast<uint8_t>(synthetic_frame_.get_winch());
    status.winch_state = can::winch_state_to_string(synthetic_frame_.get_winch());
    status.steering_mode_closed_loop = synthetic_frame_.get_steering_mode() == can::SteeringMode::CloseLoop;
    status.reserved_byte_7 = synthetic_frame_.reserved_raw();
    status.emergency_stop_active = false;
    status.remote_connected = has_cmd_vel_;
    status.deadman_active = false;
    status.command_timeout_active = !fresh;
    status.can_debug_enabled = false;
    status.safety_state = "SafetyUnlocked";
    status.has_main_controller_version = false;
    status.main_hardware_revision_raw = 0u;
    status.main_software_revision_raw = 0u;
    status.has_battery_controller_version = false;
    status.battery_hardware_revision_raw = 0u;
    status.battery_software_revision_raw = 0u;
    status_pub_->publish(status);
  }
};

}  // namespace mtt

RCLCPP_COMPONENTS_REGISTER_NODE(mtt::MttSyntheticTachometerNode)
