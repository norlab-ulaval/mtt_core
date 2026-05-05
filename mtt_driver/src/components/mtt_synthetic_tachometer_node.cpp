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
#include <mtt_msgs/msg/mtt_aux_command.hpp>
#include <mtt_msgs/msg/mtt_tachometer_data.hpp>

#include "mtt_driver/logic/command_motion_model.hpp"
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

    if (cmd_angular_mode_ != "normalized_steer" && cmd_angular_mode_ != "yaw_rate") {
      RCLCPP_WARN(
        get_logger(),
        "Unknown cmd_angular_mode '%s', falling back to 'normalized_steer'",
        cmd_angular_mode_.c_str());
      cmd_angular_mode_ = "normalized_steer";
    }

    motion_model_.set_params(motion_model_params_);

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      cmd_vel_topic_, 10,
      [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) { on_cmd_vel(msg); });
    aux_cmd_sub_ = create_subscription<mtt_msgs::msg::MttAuxCommand>(
      "mtt_aux_cmd", 10,
      [this](const mtt_msgs::msg::MttAuxCommand::SharedPtr msg) { on_aux_cmd(msg); });

    tacho_pub_ = create_publisher<mtt_msgs::msg::MttTachometerData>(
      tacho_topic_, rclcpp::SensorDataQoS());

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
  std::string cmd_angular_mode_;
  double publish_rate_hz_{50.0};
  double command_timeout_s_{0.5};
  double max_linear_speed_ms_{1.0};
  logic::CommandMotionParams motion_model_params_{};
  logic::CommandMotionModel motion_model_{};
  double current_linear_command_ms_{0.0};
  double current_steer_cmd_{0.0};
  double current_brake_norm_{0.0};
  bool has_cmd_vel_{false};
  std::chrono::steady_clock::time_point last_cmd_vel_time_{};
  std::chrono::steady_clock::time_point last_publish_time_{};
  bool publish_initialized_{false};

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<mtt_msgs::msg::MttAuxCommand>::SharedPtr aux_cmd_sub_;
  rclcpp::Publisher<mtt_msgs::msg::MttTachometerData>::SharedPtr tacho_pub_;
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

  void publish_synthetic_tachometer()
  {
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
    logic::CommandMotionCommand motion_command;
    motion_command.linear_speed_cmd_ms = fresh ? current_linear_command_ms_ : 0.0;
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
  }
};

}  // namespace mtt

RCLCPP_COMPONENTS_REGISTER_NODE(mtt::MttSyntheticTachometerNode)
