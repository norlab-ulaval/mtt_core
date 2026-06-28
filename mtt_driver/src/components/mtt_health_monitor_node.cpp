#include "mtt_driver/components/mtt_health_monitor_node.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <sstream>
#include <utility>
#include <vector>

#include <rclcpp_components/register_node_macro.hpp>

#include "mtt_driver/logic/can_frame_codec.hpp"
#include "mtt_driver/logic/vehicle_params.hpp"

namespace mtt {

using namespace std::chrono_literals;

void MttHealthMonitorNode::FrameRateTracker::update(std::chrono::steady_clock::time_point now)
{
  samples.push_back(now);
  while (!samples.empty() &&
         std::chrono::duration<double>(now - samples.front()).count() > window_s) {
    samples.pop_front();
  }
}

float MttHealthMonitorNode::FrameRateTracker::hz(std::chrono::steady_clock::time_point now) const
{
  if (samples.size() < 2) {
    return 0.0F;
  }

  while (!samples.empty() &&
         std::chrono::duration<double>(now - samples.front()).count() > window_s) {
    // no-op: const method, caller is expected to call update periodically
    break;
  }

  const double dt = std::chrono::duration<double>(samples.back() - samples.front()).count();
  if (dt <= 1e-6) {
    return 0.0F;
  }
  return static_cast<float>((static_cast<double>(samples.size()) - 1.0) / dt);
}

MttHealthMonitorNode::MttHealthMonitorNode(const rclcpp::NodeOptions& options)
: rclcpp::Node("mtt_health_monitor_node", options)
{
  base_frame_ = declare_parameter("base_frame", std::string("base_footprint"));
  odom_frame_ = declare_parameter("odom_frame", std::string("odom"));
  health_topic_ = declare_parameter("health_topic", std::string("mtt_health"));
  fallback_odom_topic_ = declare_parameter(
    "fallback_odom_topic", std::string("mtt_monitor/cmd_fallback_odom"));
  status_topic_ = declare_parameter("status_topic", std::string("mtt_status"));
  battery_topic_ = declare_parameter("battery_topic", std::string("mtt_battery/status"));
  tachometer_topic_ = declare_parameter("tachometer_topic", std::string("mtt_tachometer"));
  cmd_vel_topic_ = declare_parameter("cmd_vel_topic", std::string("cmd_vel"));
  can_debug_topic_ = declare_parameter("can_debug_topic", std::string("mtt_can/debug_frames"));
  steer_control_mode_ = declare_parameter("steer_control_mode", std::string("closed_loop"));
  cmd_angular_mode_ = declare_parameter("cmd_angular_mode", std::string("normalized_steer"));
  health_publish_hz_ = declare_parameter("health_publish_hz", 10.0);
  cmd_vel_timeout_s_ = declare_parameter("cmd_vel_timeout_seconds", 0.5);
  wheelbase_m_ = declare_parameter("wheelbase_m", VehicleParams::total_wheelbase());
  max_articulation_rad_ = declare_parameter(
    "max_articulation_deg", VehicleParams::max_articulation_deg) * M_PI / 180.0;
  min_steer_speed_ms_ = declare_parameter("min_steer_speed_ms", VehicleParams::min_speed_for_steering);
  motion_model_params_.wheelbase_m = declare_parameter("model_wheelbase_m", wheelbase_m_);
  motion_model_params_.max_articulation_rad =
    declare_parameter("model_max_articulation_deg", VehicleParams::max_articulation_deg) * M_PI / 180.0;
  motion_model_params_.min_turn_speed_ms = min_steer_speed_ms_;
  motion_model_params_.speed_response_gain = declare_parameter("model_speed_response_gain", 3.0);
  motion_model_params_.articulation_response_gain =
    declare_parameter("model_articulation_response_gain", VehicleParams::articulation_response);
  motion_model_params_.brake_gain = declare_parameter("model_brake_gain", 1.0);
  motion_model_params_.use_slip_heuristic = declare_parameter("model_use_slip_heuristic", true);
  motion_model_params_.yaw_slip_base = declare_parameter("model_yaw_slip_base", 0.10);
  motion_model_params_.yaw_slip_speed_gain = declare_parameter("model_yaw_slip_speed_gain", 0.05);
  motion_model_params_.yaw_slip_articulation_gain =
    declare_parameter("model_yaw_slip_articulation_gain", 0.15);
  motion_model_params_.yaw_slip_min_scale = declare_parameter("model_yaw_slip_min_scale", 0.55);
  throttle_conflict_raw_threshold_ = declare_parameter("throttle_conflict_raw_threshold", 20);
  brake_conflict_raw_threshold_ = declare_parameter("brake_conflict_raw_threshold", 20);
  encoder_temp_warn_c_ = declare_parameter("encoder_temp_warn_c", 50.0);
  encoder_temp_danger_c_ = declare_parameter("encoder_temp_danger_c", 80.0);
  encoder_temp_critical_c_ = declare_parameter("encoder_temp_critical_c", 100.0);
  controller_temp_warn_c_ = declare_parameter("controller_temp_warn_c", 70.0);
  controller_temp_danger_c_ = declare_parameter("controller_temp_danger_c", 90.0);
  controller_temp_critical_c_ = declare_parameter("controller_temp_critical_c", 100.0);
  battery_temp_warn_c_ = declare_parameter("battery_temp_warn_c", 55.0);
  battery_temp_danger_c_ = declare_parameter("battery_temp_danger_c", 70.0);
  battery_temp_critical_c_ = declare_parameter("battery_temp_critical_c", 85.0);
  low_soc_warning_percent_ = declare_parameter("low_soc_warning_percent", 15.0);
  can_stale_warning_s_ = declare_parameter("can_stale_warning_seconds", 1.0);

  if (cmd_angular_mode_ != "normalized_steer" && cmd_angular_mode_ != "yaw_rate") {
    RCLCPP_WARN(
      get_logger(),
      "Unknown cmd_angular_mode '%s', falling back to 'normalized_steer'",
      cmd_angular_mode_.c_str());
    cmd_angular_mode_ = "normalized_steer";
  }
  if (steer_control_mode_ != "open_loop" && steer_control_mode_ != "closed_loop") {
    RCLCPP_WARN(
      get_logger(),
      "Unknown steer_control_mode '%s', falling back to 'closed_loop'",
      steer_control_mode_.c_str());
    steer_control_mode_ = "closed_loop";
  }

  fallback_motion_model_.set_params(motion_model_params_);

  status_sub_ = create_subscription<mtt_msgs::msg::MttVehicleStatus>(
    status_topic_, 10,
    [this](const mtt_msgs::msg::MttVehicleStatus::SharedPtr msg) { on_status(msg); });
  bms_sub_ = create_subscription<mtt_msgs::msg::MttBmsData>(
    battery_topic_, 10,
    [this](const mtt_msgs::msg::MttBmsData::SharedPtr msg) { on_bms(msg); });
  tacho_sub_ = create_subscription<mtt_msgs::msg::MttTachometerData>(
    tachometer_topic_, rclcpp::SensorDataQoS(),
    [this](const mtt_msgs::msg::MttTachometerData::SharedPtr msg) { on_tacho(msg); });
  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    cmd_vel_topic_, 10,
    [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) { on_cmd_vel(msg); });
  can_debug_sub_ = create_subscription<mtt_msgs::msg::MttCanFrame>(
    can_debug_topic_, 50,
    [this](const mtt_msgs::msg::MttCanFrame::SharedPtr msg) { on_can_debug(msg); });

  health_pub_ = create_publisher<mtt_msgs::msg::MttHealthState>(health_topic_, 10);
  fallback_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(fallback_odom_topic_, 10);

  publish_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / std::max(1e-3, health_publish_hz_)),
    [this]() { publish_health(); });

  RCLCPP_INFO(
    get_logger(),
    "MttHealthMonitorNode started (topic=%s, fallback_odom=%s, status=%s, can_debug=%s, steer=%s, angular=%s)",
    health_topic_.c_str(),
    fallback_odom_topic_.c_str(),
    status_topic_.c_str(),
    can_debug_topic_.c_str(),
    steer_control_mode_.c_str(),
    cmd_angular_mode_.c_str());
}

void MttHealthMonitorNode::on_status(const mtt_msgs::msg::MttVehicleStatus::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  last_status_ = *msg;
  has_status_ = true;
}

void MttHealthMonitorNode::on_bms(const mtt_msgs::msg::MttBmsData::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  last_bms_ = *msg;
  has_bms_ = true;
}

void MttHealthMonitorNode::on_tacho(const mtt_msgs::msg::MttTachometerData::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  last_tacho_ = *msg;
  has_tacho_ = true;
}

void MttHealthMonitorNode::on_cmd_vel(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  last_cmd_vel_ = *msg;
  last_cmd_vel_time_ = std::chrono::steady_clock::now();
  has_cmd_vel_ = true;
}

void MttHealthMonitorNode::on_can_debug(const mtt_msgs::msg::MttCanFrame::SharedPtr msg)
{
  if (msg->is_tx) {
    return;
  }

  const auto now_tp = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(state_mutex_);
  saw_can_debug_ = true;
  last_can_debug_time_ = now_tp;

  switch (msg->can_id) {
    case can::kTelemetryId:
      telemetry_tracker_.update(now_tp);
      break;
    case can::kBmsCellTempsId:
      bms_cell_tracker_.update(now_tp);
      break;
    case can::kBmsSysTempsId:
      bms_sys_tracker_.update(now_tp);
      break;
    case can::kBmsCoreId:
      bms_core_tracker_.update(now_tp);
      break;
    case can::kBmsDateTimeId:
      bms_datetime_tracker_.update(now_tp);
      break;
    case can::kChargerStatusId:
      charger_status_tracker_.update(now_tp);
      break;
    default:
      break;
  }
}

double MttHealthMonitorNode::command_angular_to_normalized_steer(double angular_input, double speed_ms) const
{
  if (cmd_angular_mode_ == "yaw_rate") {
    return logic::CommandMotionModel::normalized_steer_from_yaw_rate(
      angular_input,
      speed_ms,
      motion_model_params_);
  }
  return std::clamp(angular_input, -1.0, 1.0);
}

double MttHealthMonitorNode::normalized_steer_to_yaw_rate(double normalized_steer, double speed_ms) const
{
  auto nominal_params = motion_model_params_;
  nominal_params.use_slip_heuristic = false;
  return logic::CommandMotionModel::yaw_rate_from_speed_and_steer(
    normalized_steer,
    speed_ms,
    nominal_params,
    false);
}

double MttHealthMonitorNode::command_to_yaw_rate(double angular_input, double speed_ms) const
{
  if (cmd_angular_mode_ == "yaw_rate") {
    return angular_input;
  }
  return normalized_steer_to_yaw_rate(angular_input, speed_ms);
}

double MttHealthMonitorNode::wrap_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

std::string MttHealthMonitorNode::temperature_state(
  double value_c,
  double warn_c,
  double danger_c,
  double critical_c,
  bool valid)
{
  if (!valid) {
    return "unknown";
  }
  if (value_c >= critical_c) {
    return "critical";
  }
  if (value_c >= danger_c) {
    return "danger";
  }
  if (value_c >= warn_c) {
    return "warn";
  }
  return "ok";
}

void MttHealthMonitorNode::append_warning(
  std::vector<std::string>& warnings,
  const std::string& text,
  bool active)
{
  if (active) {
    warnings.push_back(text);
  }
}

void MttHealthMonitorNode::publish_health()
{
  mtt_msgs::msg::MttVehicleStatus status;
  mtt_msgs::msg::MttBmsData bms;
  mtt_msgs::msg::MttTachometerData tacho;
  geometry_msgs::msg::TwistStamped cmd_vel;
  bool has_status = false;
  bool has_bms = false;
  bool has_tacho = false;
  bool has_cmd_vel = false;
  bool saw_can_debug = false;
  std::chrono::steady_clock::time_point cmd_vel_time;
  std::chrono::steady_clock::time_point last_can_debug_time;
  float telemetry_hz = 0.0F;
  float bms_cell_hz = 0.0F;
  float bms_sys_hz = 0.0F;
  float bms_core_hz = 0.0F;
  float bms_datetime_hz = 0.0F;
  float charger_status_hz = 0.0F;
  double fallback_speed_ms = 0.0;
  double fallback_yaw_rate_rad_s = 0.0;
  double fallback_model_command_linear_speed_ms = 0.0;
  double fallback_model_speed_ms = 0.0;
  double fallback_model_articulation_command_rad = 0.0;
  double fallback_model_articulation_effective_rad = 0.0;
  double fallback_model_curvature_nominal_m_inv = 0.0;
  double fallback_model_curvature_effective_m_inv = 0.0;
  double fallback_model_yaw_rate_nominal_rad_s = 0.0;
  double fallback_model_yaw_rate_effective_rad_s = 0.0;
  bool fallback_model_state_valid = false;
  const auto now_tp = std::chrono::steady_clock::now();

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    status = last_status_;
    bms = last_bms_;
    tacho = last_tacho_;
    cmd_vel = last_cmd_vel_;
    has_status = has_status_;
    has_bms = has_bms_;
    has_tacho = has_tacho_;
    has_cmd_vel = has_cmd_vel_;
    saw_can_debug = saw_can_debug_;
    cmd_vel_time = last_cmd_vel_time_;
    last_can_debug_time = last_can_debug_time_;
    telemetry_hz = telemetry_tracker_.hz(now_tp);
    bms_cell_hz = bms_cell_tracker_.hz(now_tp);
    bms_sys_hz = bms_sys_tracker_.hz(now_tp);
    bms_core_hz = bms_core_tracker_.hz(now_tp);
    bms_datetime_hz = bms_datetime_tracker_.hz(now_tp);
    charger_status_hz = charger_status_tracker_.hz(now_tp);

    if (!fallback_initialized_) {
      last_fallback_update_ = now_tp;
      fallback_initialized_ = true;
    }

    const double dt = std::chrono::duration<double>(now_tp - last_fallback_update_).count();
    const bool cmd_fresh =
      has_cmd_vel &&
      std::chrono::duration<double>(now_tp - last_cmd_vel_time_).count() <= cmd_vel_timeout_s_;
    const double brake_normalized = has_status
      ? std::clamp(
          static_cast<double>(status.brake_raw) / static_cast<double>(VehicleParams::brake_max),
          0.0,
          1.0)
      : 0.0;
    logic::CommandMotionCommand motion_command;
    motion_command.linear_speed_cmd_ms = cmd_fresh ? cmd_vel.twist.linear.x : 0.0;
    motion_command.normalized_steer_cmd = cmd_fresh
      ? command_angular_to_normalized_steer(cmd_vel.twist.angular.z, cmd_vel.twist.linear.x)
      : 0.0;
    motion_command.brake_normalized = brake_normalized;
    motion_command.dt = dt > 1e-6 && dt < 1.0 ? dt : 0.02;
    const auto& fallback_state = fallback_motion_model_.step(motion_command);
    fallback_x_ = fallback_state.x;
    fallback_y_ = fallback_state.y;
    fallback_heading_ = fallback_state.heading;
    fallback_distance_m_ = fallback_state.cumulative_distance_m;
    fallback_speed_ms = fallback_state.v_eff_ms;
    fallback_yaw_rate_rad_s = fallback_state.yaw_rate_effective_rad_s;
    fallback_model_command_linear_speed_ms = fallback_state.v_command_ms;
    fallback_model_speed_ms = fallback_state.v_eff_ms;
    fallback_model_articulation_command_rad = fallback_state.phi_command_rad;
    fallback_model_articulation_effective_rad = fallback_state.phi_eff_rad;
    fallback_model_curvature_nominal_m_inv = fallback_state.kappa_nominal_m_inv;
    fallback_model_curvature_effective_m_inv = fallback_state.kappa_effective_m_inv;
    fallback_model_yaw_rate_nominal_rad_s = fallback_state.yaw_rate_nominal_rad_s;
    fallback_model_yaw_rate_effective_rad_s = fallback_state.yaw_rate_effective_rad_s;
    fallback_model_state_valid = true;

    if (dt <= 1e-6 || dt >= 1.0) {
      fallback_speed_ms = fallback_state.v_eff_ms;
      fallback_yaw_rate_rad_s = fallback_state.yaw_rate_effective_rad_s;
    }

    last_fallback_update_ = now_tp;
  }

  const bool cmd_fresh =
    has_cmd_vel &&
    std::chrono::duration<double>(now_tp - cmd_vel_time).count() <= cmd_vel_timeout_s_;
  const double commanded_linear_speed_ms = cmd_fresh ? cmd_vel.twist.linear.x : 0.0;
  const double commanded_angular_input = cmd_fresh ? cmd_vel.twist.angular.z : 0.0;
  const double commanded_yaw_rate = cmd_fresh
    ? command_to_yaw_rate(commanded_angular_input, commanded_linear_speed_ms)
    : 0.0;

  const bool telemetry_fresh = has_status ? status.telemetry_fresh : (has_tacho ? tacho.telemetry_fresh : false);
  const double telemetry_age_ms = has_status ? status.telemetry_age_ms : (has_tacho ? tacho.telemetry_age_ms : 0.0);
  const bool can_debug_available =
    has_status &&
    status.can_debug_enabled &&
    saw_can_debug &&
    std::chrono::duration<double>(now_tp - last_can_debug_time).count() <= can_stale_warning_s_;
  const bool tachometer_is_synthetic =
    has_status ? status.tachometer_is_synthetic : (has_tacho ? tacho.tachometer_is_synthetic : false);

  const double main_temp_a = has_tacho ? tacho.main_sensor_temp_a : (has_status ? status.main_sensor_temp_a_raw : 0.0);
  const double main_temp_b = has_tacho ? tacho.main_sensor_temp_b : (has_status ? status.main_sensor_temp_b_raw : 0.0);
  const bool any_bms_temp =
    has_bms && (bms.has_cell_temps || bms.has_sys_temps);
  const double battery_temp_peak = any_bms_temp
    ? std::max({
        static_cast<double>(bms.cell_temp_1_c),
        static_cast<double>(bms.cell_temp_2_c),
        static_cast<double>(bms.cell_temp_3_c),
        static_cast<double>(bms.cell_temp_4_c),
        static_cast<double>(bms.ambient_temp_c),
        static_cast<double>(bms.mosfet_temp_c),
        static_cast<double>(bms.heatpad_a_temp_c),
        static_cast<double>(bms.heatpad_b_temp_c),
      })
    : 0.0;

  const std::string controller_temp_state = temperature_state(
    main_temp_a,
    controller_temp_warn_c_,
    controller_temp_danger_c_,
    controller_temp_critical_c_,
    has_tacho || has_status);
  const std::string encoder_temp_state = temperature_state(
    main_temp_b,
    encoder_temp_warn_c_,
    encoder_temp_danger_c_,
    encoder_temp_critical_c_,
    has_tacho || has_status);
  const std::string battery_temp_state = temperature_state(
    battery_temp_peak,
    battery_temp_warn_c_,
    battery_temp_danger_c_,
    battery_temp_critical_c_,
    any_bms_temp);

  const bool brake_and_throttle_conflict =
    has_status &&
    status.throttle_raw >= static_cast<uint8_t>(std::max(0, throttle_conflict_raw_threshold_)) &&
    status.brake_raw >= static_cast<uint8_t>(std::max(0, brake_conflict_raw_threshold_));
  const bool high_encoder_temp = encoder_temp_state != "ok" && encoder_temp_state != "unknown";
  const bool high_controller_temp = controller_temp_state != "ok" && controller_temp_state != "unknown";
  const bool bms_temp_warning = battery_temp_state != "ok" && battery_temp_state != "unknown";
  const bool tachometer_fault_suspected = !telemetry_fresh || telemetry_age_ms > (1000.0 * can_stale_warning_s_);
  const bool low_soc_warning = has_bms && bms.has_soc && bms.soc_percent <= low_soc_warning_percent_;
  const bool can_stale_warning =
    has_status &&
    status.can_debug_enabled &&
    (!can_debug_available || telemetry_hz <= 0.0F);
  const bool steering_mode_mismatch =
    has_status &&
    status.steering_mode_closed_loop != (steer_control_mode_ == "closed_loop");

  std::vector<std::string> warnings;
  append_warning(
    warnings,
    "Throttle et frein envoyes en meme temps",
    brake_and_throttle_conflict);
  append_warning(
    warnings,
    "Temp B elevee sur le module principal / cote encodeur-tachy",
    high_encoder_temp);
  append_warning(
    warnings,
    "Temp A elevee sur le module principal",
    high_controller_temp);
  append_warning(
    warnings,
    "Temperature batterie / BMS elevee",
    bms_temp_warning);
  append_warning(
    warnings,
    "Telemetrie tachymetre stale ou absente",
    tachometer_fault_suspected);
  append_warning(
    warnings,
    "SOC batterie bas",
    low_soc_warning);
  append_warning(
    warnings,
    "Debug CAN active mais flux stale ou incomplet",
    can_stale_warning);
  append_warning(
    warnings,
    "Mode de steering observe different du mode configure",
    steering_mode_mismatch);
  if (has_status && status.command_timeout_active) {
    warnings.emplace_back("cmd_vel stale: le driver a neutralise throttle et steering");
  }
  if (!has_status) {
    warnings.emplace_back("En attente de /mtt_status");
  }

  const bool fallback_active = !telemetry_fresh || tachometer_is_synthetic;
  const bool fallback_low_confidence = true;
  std::string fallback_reason = "tachymetre sain";
  if (tachometer_is_synthetic && cmd_fresh) {
    fallback_reason = "tachymetre synthetique: estimation commande-only active";
  } else if (tachometer_is_synthetic && !cmd_fresh) {
    fallback_reason = "tachymetre synthetique et cmd_vel stale: modele maintenu mais peu informatif";
  } else if (!telemetry_fresh && cmd_fresh) {
    fallback_reason = "tachymetre stale: estimation commande-only active";
  } else if (!telemetry_fresh && !cmd_fresh) {
    fallback_reason = "tachymetre stale et cmd_vel stale: estimation commande-only peu informative";
  }

  mtt_msgs::msg::MttHealthState msg;
  msg.header.stamp = now();
  msg.header.frame_id = base_frame_;
  msg.can_interface = has_status ? status.can_interface : "";
  msg.command_can_id = has_status ? status.command_can_id : 0u;
  msg.external_control_active = has_status && status.command_can_id == can::kComMotorCommandId;
  msg.can_debug_enabled = has_status && status.can_debug_enabled;
  msg.can_debug_available = can_debug_available;
  msg.steer_control_mode = steer_control_mode_;
  msg.cmd_angular_mode = cmd_angular_mode_;
  msg.direction = has_status ? status.direction : (has_tacho ? tacho.direction : "Unknown");
  msg.security_unlocked = has_status && status.security_unlocked;
  msg.light_off_estop_patch = has_status && status.light_off_estop_patch;
  msg.emergency_stop_active = has_status && status.emergency_stop_active;
  msg.deadman_active = has_status && status.deadman_active;
  msg.remote_connected = has_status && status.remote_connected;
  msg.command_timeout_active = has_status && status.command_timeout_active;

  msg.throttle_raw = has_status ? status.throttle_raw : 0u;
  msg.brake_raw = has_status ? status.brake_raw : 0u;
  msg.steer_raw = has_status ? status.steer_raw : 0u;
  msg.steer_normalized = has_status ? status.steer_normalized : 0.0;
  msg.commanded_linear_speed_ms = commanded_linear_speed_ms;
  msg.commanded_angular_input = commanded_angular_input;
  msg.commanded_yaw_rate_rad_s = commanded_yaw_rate;

  msg.telemetry_fresh = telemetry_fresh;
  msg.telemetry_age_ms = telemetry_age_ms;
  msg.tachometer_present = has_status ? status.telemetry_seen_once : (has_tacho ? tacho.telemetry_seen_once : false);
  msg.tachometer_stale = msg.tachometer_present && !telemetry_fresh;
  msg.tachometer_is_synthetic = tachometer_is_synthetic;
  msg.tachometer_source = has_status ? status.tachometer_source : (has_tacho ? tacho.tachometer_source : std::string("unknown"));
  msg.telemetry_frame_hz = telemetry_hz;
  msg.bms_cell_frame_hz = bms_cell_hz;
  msg.bms_sys_frame_hz = bms_sys_hz;
  msg.bms_core_frame_hz = bms_core_hz;
  msg.bms_datetime_frame_hz = bms_datetime_hz;
  msg.charger_status_frame_hz = charger_status_hz;

  msg.main_sensor_temp_a_c = main_temp_a;
  msg.main_sensor_temp_b_c = main_temp_b;
  if (has_bms) {
    msg.cell_temp_1_c = bms.cell_temp_1_c;
    msg.cell_temp_2_c = bms.cell_temp_2_c;
    msg.cell_temp_3_c = bms.cell_temp_3_c;
    msg.cell_temp_4_c = bms.cell_temp_4_c;
    msg.ambient_temp_c = bms.ambient_temp_c;
    msg.mosfet_temp_c = bms.mosfet_temp_c;
    msg.heatpad_a_temp_c = bms.heatpad_a_temp_c;
    msg.heatpad_b_temp_c = bms.heatpad_b_temp_c;
    msg.soc_percent = bms.soc_percent;
    msg.low_soc_warning = low_soc_warning;
    msg.battery_current_raw = bms.battery_current_raw;
    msg.battery_current_estimated_a = bms.battery_current_estimated_a;
    msg.battery_current_estimated_valid = bms.battery_current_estimated_valid;
    msg.battery_voltage_raw = bms.battery_voltage_raw;
    msg.battery_voltage_v = bms.battery_voltage_v;
    msg.battery_voltage_valid = bms.battery_voltage_valid;
    msg.power_watts = bms.power_watts;
    msg.power_valid = bms.power_valid;
    msg.charge_time_remaining_min = bms.charge_time_remaining_min;
    msg.heatpad_a_on = bms.heatpad_a_on;
    msg.heatpad_b_on = bms.heatpad_b_on;
  }
  msg.controller_temp_state = controller_temp_state;
  msg.encoder_temp_state = encoder_temp_state;
  msg.battery_temp_state = battery_temp_state;

  msg.fallback_active = fallback_active;
  msg.fallback_low_confidence = fallback_low_confidence;
  msg.fallback_reason = fallback_reason;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    msg.fallback_speed_ms = fallback_speed_ms;
    msg.fallback_yaw_rate_rad_s = fallback_yaw_rate_rad_s;
    msg.fallback_distance_m = fallback_distance_m_;
    msg.fallback_heading_rad = fallback_heading_;
  }
  const bool tacho_model_state_valid = has_tacho && tacho.model_state_valid;
  msg.model_state_valid = tacho_model_state_valid || fallback_model_state_valid;
  if (tacho_model_state_valid) {
    msg.model_command_linear_speed_ms = tacho.model_command_linear_speed_ms;
    msg.model_speed_ms = tacho.model_speed_ms;
    msg.model_articulation_command_rad = tacho.model_articulation_command_rad;
    msg.model_articulation_effective_rad = tacho.model_articulation_effective_rad;
    msg.model_curvature_nominal_m_inv = tacho.model_curvature_nominal_m_inv;
    msg.model_curvature_effective_m_inv = tacho.model_curvature_effective_m_inv;
    msg.model_yaw_rate_nominal_rad_s = tacho.model_yaw_rate_nominal_rad_s;
    msg.model_yaw_rate_effective_rad_s = tacho.model_yaw_rate_effective_rad_s;
  } else {
    msg.model_command_linear_speed_ms = fallback_model_command_linear_speed_ms;
    msg.model_speed_ms = fallback_model_speed_ms;
    msg.model_articulation_command_rad = fallback_model_articulation_command_rad;
    msg.model_articulation_effective_rad = fallback_model_articulation_effective_rad;
    msg.model_curvature_nominal_m_inv = fallback_model_curvature_nominal_m_inv;
    msg.model_curvature_effective_m_inv = fallback_model_curvature_effective_m_inv;
    msg.model_yaw_rate_nominal_rad_s = fallback_model_yaw_rate_nominal_rad_s;
    msg.model_yaw_rate_effective_rad_s = fallback_model_yaw_rate_effective_rad_s;
  }

  msg.brake_and_throttle_conflict = brake_and_throttle_conflict;
  msg.high_encoder_temp = high_encoder_temp;
  msg.high_controller_temp = high_controller_temp;
  msg.bms_temp_warning = bms_temp_warning;
  msg.tachometer_fault_suspected = tachometer_fault_suspected;
  msg.can_stale_warning = can_stale_warning;
  msg.steering_mode_mismatch = steering_mode_mismatch;
  msg.warnings = warnings;
  msg.health_summary = warnings.empty() ? "ok" : ("warn: " + warnings.front());
  health_pub_->publish(msg);

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = msg.header.stamp;
  odom.header.frame_id = odom_frame_;
  odom.child_frame_id = base_frame_;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    odom.pose.pose.position.x = fallback_x_;
    odom.pose.pose.position.y = fallback_y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.z = std::sin(fallback_heading_ / 2.0);
    odom.pose.pose.orientation.w = std::cos(fallback_heading_ / 2.0);
  }
  odom.twist.twist.linear.x = fallback_speed_ms;
  odom.twist.twist.angular.z = fallback_yaw_rate_rad_s;
  odom.pose.covariance[0] = 25.0;
  odom.pose.covariance[7] = 25.0;
  odom.pose.covariance[14] = 9999.0;
  odom.pose.covariance[21] = 9999.0;
  odom.pose.covariance[28] = 9999.0;
  odom.pose.covariance[35] = 9.0;
  odom.twist.covariance[0] = 4.0;
  odom.twist.covariance[7] = 9999.0;
  odom.twist.covariance[14] = 9999.0;
  odom.twist.covariance[21] = 9999.0;
  odom.twist.covariance[28] = 9999.0;
  odom.twist.covariance[35] = 4.0;
  fallback_odom_pub_->publish(odom);
}

}  // namespace mtt

RCLCPP_COMPONENTS_REGISTER_NODE(mtt::MttHealthMonitorNode)
