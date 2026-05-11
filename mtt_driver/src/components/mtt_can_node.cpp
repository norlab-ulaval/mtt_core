// MTT-154 CAN Driver Node — implementation
// Replaces mtt_ros_wrapper.py. Uses a dedicated std::thread for CAN receive
// so it never blocks ROS callbacks.

#include "mtt_driver/components/mtt_can_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

#include <rclcpp_components/register_node_macro.hpp>

namespace mtt {

using namespace std::chrono_literals;

MttCanNode::MttCanNode(const rclcpp::NodeOptions& options)
: rclcpp::Node("mtt_can_node", options)
{
  // ── Declare parameters ──────────────────────────────────────────────
  can_interface_name_  = declare_parameter("can_interface",        std::string("can0"));
  can_id_              = static_cast<uint32_t>(declare_parameter("can_id",              0x001));
  control_freq_hz_     = declare_parameter("control_frequency_hz", 50.0);
  can_frame_freq_hz_   = declare_parameter("can_frame_frequency_hz", 20.0);
  telemetry_timeout_ms_= declare_parameter("telemetry_timeout_ms", 500.0);
  command_timeout_s_   = declare_parameter("command_timeout_seconds", 0.5);
  max_linear_speed_ms_ = declare_parameter("max_linear_speed_ms", 1.0);
  throttle_deadband_   = declare_parameter("throttle_deadband",   0.05);
  steer_deadband_      = declare_parameter("steer_deadband",      0.05);
  wheelbase_m_         = declare_parameter("wheelbase_m",         VehicleParams::total_wheelbase());
  min_steer_speed_ms_  = declare_parameter("min_steer_speed_ms",  VehicleParams::min_speed_for_steering);
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
  hold_assist_params_.enabled = declare_parameter("hold_assist_enabled", true);
  hold_assist_params_.entry_speed_ms = declare_parameter("hold_assist_entry_speed_ms", 0.03);
  hold_assist_params_.release_speed_ms = declare_parameter("hold_assist_release_speed_ms", 0.015);
  hold_assist_params_.exit_command_ms = declare_parameter("hold_assist_exit_command_ms", 0.08);
  hold_assist_params_.kp = declare_parameter("hold_assist_kp", 1.2);
  hold_assist_params_.ki = declare_parameter("hold_assist_ki", 0.8);
  hold_assist_params_.integrator_limit = declare_parameter("hold_assist_integrator_limit", 0.25);
  hold_assist_params_.output_limit = declare_parameter("hold_assist_output_limit", 0.35);
  hold_assist_params_.deadband_compensation =
    declare_parameter("hold_assist_deadband_compensation", 0.12);
  hold_assist_params_.dither_enabled = declare_parameter("hold_assist_dither_enabled", false);
  hold_assist_params_.dither_amplitude = declare_parameter("hold_assist_dither_amplitude", 0.02);
  hold_assist_params_.dither_frequency_hz =
    declare_parameter("hold_assist_dither_frequency_hz", 6.0);
  hold_assist_params_.active_decel_enabled =
    declare_parameter("hold_assist_active_decel_enabled", false);
  hold_assist_params_.active_decel_entry_speed_ms =
    declare_parameter("hold_assist_active_decel_entry_speed_ms", 0.15);
  hold_assist_params_.active_decel_output_limit =
    declare_parameter("hold_assist_active_decel_output_limit", 0.60);
  base_frame_          = declare_parameter("base_frame",           std::string("base_footprint"));
  cmd_angular_mode_    = declare_parameter("cmd_angular_mode",     std::string("normalized_steer"));
  steer_control_mode_  = declare_parameter("steer_control_mode",   std::string("closed_loop"));
  tachometer_mode_     = declare_parameter("tachometer_mode",      std::string("real"));
  invert_inferred_tachometer_direction_ =
    declare_parameter("invert_inferred_tachometer_direction", false);
  publish_can_debug_   = declare_parameter("publish_can_debug",    false);
  can_debug_topic_     = declare_parameter("can_debug_topic",      std::string("mtt_can/debug_frames"));

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

  if (tachometer_mode_ == "fake") {
    tachometer_mode_ = "cmd_sim";
  }
  if (tachometer_mode_ != "real" && tachometer_mode_ != "cmd_sim") {
    RCLCPP_WARN(
      get_logger(),
      "Unknown tachometer_mode '%s', falling back to 'real'",
      tachometer_mode_.c_str());
    tachometer_mode_ = "real";
  }

  synthetic_motion_model_.set_params(motion_model_params_);
  hold_assist_controller_.set_params(hold_assist_params_);

  // ── Initialize command frame to safe defaults ────────────────────────
  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    command_frame_.init_defaults();
    command_frame_.set_steering_mode(
      steer_control_mode_ == "closed_loop"
        ? can::SteeringMode::CloseLoop
        : can::SteeringMode::OpenLoop);
  }

  // ── Open CAN interface ───────────────────────────────────────────────
  init_can_interface();

  // ── Publishers ───────────────────────────────────────────────────────
  // SensorDataQoS (BEST_EFFORT) matches all subscribers (mtt_odometry, joint_state_builder, health).
  tachometer_pub_   = create_publisher<mtt_msgs::msg::MttTachometerData>("mtt_tachometer", rclcpp::SensorDataQoS());
  status_pub_       = create_publisher<mtt_msgs::msg::MttVehicleStatus>("mtt_status", 10);
  driving_mode_pub_ = create_publisher<mtt_msgs::msg::MttDrivingMode>("mtt_driving_mode", 10);
  articulation_cmd_pub_ = create_publisher<std_msgs::msg::Float64>("mtt/articulation_cmd", rclcpp::SensorDataQoS());
  bms_pub_          = create_publisher<mtt_msgs::msg::MttBmsData>("mtt_battery/status", 10);
  if (publish_can_debug_) {
    can_debug_pub_ = create_publisher<mtt_msgs::msg::MttCanFrame>(can_debug_topic_, 50);
  }

  // ── Subscribers ──────────────────────────────────────────────────────
  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    "cmd_vel", 10,
    [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg){ on_cmd_vel(msg); });
  aux_cmd_sub_ = create_subscription<mtt_msgs::msg::MttAuxCommand>(
    "mtt_aux_cmd", 10,
    [this](const mtt_msgs::msg::MttAuxCommand::SharedPtr msg){ on_aux_cmd(msg); });
  estop_sub_ = create_subscription<std_msgs::msg::Bool>(
    "mtt_control/teleop_estop", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg){ on_estop(msg); });
  deadman_sub_ = create_subscription<std_msgs::msg::Bool>(
    "mtt_control/teleop_deadman", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg){ on_deadman(msg); });

  // ── Services ─────────────────────────────────────────────────────────
  set_mode_srv_ = create_service<mtt_interfaces::srv::SetVehiculeTypeSrv>(
    "mtt/set_driving_mode",
    [this](
      const mtt_interfaces::srv::SetVehiculeTypeSrv::Request::SharedPtr req,
      mtt_interfaces::srv::SetVehiculeTypeSrv::Response::SharedPtr res
    ){ on_set_mode(req, res); });
  get_mode_srv_ = create_service<mtt_interfaces::srv::GetVehiculeTypeSrv>(
    "mtt/get_driving_mode",
    [this](
      const mtt_interfaces::srv::GetVehiculeTypeSrv::Request::SharedPtr req,
      mtt_interfaces::srv::GetVehiculeTypeSrv::Response::SharedPtr res
    ){ on_get_mode(req, res); });
  set_steer_mode_srv_ = create_service<mtt_interfaces::srv::SetSteerControlMode>(
    "mtt/set_steer_control_mode",
    [this](
      const mtt_interfaces::srv::SetSteerControlMode::Request::SharedPtr req,
      mtt_interfaces::srv::SetSteerControlMode::Response::SharedPtr res
    ){ on_set_steer_mode(req, res); });

  // ── Timers ───────────────────────────────────────────────────────────
  using ms = std::chrono::duration<double, std::milli>;
  auto ctrl_ms = ms(1000.0 / std::max(1e-3, control_freq_hz_));
  auto can_ms  = ms(1000.0 / std::max(1e-3, can_frame_freq_hz_));
  control_timer_  = create_wall_timer(ctrl_ms, [this](){ control_loop(); });
  can_send_timer_ = create_wall_timer(can_ms,  [this](){ send_can_frame(); });

  // ── CAN receive thread ───────────────────────────────────────────────
  receiver_running_ = true;
  receiver_thread_ = std::thread([this](){ receiver_loop(); });

  RCLCPP_INFO(
    get_logger(),
    "MttCanNode started on %s (id=0x%03X, steer_control_mode=%s, cmd_angular_mode=%s, tachometer_mode=%s, can_debug=%s)",
    can_interface_name_.c_str(),
    can_id_,
    steer_control_mode_.c_str(),
    cmd_angular_mode_.c_str(),
    tachometer_mode_.c_str(),
    publish_can_debug_ ? can_debug_topic_.c_str() : "disabled");
}

MttCanNode::~MttCanNode()
{
  // Emergency stop before shutdown
  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    command_frame_.set_safety(can::SafetyState::Locked);
    command_frame_.set_throttle(0.0);
    command_frame_.set_brake(1.0);
    if (can_ && can_->is_open()) {
      hardware::CanFrame f;
      f.id  = can_id_;
      f.dlc = 8;
      f.data = command_frame_.data;
      can_->send(f);
    }
  }
  receiver_running_ = false;
  if (receiver_thread_.joinable()) receiver_thread_.join();
  if (can_) can_->close();
}

// ── Hardware init ─────────────────────────────────────────────────────
void MttCanNode::init_can_interface()
{
  can_ = std::make_shared<hardware::LinuxSocketCan>();
  if (!can_->open(can_interface_name_)) {
    RCLCPP_FATAL(get_logger(), "Failed to open CAN interface: %s", can_interface_name_.c_str());
    throw std::runtime_error("CAN interface unavailable: " + can_interface_name_);
  }
  RCLCPP_INFO(get_logger(), "CAN interface %s opened", can_interface_name_.c_str());
}

// ── Dedicated receiver thread ─────────────────────────────────────────
void MttCanNode::receiver_loop()
{
  const auto timeout_ms = std::chrono::milliseconds(100);
  while (receiver_running_) {
    if (!can_ || !can_->is_open()) {
      std::this_thread::sleep_for(100ms);
      continue;
    }
    auto frame = can_->receive(timeout_ms);
    if (!frame) continue;

    const bool known_frame = can::is_known_mtt_frame(frame->id);
    bool handled_by_driver = false;

    if (frame->id == can::kTelemetryId) {
      auto reading = can::TelemetryDecoder::decode(frame->data.data(), frame->dlc);
      if (reading) {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        tachometer_.update(*reading);
        handled_by_driver = true;
      }
    } else if (frame->id == can::kBmsCellTempsId ||
               frame->id == can::kBmsSysTempsId  ||
               frame->id == can::kBmsCoreId     ||
               frame->id == can::kBmsDateTimeId ||
               frame->id == can::kChargerCommandId ||
               frame->id == can::kChargerStatusId) {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      handled_by_driver = can::BmsDecoder::decode(frame->id, frame->data.data(), frame->dlc, bms_reading_);
    } else if (frame->id == can::kMainControllerVersionId ||
               frame->id == can::kBatteryControllerVersionId) {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      handled_by_driver = can::ControllerVersionDecoder::decode(
        frame->id, frame->data.data(), frame->dlc, controller_versions_);
    }

    if (publish_can_debug_ && known_frame) {
      publish_can_debug_frame(*frame, false, handled_by_driver);
    }
  }
}

// ── cmd_vel callback ──────────────────────────────────────────────────
void MttCanNode::on_cmd_vel(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  const double lin   = msg->twist.linear.x;
  const double steer = command_angular_to_normalized_steer(lin, msg->twist.angular.z);

  std::lock_guard<std::mutex> lock(frame_mutex_);
  current_steering_input_ = (std::abs(steer) < steer_deadband_) ? 0.0 : steer;
  current_linear_command_ms_ = std::clamp(lin, -max_linear_speed_ms_, max_linear_speed_ms_);
  cmd_vel_seen_           = true;
  last_cmd_vel_time_       = std::chrono::steady_clock::now();
  command_timeout_active_  = false;
}

double MttCanNode::command_angular_to_normalized_steer(double linear_x, double angular_z) const
{
  if (cmd_angular_mode_ == "yaw_rate") {
    auto params = motion_model_params_;
    params.min_turn_speed_ms = min_steer_speed_ms_;
    return logic::CommandMotionModel::normalized_steer_from_yaw_rate(angular_z, linear_x, params);
  }

  return std::clamp(angular_z, -1.0, 1.0);
}

can::Direction MttCanNode::infer_tachometer_direction(can::Direction commanded_direction) const
{
  if (!invert_inferred_tachometer_direction_) {
    return commanded_direction;
  }

  return commanded_direction == can::Direction::Reverse
    ? can::Direction::Forward
    : can::Direction::Reverse;
}

// ── aux_cmd callback ──────────────────────────────────────────────────
void MttCanNode::on_aux_cmd(const mtt_msgs::msg::MttAuxCommand::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(frame_mutex_);
  command_frame_.set_brake(static_cast<double>(msg->brake));
  command_frame_.set_light(msg->light_state == 1 ? can::LightState::On : can::LightState::Off);
  switch (msg->winch_command) {
    case 1:  command_frame_.set_winch(can::WinchState::In);  break;
    case 2:  command_frame_.set_winch(can::WinchState::Out); break;
    default: command_frame_.set_winch(can::WinchState::Neutral); break;
  }
}

// ── estop callback ────────────────────────────────────────────────────
void MttCanNode::on_estop(const std_msgs::msg::Bool::SharedPtr msg)
{
  teleop_estop_seen_   = true;
  teleop_estop_active_ = msg->data;
  set_safety_lock("teleop_estop", teleop_estop_active_);
}

void MttCanNode::on_deadman(const std_msgs::msg::Bool::SharedPtr msg)
{
  teleop_deadman_seen_ = true;
  teleop_deadman_active_ = msg->data;
}

// ── Control loop (publish + timeout watchdog) ─────────────────────────
void MttCanNode::control_loop()
{
  apply_command_timeout_if_needed();
  refresh_command_frame();
  publish_vehicle_data();
}

void MttCanNode::refresh_command_frame()
{
  const auto wall_now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(frame_mutex_);

  double dt = 1.0 / std::max(1e-3, control_freq_hz_);
  if (last_hold_assist_update_.time_since_epoch().count() != 0) {
    dt = std::chrono::duration<double>(wall_now - last_hold_assist_update_).count();
  }
  last_hold_assist_update_ = wall_now;
  dt = std::clamp(dt, 0.0, 1.0);

  const bool safety_locked = !safety_locks_.empty();
  const auto timeout = std::chrono::milliseconds(static_cast<long>(telemetry_timeout_ms_));
  const bool tachometer_is_synthetic = tachometer_mode_ == "cmd_sim";
  const bool telemetry_fresh = tachometer_is_synthetic ? true : tachometer_.is_fresh(timeout);
  const double measured_speed_ms = tachometer_is_synthetic
    ? synthetic_motion_model_.state().v_eff_ms
    : (telemetry_fresh
        ? tachometer_.speed_ms() * (
            infer_tachometer_direction(command_frame_.get_direction()) == can::Direction::Reverse ? -1.0 : 1.0)
        : 0.0);

  logic::HoldAssistInput hold_input;
  hold_input.dt = dt;
  hold_input.commanded_speed_ms = current_linear_command_ms_;
  hold_input.measured_speed_ms = measured_speed_ms;
  hold_input.telemetry_fresh = telemetry_fresh;
  hold_input.safety_locked = safety_locked;
  hold_input.brake_normalized = std::clamp(
    static_cast<double>(command_frame_.brake_raw()) / static_cast<double>(VehicleParams::brake_max),
    0.0,
    1.0);
  last_hold_assist_output_ = hold_assist_controller_.update(hold_input);

  effective_linear_command_ms_ = std::clamp(
    current_linear_command_ms_ + last_hold_assist_output_.correction_speed_ms,
    -max_linear_speed_ms_,
    max_linear_speed_ms_);

  const double throttle_norm =
    std::clamp(std::abs(effective_linear_command_ms_) / max_linear_speed_ms_, 0.0, 1.0);
  const double throttle_cmd = (throttle_norm < throttle_deadband_) ? 0.0 : throttle_norm;
  command_frame_.set_throttle(safety_locked ? 0.0 : throttle_cmd);
  command_frame_.set_steer(current_steering_input_);
  command_frame_.set_direction(
    effective_linear_command_ms_ >= 0.0 ? can::Direction::Forward : can::Direction::Reverse);
}

// ── CAN send timer ────────────────────────────────────────────────────
void MttCanNode::send_can_frame()
{
  if (!can_ || !can_->is_open()) return;
  std::lock_guard<std::mutex> lock(frame_mutex_);
  hardware::CanFrame f;
  f.id  = can_id_;
  f.dlc = 8;
  f.data = command_frame_.data;
  const bool sent = can_->send(f);
  if (publish_can_debug_) {
    publish_can_debug_frame(f, true, true);
  }
  if (!sent) {
    RCLCPP_WARN(get_logger(), "CAN send failed — attempting recovery");
    can_->try_recover();
  }
}

// ── Command timeout ───────────────────────────────────────────────────
bool MttCanNode::cmd_vel_is_fresh() const
{
  std::lock_guard<std::mutex> lock(frame_mutex_);
  if (!cmd_vel_seen_) return false;
  auto age = std::chrono::steady_clock::now() - last_cmd_vel_time_;
  return std::chrono::duration<double>(age).count() <= command_timeout_s_;
}

void MttCanNode::apply_command_timeout_if_needed()
{
  if (command_timeout_s_ <= 0.0) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (!cmd_vel_seen_) {
      return;
    }
  }

  if (cmd_vel_is_fresh()) {
    return;
  }

  bool should_log = false;
  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    command_frame_.set_throttle(0.0);
    command_frame_.set_steer(0.0);
    current_steering_input_ = 0.0;
    current_linear_command_ms_ = 0.0;
    if (!command_timeout_active_) {
      command_timeout_active_ = true;
      should_log = true;
    }
  }

  if (should_log) {
    RCLCPP_WARN(
      get_logger(),
      "cmd_vel timed out after %.3f s, neutralizing throttle and steering",
      command_timeout_s_);
  }
}

// ── Safety lock management ────────────────────────────────────────────
void MttCanNode::set_safety_lock(const std::string& reason, bool active)
{
  if (active) safety_locks_.insert(reason);
  else        safety_locks_.erase(reason);
  sync_safety_switch();
}

void MttCanNode::sync_safety_switch()
{
  std::lock_guard<std::mutex> lock(frame_mutex_);
  const bool unlocked = safety_locks_.empty();
  command_frame_.set_safety(unlocked ? can::SafetyState::Unlocked : can::SafetyState::Locked);
  if (!unlocked) {
    command_frame_.set_throttle(0.0);
    command_frame_.set_brake(1.0);   // full brake on e-stop / deadman release
    current_linear_command_ms_ = 0.0;
  }
  // On unlock: don't touch brake — on_aux_cmd controls it via RT trigger
}

std::string MttCanNode::describe_safety_state(const std::string& state) const
{
  if (safety_locks_.empty()) return state;
  std::string reasons;
  for (const auto& r : safety_locks_) reasons += r + ",";
  return state + ":" + reasons;
}

// ── Publish vehicle data ──────────────────────────────────────────────
void MttCanNode::publish_vehicle_data()
{
  TachometerState tach_snap;
  can::BmsReading bms_snap;
  can::ControllerVersionReading versions_snap;
  double steer_raw;
  double steer_cmd;
  can::Direction direction;
  can::Direction inferred_tachometer_direction;
  bool   estop_active;
  bool   deadman;
  bool   remote_connected;
  bool   command_timeout_active;
  std::string safety_str;
  can::VehicleType vehicle_type;
  can::WinchState winch_state;
  can::SteeringMode steering_mode;
  can::SafetyState safety_mode;
  bool light_off_estop_patch;
  uint8_t throttle_raw;
  uint8_t brake_raw;
  uint8_t reserved_raw;
  double synthetic_distance_m = 0.0;
  uint16_t synthetic_instant_rps = 0u;
  uint32_t synthetic_cumulative_ticks = 0u;
  double synthetic_speed_ms = 0.0;
  double synthetic_speed_kmh = 0.0;
  double synthetic_model_command_linear_speed_ms = 0.0;
  double synthetic_model_speed_ms = 0.0;
  double synthetic_model_articulation_command_rad = 0.0;
  double synthetic_model_articulation_effective_rad = 0.0;
  double synthetic_model_curvature_nominal_m_inv = 0.0;
  double synthetic_model_curvature_effective_m_inv = 0.0;
  double synthetic_model_yaw_rate_nominal_rad_s = 0.0;
  double synthetic_model_yaw_rate_effective_rad_s = 0.0;
  bool synthetic_model_state_valid = false;
  double hold_assist_output_ms = 0.0;
  bool hold_assist_active = false;
  std::string hold_assist_mode = "off";
  int8_t synthetic_temp_a = 0;
  int8_t synthetic_temp_b = 0;
  bool synthetic_seen_once = false;
  bool synthetic_fresh = false;
  double synthetic_telemetry_age_ms = 0.0;
  const auto wall_now = std::chrono::steady_clock::now();

  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    tach_snap    = tachometer_;
    bms_snap     = bms_reading_;
    versions_snap = controller_versions_;
    steer_raw    = command_frame_.steer_raw();
    steer_cmd    = current_steering_input_;
    throttle_raw = command_frame_.throttle_raw();
    brake_raw    = command_frame_.brake_raw();
    reserved_raw = command_frame_.reserved_raw();
    direction    = command_frame_.get_direction();
    inferred_tachometer_direction = infer_tachometer_direction(direction);
    vehicle_type = command_frame_.get_vehicle_type();
    winch_state  = command_frame_.get_winch();
    steering_mode = command_frame_.get_steering_mode();
    safety_mode  = command_frame_.get_safety();
    light_off_estop_patch = command_frame_.light_off_estop_patch();
    estop_active = !safety_locks_.empty();
    deadman      = teleop_deadman_seen_ && teleop_deadman_active_;
    remote_connected = teleop_deadman_seen_ || teleop_estop_seen_;
    command_timeout_active = command_timeout_active_;
    safety_str   = describe_safety_state(safety_locks_.empty() ? "SafetyUnlocked" : "SafetyLocked");
    hold_assist_output_ms = last_hold_assist_output_.correction_speed_ms;
    hold_assist_active = last_hold_assist_output_.active;
    hold_assist_mode = last_hold_assist_output_.mode;

    if (tachometer_mode_ == "cmd_sim") {
      if (!synthetic_tachometer_initialized_) {
        last_synthetic_update_ = wall_now;
        synthetic_tachometer_initialized_ = true;
      }

      double dt = std::chrono::duration<double>(wall_now - last_synthetic_update_).count();
      if (dt < 0.0 || dt > 1.0) {
        dt = 0.0;
      }
      last_synthetic_update_ = wall_now;

      logic::CommandMotionCommand motion_command;
      motion_command.linear_speed_cmd_ms = effective_linear_command_ms_;
      motion_command.normalized_steer_cmd = steer_cmd;
      motion_command.brake_normalized =
        std::clamp(
          static_cast<double>(brake_raw) / static_cast<double>(VehicleParams::brake_max),
          0.0,
          1.0);
      motion_command.dt = dt;
      const auto& synthetic_state = synthetic_motion_model_.step(motion_command);
      synthetic_distance_m_ = synthetic_state.cumulative_distance_m;
      synthetic_distance_m = synthetic_state.cumulative_distance_m;
      const double synthetic_abs_speed_ms = std::abs(synthetic_state.v_eff_ms);
      synthetic_speed_ms = synthetic_state.v_eff_ms;
      synthetic_speed_kmh = synthetic_state.v_eff_ms * 3.6;
      synthetic_model_command_linear_speed_ms = synthetic_state.v_command_ms;
      synthetic_model_speed_ms = synthetic_state.v_eff_ms;
      synthetic_model_articulation_command_rad = synthetic_state.phi_command_rad;
      synthetic_model_articulation_effective_rad = synthetic_state.phi_eff_rad;
      synthetic_model_curvature_nominal_m_inv = synthetic_state.kappa_nominal_m_inv;
      synthetic_model_curvature_effective_m_inv = synthetic_state.kappa_effective_m_inv;
      synthetic_model_yaw_rate_nominal_rad_s = synthetic_state.yaw_rate_nominal_rad_s;
      synthetic_model_yaw_rate_effective_rad_s = synthetic_state.yaw_rate_effective_rad_s;
      synthetic_model_state_valid = true;

      constexpr double encoder_ratio = VehicleParams::encoder_final_ratio();
      constexpr double track_length_m = VehicleParams::track_length_m;
      if (encoder_ratio > 0.0 && track_length_m > 1e-9) {
        const double instant_rps = synthetic_abs_speed_ms * encoder_ratio / track_length_m;
        const double cumulative_ticks = synthetic_distance_m_ * encoder_ratio / track_length_m;
        synthetic_instant_rps = static_cast<uint16_t>(std::clamp(
          std::llround(instant_rps),
          0LL,
          static_cast<long long>(std::numeric_limits<uint16_t>::max())));
        synthetic_cumulative_ticks = static_cast<uint32_t>(std::clamp(
          std::llround(cumulative_ticks),
          0LL,
          static_cast<long long>(std::numeric_limits<uint32_t>::max())));
      }

      synthetic_temp_a = tachometer_.has_data ? tachometer_.reading.temperature_a : 0;
      synthetic_temp_b = tachometer_.has_data ? tachometer_.reading.temperature_b : 0;
      synthetic_seen_once = true;
      synthetic_fresh = true;
      synthetic_telemetry_age_ms = 0.0;
    }
  }

  const auto timeout = std::chrono::milliseconds(static_cast<long>(telemetry_timeout_ms_));
  const bool tachometer_is_synthetic = tachometer_mode_ == "cmd_sim";
  const bool fresh = tachometer_is_synthetic ? synthetic_fresh : tach_snap.is_fresh(timeout);
  const double telemetry_age_ms =
    tachometer_is_synthetic ? synthetic_telemetry_age_ms : tach_snap.age_ms();
  const bool telemetry_seen_once =
    tachometer_is_synthetic ? synthetic_seen_once : tach_snap.has_data;
  const uint16_t tachometer_instant =
    tachometer_is_synthetic ? synthetic_instant_rps : (fresh ? tach_snap.reading.instant_rps : 0u);
  const uint32_t tachometer_cumulative =
    tachometer_is_synthetic ? synthetic_cumulative_ticks : tach_snap.reading.cumulative_ticks;
  const double reported_speed_ms =
    tachometer_is_synthetic ? synthetic_speed_ms : (fresh ? tach_snap.speed_ms() : 0.0);
  const double reported_speed_kmh =
    tachometer_is_synthetic ? synthetic_speed_kmh : (fresh ? tach_snap.speed_kmh() : 0.0);
  const double reported_distance_km =
    tachometer_is_synthetic ? (synthetic_distance_m / 1000.0) : (tach_snap.absolute_distance_m() / 1000.0);
  const double reported_temp_a =
    tachometer_is_synthetic ? static_cast<double>(synthetic_temp_a) : static_cast<double>(tach_snap.reading.temperature_a);
  const double reported_temp_b =
    tachometer_is_synthetic ? static_cast<double>(synthetic_temp_b) : static_cast<double>(tach_snap.reading.temperature_b);
  const int8_t reported_temp_a_raw =
    tachometer_is_synthetic ? synthetic_temp_a : tach_snap.reading.temperature_a;
  const int8_t reported_temp_b_raw =
    tachometer_is_synthetic ? synthetic_temp_b : tach_snap.reading.temperature_b;
  can::Direction reported_direction = inferred_tachometer_direction;
  if (tachometer_is_synthetic) {
    const double signed_speed_for_direction =
      std::abs(synthetic_speed_ms) > 1e-4 ? synthetic_speed_ms : synthetic_model_speed_ms;
    if (std::abs(signed_speed_for_direction) > 1e-4) {
      reported_direction = signed_speed_for_direction < 0.0
        ? can::Direction::Reverse
        : can::Direction::Forward;
    }
  }

  // Tachometer message — always published so that mtt_odometry_node stays alive
  // and joint_states / articulation_angle keep flowing even when the encoder
  // is broken or the robot is stationary.  Speed is zeroed when not fresh.
  {
    mtt_msgs::msg::MttTachometerData tacho_msg;
    tacho_msg.header.stamp    = now();
    tacho_msg.header.frame_id = base_frame_;
    tacho_msg.telemetry_can_id = can::kTelemetryId;
    tacho_msg.telemetry_seen_once = telemetry_seen_once;
    tacho_msg.telemetry_fresh = fresh;
    tacho_msg.telemetry_age_ms = telemetry_age_ms;
    tacho_msg.tachometer_is_synthetic = tachometer_is_synthetic;
    tacho_msg.tachometer_source = tachometer_mode_;
    tacho_msg.tachometer_instant = tachometer_instant;
    tacho_msg.tachometer_cumulative = tachometer_cumulative;
    tacho_msg.speed_ms = reported_speed_ms;
    tacho_msg.speed_kmh = reported_speed_kmh;
    tacho_msg.distance_km = reported_distance_km;
    tacho_msg.direction  =
      reported_direction == can::Direction::Reverse ? "Reverse" : "Forward";
    tacho_msg.main_sensor_temp_a = reported_temp_a;
    tacho_msg.main_sensor_temp_b = reported_temp_b;
    tacho_msg.steer_cmd = steer_cmd;
    tacho_msg.model_state_valid = synthetic_model_state_valid;
    tacho_msg.model_command_linear_speed_ms = synthetic_model_command_linear_speed_ms;
    tacho_msg.model_speed_ms = synthetic_model_speed_ms;
    tacho_msg.model_articulation_command_rad = synthetic_model_articulation_command_rad;
    tacho_msg.model_articulation_effective_rad = synthetic_model_articulation_effective_rad;
    tacho_msg.model_curvature_nominal_m_inv = synthetic_model_curvature_nominal_m_inv;
    tacho_msg.model_curvature_effective_m_inv = synthetic_model_curvature_effective_m_inv;
    tacho_msg.model_yaw_rate_nominal_rad_s = synthetic_model_yaw_rate_nominal_rad_s;
    tacho_msg.model_yaw_rate_effective_rad_s = synthetic_model_yaw_rate_effective_rad_s;
    tachometer_pub_->publish(tacho_msg);

    // Publish commanded articulation angle as a standalone observable topic.
    // Same value as tacho_msg.model_articulation_command_rad but directly plottable.
    std_msgs::msg::Float64 artic_cmd_msg;
    artic_cmd_msg.data = synthetic_model_articulation_command_rad;
    articulation_cmd_pub_->publish(artic_cmd_msg);
  }

  // Status message (always published for safety monitoring)
  mtt_msgs::msg::MttVehicleStatus status_msg;
  status_msg.header.stamp    = now();
  status_msg.header.frame_id = base_frame_;
  status_msg.can_interface = can_interface_name_;
  status_msg.command_can_id = can_id_;
  status_msg.telemetry_can_id = can::kTelemetryId;
  status_msg.telemetry_seen_once = telemetry_seen_once;
  status_msg.telemetry_fresh = fresh;
  status_msg.telemetry_age_ms = telemetry_age_ms;
  status_msg.tachometer_is_synthetic = tachometer_is_synthetic;
  status_msg.tachometer_source = tachometer_mode_;
  status_msg.speed_ms = reported_speed_ms;
  status_msg.speed_kmh = reported_speed_kmh;
  status_msg.distance_km = reported_distance_km;
  status_msg.direction   =
    reported_direction == can::Direction::Reverse ? "Reverse" : "Forward";
  status_msg.temperature_a = reported_temp_a;
  status_msg.temperature_b = reported_temp_b;
  status_msg.main_sensor_temp_a_raw = reported_temp_a_raw;
  status_msg.main_sensor_temp_b_raw = reported_temp_b_raw;
  status_msg.steer_position = static_cast<uint8_t>(steer_raw);
  status_msg.tachometer_instant = tachometer_instant;
  status_msg.tachometer_cumulative = tachometer_cumulative;
  status_msg.tachometer_instant_ticks_per_s = tachometer_instant;
  status_msg.tachometer_cumulative_ticks = tachometer_cumulative;
  status_msg.vehicle_type_raw = static_cast<uint8_t>(vehicle_type);
  status_msg.vehicle_type_label = can::vehicle_type_to_string(vehicle_type);
  status_msg.security_unlocked = safety_mode == can::SafetyState::Unlocked;
  status_msg.light_off_estop_patch = light_off_estop_patch;
  status_msg.direction_reverse = reported_direction == can::Direction::Reverse;
  status_msg.throttle_raw = throttle_raw;
  status_msg.brake_raw = brake_raw;
  status_msg.steer_raw = static_cast<uint8_t>(steer_raw);
  status_msg.steer_normalized = steer_cmd;
  status_msg.command_linear_speed_ms = current_linear_command_ms_;
  status_msg.effective_linear_speed_command_ms = effective_linear_command_ms_;
  status_msg.hold_assist_active = hold_assist_active;
  status_msg.hold_assist_mode = hold_assist_mode;
  status_msg.hold_assist_output_ms = hold_assist_output_ms;
  status_msg.winch_raw = static_cast<uint8_t>(winch_state);
  status_msg.winch_state = can::winch_state_to_string(winch_state);
  status_msg.steering_mode_closed_loop = steering_mode == can::SteeringMode::CloseLoop;
  status_msg.reserved_byte_7 = reserved_raw;
  status_msg.emergency_stop_active = estop_active;
  status_msg.remote_connected = remote_connected;
  status_msg.deadman_active = deadman;
  status_msg.command_timeout_active = command_timeout_active;
  status_msg.can_debug_enabled = publish_can_debug_;
  status_msg.safety_state   = safety_str;
  status_msg.has_main_controller_version = versions_snap.has_main_controller_version;
  status_msg.main_hardware_revision_raw = versions_snap.main_hardware_revision_raw;
  status_msg.main_software_revision_raw = versions_snap.main_software_revision_raw;
  status_msg.has_battery_controller_version = versions_snap.has_battery_controller_version;
  status_msg.battery_hardware_revision_raw = versions_snap.battery_hardware_revision_raw;
  status_msg.battery_software_revision_raw = versions_snap.battery_software_revision_raw;
  status_pub_->publish(status_msg);

  // BMS message (published at control rate; skips if no BMS frames received yet)
  if (bms_snap.has_soc ||
      bms_snap.has_cell_temps ||
      bms_snap.has_sys_temps ||
      bms_snap.has_datetime ||
      bms_snap.has_charger_command ||
      bms_snap.has_charger_status) {
    mtt_msgs::msg::MttBmsData bms_msg;
    bms_msg.header.stamp    = now();
    bms_msg.header.frame_id = base_frame_;

    // Frame availability
    bms_msg.has_soc        = bms_snap.has_soc;
    bms_msg.has_cell_temps = bms_snap.has_cell_temps;
    bms_msg.has_sys_temps  = bms_snap.has_sys_temps;
    bms_msg.has_datetime   = bms_snap.has_datetime;
    bms_msg.has_charger_command = bms_snap.has_charger_command;
    bms_msg.has_charger_status = bms_snap.has_charger_status;

    // Core state
    bms_msg.soc_percent         = bms_snap.soc_percent;
    bms_msg.battery_voltage_raw = bms_snap.battery_voltage_raw;
    bms_msg.battery_current_raw = bms_snap.battery_current_raw;
    bms_msg.battery_current_estimated_valid = true;
    bms_msg.battery_voltage_valid = false;
    bms_msg.power_valid = false;
    bms_msg.battery_current_estimated_a =
      static_cast<float>(bms_snap.battery_current_raw * 0.0103 - 0.72);
    bms_msg.battery_voltage_v = 0.0f;
    bms_msg.power_watts = 0.0f;
    bms_msg.energy_consumed_wh = 0.0f;
    bms_msg.charge_time_remaining_min = bms_snap.charge_time_min;
    bms_msg.heatpad_a_on = bms_snap.heatpad_a_on;
    bms_msg.heatpad_b_on = bms_snap.heatpad_b_on;
    bms_msg.heatpads_reserved = bms_snap.heatpads_reserved;

    // Cell temperatures (0x600)
    bms_msg.cell_temp_1_raw = bms_snap.cell_temp[0];
    bms_msg.cell_temp_2_raw = bms_snap.cell_temp[1];
    bms_msg.cell_temp_3_raw = bms_snap.cell_temp[2];
    bms_msg.cell_temp_4_raw = bms_snap.cell_temp[3];
    bms_msg.cell_temp_1_c = static_cast<float>(bms_snap.cell_temp[0]);
    bms_msg.cell_temp_2_c = static_cast<float>(bms_snap.cell_temp[1]);
    bms_msg.cell_temp_3_c = static_cast<float>(bms_snap.cell_temp[2]);
    bms_msg.cell_temp_4_c = static_cast<float>(bms_snap.cell_temp[3]);

    // System temperatures (0x601)
    bms_msg.ambient_temp_raw = bms_snap.ambient_temp;
    bms_msg.mosfet_temp_raw = bms_snap.mosfet_temp;
    bms_msg.heatpad_a_temp_raw = bms_snap.heatpad_a_temp;
    bms_msg.heatpad_b_temp_raw = bms_snap.heatpad_b_temp;
    bms_msg.ambient_temp_c = static_cast<float>(bms_snap.ambient_temp);
    bms_msg.mosfet_temp_c = static_cast<float>(bms_snap.mosfet_temp);
    bms_msg.heatpad_a_temp_c = static_cast<float>(bms_snap.heatpad_a_temp);
    bms_msg.heatpad_b_temp_c = static_cast<float>(bms_snap.heatpad_b_temp);

    // Raw 0x603 and charger payloads
    bms_msg.charge_time_remaining_603_raw = bms_snap.charge_time_remaining_603_raw;
    bms_msg.year_month_raw = bms_snap.year_month_raw;
    bms_msg.day_hour_raw = bms_snap.day_hour_raw;
    bms_msg.minute_second_raw = bms_snap.minute_second_raw;
    bms_msg.charger_max_voltage_raw = bms_snap.charger_max_voltage_raw;
    bms_msg.charger_max_current_raw = bms_snap.charger_max_current_raw;
    bms_msg.charger_configured_voltage_raw = bms_snap.charger_configured_voltage_raw;
    bms_msg.charger_configured_current_raw = bms_snap.charger_configured_current_raw;

    bms_pub_->publish(bms_msg);
  }
}

void MttCanNode::publish_can_debug_frame(
  const hardware::CanFrame& frame,
  bool is_tx,
  bool handled_by_driver)
{
  if (!publish_can_debug_ || !can_debug_pub_) {
    return;
  }

  mtt_msgs::msg::MttCanFrame msg;
  msg.header.stamp = now();
  msg.header.frame_id = base_frame_;
  msg.can_interface = can_interface_name_;
  msg.can_id = frame.id;
  msg.is_extended = frame.is_extended;
  msg.is_tx = is_tx;
  msg.handled_by_driver = handled_by_driver;
  msg.frame_name = can::frame_name_from_id(frame.id);
  msg.dlc = frame.dlc;
  msg.data = frame.data;
  can_debug_pub_->publish(msg);
}

// ── Services ─────────────────────────────────────────────────────────
void MttCanNode::on_set_mode(
  const mtt_interfaces::srv::SetVehiculeTypeSrv::Request::SharedPtr req,
  mtt_interfaces::srv::SetVehiculeTypeSrv::Response::SharedPtr res)
{
  int new_mode = static_cast<int>(req->vehicule_type);
  bool changed = (new_mode != current_driving_mode_);
  current_driving_mode_ = new_mode;

  mtt_msgs::msg::MttDrivingMode mode_msg;
  mode_msg.mode = static_cast<uint8_t>(new_mode);
  mode_msg.mode_parameters = "";
  driving_mode_pub_->publish(mode_msg);

  res->success = changed;
}

void MttCanNode::on_get_mode(
  const mtt_interfaces::srv::GetVehiculeTypeSrv::Request::SharedPtr,
  mtt_interfaces::srv::GetVehiculeTypeSrv::Response::SharedPtr res)
{
  static const std::array<const char*, 3> names{
    "SINGLE_TRAILER", "DUAL_DIFFERENTIAL", "DUAL_SERPENTINE"};
  res->vehicule_type = static_cast<uint8_t>(current_driving_mode_);
  res->type_name = (current_driving_mode_ >= 0 && current_driving_mode_ < 3)
                   ? names[current_driving_mode_] : "UNKNOWN";
}

void MttCanNode::on_set_steer_mode(
  const mtt_interfaces::srv::SetSteerControlMode::Request::SharedPtr req,
  mtt_interfaces::srv::SetSteerControlMode::Response::SharedPtr res)
{
  if (req->control_mode != "open_loop" && req->control_mode != "closed_loop") {
    res->success = false;
    res->message = "Invalid control_mode. Use 'open_loop' or 'closed_loop'";
    return;
  }

  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    steer_control_mode_ = req->control_mode;
    command_frame_.set_steering_mode(
      steer_control_mode_ == "closed_loop"
        ? can::SteeringMode::CloseLoop
        : can::SteeringMode::OpenLoop);
  }

  res->success = true;
  res->message = "Steer control mode set to " + steer_control_mode_;
  RCLCPP_INFO(get_logger(), "CAN steer mode → %s", steer_control_mode_.c_str());
}

}  // namespace mtt

RCLCPP_COMPONENTS_REGISTER_NODE(mtt::MttCanNode)
