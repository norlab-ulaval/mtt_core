// MTT-154 CAN Driver Node — implementation
// Replaces mtt_ros_wrapper.py. Uses a dedicated std::thread for CAN receive
// so it never blocks ROS callbacks.

#include "mtt_driver/components/mtt_can_node.hpp"

#include <chrono>
#include <cmath>

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
  base_frame_          = declare_parameter("base_frame",           std::string("base_link"));

  // ── Initialize command frame to safe defaults ────────────────────────
  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    command_frame_.init_defaults();
  }

  // ── Open CAN interface ───────────────────────────────────────────────
  init_can_interface();

  // ── Publishers ───────────────────────────────────────────────────────
  tachometer_pub_   = create_publisher<mtt_msgs::msg::MttTachometerData>("mtt_tachometer", 10);
  status_pub_       = create_publisher<mtt_msgs::msg::MttVehicleStatus>("mtt_status", 10);
  driving_mode_pub_ = create_publisher<mtt_msgs::msg::MttDrivingMode>("mtt_driving_mode", 10);
  steer_cmd_pub_    = create_publisher<std_msgs::msg::UInt8>("mtt_steer_cmd", 10);
  bms_pub_          = create_publisher<mtt_msgs::msg::MttBmsData>("mtt_battery/status", 10);

  // ── Subscribers ──────────────────────────────────────────────────────
  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    "cmd_vel", 10,
    [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg){ on_cmd_vel(msg); });
  aux_cmd_sub_ = create_subscription<mtt_msgs::msg::MttAuxCommand>(
    "mtt_aux_cmd", 10,
    [this](const mtt_msgs::msg::MttAuxCommand::SharedPtr msg){ on_aux_cmd(msg); });
  estop_sub_ = create_subscription<std_msgs::msg::Bool>(
    "teleop_estop", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg){ on_estop(msg); });

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

  // ── Timers ───────────────────────────────────────────────────────────
  using ms = std::chrono::duration<double, std::milli>;
  auto ctrl_ms = ms(1000.0 / std::max(1e-3, control_freq_hz_));
  auto can_ms  = ms(1000.0 / std::max(1e-3, can_frame_freq_hz_));
  control_timer_  = create_wall_timer(ctrl_ms, [this](){ control_loop(); });
  can_send_timer_ = create_wall_timer(can_ms,  [this](){ send_can_frame(); });

  // ── CAN receive thread ───────────────────────────────────────────────
  receiver_running_ = true;
  receiver_thread_ = std::thread([this](){ receiver_loop(); });

  RCLCPP_INFO(get_logger(), "MttCanNode started on %s (id=0x%03X)",
              can_interface_name_.c_str(), can_id_);
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

    if (frame->id == can::kTelemetryId) {
      auto reading = can::TelemetryDecoder::decode(frame->data.data(), frame->dlc);
      if (reading) {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        tachometer_.update(*reading);
      }
    } else if (frame->id == can::kBmsCellTempsId ||
               frame->id == can::kBmsSysTempsId  ||
               frame->id == can::kBmsCoreId) {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      can::BmsDecoder::decode(frame->id, frame->data.data(), frame->dlc, bms_reading_);
    }
  }
}

// ── cmd_vel callback ──────────────────────────────────────────────────
void MttCanNode::on_cmd_vel(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  const double lin   = msg->twist.linear.x;
  const double steer = std::clamp(msg->twist.angular.z, -1.0, 1.0);

  // Normalize throttle by max speed, then apply deadband
  const double throttle_norm = std::clamp(std::abs(lin) / max_linear_speed_ms_, 0.0, 1.0);
  const double throttle_cmd  = (throttle_norm < throttle_deadband_) ? 0.0 : throttle_norm;
  const double steer_cmd     = (std::abs(steer)  < steer_deadband_)  ? 0.0 : steer;

  std::lock_guard<std::mutex> lock(frame_mutex_);
  command_frame_.set_throttle(throttle_cmd);
  command_frame_.set_steer(steer_cmd);
  command_frame_.set_direction(lin >= 0.0 ? can::Direction::Forward : can::Direction::Reverse);
  current_steering_input_ = steer_cmd;
  last_cmd_vel_time_       = std::chrono::steady_clock::now();
  command_timeout_active_  = false;
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

// ── Control loop (publish + timeout watchdog) ─────────────────────────
void MttCanNode::control_loop()
{
  apply_command_timeout_if_needed();
  publish_vehicle_data();
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
  if (!can_->send(f)) {
    RCLCPP_WARN(get_logger(), "CAN send failed — attempting recovery");
    can_->try_recover();
  }
}

// ── Command timeout ───────────────────────────────────────────────────
bool MttCanNode::cmd_vel_is_fresh() const
{
  auto age = std::chrono::steady_clock::now() - last_cmd_vel_time_;
  return std::chrono::duration<double>(age).count() <= command_timeout_s_;
}

void MttCanNode::apply_command_timeout_if_needed()
{
  // Bare-bones mode: no automatic command neutralization.
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
  double steer_raw;
  double steer_cmd;
  bool   estop_active;
  bool   deadman;
  std::string safety_str;

  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    tach_snap    = tachometer_;
    bms_snap     = bms_reading_;
    steer_raw    = command_frame_.steer_raw();
    steer_cmd    = current_steering_input_;
    estop_active = !safety_locks_.empty();
    deadman      = teleop_estop_seen_ && !teleop_estop_active_;
    safety_str   = describe_safety_state(safety_locks_.empty() ? "SafetyUnlocked" : "SafetyLocked");
  }

  const auto timeout = std::chrono::milliseconds(static_cast<long>(telemetry_timeout_ms_));
  const bool fresh = tach_snap.is_fresh(timeout);

  // Tachometer message — always published so that mtt_odometry_node stays alive
  // and joint_states / articulation_angle keep flowing even when the encoder
  // is broken or the robot is stationary.  Speed is zeroed when not fresh.
  {
    mtt_msgs::msg::MttTachometerData tacho_msg;
    tacho_msg.header.stamp    = now();
    tacho_msg.header.frame_id = base_frame_;
    tacho_msg.tachometer_instant   = fresh ? tach_snap.reading.instant_rps  : 0u;
    tacho_msg.tachometer_cumulative = tach_snap.reading.cumulative_ticks;  // keep last known
    tacho_msg.speed_ms   = fresh ? tach_snap.speed_ms()  : 0.0;
    tacho_msg.speed_kmh  = fresh ? tach_snap.speed_kmh() : 0.0;
    tacho_msg.distance_km = tach_snap.absolute_distance_m() / 1000.0;
    tacho_msg.main_sensor_temp_a = tach_snap.reading.temperature_a;
    tacho_msg.main_sensor_temp_b = tach_snap.reading.temperature_b;
    tacho_msg.steer_cmd = steer_cmd;
    tachometer_pub_->publish(tacho_msg);
  }

  // Status message (always published for safety monitoring)
  mtt_msgs::msg::MttVehicleStatus status_msg;
  status_msg.header.stamp    = now();
  status_msg.header.frame_id = base_frame_;
  status_msg.speed_ms    = fresh ? tach_snap.speed_ms()  : 0.0;
  status_msg.speed_kmh   = fresh ? tach_snap.speed_kmh() : 0.0;
  status_msg.distance_km = fresh ? tach_snap.absolute_distance_m() / 1000.0 : 0.0;
  status_msg.temperature_a = fresh ? static_cast<double>(tach_snap.reading.temperature_a) : 0.0;
  status_msg.temperature_b = fresh ? static_cast<double>(tach_snap.reading.temperature_b) : 0.0;
  status_msg.steer_position = static_cast<uint8_t>(steer_raw);
  status_msg.emergency_stop_active = estop_active;
  status_msg.deadman_active = deadman;
  status_msg.safety_state   = safety_str;
  status_pub_->publish(status_msg);

  // Steer raw feedback
  std_msgs::msg::UInt8 steer_fb;
  steer_fb.data = static_cast<uint8_t>(steer_raw);
  steer_cmd_pub_->publish(steer_fb);

  // BMS message (published at control rate; skips if no BMS frames received yet)
  if (bms_snap.has_soc || bms_snap.has_cell_temps || bms_snap.has_sys_temps) {
    constexpr double kDt = 1.0 / 50.0;  // 50 Hz control loop

    mtt_msgs::msg::MttBmsData bms_msg;
    bms_msg.header.stamp    = now();
    bms_msg.header.frame_id = base_frame_;

    // State of charge & pack
    bms_msg.soc_percent         = static_cast<float>(bms_snap.soc_percent);
    // voltage/current raw — scaling (V/A per LSB) not confirmed in spec; use raw ints
    bms_msg.pack_voltage        = static_cast<float>(bms_snap.battery_voltage_raw);
    bms_msg.pack_current        = static_cast<float>(bms_snap.battery_current_raw);
    bms_msg.remaining_capacity  = static_cast<float>(bms_snap.charge_time_min);  // min→not Ah; named generically

    // Cell temperatures
    bms_msg.cell_temp_1 = static_cast<float>(bms_snap.cell_temp[0]);
    bms_msg.cell_temp_2 = static_cast<float>(bms_snap.cell_temp[1]);
    bms_msg.cell_temp_3 = static_cast<float>(bms_snap.cell_temp[2]);
    bms_msg.cell_temp_4 = static_cast<float>(bms_snap.cell_temp[3]);

    // System temperatures
    bms_msg.ambient_temp = static_cast<float>(bms_snap.ambient_temp);
    bms_msg.mosfet_temp  = static_cast<float>(bms_snap.mosfet_temp);

    // Power & energy (raw product until scaling is confirmed)
    float power = bms_msg.pack_voltage * bms_msg.pack_current;
    bms_msg.power_watts = power;
    energy_consumed_wh_ += static_cast<double>(power) * kDt / 3600.0;
    bms_msg.energy_consumed_wh = static_cast<float>(energy_consumed_wh_);

    // Freshness
    bms_msg.has_soc        = bms_snap.has_soc;
    bms_msg.has_cell_temps = bms_snap.has_cell_temps;
    bms_msg.has_sys_temps  = bms_snap.has_sys_temps;

    bms_pub_->publish(bms_msg);
  }
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

}  // namespace mtt

RCLCPP_COMPONENTS_REGISTER_NODE(mtt::MttCanNode)
