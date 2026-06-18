// MttArticulationServoNode — implementation

#include "mtt_driver/components/mtt_articulation_servo_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>

namespace mtt
{

MttArticulationServoNode::MttArticulationServoNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("mtt_articulation_servo_node", options)
{
  // ── Parameters ──────────────────────────────────────────────────────
  mode_ = declare_parameter("mode", std::string("position"));

  logic::ArticulationServoParams p;
  p.kp                   = declare_parameter("kp",                   2.0);
  p.kd                   = declare_parameter("kd",                   0.05);
  p.ki                   = declare_parameter("ki",                   0.0);
  p.integrator_limit     = declare_parameter("integrator_limit",     0.30);
  p.max_steer            = declare_parameter("max_steer",            1.0);
  p.max_articulation_rad = declare_parameter("max_articulation_rad", 1.047);
  p.max_velocity_rad_s   = declare_parameter("max_velocity_rad_s",   0.50);
  servo_.set_params(p);
  servo_.reset(0.0);

  max_articulation_rad_ = p.max_articulation_rad;
  feedback_timeout_s_   = declare_parameter("feedback_timeout_s", 0.30);
  command_timeout_s_    = declare_parameter("command_timeout_s", 0.25);

  control_frequency_hz_ = declare_parameter("control_frequency_hz", 50.0);

  const auto feedback_topic  = declare_parameter("feedback_topic",
    std::string("/hardware/articulation_angle"));
  const auto position_topic  = declare_parameter("position_cmd_topic",
    std::string("/mtt_articulation_setpoint"));
  const auto velocity_topic  = declare_parameter("velocity_cmd_topic",
    std::string("/mtt_articulation_velocity_cmd"));

  if (mode_ != "position" && mode_ != "velocity" && mode_ != "disabled") {
    RCLCPP_WARN(get_logger(),
      "Unknown mode '%s', falling back to 'position'", mode_.c_str());
    mode_ = "position";
  }

  // ── Subscribers ─────────────────────────────────────────────────────
  // Hardware encoder — primary feedback (100 Hz from STM32)
  feedback_sub_ = create_subscription<std_msgs::msg::Float64>(
    feedback_topic,
    rclcpp::SensorDataQoS(),
    [this](std_msgs::msg::Float64::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      latest_feedback_rad_   = msg->data;
      latest_feedback_stamp_ = get_clock()->now();
    });

  // Position setpoint (rad, absolute)
  position_cmd_sub_ = create_subscription<std_msgs::msg::Float64>(
    position_topic,
    rclcpp::QoS(10),
    [this](std_msgs::msg::Float64::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      latest_position_cmd_rad_ = msg->data;
      latest_command_stamp_ = get_clock()->now();
    });

  // Velocity command (rad/s)
  velocity_cmd_sub_ = create_subscription<std_msgs::msg::Float64>(
    velocity_topic,
    rclcpp::QoS(10),
    [this](std_msgs::msg::Float64::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      latest_velocity_cmd_rad_s_ = msg->data;
      latest_command_stamp_ = get_clock()->now();
    });

  // ── Publishers ──────────────────────────────────────────────────────
  steer_cmd_pub_    = create_publisher<std_msgs::msg::Float64>(
    "articulation_servo/steer_cmd", rclcpp::SensorDataQoS());

  diag_setpoint_pub_ = create_publisher<std_msgs::msg::Float64>(
    "articulation_servo/setpoint_rad", rclcpp::SensorDataQoS());
  diag_measured_pub_ = create_publisher<std_msgs::msg::Float64>(
    "articulation_servo/measured_rad", rclcpp::SensorDataQoS());
  diag_error_pub_    = create_publisher<std_msgs::msg::Float64>(
    "articulation_servo/error_rad", rclcpp::SensorDataQoS());

  // ── Timer ────────────────────────────────────────────────────────────
  using ns = std::chrono::nanoseconds;
  const auto period = ns(static_cast<int64_t>(1e9 / std::max(1.0, control_frequency_hz_)));
  control_timer_ = create_wall_timer(period, [this]() { control_loop(); });

  RCLCPP_INFO(get_logger(),
    "MttArticulationServoNode started (mode=%s, kp=%.2f, kd=%.3f, ki=%.4f, rate=%.0f Hz)",
    mode_.c_str(), p.kp, p.kd, p.ki, control_frequency_hz_);
}

// ─────────────────────────────────────────────────────────────────────────────
// Control loop (timer callback)
// ─────────────────────────────────────────────────────────────────────────────
void MttArticulationServoNode::control_loop()
{
  if (mode_ == "disabled") return;

  // Snapshot shared state
  std::optional<double> feedback_rad;
  rclcpp::Time          feedback_stamp{0, 0, RCL_ROS_TIME};
  std::optional<double> position_cmd;
  std::optional<double> velocity_cmd;
  rclcpp::Time          command_stamp{0, 0, RCL_ROS_TIME};
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    feedback_rad   = latest_feedback_rad_;
    feedback_stamp = latest_feedback_stamp_;
    position_cmd   = latest_position_cmd_rad_;
    velocity_cmd   = latest_velocity_cmd_rad_s_;
    command_stamp  = latest_command_stamp_;
  }

  // Check feedback freshness
  const rclcpp::Time now = get_clock()->now();
  const bool feedback_fresh =
    feedback_rad.has_value() &&
    (now - feedback_stamp).seconds() < feedback_timeout_s_;

  if (!feedback_fresh) {
    // Encoder stale — do not send any servo override so CAN node uses cmd_vel
    return;
  }

  const bool command_fresh =
    command_stamp.nanoseconds() != 0 &&
    (now - command_stamp).seconds() < command_timeout_s_;
  if (!command_fresh) {
    servo_.reset(*feedback_rad);
    return;
  }

  const double measured = *feedback_rad;
  const double dt = 1.0 / std::max(1.0, control_frequency_hz_);

  // ── Update setpoint based on mode ─────────────────────────────────
  if (mode_ == "position" && position_cmd.has_value()) {
    servo_.set_position(*position_cmd);
  } else if (mode_ == "velocity" && velocity_cmd.has_value()) {
    servo_.step_velocity(*velocity_cmd, dt);
  }

  // ── PD compute ────────────────────────────────────────────────────
  logic::ArticulationServoDebug dbg;
  const double steer_cmd = servo_.compute(measured, dt, dbg);

  // ── Publish ────────────────────────────────────────────────────────
  auto f64 = [](double v) {
    std_msgs::msg::Float64 m;
    m.data = v;
    return m;
  };

  steer_cmd_pub_->publish(f64(steer_cmd));
  diag_setpoint_pub_->publish(f64(dbg.setpoint_rad));
  diag_measured_pub_->publish(f64(dbg.measured_rad));
  diag_error_pub_->publish(f64(dbg.error_rad));
}

}  // namespace mtt

RCLCPP_COMPONENTS_REGISTER_NODE(mtt::MttArticulationServoNode)
