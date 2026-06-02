// MttSpeedServoNode — implementation

#include "mtt_driver/components/mtt_speed_servo_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>

namespace mtt
{

MttSpeedServoNode::MttSpeedServoNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("mtt_speed_servo_node", options)
{
  // ── Parameters ──────────────────────────────────────────────────────
  mode_ = declare_parameter("mode", std::string("speed"));
  if (mode_ != "speed" && mode_ != "characterize" && mode_ != "disabled") {
    RCLCPP_WARN(get_logger(), "Unknown mode '%s', falling back to 'speed'", mode_.c_str());
    mode_ = "speed";
  }

  control_frequency_hz_ = declare_parameter("control_frequency_hz", 50.0);
  feedback_timeout_s_   = declare_parameter("feedback_timeout_s", 0.50);
  command_timeout_s_    = declare_parameter("command_timeout_s",  0.50);

  logic::SpeedServoParams p;
  p.kp               = declare_parameter("kp",               1.5);
  p.ki               = declare_parameter("ki",               0.8);
  p.kd               = declare_parameter("kd",               0.0);
  p.integrator_limit = declare_parameter("integrator_limit", 0.30);
  p.max_throttle     = declare_parameter("max_throttle",     1.0);
  p.max_speed_ms     = declare_parameter("max_speed_ms",     2.0);
  p.max_accel_ms2    = declare_parameter("max_accel_ms2",    1.5);
  p.max_decel_ms2    = declare_parameter("max_decel_ms2",    2.5);
  p.feedforward_gain = declare_parameter("feedforward_gain", 0.0);
  servo_.set_params(p);
  servo_.reset(0.0);

  // Feedforward LUT from config: flat vector [speed0, throttle0, speed1, throttle1, ...]
  const auto lut_flat = declare_parameter("feedforward_lut", std::vector<double>{});
  if (lut_flat.size() % 2 == 0 && !lut_flat.empty()) {
    std::vector<std::pair<double, double>> lut;
    lut.reserve(lut_flat.size() / 2);
    for (std::size_t i = 0; i + 1 < lut_flat.size(); i += 2) {
      lut.emplace_back(lut_flat[i], lut_flat[i + 1]);
    }
    servo_.set_feedforward_lut(std::move(lut));
    RCLCPP_INFO(get_logger(), "Loaded feedforward LUT with %zu points", lut_flat.size() / 2);
  }

  // Characterization parameters
  char_throttle_step_ = declare_parameter("characterize_throttle_step", 0.05);
  char_settle_time_s_ = declare_parameter("characterize_settle_time_s", 3.0);
  char_sample_time_s_ = declare_parameter("characterize_sample_time_s", 2.0);
  char_max_speed_ms_  = declare_parameter("max_characterize_speed_ms",  1.0);

  const auto setpoint_topic   = declare_parameter("setpoint_topic",
    std::string("/speed_setpoint"));
  const auto feedback_topic   = declare_parameter("feedback_topic",
    std::string("/mtt_tachometer"));
  const auto output_cmd_topic = declare_parameter("output_cmd_vel_topic",
    std::string("controller/cmd_vel"));

  // ── Subscribers ─────────────────────────────────────────────────────
  setpoint_sub_ = create_subscription<std_msgs::msg::Float64>(
    setpoint_topic,
    rclcpp::QoS(10),
    [this](std_msgs::msg::Float64::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      latest_setpoint_ms_    = std::abs(msg->data);
      latest_setpoint_stamp_ = get_clock()->now();
    });

  tacho_sub_ = create_subscription<mtt_msgs::msg::MttTachometerData>(
    feedback_topic,
    rclcpp::SensorDataQoS(),
    [this](mtt_msgs::msg::MttTachometerData::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      // Use absolute speed; direction tracking is informational only
      latest_speed_ms_        = std::abs(msg->speed_ms);
      latest_tacho_stamp_     = get_clock()->now();
      tacho_direction_forward_ = (msg->direction != "Reverse");
    });

  // Deadman for characterization safety
  deadman_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/mtt_control/teleop_deadman",
    rclcpp::QoS(10),
    [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      deadman_active_ = msg->data;
    });

  // ── Publishers ───────────────────────────────────────────────────────
  cmd_vel_pub_       = create_publisher<geometry_msgs::msg::TwistStamped>(
    output_cmd_topic, rclcpp::QoS(20));

  diag_setpoint_pub_ = create_publisher<std_msgs::msg::Float64>(
    "speed_servo/setpoint_ms", rclcpp::SensorDataQoS());
  diag_measured_pub_ = create_publisher<std_msgs::msg::Float64>(
    "speed_servo/measured_ms", rclcpp::SensorDataQoS());
  diag_error_pub_    = create_publisher<std_msgs::msg::Float64>(
    "speed_servo/error_ms", rclcpp::SensorDataQoS());
  diag_throttle_pub_ = create_publisher<std_msgs::msg::Float64>(
    "speed_servo/throttle_normalized", rclcpp::SensorDataQoS());
  feedforward_map_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
    "speed_servo/feedforward_map", rclcpp::QoS(10));

  // ── Timer ─────────────────────────────────────────────────────────────
  using ns = std::chrono::nanoseconds;
  const auto period = ns(static_cast<int64_t>(1e9 / std::max(1.0, control_frequency_hz_)));
  control_timer_ = create_wall_timer(period, [this]() { control_loop(); });

  RCLCPP_INFO(get_logger(),
    "MttSpeedServoNode started (mode=%s, kp=%.2f, ki=%.3f, kd=%.4f, rate=%.0f Hz)",
    mode_.c_str(), p.kp, p.ki, p.kd, control_frequency_hz_);
}

// ─────────────────────────────────────────────────────────────────────────────
// Control loop (timer callback)
// ─────────────────────────────────────────────────────────────────────────────
void MttSpeedServoNode::control_loop()
{
  if (mode_ == "disabled") return;

  if (mode_ == "characterize") {
    characterize_loop();
    return;
  }

  // Snapshot shared state
  std::optional<double> speed_ms;
  rclcpp::Time          tacho_stamp{0, 0, RCL_ROS_TIME};
  std::optional<double> setpoint_ms;
  rclcpp::Time          setpoint_stamp{0, 0, RCL_ROS_TIME};
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    speed_ms       = latest_speed_ms_;
    tacho_stamp    = latest_tacho_stamp_;
    setpoint_ms    = latest_setpoint_ms_;
    setpoint_stamp = latest_setpoint_stamp_;
  }

  const rclcpp::Time now = get_clock()->now();

  // Safety: tachometer freshness
  const bool tacho_fresh =
    speed_ms.has_value() &&
    (now - tacho_stamp).seconds() < feedback_timeout_s_;

  if (!tacho_fresh) {
    // No feedback — publish zero, do not control
    if (speed_ms.has_value()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Tachometer stale (%.1fs) — speed servo not active",
        (now - tacho_stamp).seconds());
    }
    publish_zero_cmd();
    servo_.reset(0.0);
    return;
  }

  // Safety: setpoint freshness — ramp to zero if stale
  const bool setpoint_fresh =
    setpoint_ms.has_value() &&
    (now - setpoint_stamp).seconds() < command_timeout_s_;

  if (!setpoint_fresh) {
    servo_.set_setpoint(0.0);
  } else {
    servo_.set_setpoint(*setpoint_ms);
  }

  const double dt = 1.0 / std::max(1.0, control_frequency_hz_);

  logic::SpeedServoDebug dbg;
  const double throttle = servo_.compute(*speed_ms, dt, dbg);

  // Publish cmd_vel: throttle is mapped back to m/s via max_speed_ms
  // The CAN node normalizes by max_linear_speed_ms_, so we send the equivalent m/s
  // throttle [0,1] * max_speed_ms gives the commanded m/s that the CAN node will re-normalize
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = now;
  cmd.header.frame_id = "base_link";
  cmd.twist.linear.x  = throttle * servo_.params().max_speed_ms;
  cmd.twist.angular.z = 0.0;  // steering handled by articulation servo or separately
  cmd_vel_pub_->publish(cmd);

  publish_diagnostics(dbg);
}

// ─────────────────────────────────────────────────────────────────────────────
// Characterization loop — step-ramp to build feedforward LUT
// ─────────────────────────────────────────────────────────────────────────────
void MttSpeedServoNode::characterize_loop()
{
  // State machine driven by static state + timing
  static enum class CharState {
    IDLE, SETTLING, SAMPLING, DONE
  } state = CharState::IDLE;
  static rclcpp::Time phase_start{0, 0, RCL_ROS_TIME};
  static double current_throttle = 0.0;

  // Snapshot
  std::optional<double> speed_ms;
  bool deadman;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    speed_ms = latest_speed_ms_;
    deadman  = deadman_active_;
  }

  const rclcpp::Time now = get_clock()->now();

  // Safety: abort if deadman released
  if (!deadman) {
    if (state != CharState::IDLE && state != CharState::DONE) {
      RCLCPP_WARN(get_logger(), "Characterization aborted — deadman released");
      publish_zero_cmd();
      state = CharState::DONE;
    }
    return;
  }

  // Safety: abort if speed exceeds limit
  if (speed_ms.has_value() && *speed_ms > char_max_speed_ms_ * 1.5) {
    RCLCPP_ERROR(get_logger(),
      "Characterization aborted — speed %.2f m/s exceeds safety limit",
      *speed_ms);
    publish_zero_cmd();
    state = CharState::DONE;
    return;
  }

  switch (state) {
    case CharState::IDLE:
      RCLCPP_INFO(get_logger(),
        "Starting throttle characterization: step=%.2f, settle=%.1fs, sample=%.1fs",
        char_throttle_step_, char_settle_time_s_, char_sample_time_s_);
      current_throttle = 0.0;
      char_lut_.clear();
      state = CharState::SETTLING;
      phase_start = now;
      break;

    case CharState::SETTLING: {
      // Send constant throttle during settle phase
      geometry_msgs::msg::TwistStamped cmd;
      cmd.header.stamp = now;
      // Convert throttle to m/s equivalent for CAN node
      cmd.twist.linear.x = current_throttle * servo_.params().max_speed_ms;
      cmd.twist.angular.z = 0.0;
      cmd_vel_pub_->publish(cmd);

      const double elapsed = (now - phase_start).seconds();
      if (elapsed >= char_settle_time_s_) {
        char_sample_buffer_.clear();
        state = CharState::SAMPLING;
        phase_start = now;
        RCLCPP_INFO(get_logger(),
          "Characterize: throttle=%.2f — settling done, sampling...", current_throttle);
      }
      break;
    }

    case CharState::SAMPLING: {
      // Continue sending throttle and accumulate speed samples
      geometry_msgs::msg::TwistStamped cmd;
      cmd.header.stamp = now;
      cmd.twist.linear.x = current_throttle * servo_.params().max_speed_ms;
      cmd.twist.angular.z = 0.0;
      cmd_vel_pub_->publish(cmd);

      if (speed_ms.has_value()) {
        char_sample_buffer_.push_back(*speed_ms);
      }

      const double elapsed = (now - phase_start).seconds();
      if (elapsed >= char_sample_time_s_) {
        // Compute mean speed
        double mean_speed = 0.0;
        if (!char_sample_buffer_.empty()) {
          for (double s : char_sample_buffer_) mean_speed += s;
          mean_speed /= static_cast<double>(char_sample_buffer_.size());
        }
        char_lut_.emplace_back(current_throttle, mean_speed);
        RCLCPP_INFO(get_logger(),
          "Characterize: throttle=%.2f → speed=%.3f m/s (%zu samples)",
          current_throttle, mean_speed, char_sample_buffer_.size());

        // Advance to next throttle step
        current_throttle += char_throttle_step_;
        if (current_throttle > 1.0 + char_throttle_step_ * 0.5 ||
            (speed_ms.has_value() && *speed_ms > char_max_speed_ms_))
        {
          // Done with ramp — publish LUT and stop
          state = CharState::DONE;
          publish_zero_cmd();

          // Publish as Float64MultiArray: [throttle0, speed0, throttle1, speed1, ...]
          std_msgs::msg::Float64MultiArray map_msg;
          for (const auto & [t, s] : char_lut_) {
            map_msg.data.push_back(t);
            map_msg.data.push_back(s);
          }
          feedforward_map_pub_->publish(map_msg);

          // Invert the LUT: we want speed→throttle for feedforward
          std::vector<std::pair<double, double>> inverted_lut;
          for (const auto & [t, s] : char_lut_) {
            inverted_lut.emplace_back(s, t);  // speed → throttle
          }
          std::sort(inverted_lut.begin(), inverted_lut.end(),
            [](const auto & a, const auto & b) { return a.first < b.first; });
          log_lut_yaml(inverted_lut);

          RCLCPP_INFO(get_logger(),
            "Characterization complete. %zu points collected. "
            "Copy the YAML above into config feedforward_lut and set feedforward_gain: 1.0",
            char_lut_.size());
        } else {
          state = CharState::SETTLING;
          phase_start = now;
        }
      }
      break;
    }

    case CharState::DONE:
      publish_zero_cmd();
      break;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────
void MttSpeedServoNode::publish_zero_cmd()
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = get_clock()->now();
  cmd.twist.linear.x = 0.0;
  cmd.twist.angular.z = 0.0;
  cmd_vel_pub_->publish(cmd);
}

void MttSpeedServoNode::publish_diagnostics(const logic::SpeedServoDebug & dbg)
{
  auto f64 = [](double v) {
    std_msgs::msg::Float64 m;
    m.data = v;
    return m;
  };
  diag_setpoint_pub_->publish(f64(dbg.rate_limited_setpoint_ms));
  diag_measured_pub_->publish(f64(dbg.measured_ms));
  diag_error_pub_->publish(f64(dbg.error_ms));
  diag_throttle_pub_->publish(f64(dbg.throttle_normalized));
}

void MttSpeedServoNode::log_lut_yaml(
  const std::vector<std::pair<double, double>> & lut) const
{
  RCLCPP_INFO(get_logger(), "──────── feedforward_lut (speed_ms → throttle) ────────");
  RCLCPP_INFO(get_logger(), "feedforward_lut:  # copy into speed_servo.yaml");
  for (const auto & [speed, throttle] : lut) {
    RCLCPP_INFO(get_logger(), "  - [%.3f, %.4f]  # %.3f m/s → throttle %.4f",
      speed, throttle, speed, throttle);
  }
  RCLCPP_INFO(get_logger(), "feedforward_gain: 1.0");
  RCLCPP_INFO(get_logger(), "────────────────────────────────────────────────────────");
}

}  // namespace mtt

RCLCPP_COMPONENTS_REGISTER_NODE(mtt::MttSpeedServoNode)
