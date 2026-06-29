// MTT-154 Odometry Node — ROS 2 Component
// Replaces mtt_odometry_manager.py. Subscribes mtt_tachometer, runs the
// multi-mode odometry strategy, publishes nav_msgs/Odometry + TF.

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <chrono>

#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>

#include <mtt_msgs/msg/mtt_tachometer_data.hpp>
#include <mtt_msgs/msg/mtt_driving_mode.hpp>
#include <mtt_msgs/msg/mtt_articulation_state.hpp>
#include <mtt_interfaces/srv/set_steer_control_mode.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "mtt_driver/logic/command_motion_model.hpp"
#include "mtt_driver/logic/articulation_source_selector.hpp"
#include "mtt_driver/logic/odometry_calculator.hpp"
#include "mtt_driver/logic/vehicle_params.hpp"

namespace mtt {

class MttOdometryNode : public rclcpp::Node {
public:
  explicit MttOdometryNode(const rclcpp::NodeOptions& options)
  : rclcpp::Node("mtt_odometry_node", options)
  {
    // ── Parameters ──
    odom_frame_       = declare_parameter("odom_frame",       std::string("odom"));
    base_frame_       = declare_parameter("base_frame",       std::string("base_footprint"));
    broadcast_tf_     = declare_parameter("broadcast_tf",     true);
    publish_runtime_joint_states_ =
      declare_parameter("publish_runtime_joint_states", false);
    runtime_joint_states_topic_ =
      declare_parameter("runtime_joint_states_topic", std::string("joint_states"));
    runtime_joint_pitch_rad_ =
      declare_parameter("runtime_joint_pitch_rad", 0.0);
    runtime_joint_roll_rad_ =
      declare_parameter("runtime_joint_roll_rad", 0.0);
    runtime_joint_articulation_sign_ =
      declare_parameter("runtime_joint_articulation_sign", 1.0);
    runtime_joint_articulation_offset_rad_ =
      declare_parameter("runtime_joint_articulation_offset_rad", 0.0);
    track_width_m_    = declare_parameter("track_width_m",    VehicleParams::track_width);
    wheelbase_m_      = declare_parameter("wheelbase_m",      VehicleParams::total_wheelbase());
    steer_mode_       = declare_parameter("steer_control_mode", std::string("open_loop"));
    cmd_angular_mode_ = declare_parameter("cmd_angular_mode", std::string("normalized_steer"));
    cmd_vel_topic_    = declare_parameter("cmd_vel_topic", std::string("cmd_vel"));
    cmd_vel_timeout_s_ = declare_parameter("cmd_vel_timeout_seconds", 0.5);
    pivot_turn_       = declare_parameter("pivot_turn_enabled", false);
    min_speed_turn_   = declare_parameter("min_turn_speed_ms", 0.03);
    max_articulation_rad_ = declare_parameter("max_articulation_deg",
        VehicleParams::max_articulation_deg) * M_PI / 180.0;
    yaw_slip_factor_  = declare_parameter("yaw_slip_factor",  1.0);
    wrap_threshold_m_ = declare_parameter("wrap_reset_threshold_m", 1000.0);
    initial_x_m_ = declare_parameter("initial_x_m", 0.0);
    initial_y_m_ = declare_parameter("initial_y_m", 0.0);
    initial_heading_rad_ = declare_parameter("initial_heading_rad", 0.0);
    hardware_articulation_topic_ = declare_parameter("hardware_articulation_topic", std::string("/hardware/articulation_angle"));
    hardware_articulation_timeout_s_ = declare_parameter("hardware_articulation_timeout_seconds", 0.5);
    articulation_state_topic_ = declare_parameter("articulation_state_topic", std::string(""));
    articulation_state_output_topic_ = declare_parameter("articulation_state_output_topic", std::string("mtt/articulation_state"));
    articulation_state_timeout_s_ = declare_parameter("articulation_state_timeout_seconds", 0.5);
    // Default true: fall back to trailer_detector_node's LiDAR-detected angle
    // (proven reliable — see CLAUDE.md: "authoritative phi source") when the
    // hardware potentiometer is stale/absent, instead of silently dropping
    // straight to the open-loop kinematic model estimate. Only engages when
    // hardware is NOT fresh, so this cannot degrade the hardware-available case.
    use_articulation_state_lidar_ = declare_parameter("use_articulation_state_lidar", true);
    prefer_lidar_articulation_ = declare_parameter("prefer_lidar_articulation", true);
    lidar_articulation_topic_ = declare_parameter(
      "lidar_articulation_topic", std::string("trailer/articulation_angle"));
    lidar_detected_topic_ = declare_parameter(
      "lidar_articulation_detected_topic", std::string("trailer/articulation_detected"));
    lidar_articulation_timeout_s_ = declare_parameter("lidar_articulation_timeout_seconds", 0.5);
    imu_yaw_rate_topic_ = declare_parameter("imu_yaw_rate_topic", std::string(""));
    imu_yaw_rate_timeout_s_ = declare_parameter("imu_yaw_rate_timeout_seconds", 0.2);
    imu_yaw_rate_sign_ = declare_parameter("imu_yaw_rate_sign", -1.0);
    imu_yaw_rate_bias_rad_s_ = declare_parameter("imu_yaw_rate_bias_rad_s", 0.0);
    // Complementary filter weight for IMU yaw rate.
    // alpha=1.0 → pure IMU, alpha=0.0 → pure model.
    // Applied only when both IMU and model are valid simultaneously.
    imu_complementary_alpha_ = declare_parameter("imu_complementary_alpha", 0.7);
    use_imu_heading_ = declare_parameter("use_imu_heading", true);
    motion_model_params_.wheelbase_m = declare_parameter("model_wheelbase_m", wheelbase_m_);
    motion_model_params_.max_articulation_rad =
      declare_parameter("model_max_articulation_deg", VehicleParams::max_articulation_deg) * M_PI / 180.0;
    motion_model_params_.min_turn_speed_ms = min_speed_turn_;
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

    if (cmd_angular_mode_ != "normalized_steer" && cmd_angular_mode_ != "yaw_rate") {
      RCLCPP_WARN(
        get_logger(),
        "Unknown cmd_angular_mode '%s', falling back to 'normalized_steer'",
        cmd_angular_mode_.c_str());
      cmd_angular_mode_ = "normalized_steer";
    }

    // ── Initial odometry mode ──
    calculator_ = logic::OdometryFactory::create(
      logic::DrivingMode::SingleTrailer, track_width_m_, wheelbase_m_);
    apply_initial_pose();

    // ── Publishers ──
    odom_pub_        = create_publisher<nav_msgs::msg::Odometry>("mtt_odometry", 10);
    articulation_pub_ = create_publisher<std_msgs::msg::Float64>("mtt_articulation_angle", 10);
    articulation_state_pub_ = create_publisher<mtt_msgs::msg::MttArticulationState>(
      articulation_state_output_topic_, rclcpp::SensorDataQoS());
    if (publish_runtime_joint_states_) {
      joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
        runtime_joint_states_topic_, 10);
    }

    // ── Subscribers ──
    tacho_sub_ = create_subscription<mtt_msgs::msg::MttTachometerData>(
      "mtt_tachometer", rclcpp::SensorDataQoS(),
      [this](const mtt_msgs::msg::MttTachometerData::SharedPtr msg){ on_tachometer(msg); });
    mode_sub_ = create_subscription<mtt_msgs::msg::MttDrivingMode>(
      "mtt_driving_mode", 10,
      [this](const mtt_msgs::msg::MttDrivingMode::SharedPtr msg){ on_mode_change(msg); });
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      cmd_vel_topic_, 10,
      [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg){ on_cmd_vel(msg); });
    hardware_articulation_sub_ = create_subscription<std_msgs::msg::Float64>(
      hardware_articulation_topic_, rclcpp::SensorDataQoS(),
      [this](const std_msgs::msg::Float64::SharedPtr msg){ on_hardware_articulation(msg); });
    if (!articulation_state_topic_.empty()) {
      articulation_state_sub_ = create_subscription<mtt_msgs::msg::MttArticulationState>(
        articulation_state_topic_, rclcpp::SensorDataQoS(),
        [this](const mtt_msgs::msg::MttArticulationState::SharedPtr msg) {
          on_articulation_state(msg);
        });
    }
    if (!imu_yaw_rate_topic_.empty()) {
      imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_yaw_rate_topic_, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
          on_imu(msg);
        });
    }
    // Pitch potentiometer — raw bits (always available from articulation_sensor_node)
    pitch_bits_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/hardware/articulation_pitch_bits", 10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        hardware_pitch_bits_ = msg->data;
        has_pitch_ = true;
        last_pitch_time_ = std::chrono::steady_clock::now();
      });
    // Pitch potentiometer — calibrated radians (only when LUT is configured)
    pitch_rad_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/hardware/articulation_pitch_rad", 10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        hardware_pitch_rad_ = msg->data;
        pitch_rad_calibrated_ = true;
      });
    lidar_articulation_sub_ = create_subscription<std_msgs::msg::Float64>(
      lidar_articulation_topic_, rclcpp::SensorDataQoS(),
      [this](const std_msgs::msg::Float64::SharedPtr msg){ on_lidar_articulation(msg); });
    lidar_detected_sub_ = create_subscription<std_msgs::msg::Bool>(
      lidar_detected_topic_, rclcpp::SensorDataQoS(),
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        lidar_articulation_detected_ = msg->data;
        has_lidar_detection_state_ = true;
        last_lidar_detection_time_ = std::chrono::steady_clock::now();
      });

    // ── Tacho watchdog — warn if no tachometer data arrives after 5s ──
    tacho_watchdog_timer_ = create_wall_timer(std::chrono::seconds(5), [this]() {
      if (!last_tacho_wall_time_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
          "No tachometer data received in 5s — odometry/TF will not update. "
          "If this bag has no tachometer, use 'imu_odom' instead: "
          "MAPPING_CONFIG=.../_config_hesai_imu_replay.yaml docker compose up imu_odom mapping");
      }
    });

    // ── Services ──
    reset_srv_ = create_service<std_srvs::srv::Trigger>(
      "mtt/reset_odometry",
      [this](
        const std_srvs::srv::Trigger::Request::SharedPtr req,
        std_srvs::srv::Trigger::Response::SharedPtr res
      ){ on_reset(req, res); });
    steer_mode_srv_ = create_service<mtt_interfaces::srv::SetSteerControlMode>(
      "mtt/odometry/set_steer_control_mode",
      [this](
        const mtt_interfaces::srv::SetSteerControlMode::Request::SharedPtr req,
        mtt_interfaces::srv::SetSteerControlMode::Response::SharedPtr res
      ){ on_set_steer_mode(req, res); });

    if (broadcast_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
      // Keep publishing the last known odom->base transform and articulation state
      // when telemetry pauses, so the TF tree stays usable for mapping/debugging.
      tf_fallback_timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        [this]() {
          if (!broadcast_tf_ || !tf_broadcaster_) return;
          if (!last_tacho_wall_time_) return;
          bool use_sim_time = false;
          (void)get_parameter("use_sim_time", use_sim_time);
          const auto stamp = now();
          if (use_sim_time && (stamp.nanoseconds() <= 0 || stamp.seconds() >= 1.0e8)) {
            return;
          }
          // Only publish if no tachometer update in the last 200 ms
          auto now_tp = std::chrono::steady_clock::now();
          if (now_tp - last_tacho_time_ < std::chrono::milliseconds(200)) return;
          geometry_msgs::msg::TransformStamped tf;
          double articulation_angle = 0.0;
          tf.header.stamp    = stamp;
          tf.header.frame_id = odom_frame_;
          tf.child_frame_id  = base_frame_;
          {
            std::lock_guard<std::mutex> lock(calc_mutex_);
            auto pose = calculator_->export_pose();
            tf.transform.translation.x = pose.x;
            tf.transform.translation.y = pose.y;
            tf.transform.rotation.z = std::sin(pose.heading / 2.0);
            tf.transform.rotation.w = std::cos(pose.heading / 2.0);
            articulation_angle = pose.articulation_angle;
          }
          publish_runtime_joint_state(tf.header.stamp, articulation_angle);
          tf_broadcaster_->sendTransform(tf);
        });
    }

    RCLCPP_INFO(
      get_logger(),
      "MttOdometryNode started (mode=SingleTrailer, steer=%s, cmd_angular_mode=%s, cmd_vel=%s, hardware_articulation=%s, initial=(%.2f, %.2f, %.1f deg), runtime_joint_states=%s)",
      steer_mode_.c_str(),
      cmd_angular_mode_.c_str(),
      cmd_vel_topic_.c_str(),
      hardware_articulation_topic_.c_str(),
      initial_x_m_,
      initial_y_m_,
      initial_heading_rad_ * 180.0 / M_PI,
      publish_runtime_joint_states_ ? runtime_joint_states_topic_.c_str() : "disabled");
    if (!articulation_state_topic_.empty()) {
      RCLCPP_INFO(
        get_logger(),
        "Articulation state input enabled: input=%s output=%s lidar_fallback=%s",
        articulation_state_topic_.c_str(),
        articulation_state_output_topic_.c_str(),
        use_articulation_state_lidar_ ? "true" : "false");
    }
    if (!imu_yaw_rate_topic_.empty()) {
      RCLCPP_INFO(
        get_logger(),
        "IMU yaw-rate input enabled: topic=%s sign=%.1f bias=%.4f timeout=%.2fs",
        imu_yaw_rate_topic_.c_str(),
        imu_yaw_rate_sign_,
        imu_yaw_rate_bias_rad_s_,
        imu_yaw_rate_timeout_s_);
    }
  }

private:
  void apply_initial_pose()
  {
    logic::OdometryPose pose;
    pose.x = initial_x_m_;
    pose.y = initial_y_m_;
    pose.heading = initial_heading_rad_;
    pose.articulation_angle = 0.0;
    pose.last_abs_m.reset();
    calculator_->import_pose(pose);
  }

  // ── State ──
  std::string odom_frame_, base_frame_, steer_mode_, cmd_angular_mode_, cmd_vel_topic_, runtime_joint_states_topic_;
  bool     broadcast_tf_, pivot_turn_, publish_runtime_joint_states_;
  double   track_width_m_, wheelbase_m_;
  double   max_articulation_rad_{VehicleParams::max_articulation_rad};
  double   runtime_joint_pitch_rad_{0.0};
  double   runtime_joint_roll_rad_{0.0};
  double   runtime_joint_articulation_sign_{1.0};
  double   runtime_joint_articulation_offset_rad_{0.0};
  double   min_speed_turn_, yaw_slip_factor_, wrap_threshold_m_, cmd_vel_timeout_s_;
  double   initial_x_m_{0.0};
  double   initial_y_m_{0.0};
  double   initial_heading_rad_{0.0};
  double   hardware_articulation_timeout_s_{0.5};
  double   articulation_state_timeout_s_{0.5};
  double   lidar_articulation_timeout_s_{0.5};
  double   imu_yaw_rate_timeout_s_{0.2};
  double   imu_yaw_rate_sign_{-1.0};
  double   imu_yaw_rate_bias_rad_s_{0.0};
  double   current_angular_cmd_{0.0};
  double   hardware_articulation_rad_{0.0};
  bool     has_hardware_articulation_{false};
  std::chrono::steady_clock::time_point last_hardware_articulation_time_{};
  std::string hardware_articulation_topic_;
  std::string articulation_state_topic_;
  std::string articulation_state_output_topic_;
  bool     use_articulation_state_lidar_{false};
  bool     prefer_lidar_articulation_{true};
  double   state_articulation_rad_{0.0};
  bool     has_state_articulation_{false};
  std::string state_articulation_source_;
  std::chrono::steady_clock::time_point last_state_articulation_time_{};
  double   lidar_articulation_rad_{0.0};
  bool     has_lidar_articulation_{false};
  bool     lidar_articulation_detected_{false};
  bool     has_lidar_detection_state_{false};
  std::string lidar_articulation_topic_;
  std::string lidar_detected_topic_;
  std::chrono::steady_clock::time_point last_lidar_articulation_time_{};
  std::chrono::steady_clock::time_point last_lidar_detection_time_{};
  std::string imu_yaw_rate_topic_;
  double   imu_yaw_rate_rad_s_{0.0};
  double   imu_heading_rad_{0.0};
  double   initial_imu_heading_rad_{0.0};
  bool     has_imu_yaw_rate_{false};
  bool     has_imu_heading_{false};
  bool     has_initial_imu_heading_{false};
  std::chrono::steady_clock::time_point last_imu_yaw_rate_time_{};
  double   imu_complementary_alpha_{0.7};
  bool     use_imu_heading_{true};
  logic::YawRateSource yaw_rate_source_snapshot_{logic::YawRateSource::MODEL_ONLY};
  // Pitch potentiometer (ADC1, 8-bit)
  double   hardware_pitch_bits_{0.0};
  double   hardware_pitch_rad_{0.0};
  bool     has_pitch_{false};
  bool     pitch_rad_calibrated_{false};
  std::chrono::steady_clock::time_point last_pitch_time_{};
  logic::CommandMotionParams motion_model_params_{};
  std::chrono::steady_clock::time_point last_tacho_time_{};
  std::chrono::steady_clock::time_point last_cmd_vel_time_{};
  bool has_cmd_vel_{false};
  std::optional<rclcpp::Time> last_tacho_stamp_;
  std::optional<std::chrono::steady_clock::time_point> last_tacho_wall_time_;
  std::optional<double> last_tacho_distance_m_;

  std::mutex calc_mutex_;
  std::mutex state_mutex_;
  std::unique_ptr<logic::IOdometryCalculator> calculator_;
  logic::DrivingMode current_mode_{logic::DrivingMode::SingleTrailer};

  // ── ROS I/O ──
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr  articulation_pub_;
  rclcpp::Publisher<mtt_msgs::msg::MttArticulationState>::SharedPtr articulation_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Subscription<mtt_msgs::msg::MttTachometerData>::SharedPtr tacho_sub_;
  rclcpp::Subscription<mtt_msgs::msg::MttDrivingMode>::SharedPtr    mode_sub_;
  rclcpp::TimerBase::SharedPtr tf_fallback_timer_;
  rclcpp::TimerBase::SharedPtr tacho_watchdog_timer_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr hardware_articulation_sub_;
  rclcpp::Subscription<mtt_msgs::msg::MttArticulationState>::SharedPtr articulation_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr pitch_bits_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr pitch_rad_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr lidar_articulation_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lidar_detected_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr  reset_srv_;
  rclcpp::Service<mtt_interfaces::srv::SetSteerControlMode>::SharedPtr steer_mode_srv_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  double normalized_steer_to_yaw_rate(double normalized_steer, double speed_ms) const
  {
    return logic::CommandMotionModel::yaw_rate_from_speed_and_steer(
      normalized_steer,
      speed_ms,
      motion_model_params_,
      motion_model_params_.use_slip_heuristic);
  }

  double articulation_to_yaw_rate(double articulation_rad, double speed_ms) const
  {
    return logic::CommandMotionModel::yaw_rate_from_speed_and_articulation(
      speed_ms,
      articulation_rad,
      motion_model_params_,
      motion_model_params_.use_slip_heuristic);
  }

  double command_to_yaw_rate(double angular_cmd, double speed_ms) const
  {
    if (cmd_angular_mode_ == "yaw_rate") {
      return angular_cmd;
    }

    return normalized_steer_to_yaw_rate(angular_cmd, speed_ms);
  }

  bool tachometer_distance_delta_suspicious(
    double raw_delta_m,
    double signed_speed_ms,
    double dt) const
  {
    if (!std::isfinite(raw_delta_m) || !std::isfinite(dt) || dt <= 1e-6) {
      return true;
    }
    if (raw_delta_m < 0.0) {
      return true;
    }

    constexpr double kMinAllowedSpeedMs = 8.0;
    constexpr double kSpeedMarginMs = 1.0;
    constexpr double kAbsDeltaMarginM = 0.05;
    constexpr double kRelativeDeltaMargin = 1.5;
    const double abs_delta_m = std::abs(raw_delta_m);
    const double abs_speed_ms = std::abs(signed_speed_ms);
    constexpr double kMovingSpeedThresholdMs = 0.05;
    constexpr double kStuckDistanceEpsilonM = 1.0e-6;
    if (abs_speed_ms > kMovingSpeedThresholdMs && abs_delta_m <= kStuckDistanceEpsilonM) {
      return true;
    }

    const double implied_speed_ms = abs_delta_m / dt;
    const double max_allowed_speed_ms = std::max(
      kMinAllowedSpeedMs,
      std::max(VehicleParams::max_speed_ms * 1.5, abs_speed_ms + kSpeedMarginMs));
    const double expected_delta_m = abs_speed_ms * dt;
    const double delta_margin_m = std::max(kAbsDeltaMarginM, kRelativeDeltaMargin * expected_delta_m);
    return implied_speed_ms > max_allowed_speed_ms ||
      std::abs(abs_delta_m - expected_delta_m) > delta_margin_m;
  }

  void publish_runtime_joint_state(const builtin_interfaces::msg::Time& stamp, double articulation_angle)
  {
    if (!publish_runtime_joint_states_ || !joint_state_pub_) {
      return;
    }

    const double runtime_yaw = std::clamp(
      runtime_joint_articulation_sign_ * articulation_angle + runtime_joint_articulation_offset_rad_,
      -max_articulation_rad_,
      max_articulation_rad_);

    sensor_msgs::msg::JointState msg;
    msg.header.stamp = stamp;
    msg.name = {"pitch", "yaw", "roll"};
    msg.position = {runtime_joint_pitch_rad_, runtime_yaw, runtime_joint_roll_rad_};
    joint_state_pub_->publish(msg);
  }

  // ── Callbacks ──
  void on_hardware_articulation(const std_msgs::msg::Float64::SharedPtr msg) {
    if (!std::isfinite(msg->data)) {
      return;
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    hardware_articulation_rad_ = msg->data;
    last_hardware_articulation_time_ = std::chrono::steady_clock::now();
    has_hardware_articulation_ = true;
  }

  void on_articulation_state(const mtt_msgs::msg::MttArticulationState::SharedPtr msg) {
    double angle = 0.0;
    const char* source = nullptr;

    if (msg->hardware_fresh && std::isfinite(msg->hardware_rad)) {
      angle = msg->hardware_rad;
      source = "state_hardware";
    } else if (
      use_articulation_state_lidar_ &&
      msg->lidar_detected &&
      std::isfinite(msg->lidar_rad))
    {
      angle = msg->lidar_rad;
      source = "state_lidar";
    } else if (
      msg->effective_source == "hardware" &&
      std::isfinite(msg->effective_rad))
    {
      angle = msg->effective_rad;
      source = "state_effective";
    }

    if (!source) {
      return;
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    state_articulation_rad_ = angle;
    state_articulation_source_ = source;
    last_state_articulation_time_ = std::chrono::steady_clock::now();
    has_state_articulation_ = true;
  }

  void on_lidar_articulation(const std_msgs::msg::Float64::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    lidar_articulation_rad_ = msg->data;
    last_lidar_articulation_time_ = std::chrono::steady_clock::now();
    has_lidar_articulation_ = true;
  }

  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg) {
    const double yaw_rate = imu_yaw_rate_sign_ * msg->angular_velocity.z - imu_yaw_rate_bias_rad_s_;
    if (!std::isfinite(yaw_rate)) {
      return;
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    imu_yaw_rate_rad_s_ = yaw_rate;
    last_imu_yaw_rate_time_ = std::chrono::steady_clock::now();
    has_imu_yaw_rate_ = true;
    const double raw_heading = std::atan2(
        2.0 * (msg->orientation.w * msg->orientation.z + msg->orientation.x * msg->orientation.y),
        1.0 - 2.0 * (msg->orientation.y * msg->orientation.y + msg->orientation.z * msg->orientation.z));
    if (!has_initial_imu_heading_) {
      initial_imu_heading_rad_ = raw_heading;
      has_initial_imu_heading_ = true;
    }
    imu_heading_rad_ = raw_heading - initial_imu_heading_rad_;
    has_imu_heading_ = true;
  }

  void on_cmd_vel(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    if (!std::isfinite(msg->twist.angular.z)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Ignoring cmd_vel with non-finite angular.z: %f", msg->twist.angular.z);
      return;
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_angular_cmd_ = msg->twist.angular.z;
    last_cmd_vel_time_ = std::chrono::steady_clock::now();
    has_cmd_vel_ = true;
  }

  void on_tachometer(const mtt_msgs::msg::MttTachometerData::SharedPtr msg)
  {
    const auto wall_now = std::chrono::steady_clock::now();
    last_tacho_time_ = wall_now;
    const double speed_ms   = std::isfinite(msg->speed_ms) ? msg->speed_ms : 0.0;
    const double steer_cmd  = std::isfinite(msg->steer_cmd) ? msg->steer_cmd : 0.0;
    if (!std::isfinite(msg->speed_ms) || !std::isfinite(msg->steer_cmd)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Non-finite tachometer data: speed_ms=%f steer_cmd=%f — clamped to 0",
        msg->speed_ms, msg->steer_cmd);
    }
    int dir_sign = (msg->direction == "Reverse") ? -1 : 1;
    double signed_speed_ms = 0.0;
    if (msg->model_state_valid && std::abs(msg->model_speed_ms) > 1e-4) {
      signed_speed_ms = msg->model_speed_ms;
    } else if (speed_ms < -1e-4) {
      signed_speed_ms = speed_ms;
    } else {
      signed_speed_ms = speed_ms * static_cast<double>(dir_sign);
    }
    if (std::abs(signed_speed_ms) > 1e-4) {
      dir_sign = signed_speed_ms < 0.0 ? -1 : 1;
    }

    double dt = 0.02;
    bool use_sim_time = false;
    (void)get_parameter("use_sim_time", use_sim_time);
    if (msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0) {
      const auto stamp = rclcpp::Time(msg->header.stamp);
      if (last_tacho_stamp_) {
        dt = (stamp - *last_tacho_stamp_).seconds();
      }
      last_tacho_stamp_ = stamp;
    }
    if (use_sim_time && dt <= 1e-4) {
      last_tacho_wall_time_ = wall_now;
      return;
    }
    if (!use_sim_time && (dt <= 1e-4 || dt > 1.0) && last_tacho_wall_time_) {
      dt = std::chrono::duration<double>(wall_now - *last_tacho_wall_time_).count();
    }
    last_tacho_wall_time_ = wall_now;
    if (dt <= 1e-4 || dt > 1.0) {
      dt = 0.02;
    }

    const double cur_tacho_distance_m = msg->distance_km * 1000.0;
    if (!msg->model_state_valid && last_tacho_distance_m_) {
      const double raw_delta_m = cur_tacho_distance_m - *last_tacho_distance_m_;
      if (tachometer_distance_delta_suspicious(raw_delta_m, signed_speed_ms, dt)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          1000,
          "Suspicious tachometer cumulative distance delta: raw=%.3fm dt=%.3fs implied=%.1fm/s speed_msg=%.3fm/s. Integrating speed_ms instead.",
          raw_delta_m,
          dt,
          dt > 1e-6 ? std::abs(raw_delta_m) / dt : 0.0,
          signed_speed_ms);
      }
    }
    last_tacho_distance_m_ = cur_tacho_distance_m;

    // Use only validated measurements. Live operation prefers LiDAR when it is
    // explicitly detected; STM remains the secondary source. Replay can provide
    // the same information through an external composite state message.
    bool direct_hardware_is_fresh = false;
    bool direct_lidar_is_fresh = false;
    double direct_hardware_angle = 0.0;
    bool measured_articulation_is_fresh = false;
    double measured_articulation_angle = 0.0;
    std::string measured_articulation_source = "model";
    bool imu_yaw_rate_is_fresh = false;
    double imu_yaw_rate = 0.0;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      direct_hardware_is_fresh = has_hardware_articulation_ &&
        std::chrono::duration<double>(wall_now - last_hardware_articulation_time_).count() <= hardware_articulation_timeout_s_;
      direct_hardware_angle = hardware_articulation_rad_;
      const bool state_articulation_is_fresh =
        has_state_articulation_ &&
        std::chrono::duration<double>(wall_now - last_state_articulation_time_).count() <= articulation_state_timeout_s_;
      const bool detection_is_fresh = has_lidar_detection_state_ &&
        std::chrono::duration<double>(
          wall_now - last_lidar_detection_time_).count() <= lidar_articulation_timeout_s_;
      direct_lidar_is_fresh = use_articulation_state_lidar_ &&
        has_lidar_articulation_ &&
        detection_is_fresh && lidar_articulation_detected_ &&
        std::chrono::duration<double>(
          wall_now - last_lidar_articulation_time_).count() <= lidar_articulation_timeout_s_;
      const auto selection = logic::select_articulation_measurement(
        prefer_lidar_articulation_,
        direct_lidar_is_fresh, lidar_articulation_rad_,
        direct_hardware_is_fresh, direct_hardware_angle,
        state_articulation_is_fresh, state_articulation_rad_);
      measured_articulation_is_fresh = selection.valid;
      measured_articulation_angle = selection.angle_rad;
      switch (selection.source) {
        case logic::ArticulationMeasurementSource::LIDAR:
          measured_articulation_source = "lidar";
          break;
        case logic::ArticulationMeasurementSource::HARDWARE:
          measured_articulation_source = "hardware";
          break;
        case logic::ArticulationMeasurementSource::EXTERNAL_STATE:
          measured_articulation_source = state_articulation_source_;
          break;
        case logic::ArticulationMeasurementSource::NONE:
          break;
      }
      imu_yaw_rate_is_fresh = has_imu_yaw_rate_ &&
        std::chrono::duration<double>(wall_now - last_imu_yaw_rate_time_).count() <= imu_yaw_rate_timeout_s_;
      imu_yaw_rate = imu_yaw_rate_rad_s_;
    }

    // Compute effective angular velocity
    double eff_ang = 0.0;
    bool closed_loop = false;
    logic::DrivingMode mode_snapshot;
    double current_angular_cmd = 0.0;
    bool cmd_is_fresh = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      closed_loop = (steer_mode_ == "closed_loop");
      if (msg->model_state_valid) {
        eff_ang = msg->model_yaw_rate_effective_rad_s;
      } else if (closed_loop) {
        eff_ang = measured_articulation_is_fresh
          ? articulation_to_yaw_rate(measured_articulation_angle, signed_speed_ms)
          : normalized_steer_to_yaw_rate(steer_cmd, signed_speed_ms);
      } else {
        cmd_is_fresh =
          has_cmd_vel_ &&
          std::chrono::duration<double>(wall_now - last_cmd_vel_time_).count() <= cmd_vel_timeout_s_;
        current_angular_cmd = current_angular_cmd_;
        eff_ang = cmd_is_fresh ? command_to_yaw_rate(current_angular_cmd, signed_speed_ms) : 0.0;
      }
      // IMU yaw rate fusion via complementary filter.
      // When both IMU and model are valid: blend (alpha*IMU + (1-alpha)*model).
      // When model is invalid but IMU is fresh: use IMU directly.
      // When IMU is stale: fall through with model/closed-loop estimate.
      if (imu_yaw_rate_is_fresh) {
        if (msg->model_state_valid) {
          eff_ang = imu_complementary_alpha_ * imu_yaw_rate +
                    (1.0 - imu_complementary_alpha_) * eff_ang;
        } else {
          eff_ang = imu_yaw_rate;
        }
      }
      // Track which source was used for covariance scaling downstream
      yaw_rate_source_snapshot_ =
        (imu_yaw_rate_is_fresh && msg->model_state_valid)
          ? logic::YawRateSource::IMU_BLEND
        : imu_yaw_rate_is_fresh
          ? logic::YawRateSource::IMU_ONLY
        : measured_articulation_is_fresh
          ? logic::YawRateSource::HARDWARE_CLOSED_LOOP
          : logic::YawRateSource::MODEL_ONLY;
    }
    {
      std::lock_guard<std::mutex> lock(calc_mutex_);
      mode_snapshot = current_mode_;
    }

    // Suppress yaw when near-stationary (no pivot turns unless enabled)
    if (!pivot_turn_ && std::abs(signed_speed_ms) < min_speed_turn_) eff_ang = 0.0;
    eff_ang = std::clamp(eff_ang * (msg->model_state_valid ? 1.0 : yaw_slip_factor_),
                         -VehicleParams::max_yaw_rate_rad_s,
                          VehicleParams::max_yaw_rate_rad_s);

    logic::OdometryInput input;
    input.distance_km      = msg->distance_km;
    input.speed_ms         = signed_speed_ms;
    input.steer_cmd        = steer_cmd;
    input.angular_velocity = eff_ang;
    input.direction_sign   = dir_sign;
    input.dt               = dt;
    if (use_imu_heading_ && has_imu_heading_) {
      input.imu_heading = -imu_heading_rad_;
    }
    input.synthetic_model_valid = msg->model_state_valid;
    input.articulation_command_rad = msg->model_articulation_command_rad;
    input.articulation_measurement_valid = measured_articulation_is_fresh;
    input.yaw_rate_source  = yaw_rate_source_snapshot_;

    if (measured_articulation_is_fresh) {
      input.articulation_effective_rad = measured_articulation_angle;
    } else {
      input.articulation_effective_rad = msg->model_articulation_effective_rad;
    }

    input.curvature_nominal_m_inv = msg->model_curvature_nominal_m_inv;
    input.curvature_effective_m_inv = msg->model_curvature_effective_m_inv;
    input.yaw_rate_nominal_rad_s = msg->model_yaw_rate_nominal_rad_s;
    input.yaw_rate_effective_rad_s = msg->model_yaw_rate_effective_rad_s;

    logic::OdometryOutput out;
    {
      std::lock_guard<std::mutex> lock(calc_mutex_);
      out = calculator_->update(input);
      mode_snapshot = current_mode_;
    }

    // Build and publish nav_msgs/Odometry
    nav_msgs::msg::Odometry odom;
    if (msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0) {
      odom.header.stamp = msg->header.stamp;
    } else {
      odom.header.stamp = now();
    }
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id  = base_frame_;
    odom.pose.pose.position.x = out.x;
    odom.pose.pose.position.y = out.y;
    odom.pose.pose.orientation.z = std::sin(out.heading / 2.0);
    odom.pose.pose.orientation.w = std::cos(out.heading / 2.0);
    odom.twist.twist.linear.x  = out.vx;
    odom.twist.twist.angular.z = out.wz;
    // Covariances
    odom.pose.covariance[0]  = out.pos_cov;
    odom.pose.covariance[7]  = out.pos_cov;
    odom.pose.covariance[35] = out.heading_cov;
    odom.twist.covariance[0]  = out.vel_cov;
    odom.twist.covariance[35] = out.heading_cov * 2.0;
    odom_pub_->publish(odom);

    // Publish articulation angle for joint controller
    if (mode_snapshot == logic::DrivingMode::SingleTrailer) {
      std_msgs::msg::Float64 artic;
      artic.data = out.articulation_angle;
      articulation_pub_->publish(artic);
    }
    publish_runtime_joint_state(odom.header.stamp, out.articulation_angle);

    // Publish unified articulation state (mtt/articulation_state)
    {
      double lidar_rad = 0.0;
      bool lidar_detected = false;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        lidar_detected = direct_lidar_is_fresh;
        lidar_rad = lidar_articulation_rad_;
      }

      double pitch_bits = 0.0, pitch_rad = 0.0;
      bool pitch_fresh = false;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        pitch_bits  = hardware_pitch_bits_;
        pitch_rad   = hardware_pitch_rad_;
        pitch_fresh = has_pitch_ &&
          std::chrono::duration<double>(wall_now - last_pitch_time_).count()
            <= hardware_articulation_timeout_s_;
      }

      mtt_msgs::msg::MttArticulationState state_msg;
      state_msg.header = odom.header;
      state_msg.command_rad = msg->model_articulation_command_rad;
      state_msg.command_valid = msg->model_state_valid;
      state_msg.hardware_rad = direct_hardware_angle;
      state_msg.hardware_fresh = direct_hardware_is_fresh;
      state_msg.effective_rad = input.articulation_effective_rad;
      state_msg.effective_source = measured_articulation_is_fresh
        ? measured_articulation_source
        : "model";
      state_msg.pitch_rad      = pitch_rad_calibrated_ ? pitch_rad : 0.0;
      state_msg.pitch_bits_raw = pitch_bits;
      // pitch_fresh requires BOTH fresh bits AND a calibrated LUT.
      // Without LUT calibration, pitch_rad=0 and marking it "fresh" would
      // inject a false α=0 prior with hardware precision into the factor graph.
      state_msg.pitch_fresh    = pitch_fresh && pitch_rad_calibrated_;
      state_msg.lidar_rad = lidar_rad;
      state_msg.lidar_detected = lidar_detected;
      state_msg.command_residual_rad = state_msg.command_rad - state_msg.effective_rad;
      state_msg.hardware_lidar_residual_rad =
        (direct_hardware_is_fresh && lidar_detected) ? (direct_hardware_angle - lidar_rad) : 0.0;
      articulation_state_pub_->publish(state_msg);
    }

    // TF broadcast
    if (broadcast_tf_ && tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header = odom.header;
      tf.child_frame_id = base_frame_;
      tf.transform.translation.x = out.x;
      tf.transform.translation.y = out.y;
      tf.transform.rotation = odom.pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf);
    }
  }

  void on_mode_change(const mtt_msgs::msg::MttDrivingMode::SharedPtr msg)
  {
    auto new_mode = static_cast<logic::DrivingMode>(msg->mode);
    if (new_mode == current_mode_) return;

    std::unique_ptr<logic::IOdometryCalculator> new_calc;
    try {
      new_calc = logic::OdometryFactory::create(new_mode, track_width_m_, wheelbase_m_);
    } catch (const std::invalid_argument& e) {
      RCLCPP_ERROR(get_logger(),
        "Invalid driving mode %d (%s) — keeping current mode", msg->mode, e.what());
      return;
    }
    {
      std::lock_guard<std::mutex> lock(calc_mutex_);
      // Transfer pose so there's no jump at mode switch
      new_calc->import_pose(calculator_->export_pose());
      calculator_ = std::move(new_calc);
      current_mode_ = new_mode;
    }
    RCLCPP_INFO(get_logger(), "Odometry mode → %s", calculator_->mode_name().c_str());
  }

  void on_reset(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr res)
  {
    std::lock_guard<std::mutex> lock(calc_mutex_);
    calculator_->reset();
    last_tacho_distance_m_.reset();
    apply_initial_pose();
    res->success = true;
    res->message = "Odometry reset";
  }

  void on_set_steer_mode(
    const mtt_interfaces::srv::SetSteerControlMode::Request::SharedPtr req,
    mtt_interfaces::srv::SetSteerControlMode::Response::SharedPtr res)
  {
    if (req->control_mode != "open_loop" && req->control_mode != "closed_loop") {
      res->success = false;
      res->message = "Invalid control_mode. Use 'open_loop' or 'closed_loop'";
      return;
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    steer_mode_ = req->control_mode;
    res->success = true;
    res->message = "Steer control mode set to " + steer_mode_;
    RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
  }
};

}  // namespace mtt

RCLCPP_COMPONENTS_REGISTER_NODE(mtt::MttOdometryNode)
