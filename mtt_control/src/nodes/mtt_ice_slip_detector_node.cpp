#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mtt_msgs/msg/mtt_tachometer_data.hpp>
#include <mtt_msgs/msg/mtt_vehicle_status.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/string.hpp>

namespace
{

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

bool finite_pose(const geometry_msgs::msg::Pose & pose)
{
  return std::isfinite(pose.position.x) &&
         std::isfinite(pose.position.y) &&
         std::isfinite(pose.orientation.x) &&
         std::isfinite(pose.orientation.y) &&
         std::isfinite(pose.orientation.z) &&
         std::isfinite(pose.orientation.w);
}

double finite_or_zero(double value)
{
  return std::isfinite(value) ? value : 0.0;
}

}  // namespace

namespace mtt_control
{

class MttIceSlipDetectorNode : public rclcpp::Node
{
public:
  // Reference pose-source priority, highest first. VSLAM is preferred
  // (GPU-accelerated, ~60Hz, real loop-closure SLAM) but is also the most
  // likely to actually fail on ice (textureless white surface, glare) --
  // ICP (/mapping/icp_odom) is the proven fallback, and IMU-derived odom
  // (/imu_odom, dead-reckoning-ish, drifts fastest but never depends on a
  // camera or a converged map) is the last resort so this node still
  // reports SOMETHING rather than going silent. Added 2026-07-29 alongside
  // wiring isaac_vslam into demos/mathis_com_shift -- mirrors the same
  // vslam-primary/icp-fallback pattern already proven in
  // scripts/mtt_experiment_monitor.py (data_collection), extended here with
  // an IMU-odom third tier that the monitor script does not have.
  enum class Source
  {
    kVslam,
    kIcp,
    kImuOdom,
    kNone,
  };

  static const char * source_name(Source s)
  {
    switch (s) {
      case Source::kVslam: return "vslam";
      case Source::kIcp: return "icp";
      case Source::kImuOdom: return "imu_odom";
      default: return "none";
    }
  }

  explicit MttIceSlipDetectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("mtt_ice_slip_detector_node", options)
  {
    cmd_topic_ = declare_parameter("cmd_topic", std::string("cmd_vel"));
    vslam_odom_topic_ = declare_parameter("vslam_odom_topic", std::string("/isaac/vslam/odometry"));
    icp_odom_topic_ = declare_parameter("icp_odom_topic", std::string("/mapping/icp_odom"));
    imu_odom_topic_ = declare_parameter("imu_odom_topic", std::string("/imu_odom"));
    tacho_topic_ = declare_parameter("tacho_topic", std::string("/mtt_tachometer"));
    vehicle_status_topic_ = declare_parameter("vehicle_status_topic", std::string("/mtt_status"));
    com_direction_topic_ =
      declare_parameter("com_direction_topic", std::string("mtt_control/com_direction_sign"));
    imu_topic_ = declare_parameter("imu_topic", std::string("/vectornav/imu"));

    slip_threshold_ = declare_parameter("slip_threshold", 0.20);
    min_cmd_speed_ms_ = declare_parameter("min_cmd_speed_ms", 0.05);
    min_odom_dt_s_ = declare_parameter("min_odom_dt_s", 0.02);
    max_odom_dt_s_ = declare_parameter("max_odom_dt_s", 0.50);
    tacho_fresh_timeout_s_ = declare_parameter("tacho_fresh_timeout_s", 0.25);
    cmd_fresh_timeout_s_ = declare_parameter("cmd_fresh_timeout_s", 0.50);
    use_synthetic_tachometer_ = declare_parameter("use_synthetic_tachometer", false);

    // Staleness gates for the fallback chain: a source is only eligible to
    // be "active" if a message was received within this window of NOW (wall
    // clock, not message stamp -- a source that stops publishing entirely,
    // e.g. the isaac_vslam container dying, must fall through even though
    // its last message's own header stamp doesn't age on its own).
    // 0.3s matches mtt_experiment_monitor.py's vslam_staleness_s.
    vslam_staleness_s_ = declare_parameter("vslam_staleness_s", 0.3);
    icp_staleness_s_ = declare_parameter("icp_staleness_s", 0.3);
    // /imu_odom is the last resort -- give it a little more slack since it
    // is not expected to run at as tight a rate as VSLAM/ICP in every demo.
    imu_odom_staleness_s_ = declare_parameter("imu_odom_staleness_s", 0.5);

    filter_process_variance_ = declare_parameter("filter_process_variance", 0.20);
    // Per-source measurement variance for the tachometer Kalman fusion --
    // VSLAM is noisier/jumpier frame-to-frame than ICP (no map-level
    // optimization backing each pose the way ICP has), IMU odom drifts
    // fastest of the three. Loosen in that order unless field data says
    // otherwise -- these are starting points, not validated on ice yet.
    vslam_speed_variance_ = declare_parameter("vslam_speed_variance", 0.10);
    icp_speed_variance_ = declare_parameter("icp_speed_variance", 0.04);
    imu_odom_speed_variance_ = declare_parameter("imu_odom_speed_variance", 0.25);
    tacho_speed_variance_ = declare_parameter("tacho_speed_variance", 1.00);

    auto_com_switch_enabled_ = declare_parameter("auto_com_switch_enabled", false);
    auto_com_switch_cooldown_s_ = declare_parameter("auto_com_switch_cooldown_s", 2.0);

    cmd_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      cmd_topic_, 20,
      [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) { on_cmd(msg); });
    vslam_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      vslam_odom_topic_, 20,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) { on_vslam_odom(msg); });
    icp_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      icp_odom_topic_, 20,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) { on_icp_odom(msg); });
    imu_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      imu_odom_topic_, 20,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) { on_imu_odom(msg); });
    tacho_sub_ = create_subscription<mtt_msgs::msg::MttTachometerData>(
      tacho_topic_, 20,
      [this](const mtt_msgs::msg::MttTachometerData::SharedPtr msg) { on_tacho(msg); });
    vehicle_status_sub_ = create_subscription<mtt_msgs::msg::MttVehicleStatus>(
      vehicle_status_topic_, 20,
      [this](const mtt_msgs::msg::MttVehicleStatus::SharedPtr msg) { on_vehicle_status(msg); });
    com_direction_sub_ = create_subscription<std_msgs::msg::Float64>(
      com_direction_topic_, 10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) { on_com_direction(msg); });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, 20,
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) { on_imu(msg); });

    slip_bool_pub_ = create_publisher<std_msgs::msg::Bool>("/ice_slip/slip_detected", 10);
    slip_int_pub_ = create_publisher<std_msgs::msg::Int8>("/slip_detected", 10);
    slip_ratio_pub_ = create_publisher<std_msgs::msg::Float64>("/ice_slip/slip_ratio", 10);
    // "icp_slip_ratio"/"body_speed_ms" names kept for dashboard compatibility,
    // but as of 2026-07-29 they report whatever source is currently ACTIVE
    // (vslam/icp/imu_odom), not always literally ICP -- see
    // /ice_slip/active_pose_source to know which. Per-source raw speeds
    // below are always published regardless of which is active, so VSLAM's
    // own contribution is directly observable even when it isn't selected.
    icp_slip_ratio_pub_ = create_publisher<std_msgs::msg::Float64>("/ice_slip/icp_slip_ratio", 10);
    icp_speed_pub_ = create_publisher<std_msgs::msg::Float64>("/ice_slip/body_speed_ms", 10);
    fused_speed_pub_ = create_publisher<std_msgs::msg::Float64>("/ice_slip/fused_speed_ms", 10);
    tacho_speed_pub_ = create_publisher<std_msgs::msg::Float64>("/ice_slip/tacho_speed_ms", 10);
    cmd_speed_pub_ = create_publisher<std_msgs::msg::Float64>("/ice_slip/cmd_speed_ms", 10);
    driver_command_speed_pub_ =
      create_publisher<std_msgs::msg::Float64>("/ice_slip/driver_command_speed_ms", 10);
    driver_effective_command_speed_pub_ =
      create_publisher<std_msgs::msg::Float64>("/ice_slip/driver_effective_command_speed_ms", 10);
    cmd_driver_delta_pub_ =
      create_publisher<std_msgs::msg::Float64>("/ice_slip/cmd_driver_delta_ms", 10);
    yaw_rate_mismatch_pub_ =
      create_publisher<std_msgs::msg::Float64>("/ice_slip/yaw_rate_mismatch_rad_s", 10);
    lateral_speed_pub_ =
      create_publisher<std_msgs::msg::Float64>("/ice_slip/lateral_speed_ms", 10);
    auto_switch_enabled_pub_ =
      create_publisher<std_msgs::msg::Bool>("/ice_slip/auto_com_switch_enabled", 10);
    com_direction_state_pub_ =
      create_publisher<std_msgs::msg::Float64>("/ice_slip/com_direction_sign", 10);
    com_direction_pub_ = create_publisher<std_msgs::msg::Float64>(com_direction_topic_, 10);

    // Visibility into the fallback chain -- added 2026-07-29 specifically so
    // "is VSLAM actually being consumed" is directly answerable from a topic
    // echo/Foxglove panel, not an inference from code reading.
    active_pose_source_pub_ = create_publisher<std_msgs::msg::String>("/ice_slip/active_pose_source", 10);
    vslam_speed_pub_ = create_publisher<std_msgs::msg::Float64>("/ice_slip/vslam_speed_ms", 10);
    icp_raw_speed_pub_ = create_publisher<std_msgs::msg::Float64>("/ice_slip/icp_raw_speed_ms", 10);
    imu_odom_speed_pub_ = create_publisher<std_msgs::msg::Float64>("/ice_slip/imu_odom_speed_ms", 10);

    RCLCPP_INFO(
      get_logger(),
      "Ice slip detector ready: cmd=%s pose_source_priority=[vslam:%s icp:%s imu_odom:%s] "
      "tacho=%s status=%s threshold=%.2f auto_com_switch=%s",
      cmd_topic_.c_str(), vslam_odom_topic_.c_str(), icp_odom_topic_.c_str(),
      imu_odom_topic_.c_str(), tacho_topic_.c_str(), vehicle_status_topic_.c_str(),
      slip_threshold_, auto_com_switch_enabled_ ? "true" : "false");
  }

private:
  // Per-source consecutive-pose differencer. Kept fully independent per
  // source (never differences across two DIFFERENT sources' messages) --
  // ICP's map frame and Isaac VSLAM's own map/odom frame do not share an
  // origin, so a position delta computed between a VSLAM message and the
  // next ICP message (e.g. right at a failover) would be nonsense, not a
  // real velocity. Each source's body speed is always computed purely from
  // its own consecutive messages; only the OUTPUT SELECTION (which source's
  // already-computed speed actually drives slip detection this tick)
  // depends on the priority/staleness logic below.
  struct OdomSourceState
  {
    bool has_prev{false};
    rclcpp::Time prev_stamp{0, 0, RCL_ROS_TIME};
    double prev_x{0.0};
    double prev_y{0.0};
    double prev_yaw{0.0};

    bool has_speed{false};
    double body_speed_ms{0.0};
    double lateral_speed_ms{0.0};

    bool ever_seen{false};
    rclcpp::Time last_seen{0, 0, RCL_ROS_TIME};
  };

  void on_cmd(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    if (!std::isfinite(msg->twist.linear.x) || !std::isfinite(msg->twist.angular.z)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Ignoring non-finite cmd speed");
      return;
    }
    last_cmd_speed_ms_ = msg->twist.linear.x;
    last_cmd_yaw_rate_rad_s_ = msg->twist.angular.z;
    last_cmd_stamp_ = msg->header.stamp;
    has_cmd_ = true;
  }

  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    if (!std::isfinite(msg->angular_velocity.z)) {
      return;
    }
    last_imu_yaw_rate_rad_s_ = msg->angular_velocity.z;
    has_imu_ = true;
  }

  void on_tacho(const mtt_msgs::msg::MttTachometerData::SharedPtr msg)
  {
    if (!std::isfinite(msg->speed_ms)) {
      return;
    }
    last_tacho_speed_ms_ = msg->speed_ms;
    last_tacho_stamp_ = msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0
      ? now()
      : rclcpp::Time(msg->header.stamp);
    last_tacho_is_synthetic_ = msg->tachometer_is_synthetic;
    has_tacho_ = true;
  }

  void on_vehicle_status(const mtt_msgs::msg::MttVehicleStatus::SharedPtr msg)
  {
    last_driver_command_speed_ms_ = finite_or_zero(msg->command_linear_speed_ms);
    last_driver_effective_command_speed_ms_ =
      finite_or_zero(msg->effective_linear_speed_command_ms);
    has_vehicle_status_ = true;
  }

  void on_com_direction(const std_msgs::msg::Float64::SharedPtr msg)
  {
    if (!std::isfinite(msg->data)) {
      return;
    }
    com_direction_sign_ = msg->data < 0.0 ? -1.0 : 1.0;
  }

  // Updates `state` from a new odometry message, purely within that source's
  // own consecutive-pose history (see OdomSourceState comment). Returns true
  // if a fresh body_speed_ms/lateral_speed_ms was computed this call (false
  // on the very first message from this source, or when dt fell outside
  // [min_odom_dt_s_, max_odom_dt_s_]).
  bool update_source(OdomSourceState & state, const nav_msgs::msg::Odometry & msg)
  {
    if (!finite_pose(msg.pose.pose)) {
      return false;
    }
    state.ever_seen = true;
    state.last_seen = now();

    const rclcpp::Time stamp(msg.header.stamp);
    if (!state.has_prev) {
      state.prev_stamp = stamp;
      state.prev_x = msg.pose.pose.position.x;
      state.prev_y = msg.pose.pose.position.y;
      state.prev_yaw = yaw_from_quaternion(msg.pose.pose.orientation);
      state.has_prev = true;
      return false;
    }

    const double dt_s = (stamp - state.prev_stamp).seconds();
    const bool dt_ok = dt_s >= min_odom_dt_s_ && dt_s <= max_odom_dt_s_;
    if (dt_ok) {
      const double dx = msg.pose.pose.position.x - state.prev_x;
      const double dy = msg.pose.pose.position.y - state.prev_y;
      const double c = std::cos(state.prev_yaw);
      const double s = std::sin(state.prev_yaw);
      state.body_speed_ms = (c * dx + s * dy) / dt_s;
      state.lateral_speed_ms = (-s * dx + c * dy) / dt_s;
      state.has_speed = true;
    }
    state.prev_stamp = stamp;
    state.prev_x = msg.pose.pose.position.x;
    state.prev_y = msg.pose.pose.position.y;
    state.prev_yaw = yaw_from_quaternion(msg.pose.pose.orientation);
    return dt_ok;
  }

  // Highest-priority source that has ever published AND was seen within its
  // own staleness window, as of right now. Wall-clock based (not message
  // stamp based) so a source that stops publishing entirely is detected
  // even though nothing about its last message ages on its own.
  Source active_source() const
  {
    const rclcpp::Time t = now();
    if (vslam_state_.ever_seen &&
        (t - vslam_state_.last_seen).seconds() <= vslam_staleness_s_)
    {
      return Source::kVslam;
    }
    if (icp_state_.ever_seen &&
        (t - icp_state_.last_seen).seconds() <= icp_staleness_s_)
    {
      return Source::kIcp;
    }
    if (imu_odom_state_.ever_seen &&
        (t - imu_odom_state_.last_seen).seconds() <= imu_odom_staleness_s_)
    {
      return Source::kImuOdom;
    }
    return Source::kNone;
  }

  const OdomSourceState & state_for(Source s) const
  {
    switch (s) {
      case Source::kVslam: return vslam_state_;
      case Source::kIcp: return icp_state_;
      default: return imu_odom_state_;
    }
  }

  double variance_for(Source s) const
  {
    switch (s) {
      case Source::kVslam: return vslam_speed_variance_;
      case Source::kIcp: return icp_speed_variance_;
      default: return imu_odom_speed_variance_;
    }
  }

  void on_vslam_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const bool updated = update_source(vslam_state_, *msg);
    publish_float(vslam_speed_pub_, vslam_state_.has_speed ? vslam_state_.body_speed_ms : 0.0);
    if (updated && active_source() == Source::kVslam) {
      process_active_tick(Source::kVslam, *msg);
    }
  }

  void on_icp_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const bool updated = update_source(icp_state_, *msg);
    publish_float(icp_raw_speed_pub_, icp_state_.has_speed ? icp_state_.body_speed_ms : 0.0);
    if (updated && active_source() == Source::kIcp) {
      process_active_tick(Source::kIcp, *msg);
    }
  }

  void on_imu_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const bool updated = update_source(imu_odom_state_, *msg);
    publish_float(imu_odom_speed_pub_, imu_odom_state_.has_speed ? imu_odom_state_.body_speed_ms : 0.0);
    if (updated && active_source() == Source::kImuOdom) {
      process_active_tick(Source::kImuOdom, *msg);
    }
  }

  // Runs the existing slip-detection pipeline (Kalman fusion with
  // tachometer, slip ratio, auto COM switch, publish) using whichever
  // source is CURRENTLY ACTIVE's own just-computed body speed. This is the
  // exact continuation of the old single-source on_odom(), just fed from
  // state_for(active)/variance_for(active) instead of a single hardcoded
  // ICP subscription.
  void process_active_tick(Source active, const nav_msgs::msg::Odometry & msg)
  {
    if (active != last_active_source_) {
      RCLCPP_WARN(
        get_logger(), "Ice slip detector: active pose source changed %s -> %s",
        source_name(last_active_source_), source_name(active));
      last_active_source_ = active;
    }
    std_msgs::msg::String source_msg;
    source_msg.data = source_name(active);
    active_pose_source_pub_->publish(source_msg);

    const OdomSourceState & src = state_for(active);
    const double reference_body_speed_ms = src.body_speed_ms;
    const double reference_lateral_speed_ms = src.lateral_speed_ms;
    const rclcpp::Time stamp(msg.header.stamp);

    predict_filter(reference_dt_s(active));
    const bool tacho_used = maybe_update_with_tacho(stamp);
    kalman_update(reference_body_speed_ms, variance_for(active));

    // Guard against a stale /cmd_vel: has_cmd_ latches true forever once a
    // message arrives, so without an age check a released stick / dropped
    // arbiter output leaves last_cmd_speed_ms_ pinned at its last nonzero
    // value and the node keeps reporting slip against motion that is no
    // longer commanded ("phantom slip").
    const bool cmd_valid =
      has_cmd_ &&
      std::abs((stamp - last_cmd_stamp_).seconds()) <= cmd_fresh_timeout_s_;

    const double fused_speed_ms = filter_initialized_ ? fused_speed_ms_ : reference_body_speed_ms;
    const double slip_ratio = compute_slip_ratio(fused_speed_ms, cmd_valid);
    const double reference_slip_ratio = compute_slip_ratio(reference_body_speed_ms, cmd_valid);
    const bool slipping =
      cmd_valid &&
      std::abs(last_cmd_speed_ms_) >= min_cmd_speed_ms_ &&
      std::abs(slip_ratio) > slip_threshold_;

    const double cmd_yaw_rate = cmd_valid ? last_cmd_yaw_rate_rad_s_ : 0.0;
    const double yaw_rate_mismatch = has_imu_ ? (cmd_yaw_rate - last_imu_yaw_rate_rad_s_) : 0.0;

    try_auto_com_switch(slipping);
    publish(
      slipping, slip_ratio, reference_slip_ratio, reference_body_speed_ms, fused_speed_ms,
      reference_lateral_speed_ms, yaw_rate_mismatch);

    if (slipping) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Slip detected [source=%s]: cmd=%.2f m/s fused=%.2f m/s ref=%.2f m/s lat=%.2f m/s "
        "tacho=%.2f m/s ratio=%.2f tacho_used=%s",
        source_name(active), last_cmd_speed_ms_, fused_speed_ms, reference_body_speed_ms,
        reference_lateral_speed_ms, last_tacho_speed_ms_, slip_ratio, tacho_used ? "true" : "false");
    }
  }

  // dt used only for the Kalman process-noise prediction step -- reuses the
  // active source's own last inter-message interval rather than a separate
  // node-wide timer, matching the original single-source design's timing.
  double reference_dt_s(Source active) const
  {
    (void)active;
    return std::clamp(min_odom_dt_s_, 0.0, max_odom_dt_s_);
  }

  void predict_filter(double dt_s)
  {
    if (!filter_initialized_) {
      return;
    }
    fused_speed_variance_ = std::max(
      fused_speed_variance_ + std::max(0.0, filter_process_variance_) * std::max(0.0, dt_s),
      1e-6);
  }

  bool maybe_update_with_tacho(const rclcpp::Time & stamp)
  {
    if (!has_tacho_) {
      return false;
    }
    if (last_tacho_is_synthetic_ && !use_synthetic_tachometer_) {
      return false;
    }
    const double age_s = std::abs((stamp - last_tacho_stamp_).seconds());
    if (age_s > tacho_fresh_timeout_s_) {
      return false;
    }
    kalman_update(last_tacho_speed_ms_, tacho_speed_variance_);
    return true;
  }

  void kalman_update(double measurement, double variance)
  {
    if (!std::isfinite(measurement)) {
      return;
    }
    const double safe_variance = std::max(variance, 1e-6);
    if (!filter_initialized_) {
      fused_speed_ms_ = measurement;
      fused_speed_variance_ = safe_variance;
      filter_initialized_ = true;
      return;
    }

    const double gain = fused_speed_variance_ / (fused_speed_variance_ + safe_variance);
    fused_speed_ms_ += gain * (measurement - fused_speed_ms_);
    fused_speed_variance_ = std::max((1.0 - gain) * fused_speed_variance_, 1e-6);
  }

  double compute_slip_ratio(double measured_speed_ms, bool cmd_valid) const
  {
    if (!cmd_valid || std::abs(last_cmd_speed_ms_) < min_cmd_speed_ms_) {
      return 0.0;
    }
    return (last_cmd_speed_ms_ - measured_speed_ms) /
      std::max(std::abs(last_cmd_speed_ms_), 1e-3);
  }

  void try_auto_com_switch(bool slipping)
  {
    if (!auto_com_switch_enabled_) {
      last_slipping_ = slipping;
      return;
    }
    if (!slipping || last_slipping_) {
      last_slipping_ = slipping;
      return;
    }

    const rclcpp::Time now_time = now();
    if (has_last_auto_switch_time_) {
      const double elapsed_s = (now_time - last_auto_switch_time_).seconds();
      if (elapsed_s < auto_com_switch_cooldown_s_) {
        last_slipping_ = slipping;
        return;
      }
    }

    com_direction_sign_ = com_direction_sign_ < 0.0 ? 1.0 : -1.0;
    auto sign_msg = std_msgs::msg::Float64();
    sign_msg.data = com_direction_sign_;
    com_direction_pub_->publish(sign_msg);
    last_auto_switch_time_ = now_time;
    has_last_auto_switch_time_ = true;
    RCLCPP_WARN(
      get_logger(),
      "Auto COM direction switch on slip: sign=%.0f", com_direction_sign_);
    last_slipping_ = slipping;
  }

  void publish(
    bool slipping,
    double slip_ratio,
    double reference_slip_ratio,
    double reference_body_speed_ms,
    double fused_speed_ms,
    double reference_lateral_speed_ms,
    double yaw_rate_mismatch)
  {
    std_msgs::msg::Bool slip_bool;
    slip_bool.data = slipping;
    slip_bool_pub_->publish(slip_bool);

    std_msgs::msg::Int8 slip_int;
    slip_int.data = slipping ? 1 : 0;
    slip_int_pub_->publish(slip_int);

    publish_float(slip_ratio_pub_, slip_ratio);
    publish_float(icp_slip_ratio_pub_, reference_slip_ratio);
    publish_float(icp_speed_pub_, reference_body_speed_ms);
    publish_float(fused_speed_pub_, fused_speed_ms);
    publish_float(tacho_speed_pub_, has_tacho_ ? last_tacho_speed_ms_ : 0.0);
    publish_float(cmd_speed_pub_, has_cmd_ ? last_cmd_speed_ms_ : 0.0);
    publish_float(
      driver_command_speed_pub_,
      has_vehicle_status_ ? last_driver_command_speed_ms_ : 0.0);
    publish_float(
      driver_effective_command_speed_pub_,
      has_vehicle_status_ ? last_driver_effective_command_speed_ms_ : 0.0);
    publish_float(
      cmd_driver_delta_pub_,
      has_vehicle_status_ && has_cmd_
        ? last_cmd_speed_ms_ - last_driver_effective_command_speed_ms_
        : 0.0);
    publish_float(lateral_speed_pub_, reference_lateral_speed_ms);
    publish_float(yaw_rate_mismatch_pub_, yaw_rate_mismatch);

    std_msgs::msg::Bool auto_enabled;
    auto_enabled.data = auto_com_switch_enabled_;
    auto_switch_enabled_pub_->publish(auto_enabled);
    publish_float(com_direction_state_pub_, com_direction_sign_);
  }

  void publish_float(
    const rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr & publisher,
    double value)
  {
    std_msgs::msg::Float64 msg;
    msg.data = value;
    publisher->publish(msg);
  }

  std::string cmd_topic_;
  std::string vslam_odom_topic_;
  std::string icp_odom_topic_;
  std::string imu_odom_topic_;
  std::string tacho_topic_;
  std::string vehicle_status_topic_;
  std::string com_direction_topic_;
  std::string imu_topic_;
  double slip_threshold_{0.20};
  double min_cmd_speed_ms_{0.05};
  double min_odom_dt_s_{0.02};
  double max_odom_dt_s_{0.50};
  double tacho_fresh_timeout_s_{0.25};
  double cmd_fresh_timeout_s_{0.50};
  bool use_synthetic_tachometer_{false};
  double vslam_staleness_s_{0.3};
  double icp_staleness_s_{0.3};
  double imu_odom_staleness_s_{0.5};
  double filter_process_variance_{0.20};
  double vslam_speed_variance_{0.10};
  double icp_speed_variance_{0.04};
  double imu_odom_speed_variance_{0.25};
  double tacho_speed_variance_{1.00};
  bool auto_com_switch_enabled_{false};
  double auto_com_switch_cooldown_s_{2.0};

  bool has_cmd_{false};
  double last_cmd_speed_ms_{0.0};
  double last_cmd_yaw_rate_rad_s_{0.0};
  rclcpp::Time last_cmd_stamp_{0, 0, RCL_ROS_TIME};

  bool has_imu_{false};
  double last_imu_yaw_rate_rad_s_{0.0};

  bool has_tacho_{false};
  double last_tacho_speed_ms_{0.0};
  bool last_tacho_is_synthetic_{false};
  rclcpp::Time last_tacho_stamp_{0, 0, RCL_ROS_TIME};

  bool has_vehicle_status_{false};
  double last_driver_command_speed_ms_{0.0};
  double last_driver_effective_command_speed_ms_{0.0};

  bool filter_initialized_{false};
  double fused_speed_ms_{0.0};
  double fused_speed_variance_{1.0};

  OdomSourceState vslam_state_;
  OdomSourceState icp_state_;
  OdomSourceState imu_odom_state_;
  Source last_active_source_{Source::kNone};

  bool last_slipping_{false};
  double com_direction_sign_{1.0};
  bool has_last_auto_switch_time_{false};
  rclcpp::Time last_auto_switch_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vslam_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr icp_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr imu_odom_sub_;
  rclcpp::Subscription<mtt_msgs::msg::MttTachometerData>::SharedPtr tacho_sub_;
  rclcpp::Subscription<mtt_msgs::msg::MttVehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr com_direction_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr slip_bool_pub_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr slip_int_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr slip_ratio_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr icp_slip_ratio_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr icp_speed_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr fused_speed_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr tacho_speed_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cmd_speed_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr driver_command_speed_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr driver_effective_command_speed_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cmd_driver_delta_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_rate_mismatch_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr lateral_speed_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr auto_switch_enabled_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr com_direction_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr com_direction_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr active_pose_source_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vslam_speed_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr icp_raw_speed_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr imu_odom_speed_pub_;
};

}  // namespace mtt_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mtt_control::MttIceSlipDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
