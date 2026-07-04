#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mtt_msgs/msg/mtt_tachometer_data.hpp>
#include <mtt_msgs/msg/mtt_vehicle_status.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int8.hpp>

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
  explicit MttIceSlipDetectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("mtt_ice_slip_detector_node", options)
  {
    cmd_topic_ = declare_parameter("cmd_topic", std::string("cmd_vel"));
    odom_topic_ = declare_parameter("odom_topic", std::string("/mapping/icp_odom"));
    tacho_topic_ = declare_parameter("tacho_topic", std::string("/mtt_tachometer"));
    vehicle_status_topic_ = declare_parameter("vehicle_status_topic", std::string("/mtt_status"));
    com_direction_topic_ =
      declare_parameter("com_direction_topic", std::string("mtt_control/com_direction_sign"));

    slip_threshold_ = declare_parameter("slip_threshold", 0.20);
    min_cmd_speed_ms_ = declare_parameter("min_cmd_speed_ms", 0.05);
    min_odom_dt_s_ = declare_parameter("min_odom_dt_s", 0.02);
    max_odom_dt_s_ = declare_parameter("max_odom_dt_s", 0.50);
    tacho_fresh_timeout_s_ = declare_parameter("tacho_fresh_timeout_s", 0.25);
    use_synthetic_tachometer_ = declare_parameter("use_synthetic_tachometer", false);

    filter_process_variance_ = declare_parameter("filter_process_variance", 0.20);
    icp_speed_variance_ = declare_parameter("icp_speed_variance", 0.04);
    tacho_speed_variance_ = declare_parameter("tacho_speed_variance", 1.00);

    auto_com_switch_enabled_ = declare_parameter("auto_com_switch_enabled", false);
    auto_com_switch_cooldown_s_ = declare_parameter("auto_com_switch_cooldown_s", 2.0);

    cmd_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      cmd_topic_, 20,
      [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) { on_cmd(msg); });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 20,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) { on_odom(msg); });
    tacho_sub_ = create_subscription<mtt_msgs::msg::MttTachometerData>(
      tacho_topic_, 20,
      [this](const mtt_msgs::msg::MttTachometerData::SharedPtr msg) { on_tacho(msg); });
    vehicle_status_sub_ = create_subscription<mtt_msgs::msg::MttVehicleStatus>(
      vehicle_status_topic_, 20,
      [this](const mtt_msgs::msg::MttVehicleStatus::SharedPtr msg) { on_vehicle_status(msg); });
    com_direction_sub_ = create_subscription<std_msgs::msg::Float64>(
      com_direction_topic_, 10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) { on_com_direction(msg); });

    slip_bool_pub_ = create_publisher<std_msgs::msg::Bool>("/ice_slip/slip_detected", 10);
    slip_int_pub_ = create_publisher<std_msgs::msg::Int8>("/slip_detected", 10);
    slip_ratio_pub_ = create_publisher<std_msgs::msg::Float64>("/ice_slip/slip_ratio", 10);
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
    auto_switch_enabled_pub_ =
      create_publisher<std_msgs::msg::Bool>("/ice_slip/auto_com_switch_enabled", 10);
    com_direction_state_pub_ =
      create_publisher<std_msgs::msg::Float64>("/ice_slip/com_direction_sign", 10);
    com_direction_pub_ = create_publisher<std_msgs::msg::Float64>(com_direction_topic_, 10);

    RCLCPP_INFO(
      get_logger(),
      "Ice slip detector ready: cmd=%s odom=%s tacho=%s status=%s threshold=%.2f "
      "auto_com_switch=%s",
      cmd_topic_.c_str(), odom_topic_.c_str(), tacho_topic_.c_str(),
      vehicle_status_topic_.c_str(), slip_threshold_,
      auto_com_switch_enabled_ ? "true" : "false");
  }

private:
  void on_cmd(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    if (!std::isfinite(msg->twist.linear.x)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Ignoring non-finite cmd speed");
      return;
    }
    last_cmd_speed_ms_ = msg->twist.linear.x;
    last_cmd_stamp_ = msg->header.stamp;
    has_cmd_ = true;
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

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!finite_pose(msg->pose.pose)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Ignoring non-finite ICP odom pose");
      return;
    }

    const rclcpp::Time stamp(msg->header.stamp);
    if (!has_prev_odom_) {
      update_prev(*msg, stamp);
      return;
    }

    const double dt_s = (stamp - prev_stamp_).seconds();
    if (dt_s < min_odom_dt_s_ || dt_s > max_odom_dt_s_) {
      update_prev(*msg, stamp);
      publish(false, 0.0, 0.0, 0.0, 0.0);
      return;
    }

    const double dx = msg->pose.pose.position.x - prev_x_;
    const double dy = msg->pose.pose.position.y - prev_y_;
    const double c = std::cos(prev_yaw_);
    const double s = std::sin(prev_yaw_);
    const double body_dx = c * dx + s * dy;
    const double icp_body_speed_ms = body_dx / dt_s;

    predict_filter(dt_s);
    const bool tacho_used = maybe_update_with_tacho(stamp);
    kalman_update(icp_body_speed_ms, icp_speed_variance_);

    const double fused_speed_ms = filter_initialized_ ? fused_speed_ms_ : icp_body_speed_ms;
    const double slip_ratio = compute_slip_ratio(fused_speed_ms);
    const double icp_slip_ratio = compute_slip_ratio(icp_body_speed_ms);
    const bool slipping =
      has_cmd_ &&
      std::abs(last_cmd_speed_ms_) >= min_cmd_speed_ms_ &&
      std::abs(slip_ratio) > slip_threshold_;

    try_auto_com_switch(slipping);
    publish(slipping, slip_ratio, icp_slip_ratio, icp_body_speed_ms, fused_speed_ms);

    if (slipping) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Slip detected: cmd=%.2f m/s fused=%.2f m/s icp=%.2f m/s tacho=%.2f m/s "
        "ratio=%.2f tacho_used=%s",
        last_cmd_speed_ms_, fused_speed_ms, icp_body_speed_ms, last_tacho_speed_ms_,
        slip_ratio, tacho_used ? "true" : "false");
    }

    update_prev(*msg, stamp);
  }

  void update_prev(const nav_msgs::msg::Odometry & msg, const rclcpp::Time & stamp)
  {
    prev_stamp_ = stamp;
    prev_x_ = msg.pose.pose.position.x;
    prev_y_ = msg.pose.pose.position.y;
    prev_yaw_ = yaw_from_quaternion(msg.pose.pose.orientation);
    has_prev_odom_ = true;
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

  double compute_slip_ratio(double measured_speed_ms) const
  {
    if (!has_cmd_ || std::abs(last_cmd_speed_ms_) < min_cmd_speed_ms_) {
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
    double icp_slip_ratio,
    double icp_body_speed_ms,
    double fused_speed_ms)
  {
    std_msgs::msg::Bool slip_bool;
    slip_bool.data = slipping;
    slip_bool_pub_->publish(slip_bool);

    std_msgs::msg::Int8 slip_int;
    slip_int.data = slipping ? 1 : 0;
    slip_int_pub_->publish(slip_int);

    publish_float(slip_ratio_pub_, slip_ratio);
    publish_float(icp_slip_ratio_pub_, icp_slip_ratio);
    publish_float(icp_speed_pub_, icp_body_speed_ms);
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
  std::string odom_topic_;
  std::string tacho_topic_;
  std::string vehicle_status_topic_;
  std::string com_direction_topic_;
  double slip_threshold_{0.20};
  double min_cmd_speed_ms_{0.05};
  double min_odom_dt_s_{0.02};
  double max_odom_dt_s_{0.50};
  double tacho_fresh_timeout_s_{0.25};
  bool use_synthetic_tachometer_{false};
  double filter_process_variance_{0.20};
  double icp_speed_variance_{0.04};
  double tacho_speed_variance_{1.00};
  bool auto_com_switch_enabled_{false};
  double auto_com_switch_cooldown_s_{2.0};

  bool has_cmd_{false};
  double last_cmd_speed_ms_{0.0};
  rclcpp::Time last_cmd_stamp_{0, 0, RCL_ROS_TIME};

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

  bool has_prev_odom_{false};
  rclcpp::Time prev_stamp_{0, 0, RCL_ROS_TIME};
  double prev_x_{0.0};
  double prev_y_{0.0};
  double prev_yaw_{0.0};

  bool last_slipping_{false};
  double com_direction_sign_{1.0};
  bool has_last_auto_switch_time_{false};
  rclcpp::Time last_auto_switch_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<mtt_msgs::msg::MttTachometerData>::SharedPtr tacho_sub_;
  rclcpp::Subscription<mtt_msgs::msg::MttVehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr com_direction_sub_;
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
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr auto_switch_enabled_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr com_direction_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr com_direction_pub_;
};

}  // namespace mtt_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mtt_control::MttIceSlipDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
