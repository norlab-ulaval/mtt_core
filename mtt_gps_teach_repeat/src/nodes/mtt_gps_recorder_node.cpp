#include "mtt_gps_teach_repeat/nodes/mtt_gps_recorder_node.hpp"

#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <vector>

namespace mtt_gps_teach_repeat
{

MttGpsRecorderNode::MttGpsRecorderNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("mtt_gps_recorder_node", options)
{
  const std::string odom_topic = declare_parameter("odom_topic", std::string("localization/odom"));
  const std::string fix_topic = declare_parameter("fix_topic", std::string("/gps_front/fix"));
  configured_output_file_ = declare_parameter("output_file", std::string(""));
  routes_directory_ = declare_parameter(
    "routes_directory", std::string("data/gps_routes"));
  route_prefix_ = declare_parameter("route_prefix", std::string("gps_route"));
  map_frame_id_ = declare_parameter("map_frame_id", std::string("map"));
  autosave_on_shutdown_ = declare_parameter("autosave_on_shutdown", false);
  fix_timeout_s_ = declare_parameter("fix_timeout_s", 1.0);
  const double min_dist = declare_parameter("min_dist_between_points", 0.10);
  const double min_speed = declare_parameter("min_speed", 0.10);
  const double max_plausible_speed_ms = declare_parameter("max_plausible_speed_ms", 5.0);
  const int smoothing_window_size = declare_parameter("smoothing_window_size", 2);
  const std::string min_fix_quality_str = declare_parameter(
    "min_fix_quality", std::string("rtk_float"));
  min_fix_quality_ = logic::gps_fix_quality_from_string(min_fix_quality_str);

  const std::vector<double> anchor = declare_parameter(
    "anchor", std::vector<double>{46.778879, -71.277157, 0.0});
  if (anchor.size() == 3) {
    anchor_wgs84_ = {anchor[0], anchor[1], anchor[2]};
  } else {
    RCLCPP_WARN(get_logger(), "anchor parameter must have 3 elements [lat, lon, alt]; using [0,0,0]");
  }

  recorder_.configure(min_dist, min_speed, max_plausible_speed_ms, smoothing_window_size);

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, 20, std::bind(&MttGpsRecorderNode::on_odom, this, std::placeholders::_1));
  fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
    fix_topic, rclcpp::SensorDataQoS(),
    std::bind(&MttGpsRecorderNode::on_fix, this, std::placeholders::_1));
  state_pub_ = create_publisher<std_msgs::msg::String>(
    "/mtt/gps_recording/state", rclcpp::QoS(1).transient_local());
  fix_quality_pub_ = create_publisher<std_msgs::msg::String>(
    "/mtt/gps_recording/fix_quality", 10);
  horizontal_accuracy_pub_ = create_publisher<std_msgs::msg::Float64>(
    "/mtt/gps_recording/horizontal_accuracy_m", 10);
  diagnostics_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
    "/mtt/gps_recording/diagnostics", 10);
  teach_path_pub_ = create_publisher<nav_msgs::msg::Path>(
    "/mtt/gps_recording/teach_path", rclcpp::QoS(1).transient_local());
  saved_route_pub_ = create_publisher<std_msgs::msg::String>(
    "/mtt/gps_recording/saved_route", rclcpp::QoS(1).transient_local());
  control_pub_ = create_publisher<std_msgs::msg::String>(
    "/mtt/gps/control", rclcpp::QoS(1).transient_local());
  start_recording_srv_ = create_service<std_srvs::srv::Trigger>(
    "/mtt_gps/teach_start",
    std::bind(
      &MttGpsRecorderNode::handle_start_recording, this, std::placeholders::_1,
      std::placeholders::_2));
  stop_recording_srv_ = create_service<std_srvs::srv::Trigger>(
    "/mtt_gps/teach_stop",
    std::bind(
      &MttGpsRecorderNode::handle_stop_recording, this, std::placeholders::_1,
      std::placeholders::_2));
  clear_trajectory_srv_ = create_service<std_srvs::srv::Trigger>(
    "/mtt_gps/clear_trajectory",
    std::bind(
      &MttGpsRecorderNode::handle_clear_trajectory, this, std::placeholders::_1,
      std::placeholders::_2));

  // Keep the original private services as compatibility aliases for older
  // Foxglove layouts and operator scripts.
  legacy_start_recording_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/start_recording",
    std::bind(
      &MttGpsRecorderNode::handle_start_recording, this, std::placeholders::_1,
      std::placeholders::_2));
  legacy_stop_recording_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/stop_recording",
    std::bind(
      &MttGpsRecorderNode::handle_stop_recording, this, std::placeholders::_1,
      std::placeholders::_2));
  legacy_clear_trajectory_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/clear_trajectory",
    std::bind(
      &MttGpsRecorderNode::handle_clear_trajectory, this, std::placeholders::_1,
      std::placeholders::_2));

  publish_state("ready");
  publish_teach_path();
  RCLCPP_INFO(
    get_logger(),
    "GPS recorder ready: fix=%s routes=%s anchor=[%.6f, %.6f, %.1f]",
    fix_topic.c_str(), routes_directory_.c_str(), anchor_wgs84_[0], anchor_wgs84_[1],
    anchor_wgs84_[2]);
}

MttGpsRecorderNode::~MttGpsRecorderNode()
{
  if (autosave_on_shutdown_) {
    save_if_recorded();
  }
}

void MttGpsRecorderNode::on_fix(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_fix_valid_ = msg->status.status >= sensor_msgs::msg::NavSatStatus::STATUS_FIX &&
    msg->position_covariance_type != sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN &&
    std::isfinite(msg->latitude) && std::isfinite(msg->longitude) &&
    std::isfinite(msg->position_covariance[0]);
  latest_horizontal_accuracy_m_ = latest_fix_valid_ ?
    std::sqrt(std::max(msg->position_covariance[0], 0.0)) :
    std::numeric_limits<double>::infinity();
  latest_fix_quality_ = latest_fix_valid_ ?
    logic::classify_gps_fix_quality(latest_horizontal_accuracy_m_) :
    logic::GpsFixQuality::NoFix;
  latest_fix_received_stamp_ = now();

  std_msgs::msg::String quality_msg;
  quality_msg.data = logic::to_string(latest_fix_quality_);
  fix_quality_pub_->publish(quality_msg);

  std_msgs::msg::Float64 accuracy_msg;
  accuracy_msg.data = latest_horizontal_accuracy_m_;
  horizontal_accuracy_pub_->publish(accuracy_msg);
}

void MttGpsRecorderNode::on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!recording_) {
    return;
  }

  const bool fix_fresh = latest_fix_received_stamp_.nanoseconds() > 0 &&
    (now() - latest_fix_received_stamp_).seconds() <= fix_timeout_s_;
  if (!fix_fresh || logic::rank(latest_fix_quality_) < logic::rank(min_fix_quality_)) {
    ++rejected_low_quality_count_;
    return;
  }

  const double stamp_s = rclcpp::Time(msg->header.stamp).seconds();
  const auto outcome = recorder_.add_sample(
    msg->pose.pose.position.x, msg->pose.pose.position.y, msg->twist.twist.linear.x, stamp_s);
  if (outcome == logic::GpsSampleOutcome::Added) {
    publish_teach_path();
  }
  publish_diagnostics();
}

void MttGpsRecorderNode::handle_start_recording(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (recording_) {
    response->success = false;
    response->message = "recording already active";
    return;
  }
  const bool fix_fresh = latest_fix_received_stamp_.nanoseconds() > 0 &&
    (now() - latest_fix_received_stamp_).seconds() <= fix_timeout_s_;
  if (!latest_fix_valid_ || !fix_fresh ||
    logic::rank(latest_fix_quality_) < logic::rank(min_fix_quality_))
  {
    response->success = false;
    response->message = std::string("recording refused: GPS fix is stale or quality is ") +
      logic::to_string(latest_fix_quality_);
    publish_state("refused: GPS fix quality");
    return;
  }
  recorder_.reset();
  rejected_low_quality_count_ = 0;
  recording_ = true;
  saved_ = false;
  active_route_name_ = make_route_name();
  if (configured_output_file_.empty()) {
    output_file_ = (std::filesystem::path(routes_directory_) /
      (active_route_name_ + ".traj")).string();
  } else {
    output_file_ = configured_output_file_;
    active_route_name_ = std::filesystem::path(configured_output_file_).stem().string();
  }
  response->success = true;
  response->message = "recording started: " + active_route_name_;
  publish_teach_path();
  publish_state("recording:" + active_route_name_);
}

void MttGpsRecorderNode::handle_stop_recording(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!recording_) {
    response->success = false;
    response->message = "no GPS Teach recording is active";
    return;
  }
  recording_ = false;
  if (recorder_.empty()) {
    response->success = false;
    response->message = "no waypoints recorded";
    publish_state("stopped: empty");
    return;
  }

  recorder_.smooth();

  try {
    recorder_.save(output_file_, anchor_wgs84_);
    saved_ = true;
    response->success = true;
    response->message = "saved " + std::to_string(recorder_.point_count()) + " points to " +
      output_file_ + " (" + std::to_string(recorder_.rejected_jump_count()) + " jumps, " +
      std::to_string(rejected_low_quality_count_) + " low-quality samples rejected)";
    std_msgs::msg::String saved;
    saved.data = active_route_name_;
    saved_route_pub_->publish(saved);
    publish_teach_path();
    publish_state("saved:" + active_route_name_);
    // The next Teach always receives a fresh unique path.
    output_file_.clear();
  } catch (const std::exception & e) {
    response->success = false;
    response->message = std::string("save failed: ") + e.what();
    publish_state("save failed");
  }
}

void MttGpsRecorderNode::handle_clear_trajectory(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  std::lock_guard<std::mutex> lock(mutex_);
  recording_ = false;
  saved_ = false;
  recorder_.reset();
  output_file_.clear();
  active_route_name_.clear();
  rejected_low_quality_count_ = 0;
  publish_teach_path();
  std_msgs::msg::String control;
  control.data = "clear";
  control_pub_->publish(control);
  publish_state("cleared");
  response->success = true;
  response->message = "active GPS trajectory cleared; saved routes were preserved";
}

void MttGpsRecorderNode::publish_state(const std::string & detail)
{
  std_msgs::msg::String msg;
  msg.data = detail;
  state_pub_->publish(msg);
}

void MttGpsRecorderNode::publish_diagnostics()
{
  std_msgs::msg::Float64MultiArray msg;
  msg.data = {
    static_cast<double>(recorder_.point_count()),
    static_cast<double>(recorder_.section_count()),
    static_cast<double>(recorder_.rejected_jump_count()),
    static_cast<double>(rejected_low_quality_count_),
    latest_horizontal_accuracy_m_,
  };
  diagnostics_pub_->publish(msg);
}

void MttGpsRecorderNode::publish_teach_path()
{
  nav_msgs::msg::Path path;
  path.header.stamp = now();
  path.header.frame_id = map_frame_id_;
  for (const auto & section : recorder_.sections()) {
    for (std::size_t i = 0; i < section.size(); ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = section[i].x;
      pose.pose.position.y = section[i].y;
      double yaw = 0.0;
      if (section.size() > 1) {
        const std::size_t before = i == 0 ? 0 : i - 1;
        const std::size_t after = i + 1 < section.size() ? i + 1 : section.size() - 1;
        yaw = std::atan2(
          section[after].y - section[before].y,
          section[after].x - section[before].x);
      }
      pose.pose.orientation.z = std::sin(0.5 * yaw);
      pose.pose.orientation.w = std::cos(0.5 * yaw);
      path.poses.push_back(pose);
    }
  }
  teach_path_pub_->publish(path);
}

std::string MttGpsRecorderNode::make_route_name() const
{
  const auto now_system = std::chrono::system_clock::now();
  const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
    now_system.time_since_epoch()) % 1000;
  const std::time_t time = std::chrono::system_clock::to_time_t(now_system);
  std::tm local{};
  localtime_r(&time, &local);
  std::ostringstream name;
  name << route_prefix_ << '_' << std::put_time(&local, "%Y-%m-%d_%H%M%S") << '_'
       << std::setw(3) << std::setfill('0') << milliseconds.count();
  return name.str();
}

void MttGpsRecorderNode::save_if_recorded()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (saved_ || recorder_.empty()) {
    return;
  }
  recorder_.smooth();
  try {
    recorder_.save(output_file_, anchor_wgs84_);
    RCLCPP_INFO(get_logger(), "Saved trajectory on shutdown: %s", output_file_.c_str());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to save trajectory on shutdown: %s", e.what());
  }
}

}  // namespace mtt_gps_teach_repeat

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mtt_gps_teach_repeat::MttGpsRecorderNode>());
  rclcpp::shutdown();
  return 0;
}
