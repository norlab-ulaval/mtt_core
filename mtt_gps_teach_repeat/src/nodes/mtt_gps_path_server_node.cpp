#include "mtt_gps_teach_repeat/nodes/mtt_gps_path_server_node.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <vector>

#include <geometry_msgs/msg/quaternion.hpp>

#include <romea_core_common/geodesy/GeodeticCoordinates.hpp>
#include <romea_core_path/PathSection2D.hpp>

namespace mtt_gps_teach_repeat
{

namespace
{
constexpr double kDegToRad = M_PI / 180.0;

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
}  // namespace

MttGpsPathServerNode::MttGpsPathServerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("mtt_gps_path_server_node", options)
{
  const std::string odom_topic = declare_parameter("odom_topic", std::string("localization/odom"));
  trajectory_file_ = declare_parameter(
    "trajectory_file", std::string(""));
  routes_directory_ = declare_parameter(
    "routes_directory", std::string("data/gps_routes"));
  const std::vector<double> anchor = declare_parameter(
    "anchor", std::vector<double>{46.778879, -71.277157, 0.0});
  if (anchor.size() == 3) {
    anchor_wgs84_ = {anchor[0], anchor[1], anchor[2]};
  } else {
    RCLCPP_ERROR(get_logger(), "anchor parameter must have 3 elements [lat, lon, alt]");
  }
  max_research_radius_ = declare_parameter("max_research_radius", 10.0);
  interpolation_window_ = declare_parameter("interpolation_window", 0.5);
  horizon_length_m_ = declare_parameter("horizon_length", 15.0);
  map_frame_id_ = declare_parameter("map_frame_id", std::string("map"));
  const double diagnostic_rate_hz = declare_parameter("diagnostic_rate_hz", 10.0);

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, 20, std::bind(&MttGpsPathServerNode::on_odom, this, std::placeholders::_1));
  saved_route_sub_ = create_subscription<std_msgs::msg::String>(
    "/mtt/gps_recording/saved_route", rclcpp::QoS(1).transient_local(),
    std::bind(&MttGpsPathServerNode::on_saved_route, this, std::placeholders::_1));
  control_sub_ = create_subscription<std_msgs::msg::String>(
    "/mtt/gps/control", rclcpp::QoS(1).transient_local(),
    std::bind(&MttGpsPathServerNode::on_control, this, std::placeholders::_1));
  local_horizon_pub_ = create_publisher<nav_msgs::msg::Path>(
    "/mtt/gps_path_server/local_horizon", 10);
  full_path_pub_ = create_publisher<nav_msgs::msg::Path>(
    "/mtt/gps_path_server/full_path", rclcpp::QoS(1).transient_local());
  matched_info_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
    "/mtt/gps_path_server/matched_info", 10);
  diagnostics_pub_ = create_publisher<std_msgs::msg::String>(
    "/mtt/gps_path_server/diagnostics", rclcpp::QoS(1).transient_local());
  route_loaded_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/mtt/gps_path_server/route_loaded", rclcpp::QoS(1).transient_local());
  active_route_pub_ = create_publisher<std_msgs::msg::String>(
    "/mtt/gps_path_server/active_route", rclcpp::QoS(1).transient_local());
  reload_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/reload",
    std::bind(&MttGpsPathServerNode::handle_reload, this, std::placeholders::_1,
      std::placeholders::_2));
  list_srv_ = create_service<mtt_interfaces::srv::RouteList>(
    "/mtt_gps/list",
    std::bind(
      &MttGpsPathServerNode::handle_list, this, std::placeholders::_1,
      std::placeholders::_2));
  load_srv_ = create_service<mtt_interfaces::srv::RouteCommand>(
    "/mtt_gps/load",
    std::bind(
      &MttGpsPathServerNode::handle_load, this, std::placeholders::_1,
      std::placeholders::_2));
  status_srv_ = create_service<mtt_interfaces::srv::RouteStatus>(
    "/mtt_gps/status",
    std::bind(
      &MttGpsPathServerNode::handle_status, this, std::placeholders::_1,
      std::placeholders::_2));

  std_msgs::msg::Bool loaded;
  loaded.data = false;
  route_loaded_pub_->publish(loaded);
  if (!trajectory_file_.empty()) {
    load_path(trajectory_file_, std::filesystem::path(trajectory_file_).stem().string());
  }

  diagnostic_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / std::max(1.0, diagnostic_rate_hz)),
    std::bind(&MttGpsPathServerNode::on_diagnostic_timer, this));
}

bool MttGpsPathServerNode::load_path(
  const std::string & filepath, const std::string & route_name)
{
  path_matching_.reset();
  matched_ = false;
  trajectory_file_ = filepath;

  try {
    const romea::core::GeodeticCoordinates wgs84_anchor = romea::core::makeGeodeticCoordinates(
      anchor_wgs84_[0] * kDegToRad, anchor_wgs84_[1] * kDegToRad, anchor_wgs84_[2]);
    path_matching_ = std::make_unique<romea::core::PathMatching>(
      filepath, wgs84_anchor, max_research_radius_, interpolation_window_);
    active_route_ = route_name;
    last_detail_ = "loaded:" + route_name;
    RCLCPP_INFO(get_logger(), "Loaded GPS path: %s", filepath.c_str());
    publish_full_path();
    std_msgs::msg::Bool loaded;
    loaded.data = true;
    route_loaded_pub_->publish(loaded);
    std_msgs::msg::String active;
    active.data = active_route_;
    active_route_pub_->publish(active);
    return true;
  } catch (const std::exception & e) {
    active_route_.clear();
    last_detail_ = std::string("load failed: ") + e.what();
    std_msgs::msg::Bool loaded;
    loaded.data = false;
    route_loaded_pub_->publish(loaded);
    RCLCPP_ERROR(get_logger(), "Failed to load trajectory file %s: %s", filepath.c_str(), e.what());
    return false;
  }
}

void MttGpsPathServerNode::handle_reload(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (trajectory_file_.empty()) {
    response->success = false;
    response->message = "no active GPS route to reload";
    return;
  }
  response->success = load_path(trajectory_file_, active_route_);
  response->message = response->success ?
    "reloaded " + trajectory_file_ : "failed to reload " + trajectory_file_;
}

void MttGpsPathServerNode::handle_list(
  const std::shared_ptr<mtt_interfaces::srv::RouteList::Request>,
  std::shared_ptr<mtt_interfaces::srv::RouteList::Response> response)
{
  std::error_code ec;
  const std::filesystem::path directory(routes_directory_);
  if (std::filesystem::exists(directory, ec)) {
    for (const auto & entry : std::filesystem::directory_iterator(directory, ec)) {
      if (ec) {
        break;
      }
      if (entry.is_regular_file() && entry.path().extension() == ".traj") {
        response->route_names.push_back(entry.path().stem().string());
      }
    }
  }
  std::sort(response->route_names.begin(), response->route_names.end());
  std::lock_guard<std::mutex> lock(mutex_);
  response->active_route = active_route_;
}

void MttGpsPathServerNode::handle_load(
  const std::shared_ptr<mtt_interfaces::srv::RouteCommand::Request> request,
  std::shared_ptr<mtt_interfaces::srv::RouteCommand::Response> response)
{
  const std::string filepath = resolve_route_path(request->route_name);
  if (filepath.empty()) {
    response->success = false;
    response->message = "invalid GPS route name";
    return;
  }
  if (!std::filesystem::exists(filepath)) {
    response->success = false;
    response->message = "GPS route not found: " + filepath;
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  const std::string route_name = std::filesystem::path(filepath).stem().string();
  response->success = load_path(filepath, route_name);
  response->message = response->success ?
    "loaded " + route_name : last_detail_;
}

void MttGpsPathServerNode::handle_status(
  const std::shared_ptr<mtt_interfaces::srv::RouteStatus::Request>,
  std::shared_ptr<mtt_interfaces::srv::RouteStatus::Response> response)
{
  std::lock_guard<std::mutex> lock(mutex_);
  response->success = path_matching_ != nullptr;
  response->message = last_detail_;
  response->active_route = active_route_;
  response->route_loaded = path_matching_ != nullptr;
  response->route_valid = matched_;
  response->autonomy_allowed = path_matching_ != nullptr && matched_;
  response->refusal_reason = response->autonomy_allowed ? "" : last_detail_;
  response->route_distance_m = matched_ ? std::abs(lateral_error_m_) : -1.0;
  response->heading_error_rad = matched_ ? std::abs(heading_error_rad_) : -1.0;
  response->obstacle_clearance_min_m = -1.0;
}

void MttGpsPathServerNode::on_saved_route(const std_msgs::msg::String::SharedPtr msg)
{
  const std::string filepath = resolve_route_path(msg->data);
  if (filepath.empty() || !std::filesystem::exists(filepath)) {
    RCLCPP_ERROR(get_logger(), "Saved GPS route cannot be resolved: %s", msg->data.c_str());
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  const std::string route_name = std::filesystem::path(filepath).stem().string();
  if (load_path(filepath, route_name)) {
    RCLCPP_INFO(get_logger(), "Automatically armed newly taught GPS route: %s", route_name.c_str());
  }
}

void MttGpsPathServerNode::on_control(const std_msgs::msg::String::SharedPtr msg)
{
  if (msg->data != "clear") {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  clear_path("cleared");
}

void MttGpsPathServerNode::clear_path(const std::string & reason)
{
  path_matching_.reset();
  matched_ = false;
  trajectory_file_.clear();
  active_route_.clear();
  last_detail_ = reason;

  nav_msgs::msg::Path empty;
  empty.header.stamp = now();
  empty.header.frame_id = map_frame_id_;
  full_path_pub_->publish(empty);
  local_horizon_pub_->publish(empty);
  std_msgs::msg::Bool loaded;
  loaded.data = false;
  route_loaded_pub_->publish(loaded);
  std_msgs::msg::String active;
  active_route_pub_->publish(active);
}

std::string MttGpsPathServerNode::resolve_route_path(const std::string & route_name) const
{
  if (route_name.empty()) {
    return "";
  }
  const std::filesystem::path requested(route_name);
  if (requested.is_absolute() || requested.has_parent_path() ||
    route_name.find("..") != std::string::npos)
  {
    return "";
  }
  std::filesystem::path filename = requested.filename();
  if (filename.extension().empty()) {
    filename += ".traj";
  }
  if (filename.extension() != ".traj") {
    return "";
  }
  return (std::filesystem::path(routes_directory_) / filename).string();
}

void MttGpsPathServerNode::publish_full_path()
{
  // The full, ROMEA-interpolated curve through every recorded/loaded
  // waypoint (not just the section ahead of the robot) — lets an operator
  // review the taught route as a smooth path before driving it.
  if (!path_matching_) {
    return;
  }

  nav_msgs::msg::Path path_msg;
  path_msg.header.stamp = now();
  path_msg.header.frame_id = map_frame_id_;

  const std::size_t section_count = path_matching_->getPath().getSections().size();
  for (std::size_t section_index = 0; section_index < section_count; ++section_index) {
    const romea::core::PathSection2D & section = path_matching_->getSection(section_index);
    const auto & x = section.getX();
    const auto & y = section.getY();
    for (std::size_t i = 0; i < x.size(); ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path_msg.header;
      pose.pose.position.x = x[i];
      pose.pose.position.y = y[i];
      const std::size_t before = i == 0 ? 0 : i - 1;
      const std::size_t after = i + 1 < x.size() ? i + 1 : x.size() - 1;
      const double yaw = std::atan2(y[after] - y[before], x[after] - x[before]);
      pose.pose.orientation.z = std::sin(0.5 * yaw);
      pose.pose.orientation.w = std::cos(0.5 * yaw);
      path_msg.poses.push_back(pose);
    }
  }

  full_path_pub_->publish(path_msg);
}

void MttGpsPathServerNode::on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!path_matching_) {
    return;
  }

  romea::core::Pose2D vehicle_pose;
  vehicle_pose.position.x() = msg->pose.pose.position.x;
  vehicle_pose.position.y() = msg->pose.pose.position.y;
  vehicle_pose.yaw = yaw_from_quaternion(msg->pose.pose.orientation);

  romea::core::Twist2D vehicle_twist;
  vehicle_twist.linearSpeeds.x() = msg->twist.twist.linear.x;
  vehicle_twist.linearSpeeds.y() = msg->twist.twist.linear.y;
  vehicle_twist.angularSpeed = msg->twist.twist.angular.z;

  const romea::core::Duration stamp(rclcpp::Time(msg->header.stamp).nanoseconds());

  const std::vector<romea::core::PathMatchedPoint2D> matched_points =
    path_matching_->match(stamp, vehicle_pose, vehicle_twist);

  if (matched_points.empty()) {
    matched_ = false;
    return;
  }

  matched_ = true;
  const romea::core::PathMatchedPoint2D & matched_point = matched_points.front();
  lateral_error_m_ = matched_point.frenetPose.lateralDeviation;
  heading_error_rad_ = matched_point.frenetPose.courseDeviation;
  last_detail_ = "matched:" + active_route_;
  publish_local_horizon(matched_point);
  publish_matched_info(matched_point);
}

void MttGpsPathServerNode::publish_local_horizon(
  const romea::core::PathMatchedPoint2D & matched_point)
{
  const romea::core::PathSection2D & section =
    path_matching_->getSection(matched_point.sectionIndex);
  const romea::core::PathSection2D::CurvilinearAbscissa & abscissa =
    section.getCurvilinearAbscissa();
  const auto & x = section.getX();
  const auto & y = section.getY();

  const double start_abscissa = matched_point.frenetPose.curvilinearAbscissa;
  const size_t start_index = section.findIndex(start_abscissa);

  nav_msgs::msg::Path path_msg;
  path_msg.header.stamp = now();
  path_msg.header.frame_id = map_frame_id_;

  for (size_t i = start_index; i < abscissa.size(); ++i) {
    if (abscissa[i] - start_abscissa > horizon_length_m_) {
      break;
    }
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path_msg.header;
    pose.pose.position.x = x[i];
    pose.pose.position.y = y[i];
    const std::size_t before = i == 0 ? 0 : i - 1;
    const std::size_t after = i + 1 < x.size() ? i + 1 : x.size() - 1;
    const double yaw = std::atan2(y[after] - y[before], x[after] - x[before]);
    pose.pose.orientation.z = std::sin(0.5 * yaw);
    pose.pose.orientation.w = std::cos(0.5 * yaw);
    path_msg.poses.push_back(pose);
  }

  local_horizon_pub_->publish(path_msg);
}

void MttGpsPathServerNode::publish_matched_info(
  const romea::core::PathMatchedPoint2D & matched_point)
{
  const romea::core::PathSection2D & section =
    path_matching_->getSection(matched_point.sectionIndex);
  const auto & abscissa = section.getCurvilinearAbscissa();
  double remaining_distance = abscissa.size() == 0 ? 0.0 : std::max(
    0.0, abscissa[abscissa.size() - 1] - matched_point.frenetPose.curvilinearAbscissa);
  const std::size_t section_count = path_matching_->getPath().getSections().size();
  for (std::size_t i = matched_point.sectionIndex + 1; i < section_count; ++i) {
    const auto & future_abscissa = path_matching_->getSection(i).getCurvilinearAbscissa();
    if (future_abscissa.size() > 0) {
      remaining_distance += future_abscissa[future_abscissa.size() - 1] - future_abscissa[0];
    }
  }
  const bool final_section = matched_point.sectionIndex + 1 >= section_count;
  std_msgs::msg::Float64MultiArray msg;
  msg.data = {
    matched_point.frenetPose.lateralDeviation,
    matched_point.frenetPose.courseDeviation,
    matched_point.pathPosture.curvature,
    matched_point.futureCurvature,
    matched_point.desiredSpeed,
    matched_point.frenetPose.curvilinearAbscissa,
    static_cast<double>(matched_point.sectionIndex),
    remaining_distance,
    final_section ? 1.0 : 0.0,
  };
  matched_info_pub_->publish(msg);
}

void MttGpsPathServerNode::on_diagnostic_timer()
{
  std::lock_guard<std::mutex> lock(mutex_);
  std_msgs::msg::String msg;
  if (!path_matching_) {
    msg.data = "no path loaded";
  } else if (matched_) {
    msg.data = "matched";
  } else {
    msg.data = "lost";
  }
  diagnostics_pub_->publish(msg);
}

}  // namespace mtt_gps_teach_repeat

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mtt_gps_teach_repeat::MttGpsPathServerNode>());
  rclcpp::shutdown();
  return 0;
}
