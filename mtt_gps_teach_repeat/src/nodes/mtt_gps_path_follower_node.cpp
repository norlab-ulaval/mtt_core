#include "mtt_gps_teach_repeat/nodes/mtt_gps_path_follower_node.hpp"

#include <algorithm>
#include <cmath>

namespace mtt_gps_teach_repeat
{

MttGpsPathFollowerNode::MttGpsPathFollowerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("mtt_gps_path_follower_node", options)
{
  const std::string matched_info_topic = declare_parameter(
    "matched_info_topic", std::string("/mtt/gps_path_server/matched_info"));
  const std::string articulation_output_topic = declare_parameter(
    "articulation_output_topic",
    std::string("/mtt_control/auto/gps/articulation_setpoint"));
  cmd_vel_output_topic_ = declare_parameter(
    "cmd_vel_output_topic", std::string("/mtt_control/auto/gps/cmd_vel"));
  const std::string autonomy_request_topic = declare_parameter(
    "autonomy_request_topic", std::string("/mtt_control/autonomy/request"));
  const std::string autonomy_release_topic = declare_parameter(
    "autonomy_release_topic", std::string("/mtt_control/autonomy/release"));
  const std::string autonomy_selected_topic = declare_parameter(
    "autonomy_selected_topic", std::string("/mtt_control/autonomy/selected"));

  matched_info_timeout_s_ = declare_parameter("matched_info_timeout_s", 0.5);
  deadman_timeout_s_ = declare_parameter("deadman_timeout_s", 0.5);
  obstacle_timeout_s_ = declare_parameter("obstacle_timeout_s", 0.5);
  goal_tolerance_m_ = declare_parameter("goal_tolerance_m", 0.40);
  executed_path_min_distance_m_ = declare_parameter("executed_path_min_distance_m", 0.05);
  const std::string odom_topic = declare_parameter(
    "odom_topic", std::string("localization/odom"));
  map_frame_id_ = declare_parameter("map_frame_id", std::string("map"));
  require_deadman_ = declare_parameter("require_deadman", true);
  require_obstacle_gate_ = declare_parameter("require_obstacle_gate", true);
  publish_rate_hz_ = declare_parameter("publish_rate_hz", 20.0);

  logic::ArticulatedGpsControllerParams controller_params;
  controller_params.k_y = declare_parameter("k_y", 0.20);
  controller_params.k_theta = declare_parameter("k_theta", 0.90);
  controller_params.kappa_max = declare_parameter("kappa_max", 0.35);
  controller_params.slowdown_alpha = declare_parameter("slowdown_alpha", 0.60);
  controller_params.min_speed_ms = declare_parameter("min_speed_ms", 0.35);
  controller_params.max_speed_ms = declare_parameter("max_speed_ms", 1.60);

  mtt_control::logic::ArticulatedCommandParams model_params;
  model_params.equivalent_length_m = declare_parameter("equivalent_length_m", 2.40);
  model_params.max_articulation_rad = declare_parameter("max_articulation_rad", 0.733);
  model_params.max_articulation_rate_rad_s = declare_parameter(
    "max_articulation_rate_rad_s", 0.50);
  model_params.use_slip_heuristic = declare_parameter("use_slip_compensation", false);
  model_params.yaw_slip_base = declare_parameter("yaw_slip_base", 0.10);
  model_params.yaw_slip_speed_gain = declare_parameter("yaw_slip_speed_gain", 0.05);
  model_params.yaw_slip_min_scale = declare_parameter("yaw_slip_min_scale", 0.55);

  controller_ = std::make_unique<logic::ArticulatedGpsController>(
    controller_params, mtt_control::logic::ArticulatedCommandModel(model_params));

  matched_info_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
    matched_info_topic, 10,
    std::bind(&MttGpsPathFollowerNode::on_matched_info, this, std::placeholders::_1));
  selected_mode_sub_ = create_subscription<std_msgs::msg::String>(
    "mtt_control/selected_mode", rclcpp::QoS(1).transient_local(),
    std::bind(&MttGpsPathFollowerNode::on_selected_mode, this, std::placeholders::_1));
  autonomy_selected_sub_ = create_subscription<std_msgs::msg::String>(
    autonomy_selected_topic, rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&MttGpsPathFollowerNode::on_autonomy_selected, this, std::placeholders::_1));
  deadman_sub_ = create_subscription<std_msgs::msg::Bool>(
    "mtt_control/teleop_deadman", 20,
    std::bind(&MttGpsPathFollowerNode::on_deadman, this, std::placeholders::_1));
  route_loaded_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/mtt/gps_path_server/route_loaded", rclcpp::QoS(1).transient_local(),
    std::bind(&MttGpsPathFollowerNode::on_route_loaded, this, std::placeholders::_1));
  obstacle_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/mtt_obstacle/stop_requested", 10,
    std::bind(&MttGpsPathFollowerNode::on_obstacle_stop, this, std::placeholders::_1));
  obstacle_slowdown_sub_ = create_subscription<std_msgs::msg::Float32>(
    "/mtt_obstacle/slowdown_scale", 10,
    std::bind(&MttGpsPathFollowerNode::on_obstacle_slowdown, this, std::placeholders::_1));
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, 20,
    std::bind(&MttGpsPathFollowerNode::on_odom, this, std::placeholders::_1));

  articulation_pub_ = create_publisher<std_msgs::msg::Float64>(articulation_output_topic, 10);
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(cmd_vel_output_topic_, 10);
  diagnostics_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
    "/mtt/gps_path_follower/diagnostics", rclcpp::QoS(1).transient_local());
  state_pub_ = create_publisher<std_msgs::msg::String>(
    "/mtt/gps_path_follower/state", rclcpp::QoS(1).transient_local());
  executed_path_pub_ = create_publisher<nav_msgs::msg::Path>(
    "/mtt/gps_path_follower/executed_path", rclcpp::QoS(1).transient_local());
  autonomy_request_pub_ = create_publisher<std_msgs::msg::String>(autonomy_request_topic, 10);
  autonomy_release_pub_ = create_publisher<std_msgs::msg::String>(autonomy_release_topic, 10);
  replay_srv_ = create_service<std_srvs::srv::Trigger>(
    "/mtt_gps/replay",
    std::bind(
      &MttGpsPathFollowerNode::handle_replay, this, std::placeholders::_1,
      std::placeholders::_2));
  stop_srv_ = create_service<std_srvs::srv::Trigger>(
    "/mtt_gps/stop",
    std::bind(
      &MttGpsPathFollowerNode::handle_stop, this, std::placeholders::_1,
      std::placeholders::_2));

  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_)),
    std::bind(&MttGpsPathFollowerNode::on_timer, this));
  publish_diagnostics(0.0, 0.0, 0.0);
  publish_state("IDLE: load a route, select AUTO, then press GPS REPLAY");
  executed_path_.header.frame_id = map_frame_id_;
  executed_path_pub_->publish(executed_path_);
}

void MttGpsPathFollowerNode::on_matched_info(
  const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (msg->data.size() < 9) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  lateral_deviation_m_ = msg->data[0];
  course_deviation_rad_ = msg->data[1];
  curvature_ff_ = msg->data[3];
  desired_speed_ms_ = msg->data[4];
  remaining_distance_m_ = std::max(0.0, msg->data[7]);
  final_section_ = msg->data[8] > 0.5;
  has_matched_info_ = true;
  last_matched_info_stamp_ = now();
}

void MttGpsPathFollowerNode::on_selected_mode(const std_msgs::msg::String::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  selected_mode_ = msg->data;
}

void MttGpsPathFollowerNode::on_autonomy_selected(
  const std_msgs::msg::String::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!replay_armed_ || msg->data == "gps" || msg->data == "GPS") {
    return;
  }
  publish_stop();
  publish_diagnostics(0.0, previous_articulation_rad_, 0.0);
  replay_armed_ = false;
  autonomy_claimed_ = false;
  publish_state("IDLE: yielded command ownership to " + msg->data);
}

void MttGpsPathFollowerNode::on_deadman(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  deadman_active_ = msg->data;
  last_deadman_stamp_ = now();
}

void MttGpsPathFollowerNode::on_route_loaded(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  route_loaded_ = msg->data;
  if (!route_loaded_ && replay_armed_) {
    publish_stop();
    publish_diagnostics(0.0, previous_articulation_rad_, 0.0);
    replay_armed_ = false;
    release_autonomy();
    publish_state("IDLE: active route was cleared");
  }
}

void MttGpsPathFollowerNode::on_obstacle_stop(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  obstacle_stop_ = msg->data;
  last_obstacle_stamp_ = now();
}

void MttGpsPathFollowerNode::on_obstacle_slowdown(const std_msgs::msg::Float32::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  obstacle_slowdown_ = std::clamp(static_cast<double>(msg->data), 0.0, 1.0);
  last_obstacle_stamp_ = now();
}

void MttGpsPathFollowerNode::on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!replay_armed_) {
    return;
  }

  const auto & position = msg->pose.pose.position;
  if (!executed_path_.poses.empty()) {
    const auto & previous = executed_path_.poses.back().pose.position;
    if (std::hypot(position.x - previous.x, position.y - previous.y) <
      executed_path_min_distance_m_)
    {
      return;
    }
  }

  geometry_msgs::msg::PoseStamped pose;
  pose.header = msg->header;
  pose.header.frame_id = msg->header.frame_id.empty() ? map_frame_id_ : msg->header.frame_id;
  pose.pose = msg->pose.pose;
  executed_path_.header = pose.header;
  executed_path_.poses.push_back(pose);
  executed_path_pub_->publish(executed_path_);
}

void MttGpsPathFollowerNode::handle_replay(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!route_loaded_) {
    response->success = false;
    response->message = "replay refused: no GPS route loaded";
    return;
  }
  request_autonomy();
  replay_armed_ = true;
  executed_path_.poses.clear();
  executed_path_.header.stamp = now();
  executed_path_.header.frame_id = map_frame_id_;
  executed_path_pub_->publish(executed_path_);
  response->success = true;
  response->message = "GPS replay armed; motion requires AUTO, live deadman, fresh match and obstacle gate";
  publish_state("ARMED: waiting for AUTO/deadman/path/obstacle gates");
}

void MttGpsPathFollowerNode::handle_stop(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (replay_armed_) {
    publish_stop();
  }
  publish_diagnostics(0.0, previous_articulation_rad_, 0.0);
  replay_armed_ = false;
  release_autonomy();
  response->success = true;
  response->message = "GPS replay stopped";
  publish_state("IDLE: stopped by operator");
}

void MttGpsPathFollowerNode::on_timer()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!replay_armed_) {
    return;
  }

  const rclcpp::Time stamp = now();
  const bool mode_ok = selected_mode_ == "AUTO" || selected_mode_ == "auto";
  const bool deadman_fresh = last_deadman_stamp_.nanoseconds() > 0 &&
    (stamp - last_deadman_stamp_).seconds() <= deadman_timeout_s_;
  const bool deadman_ok = !require_deadman_ || (deadman_active_ && deadman_fresh);
  const bool info_fresh = has_matched_info_ &&
    (stamp - last_matched_info_stamp_).seconds() <= matched_info_timeout_s_;
  const bool obstacle_fresh = last_obstacle_stamp_.nanoseconds() > 0 &&
    (stamp - last_obstacle_stamp_).seconds() <= obstacle_timeout_s_;
  const bool obstacle_ok = !require_obstacle_gate_ ||
    (obstacle_fresh && !obstacle_stop_);

  if (!mode_ok || !deadman_ok || !info_fresh || !obstacle_ok) {
    publish_stop();
    publish_diagnostics(0.0, previous_articulation_rad_, 0.0);
    if (!mode_ok) {
      publish_state("PAUSED: control mode is not AUTO");
    } else if (!deadman_ok) {
      publish_state("PAUSED: deadman missing or stale");
    } else if (!info_fresh) {
      publish_state("PAUSED: GPS path match missing or stale");
    } else {
      publish_state(obstacle_stop_ ? "PAUSED: obstacle stop" : "PAUSED: obstacle gate stale");
    }
    return;
  }

  if (final_section_ &&
    (remaining_distance_m_ <= goal_tolerance_m_ || std::abs(desired_speed_ms_) <= 1e-3))
  {
    publish_stop();
    publish_diagnostics(0.0, previous_articulation_rad_, 0.0);
    replay_armed_ = false;
    release_autonomy();
    publish_state("COMPLETED: end of GPS route reached");
    return;
  }

  logic::ArticulatedGpsControllerInput input;
  input.lateral_deviation_m = lateral_deviation_m_;
  input.course_deviation_rad = course_deviation_rad_;
  input.curvature_ff = curvature_ff_;
  input.desired_speed_ms = desired_speed_ms_;
  input.previous_articulation_rad = previous_articulation_rad_;
  input.dt_s = 1.0 / std::max(1.0, publish_rate_hz_);

  const logic::ArticulatedGpsControllerOutput output = controller_->compute(input);
  previous_articulation_rad_ = output.articulation_rad;

  std_msgs::msg::Float64 articulation_msg;
  articulation_msg.data = output.articulation_rad;
  articulation_pub_->publish(articulation_msg);

  geometry_msgs::msg::TwistStamped cmd_vel_msg;
  cmd_vel_msg.header.stamp = stamp;
  cmd_vel_msg.twist.linear.x = output.speed_ms * obstacle_slowdown_;
  cmd_vel_pub_->publish(cmd_vel_msg);

  publish_diagnostics(
    output.kappa_desired, output.articulation_rad, cmd_vel_msg.twist.linear.x);
  publish_state("REPLAYING");
}

void MttGpsPathFollowerNode::publish_stop()
{
  geometry_msgs::msg::TwistStamped cmd_vel_msg;
  cmd_vel_msg.header.stamp = now();
  cmd_vel_pub_->publish(cmd_vel_msg);
}

void MttGpsPathFollowerNode::publish_diagnostics(
  const double desired_curvature, const double articulation_rad, const double speed_ms)
{
  std_msgs::msg::Float64MultiArray msg;
  msg.data = {
    lateral_deviation_m_, course_deviation_rad_, desired_curvature,
    articulation_rad, speed_ms, remaining_distance_m_, obstacle_slowdown_,
  };
  diagnostics_pub_->publish(msg);
}

void MttGpsPathFollowerNode::publish_state(const std::string & state)
{
  if (state == last_published_state_) {
    return;
  }
  std_msgs::msg::String msg;
  msg.data = state;
  state_pub_->publish(msg);
  last_published_state_ = state;
}

void MttGpsPathFollowerNode::request_autonomy()
{
  std_msgs::msg::String msg;
  msg.data = "gps";
  autonomy_request_pub_->publish(msg);
  autonomy_claimed_ = true;
}

void MttGpsPathFollowerNode::release_autonomy()
{
  if (!autonomy_claimed_) {
    return;
  }
  std_msgs::msg::String msg;
  msg.data = "gps";
  autonomy_release_pub_->publish(msg);
  autonomy_claimed_ = false;
}

}  // namespace mtt_gps_teach_repeat

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mtt_gps_teach_repeat::MttGpsPathFollowerNode>());
  rclcpp::shutdown();
  return 0;
}
