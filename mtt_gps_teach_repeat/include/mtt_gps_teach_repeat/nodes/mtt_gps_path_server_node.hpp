#pragma once

#include <array>
#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <romea_core_path_matching/PathMatching.hpp>

#include "mtt_interfaces/srv/route_command.hpp"
#include "mtt_interfaces/srv/route_list.hpp"
#include "mtt_interfaces/srv/route_status.hpp"

namespace mtt_gps_teach_repeat
{

/** Loads a ROMEA `.traj` file and matches the vehicle pose onto it every time
 * an odometry message arrives, publishing the Frenet errors and a local path
 * horizon for a downstream follower.
 *
 * Extension point for future GPS obstacle avoidance (mirroring WILN's
 * wiln_obstacle_node + PathDeformer): a mtt_gps_path_deformer_node could
 * subscribe to local_horizon plus an obstacle cloud, and republish a
 * corrected Frenet point on its own topic. mtt_gps_path_follower_node reads
 * its Frenet input from the configurable `matched_info_topic` parameter, so
 * pointing that parameter at the deformer's output is enough to insert
 * avoidance later — no change needed in either existing node.
 */
class MttGpsPathServerNode : public rclcpp::Node
{
public:
  explicit MttGpsPathServerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void on_diagnostic_timer();
  void handle_reload(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void handle_list(
    const std::shared_ptr<mtt_interfaces::srv::RouteList::Request> request,
    std::shared_ptr<mtt_interfaces::srv::RouteList::Response> response);
  void handle_load(
    const std::shared_ptr<mtt_interfaces::srv::RouteCommand::Request> request,
    std::shared_ptr<mtt_interfaces::srv::RouteCommand::Response> response);
  void handle_status(
    const std::shared_ptr<mtt_interfaces::srv::RouteStatus::Request> request,
    std::shared_ptr<mtt_interfaces::srv::RouteStatus::Response> response);
  void on_saved_route(const std_msgs::msg::String::SharedPtr msg);
  void on_control(const std_msgs::msg::String::SharedPtr msg);
  bool load_path(const std::string & filepath, const std::string & route_name);
  void clear_path(const std::string & reason);
  std::string resolve_route_path(const std::string & route_name) const;
  void publish_full_path();
  void publish_local_horizon(const romea::core::PathMatchedPoint2D & matched_point);
  void publish_matched_info(const romea::core::PathMatchedPoint2D & matched_point);

  std::string trajectory_file_;
  std::string routes_directory_;
  std::string active_route_;
  std::string last_detail_{"no path loaded"};
  std::array<double, 3> anchor_wgs84_{0.0, 0.0, 0.0};
  double max_research_radius_{10.0};
  double interpolation_window_{0.5};
  double horizon_length_m_{15.0};
  std::string map_frame_id_{"map"};

  std::mutex mutex_;
  std::unique_ptr<romea::core::PathMatching> path_matching_;
  bool matched_{false};
  double lateral_error_m_{0.0};
  double heading_error_rad_{0.0};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr saved_route_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_horizon_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr full_path_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr matched_info_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostics_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr route_loaded_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr active_route_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reload_srv_;
  rclcpp::Service<mtt_interfaces::srv::RouteList>::SharedPtr list_srv_;
  rclcpp::Service<mtt_interfaces::srv::RouteCommand>::SharedPtr load_srv_;
  rclcpp::Service<mtt_interfaces::srv::RouteStatus>::SharedPtr status_srv_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
};

}  // namespace mtt_gps_teach_repeat
