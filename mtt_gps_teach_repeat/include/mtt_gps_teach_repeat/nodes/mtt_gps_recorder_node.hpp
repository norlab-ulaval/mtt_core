#pragma once

#include <array>
#include <limits>
#include <mutex>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "mtt_gps_teach_repeat/logic/gps_fix_quality.hpp"
#include "mtt_gps_teach_repeat/logic/gps_waypoint_recorder.hpp"

namespace mtt_gps_teach_repeat
{

class MttGpsRecorderNode : public rclcpp::Node
{
public:
  explicit MttGpsRecorderNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~MttGpsRecorderNode() override;

private:
  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void on_fix(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void handle_start_recording(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void handle_stop_recording(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void handle_clear_trajectory(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void publish_state(const std::string & detail);
  void publish_diagnostics();
  void publish_teach_path();
  void save_if_recorded();
  std::string make_route_name() const;

  std::string output_file_;
  std::string configured_output_file_;
  std::string routes_directory_;
  std::string route_prefix_;
  std::string map_frame_id_{"map"};
  std::string active_route_name_;
  bool autosave_on_shutdown_{false};
  double fix_timeout_s_{1.0};
  std::array<double, 3> anchor_wgs84_{0.0, 0.0, 0.0};
  logic::GpsFixQuality min_fix_quality_{logic::GpsFixQuality::RtkFloat};

  std::mutex mutex_;
  logic::GpsWaypointRecorder recorder_;
  bool recording_{false};
  bool saved_{false};
  double latest_horizontal_accuracy_m_{std::numeric_limits<double>::infinity()};
  bool latest_fix_valid_{false};
  rclcpp::Time latest_fix_received_stamp_;
  logic::GpsFixQuality latest_fix_quality_{logic::GpsFixQuality::NoFix};
  std::size_t rejected_low_quality_count_{0};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fix_quality_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr horizontal_accuracy_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr diagnostics_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr teach_path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr saved_route_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr control_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_recording_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_recording_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_trajectory_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr legacy_start_recording_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr legacy_stop_recording_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr legacy_clear_trajectory_srv_;
};

}  // namespace mtt_gps_teach_repeat
