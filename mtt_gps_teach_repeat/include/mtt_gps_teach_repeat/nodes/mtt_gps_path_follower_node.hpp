#pragma once

#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "mtt_gps_teach_repeat/logic/articulated_gps_controller.hpp"

namespace mtt_gps_teach_repeat
{

/** Consumes mtt_gps_path_server_node's matched Frenet errors and drives its
 * private autonomy-mux inputs (articulation setpoint + velocity), gated on
 * AUTO mode and the deadman switch — same safety contract as WILN.
 *
 * No obstacle avoidance yet (LiDAR WILN has wiln_obstacle_node + PathDeformer;
 * GPS does not). The door is left open deliberately: this node only ever
 * reads its Frenet input from `matched_info_topic`, so a future
 * mtt_gps_path_deformer_node can sit between the path server and this node
 * (consuming local_horizon + an obstacle cloud, republishing a corrected
 * Frenet point) by pointing that one parameter at its output — no change
 * needed here.
 */
class MttGpsPathFollowerNode : public rclcpp::Node
{
public:
  explicit MttGpsPathFollowerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_matched_info(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void on_selected_mode(const std_msgs::msg::String::SharedPtr msg);
  void on_autonomy_selected(const std_msgs::msg::String::SharedPtr msg);
  void on_deadman(const std_msgs::msg::Bool::SharedPtr msg);
  void on_route_loaded(const std_msgs::msg::Bool::SharedPtr msg);
  void on_obstacle_stop(const std_msgs::msg::Bool::SharedPtr msg);
  void on_obstacle_slowdown(const std_msgs::msg::Float32::SharedPtr msg);
  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void handle_replay(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void handle_stop(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void on_timer();
  void publish_stop();
  void publish_diagnostics(double desired_curvature, double articulation_rad, double speed_ms);
  void publish_state(const std::string & state);
  void request_autonomy();
  void release_autonomy();

  bool require_deadman_{true};
  bool require_obstacle_gate_{true};
  double matched_info_timeout_s_{0.5};
  double deadman_timeout_s_{0.5};
  double obstacle_timeout_s_{0.5};
  double goal_tolerance_m_{0.40};
  double executed_path_min_distance_m_{0.05};
  double publish_rate_hz_{20.0};

  std::mutex mutex_;
  std::unique_ptr<logic::ArticulatedGpsController> controller_;
  double previous_articulation_rad_{0.0};
  bool has_matched_info_{false};
  rclcpp::Time last_matched_info_stamp_;
  rclcpp::Time last_deadman_stamp_;
  rclcpp::Time last_obstacle_stamp_;
  double lateral_deviation_m_{0.0};
  double course_deviation_rad_{0.0};
  double curvature_ff_{0.0};
  double desired_speed_ms_{0.0};
  double remaining_distance_m_{0.0};
  bool final_section_{false};

  std::string selected_mode_{"STOP"};
  bool deadman_active_{false};
  bool route_loaded_{false};
  bool replay_armed_{false};
  bool autonomy_claimed_{false};
  bool obstacle_stop_{true};
  double obstacle_slowdown_{0.0};
  std::string cmd_vel_output_topic_;
  std::string map_frame_id_{"map"};
  std::string last_published_state_;
  nav_msgs::msg::Path executed_path_;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr matched_info_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr selected_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr autonomy_selected_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr deadman_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr route_loaded_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr obstacle_stop_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr obstacle_slowdown_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr articulation_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr diagnostics_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr executed_path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr autonomy_request_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr autonomy_release_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr replay_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mtt_gps_teach_repeat
