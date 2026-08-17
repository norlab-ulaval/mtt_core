#pragma once

#include <mutex>
#include <string>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

namespace mtt_control
{

class MttAutonomyMuxNode : public rclcpp::Node
{
public:
  explicit MttAutonomyMuxNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  struct Input
  {
    geometry_msgs::msg::TwistStamped command;
    double articulation_rad{0.0};
    rclcpp::Time command_received;
    rclcpp::Time articulation_received;
    bool has_command{false};
    bool has_articulation{false};
  };

  void on_request(const std_msgs::msg::String::SharedPtr msg);
  void on_release(const std_msgs::msg::String::SharedPtr msg);
  void on_command(const geometry_msgs::msg::TwistStamped::SharedPtr msg, Input & input);
  void on_articulation(const std_msgs::msg::Float64::SharedPtr msg, Input & input);
  void on_timer();
  void select_locked(const std::string & owner);
  void publish_zero();
  static std::string normalized_owner(const std::string & owner);

  std::mutex mutex_;
  Input wiln_;
  Input gps_;
  Input experiment_;
  std::string owner_{"idle"};
  double input_timeout_s_{0.5};
  double max_pair_skew_s_{0.15};
  double publish_rate_hz_{50.0};

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr wiln_command_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr wiln_articulation_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr gps_command_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr gps_articulation_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr experiment_command_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr experiment_articulation_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr request_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr release_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr articulation_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr selected_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mtt_control
