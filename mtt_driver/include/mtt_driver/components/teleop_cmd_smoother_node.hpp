#ifndef MTT_DRIVER__COMPONENTS__TELEOP_CMD_SMOOTHER_NODE_HPP_
#define MTT_DRIVER__COMPONENTS__TELEOP_CMD_SMOOTHER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include <string>

namespace mtt
{

class TeleopCmdSmootherNode : public rclcpp::Node
{
public:
  explicit TeleopCmdSmootherNode(const rclcpp::NodeOptions & options);

private:
  void input_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void publish_smoothed_cmd();
  
  double step_towards(double current, double target, double max_delta) const;

  std::string input_topic_;
  std::string output_topic_;
  double input_timeout_;
  double rate_hz_;
  double max_accel_linear_;
  double max_accel_angular_;
  double zero_epsilon_{1e-3};

  struct VelocityState {
    double linear_x = 0.0;
    double angular_z = 0.0;
  };

  VelocityState target_;
  VelocityState current_;

  rclcpp::Time last_input_;
  rclcpp::Time last_update_;

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mtt

#endif  // MTT_DRIVER__COMPONENTS__TELEOP_CMD_SMOOTHER_NODE_HPP_
