#ifndef MTT_DRIVER__COMPONENTS__MTT_JOINT_STATE_BUILDER_NODE_HPP_
#define MTT_DRIVER__COMPONENTS__MTT_JOINT_STATE_BUILDER_NODE_HPP_

#include <array>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"

#include "mtt_msgs/msg/mtt_tachometer_data.hpp"

namespace mtt
{

class MttJointStateBuilderNode : public rclcpp::Node
{
public:
  explicit MttJointStateBuilderNode(const rclcpp::NodeOptions & options);

private:
  void on_articulation(const std_msgs::msg::Float64::SharedPtr msg);
  void on_tachometer(const mtt_msgs::msg::MttTachometerData::SharedPtr msg);
  void publish_joint_states();
  rclcpp::Time message_stamp_or_now(const std_msgs::msg::Header & header) const;
  double signed_speed_from_tachometer(const mtt_msgs::msg::MttTachometerData & msg) const;

  std::string joint_state_topic_;
  std::string articulation_topic_;
  std::string tachometer_topic_;
  double publish_rate_hz_{50.0};
  double pitch_rest_rad_{0.0};
  double yaw_rest_rad_{0.0};
  double roll_rest_rad_{0.0};
  double articulation_sign_{1.0};
  double max_articulation_rad_{0.0};
  double trailer_left_link_rest_rad_{0.0};
  double trailer_right_link_rest_rad_{0.0};
  double drive_joint_radius_m_{0.0202};
  double drive_joint_rotation_sign_{-1.0};
  double trailer_wheel_radius_m_{0.0508};
  double left_wheel_rotation_sign_{1.0};
  double right_wheel_rotation_sign_{1.0};

  double articulation_rad_{0.0};
  double cumulative_distance_m_{0.0};
  std::optional<rclcpp::Time> last_tacho_stamp_;

  std::vector<std::string> joint_names_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr articulation_sub_;
  rclcpp::Subscription<mtt_msgs::msg::MttTachometerData>::SharedPtr tachometer_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace mtt

#endif  // MTT_DRIVER__COMPONENTS__MTT_JOINT_STATE_BUILDER_NODE_HPP_
