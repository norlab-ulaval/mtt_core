#include "mtt_driver/components/mtt_joint_state_builder_node.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <utility>

#include "rclcpp_components/register_node_macro.hpp"

#include "mtt_driver/logic/vehicle_params.hpp"

namespace mtt
{

MttJointStateBuilderNode::MttJointStateBuilderNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("mtt_joint_state_builder_node", options)
{
  joint_state_topic_ = declare_parameter("joint_state_topic", std::string("joint_states"));
  articulation_topic_ = declare_parameter("articulation_topic", std::string("mtt_articulation_angle"));
  tachometer_topic_ = declare_parameter("tachometer_topic", std::string("mtt_tachometer"));
  publish_rate_hz_ = declare_parameter("publish_rate_hz", 50.0);
  pitch_rest_rad_ = declare_parameter("pitch_rest_rad", -M_PI_2);
  yaw_rest_rad_ = declare_parameter("yaw_rest_rad", M_PI_2);
  roll_rest_rad_ = declare_parameter("roll_rest_rad", -M_PI_2);
  articulation_sign_ = declare_parameter("articulation_sign", 1.0);
  max_articulation_rad_ = declare_parameter(
    "max_articulation_deg",
    VehicleParams::max_articulation_deg) * M_PI / 180.0;
  trailer_left_link_rest_rad_ = declare_parameter("trailer_left_link_rest_rad", 0.0);
  trailer_right_link_rest_rad_ = declare_parameter("trailer_right_link_rest_rad", 0.0);
  trailer_wheel_radius_m_ = declare_parameter("trailer_wheel_radius_m", 0.0508);
  left_wheel_rotation_sign_ = declare_parameter("left_wheel_rotation_sign", 1.0);
  right_wheel_rotation_sign_ = declare_parameter("right_wheel_rotation_sign", 1.0);

  joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(joint_state_topic_, 10);
  articulation_sub_ = create_subscription<std_msgs::msg::Float64>(
    articulation_topic_, 10,
    std::bind(&MttJointStateBuilderNode::on_articulation, this, std::placeholders::_1));
  tachometer_sub_ = create_subscription<mtt_msgs::msg::MttTachometerData>(
    tachometer_topic_, rclcpp::SensorDataQoS(),
    std::bind(&MttJointStateBuilderNode::on_tachometer, this, std::placeholders::_1));

  const auto timer_period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_));
  publish_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(timer_period),
    std::bind(&MttJointStateBuilderNode::publish_joint_states, this));

  RCLCPP_INFO(
    get_logger(),
    "Joint-state builder started (joint_states=%s, articulation=%s, tachometer=%s)",
    joint_state_topic_.c_str(),
    articulation_topic_.c_str(),
    tachometer_topic_.c_str());
}

void MttJointStateBuilderNode::on_articulation(const std_msgs::msg::Float64::SharedPtr msg)
{
  articulation_rad_ = msg->data;
}

void MttJointStateBuilderNode::on_tachometer(const mtt_msgs::msg::MttTachometerData::SharedPtr msg)
{
  const auto stamp = message_stamp_or_now(msg->header);
  if (last_tacho_stamp_) {
    const double dt = (stamp - *last_tacho_stamp_).seconds();
    if (dt > 0.0 && dt < 1.0) {
      cumulative_distance_m_ += signed_speed_from_tachometer(*msg) * dt;
    }
  }
  last_tacho_stamp_ = stamp;
}

void MttJointStateBuilderNode::publish_joint_states()
{
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = now();
  msg.name = joint_names_;
  msg.position.resize(joint_names_.size(), 0.0);

  const double articulation_delta = std::clamp(
    articulation_sign_ * articulation_rad_,
    -max_articulation_rad_,
    max_articulation_rad_);

  const double left_wheel_angle =
    (trailer_wheel_radius_m_ > 1e-6)
    ? left_wheel_rotation_sign_ * cumulative_distance_m_ / trailer_wheel_radius_m_
    : 0.0;
  const double right_wheel_angle =
    (trailer_wheel_radius_m_ > 1e-6)
    ? right_wheel_rotation_sign_ * cumulative_distance_m_ / trailer_wheel_radius_m_
    : 0.0;

  msg.position[0] = pitch_rest_rad_;
  msg.position[1] = yaw_rest_rad_ + articulation_delta;
  msg.position[2] = roll_rest_rad_;
  msg.position[3] = trailer_left_link_rest_rad_;
  msg.position[4] = trailer_right_link_rest_rad_;
  msg.position[5] = left_wheel_angle;
  msg.position[6] = left_wheel_angle;
  msg.position[7] = right_wheel_angle;
  msg.position[8] = right_wheel_angle;

  joint_state_pub_->publish(std::move(msg));
}

rclcpp::Time MttJointStateBuilderNode::message_stamp_or_now(const std_msgs::msg::Header & header) const
{
  if (header.stamp.sec == 0 && header.stamp.nanosec == 0) {
    return now();
  }
  return rclcpp::Time(header.stamp);
}

double MttJointStateBuilderNode::signed_speed_from_tachometer(
  const mtt_msgs::msg::MttTachometerData & msg) const
{
  if (msg.model_state_valid && std::abs(msg.model_speed_ms) > 1e-4) {
    return msg.model_speed_ms;
  }
  if (msg.speed_ms < -1e-4) {
    return msg.speed_ms;
  }
  const double direction_sign = (msg.direction == "Reverse") ? -1.0 : 1.0;
  return msg.speed_ms * direction_sign;
}

}  // namespace mtt

RCLCPP_COMPONENTS_REGISTER_NODE(mtt::MttJointStateBuilderNode)
