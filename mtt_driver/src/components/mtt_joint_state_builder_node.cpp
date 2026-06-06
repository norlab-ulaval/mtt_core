#include "mtt_driver/components/mtt_joint_state_builder_node.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
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
  drive_joint_radius_m_ = declare_parameter("drive_joint_radius_m", VehicleParams::wheel_radius());
  drive_joint_rotation_sign_ = declare_parameter("drive_joint_rotation_sign", -1.0);
  trailer_wheel_radius_m_ = declare_parameter("trailer_wheel_radius_m", 0.0508);
  left_wheel_rotation_sign_ = declare_parameter("left_wheel_rotation_sign", 1.0);
  right_wheel_rotation_sign_ = declare_parameter("right_wheel_rotation_sign", 1.0);
  // pitch_topic: subscribe to hardware pitch sensor when non-empty.
  // Requires pitch_deg_per_bit calibration in mtt_driver.yaml before enabling.
  pitch_topic_ = declare_parameter("pitch_topic", std::string(""));
  pitch_sign_  = declare_parameter("pitch_sign", 1.0);

  joint_names_ = {
    "pitch",
    "yaw",
    "roll",
    "Remorque_lien_roue_gauche_joint",
    "Remorque_lien_roue_droite_joint",
    "frontleft_wheel",
    "backleft_wheel",
    "frontright_wheel",
    "backright_wheel",
  };
  for (int i = 1; i <= 20; ++i) {
    joint_names_.push_back(std::to_string(i) + "_continuous");
  }

  joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(joint_state_topic_, 10);
  articulation_sub_ = create_subscription<std_msgs::msg::Float64>(
    articulation_topic_, 10,
    std::bind(&MttJointStateBuilderNode::on_articulation, this, std::placeholders::_1));
  tachometer_sub_ = create_subscription<mtt_msgs::msg::MttTachometerData>(
    tachometer_topic_, rclcpp::SensorDataQoS(),
    std::bind(&MttJointStateBuilderNode::on_tachometer, this, std::placeholders::_1));

  if (!pitch_topic_.empty()) {
    pitch_sub_ = create_subscription<std_msgs::msg::Float64>(
      pitch_topic_, 10,
      std::bind(&MttJointStateBuilderNode::on_pitch, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Hardware pitch subscription enabled on %s", pitch_topic_.c_str());
  }

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
  std::lock_guard<std::mutex> lock(state_mutex_);
  articulation_rad_ = msg->data;
}

void MttJointStateBuilderNode::on_pitch(const std_msgs::msg::Float64::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  hardware_pitch_rad_ = pitch_sign_ * msg->data;
}

void MttJointStateBuilderNode::on_tachometer(const mtt_msgs::msg::MttTachometerData::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
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
  std::lock_guard<std::mutex> lock(state_mutex_);
  bool use_sim_time = false;
  (void)get_parameter("use_sim_time", use_sim_time);
  const auto stamp = now();
  if (use_sim_time && (stamp.nanoseconds() <= 0 || stamp.seconds() >= 1.0e8)) {
    return;
  }

  sensor_msgs::msg::JointState msg;
  msg.header.stamp = stamp;
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
  const double drive_joint_angle =
    (drive_joint_radius_m_ > 1e-6)
    ? drive_joint_rotation_sign_ * cumulative_distance_m_ / drive_joint_radius_m_
    : 0.0;

  // pitch joint: rest offset + live hardware measurement when pitch_topic is configured
  msg.position[0] = pitch_rest_rad_ + (pitch_sub_ ? hardware_pitch_rad_ : 0.0);
  msg.position[1] = yaw_rest_rad_ + articulation_delta;
  msg.position[2] = roll_rest_rad_;
  msg.position[3] = trailer_left_link_rest_rad_;
  msg.position[4] = trailer_right_link_rest_rad_;
  msg.position[5] = left_wheel_angle;
  msg.position[6] = left_wheel_angle;
  msg.position[7] = right_wheel_angle;
  msg.position[8] = right_wheel_angle;
  for (std::size_t i = 9; i < msg.position.size(); ++i) {
    msg.position[i] = drive_joint_angle;
  }

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
