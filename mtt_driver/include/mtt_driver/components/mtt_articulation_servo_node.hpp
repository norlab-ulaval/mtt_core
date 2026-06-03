#ifndef MTT_DRIVER__COMPONENTS__MTT_ARTICULATION_SERVO_NODE_HPP_
#define MTT_DRIVER__COMPONENTS__MTT_ARTICULATION_SERVO_NODE_HPP_

// MttArticulationServoNode — Software closed-loop articulation controller
//
// Adds a software outer PD loop on top of the hardware servo (CAN closed_loop mode).
// Uses the hardware encoder (/hardware/articulation_angle, 100 Hz) as primary feedback
// to correct for calibration drift, backlash, and steer-byte nonlinearity.
//
//  ┌────────────────────────────────────────────────────────────┐
//  │ Position mode (mode="position")                            │
//  │   /mtt_articulation_setpoint (rad) ──> PD ──> steer_cmd   │
//  │                                   ↑                        │
//  │   /hardware/articulation_angle ───┘                        │
//  ├────────────────────────────────────────────────────────────┤
//  │ Velocity mode (mode="velocity")                            │
//  │   /mtt_articulation_velocity_cmd (rad/s)                   │
//  │       → integrate → setpoint → PD → steer_cmd             │
//  └────────────────────────────────────────────────────────────┘
//
// Output: /articulation_servo/steer_cmd (Float64, [-1,+1])
//   → connects to mtt_can_node servo_steer_override subscription
//   → CAN node uses this as steering input when fresh (overrides cmd_vel angular.z)
//
// Diagnostics (for characterization / step response / Foxglove):
//   /articulation_servo/setpoint_rad  — target angle
//   /articulation_servo/measured_rad  — encoder feedback
//   /articulation_servo/error_rad     — position error
//   /articulation_servo/steer_cmd     — PD output (same as main output)

#include <chrono>
#include <mutex>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include "mtt_driver/logic/articulation_servo.hpp"

namespace mtt
{

class MttArticulationServoNode final : public rclcpp::Node
{
public:
  explicit MttArticulationServoNode(const rclcpp::NodeOptions & options);

private:
  // ── Timer callback — runs PD controller ────────────────────────────
  void control_loop();

  // ── Parameters ─────────────────────────────────────────────────────
  std::string mode_;            ///< "position" | "velocity" | "disabled"
  double feedback_timeout_s_;   ///< drop to open-loop if encoder stale > this
  double max_articulation_rad_; ///< ±physical limit (rad)

  // ── Controller ─────────────────────────────────────────────────────
  logic::ArticulationServo servo_;

  // ── Shared state (mutex protected) ────────────────────────────────
  mutable std::mutex state_mutex_;

  std::optional<double> latest_feedback_rad_;
  rclcpp::Time          latest_feedback_stamp_{0, 0, RCL_ROS_TIME};

  std::optional<double> latest_position_cmd_rad_;
  std::optional<double> latest_velocity_cmd_rad_s_;

  // ── ROS interfaces ────────────────────────────────────────────────
  // Subscriptions
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr position_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr feedback_sub_;

  // Output
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steer_cmd_pub_;

  // Diagnostics
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr diag_setpoint_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr diag_measured_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr diag_error_pub_;

  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace mtt

#endif  // MTT_DRIVER__COMPONENTS__MTT_ARTICULATION_SERVO_NODE_HPP_
