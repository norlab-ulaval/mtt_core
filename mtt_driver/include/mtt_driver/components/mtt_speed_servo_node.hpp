#ifndef MTT_DRIVER__COMPONENTS__MTT_SPEED_SERVO_NODE_HPP_
#define MTT_DRIVER__COMPONENTS__MTT_SPEED_SERVO_NODE_HPP_

// MttSpeedServoNode — Software closed-loop speed controller
//
// Replaces the open-loop cmd_vel.linear.x → throttle_byte mapping in the CAN node
// with a PI controller that uses tachometer feedback to track the desired speed.
//
//  ┌──────────────────────────────────────────────────────────────────┐
//  │ Speed mode (mode="speed")                                        │
//  │   /speed_setpoint (Float64, m/s) ──> PI + feedforward           │
//  │                                   ↑   ↓                         │
//  │   /mtt_tachometer ────────────────┘   └──> speed_servo/cmd_vel  │
//  ├──────────────────────────────────────────────────────────────────┤
//  │ Characterize mode (mode="characterize")                          │
//  │   Automated open-loop throttle ramp → builds feedforward LUT    │
//  │   Output: speed_servo/feedforward_map (Float64MultiArray)        │
//  │           + YAML log to console for copy-paste into config       │
//  └──────────────────────────────────────────────────────────────────┘
//
// Output: speed_servo/cmd_vel (TwistStamped)
//   → feeds into mtt_cmd_arbiter_node as the "auto" source
//   → angular.z is forwarded from the original /speed_setpoint header
//     (steering is handled separately by articulation servo or cmd_vel)
//
// Diagnostics (SensorDataQoS — Foxglove / characterization):
//   /speed_servo/setpoint_ms          — desired speed after rate limiter
//   /speed_servo/measured_ms          — tachometer feedback
//   /speed_servo/error_ms             — setpoint - measured
//   /speed_servo/throttle_normalized  — final PI output [0, 1]
//   /speed_servo/feedforward_map      — LUT as flat array [s0,t0, s1,t1, ...] (characterize mode)

#include <atomic>
#include <chrono>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "mtt_msgs/msg/mtt_tachometer_data.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "mtt_driver/logic/speed_servo.hpp"

namespace mtt
{

class MttSpeedServoNode final : public rclcpp::Node
{
public:
  explicit MttSpeedServoNode(const rclcpp::NodeOptions & options);

private:
  // ── Timer callbacks ────────────────────────────────────────────────
  void control_loop();
  void characterize_loop();

  // ── Helpers ────────────────────────────────────────────────────────
  void publish_zero_cmd();
  void publish_diagnostics(const logic::SpeedServoDebug & dbg);
  void log_lut_yaml(const std::vector<std::pair<double, double>> & lut) const;

  // ── Parameters ─────────────────────────────────────────────────────
  std::string mode_;
  double feedback_timeout_s_;
  double command_timeout_s_;
  double control_frequency_hz_;

  // Characterization params
  double char_throttle_step_;
  double char_settle_time_s_;
  double char_sample_time_s_;
  double char_max_speed_ms_;

  // ── Controller ─────────────────────────────────────────────────────
  logic::SpeedServo servo_;

  // ── Shared state (mutex protected) ────────────────────────────────
  mutable std::mutex state_mutex_;

  std::optional<double> latest_speed_ms_;        ///< from /mtt_tachometer
  rclcpp::Time          latest_tacho_stamp_{0, 0, RCL_ROS_TIME};
  bool                  tacho_direction_forward_{true};

  std::optional<double> latest_setpoint_ms_;     ///< from /speed_setpoint
  rclcpp::Time          latest_setpoint_stamp_{0, 0, RCL_ROS_TIME};

  bool deadman_active_{false};                   ///< for characterize mode safety

  // Characterization state
  std::atomic<bool>     char_running_{false};
  double                char_current_throttle_{0.0};
  std::vector<double>   char_sample_buffer_;
  std::vector<std::pair<double, double>> char_lut_;

  // ── ROS interfaces ────────────────────────────────────────────────
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr              setpoint_sub_;
  rclcpp::Subscription<mtt_msgs::msg::MttTachometerData>::SharedPtr    tacho_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                 deadman_sub_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr       cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr                 diag_setpoint_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr                 diag_measured_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr                 diag_error_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr                 diag_throttle_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr       feedforward_map_pub_;

  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace mtt

#endif  // MTT_DRIVER__COMPONENTS__MTT_SPEED_SERVO_NODE_HPP_
