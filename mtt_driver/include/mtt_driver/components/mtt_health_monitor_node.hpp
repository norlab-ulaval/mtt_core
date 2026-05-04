// MTT health monitor component.
// Aggregates operator-facing health, command, battery, freshness, and a
// command-only fallback motion estimate for degraded tachometer situations.

#pragma once

#include <chrono>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <mtt_msgs/msg/mtt_bms_data.hpp>
#include <mtt_msgs/msg/mtt_can_frame.hpp>
#include <mtt_msgs/msg/mtt_health_state.hpp>
#include <mtt_msgs/msg/mtt_tachometer_data.hpp>
#include <mtt_msgs/msg/mtt_vehicle_status.hpp>

#include "mtt_driver/logic/command_motion_model.hpp"

namespace mtt {

class MttHealthMonitorNode : public rclcpp::Node {
public:
  explicit MttHealthMonitorNode(const rclcpp::NodeOptions& options);

private:
  struct FrameRateTracker {
    void update(std::chrono::steady_clock::time_point now);
    float hz(std::chrono::steady_clock::time_point now) const;

    std::deque<std::chrono::steady_clock::time_point> samples;
    double window_s{2.0};
  };

  // Parameters
  std::string base_frame_;
  std::string odom_frame_;
  std::string health_topic_;
  std::string fallback_odom_topic_;
  std::string status_topic_;
  std::string battery_topic_;
  std::string tachometer_topic_;
  std::string cmd_vel_topic_;
  std::string can_debug_topic_;
  std::string steer_control_mode_;
  std::string cmd_angular_mode_;
  double health_publish_hz_{10.0};
  double cmd_vel_timeout_s_{0.5};
  double wheelbase_m_{0.0};
  double max_articulation_rad_{0.0};
  double min_steer_speed_ms_{0.1};
  logic::CommandMotionParams motion_model_params_{};
  int throttle_conflict_raw_threshold_{20};
  int brake_conflict_raw_threshold_{20};
  double encoder_temp_warn_c_{50.0};
  double encoder_temp_danger_c_{80.0};
  double encoder_temp_critical_c_{100.0};
  double controller_temp_warn_c_{70.0};
  double controller_temp_danger_c_{90.0};
  double controller_temp_critical_c_{100.0};
  double battery_temp_warn_c_{55.0};
  double battery_temp_danger_c_{70.0};
  double battery_temp_critical_c_{85.0};
  double low_soc_warning_percent_{15.0};
  double can_stale_warning_s_{1.0};

  // State
  mutable std::mutex state_mutex_;
  mtt_msgs::msg::MttVehicleStatus last_status_;
  mtt_msgs::msg::MttBmsData last_bms_;
  mtt_msgs::msg::MttTachometerData last_tacho_;
  geometry_msgs::msg::TwistStamped last_cmd_vel_;
  bool has_status_{false};
  bool has_bms_{false};
  bool has_tacho_{false};
  bool has_cmd_vel_{false};
  bool saw_can_debug_{false};
  std::chrono::steady_clock::time_point last_cmd_vel_time_{};
  std::chrono::steady_clock::time_point last_can_debug_time_{};
  FrameRateTracker telemetry_tracker_;
  FrameRateTracker bms_cell_tracker_;
  FrameRateTracker bms_sys_tracker_;
  FrameRateTracker bms_core_tracker_;
  FrameRateTracker bms_datetime_tracker_;
  FrameRateTracker charger_status_tracker_;
  double fallback_x_{0.0};
  double fallback_y_{0.0};
  double fallback_heading_{0.0};
  double fallback_distance_m_{0.0};
  logic::CommandMotionModel fallback_motion_model_{};
  std::chrono::steady_clock::time_point last_fallback_update_{};
  bool fallback_initialized_{false};

  // ROS
  rclcpp::Subscription<mtt_msgs::msg::MttVehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<mtt_msgs::msg::MttBmsData>::SharedPtr bms_sub_;
  rclcpp::Subscription<mtt_msgs::msg::MttTachometerData>::SharedPtr tacho_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<mtt_msgs::msg::MttCanFrame>::SharedPtr can_debug_sub_;
  rclcpp::Publisher<mtt_msgs::msg::MttHealthState>::SharedPtr health_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fallback_odom_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // Callbacks
  void on_status(const mtt_msgs::msg::MttVehicleStatus::SharedPtr msg);
  void on_bms(const mtt_msgs::msg::MttBmsData::SharedPtr msg);
  void on_tacho(const mtt_msgs::msg::MttTachometerData::SharedPtr msg);
  void on_cmd_vel(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void on_can_debug(const mtt_msgs::msg::MttCanFrame::SharedPtr msg);
  void publish_health();

  // Helpers
  double command_angular_to_normalized_steer(double angular_input, double speed_ms) const;
  double normalized_steer_to_yaw_rate(double normalized_steer, double speed_ms) const;
  double command_to_yaw_rate(double angular_input, double speed_ms) const;
  static double wrap_angle(double angle);
  static std::string temperature_state(
    double value_c, double warn_c, double danger_c, double critical_c, bool valid);
  static void append_warning(std::vector<std::string>& warnings, const std::string& text, bool active);
};

}  // namespace mtt
