// MTT-154 CAN Driver Node — ROS 2 Component
// Replaces mtt_ros_wrapper.py.
// Owns the CAN interface, command frame, and publishes tachometer + vehicle status.

#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

#include <mtt_msgs/msg/mtt_tachometer_data.hpp>
#include <mtt_msgs/msg/mtt_vehicle_status.hpp>
#include <mtt_msgs/msg/mtt_aux_command.hpp>
#include <mtt_msgs/msg/mtt_driving_mode.hpp>
#include <mtt_msgs/msg/mtt_bms_data.hpp>
#include <mtt_msgs/msg/mtt_can_frame.hpp>
#include <mtt_interfaces/srv/set_vehicule_type_srv.hpp>
#include <mtt_interfaces/srv/get_vehicule_type_srv.hpp>
#include <mtt_interfaces/srv/set_steer_control_mode.hpp>

#include "mtt_driver/hardware/can_interface.hpp"
#include "mtt_driver/hardware/linux_socket_can.hpp"
#include "mtt_driver/logic/can_frame_codec.hpp"
#include "mtt_driver/logic/command_motion_model.hpp"
#include "mtt_driver/logic/hold_assist.hpp"
#include "mtt_driver/logic/tachometer.hpp"
#include "mtt_driver/logic/vehicle_params.hpp"

namespace mtt {

class MttCanNode : public rclcpp::Node {
public:
  explicit MttCanNode(const rclcpp::NodeOptions& options);
  ~MttCanNode() override;

private:
  // ── Parameters ──────────────────────────────────────────────────────
  std::string can_interface_name_;
  uint32_t    can_id_;
  double      control_freq_hz_;
  double      can_frame_freq_hz_;
  double      telemetry_timeout_ms_;
  double      command_timeout_s_;
  double      max_linear_speed_ms_;
  double      throttle_deadband_;
  double      steer_deadband_;
  double      wheelbase_m_;
  double      min_steer_speed_ms_;
  logic::CommandMotionParams motion_model_params_{};
  logic::HoldAssistParams hold_assist_params_{};
  std::string base_frame_;
  std::string cmd_angular_mode_;
  std::string steer_control_mode_;
  std::string tachometer_mode_;
  bool        invert_inferred_tachometer_direction_;
  bool        publish_can_debug_;
  std::string can_debug_topic_;

  // ── Hardware ─────────────────────────────────────────────────────────
  std::shared_ptr<hardware::ICanInterface> can_;
  std::thread receiver_thread_;
  std::atomic<bool> receiver_running_{false};

  // ── State (protected by frame_mutex_) ────────────────────────────────
  mutable std::mutex frame_mutex_;
  can::CommandFrame  command_frame_;
  TachometerState    tachometer_;
  can::BmsReading    bms_reading_;
  can::ControllerVersionReading controller_versions_;
  std::set<std::string> safety_locks_;
  double   current_steering_input_{0.0};
  double   current_linear_command_ms_{0.0};
  double   effective_linear_command_ms_{0.0};
  int      current_driving_mode_{0};
  bool     command_timeout_active_{false};
  bool     cmd_vel_seen_{false};
  bool     teleop_estop_active_{false};
  bool     teleop_estop_seen_{false};
  bool     teleop_deadman_active_{false};
  bool     teleop_deadman_seen_{false};
  double   synthetic_distance_m_{0.0};
  logic::CommandMotionModel synthetic_motion_model_{};
  logic::HoldAssistController hold_assist_controller_{};
  logic::HoldAssistOutput last_hold_assist_output_{};
  bool     synthetic_tachometer_initialized_{false};
  std::chrono::steady_clock::time_point last_synthetic_update_{};
  std::chrono::steady_clock::time_point last_hold_assist_update_{};
  std::chrono::steady_clock::time_point last_cmd_vel_time_{};

  // ── ROS I/O ──────────────────────────────────────────────────────────
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<mtt_msgs::msg::MttAuxCommand>::SharedPtr aux_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr deadman_sub_;

  rclcpp::Publisher<mtt_msgs::msg::MttTachometerData>::SharedPtr tachometer_pub_;
  rclcpp::Publisher<mtt_msgs::msg::MttVehicleStatus>::SharedPtr  status_pub_;
  rclcpp::Publisher<mtt_msgs::msg::MttDrivingMode>::SharedPtr    driving_mode_pub_;
  // mtt/articulation_cmd: commanded articulation angle in rad (standalone, SensorDataQoS).
  // Mirrors mtt_tachometer.model_articulation_command_rad for direct Foxglove/rqt monitoring.
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr           articulation_cmd_pub_;
  rclcpp::Publisher<mtt_msgs::msg::MttBmsData>::SharedPtr        bms_pub_;
  rclcpp::Publisher<mtt_msgs::msg::MttCanFrame>::SharedPtr       can_debug_pub_;

  rclcpp::Service<mtt_interfaces::srv::SetVehiculeTypeSrv>::SharedPtr set_mode_srv_;
  rclcpp::Service<mtt_interfaces::srv::GetVehiculeTypeSrv>::SharedPtr get_mode_srv_;
  rclcpp::Service<mtt_interfaces::srv::SetSteerControlMode>::SharedPtr set_steer_mode_srv_;

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr can_send_timer_;

  // ── Methods ───────────────────────────────────────────────────────────
  void init_can_interface();
  void receiver_loop();

  void on_cmd_vel(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void on_aux_cmd(const mtt_msgs::msg::MttAuxCommand::SharedPtr msg);
  void on_estop(const std_msgs::msg::Bool::SharedPtr msg);
  void on_deadman(const std_msgs::msg::Bool::SharedPtr msg);

  void control_loop();
  void send_can_frame();
  void publish_vehicle_data();
  void publish_can_debug_frame(const hardware::CanFrame& frame, bool is_tx, bool handled_by_driver);
  void refresh_command_frame();

  void apply_command_timeout_if_needed();
  bool cmd_vel_is_fresh() const;
  double command_angular_to_normalized_steer(double linear_x, double angular_z) const;
  can::Direction infer_tachometer_direction(can::Direction commanded_direction) const;

  void set_safety_lock(const std::string& reason, bool active);
  void sync_safety_switch();
  std::string describe_safety_state(const std::string& driver_state) const;

  void on_set_mode(
    const mtt_interfaces::srv::SetVehiculeTypeSrv::Request::SharedPtr req,
    mtt_interfaces::srv::SetVehiculeTypeSrv::Response::SharedPtr res);
  void on_get_mode(
    const mtt_interfaces::srv::GetVehiculeTypeSrv::Request::SharedPtr req,
    mtt_interfaces::srv::GetVehiculeTypeSrv::Response::SharedPtr res);
  void on_set_steer_mode(
    const mtt_interfaces::srv::SetSteerControlMode::Request::SharedPtr req,
    mtt_interfaces::srv::SetSteerControlMode::Response::SharedPtr res);
};

}  // namespace mtt
