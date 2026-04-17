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
#include <std_msgs/msg/u_int8.hpp>

#include <mtt_msgs/msg/mtt_tachometer_data.hpp>
#include <mtt_msgs/msg/mtt_vehicle_status.hpp>
#include <mtt_msgs/msg/mtt_aux_command.hpp>
#include <mtt_msgs/msg/mtt_driving_mode.hpp>
#include <mtt_msgs/msg/mtt_bms_data.hpp>
#include <mtt_interfaces/srv/set_vehicule_type_srv.hpp>
#include <mtt_interfaces/srv/get_vehicule_type_srv.hpp>

#include "mtt_driver/hardware/can_interface.hpp"
#include "mtt_driver/hardware/linux_socket_can.hpp"
#include "mtt_driver/logic/can_frame_codec.hpp"
#include "mtt_driver/logic/tachometer.hpp"

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
  std::string base_frame_;

  // ── Hardware ─────────────────────────────────────────────────────────
  std::shared_ptr<hardware::ICanInterface> can_;
  std::thread receiver_thread_;
  std::atomic<bool> receiver_running_{false};

  // ── State (protected by frame_mutex_) ────────────────────────────────
  mutable std::mutex frame_mutex_;
  can::CommandFrame  command_frame_;
  TachometerState    tachometer_;
  can::BmsReading    bms_reading_;
  double energy_consumed_wh_{0.0};  // cumulative session energy (Wh)
  std::set<std::string> safety_locks_;
  double   current_steering_input_{0.0};
  int      current_driving_mode_{0};
  bool     command_timeout_active_{false};
  bool     teleop_estop_active_{false};
  bool     teleop_estop_seen_{false};
  std::chrono::steady_clock::time_point last_cmd_vel_time_{};

  // ── ROS I/O ──────────────────────────────────────────────────────────
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<mtt_msgs::msg::MttAuxCommand>::SharedPtr aux_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;

  rclcpp::Publisher<mtt_msgs::msg::MttTachometerData>::SharedPtr tachometer_pub_;
  rclcpp::Publisher<mtt_msgs::msg::MttVehicleStatus>::SharedPtr  status_pub_;
  rclcpp::Publisher<mtt_msgs::msg::MttDrivingMode>::SharedPtr    driving_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr             steer_cmd_pub_;
  rclcpp::Publisher<mtt_msgs::msg::MttBmsData>::SharedPtr        bms_pub_;

  rclcpp::Service<mtt_interfaces::srv::SetVehiculeTypeSrv>::SharedPtr set_mode_srv_;
  rclcpp::Service<mtt_interfaces::srv::GetVehiculeTypeSrv>::SharedPtr get_mode_srv_;

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr can_send_timer_;

  // ── Methods ───────────────────────────────────────────────────────────
  void init_can_interface();
  void receiver_loop();

  void on_cmd_vel(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void on_aux_cmd(const mtt_msgs::msg::MttAuxCommand::SharedPtr msg);
  void on_estop(const std_msgs::msg::Bool::SharedPtr msg);

  void control_loop();
  void send_can_frame();
  void publish_vehicle_data();

  void apply_command_timeout_if_needed();
  bool cmd_vel_is_fresh() const;

  void set_safety_lock(const std::string& reason, bool active);
  void sync_safety_switch();
  std::string describe_safety_state(const std::string& driver_state) const;

  void on_set_mode(
    const mtt_interfaces::srv::SetVehiculeTypeSrv::Request::SharedPtr req,
    mtt_interfaces::srv::SetVehiculeTypeSrv::Response::SharedPtr res);
  void on_get_mode(
    const mtt_interfaces::srv::GetVehiculeTypeSrv::Request::SharedPtr req,
    mtt_interfaces::srv::GetVehiculeTypeSrv::Response::SharedPtr res);
};

}  // namespace mtt
