// ============================================================================
// ComPositionNode — spring-return motor position controller.
//
// C++20 port of com_dynamic_shift/joy_com_position_node.py (fixed/complete).
//
// State machine:
//   OFF   : no output; driver holds its last position.
//   SETUP : slow jog-velocity mode, no limits.
//           Set home via /mtt_control/com_set_home → transitions to RUN.
//   RUN   : spring position = home + steer × amplitude.
//           Dead-man released → position freezes.
//           Dead-man re-armed → slide-in at rearm_slew (avoids jerk).
//   PARK  : autonomous return to home at rearm_slew (ignores dead-man).
//           Calibration file written on arrival. Safe to power off.
//
// Topics consumed:
//   mtt_control/com_mode      (Bool)    — ON/OFF toggle from operator node
//   mtt_control/com_steer     (Float64) — normalised [-1, 1] stick value
//   mtt_control/com_set_home  (Empty)   — sets home at current cmd position
//   mtt_control/com_park      (Empty)   — return to home, save calibration
//   mtt_control/teleop_deadman(Bool)    — dead-man switch
//   /motor/joint_state        (JointState) — actual motor position (counts)
//
// Topic published:
//   /motor/cmd_position       (Float64) — target position in encoder counts
//
// Calibration persistence (survives container restarts, detects power cycles):
//   On SET_HOME or PARK arrival → writes calibration_file with
//     home_counts and actual_at_save.
//   On startup → reads file; if |actual − actual_at_save| < consistency_threshold
//     the motor session is continuous → auto-restores home, skips SETUP.
//   If mismatch → motor was power-cycled → SETUP required (but since the
//     operator parked at home before shutdown, re-homing is just pressing SET_HOME).
// ============================================================================
#pragma once

#include <cmath>
#include <limits>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64.hpp>

namespace mtt_motor_control
{

class ComPositionNode : public rclcpp::Node
{
public:
    explicit ComPositionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    // -----------------------------------------------------------------------
    // State machine
    // -----------------------------------------------------------------------
    enum class State { OFF, SETUP, RUN, PARK };

    void enter_setup();
    void enter_run();                       // called by set_home callback
    void enter_run_at_configured_home();    // called when home_position_counts_ is set
    void enter_park();                      // called by com_park topic

    // -----------------------------------------------------------------------
    // Subscription callbacks
    // -----------------------------------------------------------------------
    void on_com_mode(const std_msgs::msg::Bool::SharedPtr msg);
    void on_com_steer(const std_msgs::msg::Float64::SharedPtr msg);
    void on_set_home(const std_msgs::msg::Empty::SharedPtr msg);
    void on_com_park(const std_msgs::msg::Empty::SharedPtr msg);
    void on_deadman(const std_msgs::msg::Bool::SharedPtr msg);
    void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg);

    // -----------------------------------------------------------------------
    // 100-Hz control loop
    // -----------------------------------------------------------------------
    void loop();

    // -----------------------------------------------------------------------
    // Calibration persistence
    // -----------------------------------------------------------------------
    void save_calibration() const;
    bool try_restore_from_calibration();    // sets home_position_counts_, returns true on success

    // -----------------------------------------------------------------------
    // Parameter hot-reload
    // -----------------------------------------------------------------------
    rcl_interfaces::msg::SetParametersResult on_param_change(
        const std::vector<rclcpp::Parameter> & params);

    // -----------------------------------------------------------------------
    // Geometry helper
    // -----------------------------------------------------------------------
    static double move_toward(double cur, double target, double step) noexcept;

    // -----------------------------------------------------------------------
    // Parameters (all reloadable at runtime)
    // -----------------------------------------------------------------------
    double amplitude_    {50000.0};  // counts — half-range in RUN mode
    double setup_slew_   { 4000.0};  // counts/s — jog speed in SETUP
    double run_slew_     {20000.0};  // counts/s — max speed in RUN
    double rearm_slew_   { 4000.0};  // counts/s — slide-in / park speed
    // Pre-configured home (NaN = not set → use manual SETUP or calibration file).
    double home_position_counts_ {std::numeric_limits<double>::quiet_NaN()};
    // Calibration persistence
    std::string calib_file_           {"/data/mtt/com_calibration.yaml"};
    double      consistency_threshold_{1000.0};  // counts — max drift to auto-restore home

    // -----------------------------------------------------------------------
    // Runtime state
    // -----------------------------------------------------------------------
    State               state_        {State::OFF};
    double              cmd_          {0.0};    // current commanded position (counts)
    std::optional<double> home_;                // home position in counts (set by set_home)
    double              actual_       {0.0};    // latest position from joint_state
    bool                seeded_       {false};  // true once first joint_state received
    double              steer_        {0.0};    // [-1, 1] from com_steer topic
    bool                deadman_      {false};
    bool                prev_deadman_ {false};
    bool                com_mode_     {false};  // last value of com_mode topic
    bool                sliding_in_   {false};  // slide-in flag on dead-man re-arm
    bool                pending_setup_{false};
    bool                park_arrived_ {false};  // true once PARK reaches home

    double dt_ {0.01};  // seconds between loop ticks (1/rate)

    // -----------------------------------------------------------------------
    // ROS interfaces
    // -----------------------------------------------------------------------
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cmd_pub_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr      com_mode_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr   steer_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr     set_home_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr     com_park_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr      deadman_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;

    rclcpp::TimerBase::SharedPtr loop_timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

}  // namespace mtt_motor_control
