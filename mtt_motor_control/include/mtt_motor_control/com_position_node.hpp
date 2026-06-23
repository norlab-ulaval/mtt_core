// ── ComPositionNode: motor position controller with free + spring modes ──
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
    // ── State machine ──
    enum class State { OFF, SETUP, RUN, PARK };

    void enter_setup();
    void enter_run();                       // called by com_spring ON
    void enter_park();                      // called by com_park topic

    // ── Subscription callbacks ──
    void on_com_mode(const std_msgs::msg::Bool::SharedPtr msg);
    void on_com_steer(const std_msgs::msg::Float64::SharedPtr msg);
    void on_set_home(const std_msgs::msg::Empty::SharedPtr msg);
    void on_com_park(const std_msgs::msg::Empty::SharedPtr msg);
    void on_com_spring(const std_msgs::msg::Bool::SharedPtr msg);
    void on_deadman(const std_msgs::msg::Bool::SharedPtr msg);
    void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg);

    // ── Control loop ──
    void loop();

    // ── Calibration persistence ──
    void save_calibration() const;
    bool try_restore_from_calibration();    // sets home_position_counts_, returns true on success

    // ── Params hot-reload ──
    rcl_interfaces::msg::SetParametersResult on_param_change(
        const std::vector<rclcpp::Parameter> & params);

    // ── Geometry helper ──
    static double move_toward(double cur, double target, double step) noexcept;

    // ── Parameters ──
    double amplitude_    {50000.0};  // counts — half-range in RUN mode
    double setup_slew_   { 4000.0};  // counts/s — jog speed in SETUP
    double run_slew_     {20000.0};  // counts/s — max speed in RUN
    double rearm_slew_   { 4000.0};  // counts/s — slide-in / park speed
    double steer_deadband_ {0.05};   // normalized — zero zone around stick centre
    // Pre-configured home (NaN = not set → use calibration file or manual SET_HOME).
    double home_position_counts_ {std::numeric_limits<double>::quiet_NaN()};

    // Calibration persistence
    std::string calib_file_           {"/data/mtt/com_calibration.yaml"};
    double      consistency_threshold_{1000.0};  // counts — max drift to auto-restore home

    // ── Runtime state ──
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
    bool                spring_enabled_{false}; // true = RUN, false = SETUP

    double dt_ {0.01};  // seconds between loop ticks (1/rate)

    // ── ROS interfaces ──
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cmd_pub_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr      com_mode_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr   steer_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr     set_home_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr     com_park_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr      spring_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr      deadman_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;

    rclcpp::TimerBase::SharedPtr loop_timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

}  // namespace mtt_motor_control
