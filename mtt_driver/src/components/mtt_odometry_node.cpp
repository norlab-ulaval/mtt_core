// MTT-154 Odometry Node — ROS 2 Component
// Replaces mtt_odometry_manager.py. Subscribes mtt_tachometer, runs the
// multi-mode odometry strategy, publishes nav_msgs/Odometry + TF.

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <chrono>

#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <mtt_msgs/msg/mtt_tachometer_data.hpp>
#include <mtt_msgs/msg/mtt_driving_mode.hpp>
#include <mtt_interfaces/srv/set_steer_control_mode.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "mtt_driver/logic/command_motion_model.hpp"
#include "mtt_driver/logic/odometry_calculator.hpp"
#include "mtt_driver/logic/vehicle_params.hpp"

namespace mtt {

class MttOdometryNode : public rclcpp::Node {
public:
  explicit MttOdometryNode(const rclcpp::NodeOptions& options)
  : rclcpp::Node("mtt_odometry_node", options)
  {
    // ── Parameters ────────────────────────────────────────────────────
    odom_frame_       = declare_parameter("odom_frame",       std::string("odom"));
    base_frame_       = declare_parameter("base_frame",       std::string("base_footprint"));
    broadcast_tf_     = declare_parameter("broadcast_tf",     true);
    publish_runtime_joint_states_ =
      declare_parameter("publish_runtime_joint_states", false);
    runtime_joint_states_topic_ =
      declare_parameter("runtime_joint_states_topic", std::string("joint_states"));
    runtime_joint_pitch_rad_ =
      declare_parameter("runtime_joint_pitch_rad", 0.0);
    runtime_joint_roll_rad_ =
      declare_parameter("runtime_joint_roll_rad", 0.0);
    runtime_joint_articulation_sign_ =
      declare_parameter("runtime_joint_articulation_sign", 1.0);
    runtime_joint_articulation_offset_rad_ =
      declare_parameter("runtime_joint_articulation_offset_rad", 0.0);
    track_width_m_    = declare_parameter("track_width_m",    VehicleParams::track_width);
    wheelbase_m_      = declare_parameter("wheelbase_m",      VehicleParams::total_wheelbase());
    steer_mode_       = declare_parameter("steer_control_mode", std::string("open_loop"));
    cmd_angular_mode_ = declare_parameter("cmd_angular_mode", std::string("normalized_steer"));
    cmd_vel_topic_    = declare_parameter("cmd_vel_topic", std::string("cmd_vel"));
    cmd_vel_timeout_s_ = declare_parameter("cmd_vel_timeout_seconds", 0.5);
    pivot_turn_       = declare_parameter("pivot_turn_enabled", false);
    min_speed_turn_   = declare_parameter("min_turn_speed_ms", 0.03);
    max_articulation_rad_ = declare_parameter("max_articulation_deg",
        VehicleParams::max_articulation_deg) * M_PI / 180.0;
    yaw_slip_factor_  = declare_parameter("yaw_slip_factor",  1.0);
    wrap_threshold_m_ = declare_parameter("wrap_reset_threshold_m", 1000.0);
    motion_model_params_.wheelbase_m = declare_parameter("model_wheelbase_m", wheelbase_m_);
    motion_model_params_.max_articulation_rad =
      declare_parameter("model_max_articulation_deg", VehicleParams::max_articulation_deg) * M_PI / 180.0;
    motion_model_params_.min_turn_speed_ms = min_speed_turn_;
    motion_model_params_.speed_response_gain = declare_parameter("model_speed_response_gain", 3.0);
    motion_model_params_.articulation_response_gain =
      declare_parameter("model_articulation_response_gain", VehicleParams::articulation_response);
    motion_model_params_.brake_gain = declare_parameter("model_brake_gain", 1.0);
    motion_model_params_.use_slip_heuristic = declare_parameter("model_use_slip_heuristic", true);
    motion_model_params_.yaw_slip_base = declare_parameter("model_yaw_slip_base", 0.10);
    motion_model_params_.yaw_slip_speed_gain = declare_parameter("model_yaw_slip_speed_gain", 0.05);
    motion_model_params_.yaw_slip_articulation_gain =
      declare_parameter("model_yaw_slip_articulation_gain", 0.15);
    motion_model_params_.yaw_slip_min_scale = declare_parameter("model_yaw_slip_min_scale", 0.55);

    if (cmd_angular_mode_ != "normalized_steer" && cmd_angular_mode_ != "yaw_rate") {
      RCLCPP_WARN(
        get_logger(),
        "Unknown cmd_angular_mode '%s', falling back to 'normalized_steer'",
        cmd_angular_mode_.c_str());
      cmd_angular_mode_ = "normalized_steer";
    }

    // ── Initial odometry mode ─────────────────────────────────────────
    calculator_ = logic::OdometryFactory::create(
      logic::DrivingMode::SingleTrailer, track_width_m_, wheelbase_m_);

    // ── Publishers ────────────────────────────────────────────────────
    odom_pub_        = create_publisher<nav_msgs::msg::Odometry>("mtt_odometry", 10);
    articulation_pub_ = create_publisher<std_msgs::msg::Float64>("mtt_articulation_angle", 10);
    if (publish_runtime_joint_states_) {
      joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
        runtime_joint_states_topic_, 10);
    }

    // ── Subscribers ────────────────────────────────────────────────────────────
    tacho_sub_ = create_subscription<mtt_msgs::msg::MttTachometerData>(
      "mtt_tachometer", rclcpp::SensorDataQoS(),
      [this](const mtt_msgs::msg::MttTachometerData::SharedPtr msg){ on_tachometer(msg); });
    mode_sub_ = create_subscription<mtt_msgs::msg::MttDrivingMode>(
      "mtt_driving_mode", 10,
      [this](const mtt_msgs::msg::MttDrivingMode::SharedPtr msg){ on_mode_change(msg); });
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      cmd_vel_topic_, 10,
      [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg){ on_cmd_vel(msg); });

    // ── Services ────────────────────────────────────────────────────────────
    reset_srv_ = create_service<std_srvs::srv::Trigger>(
      "mtt/reset_odometry",
      [this](
        const std_srvs::srv::Trigger::Request::SharedPtr req,
        std_srvs::srv::Trigger::Response::SharedPtr res
      ){ on_reset(req, res); });
    steer_mode_srv_ = create_service<mtt_interfaces::srv::SetSteerControlMode>(
      "mtt/set_steer_control_mode",
      [this](
        const mtt_interfaces::srv::SetSteerControlMode::Request::SharedPtr req,
        mtt_interfaces::srv::SetSteerControlMode::Response::SharedPtr res
      ){ on_set_steer_mode(req, res); });

    if (broadcast_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
      // Keep publishing the last known odom->base transform and articulation state
      // when telemetry pauses, so the TF tree stays usable for mapping/debugging.
      tf_fallback_timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        [this]() {
          if (!broadcast_tf_ || !tf_broadcaster_) return;
          if (!last_tacho_wall_time_) return;
          bool use_sim_time = false;
          (void)get_parameter("use_sim_time", use_sim_time);
          const auto stamp = now();
          if (use_sim_time && (stamp.nanoseconds() <= 0 || stamp.seconds() >= 1.0e8)) {
            return;
          }
          // Only publish if no tachometer update in the last 200 ms
          auto now_tp = std::chrono::steady_clock::now();
          if (now_tp - last_tacho_time_ < std::chrono::milliseconds(200)) return;
          geometry_msgs::msg::TransformStamped tf;
          double articulation_angle = 0.0;
          tf.header.stamp    = stamp;
          tf.header.frame_id = odom_frame_;
          tf.child_frame_id  = base_frame_;
          {
            std::lock_guard<std::mutex> lock(calc_mutex_);
            auto pose = calculator_->export_pose();
            tf.transform.translation.x = pose.x;
            tf.transform.translation.y = pose.y;
            tf.transform.rotation.z = std::sin(pose.heading / 2.0);
            tf.transform.rotation.w = std::cos(pose.heading / 2.0);
            articulation_angle = pose.articulation_angle;
          }
          publish_runtime_joint_state(tf.header.stamp, articulation_angle);
          tf_broadcaster_->sendTransform(tf);
        });
    }

    RCLCPP_INFO(
      get_logger(),
      "MttOdometryNode started (mode=SingleTrailer, steer=%s, cmd_angular_mode=%s, cmd_vel=%s, runtime_joint_states=%s)",
      steer_mode_.c_str(),
      cmd_angular_mode_.c_str(),
      cmd_vel_topic_.c_str(),
      publish_runtime_joint_states_ ? runtime_joint_states_topic_.c_str() : "disabled");
  }

private:
  // ── State ─────────────────────────────────────────────────────────
  std::string odom_frame_, base_frame_, steer_mode_, cmd_angular_mode_, cmd_vel_topic_, runtime_joint_states_topic_;
  bool     broadcast_tf_, pivot_turn_, publish_runtime_joint_states_;
  double   track_width_m_, wheelbase_m_;
  double   max_articulation_rad_{VehicleParams::max_articulation_rad};
  double   runtime_joint_pitch_rad_{0.0};
  double   runtime_joint_roll_rad_{0.0};
  double   runtime_joint_articulation_sign_{1.0};
  double   runtime_joint_articulation_offset_rad_{0.0};
  double   min_speed_turn_, yaw_slip_factor_, wrap_threshold_m_, cmd_vel_timeout_s_;
  double   current_angular_cmd_{0.0};
  logic::CommandMotionParams motion_model_params_{};
  std::chrono::steady_clock::time_point last_tacho_time_{};
  std::chrono::steady_clock::time_point last_cmd_vel_time_{};
  bool has_cmd_vel_{false};
  std::optional<rclcpp::Time> last_tacho_stamp_;
  std::optional<std::chrono::steady_clock::time_point> last_tacho_wall_time_;

  std::mutex calc_mutex_;
  std::mutex state_mutex_;
  std::unique_ptr<logic::IOdometryCalculator> calculator_;
  logic::DrivingMode current_mode_{logic::DrivingMode::SingleTrailer};

  // ── ROS I/O ───────────────────────────────────────────────────────
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr  articulation_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Subscription<mtt_msgs::msg::MttTachometerData>::SharedPtr tacho_sub_;
  rclcpp::Subscription<mtt_msgs::msg::MttDrivingMode>::SharedPtr    mode_sub_;
  rclcpp::TimerBase::SharedPtr tf_fallback_timer_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr  reset_srv_;
  rclcpp::Service<mtt_interfaces::srv::SetSteerControlMode>::SharedPtr steer_mode_srv_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  double normalized_steer_to_yaw_rate(double normalized_steer, double speed_ms) const
  {
    return logic::CommandMotionModel::yaw_rate_from_speed_and_steer(
      normalized_steer,
      speed_ms,
      motion_model_params_,
      motion_model_params_.use_slip_heuristic);
  }

  double command_to_yaw_rate(double angular_cmd, double speed_ms) const
  {
    if (cmd_angular_mode_ == "yaw_rate") {
      return angular_cmd;
    }

    return normalized_steer_to_yaw_rate(angular_cmd, speed_ms);
  }

  void publish_runtime_joint_state(const builtin_interfaces::msg::Time& stamp, double articulation_angle)
  {
    if (!publish_runtime_joint_states_ || !joint_state_pub_) {
      return;
    }

    const double runtime_yaw = std::clamp(
      runtime_joint_articulation_sign_ * articulation_angle + runtime_joint_articulation_offset_rad_,
      -max_articulation_rad_,
      max_articulation_rad_);

    sensor_msgs::msg::JointState msg;
    msg.header.stamp = stamp;
    msg.name = {"pitch", "yaw", "roll"};
    msg.position = {runtime_joint_pitch_rad_, runtime_yaw, runtime_joint_roll_rad_};
    joint_state_pub_->publish(msg);
  }

  // ── Callbacks ─────────────────────────────────────────────────────
  void on_cmd_vel(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_angular_cmd_ = msg->twist.angular.z;
    last_cmd_vel_time_ = std::chrono::steady_clock::now();
    has_cmd_vel_ = true;
  }

  void on_tachometer(const mtt_msgs::msg::MttTachometerData::SharedPtr msg)
  {
    const auto wall_now = std::chrono::steady_clock::now();
    last_tacho_time_ = wall_now;
    const double speed_ms   = msg->speed_ms;
    const double steer_cmd  = msg->steer_cmd;
    int dir_sign = (msg->direction == "Reverse") ? -1 : 1;
    double signed_speed_ms = 0.0;
    if (msg->model_state_valid && std::abs(msg->model_speed_ms) > 1e-4) {
      signed_speed_ms = msg->model_speed_ms;
    } else if (speed_ms < -1e-4) {
      signed_speed_ms = speed_ms;
    } else {
      signed_speed_ms = speed_ms * static_cast<double>(dir_sign);
    }
    if (std::abs(signed_speed_ms) > 1e-4) {
      dir_sign = signed_speed_ms < 0.0 ? -1 : 1;
    }

    double dt = 0.02;
    if (msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0) {
      const auto stamp = rclcpp::Time(msg->header.stamp);
      if (last_tacho_stamp_) {
        dt = (stamp - *last_tacho_stamp_).seconds();
      }
      last_tacho_stamp_ = stamp;
    }
    if ((dt <= 1e-4 || dt > 1.0) && last_tacho_wall_time_) {
      dt = std::chrono::duration<double>(wall_now - *last_tacho_wall_time_).count();
    }
    last_tacho_wall_time_ = wall_now;
    if (dt <= 1e-4 || dt > 1.0) {
      dt = 0.02;
    }

    // Compute effective angular velocity
    double eff_ang = 0.0;
    bool closed_loop = false;
    logic::DrivingMode mode_snapshot;
    double current_angular_cmd = 0.0;
    bool cmd_is_fresh = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      closed_loop = (steer_mode_ == "closed_loop");
      if (msg->model_state_valid) {
        eff_ang = msg->model_yaw_rate_effective_rad_s;
      } else if (closed_loop) {
        eff_ang = normalized_steer_to_yaw_rate(steer_cmd, signed_speed_ms);
      } else {
        cmd_is_fresh =
          has_cmd_vel_ &&
          std::chrono::duration<double>(wall_now - last_cmd_vel_time_).count() <= cmd_vel_timeout_s_;
        current_angular_cmd = current_angular_cmd_;
        eff_ang = cmd_is_fresh ? command_to_yaw_rate(current_angular_cmd, signed_speed_ms) : 0.0;
      }
    }
    {
      std::lock_guard<std::mutex> lock(calc_mutex_);
      mode_snapshot = current_mode_;
    }

    // Suppress yaw when near-stationary (no pivot turns unless enabled)
    if (!pivot_turn_ && std::abs(signed_speed_ms) < min_speed_turn_) eff_ang = 0.0;
    eff_ang = std::clamp(eff_ang * (msg->model_state_valid ? 1.0 : yaw_slip_factor_),
                         -VehicleParams::max_yaw_rate_rad_s,
                          VehicleParams::max_yaw_rate_rad_s);

    logic::OdometryInput input;
    input.distance_km      = msg->distance_km;
    input.speed_ms         = signed_speed_ms;
    input.steer_cmd        = steer_cmd;
    input.angular_velocity = eff_ang;
    input.direction_sign   = dir_sign;
    input.dt               = dt;
    input.synthetic_model_valid = msg->model_state_valid;
    input.articulation_command_rad = msg->model_articulation_command_rad;
    input.articulation_effective_rad = msg->model_articulation_effective_rad;
    input.curvature_nominal_m_inv = msg->model_curvature_nominal_m_inv;
    input.curvature_effective_m_inv = msg->model_curvature_effective_m_inv;
    input.yaw_rate_nominal_rad_s = msg->model_yaw_rate_nominal_rad_s;
    input.yaw_rate_effective_rad_s = msg->model_yaw_rate_effective_rad_s;

    logic::OdometryOutput out;
    {
      std::lock_guard<std::mutex> lock(calc_mutex_);
      out = calculator_->update(input);
      mode_snapshot = current_mode_;
    }

    // Build and publish nav_msgs/Odometry
    nav_msgs::msg::Odometry odom;
    if (msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0) {
      odom.header.stamp = msg->header.stamp;
    } else {
      odom.header.stamp = now();
    }
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id  = base_frame_;
    odom.pose.pose.position.x = out.x;
    odom.pose.pose.position.y = out.y;
    odom.pose.pose.orientation.z = std::sin(out.heading / 2.0);
    odom.pose.pose.orientation.w = std::cos(out.heading / 2.0);
    odom.twist.twist.linear.x  = out.vx;
    odom.twist.twist.angular.z = out.wz;
    // Covariances
    odom.pose.covariance[0]  = out.pos_cov;
    odom.pose.covariance[7]  = out.pos_cov;
    odom.pose.covariance[35] = out.heading_cov;
    odom.twist.covariance[0]  = out.vel_cov;
    odom.twist.covariance[35] = out.heading_cov * 2.0;
    odom_pub_->publish(odom);

    // Publish articulation angle for joint controller
    if (mode_snapshot == logic::DrivingMode::SingleTrailer) {
      std_msgs::msg::Float64 artic;
      artic.data = out.articulation_angle;
      articulation_pub_->publish(artic);
    }
    publish_runtime_joint_state(odom.header.stamp, out.articulation_angle);

    // TF broadcast
    if (broadcast_tf_ && tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header = odom.header;
      tf.child_frame_id = base_frame_;
      tf.transform.translation.x = out.x;
      tf.transform.translation.y = out.y;
      tf.transform.rotation = odom.pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf);
    }
  }

  void on_mode_change(const mtt_msgs::msg::MttDrivingMode::SharedPtr msg)
  {
    auto new_mode = static_cast<logic::DrivingMode>(msg->mode);
    if (new_mode == current_mode_) return;

    auto new_calc = logic::OdometryFactory::create(new_mode, track_width_m_, wheelbase_m_);
    {
      std::lock_guard<std::mutex> lock(calc_mutex_);
      // Transfer pose so there's no jump at mode switch
      new_calc->import_pose(calculator_->export_pose());
      calculator_ = std::move(new_calc);
      current_mode_ = new_mode;
    }
    RCLCPP_INFO(get_logger(), "Odometry mode → %s", calculator_->mode_name().c_str());
  }

  void on_reset(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr res)
  {
    std::lock_guard<std::mutex> lock(calc_mutex_);
    calculator_->reset();
    res->success = true;
    res->message = "Odometry reset";
  }

  void on_set_steer_mode(
    const mtt_interfaces::srv::SetSteerControlMode::Request::SharedPtr req,
    mtt_interfaces::srv::SetSteerControlMode::Response::SharedPtr res)
  {
    if (req->control_mode != "open_loop" && req->control_mode != "closed_loop") {
      res->success = false;
      res->message = "Invalid control_mode. Use 'open_loop' or 'closed_loop'";
      return;
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    steer_mode_ = req->control_mode;
    res->success = true;
    res->message = "Steer control mode set to " + steer_mode_;
    RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
  }
};

}  // namespace mtt

RCLCPP_COMPONENTS_REGISTER_NODE(mtt::MttOdometryNode)
