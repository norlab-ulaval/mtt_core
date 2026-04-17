// MTT-154 Odometry Node — ROS 2 Component
// Replaces mtt_odometry_manager.py. Subscribes mtt_tachometer, runs the
// multi-mode odometry strategy, publishes nav_msgs/Odometry + TF.

#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <mtt_msgs/msg/mtt_tachometer_data.hpp>
#include <mtt_msgs/msg/mtt_driving_mode.hpp>
#include <mtt_interfaces/srv/set_steer_control_mode.hpp>
#include <std_srvs/srv/trigger.hpp>

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
    base_frame_       = declare_parameter("base_frame",       std::string("base_link"));
    broadcast_tf_     = declare_parameter("broadcast_tf",     true);
    track_width_m_    = declare_parameter("track_width_m",    VehicleParams::track_width);
    wheelbase_m_      = declare_parameter("wheelbase_m",      VehicleParams::total_wheelbase());
    steer_mode_       = declare_parameter("steer_control_mode", std::string("open_loop"));
    pivot_turn_       = declare_parameter("pivot_turn_enabled", false);
    min_speed_turn_   = declare_parameter("min_turn_speed_ms", 0.03);
    max_articulation_rad_ = declare_parameter("max_articulation_deg",
        VehicleParams::max_articulation_deg) * M_PI / 180.0;
    yaw_slip_factor_  = declare_parameter("yaw_slip_factor",  1.0);
    wrap_threshold_m_ = declare_parameter("wrap_reset_threshold_m", 1000.0);

    // ── Initial odometry mode ─────────────────────────────────────────
    calculator_ = logic::OdometryFactory::create(
      logic::DrivingMode::SingleTrailer, track_width_m_, wheelbase_m_);

    // ── Publishers ────────────────────────────────────────────────────
    odom_pub_        = create_publisher<nav_msgs::msg::Odometry>("mtt_odometry", 10);
    articulation_pub_ = create_publisher<std_msgs::msg::Float64>("mtt_articulation_angle", 10);

    // ── Subscribers ────────────────────────────────────────────────────────────
    tacho_sub_ = create_subscription<mtt_msgs::msg::MttTachometerData>(
      "mtt_tachometer", rclcpp::SensorDataQoS(),
      [this](const mtt_msgs::msg::MttTachometerData::SharedPtr msg){ on_tachometer(msg); });
    mode_sub_ = create_subscription<mtt_msgs::msg::MttDrivingMode>(
      "mtt_driving_mode", 10,
      [this](const mtt_msgs::msg::MttDrivingMode::SharedPtr msg){ on_mode_change(msg); });
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "cmd_vel/pid", 10,
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
      // Publish identity TF at 10 Hz so ICP mapper always has a valid transform,
      // even when the tachometer is dead (encoder failure, cold start, etc.).
      tf_fallback_timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        [this]() {
          if (!broadcast_tf_ || !tf_broadcaster_) return;
          // Only publish if no tachometer update in the last 200 ms
          auto now_tp = std::chrono::steady_clock::now();
          if (now_tp - last_tacho_time_ < std::chrono::milliseconds(200)) return;
          geometry_msgs::msg::TransformStamped tf;
          tf.header.stamp    = now();
          tf.header.frame_id = odom_frame_;
          tf.child_frame_id  = base_frame_;
          {
            std::lock_guard<std::mutex> lock(calc_mutex_);
            auto pose = calculator_->export_pose();
            tf.transform.translation.x = pose.x;
            tf.transform.translation.y = pose.y;
            tf.transform.rotation.z = std::sin(pose.heading / 2.0);
            tf.transform.rotation.w = std::cos(pose.heading / 2.0);
          }
          tf_broadcaster_->sendTransform(tf);
        });
    }

    RCLCPP_INFO(get_logger(), "MttOdometryNode started (mode=SingleTrailer, steer=%s)",
                steer_mode_.c_str());
  }

private:
  // ── State ─────────────────────────────────────────────────────────
  std::string odom_frame_, base_frame_, steer_mode_;
  bool     broadcast_tf_, pivot_turn_;
  double   track_width_m_, wheelbase_m_;
  double   max_articulation_rad_{VehicleParams::max_articulation_rad};
  double   min_speed_turn_, yaw_slip_factor_, wrap_threshold_m_;
  double   current_angular_vel_{0.0};
  std::chrono::steady_clock::time_point last_tacho_time_{};

  std::mutex calc_mutex_;
  std::unique_ptr<logic::IOdometryCalculator> calculator_;
  logic::DrivingMode current_mode_{logic::DrivingMode::SingleTrailer};

  // ── ROS I/O ───────────────────────────────────────────────────────
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr  articulation_pub_;
  rclcpp::Subscription<mtt_msgs::msg::MttTachometerData>::SharedPtr tacho_sub_;
  rclcpp::Subscription<mtt_msgs::msg::MttDrivingMode>::SharedPtr    mode_sub_;
  rclcpp::TimerBase::SharedPtr tf_fallback_timer_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr  reset_srv_;
  rclcpp::Service<mtt_interfaces::srv::SetSteerControlMode>::SharedPtr steer_mode_srv_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // ── Callbacks ─────────────────────────────────────────────────────
  void on_cmd_vel(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    current_angular_vel_ = msg->twist.angular.z;
  }

  void on_tachometer(const mtt_msgs::msg::MttTachometerData::SharedPtr msg)
  {
    last_tacho_time_ = std::chrono::steady_clock::now();
    const double speed_ms   = msg->speed_ms;
    const double steer_cmd  = msg->steer_cmd;
    const int    dir_sign   = (msg->direction == "Reverse") ? -1 : 1;

    // Compute effective angular velocity
    double eff_ang = 0.0;
    if (steer_mode_ == "closed_loop") {
      double phi = std::clamp(steer_cmd * max_articulation_rad_,
                              -max_articulation_rad_,
                               max_articulation_rad_);
      eff_ang = speed_ms * std::tan(phi) / std::max(wheelbase_m_, 1e-6);
    } else {
      eff_ang = steer_cmd * VehicleParams::max_yaw_rate_rad_s;
    }

    // Suppress yaw when near-stationary (no pivot turns unless enabled)
    if (!pivot_turn_ && std::abs(speed_ms) < min_speed_turn_) eff_ang = 0.0;
    eff_ang = std::clamp(eff_ang * yaw_slip_factor_,
                         -VehicleParams::max_yaw_rate_rad_s,
                          VehicleParams::max_yaw_rate_rad_s);

    logic::OdometryInput input;
    input.distance_km      = msg->distance_km;
    input.speed_ms         = speed_ms * dir_sign;
    input.steer_cmd        = steer_cmd;
    input.angular_velocity = eff_ang;
    input.direction_sign   = dir_sign;
    input.dt               = 0.02;  // nominal; TODO: derive from stamp delta

    logic::OdometryOutput out;
    {
      std::lock_guard<std::mutex> lock(calc_mutex_);
      out = calculator_->update(input);
    }

    // Build and publish nav_msgs/Odometry
    nav_msgs::msg::Odometry odom;
    odom.header.stamp    = now();
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
    if (current_mode_ == logic::DrivingMode::SingleTrailer) {
      std_msgs::msg::Float64 artic;
      artic.data = out.articulation_angle;
      articulation_pub_->publish(artic);
    }

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
    steer_mode_ = req->control_mode;
    res->success = true;
    res->message = "Steer control mode set to " + steer_mode_;
    RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
  }
};

}  // namespace mtt

RCLCPP_COMPONENTS_REGISTER_NODE(mtt::MttOdometryNode)
