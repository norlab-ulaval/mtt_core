// ISAM2-based Factor Graph state estimator for MTT.
// Fuses: IMU (preintegrated), track odom, GPS position, GPS heading,
// LiDAR odom, and visual odom in SE(3).
//
// Key design: each sensor callback adds factors at its own rate.
// ISAM2 is updated at a configurable rate to produce the optimized state.
// IMU is pre-integrated between keyframes for computational efficiency.

#include <chrono>
#include <memory>
#include <mutex>
#include <deque>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/inference/Symbol.h>

#include "mtt_localization/state.hpp"

using namespace std::chrono_literals;
using gtsam::symbol_shorthand::X;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::B;

// WGS84 for GPS → local conversion
static constexpr double kEarthRadius = 6378137.0;
static constexpr double kDeg2Rad = M_PI / 180.0;

class FactorGraphNode : public rclcpp::Node {
public:
  FactorGraphNode() : Node("factor_graph_node") {
    declare_parameters();
    load_parameters();
    setup_isam2();
    setup_imu_preintegration();
    setup_publishers();
    setup_subscribers();

    // Optimization timer
    double rate = get_parameter("publish_rate").as_double();
    opt_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / rate),
        std::bind(&FactorGraphNode::optimize_and_publish, this));

    RCLCPP_INFO(get_logger(), "Factor Graph node started (ISAM2, SE(3))");
  }

private:
  // ─── Parameter declarations ────────────────────────────────────────
  void declare_parameters() {
    declare_parameter("use_imu_primary", true);
    declare_parameter("use_track_odom", true);
    declare_parameter("use_gps", true);
    declare_parameter("use_gps_heading", true);
    declare_parameter("use_lidar_odom", true);
    declare_parameter("use_visual_odom", false);

    declare_parameter("map_frame", "map");
    declare_parameter("odom_frame", "odom");
    declare_parameter("base_frame", "base_link");
    declare_parameter("publish_rate", 50.0);

    declare_parameter("isam2_relinearize_threshold", 0.1);
    declare_parameter("isam2_relinearize_skip", 10);

    // Noise
    declare_parameter("imu_accel_noise", 0.01);
    declare_parameter("imu_gyro_noise", 0.0001);
    declare_parameter("imu_accel_walk", 0.0001);
    declare_parameter("imu_gyro_walk", 1e-6);
    declare_parameter("odom_linear_noise", 0.1);
    declare_parameter("odom_angular_noise", 0.05);
    declare_parameter("gps_position_noise_xy", 1.0);
    declare_parameter("gps_position_noise_z", 2.0);
    declare_parameter("gps_heading_noise", 0.05);

    // GPS origin (first fix sets this)
    declare_parameter("gps_origin_lat", 0.0);
    declare_parameter("gps_origin_lon", 0.0);
    declare_parameter("gps_origin_alt", 0.0);
  }

  void load_parameters() {
    use_imu_ = get_parameter("use_imu_primary").as_bool();
    use_odom_ = get_parameter("use_track_odom").as_bool();
    use_gps_ = get_parameter("use_gps").as_bool();
    use_gps_heading_ = get_parameter("use_gps_heading").as_bool();
    use_lidar_odom_ = get_parameter("use_lidar_odom").as_bool();
    use_visual_odom_ = get_parameter("use_visual_odom").as_bool();

    map_frame_ = get_parameter("map_frame").as_string();
    odom_frame_ = get_parameter("odom_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();

    noise_.accel_noise_density = get_parameter("imu_accel_noise").as_double();
    noise_.gyro_noise_density = get_parameter("imu_gyro_noise").as_double();
    noise_.accel_random_walk = get_parameter("imu_accel_walk").as_double();
    noise_.gyro_random_walk = get_parameter("imu_gyro_walk").as_double();
    noise_.odom_linear_noise = get_parameter("odom_linear_noise").as_double();
    noise_.odom_angular_noise = get_parameter("odom_angular_noise").as_double();
    noise_.gps_position_noise_xy = get_parameter("gps_position_noise_xy").as_double();
    noise_.gps_position_noise_z = get_parameter("gps_position_noise_z").as_double();
    noise_.gps_heading_noise = get_parameter("gps_heading_noise").as_double();
  }

  // ─── ISAM2 setup ──────────────────────────────────────────────────
  void setup_isam2() {
    gtsam::ISAM2Params params;
    params.relinearizeThreshold =
        get_parameter("isam2_relinearize_threshold").as_double();
    params.relinearizeSkip =
        static_cast<int>(get_parameter("isam2_relinearize_skip").as_int());
    isam2_ = std::make_unique<gtsam::ISAM2>(params);

    // Initial state: identity pose, zero velocity, zero bias
    gtsam::Pose3 prior_pose = gtsam::Pose3::Identity();
    gtsam::Vector3 prior_vel = gtsam::Vector3::Zero();
    gtsam::imuBias::ConstantBias prior_bias;

    auto pose_noise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector6() << 0.1, 0.1, 0.1, 0.5, 0.5, 0.5).finished());
    auto vel_noise = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
    auto bias_noise = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3);

    graph_.addPrior(X(0), prior_pose, pose_noise);
    graph_.addPrior(V(0), prior_vel, vel_noise);
    graph_.addPrior(B(0), prior_bias, bias_noise);

    initial_values_.insert(X(0), prior_pose);
    initial_values_.insert(V(0), prior_vel);
    initial_values_.insert(B(0), prior_bias);

    current_state_.pose = prior_pose;
    current_state_.velocity = prior_vel;
    current_state_.imu_bias = prior_bias;
    current_state_.key_index = 0;

    RCLCPP_INFO(get_logger(), "ISAM2 initialized with identity prior");
  }

  void setup_imu_preintegration() {
    auto p = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(9.81);
    p->accelerometerCovariance =
        gtsam::I_3x3 * std::pow(noise_.accel_noise_density, 2);
    p->gyroscopeCovariance =
        gtsam::I_3x3 * std::pow(noise_.gyro_noise_density, 2);
    p->biasAccCovariance =
        gtsam::I_3x3 * std::pow(noise_.accel_random_walk, 2);
    p->biasOmegaCovariance =
        gtsam::I_3x3 * std::pow(noise_.gyro_random_walk, 2);
    p->integrationCovariance = gtsam::I_3x3 * 1e-8;

    imu_preint_ = std::make_unique<gtsam::PreintegratedCombinedMeasurements>(
        p, current_state_.imu_bias);
  }

  // ─── Publishers ────────────────────────────────────────────────────
  void setup_publishers() {
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
        "localization/odom", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  // ─── Subscribers ───────────────────────────────────────────────────
  void setup_subscribers() {
    if (use_imu_) {
      imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
          "/mti100/data", rclcpp::SensorDataQoS(),
          std::bind(&FactorGraphNode::imu_callback, this, std::placeholders::_1));
    }

    if (use_odom_) {
      odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
          "/mtt/odom", 10,
          std::bind(&FactorGraphNode::odom_callback, this, std::placeholders::_1));
    }

    if (use_gps_) {
      gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
          "/gps_left/fix", 10,
          std::bind(&FactorGraphNode::gps_callback, this, std::placeholders::_1));
    }

    if (use_gps_heading_) {
      heading_sub_ = create_subscription<geometry_msgs::msg::QuaternionStamped>(
          "gps/heading", 10,
          std::bind(&FactorGraphNode::heading_callback, this, std::placeholders::_1));
    }

    if (use_lidar_odom_) {
      lidar_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
          "/icp_odom", 10,
          std::bind(&FactorGraphNode::lidar_odom_callback, this, std::placeholders::_1));
    }

    if (use_visual_odom_) {
      visual_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
          "/zed/zed_node/odom", 10,
          std::bind(&FactorGraphNode::visual_odom_callback, this, std::placeholders::_1));
    }
  }

  // ─── IMU callback: accumulate for preintegration ───────────────────
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mtx_);

    double t = rclcpp::Time(msg->header.stamp).seconds();
    if (last_imu_t_ < 0) {
      last_imu_t_ = t;
      return;
    }
    double dt = t - last_imu_t_;
    if (dt <= 0 || dt > 1.0) {
      last_imu_t_ = t;
      return;
    }

    gtsam::Vector3 acc(msg->linear_acceleration.x,
                       msg->linear_acceleration.y,
                       msg->linear_acceleration.z);
    gtsam::Vector3 gyro(msg->angular_velocity.x,
                        msg->angular_velocity.y,
                        msg->angular_velocity.z);

    imu_preint_->integrateMeasurement(acc, gyro, dt);
    last_imu_t_ = t;
    imu_data_count_++;
  }

  // ─── Track odometry callback ──────────────────────────────────────
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    pending_odom_ = *msg;
    has_pending_odom_ = true;
  }

  // ─── GPS position callback ────────────────────────────────────────
  void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    if (msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) return;

    std::lock_guard<std::mutex> lock(mtx_);

    // Set GPS origin on first valid fix
    if (!gps_origin_set_) {
      gps_origin_lat_ = msg->latitude;
      gps_origin_lon_ = msg->longitude;
      gps_origin_alt_ = msg->altitude;
      gps_origin_set_ = true;
      RCLCPP_INFO(get_logger(), "GPS origin set: lat=%.7f lon=%.7f alt=%.2f",
                  gps_origin_lat_, gps_origin_lon_, gps_origin_alt_);
    }

    pending_gps_ = *msg;
    has_pending_gps_ = true;
  }

  // ─── GPS heading callback ─────────────────────────────────────────
  void heading_callback(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    pending_heading_ = *msg;
    has_pending_heading_ = true;
  }

  // ─── LiDAR odometry callback ──────────────────────────────────────
  void lidar_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mtx_);

    if (!has_last_lidar_odom_) {
      last_lidar_odom_ = *msg;
      has_last_lidar_odom_ = true;
      return;
    }

    // Compute relative transform
    pending_lidar_odom_ = *msg;
    has_pending_lidar_odom_ = true;
  }

  // ─── Visual odometry callback ─────────────────────────────────────
  void visual_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mtx_);

    if (!has_last_visual_odom_) {
      last_visual_odom_ = *msg;
      has_last_visual_odom_ = true;
      return;
    }

    pending_visual_odom_ = *msg;
    has_pending_visual_odom_ = true;
  }

  // ─── Convert GPS to local ENU ─────────────────────────────────────
  gtsam::Point3 gps_to_local(double lat, double lon, double alt) const {
    double lat_ref = gps_origin_lat_ * kDeg2Rad;
    double m_per_deg_lat = kEarthRadius * kDeg2Rad;
    double m_per_deg_lon = kEarthRadius * kDeg2Rad * std::cos(lat_ref);

    double x = (lat - gps_origin_lat_) * m_per_deg_lat;   // north
    double y = (lon - gps_origin_lon_) * m_per_deg_lon;   // east
    double z = alt - gps_origin_alt_;                      // up
    return gtsam::Point3(x, y, z);
  }

  // ─── Odometry message to Pose3 ────────────────────────────────────
  gtsam::Pose3 odom_to_pose3(const nav_msgs::msg::Odometry& msg) const {
    auto& p = msg.pose.pose.position;
    auto& q = msg.pose.pose.orientation;
    return gtsam::Pose3(
        gtsam::Rot3::Quaternion(q.w, q.x, q.y, q.z),
        gtsam::Point3(p.x, p.y, p.z));
  }

  // ─── Main optimization + publish ──────────────────────────────────
  void optimize_and_publish() {
    std::lock_guard<std::mutex> lock(mtx_);

    // Only proceed if we have enough IMU data for a new keyframe
    if (imu_data_count_ < 10) return;

    uint64_t prev_key = current_state_.key_index;
    uint64_t curr_key = prev_key + 1;

    // IMU factor: always add if we have preintegrated data
    if (use_imu_ && imu_data_count_ > 0) {
      auto imu_factor = gtsam::CombinedImuFactor(
          X(prev_key), V(prev_key),
          X(curr_key), V(curr_key),
          B(prev_key), B(curr_key),
          *imu_preint_);
      graph_.add(imu_factor);
    }

    // Predict new state from IMU
    auto predicted = imu_preint_->predict(
        gtsam::NavState(current_state_.pose, current_state_.velocity),
        current_state_.imu_bias);

    initial_values_.insert(X(curr_key), predicted.pose());
    initial_values_.insert(V(curr_key), predicted.velocity());
    initial_values_.insert(B(curr_key), current_state_.imu_bias);

    // GPS position factor
    if (has_pending_gps_ && gps_origin_set_) {
      auto local = gps_to_local(pending_gps_.latitude,
                                pending_gps_.longitude,
                                pending_gps_.altitude);
      // Adaptative noise based on fix quality
      double noise_xy = noise_.gps_position_noise_xy;
      double noise_z = noise_.gps_position_noise_z;
      if (pending_gps_.status.status == sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX) {
        noise_xy = 0.02;  // RTK Fix: 2cm
        noise_z = 0.04;
      }
      auto gps_noise = gtsam::noiseModel::Diagonal::Sigmas(
          gtsam::Vector3(noise_xy, noise_xy, noise_z));
      graph_.add(gtsam::GPSFactor(X(curr_key), local, gps_noise));
      has_pending_gps_ = false;
    }

    // GPS heading factor — add as a prior on rotation
    if (has_pending_heading_) {
      auto& q = pending_heading_.quaternion;
      gtsam::Rot3 heading_rot = gtsam::Rot3::Quaternion(q.w, q.x, q.y, q.z);
      // Only constrain yaw, not roll/pitch (GPS can't measure those)
      auto heading_noise = gtsam::noiseModel::Diagonal::Sigmas(
          (gtsam::Vector6() << 99.0, 99.0, noise_.gps_heading_noise,
                               99.0, 99.0, 99.0).finished());
      gtsam::Pose3 heading_pose(heading_rot, predicted.pose().translation());
      graph_.addPrior(X(curr_key), heading_pose, heading_noise);
      has_pending_heading_ = false;
    }

    // LiDAR odometry: relative constraint between keyframes
    if (has_pending_lidar_odom_ && has_last_lidar_odom_) {
      gtsam::Pose3 prev_lo = odom_to_pose3(last_lidar_odom_);
      gtsam::Pose3 curr_lo = odom_to_pose3(pending_lidar_odom_);
      gtsam::Pose3 delta = prev_lo.between(curr_lo);

      auto lo_noise = gtsam::noiseModel::Diagonal::Sigmas(noise_.lidar_odom_noise);
      graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
          X(prev_key), X(curr_key), delta, lo_noise));

      last_lidar_odom_ = pending_lidar_odom_;
      has_pending_lidar_odom_ = false;
    }

    // Visual odometry: relative constraint
    if (has_pending_visual_odom_ && has_last_visual_odom_) {
      gtsam::Pose3 prev_vo = odom_to_pose3(last_visual_odom_);
      gtsam::Pose3 curr_vo = odom_to_pose3(pending_visual_odom_);
      gtsam::Pose3 delta = prev_vo.between(curr_vo);

      auto vo_noise = gtsam::noiseModel::Diagonal::Sigmas(noise_.visual_odom_noise);
      graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
          X(prev_key), X(curr_key), delta, vo_noise));

      last_visual_odom_ = pending_visual_odom_;
      has_pending_visual_odom_ = false;
    }

    // ─── Run ISAM2 update ───────────────────────────────────────────
    try {
      isam2_->update(graph_, initial_values_);
      auto result = isam2_->calculateEstimate();

      current_state_.pose = result.at<gtsam::Pose3>(X(curr_key));
      current_state_.velocity = result.at<gtsam::Vector3>(V(curr_key));
      current_state_.imu_bias =
          result.at<gtsam::imuBias::ConstantBias>(B(curr_key));
      current_state_.key_index = curr_key;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "ISAM2 optimization failed: %s", e.what());
      // Keep previous state, don't crash
      current_state_.key_index = curr_key;
    }

    // Clear for next iteration
    graph_.resize(0);
    initial_values_.clear();

    // Reset IMU preintegration with updated bias
    imu_preint_->resetIntegrationAndSetBias(current_state_.imu_bias);
    imu_data_count_ = 0;

    // ─── Publish ────────────────────────────────────────────────────
    publish_odometry();
    publish_tf();
  }

  void publish_odometry() {
    nav_msgs::msg::Odometry msg;
    msg.header.stamp = now();
    msg.header.frame_id = map_frame_;
    msg.child_frame_id = base_frame_;

    auto& t = current_state_.pose.translation();
    auto q = current_state_.pose.rotation().toQuaternion();

    msg.pose.pose.position.x = t.x();
    msg.pose.pose.position.y = t.y();
    msg.pose.pose.position.z = t.z();
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    msg.pose.pose.orientation.w = q.w();

    msg.twist.twist.linear.x = current_state_.velocity.x();
    msg.twist.twist.linear.y = current_state_.velocity.y();
    msg.twist.twist.linear.z = current_state_.velocity.z();

    odom_pub_->publish(msg);
  }

  void publish_tf() {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now();
    tf.header.frame_id = map_frame_;
    tf.child_frame_id = odom_frame_;

    auto& t = current_state_.pose.translation();
    auto q = current_state_.pose.rotation().toQuaternion();

    tf.transform.translation.x = t.x();
    tf.transform.translation.y = t.y();
    tf.transform.translation.z = t.z();
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(tf);
  }

  // ─── Members ──────────────────────────────────────────────────────
  std::mutex mtx_;

  // ISAM2
  std::unique_ptr<gtsam::ISAM2> isam2_;
  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values initial_values_;

  // IMU preintegration
  std::unique_ptr<gtsam::PreintegratedCombinedMeasurements> imu_preint_;
  double last_imu_t_{-1.0};
  int imu_data_count_{0};

  // Current optimized state
  mtt_loc::NavState current_state_;
  mtt_loc::SensorNoiseParams noise_;

  // Pending sensor data
  sensor_msgs::msg::NavSatFix pending_gps_;
  geometry_msgs::msg::QuaternionStamped pending_heading_;
  nav_msgs::msg::Odometry pending_odom_;
  nav_msgs::msg::Odometry pending_lidar_odom_;
  nav_msgs::msg::Odometry pending_visual_odom_;
  nav_msgs::msg::Odometry last_lidar_odom_;
  nav_msgs::msg::Odometry last_visual_odom_;

  bool has_pending_gps_{false};
  bool has_pending_heading_{false};
  bool has_pending_odom_{false};
  bool has_pending_lidar_odom_{false};
  bool has_pending_visual_odom_{false};
  bool has_last_lidar_odom_{false};
  bool has_last_visual_odom_{false};

  // GPS origin
  double gps_origin_lat_{0.0};
  double gps_origin_lon_{0.0};
  double gps_origin_alt_{0.0};
  bool gps_origin_set_{false};

  // Feature flags
  bool use_imu_, use_odom_, use_gps_, use_gps_heading_;
  bool use_lidar_odom_, use_visual_odom_;

  // Frame IDs
  std::string map_frame_, odom_frame_, base_frame_;

  // ROS interfaces
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr heading_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lidar_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr visual_odom_sub_;

  rclcpp::TimerBase::SharedPtr opt_timer_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FactorGraphNode>());
  rclcpp::shutdown();
  return 0;
}
