// ISAM2-based Factor Graph state estimator for MTT.
// Fuses: IMU (preintegrated), track odom, GPS position, GPS heading,
// LiDAR odom, visual odom, and articulation (encoder + trailer LiDAR pose).
//
// Extended state at keyframe k:
//   X(k) = Pose3   — tractor pose in map
//   V(k) = Vector3 — tractor velocity
//   B(k) = ImuBias — IMU bias
//   H(k) = double  — hitch yaw angle φ (articulation, symbol 'h')
//
// Articulation factors:
//   1. EncoderFactor     : PriorFactor<double> on H(k)    from hardware encoder
//   2. ArticulationDyn   : BetweenFactor<double> H(k-1)→H(k)  (random-walk prior)
//   3. TrailerPoseFactor : custom NoiseModelFactor1<double> from trailer/pose LiDAR
//
// Bayesian mutual aid (no circular dependency):
//   • Tractor sensors (IMU/GPS/odom) constrain X(k) independently.
//   • Encoder + trailer LiDAR constrain H(k) independently.
//   • The joint posterior p(X(k), H(k) | all_z) is the product of independent
//     likelihoods — no measurement appears twice.
//   • The off-diagonal block Σ_{X,H} in the joint marginal encodes the
//     "corroboration": once H(k) is well-known, X(k)'s heading uncertainty
//     is further reduced through the ISAM2 Bayes-tree marginalisation.
//
// Publishes:
//   localization/odom              — nav_msgs/Odometry  (map frame, with Σ from ISAM2)
//   localization/articulation_angle — std_msgs/Float64   (optimised φ for downstream)

#include <chrono>
#include <memory>
#include <mutex>
#include <deque>
#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

#include "mtt_msgs/msg/mtt_articulation_state.hpp"

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
#include "mtt_localization/trailer_pose_factor.hpp"

using namespace std::chrono_literals;
using gtsam::symbol_shorthand::X;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::B;

/// Hitch yaw angle symbol — letter 'h' avoids collision with X/V/B
inline gtsam::Key H_key(uint64_t i) { return gtsam::Symbol('h', i); }

// WGS84
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

    double rate = get_parameter("publish_rate").as_double();
    opt_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / rate),
        std::bind(&FactorGraphNode::optimize_and_publish, this));

    RCLCPP_INFO(get_logger(), "Factor Graph node started (ISAM2, SE(3) + φ articulation)");
  }

private:
  // ─── Parameter declarations ───────────────────────────────────────
  void declare_parameters() {
    declare_parameter("use_imu_primary", true);
    declare_parameter("use_track_odom", true);
    declare_parameter("use_gps", true);
    declare_parameter("use_gps_heading", true);
    declare_parameter("use_lidar_odom", true);
    declare_parameter("use_visual_odom", false);
    declare_parameter("use_articulation", true);
    declare_parameter("use_trailer_pose", true);

    declare_parameter("map_frame", "map");
    declare_parameter("odom_frame", "odom");
    declare_parameter("base_frame", "base_footprint");
    declare_parameter("imu_topic", "mti100/data");
    declare_parameter("track_odom_topic", "mtt_odometry");
    declare_parameter("gps_fix_topic", "gps_left/fix");
    declare_parameter("gps_heading_topic", "gps/heading");
    declare_parameter("lidar_odom_topic", "mapping/icp_odom");
    declare_parameter("visual_odom_topic", "zed/zed_node/odom");
    declare_parameter("articulation_topic", "/mtt/articulation_state");
    declare_parameter("trailer_pose_topic", "/trailer/pose");
    declare_parameter("trailer_confidence_topic", "/trailer/pose_confidence");
    declare_parameter("publish_rate", 50.0);
    declare_parameter("min_imu_samples_per_update", 10);
    declare_parameter("extract_covariance", true);

    declare_parameter("isam2_relinearize_threshold", 0.1);
    declare_parameter("isam2_relinearize_skip", 10);

    // IMU noise
    declare_parameter("imu_accel_noise", 0.01);
    declare_parameter("imu_gyro_noise", 0.0001);
    declare_parameter("imu_accel_walk", 0.0001);
    declare_parameter("imu_gyro_walk", 1e-6);
    // Odometry noise
    declare_parameter("odom_linear_noise", 0.1);
    declare_parameter("odom_angular_noise", 0.05);
    // GPS noise
    declare_parameter("gps_position_noise_xy", 1.0);
    declare_parameter("gps_position_noise_z", 2.0);
    declare_parameter("gps_heading_noise", 0.05);
    // GPS origin
    declare_parameter("gps_origin_lat", 0.0);
    declare_parameter("gps_origin_lon", 0.0);
    declare_parameter("gps_origin_alt", 0.0);
    // Articulation noise
    declare_parameter("phi_sigma_hardware", 0.008);    // rad — encoder fresh
    declare_parameter("phi_sigma_model", 0.035);       // rad — model/stale
    declare_parameter("phi_sigma_dynamics", 0.015);    // rad — random-walk σ per keyframe
    declare_parameter("phi_prior_sigma", 0.5);         // rad — initial prior on φ
    // Trailer LiDAR factor noise
    declare_parameter("trailer_sigma_rot", 0.04);      // rad  base rotation noise
    declare_parameter("trailer_sigma_trans", 0.10);    // m    base translation noise
    declare_parameter("trailer_min_confidence", 0.20); // [0,1] skip below this
  }

  void load_parameters() {
    use_imu_ = get_parameter("use_imu_primary").as_bool();
    use_odom_ = get_parameter("use_track_odom").as_bool();
    use_gps_ = get_parameter("use_gps").as_bool();
    use_gps_heading_ = get_parameter("use_gps_heading").as_bool();
    use_lidar_odom_ = get_parameter("use_lidar_odom").as_bool();
    use_visual_odom_ = get_parameter("use_visual_odom").as_bool();
    use_articulation_ = get_parameter("use_articulation").as_bool();
    use_trailer_pose_ = get_parameter("use_trailer_pose").as_bool();
    extract_covariance_ = get_parameter("extract_covariance").as_bool();

    map_frame_ = get_parameter("map_frame").as_string();
    odom_frame_ = get_parameter("odom_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    imu_topic_ = get_parameter("imu_topic").as_string();
    track_odom_topic_ = get_parameter("track_odom_topic").as_string();
    gps_fix_topic_ = get_parameter("gps_fix_topic").as_string();
    gps_heading_topic_ = get_parameter("gps_heading_topic").as_string();
    lidar_odom_topic_ = get_parameter("lidar_odom_topic").as_string();
    visual_odom_topic_ = get_parameter("visual_odom_topic").as_string();
    articulation_topic_ = get_parameter("articulation_topic").as_string();
    trailer_pose_topic_ = get_parameter("trailer_pose_topic").as_string();
    trailer_confidence_topic_ = get_parameter("trailer_confidence_topic").as_string();
    min_imu_samples_per_update_ =
        static_cast<int>(get_parameter("min_imu_samples_per_update").as_int());

    noise_.accel_noise_density = get_parameter("imu_accel_noise").as_double();
    noise_.gyro_noise_density = get_parameter("imu_gyro_noise").as_double();
    noise_.accel_random_walk = get_parameter("imu_accel_walk").as_double();
    noise_.gyro_random_walk = get_parameter("imu_gyro_walk").as_double();
    noise_.odom_linear_noise = get_parameter("odom_linear_noise").as_double();
    noise_.odom_angular_noise = get_parameter("odom_angular_noise").as_double();
    noise_.gps_position_noise_xy = get_parameter("gps_position_noise_xy").as_double();
    noise_.gps_position_noise_z = get_parameter("gps_position_noise_z").as_double();
    noise_.gps_heading_noise = get_parameter("gps_heading_noise").as_double();
    noise_.phi_sigma_hardware = get_parameter("phi_sigma_hardware").as_double();
    noise_.phi_sigma_model = get_parameter("phi_sigma_model").as_double();
    noise_.phi_sigma_dynamics = get_parameter("phi_sigma_dynamics").as_double();
    noise_.phi_prior_sigma = get_parameter("phi_prior_sigma").as_double();
    noise_.trailer_sigma_rot = get_parameter("trailer_sigma_rot").as_double();
    noise_.trailer_sigma_trans = get_parameter("trailer_sigma_trans").as_double();
    trailer_min_confidence_ = get_parameter("trailer_min_confidence").as_double();
  }

  // ─── ISAM2 setup ─────────────────────────────────────────────────
  void setup_isam2() {
    gtsam::ISAM2Params params;
    params.relinearizeThreshold =
        get_parameter("isam2_relinearize_threshold").as_double();
    params.relinearizeSkip =
        static_cast<int>(get_parameter("isam2_relinearize_skip").as_int());
    isam2_ = std::make_unique<gtsam::ISAM2>(params);

    // ── Initial tractor state ──
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

    // ── Initial articulation state ──
    if (use_articulation_) {
      const double prior_phi = 0.0;  // straight ahead
      auto phi_prior_noise = gtsam::noiseModel::Isotropic::Sigma(
          1, noise_.phi_prior_sigma);
      graph_.addPrior(H_key(0), prior_phi, phi_prior_noise);
      initial_values_.insert(H_key(0), prior_phi);
      current_state_.trailer_angle = prior_phi;
    }

    current_state_.pose = prior_pose;
    current_state_.velocity = prior_vel;
    current_state_.imu_bias = prior_bias;
    current_state_.key_index = 0;

    RCLCPP_INFO(get_logger(), "ISAM2 initialised (tractor SE(3) + articulation φ)");
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

  // ─── Publishers ──────────────────────────────────────────────────
  void setup_publishers() {
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("localization/odom", 10);
    articulation_pub_ = create_publisher<std_msgs::msg::Float64>(
        "localization/articulation_angle", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  // ─── Subscribers ─────────────────────────────────────────────────
  void setup_subscribers() {
    if (use_imu_) {
      imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
          imu_topic_, rclcpp::SensorDataQoS(),
          std::bind(&FactorGraphNode::imu_callback, this, std::placeholders::_1));
    }
    if (use_odom_) {
      odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
          track_odom_topic_, 10,
          std::bind(&FactorGraphNode::odom_callback, this, std::placeholders::_1));
    }
    if (use_gps_) {
      gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
          gps_fix_topic_, 10,
          std::bind(&FactorGraphNode::gps_callback, this, std::placeholders::_1));
    }
    if (use_gps_heading_) {
      heading_sub_ = create_subscription<geometry_msgs::msg::QuaternionStamped>(
          gps_heading_topic_, 10,
          std::bind(&FactorGraphNode::heading_callback, this, std::placeholders::_1));
    }
    if (use_lidar_odom_) {
      lidar_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
          lidar_odom_topic_, 10,
          std::bind(&FactorGraphNode::lidar_odom_callback, this, std::placeholders::_1));
    }
    if (use_visual_odom_) {
      visual_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
          visual_odom_topic_, 10,
          std::bind(&FactorGraphNode::visual_odom_callback, this, std::placeholders::_1));
    }
    if (use_articulation_) {
      articulation_sub_ = create_subscription<mtt_msgs::msg::MttArticulationState>(
          articulation_topic_, rclcpp::SensorDataQoS(),
          std::bind(&FactorGraphNode::articulation_callback, this, std::placeholders::_1));
    }
    if (use_articulation_ && use_trailer_pose_) {
      trailer_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
          trailer_pose_topic_, rclcpp::SensorDataQoS(),
          std::bind(&FactorGraphNode::trailer_pose_callback, this, std::placeholders::_1));
      trailer_confidence_sub_ = create_subscription<std_msgs::msg::Float64>(
          trailer_confidence_topic_, rclcpp::SensorDataQoS(),
          std::bind(&FactorGraphNode::trailer_confidence_callback, this, std::placeholders::_1));
    }

    RCLCPP_INFO(get_logger(),
        "Subscriptions: imu=%d odom=%d gps=%d heading=%d lidar_odom=%d "
        "articulation=%d trailer_pose=%d",
        use_imu_, use_odom_, use_gps_, use_gps_heading_, use_lidar_odom_,
        use_articulation_, use_articulation_ && use_trailer_pose_);
  }

  // ─── Sensor callbacks ────────────────────────────────────────────
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    double t = rclcpp::Time(msg->header.stamp).seconds();
    if (last_imu_t_ < 0) { last_imu_t_ = t; return; }
    double dt = t - last_imu_t_;
    if (dt <= 0 || dt > 1.0) { last_imu_t_ = t; return; }

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

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    latest_odom_ = *msg;
    has_latest_odom_ = true;
    if (!has_last_odom_) { last_odom_ = *msg; has_last_odom_ = true; return; }
    pending_odom_ = *msg;
    has_pending_odom_ = true;
  }

  void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    if (msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) return;
    std::lock_guard<std::mutex> lock(mtx_);
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

  void heading_callback(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    pending_heading_ = *msg;
    has_pending_heading_ = true;
  }

  void lidar_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!has_last_lidar_odom_) {
      last_lidar_odom_ = *msg; has_last_lidar_odom_ = true; return;
    }
    pending_lidar_odom_ = *msg;
    has_pending_lidar_odom_ = true;
  }

  void visual_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!has_last_visual_odom_) {
      last_visual_odom_ = *msg; has_last_visual_odom_ = true; return;
    }
    pending_visual_odom_ = *msg;
    has_pending_visual_odom_ = true;
  }

  void articulation_callback(
      const mtt_msgs::msg::MttArticulationState::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    pending_articulation_ = *msg;
    has_pending_articulation_ = true;
  }

  void trailer_pose_callback(
      const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    pending_trailer_pose_ = *msg;
    has_pending_trailer_pose_ = true;
  }

  void trailer_confidence_callback(
      const std_msgs::msg::Float64::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    latest_trailer_confidence_ = msg->data;
  }

  // ─── Helpers ─────────────────────────────────────────────────────
  gtsam::Point3 gps_to_local(double lat, double lon, double alt) const {
    double lat_ref = gps_origin_lat_ * kDeg2Rad;
    double m_per_deg_lat = kEarthRadius * kDeg2Rad;
    double m_per_deg_lon = kEarthRadius * kDeg2Rad * std::cos(lat_ref);
    return {(lat - gps_origin_lat_) * m_per_deg_lat,
            (lon - gps_origin_lon_) * m_per_deg_lon,
            alt - gps_origin_alt_};
  }

  gtsam::Pose3 odom_to_pose3(const nav_msgs::msg::Odometry & msg) const {
    auto & p = msg.pose.pose.position;
    auto & q = msg.pose.pose.orientation;
    return {gtsam::Rot3::Quaternion(q.w, q.x, q.y, q.z),
            gtsam::Point3(p.x, p.y, p.z)};
  }

  gtsam::Pose3 stamped_to_pose3(const geometry_msgs::msg::PoseStamped & msg) const {
    auto & p = msg.pose.position;
    auto & q = msg.pose.orientation;
    return {gtsam::Rot3::Quaternion(q.w, q.x, q.y, q.z),
            gtsam::Point3(p.x, p.y, p.z)};
  }

  // ─── Main optimisation loop ───────────────────────────────────────
  void optimize_and_publish() {
    std::lock_guard<std::mutex> lock(mtx_);

    const bool has_pending =
        has_pending_odom_ || has_pending_gps_ || has_pending_heading_ ||
        has_pending_lidar_odom_ || has_pending_visual_odom_ ||
        has_pending_articulation_ || has_pending_trailer_pose_;

    if (use_imu_) {
      if (imu_data_count_ < min_imu_samples_per_update_) return;
    } else if (!has_pending) {
      return;
    }

    const uint64_t prev_key = current_state_.key_index;
    const uint64_t curr_key = prev_key + 1;

    // ── IMU preintegration factor ───────────────────────────────────
    if (use_imu_ && imu_data_count_ > 0) {
      graph_.add(gtsam::CombinedImuFactor(
          X(prev_key), V(prev_key), X(curr_key), V(curr_key),
          B(prev_key), B(curr_key), *imu_preint_));
    }

    // Predict tractor state from IMU
    const auto predicted = (use_imu_ && imu_data_count_ > 0)
        ? imu_preint_->predict(
              gtsam::NavState(current_state_.pose, current_state_.velocity),
              current_state_.imu_bias)
        : gtsam::NavState(current_state_.pose, current_state_.velocity);

    initial_values_.insert(X(curr_key), predicted.pose());
    initial_values_.insert(V(curr_key), predicted.velocity());
    initial_values_.insert(B(curr_key), current_state_.imu_bias);

    // ── GPS position ─────────────────────────────────────────────────
    if (has_pending_gps_ && gps_origin_set_) {
      auto local = gps_to_local(pending_gps_.latitude,
                                pending_gps_.longitude,
                                pending_gps_.altitude);
      double noise_xy = noise_.gps_position_noise_xy;
      double noise_z  = noise_.gps_position_noise_z;
      if (pending_gps_.status.status ==
          sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX) {
        noise_xy = 0.02; noise_z = 0.04;
      }
      graph_.add(gtsam::GPSFactor(X(curr_key), local,
          gtsam::noiseModel::Diagonal::Sigmas(
              gtsam::Vector3(noise_xy, noise_xy, noise_z))));
      has_pending_gps_ = false;
    }

    // ── GPS heading ──────────────────────────────────────────────────
    if (has_pending_heading_) {
      auto & q = pending_heading_.quaternion;
      auto heading_noise = gtsam::noiseModel::Diagonal::Sigmas(
          (gtsam::Vector6() << 99.0, 99.0, noise_.gps_heading_noise,
                               99.0, 99.0, 99.0).finished());
      graph_.addPrior(X(curr_key),
          gtsam::Pose3(gtsam::Rot3::Quaternion(q.w, q.x, q.y, q.z),
                       predicted.pose().translation()),
          heading_noise);
      has_pending_heading_ = false;
    }

    // ── Track odometry ───────────────────────────────────────────────
    if (has_pending_odom_ && has_last_odom_) {
      gtsam::Pose3 delta =
          odom_to_pose3(last_odom_).between(odom_to_pose3(pending_odom_));
      graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(X(prev_key), X(curr_key), delta,
          gtsam::noiseModel::Diagonal::Sigmas(
              (gtsam::Vector6() << 99.0, 99.0, noise_.odom_angular_noise,
                                   noise_.odom_linear_noise,
                                   noise_.odom_linear_noise, 99.0).finished())));
      last_odom_ = pending_odom_;
      has_pending_odom_ = false;
    }

    // ── LiDAR odometry ───────────────────────────────────────────────
    if (has_pending_lidar_odom_ && has_last_lidar_odom_) {
      gtsam::Pose3 delta =
          odom_to_pose3(last_lidar_odom_).between(odom_to_pose3(pending_lidar_odom_));
      graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(X(prev_key), X(curr_key), delta,
          gtsam::noiseModel::Diagonal::Sigmas(noise_.lidar_odom_noise)));
      last_lidar_odom_ = pending_lidar_odom_;
      has_pending_lidar_odom_ = false;
    }

    // ── Visual odometry ──────────────────────────────────────────────
    if (has_pending_visual_odom_ && has_last_visual_odom_) {
      gtsam::Pose3 delta =
          odom_to_pose3(last_visual_odom_).between(odom_to_pose3(pending_visual_odom_));
      graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(X(prev_key), X(curr_key), delta,
          gtsam::noiseModel::Diagonal::Sigmas(noise_.visual_odom_noise)));
      last_visual_odom_ = pending_visual_odom_;
      has_pending_visual_odom_ = false;
    }

    // ══════════════════════════════════════════════════════════════════
    // ARTICULATION FACTORS — extend state with H(curr_key) = φ
    // ══════════════════════════════════════════════════════════════════
    if (use_articulation_) {
      // Initial value for H(curr_key): propagate from previous optimised φ
      const double phi_predict = current_state_.trailer_angle;
      initial_values_.insert(H_key(curr_key), phi_predict);

      // ── Factor 1: ArticulationDynamics ─────────────────────────────
      // BetweenFactor<double>: penalises large φ changes between keyframes.
      // Measurement = 0 (zero expected change without command info).
      // σ_dynamics = random-walk noise (configurable).
      //
      // Math: e = (φ_k − φ_{k-1}) − 0 = Δφ
      //       Jacobians: ∂e/∂φ_{k-1}=−1,  ∂e/∂φ_k=+1
      {
        auto dyn_noise = gtsam::noiseModel::Isotropic::Sigma(
            1, noise_.phi_sigma_dynamics);
        graph_.add(gtsam::BetweenFactor<double>(
            H_key(prev_key), H_key(curr_key), 0.0, dyn_noise));
      }

      // ── Factor 2: EncoderFactor ────────────────────────────────────
      // PriorFactor<double> on H(curr_key) from the hardware encoder.
      // σ switches between σ_hw (encoder fresh, ±0.5°) and σ_model (stale).
      //
      // Information flow: encoder → constrains φ directly, independent of X.
      if (has_pending_articulation_) {
        const auto & art = pending_articulation_;
        const double phi_meas =
            art.hardware_fresh ? art.hardware_rad : art.effective_rad;
        const double sigma_enc =
            art.hardware_fresh ? noise_.phi_sigma_hardware : noise_.phi_sigma_model;

        graph_.addPrior(H_key(curr_key), phi_meas,
            gtsam::noiseModel::Isotropic::Sigma(1, sigma_enc));

        // ── Factor 3: TrailerPoseFactor ──────────────────────────────
        // Custom 6D factor: residual = Pose3::Logmap(Δ(φ)⁻¹ · T_measured)
        // Noise scaled by 1/√confidence — bad detections contribute little.
        //
        // Information flow: trailer LiDAR → constrains φ via full SE(3) geometry.
        // The 6D residual is overdetermined for 1D φ; GTSAM resolves via
        // weighted least-squares on the Bayes tree.
        //
        // Independence guarantee: trailer/pose uses the encoder prior only for
        // ROI selection (weak geometric hint), not as a direct measurement.
        // The actual measurement is the LiDAR point cloud — statistically
        // independent from the encoder.
        if (use_trailer_pose_ && has_pending_trailer_pose_) {
          const double conf = std::clamp(latest_trailer_confidence_, 0.01, 1.0);

          if (conf >= trailer_min_confidence_) {
            // Scale noise by 1/√confidence: high confidence → tight noise
            const double scale = 1.0 / std::sqrt(conf);
            const double sr = noise_.trailer_sigma_rot   * scale;
            const double st = noise_.trailer_sigma_trans * scale;

            // 6D diagonal noise model [ω; v] = [rx, ry, rz, tx, ty, tz]
            auto trailer_noise = gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector6() << sr, sr, sr, st, st, st).finished());

            const gtsam::Pose3 T_meas = stamped_to_pose3(pending_trailer_pose_);
            graph_.add(mtt_loc::TrailerPoseFactor(
                H_key(curr_key), T_meas, trailer_noise));
          }
          has_pending_trailer_pose_ = false;
        }

        has_pending_articulation_ = false;
      }
    }

    // ══════════════════════════════════════════════════════════════════
    // ISAM2 update
    // ══════════════════════════════════════════════════════════════════
    try {
      isam2_->update(graph_, initial_values_);
      auto result = isam2_->calculateEstimate();

      current_state_.pose     = result.at<gtsam::Pose3>(X(curr_key));
      current_state_.velocity = result.at<gtsam::Vector3>(V(curr_key));
      current_state_.imu_bias =
          result.at<gtsam::imuBias::ConstantBias>(B(curr_key));
      current_state_.key_index = curr_key;

      if (use_articulation_) {
        current_state_.trailer_angle = result.at<double>(H_key(curr_key));
      }

      // ── Extract marginal covariances ─────────────────────────────
      // isam2_->marginalCovariance(key) runs back-substitution on the
      // Bayes tree — O(n) but cheap for a single key at 50 Hz.
      if (extract_covariance_) {
        try {
          // Tractor pose covariance (6×6)
          cov_pose_ = isam2_->marginalCovariance(X(curr_key));
          // Articulation variance (1×1)
          if (use_articulation_) {
            cov_phi_ = isam2_->marginalCovariance(H_key(curr_key));
          }
          has_covariance_ = true;
        } catch (const std::exception & e) {
          RCLCPP_WARN_ONCE(get_logger(),
              "Covariance extraction failed: %s — using zeros", e.what());
          has_covariance_ = false;
        }
      }

    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "ISAM2 update failed: %s", e.what());
      current_state_.key_index = curr_key;
    }

    // Clear pending data and reset IMU
    graph_.resize(0);
    initial_values_.clear();
    imu_preint_->resetIntegrationAndSetBias(current_state_.imu_bias);
    imu_data_count_ = 0;

    publish_odometry();
    publish_articulation();
    publish_tf();
  }

  // ─── Publish tractor odometry (with covariance from ISAM2) ───────
  void publish_odometry() {
    nav_msgs::msg::Odometry msg;
    msg.header.stamp    = has_latest_odom_ ? latest_odom_.header.stamp
                                           : static_cast<builtin_interfaces::msg::Time>(now());
    msg.header.frame_id = map_frame_;
    msg.child_frame_id  = base_frame_;

    const auto & t = current_state_.pose.translation();
    const auto   q = current_state_.pose.rotation().toQuaternion();

    msg.pose.pose.position.x    = t.x();
    msg.pose.pose.position.y    = t.y();
    msg.pose.pose.position.z    = t.z();
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    msg.pose.pose.orientation.w = q.w();

    msg.twist.twist.linear.x = current_state_.velocity.x();
    msg.twist.twist.linear.y = current_state_.velocity.y();
    msg.twist.twist.linear.z = current_state_.velocity.z();

    // Populate 6×6 covariance from ISAM2 marginals (GTSAM: [ω;v] → ROS: [v;ω])
    // GTSAM Pose3 covariance is in the order [rx,ry,rz,tx,ty,tz]
    // ROS Odometry covariance is in the order [x,y,z,rx,ry,rz]
    // We permute: ROS[i][j] = GTSAM[perm[i]][perm[j]], perm = {3,4,5,0,1,2}
    if (has_covariance_) {
      constexpr int perm[6] = {3, 4, 5, 0, 1, 2};  // GTSAM→ROS reorder
      for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
          msg.pose.covariance[i * 6 + j] = cov_pose_(perm[i], perm[j]);
        }
      }
    }

    odom_pub_->publish(msg);
  }

  // ─── Publish optimised articulation angle ─────────────────────────
  void publish_articulation() {
    if (!use_articulation_) return;
    std_msgs::msg::Float64 msg;
    msg.data = current_state_.trailer_angle;
    articulation_pub_->publish(msg);
  }

  // ─── TF broadcast: map → odom ────────────────────────────────────
  void publish_tf() {
    if (!has_latest_odom_) return;
    gtsam::Pose3 map_to_base = current_state_.pose;
    gtsam::Pose3 odom_to_base = odom_to_pose3(latest_odom_);
    gtsam::Pose3 map_to_odom = map_to_base.compose(odom_to_base.inverse());

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp    = latest_odom_.header.stamp;
    tf.header.frame_id = map_frame_;
    tf.child_frame_id  = odom_frame_;
    const auto & t = map_to_odom.translation();
    const auto   q = map_to_odom.rotation().toQuaternion();
    tf.transform.translation.x = t.x();
    tf.transform.translation.y = t.y();
    tf.transform.translation.z = t.z();
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(tf);
  }

  // ─── Members ─────────────────────────────────────────────────────
  std::mutex mtx_;

  // ISAM2
  std::unique_ptr<gtsam::ISAM2> isam2_;
  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values initial_values_;
  gtsam::Matrix66 cov_pose_ = gtsam::Matrix66::Zero();
  gtsam::Matrix11 cov_phi_  = gtsam::Matrix11::Zero();
  bool has_covariance_{false};

  // IMU
  std::unique_ptr<gtsam::PreintegratedCombinedMeasurements> imu_preint_;
  double last_imu_t_{-1.0};
  int imu_data_count_{0};

  // Optimised state
  mtt_loc::NavState current_state_;
  mtt_loc::SensorNoiseParams noise_;
  double trailer_min_confidence_{0.20};

  // Pending sensor data (tractor)
  sensor_msgs::msg::NavSatFix pending_gps_;
  geometry_msgs::msg::QuaternionStamped pending_heading_;
  nav_msgs::msg::Odometry pending_odom_, last_odom_, latest_odom_;
  nav_msgs::msg::Odometry pending_lidar_odom_, last_lidar_odom_;
  nav_msgs::msg::Odometry pending_visual_odom_, last_visual_odom_;

  bool has_pending_gps_{false}, has_pending_heading_{false};
  bool has_pending_odom_{false}, has_last_odom_{false}, has_latest_odom_{false};
  bool has_pending_lidar_odom_{false}, has_last_lidar_odom_{false};
  bool has_pending_visual_odom_{false}, has_last_visual_odom_{false};

  // Pending sensor data (articulation)
  mtt_msgs::msg::MttArticulationState pending_articulation_;
  geometry_msgs::msg::PoseStamped pending_trailer_pose_;
  double latest_trailer_confidence_{0.0};
  bool has_pending_articulation_{false};
  bool has_pending_trailer_pose_{false};

  // GPS origin
  double gps_origin_lat_{0.0}, gps_origin_lon_{0.0}, gps_origin_alt_{0.0};
  bool gps_origin_set_{false};

  // Feature flags
  bool use_imu_, use_odom_, use_gps_, use_gps_heading_;
  bool use_lidar_odom_, use_visual_odom_;
  bool use_articulation_, use_trailer_pose_;
  bool extract_covariance_;

  // Frame IDs / topics
  std::string map_frame_, odom_frame_, base_frame_;
  std::string imu_topic_, track_odom_topic_, gps_fix_topic_;
  std::string gps_heading_topic_, lidar_odom_topic_, visual_odom_topic_;
  std::string articulation_topic_, trailer_pose_topic_, trailer_confidence_topic_;
  int min_imu_samples_per_update_{10};

  // ROS interfaces
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr articulation_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr heading_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lidar_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr visual_odom_sub_;
  rclcpp::Subscription<mtt_msgs::msg::MttArticulationState>::SharedPtr articulation_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr trailer_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr trailer_confidence_sub_;

  rclcpp::TimerBase::SharedPtr opt_timer_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FactorGraphNode>());
  rclcpp::shutdown();
  return 0;
}
