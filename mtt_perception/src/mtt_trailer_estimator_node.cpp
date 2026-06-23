// mtt_trailer_estimator_node.cpp — MTT Trailer Pose Estimator V1.0
//
// Every mathematical step is commented with a reference to the relevant
// equation in one of the standard references:
//   [TR]  Thrun, Burgard, Fox — "Probabilistic Robotics" (2005)
//   [MV]  Hartley, Zisserman — "Multiple View Geometry" (2nd ed.)
//   [SE3] Murray, Li, Sastry — "A Mathematical Introduction to Robotic Manipulation" (1994)
//
// Build requirements: C++20, Eigen3, ROS 2 Jazzy, PCL (for PointCloud2 iteration only).

#include "mtt_perception/mtt_trailer_estimator_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <random>
#include <sstream>

#include <Eigen/Dense>
#include <Eigen/SVD>

#include "builtin_interfaces/msg/duration.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/exceptions.h"
#include "visualization_msgs/msg/marker.hpp"

namespace mtt_perception
{

// ── Anonymous-namespace helpers — internal to this translation unit ──
namespace
{

constexpr double kPi     = 3.141592653589793238462643383279502884;
constexpr double kPi2    = kPi * 0.5;
constexpr double kTwoPi  = kPi * 2.0;

// URDF joint origin: Trans(xyz) · RotRPY(rpy)  — URDF convention Rz·Ry·Rx.
[[nodiscard]] Eigen::Isometry3d urdfOrigin(
  const Eigen::Vector3d & xyz,
  const Eigen::Vector3d & rpy)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = xyz;
  T.linear() =
    (Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
     Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
     Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX()))
    .toRotationMatrix();
  return T;
}

// URDF revolute joint: Trans(xyz) · RotRPY(rpy) · Rz(q).
[[nodiscard]] Eigen::Isometry3d urdfJoint(
  const Eigen::Vector3d & xyz,
  const Eigen::Vector3d & rpy,
  double q)
{
  return urdfOrigin(xyz, rpy) *
         Eigen::Isometry3d(Eigen::AngleAxisd(q, Eigen::Vector3d::UnitZ()));
}

// 3×3 skew-symmetric matrix of v — used in the adjoint operator.
[[nodiscard]] Eigen::Matrix3d skew3(const Eigen::Vector3d & v)
{
  Eigen::Matrix3d S;
  S <<    0.0, -v.z(),  v.y(),
        v.z(),    0.0, -v.x(),
       -v.y(),  v.x(),    0.0;
  return S;
}

// 6×6 adjoint representation of T for [position; rotation] convention.
// Ad(T) maps body-frame twist to world-frame twist.
// [SE3] eq. 2.58:  Ad(T) = | R    [t]×·R |
//                           | 0       R   |
[[nodiscard]] Eigen::Matrix<double, 6, 6> adjoint6(const Eigen::Isometry3d & T)
{
  const Eigen::Matrix3d R    = T.rotation();
  const Eigen::Matrix3d tx_R = skew3(T.translation()) * R;
  Eigen::Matrix<double, 6, 6> Ad = Eigen::Matrix<double, 6, 6>::Zero();
  Ad.topLeftCorner<3, 3>()     = R;
  Ad.topRightCorner<3, 3>()    = tx_R;
  Ad.bottomRightCorner<3, 3>() = R;
  return Ad;
}

// Extract quaternion RPY (roll, pitch, yaw) from a rotation matrix.
// Returns [roll, pitch, yaw] using the atan2 / asin convention.
// Guards against degenerate inputs (gimbal lock at pitch = ±π/2).
[[nodiscard]] Eigen::Vector3d rotToRpy(const Eigen::Matrix3d & R)
{
  // pitch = asin(-R(2,0)), but clamp to avoid domain error.
  const double sin_pitch = std::clamp(-R(2, 0), -1.0, 1.0);
  const double pitch = std::asin(sin_pitch);
  double roll{0.0};
  double yaw{0.0};
  if (std::abs(std::abs(sin_pitch) - 1.0) < 1e-6) {
    // Gimbal lock — set roll=0, solve for yaw only.
    yaw  = std::atan2(-R(0, 1), R(0, 2));
  } else {
    roll = std::atan2(R(2, 1), R(2, 2));
    yaw  = std::atan2(R(1, 0), R(0, 0));
  }
  return {roll, pitch, yaw};
}

// Build a geometry_msgs::Point from an Eigen vector.
[[nodiscard]] geometry_msgs::msg::Point pointMsg(const Eigen::Vector3d & p)
{
  geometry_msgs::msg::Point out;
  out.x = p.x();
  out.y = p.y();
  out.z = p.z();
  return out;
}

// Marker lifetime helper.
[[nodiscard]] builtin_interfaces::msg::Duration markerLifetime(double seconds)
{
  builtin_interfaces::msg::Duration d;
  d.sec = static_cast<int32_t>(std::floor(seconds));
  d.nanosec = static_cast<uint32_t>((seconds - static_cast<double>(d.sec)) * 1e9);
  return d;
}

// Wrap angle to (−π, π].
[[nodiscard]] double normalizeAngle(double a) noexcept
{
  a = std::fmod(a + kPi, kTwoPi);
  if (a < 0.0) { a += kTwoPi; }
  return a - kPi;
}

// Chi-squared quantile constants (precomputed from SciPy chi2.ppf).
constexpr double kChi2_3dof_99 = 11.3449;  // χ²(3, p=0.99)
constexpr double kChi2_4dof_99 = 13.2767;  // χ²(4, p=0.99)

}  // anonymous namespace

// ── Constructor ──
TrailerEstimatorNode::TrailerEstimatorNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("mtt_trailer_estimator_node", options),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_)
{
  // ── Declare parameters ──
  map_frame_         = declare_parameter("map_frame", std::string("map"));
  trailer_tf_frame_  = declare_parameter("trailer_tf_frame", std::string("trailer_body"));
  broadcast_tf_      = declare_parameter("broadcast_tf", true);
  publish_rate_      = declare_parameter("publish_rate", 50.0);

  // Sensor topics
  const std::string art_topic   = declare_parameter("articulation_angle_topic",
    std::string("/trailer/articulation_angle"));
  const std::string hw_art_topic = declare_parameter("hardware_angle_topic",
    std::string("/hardware/articulation_angle"));
  const std::string odom_topic   = declare_parameter("tractor_odom_topic",
    std::string("/mtt_odometry"));
  const std::string icp_topic    = declare_parameter("icp_odom_topic",
    std::string("/mapping/icp_odom"));
  const std::string rsairy_topic = declare_parameter("rsairy_topic",
    std::string("/rsairy_ns/points"));
  const std::string hesai_topic  = declare_parameter("hesai_topic",
    std::string("/hesai_lidar/points"));
  const std::string imu_topic    = declare_parameter("imu_topic",
    std::string("/mti100/data"));

  enable_hesai_  = declare_parameter("enable_hesai", true);
  enable_icp_    = declare_parameter("enable_icp", true);
  icp_odom_max_age_s_ = declare_parameter("icp_odom_max_age_s", 0.25);
  phi_max_age_s_ = declare_parameter("phi_max_age_s", 0.5);

  // Trailer geometry (for URDF model and ICP wireframe)
  model_.s_front    = declare_parameter("model.s_front",    -0.05);
  model_.s_rear     = declare_parameter("model.s_rear",      1.90);
  model_.l_half     = declare_parameter("model.l_half",      0.40);
  model_.h_rail     = declare_parameter("model.h_rail",      0.30);
  model_.point_spacing = declare_parameter("model.point_spacing", 0.10);

  // ROI
  roi_half_s_ = declare_parameter("roi_half_s", 1.05);
  roi_half_l_ = declare_parameter("roi_half_l", 0.65);
  roi_half_h_ = declare_parameter("roi_half_h", 0.80);
  voxel_size_ = declare_parameter("voxel_size", 0.04);
  min_pts_after_ground_ = declare_parameter("min_pts_after_ground", 20);

  // Ground RANSAC
  ground_ransac_iters_   = declare_parameter("ground.ransac_iters",    50);
  ground_inlier_thresh_  = declare_parameter("ground.inlier_thresh_m", 0.05);
  ground_max_tilt_rad_   = declare_parameter("ground.max_tilt_rad",    0.52);

  // Line RANSAC
  line_ransac_iters_    = declare_parameter("line.ransac_iters",     100);
  line_inlier_thresh_   = declare_parameter("line.inlier_thresh_m",  0.05);
  line_min_inlier_frac_ = declare_parameter("line.min_inlier_frac",  0.6);

  // ICP
  icp_max_iters_       = declare_parameter("icp.max_iters",        20);
  icp_max_corr_dist_   = declare_parameter("icp.max_corr_dist_m",  0.5);
  icp_convergence_tol_ = declare_parameter("icp.convergence_tol",  1e-4);
  icp_min_inliers_     = declare_parameter("icp.min_inliers",      10);

  // EKF tuning
  velocity_decay_      = declare_parameter("ekf.velocity_decay",       0.95);
  q_xy_                = declare_parameter("ekf.q_xy",                  0.01);
  q_z_                 = declare_parameter("ekf.q_z",                   0.001);
  q_rp_                = declare_parameter("ekf.q_rp",                  0.001);
  q_yaw_               = declare_parameter("ekf.q_yaw",                 0.005);
  q_vxy_               = declare_parameter("ekf.q_vxy",                 0.1);
  q_vz_                = declare_parameter("ekf.q_vz",                  0.01);
  q_vyaw_              = declare_parameter("ekf.q_vyaw",                0.05);
  divergence_trace_max_= declare_parameter("ekf.divergence_trace_max",  50.0);
  chi2_kinematic_      = kChi2_4dof_99;
  chi2_lidar_          = kChi2_3dof_99;

  filter_period_ms_    = declare_parameter("filter_period_ms", 10);

  // Hitch cross-validation
  hitch_validation_sigma_      = declare_parameter("hitch.validation_sigma",      0.025);
  hitch_disagreement_timeout_  = declare_parameter("hitch.disagreement_timeout_s", 0.5);

  // Hitch + body geometry in base_link (must match URDF / trailer_pose_node values)
  hitch_base_x_         = declare_parameter("hitch_base_x",         -1.45);
  hitch_base_y_         = declare_parameter("hitch_base_y",         -0.085);
  hitch_base_z_         = declare_parameter("hitch_base_z",          0.35);
  trailer_body_offset_  = declare_parameter("trailer_body_offset",   0.90);
  trailer_front_offset_ = declare_parameter("trailer_front_offset",  0.05);
  trailer_rear_offset_  = declare_parameter("trailer_rear_offset",   1.90);

  // PCA detection — trailer_pose_node V4.0 algorithm
  pca_min_points_           = declare_parameter("pca.min_points",            30.0);
  pca_min_points_after_ground_ = declare_parameter("pca.min_points_after_ground", 20.0);
  pca_min_ratio_            = declare_parameter("pca.min_ratio",             2.2);
  pca_max_yaw_correction_   = declare_parameter("pca.max_yaw_correction",    0.20);
  pca_min_span_s_           = declare_parameter("pca.min_span_s",            0.45);
  pca_max_yaw_jump_         = declare_parameter("pca.max_yaw_jump",          0.15);
  pca_max_pos_s_            = declare_parameter("pca.max_pos_s",             0.05);
  pca_max_pos_l_            = declare_parameter("pca.max_pos_l",             0.08);
  pca_max_pos_h_            = declare_parameter("pca.max_pos_h",             0.06);
  pca_ground_quantile_      = declare_parameter("pca.ground_quantile",       0.10);
  pca_ground_margin_        = declare_parameter("pca.ground_margin",         0.06);
  pca_r_base_               = declare_parameter("pca.r_base",                0.005);
  pca_r_scale_              = declare_parameter("pca.r_scale",               0.05);

  // EMA filter
  alpha_yaw_       = declare_parameter("pca.alpha_yaw",      0.25);
  alpha_pitch_     = declare_parameter("pca.alpha_pitch",    0.02);
  alpha_position_  = declare_parameter("pca.alpha_position", 0.20);
  yaw_decay_       = declare_parameter("pca.yaw_decay",      0.85);
  pitch_decay_     = declare_parameter("pca.pitch_decay",    0.80);
  position_decay_  = declare_parameter("pca.position_decay", 0.70);

  // ── Build Q base matrix (diagonal) ──
  // Q is scaled by dt in ekfPredict().  Units: variance per second.
  Q_base_.setZero();
  Q_base_(idx::kX,       idx::kX)       = q_xy_;
  Q_base_(idx::kY,       idx::kY)       = q_xy_;
  Q_base_(idx::kZ,       idx::kZ)       = q_z_;
  Q_base_(idx::kRoll,    idx::kRoll)    = q_rp_;
  Q_base_(idx::kPitch,   idx::kPitch)   = q_rp_;
  Q_base_(idx::kYaw,     idx::kYaw)     = q_yaw_;
  Q_base_(idx::kVx,      idx::kVx)      = q_vxy_;
  Q_base_(idx::kVy,      idx::kVy)      = q_vxy_;
  Q_base_(idx::kVz,      idx::kVz)      = q_vz_;
  Q_base_(idx::kYawRate, idx::kYawRate) = q_vyaw_;

  // ── Precompute URDF chain and model ──
  initUrdfChain();
  initTrailerModel();

  // ── Publishers ──
  const rclcpp::SensorDataQoS sensor_qos;

  pose_pub_       = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/trailer/pose", rclcpp::QoS(10));
  pose_in_map_pub_= create_publisher<geometry_msgs::msg::PoseStamped>(
    "/trailer/pose_in_map", rclcpp::QoS(10));
  odom_pub_       = create_publisher<nav_msgs::msg::Odometry>(
    "/trailer/odom", rclcpp::QoS(10));
  // Back-computed articulation angle (for monitoring/cross-validation only).
  // Published on a DIFFERENT topic than the primary source (/trailer/articulation_angle
  // from trailer_detector_node) to avoid a self-feedback loop.
  articulation_angle_pub_ = create_publisher<std_msgs::msg::Float64>(
    "/trailer/articulation_angle_estimated", sensor_qos);
  markers_pub_    = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/trailer/body_markers", rclcpp::QoS(10));
  roi_cloud_pub_  = create_publisher<sensor_msgs::msg::PointCloud2>(
    "/trailer/trailer_roi_cloud", sensor_qos);

  diag_latency_pub_     = create_publisher<std_msgs::msg::Float64>(
    "/trailer/diag/filter_latency_ms", sensor_qos);
  diag_lidar_error_pub_ = create_publisher<std_msgs::msg::Float64>(
    "/trailer/diag/lidar_alignment_error_m", sensor_qos);
  diag_inlier_ratio_pub_= create_publisher<std_msgs::msg::Float64>(
    "/trailer/diag/line_inlier_ratio", sensor_qos);

  yaw_prior_pub_  = create_publisher<std_msgs::msg::Float64>(
    "/trailer/yaw_prior", sensor_qos);
  pitch_used_pub_ = create_publisher<std_msgs::msg::Float64>(
    "/trailer/pitch_used", sensor_qos);
  roll_used_pub_  = create_publisher<std_msgs::msg::Float64>(
    "/trailer/roll_used", sensor_qos);

  if (broadcast_tf_) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  // ── Subscribers ──
  articulation_angle_sub_ = create_subscription<std_msgs::msg::Float64>(
    art_topic, sensor_qos,
    [this](std_msgs::msg::Float64::ConstSharedPtr m) { onArticulationAngle(m); });

  hardware_angle_sub_ = create_subscription<std_msgs::msg::Float64>(
    hw_art_topic, sensor_qos,
    [this](std_msgs::msg::Float64::ConstSharedPtr m) { onHardwareArticulationAngle(m); });

  tractor_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, rclcpp::QoS(10),
    [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { onTractorOdom(m); });

  icp_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    icp_topic, rclcpp::QoS(10),
    [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { onIcpOdom(m); });

  rsairy_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    rsairy_topic, sensor_qos,
    [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) { onRsairyCloud(m); });

  if (enable_hesai_) {
    hesai_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      hesai_topic, sensor_qos,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) { onHesaiCloud(m); });
  }

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    imu_topic, sensor_qos,
    [this](sensor_msgs::msg::Imu::ConstSharedPtr m) { onImu(m); });

  // ── Filter thread ──
  stop_flag_.store(false, std::memory_order_relaxed);
  filter_thread_ = std::thread(&TrailerEstimatorNode::filterThreadFunc, this);

  // ── Publish timer ──
  const auto period = std::chrono::duration<double>(1.0 / publish_rate_);
  publish_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    [this]() { publishTimerCallback(); });

  RCLCPP_INFO(get_logger(),
    "TrailerEstimatorNode V1.0 ready — filter_period=%d ms, publish=%.0f Hz, hesai=%s, icp=%s",
    filter_period_ms_, publish_rate_,
    enable_hesai_ ? "on" : "off",
    enable_icp_   ? "on" : "off");
}

TrailerEstimatorNode::~TrailerEstimatorNode()
{
  stop_flag_.store(true, std::memory_order_release);
  if (filter_thread_.joinable()) {
    filter_thread_.join();
  }
}

// ── initUrdfChain — precompute URDF kinematic constants. Values copied verbatim from mtt_description/urdf/robot.urdf.xacro and cross-checked against trailer_localizer_node.cpp.  Do not edit independently. Kinematic chain: base_footprint → base_link → pitch → yaw → roll → MTT_remorque Δ(φ, α) = A_prefix_ · Rz(-π/2+α) · A_suffix_ · Rz(π/2+φ) · B_ A_prefix_ = T_bf_bl · T_pitch_origin A_suffix_ = T_yaw_origin B_        = T_roll (constant) ──
void TrailerEstimatorNode::initUrdfChain()
{
  // base_footprint → base_link: fixed, z=-0.1
  Eigen::Isometry3d T_bf_bl = Eigen::Isometry3d::Identity();
  T_bf_bl.translation() << 0.0, 0.0, -0.1;

  // pitch joint origin (rpy=(-π/2,0,0) — joint rotation Rz(-π/2+α) applied at runtime)
  const Eigen::Isometry3d T_pitch_origin = urdfOrigin(
    {-1.0511878376018575, 0.2125028310306423, 0.3577510511469179},
    {-kPi2, 0.0, 0.0});

  // yaw joint origin (rpy=(-π/2,0,0) — joint rotation Rz(π/2+φ) applied at runtime)
  const Eigen::Isometry3d T_yaw_origin = urdfOrigin(
    {0.0, 0.05714999999950, -0.26352500000200},
    {-kPi2, 0.0, 0.0});

  // roll joint: constant rotation q=-π/2 around local Z
  const Eigen::Isometry3d T_roll = urdfJoint(
    {0.0, 0.0, -0.05714999999985},
    {-kPi2, 0.0, -kPi2},
    -kPi2);

  A_prefix_ = T_bf_bl * T_pitch_origin;  // before pitch rotation
  A_suffix_ = T_yaw_origin;              // between pitch and yaw
  B_        = T_roll;                    // after yaw rotation (constant)

  const Eigen::Isometry3d delta0 = computeDelta(0.0, 0.0);
  yaw_delta_ref_ = rotToRpy(delta0.rotation()).z();

  RCLCPP_INFO(get_logger(),
    "URDF chain: Δ(φ=0,α=0) → trailer at [%.3f, %.3f, %.3f] in base_footprint, yaw_ref=%.3f rad",
    delta0.translation().x(), delta0.translation().y(), delta0.translation().z(),
    yaw_delta_ref_);
}

// ── initTrailerModel — sample wireframe model into pts_local ──
void TrailerEstimatorNode::initTrailerModel()
{
  model_.pts_local.clear();
  const double step = model_.point_spacing;
  const double s_len = model_.s_rear - model_.s_front;
  const int n_rail = static_cast<int>(std::ceil(s_len / step)) + 1;
  const int n_beam = static_cast<int>(std::ceil(2.0 * model_.l_half / step)) + 1;
  model_.pts_local.reserve(static_cast<std::size_t>(2 * n_rail + n_beam));

  // Left and right rails (along s-axis).
  //
  // CONVENTION: The MTT_remorque URDF frame's +x axis points in the TRACTOR'S forward
  // direction.  The physical trailer body extends in the OPPOSITE direction (behind the
  // hitch).  We therefore negate s when storing so that model_.pts_local[i].x() < 0
  // for all physical trailer points, and Rz(yaw) · model_pt correctly places them
  // BEHIND the hitch in the map frame.
  //
  //   s_front ≥ 0  →  stored as  -s_front  (slightly behind hitch)
  //   s_rear  > 0  →  stored as  -s_rear   (further behind → trailer end)
  for (int i = 0; i < n_rail; ++i) {
    const double s = model_.s_front + static_cast<double>(i) * step;
    model_.pts_local.emplace_back(-s,  model_.l_half, model_.h_rail);  // left  (negated)
    model_.pts_local.emplace_back(-s, -model_.l_half, model_.h_rail);  // right (negated)
  }

  // Rear crossbeam (along l-axis at s_rear).
  for (int i = 0; i < n_beam; ++i) {
    const double l = -model_.l_half + static_cast<double>(i) * step;
    model_.pts_local.emplace_back(-model_.s_rear, l, model_.h_rail);  // negated
  }

  RCLCPP_INFO(get_logger(),
    "Trailer model: %zu points, rails=[%.2f..%.2f]m, width=%.2f m, h=%.2f m",
    model_.pts_local.size(), model_.s_front, model_.s_rear,
    2.0 * model_.l_half, model_.h_rail);
}

// ── Kinematic chain computation — exact replica of TrailerLocalizerNode. Δ(φ, α) = A_prefix_ · Rz(-π/2+α) · A_suffix_ · Rz(π/2+φ) · B_ ──
Eigen::Isometry3d TrailerEstimatorNode::computeDelta(double phi, double alpha) const noexcept
{
  const Eigen::Isometry3d R_pitch(Eigen::AngleAxisd(-kPi2 + alpha, Eigen::Vector3d::UnitZ()));
  const Eigen::Isometry3d R_yaw(Eigen::AngleAxisd( kPi2 + phi,   Eigen::Vector3d::UnitZ()));
  return A_prefix_ * R_pitch * A_suffix_ * R_yaw * B_;
}

// ── Sensor callbacks — fast, no heavy computation ──
void TrailerEstimatorNode::onArticulationAngle(std_msgs::msg::Float64::ConstSharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(mutex_phi_);
    phi_lidar_       = normalizeAngle(msg->data);
    phi_lidar_stamp_ = get_clock()->now();
    phi_lidar_valid_ = true;
  }
  // Attempt to queue a kinematic measurement using the freshest tractor pose.
  computeAndQueueKinematicMeas(normalizeAngle(msg->data), get_clock()->now());
}

void TrailerEstimatorNode::onHardwareArticulationAngle(
  std_msgs::msg::Float64::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_phi_);
  phi_hardware_       = normalizeAngle(msg->data);
  phi_hardware_stamp_ = get_clock()->now();
  phi_hardware_valid_ = true;
}

void TrailerEstimatorNode::onTractorOdom(nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_tractor_);
  latest_odom_ = *msg;
}

void TrailerEstimatorNode::onIcpOdom(nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_tractor_);
  latest_icp_odom_ = *msg;
  icp_odom_stamp_  = rclcpp::Time(msg->header.stamp);
}

void TrailerEstimatorNode::onRsairyCloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  if (!enable_icp_) { return; }
  processLidarCloud(*msg, rsairy_queue_, "rsairy");
}

void TrailerEstimatorNode::onHesaiCloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  if (!enable_icp_) { return; }
  processLidarCloud(*msg, hesai_queue_, "hesai");
}

void TrailerEstimatorNode::onImu(sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  // Cache pitch and roll from IMU orientation for ICP constraint.
  // Quaternion → RPY (single-line, no branch — IMU rate is 400 Hz).
  const Eigen::Quaterniond q(
    msg->orientation.w, msg->orientation.x,
    msg->orientation.y, msg->orientation.z);
  const Eigen::Vector3d rpy = rotToRpy(q.normalized().toRotationMatrix());
  imu_roll_.store(rpy.x(),  std::memory_order_relaxed);
  imu_pitch_.store(rpy.y(), std::memory_order_relaxed);
}

// ── computeAndQueueKinematicMeas — build the kinematic pseudo-measurement. Mathematical model: T_map_trailer = T_map_base_footprint · Δ(φ, α) z_kinematic = [x, y, z, yaw] extracted from T_map_trailer Covariance propagation ([TR] eq. 3.17-style first-order Jacobian): Σ_trailer ≈ J_T · Σ_tractor · J_T' + J_φ · σ²_φ · J_φ' + J_α · σ²_α · J_α' ──
void TrailerEstimatorNode::computeAndQueueKinematicMeas(
  double phi, const rclcpp::Time & stamp)
{
  // Get tractor pose in MAP frame via TF lookup.
  // This is the only correct way: /mtt_odometry and /mapping/icp_odom are in
  // the odom frame; using them directly would place the EKF in odom frame while
  // the LiDAR ICP outputs are in map frame — causing a systematic frame mismatch.
  // rclcpp::Time(0) = "latest available TF" — robust for bag replay.
  Eigen::Isometry3d T_tractor = Eigen::Isometry3d::Identity();
  try {
    const geometry_msgs::msg::TransformStamped tf_msg =
      tf_buffer_.lookupTransform(map_frame_, "base_footprint",
        rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1));
    const auto & tr = tf_msg.transform.translation;
    const auto & qr = tf_msg.transform.rotation;
    T_tractor.translation() << tr.x, tr.y, tr.z;
    T_tractor.linear() =
      Eigen::Quaterniond(qr.w, qr.x, qr.y, qr.z).normalized().toRotationMatrix();
  } catch (const tf2::TransformException &) {
    return;  // Map frame not yet available — wait for ICP mapper to initialise
  }

  // Odom covariance for propagation (use latest odom if available).
  Cov6d sigma_tractor;
  {
    std::lock_guard<std::mutex> lock(mutex_tractor_);
    if (!latest_odom_) {
      // Use conservative fallback before first odom arrives.
      sigma_tractor = Cov6d::Zero();
      sigma_tractor.diagonal() << 0.1, 0.1, 0.05, 0.02, 0.02, 0.05;
    } else {
      const rclcpp::Time now = get_clock()->now();
      const bool icp_fresh =
        latest_icp_odom_.has_value() &&
        (now - icp_odom_stamp_).seconds() < icp_odom_max_age_s_;
      sigma_tractor = covFromOdom(icp_fresh ? *latest_icp_odom_ : *latest_odom_);
    }
  }

  // α = IMU pitch (best available, or 0 on flat terrain).
  const double alpha = imu_pitch_.load(std::memory_order_relaxed);

  // Forward kinematics: T_map_trailer = T_map_base_footprint · Δ(φ, α).
  const Eigen::Isometry3d delta    = computeDelta(phi, alpha);
  const Eigen::Isometry3d T_trailer = T_tractor * delta;

  // Extract measurement: [x, y, z, yaw].
  const Eigen::Vector3d rpy = rotToRpy(T_trailer.rotation());
  KinematicMeas meas;
  meas.stamp   = stamp;
  meas.z(0)    = T_trailer.translation().x();  // x in map
  meas.z(1)    = T_trailer.translation().y();  // y in map
  meas.z(2)    = T_trailer.translation().z();  // z in map
  meas.z(3)    = rpy.z();                      // yaw in map

  // Covariance propagation.
  // σ²_φ: lidar KF uncertainty (≈0.005 rad from detector), hardware fallback 0.008.
  // σ²_α: IMU-fused pitch uncertainty (~0.020 rad from MTi-100 spec).
  constexpr double kSigmaPhiSq    = 0.005 * 0.005;  // rad²
  constexpr double kSigmaAlphaSq  = 0.020 * 0.020;  // rad²
  // Hitch geometry uncertainty (σ_L ≈ 0.01 m → contributes to x,y).
  constexpr double kSigmaLSq      = 0.01  * 0.01;   // m²

  const Eigen::Matrix<double, 6, 1> J_phi = jacobianPhi(T_tractor, phi, alpha);
  meas.R = propagateKinematicCov(sigma_tractor, kSigmaPhiSq, kSigmaAlphaSq, J_phi);

  // Add hitch geometry uncertainty on x,y.
  meas.R(0, 0) += kSigmaLSq;
  meas.R(1, 1) += kSigmaLSq;

  meas.valid = true;
  if (!kinematic_queue_.push(meas)) {
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,
      "kinematic_queue_ full — dropping measurement");
  }
}

// ── jacobianPhi — ∂T_trailer/∂φ via central differences. Returns 6×1 body-frame twist per radian of φ. [TR] numerical Jacobian approach used in trailer_localizer_node ──
Eigen::Matrix<double, 6, 1> TrailerEstimatorNode::jacobianPhi(
  const Eigen::Isometry3d & T_tractor, double phi, double alpha) const
{
  constexpr double kEps = 1e-5;
  const Eigen::Isometry3d T_nom   = T_tractor * computeDelta(phi,       alpha);
  const Eigen::Isometry3d T_plus  = T_tractor * computeDelta(phi + kEps, alpha);
  const Eigen::Isometry3d T_minus = T_tractor * computeDelta(phi - kEps, alpha);

  // Finite difference in local (body) frame: T_nom^{-1} · T_±
  const Eigen::Isometry3d dT_p = T_nom.inverse() * T_plus;
  const Eigen::Isometry3d dT_m = T_nom.inverse() * T_minus;

  Eigen::Matrix<double, 6, 1> J_p, J_m;
  J_p.head<3>() = dT_p.translation();
  J_m.head<3>() = dT_m.translation();
  const Eigen::AngleAxisd aa_p(dT_p.rotation()), aa_m(dT_m.rotation());
  J_p.tail<3>() = aa_p.axis() * aa_p.angle();
  J_m.tail<3>() = aa_m.axis() * aa_m.angle();
  return (J_p - J_m) / (2.0 * kEps);  // central difference
}

// ── propagateKinematicCov — project 6×6 tractor covariance to 4×4 trailer [x,y,z,yaw] ──
Cov4d TrailerEstimatorNode::propagateKinematicCov(
  const Cov6d & sigma_tractor,
  double sigma2_phi,
  double sigma2_alpha,
  const Eigen::Matrix<double, 6, 1> & J_phi) const
{
  // Right-perturbation: J_T = Ad(Δ^{-1}) maps tractor body-twist to trailer uncertainty.
  // We only need [x,y,z,yaw] rows of the full 6×6 propagated covariance.
  // Full: Σ_trailer6 = J_T · Σ_tractor · J_T' + σ²_φ · J_φ · J_φ'
  // (α uncertainty has small effect on x,y,z,yaw — included via a diagonal floor.)
  constexpr double kSigmaAlphaXY = 0.01 * 0.01;  // ≈ hitch height × sigma_alpha

  // Selector matrix to extract [x,y,z,yaw] from the 6-DOF (position;rotation) vector.
  // [TR] uses [v; ω] convention: rows 0,1,2 = translational, rows 3,4,5 = rotational.
  // We want [x, y, z, yaw] = rows [0, 1, 2, 5] of the 6-DOF state.
  Eigen::Matrix<double, 4, 6> S = Eigen::Matrix<double, 4, 6>::Zero();
  S(0, 0) = 1.0;  // x
  S(1, 1) = 1.0;  // y
  S(2, 2) = 1.0;  // z
  S(3, 5) = 1.0;  // yaw (row 5 of [v;ω])

  // Propagate: Σ_4 ≈ S · (J_T · Σ_tractor · J_T') · S'  +  φ-Jacobian contribution
  // Approximate J_T ≈ Identity (first-order, valid when Δ is near identity for small |φ|).
  // For the full formula, we would need: const auto J_T = adjoint6(delta.inverse());
  // For the 4-DOF extraction we use a simplified scalar-row approach:
  const Cov4d Sigma_tractor_4 = S * sigma_tractor * S.transpose();  // extract [x,y,z,yaw] block
  const Eigen::Matrix<double, 4, 1> J_phi_4 = S * J_phi;  // project φ Jacobian

  Cov4d R = Sigma_tractor_4
           + sigma2_phi  * J_phi_4  * J_phi_4.transpose();

  // α uncertainty contribution: primarily affects z and yaw.
  R(2, 2) += kSigmaAlphaXY;
  R(3, 3) += sigma2_alpha * 0.1;  // α has smaller effect on yaw than φ

  // Regularise: ensure positive definiteness.
  R = 0.5 * (R + R.transpose());
  constexpr double kRegFloor = 1e-6;
  for (int i = 0; i < 4; ++i) { R(i, i) = std::max(R(i, i), kRegFloor); }

  return R;
}

// ── covFromOdom — extract 6×6 pose covariance from nav_msgs/Odometry. ROS convention: [x,y,z,rx,ry,rz] row-major.  Fallback diagonal if all-zero ──
Cov6d TrailerEstimatorNode::covFromOdom(const nav_msgs::msg::Odometry & odom) noexcept
{
  const Cov6d cov = Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(
    odom.pose.covariance.data());

  if (cov.isZero(1e-12)) {
    // Factor-graph odom may not populate covariance — use conservative defaults.
    Cov6d fallback = Cov6d::Zero();
    fallback.diagonal() << 0.05 * 0.05, 0.05 * 0.05, 0.05 * 0.05,
                           0.01 * 0.01, 0.01 * 0.01, 0.01 * 0.01;
    return fallback;
  }
  return cov;
}

// ── lookupTfToMap — TF lookup with timeout, returns Isometry3d or nullopt ──
std::optional<Eigen::Isometry3d> TrailerEstimatorNode::lookupTfToMap(
  const std::string & sensor_frame,
  const rclcpp::Time & stamp) const
{
  try {
    const geometry_msgs::msg::TransformStamped tf =
      tf_buffer_.lookupTransform(map_frame_, sensor_frame, stamp,
        rclcpp::Duration::from_seconds(0.05));

    const auto & tr = tf.transform.translation;
    const auto & qr = tf.transform.rotation;
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translation() << tr.x, tr.y, tr.z;
    T.linear() = Eigen::Quaterniond(qr.w, qr.x, qr.y, qr.z).normalized().toRotationMatrix();
    return T;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 3000,
      "TF %s→%s unavailable: %s", sensor_frame.c_str(), map_frame_.c_str(), ex.what());
    return std::nullopt;
  }
}

// ── cacheStaticTf — look up sensor_frame ← base_link once and cache the result. The RS-Airy is rigidly mounted (URDF-defined), so this TF is static. On first call: perform lookup using rclcpp::Time(0) (latest available). Subsequent calls: return immediately if the same sensor_frame is cached. Returns true when the TF has been successfully cached ──
bool TrailerEstimatorNode::cacheStaticTf(const std::string & sensor_frame)
{
  if (tf_sensor_cached_ && cached_sensor_frame_ == sensor_frame) { return true; }

  try {
    const geometry_msgs::msg::TransformStamped tf_msg =
      tf_buffer_.lookupTransform(sensor_frame, "base_link",
        rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1));
    const auto & tr = tf_msg.transform.translation;
    const auto & qr = tf_msg.transform.rotation;
    R_sensor_from_bl_ =
      Eigen::Quaterniond(qr.w, qr.x, qr.y, qr.z).normalized().toRotationMatrix();
    t_sensor_from_bl_ << tr.x, tr.y, tr.z;
    cached_sensor_frame_ = sensor_frame;
    tf_sensor_cached_ = true;
    RCLCPP_INFO(get_logger(),
      "[estimator] Cached TF %s←base_link: t=[%.3f, %.3f, %.3f]",
      sensor_frame.c_str(), tr.x, tr.y, tr.z);
    return true;
  } catch (const tf2::TransformException & e) {
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 5000,
      "[estimator] Cannot cache TF %s←base_link: %s", sensor_frame.c_str(), e.what());
    return false;
  }
}

// ── buildKinematicPrior — compute trailer ROI centre in base_link and sensor frame. Same formula as trailer_pose_node (V4.0): yaw_prior = π − theta   (theta = articulation angle from detector KF) u_base    = [cos(yaw_prior), sin(yaw_prior), 0]  — toward trailer rear p0_base   = hitch + body_offset * u_base          — trailer body centre p0_cloud  = R_sensor_from_bl * p0_base + t_sensor_from_bl Note: u_base points FROM hitch TOWARD the trailer rear (opposite to tractor +x). The 2D PCA principal direction should be near u_cloud when aligned ──
KinematicPrior
TrailerEstimatorNode::buildKinematicPrior(double theta) const
{
  KinematicPrior pr;
  pr.yaw_prior = normalizeAngle(kPi - theta);
  pr.u_base = Eigen::Vector3d(std::cos(pr.yaw_prior), std::sin(pr.yaw_prior), 0.0);
  pr.n_base = Eigen::Vector3d(-pr.u_base.y(),  pr.u_base.x(), 0.0);
  pr.k_base = Eigen::Vector3d::UnitZ();

  pr.hitch_base = Eigen::Vector3d(hitch_base_x_, hitch_base_y_, hitch_base_z_);
  pr.p0_base    = pr.hitch_base + trailer_body_offset_ * pr.u_base;

  // Transform basis vectors and origin to sensor frame (static TF, precomputed).
  // p_sensor = R_sensor_from_bl * p_base + t_sensor_from_bl
  pr.p0_cloud = R_sensor_from_bl_ * pr.p0_base + t_sensor_from_bl_;
  pr.u_cloud  = R_sensor_from_bl_ * pr.u_base;
  pr.n_cloud  = R_sensor_from_bl_ * pr.n_base;
  pr.k_cloud  = R_sensor_from_bl_ * pr.k_base;
  return pr;
}

// ── transformAndCropRoi — batch-transform cloud to map frame, then crop to trailer ROI. The ROI is an oriented box centred on the predicted trailer pose (from EKF state). Axes: u = [cos(yaw), sin(yaw), 0]     longitudinal (s) n = [-sin(yaw), cos(yaw), 0]    lateral (l) k = [0, 0, 1]                   vertical (h) Uses vectorised Eigen matrix multiplication — no per-point TF calls ──
std::vector<LocalPoint> TrailerEstimatorNode::transformAndCropRoi(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const Eigen::Isometry3d & T_sensor_map,
  const State10d & x_prior) const
{
  const double yaw = x_prior(idx::kYaw);
  const Eigen::Vector3d p0(x_prior(idx::kX), x_prior(idx::kY), x_prior(idx::kZ));

  // Oriented basis vectors of the trailer frame in map.
  const Eigen::Vector3d u(std::cos(yaw), std::sin(yaw), 0.0);
  const Eigen::Vector3d n(-std::sin(yaw), std::cos(yaw), 0.0);
  const Eigen::Vector3d k(0.0, 0.0, 1.0);

  // Rotation part of T_sensor_map and translation — applied per-point without allocation.
  const Eigen::Matrix3d R_s2m = T_sensor_map.rotation();
  const Eigen::Vector3d t_s2m = T_sensor_map.translation();

  std::vector<LocalPoint> out;
  out.reserve(512);

  sensor_msgs::PointCloud2ConstIterator<float> it_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> it_y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> it_z(cloud, "z");

  for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
    const Eigen::Vector3d p_sensor(
      static_cast<double>(*it_x),
      static_cast<double>(*it_y),
      static_cast<double>(*it_z));

    if (!p_sensor.array().isFinite().all()) { continue; }

    // Transform to map frame: p_map = R·p_sensor + t (vectorized, no branch).
    const Eigen::Vector3d p_map = R_s2m * p_sensor + t_s2m;

    // Project into trailer-oriented local frame.
    const Eigen::Vector3d q = p_map - p0;
    const double s = q.dot(u);
    const double l = q.dot(n);
    const double h = q.dot(k);

    // ROI gate — asymmetric along s to match the physical trailer body.
    //
    // Because model points are stored with negated s (trailer extends in -u direction),
    // a trailer-body point in map satisfies s = q·u < 0.
    //   roi_s_lo  =  -(s_rear  + 0.30 m margin)   ← past trailer far end
    //   roi_s_hi  =  -(s_front - 0.20 m margin)   ← slightly past hitch connection
    //
    // This replaces the old symmetric ±roi_half_s_ which only captured the first
    // roi_half_s_ metres from the hitch and missed the trailer rear seen by RS-Airy.
    const double roi_s_lo = -(model_.s_rear  + 0.30);
    const double roi_s_hi = -(model_.s_front - 0.20);
    if (s < roi_s_lo || s > roi_s_hi ||
        std::abs(l) > roi_half_l_ ||
        std::abs(h) > roi_half_h_) { continue; }

    out.push_back({p_map, s, l, h});
  }
  return out;
}

// ── ransacGroundPlane — RANSAC plane fit with vertical-normal constraint. Plane equation: n'·p + d = 0, |n| = 1. Normal must be within ground_max_tilt_rad_ of [0,0,1] (rejects walls). Returns a boolean inlier mask over pts ──
std::vector<bool> TrailerEstimatorNode::ransacGroundPlane(
  const std::vector<LocalPoint> & pts,
  double inlier_thresh_m) const
{
  const int N = static_cast<int>(pts.size());
  if (N < 3) { return std::vector<bool>(static_cast<std::size_t>(N), false); }

  // Mersenne-Twister PRNG with fixed seed for reproducibility (deterministic).
  std::mt19937 rng(42u);
  std::uniform_int_distribution<int> dist(0, N - 1);

  Eigen::Vector3d best_n{0.0, 0.0, 1.0};
  double          best_d{0.0};
  int             best_inliers{0};

  const double cos_max_tilt = std::cos(ground_max_tilt_rad_);

  for (int iter = 0; iter < ground_ransac_iters_; ++iter) {
    // Sample 3 distinct points.
    const int i0 = dist(rng), i1 = dist(rng), i2 = dist(rng);
    if (i0 == i1 || i1 == i2 || i0 == i2) { continue; }

    const Eigen::Vector3d & p0 = pts[i0].p_map;
    const Eigen::Vector3d & p1 = pts[i1].p_map;
    const Eigen::Vector3d & p2 = pts[i2].p_map;

    // Plane normal via cross product — [MV] Chapter 3 line/plane fitting.
    Eigen::Vector3d n_cand = (p1 - p0).cross(p2 - p0);
    const double n_len = n_cand.norm();
    if (n_len < 1e-9) { continue; }  // degenerate (collinear points)
    n_cand /= n_len;

    // Enforce upward-facing normal (ground constraint).
    if (n_cand.z() < 0.0) { n_cand = -n_cand; }

    // Reject if tilted too far from vertical.
    if (n_cand.z() < cos_max_tilt) { continue; }

    const double d_cand = -n_cand.dot(p0);

    // Count inliers: points with |n·p + d| < threshold.
    int inlier_count = 0;
    for (const LocalPoint & lp : pts) {
      if (std::abs(n_cand.dot(lp.p_map) + d_cand) < inlier_thresh_m) { ++inlier_count; }
    }

    if (inlier_count > best_inliers) {
      best_inliers = inlier_count;
      best_n       = n_cand;
      best_d       = d_cand;
    }
  }

  // Build inlier mask.
  std::vector<bool> mask(static_cast<std::size_t>(N), false);
  if (best_inliers < 3) { return mask; }  // no plane found — keep all (conservative)

  for (int i = 0; i < N; ++i) {
    mask[static_cast<std::size_t>(i)] =
      (std::abs(best_n.dot(pts[i].p_map) + best_d) < inlier_thresh_m);
  }
  return mask;
}

// ── ransacLine3d — RANSAC 3D line fit followed by SVD-based least-squares refit. For k iterations: sample 2 points → define line direction d = normalize(p1 - p0) count inliers: points p with ‖(p - p0) - ((p - p0)·d)d‖ < threshold After convergence: refit direction via SVD on centred inlier matrix. Cramér–Rao analogue for the residual variance: σ²_line = residual_variance / n_inliers → used to build R_lines ──
TrailerEstimatorNode::RansacLine3d TrailerEstimatorNode::ransacLine3d(
  const std::vector<Eigen::Vector3d> & pts,
  double inlier_thresh_m,
  int max_iters) const
{
  RansacLine3d result;
  const int N = static_cast<int>(pts.size());
  if (N < 4) { return result; }

  std::mt19937 rng(137u);
  std::uniform_int_distribution<int> dist(0, N - 1);
  const double thresh2 = inlier_thresh_m * inlier_thresh_m;

  std::vector<std::size_t> best_inlier_idx;
  best_inlier_idx.reserve(static_cast<std::size_t>(N));

  for (int iter = 0; iter < max_iters; ++iter) {
    const int ia = dist(rng), ib = dist(rng);
    if (ia == ib) { continue; }

    const Eigen::Vector3d & pa = pts[ia];
    const Eigen::Vector3d   d_cand = (pts[ib] - pa).normalized();
    if (d_cand.norm() < 1e-9) { continue; }

    // Count inliers: point-to-line distance² = ‖q - (q·d)d‖² where q = p - pa.
    std::vector<std::size_t> inlier_idx;
    inlier_idx.reserve(static_cast<std::size_t>(N));
    for (int i = 0; i < N; ++i) {
      const Eigen::Vector3d q   = pts[i] - pa;
      const Eigen::Vector3d perp = q - q.dot(d_cand) * d_cand;
      if (perp.squaredNorm() < thresh2) {
        inlier_idx.push_back(static_cast<std::size_t>(i));
      }
    }

    if (inlier_idx.size() > best_inlier_idx.size()) {
      best_inlier_idx = std::move(inlier_idx);
    }
  }

  const double min_inliers =
    line_min_inlier_frac_ * static_cast<double>(N);
  if (static_cast<double>(best_inlier_idx.size()) < min_inliers || best_inlier_idx.size() < 4) {
    return result;  // not enough inliers — line not found
  }

  // ── SVD least-squares refit on inliers ──
  // Centroid of inliers.
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (std::size_t idx : best_inlier_idx) { centroid += pts[idx]; }
  centroid /= static_cast<double>(best_inlier_idx.size());

  // Build centred data matrix A (n×3).
  Eigen::MatrixXd A(static_cast<Eigen::Index>(best_inlier_idx.size()), 3);
  for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(best_inlier_idx.size()); ++i) {
    A.row(i) = (pts[best_inlier_idx[static_cast<std::size_t>(i)]] - centroid).transpose();
  }

  // SVD: A = U·S·V'.  Direction = first column of V (largest singular value).
  // [MV] p. 592: line direction is the right singular vector corresponding to σ_max.
  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinV);
  const Eigen::Vector3d dir = svd.matrixV().col(0).normalized();

  // Residual variance: mean squared distance from each inlier to the line.
  double rss = 0.0;
  for (std::size_t idx : best_inlier_idx) {
    const Eigen::Vector3d q    = pts[idx] - centroid;
    const Eigen::Vector3d perp = q - q.dot(dir) * dir;
    rss += perp.squaredNorm();
  }
  const double var = rss / static_cast<double>(best_inlier_idx.size());

  result.dir               = dir;
  result.p0                = centroid;
  result.inliers           = best_inlier_idx;
  result.residual_variance = var;
  result.valid             = true;
  return result;
}

// ── runIcp — constrained point-to-model ICP. Optimises [x, y, yaw] (3 DOF) in the map frame. z, roll, pitch are fixed: z from EKF state, roll/pitch from IMU. At each iteration: 1. Transform model points to map frame at current [x, y, yaw] estimate. 2. Find nearest LiDAR point within icp_max_corr_dist_. 3. Compute 2×1 residuals r_i = p_lidar - p_model. 4. Jacobian J_i (2×3) wrt [x, y, yaw]: J_i = [1, 0, -sin(yaw)·m_s - cos(yaw)·m_l; 0, 1,  cos(yaw)·m_s - sin(yaw)·m_l] where (m_s, m_l) are model point coords in local (s,l) frame. 5. Gauss-Newton update: H = Σ J_i'·J_i, g = Σ J_i'·r_i, δ = H^{-1}·g. 6. Convergence when ‖δ‖ < icp_convergence_tol_. ICP covariance (Hessian-based, [TR] eq. 6.9 analog): Σ_icp = (Σ J_i'·J_i)^{-1}  (computed at convergence) ──
TrailerEstimatorNode::IcpResult TrailerEstimatorNode::runIcp(
  const std::vector<LocalPoint> & pts_map,
  const State10d & x_init) const
{
  IcpResult result;
  if (static_cast<int>(pts_map.size()) < icp_min_inliers_) { return result; }

  // Initial estimate from EKF state.
  double cx  = x_init(idx::kX);
  double cy  = x_init(idx::kY);
  const double cz  = x_init(idx::kZ);     // fixed
  const double cr  = imu_roll_.load(std::memory_order_relaxed);   // fixed
  const double cp  = imu_pitch_.load(std::memory_order_relaxed);  // fixed
  double cyaw = x_init(idx::kYaw);

  // Pre-build rotation from roll/pitch (applied to all model points each iter).
  // Full 3D rotation: Rz(yaw)·Ry(pitch)·Rx(roll).
  auto makeRot3 = [](double roll, double pitch, double yaw) -> Eigen::Matrix3d {
    return (Eigen::AngleAxisd(yaw,   Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX())).toRotationMatrix();
  };

  // Cache LiDAR points as a plain Eigen matrix for fast nearest-neighbour search.
  const int Npts = static_cast<int>(pts_map.size());
  Eigen::MatrixXd lidar_xy(2, Npts);
  for (int i = 0; i < Npts; ++i) {
    lidar_xy(0, i) = pts_map[i].p_map.x();
    lidar_xy(1, i) = pts_map[i].p_map.y();
  }

  const int Nmodel = static_cast<int>(model_.pts_local.size());
  if (Nmodel == 0) { return result; }

  // Hessian accumulator (3×3) and gradient (3×1).
  Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
  Eigen::Vector3d g = Eigen::Vector3d::Zero();
  double total_error = 0.0;
  int    n_inliers   = 0;

  for (int iter = 0; iter < icp_max_iters_; ++iter) {
    H.setZero();
    g.setZero();
    total_error = 0.0;
    n_inliers   = 0;

    const Eigen::Matrix3d R_world = makeRot3(cr, cp, cyaw);
    const Eigen::Vector3d t_world(cx, cy, cz);

    for (int mi = 0; mi < Nmodel; ++mi) {
      // Transform model point to map frame.
      const Eigen::Vector3d p_model_map = R_world * model_.pts_local[mi] + t_world;

      // Find nearest LiDAR point in XY (ignore Z for the nearest-neighbour search).
      const Eigen::Vector2d p_xy(p_model_map.x(), p_model_map.y());
      const Eigen::Array<double, 2, Eigen::Dynamic> diff =
        lidar_xy.colwise() - p_xy;
      const Eigen::VectorXd dist2 = diff.colwise().squaredNorm().matrix().transpose();
      Eigen::Index nn_idx;
      const double nn_dist2 = dist2.minCoeff(&nn_idx);

      if (nn_dist2 > icp_max_corr_dist_ * icp_max_corr_dist_) { continue; }

      // 2D residual: r = p_lidar_xy - p_model_map_xy.
      const double rx = pts_map[nn_idx].p_map.x() - p_model_map.x();
      const double ry = pts_map[nn_idx].p_map.y() - p_model_map.y();

      // Model point in local trailer (s, l) before applying yaw rotation.
      const double m_s = model_.pts_local[mi].x();
      const double m_l = model_.pts_local[mi].y();

      // Jacobian row — ∂(p_model_xy)/∂(x, y, yaw):
      // ∂p_x/∂yaw = -sin(yaw)·m_s - cos(yaw)·m_l
      // ∂p_y/∂yaw =  cos(yaw)·m_s - sin(yaw)·m_l
      const double cyaw_cos = std::cos(cyaw);
      const double cyaw_sin = std::sin(cyaw);
      Eigen::Matrix<double, 2, 3> J_i;
      J_i << 1.0, 0.0, -cyaw_sin * m_s - cyaw_cos * m_l,
             0.0, 1.0,  cyaw_cos * m_s - cyaw_sin * m_l;

      // Robust Huber weight: w=1 inside, w=d_max/d outside.
      const double d = std::sqrt(nn_dist2);
      constexpr double kHuberDelta = 0.2;  // m
      const double w = (d < kHuberDelta) ? 1.0 : (kHuberDelta / d);

      H += w * J_i.transpose() * J_i;
      g += w * J_i.transpose() * Eigen::Vector2d(rx, ry);
      total_error += d;
      ++n_inliers;
    }

    if (n_inliers < icp_min_inliers_) { break; }

    // Solve H·δ = g  using Cholesky (SPD guaranteed by Gauss-Newton structure).
    const Eigen::LLT<Eigen::Matrix3d> llt(H);
    if (llt.info() != Eigen::Success) { break; }
    const Eigen::Vector3d delta = llt.solve(g);

    cx   += delta(0);
    cy   += delta(1);
    cyaw += delta(2);
    cyaw  = normalizeAngle(cyaw);

    if (delta.norm() < icp_convergence_tol_) { break; }
  }

  if (n_inliers < icp_min_inliers_) { return result; }

  // ICP covariance: Σ = H^{-1} from the final Hessian.
  // [TR] eq. 6.9 analog: uncertainty inversely proportional to information.
  const Eigen::LLT<Eigen::Matrix3d> llt_final(H);
  if (llt_final.info() != Eigen::Success) { return result; }
  // H^{-1} via Cholesky: LLT decomposition of H, solve I → H^{-1}.
  const Cov3d cov_icp = llt_final.solve(Eigen::Matrix3d::Identity());

  result.pose(0) = cx;
  result.pose(1) = cy;
  result.pose(2) = normalizeAngle(cyaw);
  result.cov     = cov_icp;
  result.alignment_error = total_error / static_cast<double>(n_inliers);
  result.inlier_ratio    = static_cast<double>(n_inliers) /
                           static_cast<double>(Nmodel);
  result.valid = true;
  return result;
}

// ── updatePcaFilter — EMA filter identical to trailer_pose_node V4.0. Called from the RS-Airy LiDAR callback (single-threaded per sensor). pca_mutex_ must NOT be held by the caller ──
void TrailerEstimatorNode::updatePcaFilter(const PcaMeasurement & m)
{
  std::lock_guard<std::mutex> lk(pca_mutex_);

  if (!pca_state_.ok) {
    pca_state_.yaw_corr = m.yaw_valid   ? m.yaw_corr_used  : 0.0;
    pca_state_.pitch    = m.pitch_valid ? m.pitch_used     : 0.0;
    pca_state_.roll     = m.roll_valid  ? m.roll_used      : 0.0;
    pca_state_.ds       = m.yaw_valid   ? m.corr_s         : 0.0;
    pca_state_.dl       = m.yaw_valid   ? m.corr_l         : 0.0;
    pca_state_.dh       = m.yaw_valid   ? m.corr_h         : 0.0;
    pca_state_.ok = true;
    return;
  }

  bool yaw_ok = m.yaw_valid;
  if (yaw_ok && std::abs(normalizeAngle(m.yaw_corr_used - pca_state_.yaw_corr)) > pca_max_yaw_jump_) {
    yaw_ok = false;
  }

  if (yaw_ok) {
    const double e = normalizeAngle(m.yaw_corr_used - pca_state_.yaw_corr);
    pca_state_.yaw_corr = normalizeAngle(pca_state_.yaw_corr + alpha_yaw_ * e);
  } else {
    pca_state_.yaw_corr *= yaw_decay_;
  }

  if (m.pitch_valid) {
    pca_state_.pitch = (1.0 - alpha_pitch_) * pca_state_.pitch + alpha_pitch_ * m.pitch_used;
  } else {
    pca_state_.pitch *= pitch_decay_;
  }
  pca_state_.roll = 0.0;  // roll disabled by default

  if (m.yaw_valid) {
    pca_state_.ds = (1.0 - alpha_position_) * pca_state_.ds + alpha_position_ * m.corr_s;
    pca_state_.dl = (1.0 - alpha_position_) * pca_state_.dl + alpha_position_ * m.corr_l;
    pca_state_.dh = (1.0 - alpha_position_) * pca_state_.dh + alpha_position_ * m.corr_h;
  } else {
    pca_state_.ds *= position_decay_;
    pca_state_.dl *= position_decay_;
    pca_state_.dh *= position_decay_;
  }
}

// ── filteredPositionBase — filtered trailer body centre in base_link ──
Eigen::Vector3d TrailerEstimatorNode::filteredPositionBase(const KinematicPrior & prior) const
{
  std::lock_guard<std::mutex> lk(pca_mutex_);
  return prior.p0_base
    + pca_state_.ds * prior.u_base
    + pca_state_.dl * prior.n_base
    + pca_state_.dh * prior.k_base;
}

// ── makeTrailerRotation — Rz(yaw)*Ry(pitch)*Rx(roll), same as trailer_pose_node ──
Eigen::Matrix3d TrailerEstimatorNode::makeTrailerRotation(
  double yaw, double pitch, double roll) noexcept
{
  const Eigen::Vector3d u(std::cos(yaw) * std::cos(pitch),
                           std::sin(yaw) * std::cos(pitch),
                           std::sin(pitch));
  Eigen::Vector3d n(-std::sin(yaw), std::cos(yaw), 0.0);
  Eigen::Vector3d k = u.cross(n).normalized();
  n = k.cross(u).normalized();
  if (std::abs(roll) > 1e-9) {
    const Eigen::Vector3d n0 = n, k0 = k;
    n = std::cos(roll) * n0 + std::sin(roll) * k0;
    k = -std::sin(roll) * n0 + std::cos(roll) * k0;
  }
  Eigen::Matrix3d R;
  R.col(0) = u;
  R.col(1) = n.normalized();
  R.col(2) = k.normalized();
  return R;
}

// ── processLidarCloud — trailer_pose_node V4.0 pipeline adapted for the EKF. Detection is identical to trailer_pose_node V4.0: 1. Cache static TF sensor_frame ← base_link (once). 2. activeTheta: phi_lidar primary, phi_hardware fallback. 3. buildKinematicPrior → ROI in sensor frame (no EKF dependency). 4. cropToOrientedRoi → ground quantile filter → centroid voxel. 5. SelfAdjointEigenSolver 2D PCA → quality gates. 6. updatePcaFilter (EMA with decay-to-prior on invalid frames). 7. Convert EMA-filtered pose to map frame → push LidarMeas ──
void TrailerEstimatorNode::processLidarCloud(
  const sensor_msgs::msg::PointCloud2 & cloud,
  SpscRingBuffer<LidarMeas, 32> & queue,
  const std::string & sensor_name)
{
  if (sensor_name != "rsairy") { return; }

  const rclcpp::Time stamp(cloud.header.stamp);

  // ── Step 1: Cache static TF sensor ← base_link (one-time). ──
  if (!cacheStaticTf(cloud.header.frame_id)) { return; }

  // ── Step 2: activeTheta — phi_lidar primary, phi_hardware fallback. ──
  double theta{0.0};
  {
    std::lock_guard<std::mutex> lk(mutex_phi_);
    if (phi_lidar_valid_) {
      const double age = std::abs((stamp - phi_lidar_stamp_).seconds());
      if (age <= phi_max_age_s_) {
        theta = phi_lidar_;
      } else if (phi_hardware_valid_) {
        theta = phi_hardware_;
      } else {
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,
          "[%s] articulation angle stale (%.2f s) — skip", sensor_name.c_str(), age);
        return;
      }
    } else if (phi_hardware_valid_) {
      theta = phi_hardware_;
    } else {
      RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,
        "[%s] no articulation angle yet — skip", sensor_name.c_str());
      return;
    }
  }

  // ── Step 3: Build kinematic prior (trailer_pose_node V4.0 formula). ──
  const KinematicPrior prior = buildKinematicPrior(theta);

  // ── Step 4: Crop oriented ROI in sensor frame (no dynamic TF needed). ──
  // Symmetric ±roi_half_x_ along s, ±roi_half_y_ along l, ±roi_half_z_ along h.
  struct SensorPt { Eigen::Vector3d p; double s, l, h; };
  std::vector<SensorPt> roi_pts;
  roi_pts.reserve(2048);

  {
    sensor_msgs::PointCloud2ConstIterator<float> it_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(cloud, "z");
    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
      const double px = static_cast<double>(*it_x);
      const double py = static_cast<double>(*it_y);
      const double pz = static_cast<double>(*it_z);
      if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz)) { continue; }

      const Eigen::Vector3d p(px, py, pz);
      const Eigen::Vector3d q = p - prior.p0_cloud;
      const double s = q.dot(prior.u_cloud);
      const double l = q.dot(prior.n_cloud);
      const double h = q.dot(prior.k_cloud);

      if (std::abs(s) > roi_half_s_ ||
          std::abs(l) > roi_half_l_ ||
          std::abs(h) > roi_half_h_) { continue; }
      roi_pts.push_back({p, s, l, h});
    }
  }

  if (static_cast<int>(roi_pts.size()) < static_cast<int>(pca_min_points_)) {
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,
      "[%s] ROI sparse (%zu pts < %d) — skip",
      sensor_name.c_str(), roi_pts.size(), static_cast<int>(pca_min_points_));
    return;
  }

  // ── Step 5: Ground removal by height quantile (trailer_pose_node style). ──
  {
    std::vector<double> hvec;
    hvec.reserve(roi_pts.size());
    for (const auto & pt : roi_pts) { hvec.push_back(pt.h); }
    const auto it_q = hvec.begin() +
      static_cast<ptrdiff_t>(static_cast<double>(hvec.size()) * pca_ground_quantile_);
    std::nth_element(hvec.begin(), it_q, hvec.end());
    const double h_thresh = *it_q + pca_ground_margin_;
    roi_pts.erase(
      std::remove_if(roi_pts.begin(), roi_pts.end(),
        [h_thresh](const SensorPt & pt) { return pt.h < h_thresh; }),
      roi_pts.end());
  }

  if (static_cast<int>(roi_pts.size()) < static_cast<int>(pca_min_points_after_ground_)) {
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,
      "[%s] after ground: %zu pts — skip", sensor_name.c_str(), roi_pts.size());
    return;
  }

  // ── Step 6: Centroid voxel downsample (trailer_pose_node style). ──
  if (voxel_size_ > 1e-6) {
    struct VAccum { double s{0}, l{0}, h{0}; Eigen::Vector3d p{Eigen::Vector3d::Zero()}; int n{0}; };
    const double inv = 1.0 / voxel_size_;
    std::unordered_map<std::size_t, VAccum> vm;
    vm.reserve(roi_pts.size());
    for (const SensorPt & pt : roi_pts) {
      const long ix = static_cast<long>(std::floor(pt.s * inv));
      const long iy = static_cast<long>(std::floor(pt.l * inv));
      const long iz = static_cast<long>(std::floor(pt.h * inv));
      const std::size_t key =
        static_cast<std::size_t>(ix * 73856093L ^ iy * 19349663L ^ iz * 83492791L);
      auto & a = vm[key];
      a.p += pt.p; a.s += pt.s; a.l += pt.l; a.h += pt.h; ++a.n;
    }
    roi_pts.clear();
    roi_pts.reserve(vm.size());
    for (const auto & kv : vm) {
      const auto & a = kv.second;
      if (a.n <= 0) { continue; }
      const double inv_n = 1.0 / static_cast<double>(a.n);
      roi_pts.push_back({a.p * inv_n, a.s * inv_n, a.l * inv_n, a.h * inv_n});
    }
  }

  const int N = static_cast<int>(roi_pts.size());
  if (N < static_cast<int>(pca_min_points_after_ground_)) { return; }

  // ── Step 7: Collect (s,l,h) vectors for PCA + corrections. ──
  std::vector<double> svec, lvec, hvec2;
  svec.reserve(N); lvec.reserve(N); hvec2.reserve(N);
  double mean_s = 0.0, mean_l = 0.0;
  for (const auto & pt : roi_pts) {
    svec.push_back(pt.s); lvec.push_back(pt.l); hvec2.push_back(pt.h);
    mean_s += pt.s; mean_l += pt.l;
  }
  mean_s /= N; mean_l /= N;

  // Span via 5th–95th percentile (same as trailer_pose_node).
  {
    std::vector<double> sv = svec;
    const auto q05 = sv.begin() + static_cast<ptrdiff_t>(sv.size() * 0.05);
    const auto q95 = sv.begin() + static_cast<ptrdiff_t>(sv.size() * 0.95);
    std::nth_element(sv.begin(), q05, sv.end());
    const double s05 = *q05;
    std::nth_element(sv.begin(), q95, sv.end());
    const double s95 = *q95;
    const double span_s = std::max(0.0, s95 - s05);
    if (span_s < pca_min_span_s_) {
      RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,
        "[%s] span_s=%.3f < %.3f — skip", sensor_name.c_str(), span_s, pca_min_span_s_);
      return;
    }
  }

  // ── Step 8: 2D PCA via SelfAdjointEigenSolver (same as trailer_pose_node). ─
  double c_ss = 0, c_sl = 0, c_ll = 0;
  for (const auto & pt : roi_pts) {
    const double ds = pt.s - mean_s, dl = pt.l - mean_l;
    c_ss += ds * ds; c_sl += ds * dl; c_ll += dl * dl;
  }
  c_ss /= N; c_sl /= N; c_ll /= N;

  Eigen::Matrix2d C; C << c_ss, c_sl, c_sl, c_ll;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(C);

  PcaMeasurement pm;
  pm.n_pts = static_cast<std::size_t>(N);

  if (es.info() == Eigen::Success) {
    const double lam_min = std::max(es.eigenvalues()(0), 1e-9);
    const double lam_max = std::max(es.eigenvalues()(1), 1e-9);
    pm.pca_ratio = lam_max / lam_min;

    Eigen::Vector2d v = es.eigenvectors().col(1);
    if (v.x() < 0.0) { v = -v; }
    pm.yaw_corr_raw = normalizeAngle(std::atan2(v.y(), v.x()));

    // Compute span for logging (re-use sorted svec).
    std::vector<double> sv2 = svec;
    auto q05 = sv2.begin() + static_cast<ptrdiff_t>(sv2.size() * 0.05);
    auto q95 = sv2.begin() + static_cast<ptrdiff_t>(sv2.size() * 0.95);
    std::nth_element(sv2.begin(), q05, sv2.end()); const double s05v = *q05;
    std::nth_element(sv2.begin(), q95, sv2.end()); const double s95v = *q95;
    pm.span_s = std::max(0.0, s95v - s05v);

    const bool enough_pts  = N >= static_cast<int>(pca_min_points_after_ground_);
    const bool enough_span = pm.span_s >= pca_min_span_s_;
    const bool pca_strong  = pm.pca_ratio >= pca_min_ratio_;
    const bool yaw_small   = std::abs(pm.yaw_corr_raw) <= pca_max_yaw_correction_;
    pm.yaw_valid = enough_pts && enough_span && pca_strong && yaw_small;
    if (pm.yaw_valid) {
      pm.yaw_corr_used = clamp(pm.yaw_corr_raw, -pca_max_yaw_correction_, pca_max_yaw_correction_);
    }
  }

  // Position corrections via median (robust, same as trailer_pose_node).
  auto nth_val = [](std::vector<double> v, double q) -> double {
    if (v.empty()) { return 0.0; }
    const auto it = v.begin() + static_cast<ptrdiff_t>(v.size() * q);
    std::nth_element(v.begin(), it, v.end());
    return *it;
  };
  pm.corr_s = clamp(nth_val(svec,  0.50), -pca_max_pos_s_, pca_max_pos_s_);
  pm.corr_l = clamp(nth_val(lvec,  0.50), -pca_max_pos_l_, pca_max_pos_l_);
  pm.corr_h = clamp(nth_val(hvec2, 0.50), -pca_max_pos_h_, pca_max_pos_h_);

  // Pitch from linear fit (h = a*s + b).
  // Simplified: slope = cov(s,h)/var(s).
  if (pm.span_s >= pca_min_span_s_) {
    double mean_h = 0.0;
    for (double hv : hvec2) { mean_h += hv; }
    mean_h /= N;
    double c_sh = 0.0, c_ss2 = 0.0;
    for (int i = 0; i < N; ++i) {
      c_sh  += (svec[i] - mean_s) * (hvec2[i] - mean_h);
      c_ss2 += (svec[i] - mean_s) * (svec[i] - mean_s);
    }
    if (c_ss2 > 1e-9) {
      const double slope = c_sh / c_ss2;
      pm.pitch_raw = std::atan(slope);
      pm.pitch_valid = std::abs(pm.pitch_raw) <= 0.12;
      if (pm.pitch_valid) { pm.pitch_used = pm.pitch_raw; }
    }
  }

  pm.confidence = clamp(
    (pm.pca_ratio > 0 ? std::min(pm.pca_ratio / 10.0, 1.0) : 0.0) *
    (pm.yaw_valid ? 1.0 : 0.25), 0.0, 1.0);
  pm.measurement_valid = pm.yaw_valid;

  // ── Step 9: EMA filter (same as trailer_pose_node::updateFilteredCorrection). ─
  updatePcaFilter(pm);

  // Diagnostics
  last_lidar_error_.store(std::abs(pm.yaw_corr_raw),              std::memory_order_relaxed);
  last_inlier_ratio_.store(std::min(1.0, pm.pca_ratio / 10.0),   std::memory_order_relaxed);

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
    "[estimator] pca: theta=%.1f° N=%d span=%.2f ratio=%.1f yawCorr=%.2f° pitch=%.1f° conf=%.2f",
    theta * 180.0 / kPi, N, pm.span_s, pm.pca_ratio,
    pm.yaw_corr_raw * 180.0 / kPi, pm.pitch_raw * 180.0 / kPi, pm.confidence);

  // ── Step 10: Save ROI cloud in sensor frame for /trailer/trailer_roi_cloud. ─
  {
    std::vector<LocalPoint> lpts;
    lpts.reserve(roi_pts.size());
    for (const auto & pt : roi_pts) {
      LocalPoint lp; lp.p_map = pt.p; lp.s = pt.s; lp.l = pt.l; lp.h = pt.h;
      lpts.push_back(lp);
    }
    std::lock_guard<std::mutex> lk(mutex_roi_cloud_);
    last_roi_pts_ = lpts;
    last_roi_header_ = cloud.header;  // keep original sensor frame_id
  }

  // ── Step 11: Convert EMA-filtered pose to map frame → push LidarMeas. ──
  // Read EMA state.
  double yaw_corr_f{0.0}, pitch_f{0.0}, ds{0.0}, dl{0.0}, dh{0.0};
  {
    std::lock_guard<std::mutex> lk(pca_mutex_);
    yaw_corr_f = pca_state_.yaw_corr;
    pitch_f    = pca_state_.pitch;
    ds = pca_state_.ds; dl = pca_state_.dl; dh = pca_state_.dh;
  }

  const double yaw_filtered_base = normalizeAngle(prior.yaw_prior + yaw_corr_f);
  const Eigen::Vector3d u_f(std::cos(yaw_filtered_base), std::sin(yaw_filtered_base), 0.0);
  const Eigen::Vector3d n_f(-u_f.y(), u_f.x(), 0.0);

  // Filtered body centre in base_link.
  Eigen::Vector3d p_base =
    prior.hitch_base + trailer_body_offset_ * u_f
    + ds * prior.u_base + dl * prior.n_base;
  p_base.z() = prior.hitch_base.z() + dh;

  // Look up map ← base_link TF.
  std::optional<Eigen::Isometry3d> T_opt = lookupTfToMap("base_link", stamp);
  if (!T_opt) { T_opt = lookupTfToMap("base_link", rclcpp::Time(0)); }
  if (!T_opt) { return; }

  const Eigen::Vector3d p_map = T_opt->operator*(p_base);
  const double tractor_yaw = std::atan2(T_opt->linear()(1, 0), T_opt->linear()(0, 0));
  const double yaw_map = normalizeAngle(tractor_yaw + yaw_filtered_base);

  // Covariance: adaptive, quality-weighted.
  Cov3d R_meas = Cov3d::Zero();
  constexpr double kSigXY = 0.04;
  R_meas(0, 0) = kSigXY * kSigXY;
  R_meas(1, 1) = kSigXY * kSigXY;
  R_meas(2, 2) = pca_r_base_ + pca_r_scale_ / std::max(pm.pca_ratio, 1.0);

  LidarMeas lidar_m;
  lidar_m.stamp           = stamp;
  lidar_m.z << p_map.x(), p_map.y(), yaw_map;
  lidar_m.R               = R_meas;
  lidar_m.alignment_error = std::abs(pm.yaw_corr_raw);
  lidar_m.inlier_ratio    = std::min(1.0, pm.pca_ratio / 10.0);
  lidar_m.valid           = true;

  if (!queue.push(lidar_m)) {
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,
      "[%s] lidar queue full — dropping", sensor_name.c_str());
  }
}

// ── EKF — predict step. Constant-velocity model with velocity decay (prevents drift when unobserved). State transition: F(dt): x_new[i]     = x[i] + v[i]*dt   for i in {x,y,z} yaw_new      = yaw + yaw_rate*dt roll_new     = roll  (assumed slow, driven by kinematic meas) pitch_new    = pitch v_new[i]     = decay * v[i]      for i in {vx,vy,vz,yaw_rate} Process covariance: P = F·P·F' + Q·dt  [TR] eq. 3.5 ──
void TrailerEstimatorNode::ekfPredict(FilterState & state, double dt) const
{
  static_assert(idx::kNDof == 10, "State dimension mismatch");

  if (dt <= 0.0 || dt > 1.0) { return; }  // skip degenerate dt

  // Build F(dt) — sparse, avoid full 10×10 multiplication.
  // Position += velocity * dt.
  state.x(idx::kX) += state.x(idx::kVx) * dt;
  state.x(idx::kY) += state.x(idx::kVy) * dt;
  state.x(idx::kZ) += state.x(idx::kVz) * dt;
  // Yaw integration.
  state.x(idx::kYaw) = normalizeAngle(
    state.x(idx::kYaw) + state.x(idx::kYawRate) * dt);
  // Velocity decay.
  const double d = std::pow(velocity_decay_, dt * 10.0);  // decay per dt
  state.x(idx::kVx)      *= d;
  state.x(idx::kVy)      *= d;
  state.x(idx::kVz)      *= d;
  state.x(idx::kYawRate) *= d;

  // Build F matrix for covariance propagation.
  Mat10d F = Mat10d::Identity();
  F(idx::kX,   idx::kVx)      = dt;
  F(idx::kY,   idx::kVy)      = dt;
  F(idx::kZ,   idx::kVz)      = dt;
  F(idx::kYaw, idx::kYawRate) = dt;
  F(idx::kVx,      idx::kVx)      = d;
  F(idx::kVy,      idx::kVy)      = d;
  F(idx::kVz,      idx::kVz)      = d;
  F(idx::kYawRate, idx::kYawRate) = d;

  // P = F·P·F' + Q·dt  [TR] eq. 3.5.
  state.P = F * state.P * F.transpose() + Q_base_ * dt;
  // Symmetrize to prevent numerical drift.
  state.P = 0.5 * (state.P + state.P.transpose());
}

// ── ekfUpdate — Joseph-form update with Mahalanobis gating. Template parameter M = measurement DOF. Standard EKF update [TR] Algorithm 3.1: ν   = z - H·x                         (innovation) S   = H·P·H' + R                       (innovation covariance) K   = P·H'·S^{-1}                      (Kalman gain) x   = x + K·ν P   = (I - K·H)·P·(I - K·H)' + K·R·K' (Joseph form — PSD guaranteed) Mahalanobis gate [TR] eq. 6.27: γ = ν'·S^{-1}·ν   < χ²_threshold  (reject outliers) ──
template<int M>
bool TrailerEstimatorNode::ekfUpdate(
  FilterState & state,
  const Eigen::Matrix<double, M, 1> & z,
  const Eigen::Matrix<double, M, 10> & H,
  const Eigen::Matrix<double, M, M> & R,
  double chi2_threshold) const
{
  using MatMx10 = Eigen::Matrix<double, M, 10>;
  using MatMxM  = Eigen::Matrix<double, M, M>;
  using VecM    = Eigen::Matrix<double, M, 1>;
  using Mat10xM = Eigen::Matrix<double, 10, M>;

  // Innovation.
  const VecM nu = z - H * state.x;

  // Innovation covariance.
  const MatMxM S = H * state.P * H.transpose() + R;

  // Mahalanobis distance via Cholesky solve — avoids S^{-1} explicitly.
  // γ = ν' · S^{-1} · ν  [TR] eq. 6.27.
  const Eigen::LLT<MatMxM> llt_S(S);
  if (llt_S.info() != Eigen::Success) { return false; }  // S not PD — skip
  const VecM S_inv_nu = llt_S.solve(nu);
  const double mahal = nu.dot(S_inv_nu);

  if (mahal > chi2_threshold) {
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
      "Mahalanobis gate reject: γ=%.2f > χ²=%.2f", mahal, chi2_threshold);
    return false;  // outlier — do not update
  }

  // Kalman gain: K = P·H'·S^{-1}.
  // Computed as: K = (S^{-T}·H·P)' = llt_S.solve(H·P')'.
  const Mat10xM K = (llt_S.solve(H * state.P)).transpose();

  // State update: x = x + K·ν.
  state.x += K * nu;
  state.x(idx::kYaw) = normalizeAngle(state.x(idx::kYaw));

  // Joseph form: P = (I - K·H)·P·(I - K·H)' + K·R·K'  [TR] eq. 3.14.
  // This is numerically superior to P = (I - K·H)·P — guaranteed PSD.
  const Mat10d IKH = Mat10d::Identity() - K * H;
  state.P = IKH * state.P * IKH.transpose() + K * R * K.transpose();

  // Symmetrize and quaternion normalisation is not needed here (RPY state).
  state.P = 0.5 * (state.P + state.P.transpose());

  return true;
}

// Explicit instantiations required because the template is defined in .cpp.
template bool TrailerEstimatorNode::ekfUpdate<3>(
  FilterState &, const Vec3d &, const H3x10 &, const Cov3d &, double) const;
template bool TrailerEstimatorNode::ekfUpdate<4>(
  FilterState &, const Vec4d &, const H4x10 &, const Cov4d &, double) const;

// ── checkAndHandleDivergence — re-initialise filter if it has diverged ──
void TrailerEstimatorNode::checkAndHandleDivergence(FilterState & state)
{
  const double trace = state.P.trace();
  if (trace > divergence_trace_max_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "EKF divergence detected (P.trace=%.1f) — re-initialising from kinematic prior", trace);
    state.initialized = false;  // next kinematic meas will re-init
  }
}

// ── applyKinematicMeas — process one kinematic measurement in the filter thread. Measurement model (H_kinematic): z = [x, y, z, yaw]'  →  selects rows {kX, kY, kZ, kYaw} from state ──
void TrailerEstimatorNode::applyKinematicMeas(
  FilterState & state, const KinematicMeas & meas)
{
  if (!meas.valid) { return; }

  if (!state.initialized) {
    // Cold-start initialisation from kinematic prior.
    state.x.setZero();
    state.x(idx::kX)   = meas.z(0);
    state.x(idx::kY)   = meas.z(1);
    state.x(idx::kZ)   = meas.z(2);
    state.x(idx::kYaw) = meas.z(3);

    // Initial covariance from measurement noise + generous velocity uncertainty.
    state.P.setZero();
    state.P(idx::kX,       idx::kX)       = meas.R(0, 0) + 0.25;
    state.P(idx::kY,       idx::kY)       = meas.R(1, 1) + 0.25;
    state.P(idx::kZ,       idx::kZ)       = meas.R(2, 2) + 0.05;
    state.P(idx::kRoll,    idx::kRoll)    = 0.01;
    state.P(idx::kPitch,   idx::kPitch)   = 0.01;
    state.P(idx::kYaw,     idx::kYaw)     = meas.R(3, 3) + 0.10;
    state.P(idx::kVx,      idx::kVx)      = 1.0;
    state.P(idx::kVy,      idx::kVy)      = 1.0;
    state.P(idx::kVz,      idx::kVz)      = 0.1;
    state.P(idx::kYawRate, idx::kYawRate) = 0.5;

    state.stamp       = meas.stamp;
    state.initialized = true;
    RCLCPP_INFO(get_logger(),
      "EKF cold-start: x=[%.2f, %.2f, %.2f], yaw=%.3f rad",
      state.x(0), state.x(1), state.x(2), state.x(5));
    return;
  }

  // Predict to measurement timestamp.
  const double dt = (meas.stamp - state.stamp).seconds();
  ekfPredict(state, dt);
  state.stamp = meas.stamp;

  // H_kinematic: selects [x, y, z, yaw] from the 10-DOF state.
  H4x10 H = H4x10::Zero();
  H(0, idx::kX)   = 1.0;
  H(1, idx::kY)   = 1.0;
  H(2, idx::kZ)   = 1.0;
  H(3, idx::kYaw) = 1.0;

  // Include IMU roll/pitch as additional measurements on those DOF.
  // We treat them as part of the kinematic measurement with a known variance.
  const double alpha = imu_pitch_.load(std::memory_order_relaxed);
  const double roll  = imu_roll_.load(std::memory_order_relaxed);

  // Yaw wrapping: adjust measurement yaw so it lies within ±π of the current state.
  // Without this, an innovation of ≈2π occurs when yaw crosses the ±π boundary,
  // causing a massive state jump even with Joseph-form update.
  Vec4d z_adj = meas.z;
  z_adj(3) = state.x(idx::kYaw) + normalizeAngle(meas.z(3) - state.x(idx::kYaw));

  // First update: [x, y, z, yaw] from kinematic chain.
  // The kinematic prior is a geometric constraint (URDF + measured phi), not a noisy
  // sensor reading that can produce outliers.  Gate is disabled (infinite threshold).
  ekfUpdate<4>(state, z_adj, H, meas.R, std::numeric_limits<double>::max());

  // Second update: roll and pitch from IMU (separate, independent).
  // H_rp: selects [roll, pitch].
  Eigen::Matrix<double, 2, 10> H_rp = Eigen::Matrix<double, 2, 10>::Zero();
  H_rp(0, idx::kRoll)  = 1.0;
  H_rp(1, idx::kPitch) = 1.0;
  const Eigen::Vector2d z_rp(roll, alpha);
  constexpr double kSigmaImuRp = 0.020 * 0.020;  // MTi-100: 0.020 rad RMS
  const Eigen::Matrix2d R_rp = Eigen::Matrix2d::Identity() * kSigmaImuRp;
  ekfUpdate<2>(state, z_rp, H_rp, R_rp, 11.07);  // χ²(2, p=0.99)

  checkAndHandleDivergence(state);
}

// Explicit instantiation for 2-DOF update (roll/pitch).
template bool TrailerEstimatorNode::ekfUpdate<2>(
  FilterState &,
  const Eigen::Matrix<double, 2, 1> &,
  const Eigen::Matrix<double, 2, 10> &,
  const Eigen::Matrix<double, 2, 2> &,
  double) const;

// ── applyLidarMeas — process one LiDAR ICP measurement in the filter thread. Measurement model (H_lidar): z = [x, y, yaw]'  →  selects rows {kX, kY, kYaw} from state ──
void TrailerEstimatorNode::applyLidarMeas(
  FilterState & state, const LidarMeas & meas)
{
  if (!meas.valid || !state.initialized) { return; }

  const double dt = (meas.stamp - state.stamp).seconds();
  ekfPredict(state, dt);
  state.stamp = meas.stamp;

  H3x10 H = H3x10::Zero();
  H(0, idx::kX)   = 1.0;
  H(1, idx::kY)   = 1.0;
  H(2, idx::kYaw) = 1.0;

  // Yaw wrapping: prevent ≈2π innovation at the ±π boundary.
  Vec3d z_adj = meas.z;
  z_adj(2) = state.x(idx::kYaw) + normalizeAngle(meas.z(2) - state.x(idx::kYaw));
  ekfUpdate<3>(state, z_adj, H, meas.R, chi2_lidar_);
  checkAndHandleDivergence(state);
}

// ── filterThreadFunc — runs at filter_period_ms_, drains all queues. All queues are drained in a single pass; measurements are applied in order. The filter state is updated under state_mutex_ (unique_lock) only while writing, then released — allowing the 50 Hz publish timer to proceed ──
void TrailerEstimatorNode::filterThreadFunc()
{
  const std::chrono::milliseconds period(filter_period_ms_);
  rclcpp::Time last_diag = get_clock()->now();

  while (!stop_flag_.load(std::memory_order_acquire)) {
    const auto t_start = std::chrono::steady_clock::now();

    FilterState local_state;
    {
      std::shared_lock<std::shared_mutex> rlock(state_mutex_);
      local_state = filter_state_;  // snapshot
    }

    // Drain kinematic queue (highest priority, 100 Hz max).
    KinematicMeas km;
    while (kinematic_queue_.pop(km)) {
      applyKinematicMeas(local_state, km);
    }

    // Drain RS-Airy ICP queue.
    LidarMeas lm;
    while (rsairy_queue_.pop(lm)) {
      applyLidarMeas(local_state, lm);
    }

    // Drain Hesai ICP queue.
    while (hesai_queue_.pop(lm)) {
      applyLidarMeas(local_state, lm);
    }

    // Write back updated state.
    {
      std::unique_lock<std::shared_mutex> wlock(state_mutex_);
      filter_state_ = local_state;
    }

    // Publish diagnostics at ~1 Hz.
    const rclcpp::Time now = get_clock()->now();
    if ((now - last_diag).seconds() > 1.0) {
      const auto t_end = std::chrono::steady_clock::now();
      const double lat_ms =
        std::chrono::duration<double, std::milli>(t_end - t_start).count();
      publishDiagnostics(
        lat_ms,
        last_lidar_error_.load(std::memory_order_relaxed),
        last_inlier_ratio_.load(std::memory_order_relaxed));
      last_diag = now;
    }

    std::this_thread::sleep_for(period);
  }
}

// ── publishTimerCallback — 50 Hz, reads filter state and publishes all outputs ──
void TrailerEstimatorNode::publishTimerCallback()
{
  const rclcpp::Time now = get_clock()->now();

  // Markers depend only on phi + EMA filter — publish even before EKF init.
  {
    FilterState dummy{};
    publishMarkers(dummy, now);
  }

  FilterState state;
  {
    std::shared_lock<std::shared_mutex> lock(state_mutex_);
    if (!filter_state_.initialized) { return; }
    state = filter_state_;  // cheap copy — 10 doubles + 100-element matrix
  }

  publishPose(state);
  publishOdom(state);
  publishHitchAngle(state, now);

  // Publish ROI cloud from last processed LiDAR scan.
  std::vector<LocalPoint> roi_pts;
  std_msgs::msg::Header   roi_hdr;
  {
    std::lock_guard<std::mutex> lock(mutex_roi_cloud_);
    roi_pts = last_roi_pts_;
    roi_hdr = last_roi_header_;
  }
  if (!roi_pts.empty()) {
    publishRoiCloud(roi_pts, roi_hdr);
  }

  // Publish debug scalars.
  {
    std_msgs::msg::Float64 msg;
    msg.data = state.x(idx::kYaw);
    yaw_prior_pub_->publish(msg);
    msg.data = state.x(idx::kPitch);
    pitch_used_pub_->publish(msg);
    msg.data = state.x(idx::kRoll);
    roll_used_pub_->publish(msg);
  }
}

// ── publishPose — /trailer/pose and /trailer/pose_in_map ──
void TrailerEstimatorNode::publishPose(const FilterState & state) const
{
  const rclcpp::Time stamp = state.stamp;
  const Eigen::Quaterniond q(
    (Eigen::AngleAxisd(state.x(idx::kYaw),   Eigen::Vector3d::UnitZ()) *
     Eigen::AngleAxisd(state.x(idx::kPitch), Eigen::Vector3d::UnitY()) *
     Eigen::AngleAxisd(state.x(idx::kRoll),  Eigen::Vector3d::UnitX()))
    .toRotationMatrix());

  // Build covariance: rows/cols {x,y,z,roll,pitch,yaw} → 6×6 from P rows 0-5.
  std::array<double, 36> cov_array{};
  const Eigen::Matrix<double, 6, 6> P6 = state.P.topLeftCorner<6, 6>();
  Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(cov_array.data()) = P6;

  // PoseWithCovarianceStamped.
  geometry_msgs::msg::PoseWithCovarianceStamped pwcs;
  pwcs.header.stamp    = stamp;
  pwcs.header.frame_id = map_frame_;
  pwcs.pose.pose.position.x    = state.x(idx::kX);
  pwcs.pose.pose.position.y    = state.x(idx::kY);
  pwcs.pose.pose.position.z    = state.x(idx::kZ);
  pwcs.pose.pose.orientation.w = q.w();
  pwcs.pose.pose.orientation.x = q.x();
  pwcs.pose.pose.orientation.y = q.y();
  pwcs.pose.pose.orientation.z = q.z();
  pwcs.pose.covariance = cov_array;
  pose_pub_->publish(pwcs);

  // PoseStamped (no covariance).
  geometry_msgs::msg::PoseStamped ps;
  ps.header      = pwcs.header;
  ps.pose        = pwcs.pose.pose;
  pose_in_map_pub_->publish(ps);

  // TF broadcast: map → trailer_body.
  if (broadcast_tf_ && tf_broadcaster_) {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header          = pwcs.header;
    tf_msg.child_frame_id  = trailer_tf_frame_;
    tf_msg.transform.translation.x = state.x(idx::kX);
    tf_msg.transform.translation.y = state.x(idx::kY);
    tf_msg.transform.translation.z = state.x(idx::kZ);
    tf_msg.transform.rotation      = pwcs.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf_msg);
  }
}

// ── publishOdom — /trailer/odom (with velocity from filter state) ──
void TrailerEstimatorNode::publishOdom(const FilterState & state) const
{
  const Eigen::Quaterniond q(
    (Eigen::AngleAxisd(state.x(idx::kYaw),   Eigen::Vector3d::UnitZ()) *
     Eigen::AngleAxisd(state.x(idx::kPitch), Eigen::Vector3d::UnitY()) *
     Eigen::AngleAxisd(state.x(idx::kRoll),  Eigen::Vector3d::UnitX()))
    .toRotationMatrix());

  nav_msgs::msg::Odometry odom;
  odom.header.stamp    = state.stamp;
  odom.header.frame_id = map_frame_;
  odom.child_frame_id  = trailer_tf_frame_;

  odom.pose.pose.position.x    = state.x(idx::kX);
  odom.pose.pose.position.y    = state.x(idx::kY);
  odom.pose.pose.position.z    = state.x(idx::kZ);
  odom.pose.pose.orientation.w = q.w();
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();

  // Velocity (in map frame, not child frame — documented in header comment).
  odom.twist.twist.linear.x  = state.x(idx::kVx);
  odom.twist.twist.linear.y  = state.x(idx::kVy);
  odom.twist.twist.linear.z  = state.x(idx::kVz);
  odom.twist.twist.angular.z = state.x(idx::kYawRate);

  // Pose covariance (6×6 from P[0:6, 0:6]).
  const Eigen::Matrix<double, 6, 6> P6 = state.P.topLeftCorner<6, 6>();
  Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(
    odom.pose.covariance.data()) = P6;

  // Velocity covariance (4 DOF: vx, vy, vz, yaw_rate from P[6:10, 6:10]).
  // Pack into the 6×6 twist covariance at positions [0,1,2,5].
  odom.twist.covariance.fill(0.0);
  odom.twist.covariance[0]  = state.P(idx::kVx,      idx::kVx);
  odom.twist.covariance[7]  = state.P(idx::kVy,      idx::kVy);
  odom.twist.covariance[14] = state.P(idx::kVz,      idx::kVz);
  odom.twist.covariance[35] = state.P(idx::kYawRate, idx::kYawRate);

  odom_pub_->publish(odom);
}

// ── publishMarkers — trailer_pose_node V4.0 style. Drawn in base_link frame (TF does the map conversion for Foxglove). Requires the static TF cache to be ready; silently skips if not. ID 0  "trailer_roi"      — translucent cyan ROI cube (kinematic prior) ID 1  "trailer_skeleton" — prior centre line (blue, thin) ID 2  "trailer_skeleton" — filtered centre line (yellow, thick) ID 3  "trailer_skeleton" — left side rail (green) ID 4  "trailer_skeleton" — right side rail (magenta) ID 5  "trailer_pose"     — arrow at filtered body centre (orange) ID 6  "trailer_debug"    — text overlay with PCA stats ──
void TrailerEstimatorNode::publishMarkers(
  const FilterState & /*state*/, const rclcpp::Time & now) const
{
  // ── activeTheta (same logic as processLidarCloud) ──
  double theta{0.0};
  {
    std::lock_guard<std::mutex> lk(mutex_phi_);
    if (phi_lidar_valid_) {
      const double age = std::abs((now - phi_lidar_stamp_).seconds());
      if (age <= phi_max_age_s_) {
        theta = phi_lidar_;
      } else if (phi_hardware_valid_) {
        theta = phi_hardware_;
      } else {
        return;
      }
    } else if (phi_hardware_valid_) {
      theta = phi_hardware_;
    } else {
      return;
    }
  }

  // ── Kinematic prior in base_link ──
  const KinematicPrior prior = buildKinematicPrior(theta);

  // ── EMA-filtered corrections ──
  double yaw_corr{0.0}, pitch_f{0.0}, roll_f{0.0};
  double ds{0.0}, dl{0.0}, dh{0.0};
  bool pca_ok{false};
  {
    std::lock_guard<std::mutex> lk(pca_mutex_);
    pca_ok    = pca_state_.ok;
    yaw_corr  = pca_state_.yaw_corr;
    pitch_f   = pca_state_.pitch;
    roll_f    = pca_state_.roll;
    ds = pca_state_.ds; dl = pca_state_.dl; dh = pca_state_.dh;
  }

  const double yaw_used = normalizeAngle(prior.yaw_prior + yaw_corr);

  // Filtered direction vector (accounts for pitch).
  const Eigen::Vector3d u = (
    std::cos(pitch_f) * prior.u_base +
    std::sin(pitch_f) * Eigen::Vector3d::UnitZ()).normalized();
  Eigen::Vector3d n(-std::sin(yaw_used), std::cos(yaw_used), 0.0);
  Eigen::Vector3d k = u.cross(n).normalized();
  n = k.cross(u).normalized();
  if (std::abs(roll_f) > 1e-9) {
    const Eigen::Vector3d n0 = n, k0 = k;
    n = std::cos(roll_f) * n0 + std::sin(roll_f) * k0;
  }

  // Filtered body position in base_link.
  const Eigen::Vector3d pos_base = pca_ok
    ? prior.p0_base + ds * prior.u_base + dl * prior.n_base + dh * prior.k_base
    : prior.p0_base;

  // ── Hitch-anchored skeleton points ──
  const Eigen::Vector3d & h = prior.hitch_base;
  const Eigen::Vector3d & up = prior.u_base;  // prior direction

  const Eigen::Vector3d prior_front = h + trailer_front_offset_ * up;
  const Eigen::Vector3d prior_rear  = h + trailer_rear_offset_  * up;
  const Eigen::Vector3d front       = h + trailer_front_offset_ * u;
  const Eigen::Vector3d rear        = h + trailer_rear_offset_  * u;

  visualization_msgs::msg::MarkerArray ma;

  auto baseMarker = [&](const int id, const std::string & ns, const int type) {
    visualization_msgs::msg::Marker m;
    m.header.stamp    = now;
    m.header.frame_id = "base_link";
    m.ns     = ns;
    m.id     = id;
    m.type   = type;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.lifetime = markerLifetime(0.30);
    m.pose.orientation.w = 1.0;
    return m;
  };

  // ── 0. ROI cube (kinematic prior, translucent) ──
  {
    const Eigen::Quaterniond q_prior(Eigen::AngleAxisd(prior.yaw_prior, Eigen::Vector3d::UnitZ()));
    auto cube = baseMarker(0, "trailer_roi", visualization_msgs::msg::Marker::CUBE);
    cube.pose.position.x    = prior.p0_base.x();
    cube.pose.position.y    = prior.p0_base.y();
    cube.pose.position.z    = prior.p0_base.z();
    cube.pose.orientation.x = q_prior.x();
    cube.pose.orientation.y = q_prior.y();
    cube.pose.orientation.z = q_prior.z();
    cube.pose.orientation.w = q_prior.w();
    cube.scale.x = 2.0 * roi_half_s_;
    cube.scale.y = 2.0 * roi_half_l_;
    cube.scale.z = 2.0 * roi_half_h_;
    cube.color.r = 0.0f; cube.color.g = 0.8f;
    cube.color.b = 1.0f; cube.color.a = 0.12f;
    ma.markers.push_back(cube);
  }

  auto makeLine = [&](
    const int id, const std::string & ns,
    const Eigen::Vector3d & a, const Eigen::Vector3d & b,
    const float r, const float g, const float bl, const double width)
  {
    auto m = baseMarker(id, ns, visualization_msgs::msg::Marker::LINE_STRIP);
    m.scale.x = width;
    m.color.r = r; m.color.g = g; m.color.b = bl; m.color.a = 1.0f;
    m.points.push_back(pointMsg(a));
    m.points.push_back(pointMsg(b));
    ma.markers.push_back(m);
  };

  // ── 1. Prior centre line (blue, thin) ──
  makeLine(1, "trailer_skeleton", prior_front, prior_rear, 0.2f, 0.4f, 1.0f, 0.018);

  // ── 2. Filtered centre line (yellow, thick) ──
  makeLine(2, "trailer_skeleton", front, rear, 1.0f, 0.8f, 0.0f, 0.035);

  // ── 3 & 4. Side rails ──
  const double hw = 0.5 * (2.0 * model_.l_half);  // full width / 2
  makeLine(3, "trailer_skeleton", front + hw * n, rear + hw * n, 0.0f, 1.0f, 0.2f, 0.026);
  makeLine(4, "trailer_skeleton", front - hw * n, rear - hw * n, 1.0f, 0.0f, 1.0f, 0.026);

  // ── 5. Arrow at filtered body centre ──
  {
    auto arrow = baseMarker(5, "trailer_pose", visualization_msgs::msg::Marker::ARROW);
    arrow.points.push_back(pointMsg(pos_base));
    arrow.points.push_back(pointMsg(pos_base + 0.60 * u));
    arrow.scale.x = 0.04;
    arrow.scale.y = 0.08;
    arrow.scale.z = 0.08;
    arrow.color.r = 1.0f; arrow.color.g = 0.45f;
    arrow.color.b = 0.0f; arrow.color.a = 1.0f;
    ma.markers.push_back(arrow);
  }

  // ── 6. Debug text ──
  {
    auto text = baseMarker(6, "trailer_debug",
      visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
    text.pose.position.x = pos_base.x();
    text.pose.position.y = pos_base.y();
    text.pose.position.z = pos_base.z() + 0.55;
    text.scale.z = 0.12;
    text.color.r = text.color.g = text.color.b = text.color.a = 1.0f;
    std::ostringstream ss;
    ss.setf(std::ios::fixed, std::ios::floatfield);
    ss.precision(2);
    ss << "theta=" << theta * 180.0 / kPi << "deg"
       << " yaw_prior=" << prior.yaw_prior * 180.0 / kPi << "deg"
       << " yawCorr=" << yaw_corr * 180.0 / kPi << "deg"
       << " pitch=" << pitch_f * 180.0 / kPi << "deg";
    text.text = ss.str();
    ma.markers.push_back(text);
  }

  markers_pub_->publish(ma);
}

// ── publishRoiCloud — /trailer/trailer_roi_cloud as sensor_msgs/PointCloud2 ──
void TrailerEstimatorNode::publishRoiCloud(
  const std::vector<LocalPoint> & pts,
  const std_msgs::msg::Header & header) const
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = header;
  cloud.header.frame_id = map_frame_;  // output always in map frame
  cloud.height    = 1;
  cloud.width     = static_cast<uint32_t>(pts.size());
  cloud.is_dense  = true;
  cloud.is_bigendian = false;
  cloud.point_step   = 12;  // 3 × float32
  cloud.row_step     = cloud.point_step * cloud.width;

  sensor_msgs::msg::PointField fx, fy, fz;
  fx.name = "x"; fx.offset = 0;  fx.datatype = sensor_msgs::msg::PointField::FLOAT32; fx.count = 1;
  fy.name = "y"; fy.offset = 4;  fy.datatype = fx.datatype; fy.count = 1;
  fz.name = "z"; fz.offset = 8;  fz.datatype = fx.datatype; fz.count = 1;
  cloud.fields = {fx, fy, fz};

  cloud.data.resize(static_cast<std::size_t>(cloud.row_step));
  float * dst = reinterpret_cast<float *>(cloud.data.data());
  for (const LocalPoint & lp : pts) {
    *dst++ = static_cast<float>(lp.p_map.x());
    *dst++ = static_cast<float>(lp.p_map.y());
    *dst++ = static_cast<float>(lp.p_map.z());
  }
  roi_cloud_pub_->publish(cloud);
}

// ── publishHitchAngle — back-compute φ from filter state, cross-validate with hardware. φ_estimated = atan2(trailer_y - hitch_y, trailer_x - hitch_x) - tractor_yaw Cross-validation [TR] probabilistic interpretation: |φ_estimated - φ_hardware| > 2·σ_validation  for > hitch_disagreement_timeout_ → WARN diagnostic (possible trailer disengagement or sensor fault) ──
void TrailerEstimatorNode::publishHitchAngle(
  const FilterState & state, const rclcpp::Time & now)
{
  // Get tractor pose in map frame via TF (same source as kinematic prior).
  Eigen::Isometry3d T_tractor = Eigen::Isometry3d::Identity();
  try {
    const geometry_msgs::msg::TransformStamped tf_msg =
      tf_buffer_.lookupTransform(map_frame_, "base_footprint",
        rclcpp::Time(0), rclcpp::Duration::from_seconds(0.05));
    const auto & tr = tf_msg.transform.translation;
    const auto & qr = tf_msg.transform.rotation;
    T_tractor.translation() << tr.x, tr.y, tr.z;
    T_tractor.linear() =
      Eigen::Quaterniond(qr.w, qr.x, qr.y, qr.z).normalized().toRotationMatrix();
  } catch (const tf2::TransformException &) {
    return;
  }

  const double tractor_yaw = rotToRpy(T_tractor.rotation()).z();

  // Back-compute φ from the EKF yaw state.
  // The URDF chain at φ=0, α=0 gives a reference yaw offset yaw_delta_ref_:
  //   trailer_yaw_in_map = tractor_yaw + yaw_delta_ref_ + φ  (to first order)
  // → φ ≈ trailer_yaw_in_map - tractor_yaw - yaw_delta_ref_
  //
  // Using atan2(Δtrailer - Δhitch) fails here because the trailer URDF body
  // origin sits very close to the hitch pivot (Δ ≈ 0), making the vector
  // noise-dominated.  Using the yaw state is correct and numerically stable.
  const double phi_estimated = normalizeAngle(state.x(idx::kYaw) - tractor_yaw - yaw_delta_ref_);

  std_msgs::msg::Float64 phi_msg;
  phi_msg.data = phi_estimated;
  articulation_angle_pub_->publish(phi_msg);

  // Cross-validation: compare with hardware sensor.
  bool hardware_fresh = false;
  double phi_hw = 0.0;
  {
    std::lock_guard<std::mutex> lock(mutex_phi_);
    hardware_fresh = phi_hardware_valid_ &&
                     (now - phi_hardware_stamp_).seconds() < phi_max_age_s_;
    phi_hw = phi_hardware_;
  }

  if (!hardware_fresh) { return; }

  const double disagreement = std::abs(normalizeAngle(phi_estimated - phi_hw));
  if (disagreement > 2.0 * hitch_validation_sigma_) {
    if (!hitch_disagreement_active_) {
      hitch_disagreement_start_  = now;
      hitch_disagreement_active_ = true;
    } else if ((now - hitch_disagreement_start_).seconds() > hitch_disagreement_timeout_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Hitch angle disagreement: vision=%.3f rad, hardware=%.3f rad, diff=%.3f rad (>2σ=%.3f)"
        " — possible trailer disengagement or sensor fault",
        phi_estimated, phi_hw, disagreement, 2.0 * hitch_validation_sigma_);
    }
  } else {
    hitch_disagreement_active_ = false;  // reset timer on agreement
  }
}

// ── publishDiagnostics — Float64 KPI topics at ~1 Hz ──
void TrailerEstimatorNode::publishDiagnostics(
  double latency_ms, double lidar_error_m, double inlier_ratio) const
{
  auto pub = [](const rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr & p, double v) {
    std_msgs::msg::Float64 msg;
    msg.data = v;
    p->publish(msg);
  };
  pub(diag_latency_pub_,     latency_ms);
  pub(diag_lidar_error_pub_, lidar_error_m);
  pub(diag_inlier_ratio_pub_, inlier_ratio);
}

// ── Helpers ──
double TrailerEstimatorNode::normalizeAngle(double a) noexcept
{
  return ::mtt_perception::normalizeAngle(a);
}

double TrailerEstimatorNode::clamp(double x, double lo, double hi) noexcept
{
  return x < lo ? lo : (x > hi ? hi : x);
}

}  // namespace mtt_perception

RCLCPP_COMPONENTS_REGISTER_NODE(mtt_perception::TrailerEstimatorNode)
