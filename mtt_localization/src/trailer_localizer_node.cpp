/// TrailerLocalizerNode — Trailer pose in map frame via SE(3) kinematics
///
/// URDF kinematic chain (base_footprint → MTT_remorque):
///   base_footprint → base_link      : fixed, xyz=(0,0,-0.1)
///   base_link      → Frame_fix      : pitch joint, xyz=(-1.0512, 0.2125, 0.3578),
///                                     rpy=(-π/2,0,0), q=-π/2
///   Frame_fix      → jt_simple      : yaw joint,   xyz=(0, 0.0571, -0.2635),
///                                     rpy=(-π/2,0,0), q=π/2+φ   ← THE VARIABLE
///   jt_simple      → MTT_remorque   : roll joint,  xyz=(0,0,-0.0571),
///                                     rpy=(-π/2,0,-π/2), q=-π/2
///
/// Δ(φ) = T_bf_bl · T_pitch · Trans(xyz_yaw)·RotRPY(rpy_yaw) · Rz(π/2+φ) · T_roll
///       = A · Rz(π/2+φ) · B     (A,B constant, precomputed at construction)

#include "mtt_localization/trailer_localizer_node.hpp"

#include <array>
#include <chrono>
#include <cmath>

#include <Eigen/Geometry>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace mtt_loc
{

// ─────────────────────────────────────────────────────────────────────────────
// Anonymous-namespace helpers
// ─────────────────────────────────────────────────────────────────────────────
namespace
{

/// URDF joint origin transform: Trans(xyz) · RotRPY(rpy)
/// RotRPY = Rz(yaw)·Ry(pitch)·Rx(roll)  (URDF convention)
Eigen::Isometry3d urdfOrigin(
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

/// Full revolute joint transform: Trans(xyz) · RotRPY(rpy) · Rz(q)
Eigen::Isometry3d urdfJoint(
  const Eigen::Vector3d & xyz,
  const Eigen::Vector3d & rpy,
  double q)
{
  return urdfOrigin(xyz, rpy) *
         Eigen::Isometry3d(Eigen::AngleAxisd(q, Eigen::Vector3d::UnitZ()));
}

/// 3×3 skew-symmetric matrix of v
Eigen::Matrix3d skew3(const Eigen::Vector3d & v)
{
  Eigen::Matrix3d S;
  S <<    0.0, -v.z(),  v.y(),
        v.z(),    0.0, -v.x(),
       -v.y(),  v.x(),    0.0;
  return S;
}

/// 6×6 adjoint of T for [position; rotation] convention (matches ROS covariance layout)
///
/// For ξ = [v; ω]:
///   Ad(T) = | R    t^×·R |
///           | 0      R   |
///
/// Transforms a body-frame tangent vector to world frame.
Eigen::Matrix<double, 6, 6> adjoint6(const Eigen::Isometry3d & T)
{
  const Eigen::Matrix3d R = T.rotation();
  const Eigen::Matrix3d tx_R = skew3(T.translation()) * R;

  Eigen::Matrix<double, 6, 6> Ad = Eigen::Matrix<double, 6, 6>::Zero();
  Ad.topLeftCorner<3, 3>()     = R;
  Ad.topRightCorner<3, 3>()    = tx_R;
  Ad.bottomRightCorner<3, 3>() = R;
  return Ad;
}

/// Extract 6×6 covariance from ROS array [x,y,z,rx,ry,rz] → Eigen row-major
Eigen::Matrix<double, 6, 6> covFromRos(
  const std::array<double, 36> & cov_array)
{
  return Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(cov_array.data());
}

/// Write Eigen 6×6 covariance into ROS array (row-major)
void covToRos(
  const Eigen::Matrix<double, 6, 6> & cov,
  std::array<double, 36> & out)
{
  Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(out.data()) = cov;
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// Constructor
// ─────────────────────────────────────────────────────────────────────────────
TrailerLocalizerNode::TrailerLocalizerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("trailer_localizer_node", options)
{
  // ── Declare parameters ──────────────────────────────────────────────
  const double publish_rate =
    declare_parameter("publish_rate", 50.0);

  const std::string fusion_mode_str =
    declare_parameter("fusion_mode", std::string("hardware"));

  sigma_phi_hardware_ =
    declare_parameter("sigma_phi_hardware", 0.008);

  sigma_phi_lidar_ =
    declare_parameter("sigma_phi_lidar", 0.035);

  default_tractor_sigma_xyz_ =
    declare_parameter("default_tractor_sigma_xyz", 0.05);

  default_tractor_sigma_rpy_ =
    declare_parameter("default_tractor_sigma_rpy", 0.01);

  articulation_timeout_ =
    declare_parameter("articulation_timeout", 0.5);

  trailer_tf_frame_ =
    declare_parameter("trailer_tf_frame", std::string("trailer_body"));

  broadcast_tf_ =
    declare_parameter("broadcast_tf", true);

  // Parse fusion mode
  if (fusion_mode_str == "lidar") {
    fusion_mode_ = FusionMode::kLidar;
  } else if (fusion_mode_str == "fused") {
    fusion_mode_ = FusionMode::kFused;
  } else {
    fusion_mode_ = FusionMode::kHardware;
  }

  // ── Precompute URDF kinematic chain ────────────────────────────────
  //
  // Joint origins from robot.urdf.xacro (exact values):
  //
  //   base_footprint → base_link  (fixed)
  //     xyz=(0, 0, -0.1), rpy=(0,0,0)
  //
  //   pitch: base_link → Frame_fixation_articule
  //     xyz=(-1.0511878376018575, 0.2125028310306423, 0.3577510511469179)
  //     rpy=(-π/2, 0, 0)   q=-π/2
  //
  //   yaw:  Frame_fixation_articule → joint_remorque_simple
  //     xyz=(0, 0.05714999999950, -0.26352500000200)
  //     rpy=(-π/2, 0, 0)   q = π/2 + φ  ← VARIABLE
  //
  //   roll: joint_remorque_simple → MTT_remorque
  //     xyz=(0, 0, -0.05714999999985)
  //     rpy=(-π/2, 0, -π/2)   q=-π/2

  // T_base_footprint → base_link (fixed joint, z offset only)
  Eigen::Isometry3d T_bf_bl = Eigen::Isometry3d::Identity();
  T_bf_bl.translation() << 0.0, 0.0, -0.1;

  // T_pitch (full joint at rest: q = -π/2)
  const Eigen::Isometry3d T_pitch = urdfJoint(
    {-1.0511878376018575, 0.2125028310306423, 0.3577510511469179},
    {-M_PI_2, 0.0, 0.0},
    -M_PI_2);

  // T_yaw origin (no rotation — Rz(π/2+φ) applied at runtime)
  const Eigen::Isometry3d T_yaw_origin = urdfOrigin(
    {0.0, 0.05714999999950, -0.26352500000200},
    {-M_PI_2, 0.0, 0.0});

  // T_roll (full joint at rest: q = -π/2)
  const Eigen::Isometry3d T_roll = urdfJoint(
    {0.0, 0.0, -0.05714999999985},
    {-M_PI_2, 0.0, -M_PI_2},
    -M_PI_2);

  // Precompute constant prefix and suffix
  A_ = T_bf_bl * T_pitch * T_yaw_origin;  // everything before Rz(π/2+φ)
  B_ = T_roll;                             // everything after  Rz(π/2+φ)

  RCLCPP_INFO(get_logger(),
    "TrailerLocalizerNode: A·Rz(π/2)·B → trailer at φ=0: t=[%.3f, %.3f, %.3f]",
    computeDelta(0.0).translation().x(),
    computeDelta(0.0).translation().y(),
    computeDelta(0.0).translation().z());

  // ── Publishers ──────────────────────────────────────────────────────
  trailer_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
    "trailer/odom", rclcpp::QoS(10));

  trailer_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "trailer/pose_in_map", rclcpp::QoS(10));

  if (broadcast_tf_) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  // ── Subscriptions ───────────────────────────────────────────────────
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "localization/odom",
    rclcpp::QoS(10),
    [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { onTractorOdom(msg); });

  articulation_sub_ = create_subscription<mtt_msgs::msg::MttArticulationState>(
    "/mtt/articulation_state",
    rclcpp::SensorDataQoS(),
    [this](mtt_msgs::msg::MttArticulationState::ConstSharedPtr msg) {
      onArticulationState(msg);
    });

  // ISAM2-optimised φ — preferred source when factor_graph_node is running
  isam2_phi_sub_ = create_subscription<std_msgs::msg::Float64>(
    "localization/articulation_angle",
    rclcpp::QoS(10),
    [this](std_msgs::msg::Float64::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      latest_isam2_phi_ = msg->data;
      latest_isam2_phi_stamp_ = get_clock()->now();
    });

  // ── Timer ───────────────────────────────────────────────────────────
  const auto period = std::chrono::duration<double>(1.0 / publish_rate);
  publish_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    [this]() { publishTrailerPose(); });

  RCLCPP_INFO(get_logger(),
    "TrailerLocalizerNode started (fusion_mode=%s, rate=%.0f Hz, tf_frame=%s)",
    fusion_mode_str.c_str(), publish_rate, trailer_tf_frame_.c_str());
}

// ─────────────────────────────────────────────────────────────────────────────
// Callbacks
// ─────────────────────────────────────────────────────────────────────────────
void TrailerLocalizerNode::onTractorOdom(nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  latest_odom_ = *msg;
}

void TrailerLocalizerNode::onArticulationState(
  mtt_msgs::msg::MttArticulationState::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  latest_articulation_ = *msg;
}

// ─────────────────────────────────────────────────────────────────────────────
// Timer callback — main computation
// ─────────────────────────────────────────────────────────────────────────────
void TrailerLocalizerNode::publishTrailerPose()
{
  // Take a snapshot of the shared state
  nav_msgs::msg::Odometry odom;
  mtt_msgs::msg::MttArticulationState articulation;
  std::optional<double> isam2_phi;
  rclcpp::Time isam2_phi_stamp{0, 0, RCL_ROS_TIME};
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!latest_odom_ || !latest_articulation_) {
      return;  // Not yet received — start silently
    }
    odom = *latest_odom_;
    articulation = *latest_articulation_;
    isam2_phi = latest_isam2_phi_;
    isam2_phi_stamp = latest_isam2_phi_stamp_;
  }

  // Check raw articulation freshness
  const rclcpp::Time now = get_clock()->now();
  const rclcpp::Time art_stamp(articulation.header.stamp);
  if ((now - art_stamp).seconds() > articulation_timeout_) {
    return;  // Stale raw articulation — skip silently
  }

  // ── Select φ ─────────────────────────────────────────────────────────
  // Priority: ISAM2-optimised φ (if fresh) → raw articulation_state
  // The ISAM2 φ is the posterior-optimal estimate fusing encoder + LiDAR.
  double phi;
  double sigma_phi;
  const bool isam2_fresh =
    isam2_phi.has_value() &&
    (now - isam2_phi_stamp).seconds() < articulation_timeout_;

  if (isam2_fresh) {
    phi = *isam2_phi;
    // ISAM2 posterior σ is lower than raw sources — use hardware σ as proxy
    // (the true marginal σ would require extracting cov_phi from factor_graph_node)
    sigma_phi = sigma_phi_hardware_;
  } else {
    phi = selectPhi(articulation);
    sigma_phi = selectSigmaPhi(articulation);
  }
  const double sigma2_phi = sigma_phi * sigma_phi;

  // ── Build T_map_tractor from Odometry ───────────────────────────────
  const auto & p = odom.pose.pose.position;
  const auto & q = odom.pose.pose.orientation;
  Eigen::Isometry3d T_tractor = Eigen::Isometry3d::Identity();
  T_tractor.translation() << p.x, p.y, p.z;
  T_tractor.linear() =
    Eigen::Quaterniond(q.w, q.x, q.y, q.z).toRotationMatrix();

  // ── Compute trailer pose ────────────────────────────────────────────
  const Eigen::Isometry3d delta = computeDelta(phi);
  const Eigen::Isometry3d T_trailer = T_tractor * delta;

  // ── Covariance propagation ──────────────────────────────────────────
  Eigen::Matrix<double, 6, 6> sigma_tractor = covFromRos(odom.pose.covariance);

  // If factor_graph_node publishes zero covariance (no ISAM2 marginals yet),
  // fall back to configurable diagonal defaults so covariance output is useful.
  if (sigma_tractor.isZero(1e-12)) {
    sigma_tractor = Eigen::Matrix<double, 6, 6>::Zero();
    const double sv2 = default_tractor_sigma_xyz_ * default_tractor_sigma_xyz_;
    const double sr2 = default_tractor_sigma_rpy_ * default_tractor_sigma_rpy_;
    sigma_tractor(0, 0) = sv2;
    sigma_tractor(1, 1) = sv2;
    sigma_tractor(2, 2) = sv2;
    sigma_tractor(3, 3) = sr2;
    sigma_tractor(4, 4) = sr2;
    sigma_tractor(5, 5) = sr2;
  }

  const Eigen::Matrix<double, 6, 1> J_phi =
    computeJacobianPhi(T_tractor, phi);
  const Eigen::Matrix<double, 6, 6> sigma_trailer =
    propagateCovariance(sigma_tractor, sigma2_phi, J_phi, delta);

  // ── Build output messages ────────────────────────────────────────────
  const rclcpp::Time stamp(odom.header.stamp);
  const std::string map_frame = odom.header.frame_id;

  // Convert Eigen pose to geometry_msgs
  const Eigen::Quaterniond q_trailer(T_trailer.rotation());

  // nav_msgs/Odometry
  nav_msgs::msg::Odometry odom_out;
  odom_out.header.stamp = stamp;
  odom_out.header.frame_id = map_frame;
  odom_out.child_frame_id = trailer_tf_frame_;
  odom_out.pose.pose.position.x = T_trailer.translation().x();
  odom_out.pose.pose.position.y = T_trailer.translation().y();
  odom_out.pose.pose.position.z = T_trailer.translation().z();
  odom_out.pose.pose.orientation.w = q_trailer.w();
  odom_out.pose.pose.orientation.x = q_trailer.x();
  odom_out.pose.pose.orientation.y = q_trailer.y();
  odom_out.pose.pose.orientation.z = q_trailer.z();
  covToRos(sigma_trailer, odom_out.pose.covariance);
  // twist left at zero (would require dφ/dt which is not computed here)

  trailer_odom_pub_->publish(odom_out);

  // geometry_msgs/PoseWithCovarianceStamped
  geometry_msgs::msg::PoseWithCovarianceStamped pose_out;
  pose_out.header = odom_out.header;
  pose_out.pose = odom_out.pose;
  trailer_pose_pub_->publish(pose_out);

  // TF broadcast
  if (broadcast_tf_ && tf_broadcaster_) {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = map_frame;
    tf_msg.child_frame_id = trailer_tf_frame_;
    tf_msg.transform.translation.x = T_trailer.translation().x();
    tf_msg.transform.translation.y = T_trailer.translation().y();
    tf_msg.transform.translation.z = T_trailer.translation().z();
    tf_msg.transform.rotation.w = q_trailer.w();
    tf_msg.transform.rotation.x = q_trailer.x();
    tf_msg.transform.rotation.y = q_trailer.y();
    tf_msg.transform.rotation.z = q_trailer.z();
    tf_broadcaster_->sendTransform(tf_msg);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Kinematics
// ─────────────────────────────────────────────────────────────────────────────
Eigen::Isometry3d TrailerLocalizerNode::computeDelta(double phi) const
{
  return A_ *
         Eigen::Isometry3d(Eigen::AngleAxisd(M_PI_2 + phi, Eigen::Vector3d::UnitZ())) *
         B_;
}

Eigen::Matrix<double, 6, 1> TrailerLocalizerNode::computeJacobianPhi(
  const Eigen::Isometry3d & T_tractor, double phi) const
{
  // Central differences: J_φ ≈ (log(T_nom⁻¹ · T_plus)) / eps
  // Using right perturbation convention, [position; rotation] layout
  constexpr double kEps = 1e-5;
  const Eigen::Isometry3d T_nom  = T_tractor * computeDelta(phi);
  const Eigen::Isometry3d T_plus = T_tractor * computeDelta(phi + kEps);

  // Right tangent vector: δT = T_nom⁻¹ · T_plus ≈ Exp(ε · J_φ)
  const Eigen::Isometry3d dT = T_nom.inverse() * T_plus;

  Eigen::Matrix<double, 6, 1> J;
  // Position part [0:3]
  J.head<3>() = dT.translation() / kEps;
  // Rotation part [3:6] via angle-axis
  const Eigen::AngleAxisd aa(dT.rotation());
  J.tail<3>() = aa.axis() * aa.angle() / kEps;
  return J;
}

Eigen::Matrix<double, 6, 6> TrailerLocalizerNode::propagateCovariance(
  const Eigen::Matrix<double, 6, 6> & sigma_tractor,
  double sigma2_phi,
  const Eigen::Matrix<double, 6, 1> & J_phi,
  const Eigen::Isometry3d & delta) const
{
  // Right-perturbation convention:
  //   δξ_trailer = Ad(Δ⁻¹) · δξ_tractor    →  J_T = Ad(Δ⁻¹)
  //
  // Σ_trailer = J_T · Σ_tractor · J_Tᵀ  +  J_φ · σ²_φ · J_φᵀ
  const Eigen::Matrix<double, 6, 6> J_T = adjoint6(delta.inverse());
  return J_T * sigma_tractor * J_T.transpose() +
         sigma2_phi * J_phi * J_phi.transpose();
}

// ─────────────────────────────────────────────────────────────────────────────
// Source selection
// ─────────────────────────────────────────────────────────────────────────────
double TrailerLocalizerNode::selectPhi(
  const mtt_msgs::msg::MttArticulationState & state) const
{
  switch (fusion_mode_) {
    case FusionMode::kLidar:
      return state.lidar_detected ? state.lidar_rad : state.effective_rad;

    case FusionMode::kFused: {
      const bool have_hw = state.hardware_fresh;
      const bool have_li = state.lidar_detected;
      if (!have_hw && !have_li) return state.effective_rad;
      if (!have_li) return state.hardware_rad;
      if (!have_hw) return state.lidar_rad;
      // Inverse-variance weighted average
      const double w_hw = 1.0 / (sigma_phi_hardware_ * sigma_phi_hardware_);
      const double w_li = 1.0 / (sigma_phi_lidar_ * sigma_phi_lidar_);
      return (w_hw * state.hardware_rad + w_li * state.lidar_rad) / (w_hw + w_li);
    }

    case FusionMode::kHardware:
    default:
      return state.effective_rad;
  }
}

double TrailerLocalizerNode::selectSigmaPhi(
  const mtt_msgs::msg::MttArticulationState & state) const
{
  switch (fusion_mode_) {
    case FusionMode::kLidar:
      return state.lidar_detected ? sigma_phi_lidar_ : sigma_phi_hardware_;

    case FusionMode::kFused: {
      const bool have_hw = state.hardware_fresh;
      const bool have_li = state.lidar_detected;
      if (!have_hw && !have_li) return sigma_phi_hardware_;
      if (!have_li) return sigma_phi_hardware_;
      if (!have_hw) return sigma_phi_lidar_;
      // Combined sigma for fused estimate (harmonic-mean style)
      const double var_hw = sigma_phi_hardware_ * sigma_phi_hardware_;
      const double var_li = sigma_phi_lidar_ * sigma_phi_lidar_;
      return std::sqrt((var_hw * var_li) / (var_hw + var_li));
    }

    case FusionMode::kHardware:
    default:
      return sigma_phi_hardware_;
  }
}

}  // namespace mtt_loc

RCLCPP_COMPONENTS_REGISTER_NODE(mtt_loc::TrailerLocalizerNode)
