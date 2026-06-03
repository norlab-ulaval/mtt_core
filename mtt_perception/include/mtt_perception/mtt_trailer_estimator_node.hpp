// mtt_trailer_estimator_node.hpp — MTT Trailer Pose Estimator V1.0
//
// Unified 10-DOF EKF estimator for the MTT-154 articulated trailer.
// Replaces trailer_pose_node (V4.0) and subsumes trailer_localizer_node.
// trailer_detector_node (V1.5) is kept unchanged and feeds /trailer/articulation_angle
// as the primary kinematic prior source.
//
// State vector (map frame):
//   x = [x, y, z, roll, pitch, yaw, vx, vy, vz, yaw_rate]'   (10-DOF)
//
// Measurement sources (in priority / frequency order):
//   1. Kinematic pseudo-measurement (100 Hz): URDF chain from tractor odom + phi
//   2. RS-Airy LiDAR ICP (10 Hz): constrained to [x, y, yaw], covariance from Hessian
//   3. Hesai LiDAR ICP (10 Hz): independent supplementary measurement
//
// Thread model:
//   - Sensor callbacks push into lock-free SPSC ring buffers (one per source).
//   - filter_thread_ drains all queues in timestamp order every 10 ms.
//   - 50 Hz publish_timer_ reads filter state under shared_mutex_.
//
// References:
//   [TR] Thrun, Burgard, Fox — "Probabilistic Robotics" (2005)
//   [MV] Hartley, Zisserman — "Multiple View Geometry in Computer Vision" (2nd ed.)
//   [EI] Eigen3 documentation — eigen.tuxfamily.org

#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <optional>
#include <shared_mutex>
#include <string>
#include <thread>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/float64.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace mtt_perception
{

// ─────────────────────────────────────────────────────────────────────────────
// SpscRingBuffer — single-producer / single-consumer lock-free queue.
//
// N must be a power of 2 so index masking replaces modulo (branch-free).
// head_ is written by the producer (sensor callback).
// tail_ is written by the consumer (filter thread).
// Cache-line padding (alignas(64)) prevents false sharing on x86-64.
// ─────────────────────────────────────────────────────────────────────────────
template<typename T, std::size_t N>
class SpscRingBuffer
{
  static_assert((N & (N - 1)) == 0, "SpscRingBuffer: N must be a power of 2");
  static_assert(N >= 4, "SpscRingBuffer: N must be at least 4");

  std::array<T, N> buf_{};

  // alignas(64): keep head_ and tail_ on separate cache lines to avoid
  // false sharing between producer and consumer threads (hardware prefetch).
  alignas(64) std::atomic<std::size_t> head_{0};  // producer writes
  alignas(64) std::atomic<std::size_t> tail_{0};  // consumer writes

public:
  // Returns false if the buffer is full (oldest entry is dropped — caller decides).
  [[nodiscard]] bool push(const T & item) noexcept
  {
    const std::size_t h = head_.load(std::memory_order_relaxed);
    const std::size_t next_h = (h + 1u) & (N - 1u);
    if (next_h == tail_.load(std::memory_order_acquire)) {
      return false;  // buffer full — caller should log and skip
    }
    buf_[h] = item;
    head_.store(next_h, std::memory_order_release);
    return true;
  }

  // Returns false if the buffer is empty.
  [[nodiscard]] bool pop(T & item) noexcept
  {
    const std::size_t t = tail_.load(std::memory_order_relaxed);
    if (t == head_.load(std::memory_order_acquire)) {
      return false;  // buffer empty
    }
    item = buf_[t];
    tail_.store((t + 1u) & (N - 1u), std::memory_order_release);
    return true;
  }

  [[nodiscard]] bool empty() const noexcept
  {
    return tail_.load(std::memory_order_acquire) == head_.load(std::memory_order_acquire);
  }
};

// ─────────────────────────────────────────────────────────────────────────────
// Eigen type aliases — explicit throughout, no auto where type is non-obvious.
// ─────────────────────────────────────────────────────────────────────────────
using State10d  = Eigen::Matrix<double, 10, 1>;   // EKF state vector
using Cov10d    = Eigen::Matrix<double, 10, 10>;  // EKF covariance matrix
using Mat10d    = Eigen::Matrix<double, 10, 10>;  // generic 10×10
using H4x10     = Eigen::Matrix<double,  4, 10>;  // kinematic measurement matrix
using H3x10     = Eigen::Matrix<double,  3, 10>;  // LiDAR ICP measurement matrix
using Cov4d     = Eigen::Matrix<double,  4,  4>;  // 4-DOF measurement covariance
using Cov3d     = Eigen::Matrix<double,  3,  3>;  // 3-DOF measurement covariance
using Cov6d     = Eigen::Matrix<double,  6,  6>;  // 6-DOF pose covariance
using Vec4d     = Eigen::Matrix<double,  4,  1>;
using Vec3d     = Eigen::Matrix<double,  3,  1>;

// ─────────────────────────────────────────────────────────────────────────────
// State index constants — avoids magic numbers throughout the implementation.
// ─────────────────────────────────────────────────────────────────────────────
namespace idx
{
constexpr int kX       = 0;
constexpr int kY       = 1;
constexpr int kZ       = 2;
constexpr int kRoll    = 3;
constexpr int kPitch   = 4;
constexpr int kYaw     = 5;
constexpr int kVx      = 6;
constexpr int kVy      = 7;
constexpr int kVz      = 8;
constexpr int kYawRate = 9;
constexpr int kNDof    = 10;
}  // namespace idx

// ─────────────────────────────────────────────────────────────────────────────
// Measurement structs — each pushed into its own SpscRingBuffer.
// All positions in map frame, angles in radians, times as rclcpp::Time.
// ─────────────────────────────────────────────────────────────────────────────

// Kinematic pseudo-measurement from URDF chain: [x, y, z, yaw] in map frame.
struct KinematicMeas
{
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  Vec4d        z{Vec4d::Zero()};   // [x, y, z, yaw]
  Cov4d        R{Cov4d::Identity() * 0.01};  // measurement noise (propagated from odom + phi)
  bool         valid{false};
};

// LiDAR ICP measurement: [x, y, yaw] in map frame.
// Covariance R = (J'WJ)^{-1} from ICP Hessian — [TR] eq. 6.9 analog.
struct LidarMeas
{
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  Vec3d        z{Vec3d::Zero()};     // [x, y, yaw]
  Cov3d        R{Cov3d::Identity() * 0.01};
  double       alignment_error{0.0}; // mean point-to-model distance after ICP (diagnostic)
  double       inlier_ratio{0.0};    // fraction of model points with valid correspondences
  bool         valid{false};
};

// ─────────────────────────────────────────────────────────────────────────────
// TrailerModel — 3D wireframe used for ICP matching.
//
// All coordinates in the trailer LOCAL frame:
//   +s: trailer longitudinal (from hitch toward rear)
//   +l: trailer lateral (left)
//   +h: vertical (up)
//
// The model is sampled into discrete points at `point_spacing` intervals.
// ─────────────────────────────────────────────────────────────────────────────
struct TrailerModel
{
  double s_front{-0.05};    // longitudinal position of front rail end  (m, relative to body center)
  double s_rear{1.90};      // longitudinal position of rear rail end   (m, relative to body center)
  double l_half{0.40};      // half-width (rail lateral offset)         (m)
  double h_rail{0.30};      // nominal height of rails above body origin (m)
  double point_spacing{0.10}; // model sampling density                 (m)

  // Sampled model points in local (s, l, h) frame.
  // Pre-allocated at construction — zero-allocation hot path.
  std::vector<Eigen::Vector3d> pts_local{};  // built once in initTrailerModel()
};

// ─────────────────────────────────────────────────────────────────────────────
// FilterState — EKF state + covariance + metadata, protected by shared_mutex_.
// ─────────────────────────────────────────────────────────────────────────────
struct FilterState
{
  State10d x{State10d::Zero()};   // [x,y,z,roll,pitch,yaw, vx,vy,vz,yaw_rate] map frame
  Cov10d   P{Cov10d::Identity() * 1.0};  // state covariance
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  bool initialized{false};
};

// ─────────────────────────────────────────────────────────────────────────────
// LocalPoint — a cloud point expressed in the trailer-prior local frame.
// p_map holds the point in whatever frame the pipeline operates in (map for
// ICP path, sensor frame for PCA path). frame_id is stored externally.
// ─────────────────────────────────────────────────────────────────────────────
struct LocalPoint
{
  Eigen::Vector3d p_map{Eigen::Vector3d::Zero()};  // point in pipeline frame
  double s{0.0};  // longitudinal (along prior trailer heading)
  double l{0.0};  // lateral
  double h{0.0};  // vertical
};

// ─────────────────────────────────────────────────────────────────────────────
// KinematicPrior — articulation-angle-derived trailer location in base_link
// and in the sensor frame. Same formula as trailer_pose_node (V4.0).
//   yaw_prior = π - theta   (theta = articulation angle from detector)
//   p0_base   = hitch + body_offset * u_base
//   p0_cloud  = R_sensor_from_bl * p0_base + t_sensor_from_bl
// ─────────────────────────────────────────────────────────────────────────────
struct KinematicPrior
{
  Eigen::Vector3d hitch_base{Eigen::Vector3d::Zero()};
  Eigen::Vector3d p0_base{Eigen::Vector3d::Zero()};   // trailer body center in base_link
  Eigen::Vector3d u_base{Eigen::Vector3d::UnitX()};   // trailer rear direction in base_link
  Eigen::Vector3d n_base{Eigen::Vector3d::UnitY()};   // trailer lateral direction in base_link
  Eigen::Vector3d k_base{Eigen::Vector3d::UnitZ()};
  Eigen::Vector3d p0_cloud{Eigen::Vector3d::Zero()};  // trailer body center in sensor frame
  Eigen::Vector3d u_cloud{Eigen::Vector3d::UnitX()};
  Eigen::Vector3d n_cloud{Eigen::Vector3d::UnitY()};
  Eigen::Vector3d k_cloud{Eigen::Vector3d::UnitZ()};
  double yaw_prior{0.0};  // trailer yaw in base_link (rad)
};

// ─────────────────────────────────────────────────────────────────────────────
// TrailerEstimatorNode — unified trailer pose estimator.
// ─────────────────────────────────────────────────────────────────────────────
class TrailerEstimatorNode final : public rclcpp::Node
{
public:
  explicit TrailerEstimatorNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~TrailerEstimatorNode() override;

private:
  // ── Sensor callbacks (push to SPSC queues, return fast) ─────────────────

  /// /trailer/articulation_angle — from trailer_detector_node KF (preferred phi source).
  void onArticulationAngle(std_msgs::msg::Float64::ConstSharedPtr msg);

  /// /hardware/articulation_angle — raw hardware ADC (cross-validation / fallback).
  void onHardwareArticulationAngle(std_msgs::msg::Float64::ConstSharedPtr msg);

  /// /mtt_odometry — wheel+kinematic tractor odometry (50 Hz).
  void onTractorOdom(nav_msgs::msg::Odometry::ConstSharedPtr msg);

  /// /mapping/icp_odom — ICP-corrected tractor pose in map (10 Hz).
  void onIcpOdom(nav_msgs::msg::Odometry::ConstSharedPtr msg);

  /// /rsairy_ns/rslidar_points — rear-facing LiDAR (primary trailer view, 10 Hz).
  void onRsairyCloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  /// /hesai_lidar/points — 360° LiDAR (supplementary, 10 Hz).
  void onHesaiCloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  /// /mti100/data — IMU for pitch/roll caching (400 Hz, callbacks are cheap).
  void onImu(sensor_msgs::msg::Imu::ConstSharedPtr msg);

  // ── Kinematic prior ──────────────────────────────────────────────────────

  /// Compute the URDF kinematic measurement and push into kinematic_queue_.
  /// Called from onArticulationAngle when tractor pose is available.
  void computeAndQueueKinematicMeas(
    double phi,
    const rclcpp::Time & stamp);

  /// Δ(φ, α) = A_prefix_ · Rz(-π/2+α) · A_suffix_ · Rz(π/2+φ) · B_
  /// Reproduces trailer_localizer_node::computeDelta() exactly.
  [[nodiscard]] Eigen::Isometry3d computeDelta(double phi, double alpha) const noexcept;

  /// Numerical Jacobian ∂T_trailer/∂φ via central differences.
  [[nodiscard]] Eigen::Matrix<double, 6, 1> jacobianPhi(
    const Eigen::Isometry3d & T_tractor, double phi, double alpha) const;

  /// Propagate covariance from tractor odom to trailer measurement.
  [[nodiscard]] Cov4d propagateKinematicCov(
    const Cov6d & sigma_tractor,
    double sigma2_phi,
    double sigma2_alpha,
    const Eigen::Matrix<double, 6, 1> & J_phi) const;

  // ── trailer_pose_node V4.0 detection helpers ─────────────────────────────

  /// Cache the static TF: sensor_frame ← base_link (called once on first cloud).
  bool cacheStaticTf(const std::string & sensor_frame);

  /// Build kinematic prior — identical formula to trailer_pose_node:
  ///   yaw_prior = π − theta,  p0 = hitch + body_offset × u
  [[nodiscard]] KinematicPrior buildKinematicPrior(double theta) const;

  /// Update EMA filter (same logic as trailer_pose_node::updateFilteredCorrection).
  struct PcaMeasurement {
    double yaw_corr_raw{0.0};   double yaw_corr_used{0.0};   bool yaw_valid{false};
    double pitch_raw{0.0};      double pitch_used{0.0};       bool pitch_valid{false};
    double roll_raw{0.0};       double roll_used{0.0};        bool roll_valid{false};
    double corr_s{0.0};         double corr_l{0.0};           double corr_h{0.0};
    double pca_ratio{0.0};      double span_s{0.0};           double confidence{0.0};
    bool measurement_valid{false};
    std::size_t n_pts{0};
  };
  void updatePcaFilter(const PcaMeasurement & m);

  /// Compute filtered trailer body centre in base_link.
  [[nodiscard]] Eigen::Vector3d filteredPositionBase(const KinematicPrior & prior) const;

  /// Build rotation matrix from yaw + pitch + roll (same as trailer_pose_node).
  [[nodiscard]] static Eigen::Matrix3d makeTrailerRotation(
    double yaw, double pitch, double roll) noexcept;

  // ── LiDAR processing pipeline (shared for RS-Airy and Hesai) ────────────

  /// Full pipeline: crop ROI in sensor frame using kinematic prior → ground removal →
  ///  voxel downsample → 2D PCA → push LidarMeas in map frame.
  void processLidarCloud(
    const sensor_msgs::msg::PointCloud2 & cloud,
    SpscRingBuffer<LidarMeas, 32> & queue,
    const std::string & sensor_name);

  /// Look up TF from `sensor_frame` to `map` at `stamp`.
  /// Returns std::nullopt on failure (TF not yet available).
  [[nodiscard]] std::optional<Eigen::Isometry3d> lookupTfToMap(
    const std::string & sensor_frame,
    const rclcpp::Time & stamp) const;

  /// Project all cloud points to map frame using a pre-looked-up transform.
  /// Uses batched Eigen multiplication — no tf2::doTransform per-point loop.
  [[nodiscard]] std::vector<LocalPoint> transformAndCropRoi(
    const sensor_msgs::msg::PointCloud2 & cloud,
    const Eigen::Isometry3d & T_sensor_map,
    const State10d & x_prior) const;

  /// RANSAC 3D plane fit for ground removal.
  /// Returns inlier mask; plane normal constrained within kMaxGroundTilt of [0,0,1].
  [[nodiscard]] std::vector<bool> ransacGroundPlane(
    const std::vector<LocalPoint> & pts,
    double inlier_thresh_m) const;

  /// RANSAC 3D line fit on a cluster of points.
  /// Returns (direction, point_on_line, inlier_indices) or empty on failure.
  struct RansacLine3d
  {
    Eigen::Vector3d dir{Eigen::Vector3d::UnitX()};  // unit direction
    Eigen::Vector3d p0{Eigen::Vector3d::Zero()};    // point on line
    std::vector<std::size_t> inliers{};             // indices into pts
    double residual_variance{1.0};                  // inlier residual variance (m²)
    bool valid{false};
  };
  [[nodiscard]] RansacLine3d ransacLine3d(
    const std::vector<Eigen::Vector3d> & pts,
    double inlier_thresh_m,
    int max_iters) const;

  /// Constrained ICP: optimise [x, y, yaw] in map frame.
  /// pitch and roll are fixed (from IMU / kinematic prior).
  /// Returns (refined_pose, covariance_3x3) where pose = [x, y, yaw].
  struct IcpResult
  {
    Vec3d    pose{Vec3d::Zero()};  // [x, y, yaw]
    Cov3d    cov{Cov3d::Identity() * 0.01};
    double   alignment_error{1e9};
    double   inlier_ratio{0.0};
    bool     valid{false};
  };
  [[nodiscard]] IcpResult runIcp(
    const std::vector<LocalPoint> & pts_map,
    const State10d & x_init) const;

  // ── EKF core ─────────────────────────────────────────────────────────────

  /// Constant-velocity prediction step with velocity decay.
  /// F·x, P = F·P·F' + Q(dt).  [TR] Algorithm 3.1 (prediction step).
  void ekfPredict(FilterState & state, double dt) const;

  /// Joseph-form measurement update with Mahalanobis gating.
  /// Template parameter M = measurement DOF (3 or 4).
  /// [TR] Algorithm 3.1 (update step); Joseph form: [TR] eq. 3.14.
  template<int M>
  [[nodiscard]] bool ekfUpdate(
    FilterState & state,
    const Eigen::Matrix<double, M, 1> & z,
    const Eigen::Matrix<double, M, 10> & H,
    const Eigen::Matrix<double, M, M> & R,
    double chi2_threshold) const;

  /// Check if the filter has diverged (trace or state jump).
  /// If so, re-initialise from the last valid kinematic measurement.
  void checkAndHandleDivergence(FilterState & state);

  // ── Filter thread ─────────────────────────────────────────────────────────

  /// Entry point for filter_thread_. Runs until stop_flag_ is set.
  void filterThreadFunc();

  /// Process one entry from the kinematic queue inside the filter thread.
  void applyKinematicMeas(FilterState & state, const KinematicMeas & meas);

  /// Process one entry from a LiDAR queue inside the filter thread.
  void applyLidarMeas(FilterState & state, const LidarMeas & meas);

  // ── Output / publishing ──────────────────────────────────────────────────

  /// 50 Hz timer callback — publishes all outputs from the last filter state.
  void publishTimerCallback();

  /// Build and publish /trailer/pose (PoseWithCovarianceStamped, map frame).
  void publishPose(const FilterState & state) const;

  /// Build and publish /trailer/odom (nav_msgs/Odometry, map frame).
  void publishOdom(const FilterState & state) const;

  /// Build and publish /trailer/body_markers (wireframe at estimated pose).
  void publishMarkers(const FilterState & state, const rclcpp::Time & now) const;

  /// Build and publish /trailer/trailer_roi_cloud from last processed cloud.
  void publishRoiCloud(
    const std::vector<LocalPoint> & pts,
    const std_msgs::msg::Header & header) const;

  /// Back-compute hitch angle from filter state and tractor pose.
  /// Publish /trailer/articulation_angle and run cross-validation.
  void publishHitchAngle(const FilterState & state, const rclcpp::Time & now);

  /// Publish Float64 diagnostic KPIs.
  void publishDiagnostics(
    double latency_ms,
    double lidar_error_m,
    double inlier_ratio) const;

  // ── Helpers ───────────────────────────────────────────────────────────────

  /// Wrap angle to [-π, π].
  [[nodiscard]] static double normalizeAngle(double a) noexcept;

  /// Clamp x to [lo, hi].
  [[nodiscard]] static double clamp(double x, double lo, double hi) noexcept;

  /// Extract 6×6 pose covariance from nav_msgs/Odometry (row-major ROS layout).
  [[nodiscard]] static Cov6d covFromOdom(
    const nav_msgs::msg::Odometry & odom) noexcept;

  // ── Initialisation helpers ─────────────────────────────────────────────────

  /// Pre-compute URDF kinematic constants A_prefix_, A_suffix_, B_.
  /// Values copied verbatim from trailer_localizer_node — do not edit independently.
  void initUrdfChain();

  /// Sample the trailer wireframe model into TrailerModel::pts_local.
  void initTrailerModel();

  // ── URDF kinematic chain constants (precomputed, read-only after init) ────
  //
  // Δ(φ,α) = A_prefix_ · Rz(-π/2+α) · A_suffix_ · Rz(π/2+φ) · B_
  // Source: mtt_description/urdf/robot.urdf.xacro, exactly as in TrailerLocalizerNode.
  Eigen::Isometry3d A_prefix_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d A_suffix_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d B_{Eigen::Isometry3d::Identity()};

  // ── Trailer geometry model ────────────────────────────────────────────────
  TrailerModel model_{};

  // ── EKF tuning (loaded from params) ──────────────────────────────────────
  Cov10d Q_base_{Cov10d::Zero()};  // process noise base matrix (scaled by dt in predict)
  double velocity_decay_{0.95};    // velocity damping factor per predict step
  double chi2_kinematic_{13.28};   // χ²(4 DOF, p=0.99)
  double chi2_lidar_{11.34};       // χ²(3 DOF, p=0.99)
  double divergence_trace_max_{50.0};  // re-init threshold on P.trace()

  // ── SPSC measurement queues (one per sensor source) ───────────────────────
  // Sized for ~2 s at the sensor's maximum rate.
  SpscRingBuffer<KinematicMeas, 256> kinematic_queue_{};
  SpscRingBuffer<LidarMeas,      32> rsairy_queue_{};
  SpscRingBuffer<LidarMeas,      32> hesai_queue_{};

  // ── Filter state (shared_mutex_: many readers, one writer) ───────────────
  mutable std::shared_mutex state_mutex_{};
  FilterState filter_state_{};

  // ── Filter thread ─────────────────────────────────────────────────────────
  std::thread        filter_thread_{};
  std::atomic<bool>  stop_flag_{false};
  int                filter_period_ms_{10};  // drain queues every N ms

  // ── Cached tractor state (mutex_tractor_ protects) ────────────────────────
  mutable std::mutex mutex_tractor_{};
  std::optional<nav_msgs::msg::Odometry> latest_odom_{};       // /mtt_odometry
  std::optional<nav_msgs::msg::Odometry> latest_icp_odom_{};   // /mapping/icp_odom
  rclcpp::Time icp_odom_stamp_{0, 0, RCL_ROS_TIME};
  double icp_odom_max_age_s_{0.25};  // use ICP odom only when fresher than this

  // ── Cached articulation angles ────────────────────────────────────────────
  mutable std::mutex mutex_phi_{};
  double phi_lidar_{0.0};            // from trailer_detector_node KF
  rclcpp::Time phi_lidar_stamp_{0, 0, RCL_ROS_TIME};
  bool phi_lidar_valid_{false};
  double phi_hardware_{0.0};         // raw hardware ADC
  rclcpp::Time phi_hardware_stamp_{0, 0, RCL_ROS_TIME};
  bool phi_hardware_valid_{false};
  double phi_max_age_s_{0.5};        // staleness threshold

  // ── Cached IMU pitch/roll ─────────────────────────────────────────────────
  std::atomic<double> imu_pitch_{0.0};  // rad, updated at 400 Hz
  std::atomic<double> imu_roll_{0.0};   // rad

  // ── Cross-validation state ────────────────────────────────────────────────
  rclcpp::Time hitch_disagreement_start_{0, 0, RCL_ROS_TIME};
  bool hitch_disagreement_active_{false};
  double hitch_validation_sigma_{0.025};    // 2-sigma threshold (rad)
  double hitch_disagreement_timeout_{0.5};  // seconds before warning
  double yaw_delta_ref_{0.0};  // trailer yaw in base_footprint at φ=0,α=0 (precomputed)

  // ── Last processed ROI cloud (for /trailer/trailer_roi_cloud) ─────────────
  mutable std::mutex mutex_roi_cloud_{};
  std::vector<LocalPoint> last_roi_pts_{};
  std_msgs::msg::Header last_roi_header_{};

  // ── Diagnostic accumulators ────────────────────────────────────────────────
  std::atomic<double> last_lidar_error_{0.0};   // m
  std::atomic<double> last_inlier_ratio_{0.0};  // [0,1]

  // ── TF ────────────────────────────────────────────────────────────────────
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::string map_frame_{"map"};
  std::string trailer_tf_frame_{"trailer_body"};

  // Cached static TF: sensor_frame ← base_link (precomputed once at first cloud arrival).
  // Used to transform the kinematic prior into the sensor frame for ROI cropping.
  Eigen::Matrix3d R_sensor_from_bl_{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d t_sensor_from_bl_{Eigen::Vector3d::Zero()};
  bool tf_sensor_cached_{false};
  std::string cached_sensor_frame_{};

  // ── Publishers ─────────────────────────────────────────────────────────────
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_in_map_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr articulation_angle_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr roi_cloud_pub_;
  // Diagnostics
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr diag_latency_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr diag_lidar_error_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr diag_inlier_ratio_pub_;
  // Debug scalars (compatible with existing Foxglove panels)
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_prior_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pitch_used_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr roll_used_pub_;

  // ── Subscribers ────────────────────────────────────────────────────────────
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr articulation_angle_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr hardware_angle_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr tractor_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr icp_odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr rsairy_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr hesai_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  // ── Timer ──────────────────────────────────────────────────────────────────
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // ── Hitch + body geometry in base_link (from trailer_pose_node, URDF-verified) ──
  double hitch_base_x_{-1.45};        // hitch pivot in base_link (m)
  double hitch_base_y_{-0.085};
  double hitch_base_z_{ 0.35};
  double trailer_body_offset_{0.90};  // body center from hitch along u_rear (m)
  double trailer_front_offset_{0.05}; // front rail from hitch along u_rear (m)
  double trailer_rear_offset_{1.90};  // rear rail from hitch along u_rear (m)

  // ── PCA detection parameters (trailer_pose_node V4.0 algorithm) ─────────
  // Detection gates
  double pca_min_points_{30};              // minimum cloud points for PCA
  double pca_min_points_after_ground_{20}; // minimum after ground removal
  double pca_min_ratio_{2.2};             // minimum elongation ratio
  double pca_max_yaw_correction_{0.20};   // rad — reject corrections > ~11°
  double pca_min_span_s_{0.45};           // m — minimum longitudinal extent
  double pca_max_yaw_jump_{0.15};         // rad — inter-frame jump rejection
  double pca_max_pos_s_{0.05};            // m — longitudinal position gate
  double pca_max_pos_l_{0.08};            // m — lateral position gate
  double pca_max_pos_h_{0.06};            // m — vertical position gate
  // Ground filter
  double pca_ground_quantile_{0.10};
  double pca_ground_margin_{0.06};
  // EKF measurement noise
  double pca_r_base_{0.005};              // base yaw measurement variance (rad²)
  double pca_r_scale_{0.05};             // extra = r_scale / pca_ratio

  // ── EMA filter state (trailer_pose_node V4.0, protects LiDAR callback) ──
  struct TrailerPcaState {
    double yaw_corr{0.0};   // filtered yaw correction (rad)
    double pitch{0.0};      // filtered pitch (rad)
    double roll{0.0};       // filtered roll (rad)
    double ds{0.0};         // longitudinal correction (m)
    double dl{0.0};         // lateral correction (m)
    double dh{0.0};         // vertical correction (m)
    bool   ok{false};       // true after first valid initialisation
  };
  mutable std::mutex pca_mutex_{};
  TrailerPcaState    pca_state_{};

  // EMA rates and decay-to-prior params
  double alpha_yaw_{0.25};
  double alpha_pitch_{0.02};
  double alpha_position_{0.20};
  double yaw_decay_{0.85};
  double pitch_decay_{0.80};
  double position_decay_{0.70};

  // ── ICP parameters (kept for future A/B comparison) ──────────────────────
  int    icp_max_iters_{20};
  double icp_max_corr_dist_{0.7};    // max correspondence distance (m)
  double icp_convergence_tol_{1e-4}; // convergence threshold (m)
  int    icp_min_inliers_{8};        // minimum correspondences for valid ICP

  // ── RANSAC ground plane parameters ────────────────────────────────────────
  int    ground_ransac_iters_{50};
  double ground_inlier_thresh_{0.05};  // m
  double ground_max_tilt_rad_{0.52};   // max 30° tilt from vertical

  // ── RANSAC line fit parameters ────────────────────────────────────────────
  int    line_ransac_iters_{100};
  double line_inlier_thresh_{0.05};    // m
  double line_min_inlier_frac_{0.6};   // minimum inlier fraction

  // ── ROI parameters (PCA path uses asymmetric s via front/rear offsets above) ──
  double roi_half_s_{1.05};   // m, legacy (ICP path)
  double roi_half_l_{0.90};   // m, lateral (also used for PCA path)
  double roi_half_h_{1.00};   // m, vertical (also used for PCA path)

  // ── Process noise tuning ───────────────────────────────────────────────────
  double q_xy_{0.01};       // position process noise (m²/s)
  double q_z_{0.001};       // vertical position noise (m²/s)
  double q_rp_{0.001};      // roll/pitch noise (rad²/s)
  double q_yaw_{0.005};     // yaw noise (rad²/s)
  double q_vxy_{0.1};       // velocity process noise (m²/s³)
  double q_vz_{0.01};
  double q_vyaw_{0.05};     // yaw rate process noise (rad²/s³)

  // ── Publish rate ───────────────────────────────────────────────────────────
  double publish_rate_{50.0};  // Hz

  // ── Miscellaneous parameters ───────────────────────────────────────────────
  bool   enable_hesai_{true};
  bool   enable_icp_{true};
  bool   broadcast_tf_{true};
  double voxel_size_{0.04};   // m, for LiDAR cloud downsampling
  int    min_pts_after_ground_{20};
};

}  // namespace mtt_perception
