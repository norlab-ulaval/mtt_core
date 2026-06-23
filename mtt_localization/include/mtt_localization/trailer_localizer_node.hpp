#ifndef MTT_LOCALIZATION__TRAILER_LOCALIZER_NODE_HPP_
#define MTT_LOCALIZATION__TRAILER_LOCALIZER_NODE_HPP_

#include <mutex>
#include <optional>
#include <string>

#include <Eigen/Geometry>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "mtt_msgs/msg/mtt_articulation_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace mtt_loc
{

/// Computes and publishes the trailer (MTT_remorque) pose in the map frame.
///
/// Inputs (priority order for φ):
///   localization/odom               → T_map_base_footprint  (nav_msgs/Odometry)
///   localization/articulation_angle → φ optimisé par ISAM2  (Float64, preferred)
///   /mtt/articulation_state         → φ brut + α pitch fallback (MttArticulationState)
///
/// The ISAM2-optimised φ is preferred when fresh (age < articulation_timeout).
///
/// Inputs (priority order for α — pitch):
///   1. articulation_state.pitch_rad  — hardware potentiometer (σ ≈ 0.020 rad)
///   2. /trailer/pitch_used           — LiDAR EMA pitch from trailer_pose_node (σ ≈ 0.060 rad)
///   3. α = 0                         — flat-terrain fallback (σ = sigma_alpha_model)
///
/// Outputs:
///   trailer/odom               → nav_msgs/Odometry  (map frame, with 6×6 covariance)
///   trailer/pose_in_map        → PoseWithCovarianceStamped  (map frame)
///   TF: map → trailer_body     (direct broadcast, avoids conflict with URDF chain)
///
/// Math:
///   T_map_trailer = T_map_base_footprint · Δ(φ, α)
///   Δ(φ, α) = A_prefix · Rz(-π/2+α) · A_suffix · Rz(π/2+φ) · B
///             (A_prefix, A_suffix, B precomputed from URDF constants)
///   Σ_trailer = Ad(Δ⁻¹)·Σ_tractor·Ad(Δ⁻¹)ᵀ + J_φ·σ²_φ·J_φᵀ + J_α·σ²_α·J_αᵀ
class TrailerLocalizerNode final : public rclcpp::Node
{
public:
  explicit TrailerLocalizerNode(const rclcpp::NodeOptions & options);

private:
  // ── Callbacks ──
  void onTractorOdom(nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void onArticulationState(mtt_msgs::msg::MttArticulationState::ConstSharedPtr msg);

  // ── Timer ──
  void publishTrailerPose();

  // ── Kinematics ──
  /// Δ(φ, α) = A_prefix · Rz(-π/2+α) · A_suffix · Rz(π/2+φ) · B
  Eigen::Isometry3d computeDelta(double phi, double alpha = 0.0) const;

  /// J_φ via central differences (6×1, [position; rotation] convention)
  Eigen::Matrix<double, 6, 1> computeJacobianPhi(
    const Eigen::Isometry3d & T_tractor, double phi, double alpha) const;

  /// J_α via central differences (6×1)
  Eigen::Matrix<double, 6, 1> computeJacobianAlpha(
    const Eigen::Isometry3d & T_tractor, double phi, double alpha) const;

  /// Σ_trailer = J_T · Σ_tractor · J_Tᵀ + J_φ · σ²_φ · J_φᵀ + J_α · σ²_α · J_αᵀ
  Eigen::Matrix<double, 6, 6> propagateCovariance(
    const Eigen::Matrix<double, 6, 6> & sigma_tractor,
    double sigma2_phi,
    const Eigen::Matrix<double, 6, 1> & J_phi,
    double sigma2_alpha,
    const Eigen::Matrix<double, 6, 1> & J_alpha,
    const Eigen::Isometry3d & delta) const;

  // ── Source selection ──
  enum class FusionMode { kHardware, kLidar, kFused };

  double selectPhi(const mtt_msgs::msg::MttArticulationState & state) const;
  double selectSigmaPhi(const mtt_msgs::msg::MttArticulationState & state) const;

  // ── Precomputed kinematic constants ──
  Eigen::Isometry3d A_prefix_;  ///< T_bf_bl · T_pitch_origin  (before pitch rotation)
  Eigen::Isometry3d A_suffix_;  ///< T_yaw_origin              (between pitch and yaw)
  Eigen::Isometry3d B_;         ///< T_roll                    (after yaw rotation)

  // ── Parameters ──
  FusionMode fusion_mode_{FusionMode::kHardware};
  double sigma_phi_hardware_{0.008};          ///< rad — yaw encoder σ
  double sigma_phi_lidar_{0.035};             ///< rad — yaw LiDAR KF σ
  double sigma_alpha_hardware_{0.020};        ///< rad — pitch potentiometer σ
  double sigma_alpha_lidar_{0.060};           ///< rad — pitch LiDAR (trailer_pose_node EMA) σ
  double sigma_alpha_model_{0.035};           ///< rad — pitch model/stale σ (fallback)
  double default_tractor_sigma_xyz_{0.05};    ///< m, fallback when input cov is zero
  double default_tractor_sigma_rpy_{0.01};    ///< rad, fallback when input cov is zero
  double articulation_timeout_{0.5};          ///< s
  std::string trailer_tf_frame_{"trailer_body"};
  bool broadcast_tf_{true};

  // ── Shared state (mutex-protected) ──
  mutable std::mutex state_mutex_;
  std::optional<nav_msgs::msg::Odometry> latest_odom_;
  std::optional<mtt_msgs::msg::MttArticulationState> latest_articulation_;
  // ISAM2-optimised φ (preferred over raw articulation_state when fresh)
  std::optional<double> latest_isam2_phi_;
  rclcpp::Time latest_isam2_phi_stamp_{0, 0, RCL_ROS_TIME};
  // LiDAR pitch from trailer_pose_node (fallback when hardware potentiometer is stale)
  std::optional<double> latest_lidar_pitch_;
  rclcpp::Time latest_lidar_pitch_stamp_{0, 0, RCL_ROS_TIME};

  // ── ROS interfaces ──
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<mtt_msgs::msg::MttArticulationState>::SharedPtr articulation_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr isam2_phi_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr lidar_pitch_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr trailer_odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr trailer_pose_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace mtt_loc

#endif  // MTT_LOCALIZATION__TRAILER_LOCALIZER_NODE_HPP_
