// Trailer pose estimator — V4.0
//
// Prior-locked constrained SE(3) estimator for the MTT trailer.
//
// Design rules:
//   - /trailer/articulation_angle is the validated mechanical prior.
//   - /mtt_articulation_angle is a command/debug/fallback only.
//   - /trailer/angle is a direct alias of /trailer/articulation_angle.
//   - The pose is never estimated by a free 3-D PCA. The LiDAR cloud only
//     provides small, gated corrections around the articulation-based prior.
//   - /trailer/pose and /trailer/body_markers are always published once TF is
//     available. Bad measurements decay back to the prior instead of freezing.
//
// Geometry:
//   theta = articulation angle
//   yaw_prior = pi - theta + yaw_offset
//   T_prior = hitch + trailer_body_offset * u(yaw_prior)
//   ROI is oriented in the prior frame. Points are projected into (s,l,h):
//     s = longitudinal, l = lateral, h = vertical.
//   After optional ground removal, PCA 2-D on (s,l) provides only a bounded
//   yaw correction. Pitch and roll are optional and strongly gated.

#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/float64.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace mtt_perception
{

class TrailerPoseNode final : public rclcpp::Node
{
public:
  explicit TrailerPoseNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  struct LocalPoint
  {
    Eigen::Vector3d p_cloud{0.0, 0.0, 0.0};
    double s{0.0};
    double l{0.0};
    double h{0.0};
  };

  struct PriorFrame
  {
    double theta{0.0};
    double yaw_prior{0.0};

    Eigen::Vector3d hitch_base{0.0, 0.0, 0.0};
    Eigen::Vector3d p0_base{0.0, 0.0, 0.0};
    Eigen::Vector3d u_base{1.0, 0.0, 0.0};
    Eigen::Vector3d n_base{0.0, 1.0, 0.0};
    Eigen::Vector3d k_base{0.0, 0.0, 1.0};

    Eigen::Vector3d p0_cloud{0.0, 0.0, 0.0};
    Eigen::Vector3d u_cloud{1.0, 0.0, 0.0};
    Eigen::Vector3d n_cloud{0.0, 1.0, 0.0};
    Eigen::Vector3d k_cloud{0.0, 0.0, 1.0};
  };

  struct LineFit
  {
    bool valid{false};
    double a{0.0};
    double b{0.0};
    double rms{0.0};
    std::size_t n{0};
  };

  struct Measurement
  {
    bool yaw_valid{false};
    bool pitch_valid{false};
    bool roll_valid{false};
    bool measurement_valid{false};

    std::size_t n_roi_raw{0};
    std::size_t n_after_ground{0};
    std::size_t n_after_voxel{0};
    std::size_t ground_removed{0};

    double ground_threshold{0.0};
    double span_s{0.0};
    double pca_ratio{0.0};
    double confidence{0.0};

    double yaw_corr_raw{0.0};
    double yaw_corr_used{0.0};
    // True when the two-sided line fit (left/right edges) succeeded and was
    // used for yaw_corr_raw instead of the whole-cloud PCA fallback — the
    // line-fit path has no sign ambiguity, unlike raw 2D PCA.
    bool side_lines_valid{false};

    double pitch_raw{0.0};
    double pitch_used{0.0};
    double pitch_rms{0.0};

    double roll_raw{0.0};
    double roll_used{0.0};
    double width_obs{0.0};

    double corr_s{0.0};
    double corr_l{0.0};
    double corr_h{0.0};

    double s_min{-0.5};
    double s_max{0.5};
  };

  // ROS callbacks
  void articulationAngleCallback(std_msgs::msg::Float64::ConstSharedPtr msg);
  void commandAngleCallback(std_msgs::msg::Float64::ConstSharedPtr msg);
  void cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  // TF / prior
  bool updateTfCache(const std::string & cloud_frame);
  double activeTheta() const;
  PriorFrame buildPriorFrame(double theta) const;

  // Cloud processing
  std::vector<LocalPoint> cropToOrientedRoi(
    const sensor_msgs::msg::PointCloud2 & cloud,
    const PriorFrame & prior) const;

  std::vector<LocalPoint> removeGroundByQuantile(
    const std::vector<LocalPoint> & points,
    Measurement & meas) const;

  std::vector<LocalPoint> voxelDownsampleLocal(
    const std::vector<LocalPoint> & points) const;

  Measurement estimateMeasurement(const std::vector<LocalPoint> & points) const;

  LineFit robustLineFit(
    const std::vector<double> & x,
    const std::vector<double> & y) const;

  // State update and publishing
  void updateFilteredCorrection(const Measurement & meas);
  geometry_msgs::msg::PoseStamped makePose(
    const std_msgs::msg::Header & header,
    const Eigen::Vector3d & position_base,
    double yaw,
    double pitch,
    double roll) const;

  Eigen::Matrix3d makeRotation(double yaw, double pitch, double roll) const;

  void publishTrailerRoiCloud(
    const std_msgs::msg::Header & header,
    const std::vector<LocalPoint> & points) const;

  void publishAll(
    const std_msgs::msg::Header & header,
    const PriorFrame & prior,
    const Measurement & meas,
    const std::vector<LocalPoint> & used_points);

  void publishMarkers(
    const std_msgs::msg::Header & header,
    const PriorFrame & prior,
    const Measurement & meas,
    const Eigen::Vector3d & pos_base,
    double yaw,
    double pitch,
    double roll) const;

  void publishDebugScalars(const PriorFrame & prior, const Measurement & meas) const;

  // Helpers
  static double normalizeAngle(double a) noexcept;
  static double clamp(double x, double lo, double hi) noexcept;
  static double safeRatio(double numerator, double denominator, double fallback) noexcept;
  static double quantile(std::vector<double> values, double q);
  static std_msgs::msg::Float64 f64(double value);

  Eigen::Vector3d filteredPositionBase(const PriorFrame & prior) const;
  Eigen::Vector3d directionFromYawPitch(double yaw, double pitch) const;

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  bool tf_cached_{false};
  std::string cached_cloud_frame_;
  Eigen::Matrix3d R_cloud_base_{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d t_cloud_base_{0.0, 0.0, 0.0};

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_prior_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_raw_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr trailer_roi_cloud_pub_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr confidence_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_prior_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_raw_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_used_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_corr_raw_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_corr_used_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pitch_raw_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pitch_used_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr roll_used_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr roi_count_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr roi_count_after_ground_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr ground_threshold_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr ground_removed_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr span_s_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pca_ratio_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr measurement_valid_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pitch_valid_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr roll_valid_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr command_residual_pub_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr articulation_angle_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr command_angle_sub_;

  // Angle state
  bool angle_received_{false};
  double last_articulation_angle_{0.0};
  rclcpp::Time last_articulation_angle_time_;

  std::optional<double> last_command_angle_;
  rclcpp::Time last_command_angle_time_;

  // Filtered correction state. The prior itself follows articulation instantly;
  // only residual corrections are smoothed.
  bool filter_initialized_{false};
  double yaw_corr_f_{0.0};
  double pitch_f_{0.0};
  double roll_f_{0.0};
  double corr_s_f_{0.0};
  double corr_l_f_{0.0};
  double corr_h_f_{0.0};

  // Parameters
  std::string lidar_topic_;
  std::string angle_topic_;
  std::string command_angle_topic_;
  bool use_command_as_primary_{false};
  double command_fallback_timeout_{1.0};

  double hitch_base_x_{-1.45};
  double hitch_base_y_{-0.085};
  double hitch_base_z_{0.35};
  double trailer_body_offset_{0.90};
  double trailer_front_offset_{0.05};
  double trailer_rear_offset_{1.90};
  double trailer_width_{0.80};
  double yaw_offset_{0.0};

  double roi_half_x_{0.85};
  double roi_half_y_{0.70};
  double roi_half_z_{1.10};
  double roi_marker_vertical_scale_factor_{0.60};

  bool publish_trailer_roi_cloud_{true};
  bool enable_voxel_{true};
  double voxel_size_{0.04};

  bool enable_ground_filter_{true};
  double ground_quantile_{0.10};
  double ground_margin_{0.06};

  int min_points_{30};
  int min_points_after_ground_{20};
  double min_span_s_{0.45};
  double min_centerline_pca_ratio_{2.2};
  double max_yaw_correction_{0.20};
  double max_yaw_jump_{0.15};

  double max_position_correction_s_{0.05};
  double max_position_correction_l_{0.08};
  double max_position_correction_h_{0.06};

  bool enable_pitch_estimation_{true};
  double max_pitch_{0.12};
  double max_pitch_rms_{0.08};

  bool enable_roll_estimation_{false};
  double max_roll_{0.08};
  double width_tolerance_{0.35};
  int min_side_points_{20};

  double alpha_yaw_{0.25};
  double alpha_pitch_{0.02};
  double alpha_roll_{0.02};
  double alpha_position_{0.20};
  double yaw_decay_to_prior_{0.85};
  double pitch_decay_to_zero_{0.80};
  double roll_decay_to_zero_{0.70};
  double position_decay_to_prior_{0.70};
};

}  // namespace mtt_perception

