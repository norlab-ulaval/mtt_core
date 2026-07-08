// Trailer articulation angle estimator — V1.5
//
// Crops a tight ROI around the articulation / timon zone, runs a 2D PCA on
// the X-Y projection, and feeds the result into a 1D Kalman filter.
//
// Measurement convention (LiDAR):
//   z = normalizeAngle(reference_yaw - yaw_pca)
//
// All heavy types (PCL, Eigen) are kept in the .cpp to minimise recompilation.

#pragma once

#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "mtt_perception/adaptive_sensor_confidence.hpp"

#include <Eigen/Core>

namespace mtt_perception {

class TrailerDetectorNode : public rclcpp::Node
{
public:
  explicit TrailerDetectorNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ── Result of the 2-D PCA ──
  struct PcaResult {
    bool           valid{false};
    Eigen::Vector2d axis{0.0, 1.0};   // sign-corrected principal direction
    Eigen::Vector2d centroid{0.0, 0.0};
    double          ratio{0.0};        // λ_max / λ_min  (linearity score)
    double          centroid_z{0.0};   // mean Z of ROI cloud (logged)
    std::size_t     n_points{0};
  };

  // ── Processing ──
  void cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  // Optional command prediction + secondary STM feedback callbacks
  void commandCallback(std_msgs::msg::Float64::ConstSharedPtr msg);
  void motorFeedbackCallback(std_msgs::msg::Float64::ConstSharedPtr msg);

  PcaResult computePca(
    const std::vector<Eigen::Vector3f> & pts,
    const Eigen::Vector2d & hitch) const;

  // Helper: publish angle + marker using current KF state
  void publishAngleAndMarker(
    const std_msgs::msg::Header & header, double angle, bool detected);

  static double normalizeAngle(double a) noexcept;

  // ── Publishers ──
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr         angle_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr            detected_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr         stm_confidence_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr  roi_cloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  // ── Subscribers ──
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  // Created only if use_command_prediction_ = true:
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr cmd_sub_;
  // Created only if use_motor_feedback_ = true:
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr motor_fb_sub_;

  // ── Parameters (loaded once at construction) ──
  float  roi_x_min_, roi_x_max_;
  float  roi_y_min_, roi_y_max_;
  float  roi_z_min_, roi_z_max_;
  float  voxel_size_;
  bool   enable_voxel_;
  int    min_points_;
  double pca_ratio_min_;
  double hitch_x_, hitch_y_;    // sign disambiguation in rsairy frame
  double reference_yaw_;        // LiDAR measurement: z = normalize(ref - yaw_pca)

  // Hitch pivot in base_link (marker only)
  double hitch_base_x_, hitch_base_y_, hitch_base_z_;
  double marker_length_;

  // ── Kalman filter — state and tuning ──
  // State: x = [θ, ω]ᵀ  (angle rad, angular velocity rad/s)
  Eigen::Vector2d kf_x_{0.0, 0.0};
  Eigen::Matrix2d kf_P_{Eigen::Matrix2d::Identity() * 0.1};
  rclcpp::Time    last_predict_time_;
  bool            kf_initialized_{false};

  double q_angle_;          // process noise on angle    (rad²/s)
  double q_omega_;          // process noise on omega    (rad²/s³)
  double r_lidar_base_;     // base LiDAR variance       (rad²)
  double r_lidar_scale_;    // additional variance for poor PCA quality
  double gate_soft_sigma_;  // start inflating R         (σ)
  double gate_hard_sigma_;  // reject measurement entirely (σ)
  double gate_inflation_;   // R multiplier rate beyond soft gate

  // ── Command prediction (optional) ──
  // /mtt_articulation_angle (model-estimated) feeds the prediction step only.
  // It never replaces the LiDAR measurement.
  bool   use_command_prediction_;
  double command_gain_;          // k_cmd: command → angular rate contribution
  double command_timeout_sec_;
  std::optional<double> last_command_;
  rclcpp::Time          last_command_time_;

  // ── Adaptive STM feedback (secondary to LiDAR) ──
  bool   use_motor_feedback_;
  double motor_feedback_variance_;
  double motor_feedback_timeout_s_;
  double motor_min_confidence_;
  double motor_lidar_variance_multiplier_;
  double motor_gate_hard_sigma_;
  AdaptiveSensorConfidence motor_confidence_;
  std::optional<double> last_motor_feedback_;
  rclcpp::Time          last_motor_feedback_time_;

  // ── Sliding ROI (future — disabled by default) ──
  // When enabled: shift ROI centre based on predicted angle.
  bool   use_sliding_roi_;
};

}  // namespace mtt_perception
