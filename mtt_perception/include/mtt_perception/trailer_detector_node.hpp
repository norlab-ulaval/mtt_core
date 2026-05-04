// Trailer articulation angle estimator — V1
//
// Crops a tight ROI around the articulation / timon zone, runs a 2D PCA on
// the X-Y projection, and publishes the principal-axis angle relative to the
// MTT forward direction.
//
// All heavy types (PCL, Eigen) are kept in the .cpp to minimise recompilation.

#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <Eigen/Core>

namespace mtt_perception {

class TrailerDetectorNode : public rclcpp::Node
{
public:
  explicit TrailerDetectorNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ── Result of the 2-D PCA ─────────────────────────────────────────────────
  struct PcaResult {
    bool           valid{false};
    Eigen::Vector2d axis{0.0, 1.0};   // sign-corrected principal direction
    Eigen::Vector2d centroid{0.0, 0.0};
    double          ratio{0.0};        // λ_max / λ_min  (linearity score)
    double          centroid_z{0.0};   // mean Z of ROI cloud (logged)
    std::size_t     n_points{0};
  };

  // ── Processing ────────────────────────────────────────────────────────────
  void cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  PcaResult computePca(
    const std::vector<Eigen::Vector3f> & pts,
    const Eigen::Vector2d & hitch) const;

  static double normalizeAngle(double a) noexcept;

  // ── Publishers ────────────────────────────────────────────────────────────
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr         angle_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr            detected_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr  roi_cloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  // ── Subscriber ────────────────────────────────────────────────────────────
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;

  // ── Parameters (loaded once at construction) ──────────────────────────────
  float  roi_x_min_, roi_x_max_;
  float  roi_y_min_, roi_y_max_;
  float  roi_z_min_, roi_z_max_;
  float  voxel_size_;
  bool   enable_voxel_;
  int    min_points_;
  double pca_ratio_min_;
  double hitch_x_, hitch_y_;    // reference point in cloud frame for sign fix
  double reference_yaw_;        // subtract from PCA yaw → articulation angle
};

}  // namespace mtt_perception
