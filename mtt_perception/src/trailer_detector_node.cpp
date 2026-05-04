// Trailer articulation angle estimator — V1
//
// Pipeline:
//   PointCloud2 → crop ROI → optional voxel → 2-D PCA → atan2 → publish
//
// The crop and voxel use PCL (already a dep). The PCA is done directly with
// Eigen on a plain vector of points — no extra PCL passes, no RANSAC.

#include "mtt_perception/trailer_detector_node.hpp"

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>

namespace mtt_perception {

// ── Constructor ───────────────────────────────────────────────────────────────
TrailerDetectorNode::TrailerDetectorNode(const rclcpp::NodeOptions & options)
: Node("trailer_detector_node", options)
{
  // ── Declare parameters ────────────────────────────────────────────────────
  declare_parameter<std::string>("lidar_topic", "/rsairy_ns/points");

  // ROI around the articulation / timon (in cloud frame, verified in CloudCompare)
  declare_parameter<double>("roi_x_min", -0.40);
  declare_parameter<double>("roi_x_max", -0.15);
  declare_parameter<double>("roi_y_min", -0.40);
  declare_parameter<double>("roi_y_max",  0.15);
  declare_parameter<double>("roi_z_min",  0.40);
  declare_parameter<double>("roi_z_max",  1.20);

  // Voxel downsampling (optionnel, keeps PCA fast on dense clouds)
  declare_parameter<double>("voxel_size",   0.02);
  declare_parameter<bool>  ("enable_voxel", true);

  // Detection thresholds
  declare_parameter<int>   ("min_points",    10);
  declare_parameter<double>("pca_ratio_min", 2.0);   // λ_max/λ_min — elongation

  // Sign disambiguation: hitch (pivot) position in cloud frame
  // The axis is flipped so it points FROM hitch TOWARD centroid.
  declare_parameter<double>("hitch_x", 0.0);
  declare_parameter<double>("hitch_y", 0.0);

  // reference_yaw: angle (rad) that corresponds to "straight" articulation = 0.
  // articulation_angle = normalize(yaw_pca - reference_yaw)
  // If cloud is in base_link and timon points along -X when straight,
  // set reference_yaw = M_PI. Default 0.0 keeps the raw PCA angle.
  declare_parameter<double>("reference_yaw", 0.0);

  // ── Load parameters ───────────────────────────────────────────────────────
  roi_x_min_     = static_cast<float>(get_parameter("roi_x_min").as_double());
  roi_x_max_     = static_cast<float>(get_parameter("roi_x_max").as_double());
  roi_y_min_     = static_cast<float>(get_parameter("roi_y_min").as_double());
  roi_y_max_     = static_cast<float>(get_parameter("roi_y_max").as_double());
  roi_z_min_     = static_cast<float>(get_parameter("roi_z_min").as_double());
  roi_z_max_     = static_cast<float>(get_parameter("roi_z_max").as_double());
  voxel_size_    = static_cast<float>(get_parameter("voxel_size").as_double());
  enable_voxel_  = get_parameter("enable_voxel").as_bool();
  min_points_    = get_parameter("min_points").as_int();
  pca_ratio_min_ = get_parameter("pca_ratio_min").as_double();
  hitch_x_       = get_parameter("hitch_x").as_double();
  hitch_y_       = get_parameter("hitch_y").as_double();
  reference_yaw_ = get_parameter("reference_yaw").as_double();

  const auto lidar_topic = get_parameter("lidar_topic").as_string();

  // ── Publishers (SensorDataQoS = BEST_EFFORT + volatile) ───────────────────
  const auto sq = rclcpp::SensorDataQoS();
  angle_pub_     = create_publisher<std_msgs::msg::Float64>(
                     "trailer/articulation_angle",    sq);
  detected_pub_  = create_publisher<std_msgs::msg::Bool>(
                     "trailer/articulation_detected", sq);
  roi_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
                     "trailer/articulation_roi_cloud", sq);
  marker_pub_    = create_publisher<visualization_msgs::msg::Marker>(
                     "trailer/articulation_axis_marker", 10);

  // ── Subscriber ────────────────────────────────────────────────────────────
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    lidar_topic,
    rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
      cloudCallback(msg);
    });

  RCLCPP_INFO(get_logger(),
    "TrailerDetector ready — topic: %s  "
    "ROI x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f]  "
    "voxel %s (%.2fm)  min_pts=%d  pca_ratio≥%.1f",
    lidar_topic.c_str(),
    roi_x_min_, roi_x_max_, roi_y_min_, roi_y_max_, roi_z_min_, roi_z_max_,
    enable_voxel_ ? "ON" : "OFF", voxel_size_,
    min_points_, pca_ratio_min_);
}

// ── Cloud callback ────────────────────────────────────────────────────────────
void TrailerDetectorNode::cloudCallback(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  // ── 1. Decode raw PointCloud2 directly → collect ROI points ──────────────
  // Using PointCloud2ConstIterator avoids a full pcl::fromROSMsg copy when the
  // input is large (Hesai + RSAiry merged can be 100k+ points).
  // We only allocate for the (small) ROI subset.

  std::vector<Eigen::Vector3f> roi_pts;
  roi_pts.reserve(512);  // articulation zone is compact

  sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");

  for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
    const float x = *it_x, y = *it_y, z = *it_z;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
    if (x < roi_x_min_ || x > roi_x_max_) continue;
    if (y < roi_y_min_ || y > roi_y_max_) continue;
    if (z < roi_z_min_ || z > roi_z_max_) continue;
    roi_pts.emplace_back(x, y, z);
  }

  // ── 2. Optional voxel grid (reduces PCA cost for dense clouds) ────────────
  if (enable_voxel_ && !roi_pts.empty()) {
    // Build a tiny PCL cloud from the ROI vector (already small) and voxelise.
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
    tmp->reserve(roi_pts.size());
    for (const auto & p : roi_pts)
      tmp->emplace_back(p.x(), p.y(), p.z());

    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(tmp);
    vg.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
    vg.filter(*tmp);

    roi_pts.clear();
    roi_pts.reserve(tmp->size());
    for (const auto & p : *tmp)
      roi_pts.emplace_back(p.x, p.y, p.z);
  }

  // ── 3. Publish detected=false early if not enough points ─────────────────
  std_msgs::msg::Bool det_msg;
  det_msg.data = false;

  if (static_cast<int>(roi_pts.size()) < min_points_) {
    detected_pub_->publish(det_msg);
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 3000,
      "ROI too sparse: %zu/%d pts", roi_pts.size(), min_points_);
    return;
  }

  // ── 4. Publish ROI cloud (Foxglove debug — only when subscribed) ──────────
  if (roi_cloud_pub_->get_subscription_count() > 0) {
    pcl::PointCloud<pcl::PointXYZ> roi_pcl;
    roi_pcl.reserve(roi_pts.size());
    for (const auto & p : roi_pts)
      roi_pcl.emplace_back(p.x(), p.y(), p.z());

    sensor_msgs::msg::PointCloud2 roi_out;
    pcl::toROSMsg(roi_pcl, roi_out);
    roi_out.header = msg->header;
    roi_cloud_pub_->publish(roi_out);
  }

  // ── 5. PCA 2D on (x, y) ──────────────────────────────────────────────────
  const Eigen::Vector2d hitch(hitch_x_, hitch_y_);
  const PcaResult pca = computePca(roi_pts, hitch);

  if (!pca.valid || pca.ratio < pca_ratio_min_) {
    detected_pub_->publish(det_msg);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
      "PCA weak: ratio=%.2f n=%zu (min=%.1f)",
      pca.ratio, pca.n_points, pca_ratio_min_);
    return;
  }

  // ── 6. Articulation angle relative to MTT forward direction ──────────────
  const double yaw_pca = std::atan2(pca.axis.y(), pca.axis.x());
  const double angle   = normalizeAngle(yaw_pca - reference_yaw_);

  // ── 7. Publish angle + detected ──────────────────────────────────────────
  det_msg.data = true;
  detected_pub_->publish(det_msg);

  std_msgs::msg::Float64 angle_out;
  angle_out.data = angle;
  angle_pub_->publish(angle_out);

  // ── 8. Axis marker (arrow) for Foxglove 3D panel ─────────────────────────
  if (marker_pub_->get_subscription_count() > 0) {
    const double mid_z = 0.5 * (roi_z_min_ + roi_z_max_);
    const double half  = 0.20;  // half-arrow length in metres

    visualization_msgs::msg::Marker mk;
    mk.header   = msg->header;
    mk.ns       = "trailer_articulation";
    mk.id       = 0;
    mk.type     = visualization_msgs::msg::Marker::ARROW;
    mk.action   = visualization_msgs::msg::Marker::ADD;

    // Arrow: tail → head (from centroid minus half to centroid plus half)
    mk.points.resize(2);
    mk.points[0].x = pca.centroid.x() - pca.axis.x() * half;
    mk.points[0].y = pca.centroid.y() - pca.axis.y() * half;
    mk.points[0].z = mid_z;
    mk.points[1].x = pca.centroid.x() + pca.axis.x() * half;
    mk.points[1].y = pca.centroid.y() + pca.axis.y() * half;
    mk.points[1].z = mid_z;

    mk.scale.x = 0.025;   // shaft diameter
    mk.scale.y = 0.050;   // head diameter
    mk.scale.z = 0.050;

    mk.color.r = 1.0f;  // orange
    mk.color.g = 0.5f;
    mk.color.b = 0.0f;
    mk.color.a = 1.0f;

    marker_pub_->publish(mk);
  }

  // ── 9. Throttled debug log with all metrics ───────────────────────────────
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
    "articulation: %.4f rad (%.1f°)  "
    "n=%zu  pca_ratio=%.1f  "
    "centroid (%.3f, %.3f, %.3f)",
    angle, angle * 180.0 / M_PI,
    pca.n_points, pca.ratio,
    pca.centroid.x(), pca.centroid.y(), pca.centroid_z);
}

// ── 2-D PCA ───────────────────────────────────────────────────────────────────
//
// Computes the principal direction of the X-Y projection of the point set.
// Eigen2x2 solver is analytical and O(n) — no iterations, no heap allocs.
//
TrailerDetectorNode::PcaResult TrailerDetectorNode::computePca(
  const std::vector<Eigen::Vector3f> & pts,
  const Eigen::Vector2d & hitch) const
{
  PcaResult r;
  r.n_points = pts.size();
  if (r.n_points < 2) return r;

  // ── Centroid ─────────────────────────────────────────────────────────────
  Eigen::Vector2d mu  = Eigen::Vector2d::Zero();
  double          mu_z = 0.0;
  for (const auto & p : pts) {
    mu.x() += p.x();
    mu.y() += p.y();
    mu_z   += p.z();
  }
  const double inv_n = 1.0 / static_cast<double>(r.n_points);
  mu   *= inv_n;
  mu_z *= inv_n;

  r.centroid   = mu;
  r.centroid_z = mu_z;

  // ── Covariance 2×2 ───────────────────────────────────────────────────────
  Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
  for (const auto & p : pts) {
    const Eigen::Vector2d d(p.x() - mu.x(), p.y() - mu.y());
    cov.noalias() += d * d.transpose();
  }
  cov *= inv_n;

  // ── Eigen decomposition (symmetric 2×2 — fast, analytical) ───────────────
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(cov,
    Eigen::ComputeEigenvectors);

  // Eigenvalues sorted ascending → col(0)=min, col(1)=max.
  // In base_link frame, the timon extends along X-Y → the MAJOR axis (col(1))
  // is collinear with the articulation direction.
  const double lam_max = es.eigenvalues()(1);
  const double lam_min = es.eigenvalues()(0);
  Eigen::Vector2d axis = es.eigenvectors().col(1);   // major = timon direction

  // ── Sign: axis must point FROM hitch TOWARD centroid ─────────────────────
  const Eigen::Vector2d ref = mu - hitch;  // hitch → centroid direction
  if (axis.dot(ref) < 0.0) axis = -axis;

  // ── Linearity ratio (guard for near-zero λ_min) ───────────────────────────
  r.ratio = (lam_min > 1e-9) ? (lam_max / lam_min) : 0.0;

  r.axis  = axis;
  r.valid = std::isfinite(axis.x()) && std::isfinite(axis.y()) &&
            std::isfinite(r.ratio);
  return r;
}

// ── Angle normalisation ───────────────────────────────────────────────────────
double TrailerDetectorNode::normalizeAngle(double a) noexcept
{
  while (a >  M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

}  // namespace mtt_perception

// ── Entry point ───────────────────────────────────────────────────────────────
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<mtt_perception::TrailerDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
