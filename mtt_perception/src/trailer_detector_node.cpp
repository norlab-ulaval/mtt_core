// Trailer articulation angle estimator — V1.5
//
// Pipeline:
//   PointCloud2 → crop ROI → optional voxel → 2-D PCA → Kalman 1D → publish
//
// LiDAR measurement convention:
//   z = normalizeAngle(reference_yaw - yaw_pca)
//
// Kalman state: x = [θ, ω]ᵀ  (angle, angular velocity)
// Prediction uses an optional model-estimated command as a weak input.
// Motor feedback is prepared but disabled by default.

#include "mtt_perception/trailer_detector_node.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>

namespace mtt_perception {

// ── Constructor ──
TrailerDetectorNode::TrailerDetectorNode(const rclcpp::NodeOptions & options)
: Node("trailer_detector_node", options)
{
  // ── Declare parameters ──
  declare_parameter<std::string>("lidar_topic", "/rsairy_ns/points");

  // ROI around the articulation / timon (rsairy frame, verified in CloudCompare)
  declare_parameter<double>("roi_x_min", -0.40);
  declare_parameter<double>("roi_x_max", -0.15);
  declare_parameter<double>("roi_y_min", -0.40);
  declare_parameter<double>("roi_y_max",  0.15);
  declare_parameter<double>("roi_z_min",  0.40);
  declare_parameter<double>("roi_z_max",  1.20);

  // Voxel downsampling
  declare_parameter<double>("voxel_size",   0.02);
  declare_parameter<bool>  ("enable_voxel", true);

  // Detection thresholds
  declare_parameter<int>   ("min_points",    10);
  declare_parameter<double>("pca_ratio_min", 2.0);

  // Sign disambiguation: hitch position in rsairy frame (sensor origin = good default)
  declare_parameter<double>("hitch_x", 0.0);
  declare_parameter<double>("hitch_y", 0.0);

  // LiDAR measurement: z = normalizeAngle(reference_yaw - yaw_pca)
  // Set to the raw PCA yaw observed at zero articulation (≈ -π/2 in rsairy frame).
  declare_parameter<double>("reference_yaw", 0.0);

  // Hitch pivot in base_link — used for the Foxglove marker only
  declare_parameter<double>("hitch_base_x", -1.051);
  declare_parameter<double>("hitch_base_y", -0.051);
  declare_parameter<double>("hitch_base_z",  0.301);
  declare_parameter<double>("marker_length",  0.60);

  // ── Kalman filter tuning ──
  declare_parameter<double>("kf.q_angle",         0.001);  // rad²/s
  declare_parameter<double>("kf.q_omega",         0.01);   // rad²/s³
  declare_parameter<double>("kf.r_lidar_base",    0.005);  // rad²
  declare_parameter<double>("kf.r_lidar_scale",   0.05);
  declare_parameter<double>("kf.gate_soft_sigma", 3.0);
  declare_parameter<double>("kf.gate_hard_sigma", 6.0);
  declare_parameter<double>("kf.gate_inflation",  2.0);

  // ── Command prediction (optional) ──
  // The command topic (model-estimated angle) feeds only the prediction step.
  // It never replaces the LiDAR measurement.
  declare_parameter<bool>       ("command.use_prediction", false);
  declare_parameter<std::string>("command.topic",          "/mtt_articulation_angle");
  declare_parameter<double>     ("command.gain",           0.5);
  declare_parameter<double>     ("command.timeout_sec",    0.2);

  // ── Motor feedback (future measurement — disabled by default) ──
  declare_parameter<bool>       ("motor_feedback.enabled",  false);
  // Default matches the hardware encoder published by mtt_articulation_sensor_node.
  // Enable with motor_feedback.enabled: true in the YAML config.
  declare_parameter<std::string>("motor_feedback.topic",    "/hardware/articulation_angle");
  declare_parameter<double>     ("motor_feedback.variance", 0.01);

  // ── Sliding ROI (future — disabled by default) ──
  declare_parameter<bool>("sliding_roi.enabled", false);

  // ── Load parameters ──
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
  hitch_base_x_  = get_parameter("hitch_base_x").as_double();
  hitch_base_y_  = get_parameter("hitch_base_y").as_double();
  hitch_base_z_  = get_parameter("hitch_base_z").as_double();
  marker_length_ = get_parameter("marker_length").as_double();

  q_angle_         = get_parameter("kf.q_angle").as_double();
  q_omega_         = get_parameter("kf.q_omega").as_double();
  r_lidar_base_    = get_parameter("kf.r_lidar_base").as_double();
  r_lidar_scale_   = get_parameter("kf.r_lidar_scale").as_double();
  gate_soft_sigma_ = get_parameter("kf.gate_soft_sigma").as_double();
  gate_hard_sigma_ = get_parameter("kf.gate_hard_sigma").as_double();
  gate_inflation_  = get_parameter("kf.gate_inflation").as_double();

  use_command_prediction_ = get_parameter("command.use_prediction").as_bool();
  command_gain_           = get_parameter("command.gain").as_double();
  command_timeout_sec_    = get_parameter("command.timeout_sec").as_double();

  use_motor_feedback_       = get_parameter("motor_feedback.enabled").as_bool();
  motor_feedback_variance_  = get_parameter("motor_feedback.variance").as_double();

  use_sliding_roi_ = get_parameter("sliding_roi.enabled").as_bool();

  const auto lidar_topic   = get_parameter("lidar_topic").as_string();

  // ── Publishers (SensorDataQoS = BEST_EFFORT + volatile) ──
  const auto sq = rclcpp::SensorDataQoS();
  angle_pub_     = create_publisher<std_msgs::msg::Float64>(
                     "trailer/articulation_angle",    sq);
  detected_pub_  = create_publisher<std_msgs::msg::Bool>(
                     "trailer/articulation_detected", sq);
  roi_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
                     "trailer/articulation_roi_cloud", sq);
  marker_pub_    = create_publisher<visualization_msgs::msg::Marker>(
                     "trailer/articulation_axis_marker", 10);

  // ── Subscribers ──
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    lidar_topic, rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
      cloudCallback(msg);
    });

  // Command prediction subscriber — only created if enabled
  if (use_command_prediction_) {
    const auto cmd_topic = get_parameter("command.topic").as_string();
    cmd_sub_ = create_subscription<std_msgs::msg::Float64>(
      cmd_topic, rclcpp::SensorDataQoS(),
      [this](std_msgs::msg::Float64::ConstSharedPtr msg) {
        commandCallback(msg);
      });
    RCLCPP_INFO(get_logger(), "KF command prediction ON  topic=%s  gain=%.2f  timeout=%.2fs",
      cmd_topic.c_str(), command_gain_, command_timeout_sec_);
  }

  // Motor feedback subscriber — only created if enabled
  if (use_motor_feedback_) {
    const auto fb_topic = get_parameter("motor_feedback.topic").as_string();
    motor_fb_sub_ = create_subscription<std_msgs::msg::Float64>(
      fb_topic, rclcpp::SensorDataQoS(),
      [this](std_msgs::msg::Float64::ConstSharedPtr msg) {
        motorFeedbackCallback(msg);
      });
    RCLCPP_INFO(get_logger(), "KF motor feedback ON  topic=%s  var=%.4f",
      fb_topic.c_str(), motor_feedback_variance_);
  }

  RCLCPP_INFO(get_logger(),
    "TrailerDetector V1.5 ready — topic: %s  "
    "ROI x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f]  "
    "voxel %s (%.2fm)  min_pts=%d  pca_ratio≥%.1f  "
    "KF q_θ=%.4f q_ω=%.3f R_base=%.4f",
    lidar_topic.c_str(),
    roi_x_min_, roi_x_max_, roi_y_min_, roi_y_max_, roi_z_min_, roi_z_max_,
    enable_voxel_ ? "ON" : "OFF", voxel_size_,
    min_points_, pca_ratio_min_,
    q_angle_, q_omega_, r_lidar_base_);
}

// ── Command callback ──
// Stores the latest model-estimated command angle for use in KF prediction.
// This is a prediction input only — it never overrides the LiDAR measurement.
void TrailerDetectorNode::commandCallback(
  std_msgs::msg::Float64::ConstSharedPtr msg)
{
  last_command_      = msg->data;
  last_command_time_ = now();
}

// ── Motor feedback callback (future) ──
// Stores the latest motor-side angle feedback for a second KF measurement update.
// Not used until motor_feedback.enabled = true.
void TrailerDetectorNode::motorFeedbackCallback(
  std_msgs::msg::Float64::ConstSharedPtr msg)
{
  last_motor_feedback_      = msg->data;
  last_motor_feedback_time_ = now();
}

// ── Publish helper ──
void TrailerDetectorNode::publishAngleAndMarker(
  const std_msgs::msg::Header & header, double angle, bool detected)
{
  // Angle
  std_msgs::msg::Float64 angle_out;
  angle_out.data = angle;
  angle_pub_->publish(angle_out);

  // Detected flag
  std_msgs::msg::Bool det_msg;
  det_msg.data = detected;
  detected_pub_->publish(det_msg);

  // Marker (only when subscribed — zero cost otherwise)
  if (marker_pub_->get_subscription_count() > 0) {
    // π - angle: visual rotation is mirrored vs the angle convention
    const double dir_x = std::cos(M_PI - angle);
    const double dir_y = std::sin(M_PI - angle);
    const double half  = marker_length_ * 0.5;

    visualization_msgs::msg::Marker mk;
    mk.header.stamp    = header.stamp;
    mk.header.frame_id = "base_link";
    mk.ns       = "trailer_articulation";
    mk.id       = 0;
    mk.type     = visualization_msgs::msg::Marker::ARROW;
    mk.action   = visualization_msgs::msg::Marker::ADD;

    // Arrow centred on the hitch pivot
    mk.points.resize(2);
    mk.points[0].x = hitch_base_x_ - dir_x * half;
    mk.points[0].y = hitch_base_y_ - dir_y * half;
    mk.points[0].z = hitch_base_z_;
    mk.points[1].x = hitch_base_x_ + dir_x * half;
    mk.points[1].y = hitch_base_y_ + dir_y * half;
    mk.points[1].z = hitch_base_z_;

    mk.scale.x = 0.030;  // shaft diameter
    mk.scale.y = 0.060;  // head diameter
    mk.scale.z = 0.060;

    // Orange when LiDAR-confirmed, grey when prediction-only
    if (detected) {
      mk.color.r = 1.0f; mk.color.g = 0.5f; mk.color.b = 0.0f;
    } else {
      mk.color.r = 0.6f; mk.color.g = 0.6f; mk.color.b = 0.6f;
    }
    mk.color.a = 1.0f;

    marker_pub_->publish(mk);
  }
}

// ── Cloud callback ──
void TrailerDetectorNode::cloudCallback(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  const rclcpp::Time stamp_now = msg->header.stamp;

  // ── 1. Decode raw PointCloud2 → collect ROI points ──
  // PointCloud2ConstIterator avoids a full pcl::fromROSMsg copy for the full cloud.
  std::vector<Eigen::Vector3f> roi_pts;
  roi_pts.reserve(512);

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

  // ── 2. Optional voxel grid ──
  if (enable_voxel_ && !roi_pts.empty()) {
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

  // ── 3. Kalman predict step ──
  // Always predict (when initialized) so the state stays time-consistent
  // even when PCA fails. If not yet initialized, we wait for the first valid
  // measurement to seed the state (see measurement update below).

  if (kf_initialized_) {
    const double dt = std::clamp(
      (stamp_now - last_predict_time_).seconds(), 0.001, 0.5);
    last_predict_time_ = stamp_now;

    // F: state transition
    Eigen::Matrix2d F;
    F << 1.0, dt,
         0.0, 1.0;

    // Command input u_cmd (prediction only — never a measurement)
    double u_cmd = 0.0;
    if (use_command_prediction_ && last_command_.has_value()) {
      const double age = (stamp_now - last_command_time_).seconds();
      if (age < command_timeout_sec_) {
        u_cmd = last_command_.value();
      }
    }
    const Eigen::Vector2d Bu(command_gain_ * u_cmd * dt, 0.0);

    kf_x_    = F * kf_x_ + Bu;
    kf_x_(0) = normalizeAngle(kf_x_(0));

    // Process noise Q (scaled by dt)
    Eigen::Matrix2d Q = Eigen::Matrix2d::Zero();
    Q(0, 0) = q_angle_ * dt;
    Q(1, 1) = q_omega_ * dt;
    kf_P_ = F * kf_P_ * F.transpose() + Q;
  }

  // ── 4. Sparse ROI → publish predicted angle, no measurement update ──
  if (static_cast<int>(roi_pts.size()) < min_points_) {
    if (kf_initialized_) {
      publishAngleAndMarker(msg->header, kf_x_(0), false);
    }
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 3000,
      "ROI too sparse: %zu/%d pts — publishing predicted angle",
      roi_pts.size(), min_points_);
    return;
  }

  // ── 5. Publish ROI cloud (Foxglove debug — only when subscribed) ──
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

  // ── 6. PCA 2D on (x, y) ──
  const Eigen::Vector2d hitch(hitch_x_, hitch_y_);
  const PcaResult pca = computePca(roi_pts, hitch);

  // ── 7. Weak PCA → publish predicted angle, no measurement update ──
  if (!pca.valid || pca.ratio < pca_ratio_min_) {
    if (kf_initialized_) {
      publishAngleAndMarker(msg->header, kf_x_(0), false);
    }
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
      "PCA weak: ratio=%.2f n=%zu (min=%.1f) — publishing predicted angle",
      pca.ratio, pca.n_points, pca_ratio_min_);
    return;
  }

  // ── 8. LiDAR measurement  z = normalize(reference_yaw − yaw_pca) ──
  const double yaw_pca = std::atan2(pca.axis.y(), pca.axis.x());
  const double z_lidar = normalizeAngle(reference_yaw_ - yaw_pca);

  // ── 9. KF — first measurement seeds the state ──
  if (!kf_initialized_) {
    kf_x_(0)         = z_lidar;
    kf_x_(1)         = 0.0;
    kf_P_            = Eigen::Matrix2d::Identity() * 0.1;
    last_predict_time_ = stamp_now;
    kf_initialized_  = true;
  }

  // ── 10. KF — LiDAR measurement update (adaptive R + soft gating) ──
  // Adaptive R: small when n_points and pca_ratio are high, larger otherwise.
  const double n_factor = std::clamp(
    1.0 - static_cast<double>(pca.n_points - min_points_) / 100.0, 0.0, 1.0);
  const double r_factor = std::clamp(
    1.0 - (pca.ratio - pca_ratio_min_) / 10.0, 0.0, 1.0);
  double R_lidar = r_lidar_base_ + r_lidar_scale_ * (n_factor + r_factor);

  // Innovation: angle wrapping handled by normalizeAngle
  const double innov = normalizeAngle(z_lidar - kf_x_(0));

  // Soft gating: inflate R for large innovations (don't hard-reject unless extreme)
  const double innov_sigma = std::sqrt(std::max(kf_P_(0, 0) + R_lidar, 1e-9));
  const double mahal       = std::abs(innov) / innov_sigma;

  if (mahal > gate_soft_sigma_) {
    R_lidar *= 1.0 + gate_inflation_ * (mahal - gate_soft_sigma_);
  }

  if (mahal < gate_hard_sigma_) {
    // H = [1, 0] → S = P(0,0) + R,  K = P.col(0) / S
    const double         S = kf_P_(0, 0) + R_lidar;
    const Eigen::Vector2d K = kf_P_.col(0) / S;

    kf_x_    += K * innov;
    kf_x_(0)  = normalizeAngle(kf_x_(0));

    // Joseph form: P = (I − K·H)·P  (numerically stable)
    Eigen::Matrix2d I_KH = Eigen::Matrix2d::Identity();
    I_KH(0, 0) -= K(0);
    I_KH(1, 0) -= K(1);
    kf_P_ = I_KH * kf_P_;
  } else {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "KF hard gate triggered: mahal=%.1f > %.1f — measurement rejected",
      mahal, gate_hard_sigma_);
  }

  // ── 11. Motor feedback measurement update (future, disabled by default) ──
  // When motor_feedback.enabled=true: second KF update with motor angle.
  // Less trusted than LiDAR initially; variance tuned via motor_feedback.variance.
  if (use_motor_feedback_ && last_motor_feedback_.has_value()) {
    const double fb_age = (stamp_now - last_motor_feedback_time_).seconds();
    if (fb_age < 0.2) {
      const double innov_m = normalizeAngle(last_motor_feedback_.value() - kf_x_(0));
      const double S_m     = kf_P_(0, 0) + motor_feedback_variance_;
      const Eigen::Vector2d K_m = kf_P_.col(0) / S_m;

      kf_x_    += K_m * innov_m;
      kf_x_(0)  = normalizeAngle(kf_x_(0));

      Eigen::Matrix2d I_KH_m = Eigen::Matrix2d::Identity();
      I_KH_m(0, 0) -= K_m(0);
      I_KH_m(1, 0) -= K_m(1);
      kf_P_ = I_KH_m * kf_P_;
    }
  }

  // ── 12. Publish angle + marker (LiDAR-confirmed) ──
  const double angle = kf_x_(0);
  publishAngleAndMarker(msg->header, angle, true);

  // ── 13. Throttled debug log ──
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
    "articulation: %.4f rad (%.1f°)  ω=%.3f rad/s  "
    "n=%zu  pca_ratio=%.1f  R=%.4f  innov=%.4f  mahal=%.1f",
    angle, angle * 180.0 / M_PI,
    kf_x_(1),
    pca.n_points, pca.ratio,
    R_lidar, innov, mahal);
}

// ── 2-D PCA ──
//
// Computes the principal direction of the X-Y projection of the point set.
// Eigen 2×2 solver is analytical and O(n) — no iterations, no heap allocs.
//
TrailerDetectorNode::PcaResult TrailerDetectorNode::computePca(
  const std::vector<Eigen::Vector3f> & pts,
  const Eigen::Vector2d & hitch) const
{
  PcaResult r;
  r.n_points = pts.size();
  if (r.n_points < 2) return r;

  // ── Centroid ──
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

  // ── Covariance 2×2 ──
  Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
  for (const auto & p : pts) {
    const Eigen::Vector2d d(p.x() - mu.x(), p.y() - mu.y());
    cov.noalias() += d * d.transpose();
  }
  cov *= inv_n;

  // ── Eigen decomposition (symmetric 2×2 — fast, analytical) ──
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(cov, Eigen::ComputeEigenvectors);

  // Eigenvalues sorted ascending → col(0)=min, col(1)=max.
  // In rsairy frame, the timon's cross-section dominates XY spread → col(1)
  // is the axis along the timon direction.
  const double lam_max = es.eigenvalues()(1);
  const double lam_min = es.eigenvalues()(0);
  Eigen::Vector2d axis = es.eigenvectors().col(1);

  // Sign: axis must point FROM hitch TOWARD centroid
  const Eigen::Vector2d ref = mu - hitch;
  if (axis.dot(ref) < 0.0) axis = -axis;

  // Linearity ratio (guard for near-zero λ_min)
  r.ratio = (lam_min > 1e-9) ? (lam_max / lam_min) : 0.0;

  r.axis  = axis;
  r.valid = std::isfinite(axis.x()) && std::isfinite(axis.y()) &&
            std::isfinite(r.ratio);
  return r;
}

// ── Angle normalisation ──
double TrailerDetectorNode::normalizeAngle(double a) noexcept
{
  while (a >  M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

}  // namespace mtt_perception

// ── Entry point ──
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<mtt_perception::TrailerDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
