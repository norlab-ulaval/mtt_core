// Trailer pose estimator — V4.0
// Prior-locked constrained SE(3) estimator for the MTT trailer.

#include "mtt_perception/trailer_pose_node.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <sstream>
#include <unordered_map>

#include <Eigen/Dense>

#include "builtin_interfaces/msg/duration.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "visualization_msgs/msg/marker.hpp"

namespace mtt_perception
{
namespace
{
constexpr double kPi = 3.141592653589793238462643383279502884;

struct VoxelKey
{
  int s{0};
  int l{0};
  int h{0};

  bool operator==(const VoxelKey & other) const noexcept
  {
    return s == other.s && l == other.l && h == other.h;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey & key) const noexcept
  {
    const auto a = static_cast<std::uint64_t>(static_cast<std::int64_t>(key.s)) * 73856093ULL;
    const auto b = static_cast<std::uint64_t>(static_cast<std::int64_t>(key.l)) * 19349663ULL;
    const auto c = static_cast<std::uint64_t>(static_cast<std::int64_t>(key.h)) * 83492791ULL;
    return static_cast<std::size_t>(a ^ b ^ c);
  }
};

struct VoxelAccum
{
  Eigen::Vector3d p{0.0, 0.0, 0.0};
  double s{0.0};
  double l{0.0};
  double h{0.0};
  int n{0};
};

geometry_msgs::msg::Point pointMsg(const Eigen::Vector3d & p)
{
  geometry_msgs::msg::Point out;
  out.x = p.x();
  out.y = p.y();
  out.z = p.z();
  return out;
}

builtin_interfaces::msg::Duration markerLifetime(const double seconds)
{
  builtin_interfaces::msg::Duration out;
  out.sec = static_cast<int32_t>(std::floor(seconds));
  out.nanosec = static_cast<uint32_t>((seconds - static_cast<double>(out.sec)) * 1e9);
  return out;
}

}  // namespace

TrailerPoseNode::TrailerPoseNode(const rclcpp::NodeOptions & options)
: Node("trailer_pose_node", options),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_)
{
  declare_parameter<std::string>("lidar_topic", "/rsairy_ns/points");
  declare_parameter<std::string>("angle_topic", "/trailer/articulation_angle");
  declare_parameter<std::string>("command_angle_topic", "/mtt_articulation_angle");
  declare_parameter<bool>("use_command_as_primary", false);
  declare_parameter<double>("command_fallback_timeout", 1.0);

  declare_parameter<double>("hitch_base_x", -1.45);
  declare_parameter<double>("hitch_base_y", -0.085);
  declare_parameter<double>("hitch_base_z", 0.35);
  declare_parameter<double>("trailer_body_offset", 0.90);
  declare_parameter<double>("trailer_front_offset", 0.05);
  declare_parameter<double>("trailer_rear_offset", 1.90);
  declare_parameter<double>("trailer_width", 0.80);
  declare_parameter<double>("yaw_offset", 0.0);

  declare_parameter<double>("roi_half_x", 0.85);
  declare_parameter<double>("roi_half_y", 0.70);
  declare_parameter<double>("roi_half_z", 1.10);
  declare_parameter<double>("roi_marker_vertical_scale_factor", 0.60);

  declare_parameter<bool>("publish_trailer_roi_cloud", true);
  declare_parameter<bool>("enable_voxel", true);
  declare_parameter<double>("voxel_size", 0.04);

  declare_parameter<bool>("enable_ground_filter", true);
  declare_parameter<double>("ground_quantile", 0.10);
  declare_parameter<double>("ground_margin", 0.06);

  declare_parameter<int>("min_points", 30);
  declare_parameter<int>("min_points_after_ground", 20);
  declare_parameter<double>("min_span_s", 0.45);
  declare_parameter<double>("min_centerline_pca_ratio", 2.2);
  declare_parameter<double>("max_yaw_correction", 0.20);
  declare_parameter<double>("max_yaw_jump", 0.15);

  declare_parameter<double>("max_position_correction_s", 0.05);
  declare_parameter<double>("max_position_correction_l", 0.08);
  declare_parameter<double>("max_position_correction_h", 0.06);

  declare_parameter<bool>("enable_pitch_estimation", true);
  declare_parameter<double>("max_pitch", 0.12);
  declare_parameter<double>("max_pitch_rms", 0.08);

  declare_parameter<bool>("enable_roll_estimation", false);
  declare_parameter<double>("max_roll", 0.08);
  declare_parameter<double>("width_tolerance", 0.35);
  declare_parameter<int>("min_side_points", 20);

  declare_parameter<double>("alpha_yaw", 0.25);
  declare_parameter<double>("alpha_pitch", 0.02);
  declare_parameter<double>("alpha_roll", 0.02);
  declare_parameter<double>("alpha_position", 0.20);
  declare_parameter<double>("yaw_decay_to_prior", 0.85);
  declare_parameter<double>("pitch_decay_to_zero", 0.80);
  declare_parameter<double>("roll_decay_to_zero", 0.70);
  declare_parameter<double>("position_decay_to_prior", 0.70);

  lidar_topic_ = get_parameter("lidar_topic").as_string();
  angle_topic_ = get_parameter("angle_topic").as_string();
  command_angle_topic_ = get_parameter("command_angle_topic").as_string();
  use_command_as_primary_ = get_parameter("use_command_as_primary").as_bool();
  command_fallback_timeout_ = get_parameter("command_fallback_timeout").as_double();

  hitch_base_x_ = get_parameter("hitch_base_x").as_double();
  hitch_base_y_ = get_parameter("hitch_base_y").as_double();
  hitch_base_z_ = get_parameter("hitch_base_z").as_double();
  trailer_body_offset_ = get_parameter("trailer_body_offset").as_double();
  trailer_front_offset_ = get_parameter("trailer_front_offset").as_double();
  trailer_rear_offset_ = get_parameter("trailer_rear_offset").as_double();
  trailer_width_ = get_parameter("trailer_width").as_double();
  yaw_offset_ = get_parameter("yaw_offset").as_double();

  roi_half_x_ = get_parameter("roi_half_x").as_double();
  roi_half_y_ = get_parameter("roi_half_y").as_double();
  roi_half_z_ = get_parameter("roi_half_z").as_double();
  roi_marker_vertical_scale_factor_ = get_parameter("roi_marker_vertical_scale_factor").as_double();

  publish_trailer_roi_cloud_ = get_parameter("publish_trailer_roi_cloud").as_bool();
  enable_voxel_ = get_parameter("enable_voxel").as_bool();
  voxel_size_ = get_parameter("voxel_size").as_double();

  enable_ground_filter_ = get_parameter("enable_ground_filter").as_bool();
  ground_quantile_ = get_parameter("ground_quantile").as_double();
  ground_margin_ = get_parameter("ground_margin").as_double();

  min_points_ = get_parameter("min_points").as_int();
  min_points_after_ground_ = get_parameter("min_points_after_ground").as_int();
  min_span_s_ = get_parameter("min_span_s").as_double();
  min_centerline_pca_ratio_ = get_parameter("min_centerline_pca_ratio").as_double();
  max_yaw_correction_ = get_parameter("max_yaw_correction").as_double();
  max_yaw_jump_ = get_parameter("max_yaw_jump").as_double();

  max_position_correction_s_ = get_parameter("max_position_correction_s").as_double();
  max_position_correction_l_ = get_parameter("max_position_correction_l").as_double();
  max_position_correction_h_ = get_parameter("max_position_correction_h").as_double();

  enable_pitch_estimation_ = get_parameter("enable_pitch_estimation").as_bool();
  max_pitch_ = get_parameter("max_pitch").as_double();
  max_pitch_rms_ = get_parameter("max_pitch_rms").as_double();

  enable_roll_estimation_ = get_parameter("enable_roll_estimation").as_bool();
  max_roll_ = get_parameter("max_roll").as_double();
  width_tolerance_ = get_parameter("width_tolerance").as_double();
  min_side_points_ = get_parameter("min_side_points").as_int();

  alpha_yaw_ = get_parameter("alpha_yaw").as_double();
  alpha_pitch_ = get_parameter("alpha_pitch").as_double();
  alpha_roll_ = get_parameter("alpha_roll").as_double();
  alpha_position_ = get_parameter("alpha_position").as_double();
  yaw_decay_to_prior_ = get_parameter("yaw_decay_to_prior").as_double();
  pitch_decay_to_zero_ = get_parameter("pitch_decay_to_zero").as_double();
  roll_decay_to_zero_ = get_parameter("roll_decay_to_zero").as_double();
  position_decay_to_prior_ = get_parameter("position_decay_to_prior").as_double();

  const auto sensor_qos = rclcpp::SensorDataQoS();
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/trailer/pose", sensor_qos);
  pose_prior_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/trailer/pose_prior", sensor_qos);
  pose_raw_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/trailer/pose_raw", sensor_qos);
  markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/trailer/body_markers", 10);
  trailer_roi_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "/trailer/trailer_roi_cloud", sensor_qos);

  confidence_pub_ = create_publisher<std_msgs::msg::Float64>("/trailer/pose_confidence", sensor_qos);
  yaw_prior_pub_ = create_publisher<std_msgs::msg::Float64>("/trailer/yaw_prior", sensor_qos);
  yaw_raw_pub_ = create_publisher<std_msgs::msg::Float64>("/trailer/yaw_raw", sensor_qos);
  yaw_used_pub_ = create_publisher<std_msgs::msg::Float64>("/trailer/yaw_used", sensor_qos);
  yaw_corr_raw_pub_ = create_publisher<std_msgs::msg::Float64>("/trailer/yaw_correction_raw", sensor_qos);
  yaw_corr_used_pub_ = create_publisher<std_msgs::msg::Float64>("/trailer/yaw_correction_used", sensor_qos);
  pitch_raw_pub_ = create_publisher<std_msgs::msg::Float64>("/trailer/pitch_raw", sensor_qos);
  pitch_used_pub_ = create_publisher<std_msgs::msg::Float64>("/trailer/pitch_used", sensor_qos);
  roll_used_pub_ = create_publisher<std_msgs::msg::Float64>("/trailer/roll_used", sensor_qos);
  roi_count_pub_ = create_publisher<std_msgs::msg::Float64>("/trailer/roi_point_count", sensor_qos);
  roi_count_after_ground_pub_ = create_publisher<std_msgs::msg::Float64>(
    "/trailer/roi_point_count_after_ground", sensor_qos);
  ground_threshold_pub_ = create_publisher<std_msgs::msg::Float64>(
    "/trailer/ground_threshold", sensor_qos);
  ground_removed_pub_ = create_publisher<std_msgs::msg::Float64>(
    "/trailer/ground_removed_count", sensor_qos);
  span_s_pub_ = create_publisher<std_msgs::msg::Float64>("/trailer/span_s", sensor_qos);
  pca_ratio_pub_ = create_publisher<std_msgs::msg::Float64>("/trailer/centerline_pca_ratio", sensor_qos);
  measurement_valid_pub_ = create_publisher<std_msgs::msg::Float64>(
    "/trailer/measurement_valid", sensor_qos);
  pitch_valid_pub_ = create_publisher<std_msgs::msg::Float64>("/trailer/pitch_valid", sensor_qos);
  roll_valid_pub_ = create_publisher<std_msgs::msg::Float64>("/trailer/roll_valid", sensor_qos);
  command_residual_pub_ = create_publisher<std_msgs::msg::Float64>(
    "/trailer/command_residual", sensor_qos);

  articulation_angle_sub_ = create_subscription<std_msgs::msg::Float64>(
    angle_topic_, sensor_qos,
    [this](std_msgs::msg::Float64::ConstSharedPtr msg) { articulationAngleCallback(msg); });

  command_angle_sub_ = create_subscription<std_msgs::msg::Float64>(
    command_angle_topic_, sensor_qos,
    [this](std_msgs::msg::Float64::ConstSharedPtr msg) { commandAngleCallback(msg); });

  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    lidar_topic_, sensor_qos,
    [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { cloudCallback(msg); });

  RCLCPP_INFO(
    get_logger(),
    "TrailerPose V4.0 ready — prior=/trailer/articulation_angle, command=%s, lidar=%s, ROI=(%.2f %.2f %.2f)",
    command_angle_topic_.c_str(), lidar_topic_.c_str(), roi_half_x_, roi_half_y_, roi_half_z_);
}

void TrailerPoseNode::articulationAngleCallback(std_msgs::msg::Float64::ConstSharedPtr msg)
{
  last_articulation_angle_ = normalizeAngle(msg->data);
  last_articulation_angle_time_ = get_clock()->now();
  angle_received_ = true;

  if (last_command_angle_.has_value()) {
    command_residual_pub_->publish(
      f64(normalizeAngle(last_articulation_angle_ - *last_command_angle_)));
  }
}

void TrailerPoseNode::commandAngleCallback(std_msgs::msg::Float64::ConstSharedPtr msg)
{
  last_command_angle_ = normalizeAngle(msg->data);
  last_command_angle_time_ = get_clock()->now();
}

bool TrailerPoseNode::updateTfCache(const std::string & cloud_frame)
{
  const std::string frame = cloud_frame.empty() ? std::string("base_link") : cloud_frame;
  if (tf_cached_ && frame == cached_cloud_frame_) {
    return true;
  }

  if (frame == "base_link") {
    R_cloud_base_.setIdentity();
    t_cloud_base_.setZero();
    cached_cloud_frame_ = frame;
    tf_cached_ = true;
    return true;
  }

  try {
    const auto tf = tf_buffer_.lookupTransform(
      frame, "base_link", tf2::TimePointZero, tf2::durationFromSec(0.1));

    const auto & tr = tf.transform.translation;
    const auto & qr = tf.transform.rotation;
    Eigen::Quaterniond q(qr.w, qr.x, qr.y, qr.z);
    q.normalize();

    R_cloud_base_ = q.toRotationMatrix();
    t_cloud_base_ = Eigen::Vector3d(tr.x, tr.y, tr.z);
    cached_cloud_frame_ = frame;
    tf_cached_ = true;

    RCLCPP_INFO(get_logger(), "Cached TF %s <- base_link", frame.c_str());
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Waiting for TF %s <- base_link: %s", frame.c_str(), ex.what());
    return false;
  }
}

double TrailerPoseNode::activeTheta() const
{
  const auto now = get_clock()->now();

  if (use_command_as_primary_ && last_command_angle_.has_value()) {
    return *last_command_angle_;
  }

  if (angle_received_) {
    const double age = (now - last_articulation_angle_time_).seconds();
    if (age <= command_fallback_timeout_ || !last_command_angle_.has_value()) {
      return last_articulation_angle_;
    }
  }

  if (last_command_angle_.has_value()) {
    return *last_command_angle_;
  }

  return 0.0;
}

TrailerPoseNode::PriorFrame TrailerPoseNode::buildPriorFrame(const double theta) const
{
  PriorFrame p;
  p.theta = theta;
  p.yaw_prior = normalizeAngle(kPi - theta + yaw_offset_);

  p.hitch_base = Eigen::Vector3d(hitch_base_x_, hitch_base_y_, hitch_base_z_);
  p.u_base = Eigen::Vector3d(std::cos(p.yaw_prior), std::sin(p.yaw_prior), 0.0).normalized();
  p.n_base = Eigen::Vector3d(-std::sin(p.yaw_prior), std::cos(p.yaw_prior), 0.0).normalized();
  p.k_base = Eigen::Vector3d(0.0, 0.0, 1.0);
  p.p0_base = p.hitch_base + trailer_body_offset_ * p.u_base;

  p.p0_cloud = R_cloud_base_ * p.p0_base + t_cloud_base_;
  p.u_cloud = (R_cloud_base_ * p.u_base).normalized();
  p.n_cloud = (R_cloud_base_ * p.n_base).normalized();
  p.k_cloud = (R_cloud_base_ * p.k_base).normalized();

  // Make the frame strictly orthonormal after TF numerical noise.
  p.n_cloud = (p.n_cloud - p.n_cloud.dot(p.u_cloud) * p.u_cloud).normalized();
  p.k_cloud = p.u_cloud.cross(p.n_cloud).normalized();
  p.n_cloud = p.k_cloud.cross(p.u_cloud).normalized();
  return p;
}

std::vector<TrailerPoseNode::LocalPoint> TrailerPoseNode::cropToOrientedRoi(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const PriorFrame & prior) const
{
  std::vector<LocalPoint> out;
  out.reserve(2048);

  sensor_msgs::PointCloud2ConstIterator<float> it_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> it_y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> it_z(cloud, "z");

  for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
    const double x = static_cast<double>(*it_x);
    const double y = static_cast<double>(*it_y);
    const double z = static_cast<double>(*it_z);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }

    const Eigen::Vector3d p_cloud(x, y, z);
    const Eigen::Vector3d q = p_cloud - prior.p0_cloud;
    const double s = q.dot(prior.u_cloud);
    const double l = q.dot(prior.n_cloud);
    const double h = q.dot(prior.k_cloud);

    if (std::abs(s) > roi_half_x_ || std::abs(l) > roi_half_y_ || std::abs(h) > roi_half_z_) {
      continue;
    }

    LocalPoint lp;
    lp.p_cloud = p_cloud;
    lp.s = s;
    lp.l = l;
    lp.h = h;
    out.push_back(lp);
  }

  return out;
}

std::vector<TrailerPoseNode::LocalPoint> TrailerPoseNode::removeGroundByQuantile(
  const std::vector<LocalPoint> & points,
  Measurement & meas) const
{
  meas.n_roi_raw = points.size();
  if (!enable_ground_filter_ || points.empty()) {
    meas.n_after_ground = points.size();
    return points;
  }

  std::vector<double> h_values;
  h_values.reserve(points.size());
  for (const auto & p : points) {
    h_values.push_back(p.h);
  }

  const double h_floor = quantile(h_values, ground_quantile_);
  const double threshold = h_floor + ground_margin_;
  meas.ground_threshold = threshold;

  std::vector<LocalPoint> out;
  out.reserve(points.size());
  for (const auto & p : points) {
    if (p.h > threshold) {
      out.push_back(p);
    }
  }

  meas.n_after_ground = out.size();
  meas.ground_removed = points.size() - out.size();
  return out;
}

std::vector<TrailerPoseNode::LocalPoint> TrailerPoseNode::voxelDownsampleLocal(
  const std::vector<LocalPoint> & points) const
{
  if (!enable_voxel_ || voxel_size_ <= 1e-6 || points.empty()) {
    return points;
  }

  std::unordered_map<VoxelKey, VoxelAccum, VoxelKeyHash> voxels;
  voxels.reserve(points.size());
  const double inv = 1.0 / voxel_size_;

  for (const auto & p : points) {
    const VoxelKey key{
      static_cast<int>(std::floor(p.s * inv)),
      static_cast<int>(std::floor(p.l * inv)),
      static_cast<int>(std::floor(p.h * inv))};

    auto & acc = voxels[key];
    acc.p += p.p_cloud;
    acc.s += p.s;
    acc.l += p.l;
    acc.h += p.h;
    ++acc.n;
  }

  std::vector<LocalPoint> out;
  out.reserve(voxels.size());
  for (const auto & kv : voxels) {
    const auto & a = kv.second;
    if (a.n <= 0) {
      continue;
    }
    LocalPoint p;
    const double inv_n = 1.0 / static_cast<double>(a.n);
    p.p_cloud = a.p * inv_n;
    p.s = a.s * inv_n;
    p.l = a.l * inv_n;
    p.h = a.h * inv_n;
    out.push_back(p);
  }
  return out;
}

TrailerPoseNode::Measurement TrailerPoseNode::estimateMeasurement(
  const std::vector<LocalPoint> & points) const
{
  Measurement m;
  m.n_after_voxel = points.size();
  if (static_cast<int>(points.size()) < min_points_after_ground_) {
    return m;
  }

  std::vector<double> s_values;
  std::vector<double> l_values;
  std::vector<double> h_values;
  s_values.reserve(points.size());
  l_values.reserve(points.size());
  h_values.reserve(points.size());
  for (const auto & p : points) {
    s_values.push_back(p.s);
    l_values.push_back(p.l);
    h_values.push_back(p.h);
  }

  const double q05 = quantile(s_values, 0.05);
  const double q95 = quantile(s_values, 0.95);
  m.s_min = q05;
  m.s_max = q95;
  m.span_s = std::max(0.0, q95 - q05);

  // ── PCA 2D on (s,l) ──
  double mean_s = 0.0;
  double mean_l = 0.0;
  for (const auto & p : points) {
    mean_s += p.s;
    mean_l += p.l;
  }
  mean_s /= static_cast<double>(points.size());
  mean_l /= static_cast<double>(points.size());

  double c_ss = 0.0;
  double c_sl = 0.0;
  double c_ll = 0.0;
  for (const auto & p : points) {
    const double ds = p.s - mean_s;
    const double dl = p.l - mean_l;
    c_ss += ds * ds;
    c_sl += ds * dl;
    c_ll += dl * dl;
  }
  c_ss /= static_cast<double>(points.size());
  c_sl /= static_cast<double>(points.size());
  c_ll /= static_cast<double>(points.size());

  Eigen::Matrix2d C;
  C << c_ss, c_sl,
       c_sl, c_ll;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(C);
  if (es.info() == Eigen::Success) {
    const double lambda_min = std::max(es.eigenvalues()(0), 1e-9);
    const double lambda_max = std::max(es.eigenvalues()(1), 1e-9);
    m.pca_ratio = lambda_max / lambda_min;

    Eigen::Vector2d v = es.eigenvectors().col(1);
    if (v.x() < 0.0) {
      v = -v;
    }
    m.yaw_corr_raw = normalizeAngle(std::atan2(v.y(), v.x()));

    const bool enough_points = static_cast<int>(points.size()) >= min_points_after_ground_;
    const bool enough_span = m.span_s >= min_span_s_;
    const bool pca_strong = m.pca_ratio >= min_centerline_pca_ratio_;
    const bool yaw_small = std::abs(m.yaw_corr_raw) <= max_yaw_correction_;
    m.yaw_valid = enough_points && enough_span && pca_strong && yaw_small;

    if (m.yaw_valid) {
      m.yaw_corr_used = clamp(m.yaw_corr_raw, -max_yaw_correction_, max_yaw_correction_);
    }
  }

  // Position correction: robust but intentionally tiny. The cube/prior is the
  // main position source; the cloud only nudges it.
  m.corr_s = clamp(quantile(s_values, 0.50), -max_position_correction_s_, max_position_correction_s_);
  m.corr_l = clamp(quantile(l_values, 0.50), -max_position_correction_l_, max_position_correction_l_);
  m.corr_h = clamp(quantile(h_values, 0.50), -max_position_correction_h_, max_position_correction_h_);

  // Pitch: h = a*s + b, only if the vertical line fit is clean.
  if (enable_pitch_estimation_ && m.span_s >= min_span_s_) {
    const LineFit fit = robustLineFit(s_values, h_values);
    if (fit.valid) {
      m.pitch_raw = std::atan(fit.a);
      m.pitch_rms = fit.rms;
      m.pitch_valid = std::abs(m.pitch_raw) <= max_pitch_ && fit.rms <= max_pitch_rms_;
      if (m.pitch_valid) {
        m.pitch_used = clamp(m.pitch_raw, -max_pitch_, max_pitch_);
      }
    }
  }

  // ── Two-sided lateral line fit (left/right edges of the trailer body) ──
  // Preferred over the whole-cloud 2D PCA above for yaw AND used for roll.
  // The PCA axis above is UNDIRECTED (an eigenvector has no sign of its own —
  // atan2(v.y,v.x) after the "v.x()>=0" heuristic only stays correct for SMALL
  // corrections; near a true correction close to +/-90 deg, noise can flip
  // which of the two antipodal eigenvectors gets picked, producing a spurious
  // ~180 deg jump in the published yaw). A per-side regression l = a*s + b has
  // no such ambiguity: s is anchored to the known prior direction (not a raw
  // PCA axis), so the fitted slope is a genuine SIGNED yaw correction. Splitting
  // with a q30/q70 gap (not the median) leaves a buffer around the centerline so
  // points near either edge don't get assigned to the wrong side by noise.
  const double q30_l = quantile(l_values, 0.30);
  const double q70_l = quantile(l_values, 0.70);
  std::vector<double> s_left, l_left, h_left;
  std::vector<double> s_right, l_right, h_right;
  s_left.reserve(points.size());  l_left.reserve(points.size());  h_left.reserve(points.size());
  s_right.reserve(points.size()); l_right.reserve(points.size()); h_right.reserve(points.size());
  for (const auto & p : points) {
    if (p.l >= q70_l) {
      s_left.push_back(p.s); l_left.push_back(p.l); h_left.push_back(p.h);
    } else if (p.l <= q30_l) {
      s_right.push_back(p.s); l_right.push_back(p.l); h_right.push_back(p.h);
    }
  }

  const bool enough_side_points =
    static_cast<int>(s_left.size()) >= min_side_points_ &&
    static_cast<int>(s_right.size()) >= min_side_points_;

  LineFit side_fit_l, side_fit_r;
  if (enough_side_points) {
    side_fit_l = robustLineFit(s_left, l_left);
    side_fit_r = robustLineFit(s_right, l_right);
  }
  m.side_lines_valid = enough_side_points && side_fit_l.valid && side_fit_r.valid;

  if (m.side_lines_valid) {
    // Inverse-RMS-weighted average slope — no pi-ambiguity, unlike PCA.
    const double w_l = 1.0 / std::max(side_fit_l.rms, 1e-4);
    const double w_r = 1.0 / std::max(side_fit_r.rms, 1e-4);
    const double side_yaw_corr = std::atan((w_l * side_fit_l.a + w_r * side_fit_r.a) / (w_l + w_r));
    // Width at the ROI centre (s=0): perpendicular distance between the two
    // fitted lines' intercepts — a continuous, line-fit-based width estimate,
    // more stable than a single left/right median.
    m.width_obs = std::abs(side_fit_l.b - side_fit_r.b);

    const bool enough_span = m.span_s >= min_span_s_;
    const bool width_ok = std::abs(m.width_obs - trailer_width_) <= width_tolerance_;
    const bool yaw_small = std::abs(side_yaw_corr) <= max_yaw_correction_;
    if (enough_span && width_ok && yaw_small) {
      // Overrides the PCA-based yaw_corr_raw/yaw_valid/yaw_corr_used above —
      // this is the preferred, unambiguous estimate.
      m.yaw_corr_raw = side_yaw_corr;
      m.yaw_valid = true;
      m.yaw_corr_used = clamp(side_yaw_corr, -max_yaw_correction_, max_yaw_correction_);
    }
  }

  // Roll: reuses the same left/right split and per-side line fits above —
  // height at the ROI centre (s=0) is each line's intercept in h, not a crude
  // per-side median, so it stays consistent with whatever yaw was just used.
  if (enable_roll_estimation_ && enough_side_points) {
    const LineFit h_fit_l = robustLineFit(s_left, h_left);
    const LineFit h_fit_r = robustLineFit(s_right, h_right);
    if (h_fit_l.valid && h_fit_r.valid && m.width_obs > 1e-3) {
      m.roll_raw = std::atan2(h_fit_l.b - h_fit_r.b, m.width_obs);
      const bool width_ok = std::abs(m.width_obs - trailer_width_) <= width_tolerance_;
      const bool roll_ok = std::abs(m.roll_raw) <= max_roll_;
      m.roll_valid = width_ok && roll_ok;
      if (m.roll_valid) {
        m.roll_used = clamp(m.roll_raw, -max_roll_, max_roll_);
      }
    }
  }

  const double score_n = clamp(
    safeRatio(static_cast<double>(points.size() - min_points_after_ground_), 120.0, 0.0), 0.0, 1.0);
  const double score_span = clamp(safeRatio(m.span_s, min_span_s_, 0.0), 0.0, 1.0);
  const double score_pca = clamp(safeRatio(m.pca_ratio - 1.0, min_centerline_pca_ratio_ - 1.0, 0.0), 0.0, 1.0);
  const double score_yaw = m.yaw_valid ? 1.0 : 0.25;
  m.confidence = clamp(score_n * score_span * score_pca * score_yaw, 0.0, 1.0);
  m.measurement_valid = m.yaw_valid || m.pitch_valid || m.roll_valid;
  return m;
}

TrailerPoseNode::LineFit TrailerPoseNode::robustLineFit(
  const std::vector<double> & x,
  const std::vector<double> & y) const
{
  LineFit out;
  if (x.size() != y.size() || x.size() < 3) {
    return out;
  }

  auto fit_subset = [&](const std::vector<std::size_t> & ids, LineFit & fit) {
    double sx = 0.0;
    double sy = 0.0;
    double sxx = 0.0;
    double sxy = 0.0;
    for (const auto id : ids) {
      sx += x[id];
      sy += y[id];
      sxx += x[id] * x[id];
      sxy += x[id] * y[id];
    }

    const double n = static_cast<double>(ids.size());
    const double det = n * sxx - sx * sx;
    if (std::abs(det) < 1e-9) {
      return;
    }

    fit.a = (n * sxy - sx * sy) / det;
    fit.b = (sy - fit.a * sx) / n;
    double rss = 0.0;
    for (const auto id : ids) {
      const double r = y[id] - (fit.a * x[id] + fit.b);
      rss += r * r;
    }
    fit.rms = std::sqrt(rss / n);
    fit.n = ids.size();
    fit.valid = true;
  };

  std::vector<std::size_t> ids(x.size());
  std::iota(ids.begin(), ids.end(), 0);
  fit_subset(ids, out);
  if (!out.valid || ids.size() < 8) {
    return out;
  }

  // One robust trimming pass: remove the 20% largest residuals, then refit.
  std::vector<double> residuals;
  residuals.reserve(ids.size());
  for (const auto id : ids) {
    residuals.push_back(std::abs(y[id] - (out.a * x[id] + out.b)));
  }
  const double cutoff = quantile(residuals, 0.80);

  std::vector<std::size_t> keep;
  keep.reserve(ids.size());
  for (const auto id : ids) {
    const double r = std::abs(y[id] - (out.a * x[id] + out.b));
    if (r <= cutoff) {
      keep.push_back(id);
    }
  }

  if (keep.size() >= 3) {
    LineFit refit;
    fit_subset(keep, refit);
    if (refit.valid) {
      return refit;
    }
  }
  return out;
}

void TrailerPoseNode::updateFilteredCorrection(const Measurement & meas)
{
  if (!filter_initialized_) {
    yaw_corr_f_ = meas.yaw_valid ? meas.yaw_corr_used : 0.0;
    pitch_f_ = meas.pitch_valid ? meas.pitch_used : 0.0;
    roll_f_ = meas.roll_valid ? meas.roll_used : 0.0;
    corr_s_f_ = meas.yaw_valid ? meas.corr_s : 0.0;
    corr_l_f_ = meas.yaw_valid ? meas.corr_l : 0.0;
    corr_h_f_ = meas.yaw_valid ? meas.corr_h : 0.0;
    filter_initialized_ = true;
    return;
  }

  bool yaw_ok = meas.yaw_valid;
  if (yaw_ok && std::abs(normalizeAngle(meas.yaw_corr_used - yaw_corr_f_)) > max_yaw_jump_) {
    yaw_ok = false;
  }

  if (yaw_ok) {
    const double e = normalizeAngle(meas.yaw_corr_used - yaw_corr_f_);
    yaw_corr_f_ = normalizeAngle(yaw_corr_f_ + alpha_yaw_ * e);
  } else {
    yaw_corr_f_ *= yaw_decay_to_prior_;
  }

  if (meas.pitch_valid) {
    pitch_f_ = (1.0 - alpha_pitch_) * pitch_f_ + alpha_pitch_ * meas.pitch_used;
  } else {
    pitch_f_ *= pitch_decay_to_zero_;
  }

  if (enable_roll_estimation_ && meas.roll_valid) {
    roll_f_ = (1.0 - alpha_roll_) * roll_f_ + alpha_roll_ * meas.roll_used;
  } else {
    roll_f_ *= roll_decay_to_zero_;
  }

  if (meas.yaw_valid) {
    corr_s_f_ = (1.0 - alpha_position_) * corr_s_f_ + alpha_position_ * meas.corr_s;
    corr_l_f_ = (1.0 - alpha_position_) * corr_l_f_ + alpha_position_ * meas.corr_l;
    corr_h_f_ = (1.0 - alpha_position_) * corr_h_f_ + alpha_position_ * meas.corr_h;
  } else {
    corr_s_f_ *= position_decay_to_prior_;
    corr_l_f_ *= position_decay_to_prior_;
    corr_h_f_ *= position_decay_to_prior_;
  }
}

Eigen::Vector3d TrailerPoseNode::filteredPositionBase(const PriorFrame & prior) const
{
  return prior.p0_base +
    corr_s_f_ * prior.u_base +
    corr_l_f_ * prior.n_base +
    corr_h_f_ * prior.k_base;
}

Eigen::Vector3d TrailerPoseNode::directionFromYawPitch(const double yaw, const double pitch) const
{
  const Eigen::Vector3d u_yaw(std::cos(yaw), std::sin(yaw), 0.0);
  return (std::cos(pitch) * u_yaw + std::sin(pitch) * Eigen::Vector3d::UnitZ()).normalized();
}

Eigen::Matrix3d TrailerPoseNode::makeRotation(const double yaw, const double pitch, const double roll) const
{
  const Eigen::Vector3d u = directionFromYawPitch(yaw, pitch);
  Eigen::Vector3d n(-std::sin(yaw), std::cos(yaw), 0.0);
  n.normalize();
  Eigen::Vector3d k = u.cross(n).normalized();
  n = k.cross(u).normalized();

  if (std::abs(roll) > 1e-9) {
    const Eigen::Vector3d n0 = n;
    const Eigen::Vector3d k0 = k;
    n = std::cos(roll) * n0 + std::sin(roll) * k0;
    k = -std::sin(roll) * n0 + std::cos(roll) * k0;
  }

  Eigen::Matrix3d R;
  R.col(0) = u;
  R.col(1) = n.normalized();
  R.col(2) = k.normalized();
  return R;
}

geometry_msgs::msg::PoseStamped TrailerPoseNode::makePose(
  const std_msgs::msg::Header & header,
  const Eigen::Vector3d & position_base,
  const double yaw,
  const double pitch,
  const double roll) const
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = header.stamp;
  pose.header.frame_id = "base_link";
  pose.pose.position.x = position_base.x();
  pose.pose.position.y = position_base.y();
  pose.pose.position.z = position_base.z();

  Eigen::Quaterniond q(makeRotation(yaw, pitch, roll));
  q.normalize();
  pose.pose.orientation.x = q.x();
  pose.pose.orientation.y = q.y();
  pose.pose.orientation.z = q.z();
  pose.pose.orientation.w = q.w();
  return pose;
}

void TrailerPoseNode::publishTrailerRoiCloud(
  const std_msgs::msg::Header & header,
  const std::vector<LocalPoint> & points) const
{
  if (!publish_trailer_roi_cloud_) {
    return;
  }

  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = header;
  cloud.height = 1;
  cloud.width = static_cast<uint32_t>(points.size());
  cloud.is_bigendian = false;
  cloud.is_dense = false;

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(points.size());

  sensor_msgs::PointCloud2Iterator<float> it_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> it_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> it_z(cloud, "z");
  for (const auto & p : points) {
    *it_x = static_cast<float>(p.p_cloud.x());
    *it_y = static_cast<float>(p.p_cloud.y());
    *it_z = static_cast<float>(p.p_cloud.z());
    ++it_x;
    ++it_y;
    ++it_z;
  }
  trailer_roi_cloud_pub_->publish(cloud);
}

void TrailerPoseNode::cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  if (!updateTfCache(msg->header.frame_id)) {
    return;
  }

  const double theta = activeTheta();
  const PriorFrame prior = buildPriorFrame(theta);

  Measurement meas;
  std::vector<LocalPoint> roi = cropToOrientedRoi(*msg, prior);
  meas.n_roi_raw = roi.size();

  std::vector<LocalPoint> used = removeGroundByQuantile(roi, meas);
  used = voxelDownsampleLocal(used);

  Measurement geom = estimateMeasurement(used);
  geom.n_roi_raw = meas.n_roi_raw;
  geom.n_after_ground = meas.n_after_ground;
  geom.n_after_voxel = used.size();
  geom.ground_removed = meas.ground_removed;
  geom.ground_threshold = meas.ground_threshold;

  updateFilteredCorrection(geom);
  publishAll(msg->header, prior, geom, used);

  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 1000,
    "trailer pose: theta=%.1f deg yaw_corr=%.1f/%.1f deg conf=%.2f N=%zu/%zu span=%.2f pca=%.2f pitch=%.1f deg%s",
    theta * 180.0 / kPi,
    geom.yaw_corr_raw * 180.0 / kPi,
    yaw_corr_f_ * 180.0 / kPi,
    geom.confidence,
    geom.n_after_voxel,
    geom.n_roi_raw,
    geom.span_s,
    geom.pca_ratio,
    pitch_f_ * 180.0 / kPi,
    geom.pitch_valid ? "" : "?" );
}

void TrailerPoseNode::publishAll(
  const std_msgs::msg::Header & header,
  const PriorFrame & prior,
  const Measurement & meas,
  const std::vector<LocalPoint> & used_points)
{
  const double yaw_prior = prior.yaw_prior;
  const double yaw_raw = normalizeAngle(yaw_prior + meas.yaw_corr_raw);
  const double yaw_used = normalizeAngle(yaw_prior + yaw_corr_f_);

  const Eigen::Vector3d prior_pos = prior.p0_base;
  const Eigen::Vector3d raw_pos = prior.p0_base +
    meas.corr_s * prior.u_base + meas.corr_l * prior.n_base + meas.corr_h * prior.k_base;
  const Eigen::Vector3d pos = filteredPositionBase(prior);

  pose_prior_pub_->publish(makePose(header, prior_pos, yaw_prior, 0.0, 0.0));
  pose_raw_pub_->publish(makePose(
    header, raw_pos, yaw_raw,
    meas.pitch_valid ? meas.pitch_used : 0.0,
    meas.roll_valid ? meas.roll_used : 0.0));
  pose_pub_->publish(makePose(header, pos, yaw_used, pitch_f_, roll_f_));

  publishTrailerRoiCloud(header, used_points);
  publishDebugScalars(prior, meas);
  publishMarkers(header, prior, meas, pos, yaw_used, pitch_f_, roll_f_);
}

void TrailerPoseNode::publishMarkers(
  const std_msgs::msg::Header & header,
  const PriorFrame & prior,
  const Measurement & meas,
  const Eigen::Vector3d & pos_base,
  const double yaw,
  const double pitch,
  const double roll) const
{
  visualization_msgs::msg::MarkerArray ma;

  auto baseMarker = [&](const int id, const std::string & ns, const int type) {
    visualization_msgs::msg::Marker m;
    m.header.stamp = header.stamp;
    m.header.frame_id = "base_link";
    m.ns = ns;
    m.id = id;
    m.type = type;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.lifetime = markerLifetime(0.30);
    return m;
  };

  const Eigen::Matrix3d R_prior = makeRotation(prior.yaw_prior, 0.0, 0.0);
  const Eigen::Quaterniond q_prior(R_prior);

  // ROI cube: purely kinematic, oriented by the articulation prior.
  {
    auto cube = baseMarker(0, "trailer_roi", visualization_msgs::msg::Marker::CUBE);
    cube.pose.position.x = prior.p0_base.x();
    cube.pose.position.y = prior.p0_base.y();
    cube.pose.position.z = prior.p0_base.z();
    cube.pose.orientation.x = q_prior.x();
    cube.pose.orientation.y = q_prior.y();
    cube.pose.orientation.z = q_prior.z();
    cube.pose.orientation.w = q_prior.w();
    cube.scale.x = 2.0 * roi_half_x_;
    cube.scale.y = 2.0 * roi_half_y_;
    cube.scale.z = 2.0 * roi_half_z_ * roi_marker_vertical_scale_factor_;
    cube.color.r = 0.0f;
    cube.color.g = 0.8f;
    cube.color.b = 1.0f;
    cube.color.a = 0.16f;
    ma.markers.push_back(cube);
  }

  auto makeLine = [&](
    const int id, const std::string & ns,
    const Eigen::Vector3d & a, const Eigen::Vector3d & b,
    const float r, const float g, const float bl, const double width) {
    auto m = baseMarker(id, ns, visualization_msgs::msg::Marker::LINE_STRIP);
    m.scale.x = width;
    m.color.r = r;
    m.color.g = g;
    m.color.b = bl;
    m.color.a = 1.0f;
    m.points.push_back(pointMsg(a));
    m.points.push_back(pointMsg(b));
    ma.markers.push_back(m);
  };

  const Eigen::Vector3d u_prior = prior.u_base;
  const Eigen::Vector3d n_prior = prior.n_base;
  const Eigen::Vector3d u = directionFromYawPitch(yaw, pitch);
  Eigen::Vector3d n(-std::sin(yaw), std::cos(yaw), 0.0);
  n.normalize();
  if (std::abs(roll) > 1e-9) {
    const Eigen::Vector3d k = u.cross(n).normalized();
    n = (std::cos(roll) * n + std::sin(roll) * k).normalized();
  }

  // Draw from hitch to rear, not centre ± length/2. This is much more intuitive.
  const Eigen::Vector3d prior_front = prior.hitch_base + trailer_front_offset_ * u_prior;
  const Eigen::Vector3d prior_rear = prior.hitch_base + trailer_rear_offset_ * u_prior;
  const Eigen::Vector3d front = prior.hitch_base + trailer_front_offset_ * u;
  const Eigen::Vector3d rear = prior.hitch_base + trailer_rear_offset_ * u;

  makeLine(1, "trailer_skeleton", prior_front, prior_rear, 0.2f, 0.4f, 1.0f, 0.018); // prior blue
  makeLine(2, "trailer_skeleton", front, rear, 1.0f, 0.8f, 0.0f, 0.035);             // final yellow

  const double hw = 0.5 * trailer_width_;
  makeLine(3, "trailer_skeleton", front + hw * n, rear + hw * n, 0.0f, 1.0f, 0.2f, 0.026);
  makeLine(4, "trailer_skeleton", front - hw * n, rear - hw * n, 1.0f, 0.0f, 1.0f, 0.026);

  // Pose arrow at the filtered centre.
  {
    auto arrow = baseMarker(5, "trailer_pose", visualization_msgs::msg::Marker::ARROW);
    arrow.points.push_back(pointMsg(pos_base));
    arrow.points.push_back(pointMsg(pos_base + 0.60 * u));
    arrow.scale.x = 0.04;
    arrow.scale.y = 0.08;
    arrow.scale.z = 0.08;
    arrow.color.r = 1.0f;
    arrow.color.g = 0.45f;
    arrow.color.b = 0.0f;
    arrow.color.a = 1.0f;
    ma.markers.push_back(arrow);
  }

  // Text debug.
  {
    auto text = baseMarker(6, "trailer_debug", visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
    text.pose.position.x = pos_base.x();
    text.pose.position.y = pos_base.y();
    text.pose.position.z = pos_base.z() + 0.55;
    text.scale.z = 0.12;
    text.color.r = 1.0f;
    text.color.g = 1.0f;
    text.color.b = 1.0f;
    text.color.a = 1.0f;
    std::ostringstream ss;
    ss.setf(std::ios::fixed, std::ios::floatfield);
    ss.precision(2);
    ss << "N=" << meas.n_after_voxel
       << " conf=" << meas.confidence
       << " pca=" << meas.pca_ratio
       << " span=" << meas.span_s
       << " yaw[" << (meas.side_lines_valid ? "L" : "P") << "]="
       << yaw_corr_f_ * 180.0 / kPi << "deg"
       << " width=" << meas.width_obs
       << " pitch=" << pitch_f_ * 180.0 / kPi << "deg";
    text.text = ss.str();
    ma.markers.push_back(text);
  }

  markers_pub_->publish(ma);
}

void TrailerPoseNode::publishDebugScalars(const PriorFrame & prior, const Measurement & meas) const
{
  // Guard: only serialize and publish when at least one subscriber is active.
  // At 10-20 Hz with 18 topics this saves ~200 unnecessary publish calls/s.
  if (confidence_pub_->get_subscription_count() == 0 &&
      yaw_prior_pub_->get_subscription_count() == 0 &&
      roi_count_pub_->get_subscription_count() == 0) {
    return;
  }

  const double yaw_used = normalizeAngle(prior.yaw_prior + yaw_corr_f_);
  const double yaw_raw = normalizeAngle(prior.yaw_prior + meas.yaw_corr_raw);

  confidence_pub_->publish(f64(meas.confidence));
  yaw_prior_pub_->publish(f64(prior.yaw_prior));
  yaw_raw_pub_->publish(f64(yaw_raw));
  yaw_used_pub_->publish(f64(yaw_used));
  yaw_corr_raw_pub_->publish(f64(meas.yaw_corr_raw));
  yaw_corr_used_pub_->publish(f64(yaw_corr_f_));
  pitch_raw_pub_->publish(f64(meas.pitch_raw));
  pitch_used_pub_->publish(f64(pitch_f_));
  roll_used_pub_->publish(f64(roll_f_));
  roi_count_pub_->publish(f64(static_cast<double>(meas.n_roi_raw)));
  roi_count_after_ground_pub_->publish(f64(static_cast<double>(meas.n_after_ground)));
  ground_threshold_pub_->publish(f64(meas.ground_threshold));
  ground_removed_pub_->publish(f64(static_cast<double>(meas.ground_removed)));
  span_s_pub_->publish(f64(meas.span_s));
  pca_ratio_pub_->publish(f64(meas.pca_ratio));
  measurement_valid_pub_->publish(f64(meas.measurement_valid ? 1.0 : 0.0));
  pitch_valid_pub_->publish(f64(meas.pitch_valid ? 1.0 : 0.0));
  roll_valid_pub_->publish(f64(meas.roll_valid ? 1.0 : 0.0));
}

double TrailerPoseNode::normalizeAngle(double a) noexcept
{
  return std::atan2(std::sin(a), std::cos(a));
}

double TrailerPoseNode::clamp(const double x, const double lo, const double hi) noexcept
{
  return std::max(lo, std::min(hi, x));
}

double TrailerPoseNode::safeRatio(
  const double numerator, const double denominator, const double fallback) noexcept
{
  return std::abs(denominator) > 1e-12 ? numerator / denominator : fallback;
}

double TrailerPoseNode::quantile(std::vector<double> values, const double q)
{
  if (values.empty()) {
    return 0.0;
  }
  const double qq = clamp(q, 0.0, 1.0);
  const std::size_t idx = static_cast<std::size_t>(
    std::round(qq * static_cast<double>(values.size() - 1)));
  std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(idx), values.end());
  return values[idx];
}

std_msgs::msg::Float64 TrailerPoseNode::f64(const double value)
{
  std_msgs::msg::Float64 msg;
  msg.data = value;
  return msg;
}

}  // namespace mtt_perception

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mtt_perception::TrailerPoseNode>());
  rclcpp::shutdown();
  return 0;
}

