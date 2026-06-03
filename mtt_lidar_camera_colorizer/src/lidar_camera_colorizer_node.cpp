// lidar_camera_colorizer_node.cpp — LiDAR-Camera colorization for MTT-HL
//
// Subscribes to:
//   Hesai XT-32   /hesai_lidar/points                    ~20 Hz  PointCloud2
//   ZED2i RGB     /zed/zed_node/rgb/color/rect/image     ~10 Hz  Image (via image_transport)
//   ZED2i depth   /zed/zed_node/depth/depth_registered   ~10 Hz  Image (optional)
//
// Publishes:
//   /hesai_lidar/points_colored   PointCloud2 with all original fields + rgb
//
// Architecture: latest-image-cache.
//   The LiDAR fires at 20 Hz, the camera at 10 Hz.  ApproximateTimeSynchronizer
//   would drop every other LiDAR frame (each camera frame matched at most once).
//   Instead, the image callback stores the latest decoded BGR frame under a mutex;
//   the cloud callback grabs a shallow copy of that frame and proceeds immediately.
//
// TF: the hesai_lidar → zed_left_camera_frame_optical transform is purely static
//   (both sensors are rigidly bolted to the same pod).  It is looked up once on the
//   first cloud callback and reused for every subsequent frame.
//
// RGB packing (Foxglove / PCL convention):
//   uint32_t packed = (R << 16) | (G << 8) | B;
//   float    rgb_f;
//   memcpy(&rgb_f, &packed, 4);
//   The field is named "rgb", typed FLOAT32.  Foxglove recognises this encoding
//   natively and lets the user switch between "rgb" and "intensity" color modes.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <image_transport/image_transport.hpp>
#include <image_transport/camera_subscriber.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <image_geometry/pinhole_camera_model.hpp>

#include <Eigen/Geometry>

#include <cmath>
#include <cstring>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointField  = sensor_msgs::msg::PointField;
using Image       = sensor_msgs::msg::Image;
using CameraInfo  = sensor_msgs::msg::CameraInfo;

// ── helpers ───────────────────────────────────────────────────────────────────

static bool validateXYZFields(const PointCloud2 & cloud,
                               uint32_t & off_x,
                               uint32_t & off_y,
                               uint32_t & off_z)
{
  constexpr uint32_t NONE = std::numeric_limits<uint32_t>::max();
  off_x = off_y = off_z = NONE;
  for (const auto & f : cloud.fields) {
    if (f.datatype != PointField::FLOAT32 || f.count < 1) continue;
    if (f.offset + 4u > cloud.point_step)                 continue;
    if      (f.name == "x") off_x = f.offset;
    else if (f.name == "y") off_y = f.offset;
    else if (f.name == "z") off_z = f.offset;
  }
  return (off_x != NONE && off_y != NONE && off_z != NONE);
}

static inline float packRGB(uint8_t r, uint8_t g, uint8_t b)
{
  uint32_t packed = (static_cast<uint32_t>(r) << 16) |
                    (static_cast<uint32_t>(g) <<  8) |
                     static_cast<uint32_t>(b);
  float f;
  std::memcpy(&f, &packed, 4);
  return f;
}

// ── node ─────────────────────────────────────────────────────────────────────

class LidarCameraColorizerNode : public rclcpp::Node
{
public:
  explicit LidarCameraColorizerNode(const rclcpp::NodeOptions & opts)
  : Node("lidar_camera_colorizer_node", opts),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    // ── parameters ──────────────────────────────────────────────────────────
    declare_parameter("cloud_topic",          "/hesai_lidar/points");
    declare_parameter("image_base_topic",     "/zed/zed_node/rgb/color/rect/image");
    declare_parameter("depth_base_topic",     "/zed/zed_node/depth/depth_registered");
    declare_parameter("camera_optical_frame", "zed_left_camera_frame_optical");
    declare_parameter("output_frame",         "");
    declare_parameter("min_range",            0.5);
    declare_parameter("max_range",            50.0);
    declare_parameter("image_timeout",        0.5);
    declare_parameter("tf_timeout",           5.0);
    declare_parameter("queue_size",           2);
    declare_parameter("use_depth_check",      false);
    declare_parameter("depth_tolerance",      0.5);
    declare_parameter("image_transport",      std::string("compressed"));
    declare_parameter("default_color_r",      128);
    declare_parameter("default_color_g",      128);
    declare_parameter("default_color_b",      128);

    cloud_topic_         = get_parameter("cloud_topic").as_string();
    image_base_topic_    = get_parameter("image_base_topic").as_string();
    depth_base_topic_    = get_parameter("depth_base_topic").as_string();
    camera_optical_frame_= get_parameter("camera_optical_frame").as_string();
    output_frame_        = get_parameter("output_frame").as_string();
    min_range_sq_        = std::pow(get_parameter("min_range").as_double(), 2.0);
    max_range_sq_        = std::pow(get_parameter("max_range").as_double(), 2.0);
    image_timeout_       = get_parameter("image_timeout").as_double();
    tf_timeout_          = get_parameter("tf_timeout").as_double();
    queue_size_          = static_cast<uint32_t>(get_parameter("queue_size").as_int());
    use_depth_check_     = get_parameter("use_depth_check").as_bool();
    depth_tolerance_     = get_parameter("depth_tolerance").as_double();
    transport_hint_      = get_parameter("image_transport").as_string();
    default_rgb_float_   = packRGB(
      static_cast<uint8_t>(get_parameter("default_color_r").as_int()),
      static_cast<uint8_t>(get_parameter("default_color_g").as_int()),
      static_cast<uint8_t>(get_parameter("default_color_b").as_int()));

    // ── publisher ────────────────────────────────────────────────────────────
    const std::string out_topic = cloud_topic_ + "_colored";
    pub_colored_ = create_publisher<PointCloud2>(
      out_topic, rclcpp::SensorDataQoS().keep_last(queue_size_));

    // ── cloud subscriber ────────────────────────────────────────────────────
    sub_cloud_ = create_subscription<PointCloud2>(
      cloud_topic_,
      rclcpp::SensorDataQoS().keep_last(queue_size_),
      std::bind(&LidarCameraColorizerNode::cloudCallback, this, std::placeholders::_1));

    // ── camera subscriber (image_transport handles compressed/raw) ────────
    image_transport::TransportHints hints(this, transport_hint_);
    sub_camera_ = image_transport::create_camera_subscription(
      this,
      image_base_topic_,
      std::bind(&LidarCameraColorizerNode::imageCallback, this,
                std::placeholders::_1, std::placeholders::_2),
      transport_hint_,
      rmw_qos_profile_sensor_data);

    // ── depth subscriber (optional) ─────────────────────────────────────
    if (use_depth_check_) {
      sub_depth_ = image_transport::create_subscription(
        this,
        depth_base_topic_,
        std::bind(&LidarCameraColorizerNode::depthCallback, this, std::placeholders::_1),
        transport_hint_,
        rmw_qos_profile_sensor_data);
    }

    RCLCPP_INFO(get_logger(),
      "LidarCameraColorizer ready\n"
      "  cloud:   %s\n"
      "  image:   %s [%s]\n"
      "  output:  %s\n"
      "  depth check: %s",
      cloud_topic_.c_str(),
      image_base_topic_.c_str(), transport_hint_.c_str(),
      out_topic.c_str(),
      use_depth_check_ ? "on" : "off");
  }

private:
  // ── image callback ───────────────────────────────────────────────────────
  void imageCallback(const Image::ConstSharedPtr & img_msg,
                     const CameraInfo::ConstSharedPtr & info_msg)
  {
    cv_bridge::CvImageConstPtr cv_img;
    try {
      cv_img = cv_bridge::toCvShare(img_msg, "bgr8");
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "cv_bridge exception: %s", e.what());
      return;
    }

    std::lock_guard<std::mutex> lock(image_mtx_);
    cached_image_       = cv_img->image.clone();
    cached_image_stamp_ = rclcpp::Time(img_msg->header.stamp);
    if (!cam_model_ready_) {
      cam_model_.fromCameraInfo(info_msg);
      cam_model_ready_ = true;
    }
  }

  // ── depth callback ───────────────────────────────────────────────────────
  void depthCallback(const Image::ConstSharedPtr & depth_msg)
  {
    cv_bridge::CvImageConstPtr cv_depth;
    try {
      cv_depth = cv_bridge::toCvShare(depth_msg, "32FC1");
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "depth cv_bridge exception: %s", e.what());
      return;
    }

    std::lock_guard<std::mutex> lock(depth_mtx_);
    cached_depth_       = cv_depth->image.clone();
    cached_depth_stamp_ = rclcpp::Time(depth_msg->header.stamp);
  }

  // ── TF lookup (called once, cached for all subsequent frames) ───────────
  bool cacheTransform(const rclcpp::Time & stamp)
  {
    const std::string & lidar_frame = lidar_frame_.empty()
      ? last_cloud_frame_
      : lidar_frame_;

    geometry_msgs::msg::TransformStamped tf_msg;
    try {
      // Use time(0) (latest available) for static transforms to avoid needing
      // a perfectly matching timestamp during startup or bag replay.
      tf_msg = tf_buffer_.lookupTransform(
        camera_optical_frame_,
        lidar_frame,
        tf2::TimePointZero,
        tf2::durationFromSec(tf_timeout_));
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
        "TF lookup %s → %s failed: %s",
        lidar_frame.c_str(), camera_optical_frame_.c_str(), e.what());
      return false;
    }

    T_cam_lidar_     = tf2::transformToEigen(tf_msg);
    transform_cached_= true;

    RCLCPP_INFO(get_logger(),
      "TF cached: %s → %s\n  t=[%.4f, %.4f, %.4f]",
      lidar_frame.c_str(), camera_optical_frame_.c_str(),
      T_cam_lidar_.translation().x(),
      T_cam_lidar_.translation().y(),
      T_cam_lidar_.translation().z());
    return true;
  }

  // ── cloud callback (main processing pipeline) ────────────────────────────
  void cloudCallback(const PointCloud2::ConstSharedPtr & cloud_in)
  {
    last_cloud_frame_ = cloud_in->header.frame_id;

    // ── grab image snapshot under lock ──────────────────────────────────
    cv::Mat image_snapshot;
    image_geometry::PinholeCameraModel cam_snapshot;
    bool have_image = false;
    {
      std::lock_guard<std::mutex> lock(image_mtx_);
      if (cam_model_ready_ && !cached_image_.empty()) {
        const double dt = std::abs(
          (rclcpp::Time(cloud_in->header.stamp) - cached_image_stamp_).seconds());
        if (dt <= image_timeout_) {
          image_snapshot = cached_image_;   // cv::Mat is ref-counted; cheap
          cam_snapshot   = cam_model_;
          have_image     = true;
        }
      }
    }

    // ── lazy TF caching ─────────────────────────────────────────────────
    if (have_image && !transform_cached_) {
      if (!cacheTransform(rclcpp::Time(cloud_in->header.stamp))) {
        have_image = false;  // can't colorize without transform
      }
    }

    // ── validate input fields ────────────────────────────────────────────
    uint32_t off_x, off_y, off_z;
    if (!validateXYZFields(*cloud_in, off_x, off_y, off_z)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
        "Input cloud has no valid FLOAT32 x/y/z fields — dropping frame");
      return;
    }

    const uint32_t n_pts       = cloud_in->width * cloud_in->height;
    const uint32_t in_step     = cloud_in->point_step;
    const uint32_t out_step    = in_step + 4u;   // append 4-byte rgb field
    const uint32_t rgb_offset  = in_step;        // rgb sits right after original data

    // ── build output cloud descriptor ───────────────────────────────────
    PointCloud2 cloud_out;
    cloud_out.header     = cloud_in->header;
    if (!output_frame_.empty()) cloud_out.header.frame_id = output_frame_;
    cloud_out.height     = cloud_in->height;
    cloud_out.width      = cloud_in->width;
    cloud_out.is_bigendian = cloud_in->is_bigendian;
    cloud_out.is_dense   = cloud_in->is_dense;
    cloud_out.point_step = out_step;
    cloud_out.row_step   = out_step * cloud_out.width;

    // Copy original field descriptors
    cloud_out.fields = cloud_in->fields;
    // Append rgb field
    {
      PointField rgb_field;
      rgb_field.name     = "rgb";
      rgb_field.offset   = rgb_offset;
      rgb_field.datatype = PointField::FLOAT32;
      rgb_field.count    = 1;
      cloud_out.fields.push_back(rgb_field);
    }

    cloud_out.data.resize(static_cast<size_t>(out_step) * n_pts);

    // ── pre-compute camera intrinsics ────────────────────────────────────
    double fx = 0.0, fy = 0.0, cx = 0.0, cy = 0.0;
    int img_w = 0, img_h = 0;
    if (have_image) {
      fx    = cam_snapshot.fx();
      fy    = cam_snapshot.fy();
      cx    = cam_snapshot.cx();
      cy    = cam_snapshot.cy();
      img_w = static_cast<int>(image_snapshot.cols);
      img_h = static_cast<int>(image_snapshot.rows);
    }

    // ── grab depth snapshot (optional) ──────────────────────────────────
    cv::Mat depth_snapshot;
    if (have_image && use_depth_check_) {
      std::lock_guard<std::mutex> lock(depth_mtx_);
      if (!cached_depth_.empty()) {
        depth_snapshot = cached_depth_;
      }
    }

    // ── per-point loop ───────────────────────────────────────────────────
    const uint8_t * src = cloud_in->data.data();
    uint8_t       * dst = cloud_out.data.data();

    uint32_t n_colored    = 0;
    uint32_t n_out_of_fov = 0;

    for (uint32_t i = 0; i < n_pts; ++i, src += in_step, dst += out_step) {
      // Copy original point bytes verbatim
      std::memcpy(dst, src, in_step);

      if (!have_image) {
        std::memcpy(dst + rgb_offset, &default_rgb_float_, 4);
        continue;
      }

      // Read xyz
      float x, y, z;
      std::memcpy(&x, src + off_x, 4);
      std::memcpy(&y, src + off_y, 4);
      std::memcpy(&z, src + off_z, 4);

      // Validity + range check
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        std::memcpy(dst + rgb_offset, &default_rgb_float_, 4);
        continue;
      }
      const float r2 = x * x + y * y + z * z;
      if (r2 < static_cast<float>(min_range_sq_) ||
          r2 > static_cast<float>(max_range_sq_)) {
        std::memcpy(dst + rgb_offset, &default_rgb_float_, 4);
        continue;
      }

      // Transform to camera optical frame
      // P_cam = T_cam_lidar * P_lidar
      const Eigen::Vector3d p_cam =
        T_cam_lidar_ * Eigen::Vector3d(static_cast<double>(x),
                                       static_cast<double>(y),
                                       static_cast<double>(z));

      // Behind camera
      if (p_cam.z() <= 0.0) {
        std::memcpy(dst + rgb_offset, &default_rgb_float_, 4);
        continue;
      }

      // Project: u = fx * X/Z + cx,  v = fy * Y/Z + cy
      const double inv_z = 1.0 / p_cam.z();
      const int    u     = static_cast<int>(fx * p_cam.x() * inv_z + cx + 0.5);
      const int    v     = static_cast<int>(fy * p_cam.y() * inv_z + cy + 0.5);

      if (u < 0 || u >= img_w || v < 0 || v >= img_h) {
        std::memcpy(dst + rgb_offset, &default_rgb_float_, 4);
        ++n_out_of_fov;
        continue;
      }

      // Optional depth occlusion check
      if (use_depth_check_ && !depth_snapshot.empty()) {
        const float d = depth_snapshot.at<float>(v, u);
        if (std::isfinite(d) && p_cam.z() > static_cast<double>(d) + depth_tolerance_) {
          std::memcpy(dst + rgb_offset, &default_rgb_float_, 4);
          continue;
        }
      }

      // Read BGR from cv::Mat (OpenCV default), pack as 0x00RRGGBB
      const cv::Vec3b & bgr = image_snapshot.at<cv::Vec3b>(v, u);
      const float rgb_f = packRGB(bgr[2], bgr[1], bgr[0]);
      std::memcpy(dst + rgb_offset, &rgb_f, 4);
      ++n_colored;
    }

    pub_colored_->publish(cloud_out);

    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 5000,
      "colorized %u / %u pts  (out-of-FOV: %u)",
      n_colored, n_pts, n_out_of_fov);

    if (!have_image) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "No valid image available (timeout=%.2fs) — publishing cloud with default colors",
        image_timeout_);
    }
  }

  // ── parameters ────────────────────────────────────────────────────────────
  std::string cloud_topic_;
  std::string image_base_topic_;
  std::string depth_base_topic_;
  std::string camera_optical_frame_;
  std::string output_frame_;
  std::string lidar_frame_;   // derived from cloud header
  std::string transport_hint_;
  double      min_range_sq_;
  double      max_range_sq_;
  double      image_timeout_;
  double      tf_timeout_;
  double      depth_tolerance_;
  uint32_t    queue_size_;
  bool        use_depth_check_;
  float       default_rgb_float_;

  // ── TF ────────────────────────────────────────────────────────────────────
  tf2_ros::Buffer           tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  Eigen::Isometry3d          T_cam_lidar_{Eigen::Isometry3d::Identity()};
  bool                       transform_cached_ = false;
  std::string                last_cloud_frame_;

  // ── image cache ───────────────────────────────────────────────────────────
  std::mutex                           image_mtx_;
  cv::Mat                              cached_image_;
  rclcpp::Time                         cached_image_stamp_{0, 0, RCL_ROS_TIME};
  image_geometry::PinholeCameraModel   cam_model_;
  bool                                 cam_model_ready_ = false;

  // ── depth cache ───────────────────────────────────────────────────────────
  std::mutex   depth_mtx_;
  cv::Mat      cached_depth_;
  rclcpp::Time cached_depth_stamp_{0, 0, RCL_ROS_TIME};

  // ── subscriptions / publisher ─────────────────────────────────────────────
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_cloud_;
  image_transport::CameraSubscriber            sub_camera_;
  image_transport::Subscriber                  sub_depth_;
  rclcpp::Publisher<PointCloud2>::SharedPtr    pub_colored_;
};

// ── main ─────────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions{}, 2);
  auto node = std::make_shared<LidarCameraColorizerNode>(rclcpp::NodeOptions{});
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
