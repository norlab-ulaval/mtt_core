// cloud_merger_node.cpp — Dual-LiDAR point cloud fusion for MTT-HL
//
// Subscribes to Hesai XT-32 (/hesai_lidar/points) and RS-Airy (/rsairy_ns/points),
// time-synchronizes them, transforms both into base_link frame, concatenates,
// then publishes two topics:
//   /merged_points          — full merged cloud (debug, Foxglove)
//   /merged_points_filtered — self-filtered cloud (mapper input)
//
// Self-filter removes two bounding boxes:
//   1. Robot chassis  (large box, keeps trailer which is further out)
//   2. LiDAR mounting cage (small box around sensor origin)
//
// All bounding box parameters are exposed as ROS parameters for live tuning
// in Foxglove/RViz without recompiling.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
#include <cstring>
#include <string>
#include <vector>
#include <algorithm>

using PointCloud2 = sensor_msgs::msg::PointCloud2;
using ApproxSync = message_filters::sync_policies::ApproximateTime<PointCloud2, PointCloud2>;

// ─── helpers ───────────────────────────────────────────────────────────────

static bool fields_compatible(const PointCloud2 & a, const PointCloud2 & b)
{
  if (a.point_step != b.point_step) return false;
  if (a.fields.size() != b.fields.size()) return false;
  for (size_t i = 0; i < a.fields.size(); ++i) {
    if (a.fields[i].name     != b.fields[i].name)     return false;
    if (a.fields[i].offset   != b.fields[i].offset)   return false;
    if (a.fields[i].datatype != b.fields[i].datatype)  return false;
    if (a.fields[i].count    != b.fields[i].count)    return false;
  }
  return true;
}

// Fast concatenation: both clouds must have identical field layout.
static PointCloud2 fast_concat(const PointCloud2 & a, const PointCloud2 & b,
                               const std::string & frame_id)
{
  PointCloud2 out;
  out.header.stamp    = a.header.stamp;
  out.header.frame_id = frame_id;
  out.height          = 1;
  out.width           = a.width * a.height + b.width * b.height;
  out.fields          = a.fields;
  out.is_bigendian    = a.is_bigendian;
  out.point_step      = a.point_step;
  out.row_step        = out.point_step * out.width;
  out.is_dense        = a.is_dense && b.is_dense;
  out.data.resize(static_cast<size_t>(out.row_step));
  std::memcpy(out.data.data(), a.data.data(), a.data.size());
  std::memcpy(out.data.data() + a.data.size(), b.data.data(), b.data.size());
  return out;
}

// Fallback: convert both to PointXYZI and concatenate.
static PointCloud2 xyzi_concat(const PointCloud2 & a, const PointCloud2 & b,
                               const std::string & frame_id)
{
  pcl::PointCloud<pcl::PointXYZI> pa, pb, merged;
  pcl::fromROSMsg(a, pa);
  pcl::fromROSMsg(b, pb);
  merged = pa;
  merged += pb;

  PointCloud2 out;
  pcl::toROSMsg(merged, out);
  out.header.stamp    = a.header.stamp;
  out.header.frame_id = frame_id;
  return out;
}

// In-place bounding-box self-filter.
// Removes points where ALL three conditions hold:
//   x_min < x < x_max  AND  y_min < y < y_max  AND  z_min < z < z_max
// Works directly on the serialized PointCloud2 buffer — no PCL conversion.
static void bbox_filter_inplace(PointCloud2 & cloud,
                                float x_min, float x_max,
                                float y_min, float y_max,
                                float z_min, float z_max)
{
  // Find offsets for x, y, z fields
  int off_x = -1, off_y = -1, off_z = -1;
  for (const auto & f : cloud.fields) {
    if (f.name == "x") off_x = static_cast<int>(f.offset);
    if (f.name == "y") off_y = static_cast<int>(f.offset);
    if (f.name == "z") off_z = static_cast<int>(f.offset);
  }
  if (off_x < 0 || off_y < 0 || off_z < 0) return;

  const uint32_t step = cloud.point_step;
  const uint32_t n_in = cloud.width * cloud.height;
  std::vector<uint8_t> buf_out;
  buf_out.reserve(cloud.data.size());
  uint32_t n_out = 0;

  const uint8_t * src = cloud.data.data();
  for (uint32_t i = 0; i < n_in; ++i, src += step) {
    float x, y, z;
    std::memcpy(&x, src + off_x, sizeof(float));
    std::memcpy(&y, src + off_y, sizeof(float));
    std::memcpy(&z, src + off_z, sizeof(float));

    bool inside = (x > x_min && x < x_max) &&
                  (y > y_min && y < y_max) &&
                  (z > z_min && z < z_max);
    if (!inside) {
      buf_out.insert(buf_out.end(), src, src + step);
      ++n_out;
    }
  }

  cloud.data     = std::move(buf_out);
  cloud.width    = n_out;
  cloud.height   = 1;
  cloud.row_step = step * n_out;
}

// ─── node ──────────────────────────────────────────────────────────────────

class CloudMergerNode : public rclcpp::Node
{
public:
  CloudMergerNode()
  : Node("cloud_merger_node")
  {
    // ── Parameters ─────────────────────────────────────────────────────────
    target_frame_    = declare_parameter<std::string>("target_frame", "base_link");
    sync_tolerance_  = declare_parameter<double>("sync_tolerance", 0.05);

    // Chassis bounding box (x=forward, y=left, z=up in base_link)
    // These exclude the tractor body while keeping the trailer (further back in -x).
    bbox_chassis_x_ = declare_parameter<std::vector<double>>("bbox_chassis_x", {-0.90, 0.70});
    bbox_chassis_y_ = declare_parameter<std::vector<double>>("bbox_chassis_y", {-0.40, 0.40});
    bbox_chassis_z_ = declare_parameter<std::vector<double>>("bbox_chassis_z", {-0.10, 0.80});

    // LiDAR cage/mount bounding box (tight box around the sensor cluster)
    bbox_cage_x_    = declare_parameter<std::vector<double>>("bbox_cage_x",    {-0.25, 0.15});
    bbox_cage_y_    = declare_parameter<std::vector<double>>("bbox_cage_y",    {-0.20, 0.20});
    bbox_cage_z_    = declare_parameter<std::vector<double>>("bbox_cage_z",    { 0.70, 1.10});

    // ── TF ─────────────────────────────────────────────────────────────────
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // The debug cloud can stay lightweight, but the mapper input must be
    // RELIABLE or the ICP mapper will refuse the connection during replay.
    auto debug_qos = rclcpp::SensorDataQoS();
    auto mapper_qos = rclcpp::QoS(10);
    mapper_qos.reliable();

    pub_full_     = create_publisher<PointCloud2>("merged_points", debug_qos);
    pub_filtered_ = create_publisher<PointCloud2>("merged_points_filtered", mapper_qos);

    // ── Subscriptions via message_filters ──────────────────────────────────
    sub_hesai_.subscribe(this, "/hesai_lidar/points",
      rclcpp::SensorDataQoS().get_rmw_qos_profile());
    sub_rsairy_.subscribe(this, "/rsairy_ns/points",
      rclcpp::SensorDataQoS().get_rmw_qos_profile());

    sync_ = std::make_shared<message_filters::Synchronizer<ApproxSync>>(
      ApproxSync(10),
      sub_hesai_,
      sub_rsairy_);
    sync_->setMaxIntervalDuration(
      rclcpp::Duration::from_seconds(sync_tolerance_));
    sync_->registerCallback(
      std::bind(&CloudMergerNode::callback, this,
                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(),
      "cloud_merger_node ready. target_frame=%s sync_tolerance=%.3fs",
      target_frame_.c_str(), sync_tolerance_);
  }

private:
  void callback(const PointCloud2::ConstSharedPtr & hesai_msg,
                const PointCloud2::ConstSharedPtr & rsairy_msg)
  {
    // ── 1. Check timestamp delta ──────────────────────────────────────────
    double t_hesai  = rclcpp::Time(hesai_msg->header.stamp).seconds();
    double t_rsairy = rclcpp::Time(rsairy_msg->header.stamp).seconds();
    double dt       = std::abs(t_hesai - t_rsairy);

    if (dt > sync_tolerance_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "LiDAR sync gap %.3f s > tolerance %.3f s — dropping frame",
        dt, sync_tolerance_);
      return;
    }

    // ── 2. Transform both clouds into target_frame ────────────────────────
    PointCloud2 hesai_tf, rsairy_tf;
    try {
      auto t0 = std::chrono::steady_clock::now();

      auto tf_hesai = tf_buffer_->lookupTransform(
        target_frame_, hesai_msg->header.frame_id,
        hesai_msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
      tf2::doTransform(*hesai_msg, hesai_tf, tf_hesai);

      auto tf_rsairy = tf_buffer_->lookupTransform(
        target_frame_, rsairy_msg->header.frame_id,
        rsairy_msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
      tf2::doTransform(*rsairy_msg, rsairy_tf, tf_rsairy);

      auto t1 = std::chrono::steady_clock::now();
      double tf_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
      RCLCPP_DEBUG(get_logger(), "TF lookup+transform: %.1f ms", tf_ms);
    }
    catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "TF lookup failed: %s", ex.what());
      return;
    }

    // ── 3. Concatenate ────────────────────────────────────────────────────
    PointCloud2 merged;
    if (fields_compatible(hesai_tf, rsairy_tf)) {
      merged = fast_concat(hesai_tf, rsairy_tf, target_frame_);
    } else {
      RCLCPP_WARN_ONCE(get_logger(),
        "LiDAR field layouts differ — falling back to PointXYZI concat "
        "(Hesai point_step=%u, RSAiry point_step=%u)",
        hesai_tf.point_step, rsairy_tf.point_step);
      merged = xyzi_concat(hesai_tf, rsairy_tf, target_frame_);
    }

    uint32_t n_hesai  = hesai_tf.width  * hesai_tf.height;
    uint32_t n_rsairy = rsairy_tf.width * rsairy_tf.height;
    uint32_t n_merged = merged.width    * merged.height;

    RCLCPP_DEBUG(get_logger(),
      "hesai=%u  rsairy=%u  merged=%u  dt=%.3fs",
      n_hesai, n_rsairy, n_merged, dt);

    // ── 4. Publish full cloud (debug) ─────────────────────────────────────
    if (pub_full_->get_subscription_count() > 0) {
      pub_full_->publish(merged);
    }

    // ── 5. Self-filter and publish mapper cloud ───────────────────────────
    if (pub_filtered_->get_subscription_count() > 0) {
      PointCloud2 filtered = merged;

      // BB1: chassis (large box, excludes tractor body, trailer is beyond -x edge)
      bbox_filter_inplace(filtered,
        static_cast<float>(bbox_chassis_x_[0]), static_cast<float>(bbox_chassis_x_[1]),
        static_cast<float>(bbox_chassis_y_[0]), static_cast<float>(bbox_chassis_y_[1]),
        static_cast<float>(bbox_chassis_z_[0]), static_cast<float>(bbox_chassis_z_[1]));

      // BB2: LiDAR cage/mount (small box at sensor cluster height)
      bbox_filter_inplace(filtered,
        static_cast<float>(bbox_cage_x_[0]), static_cast<float>(bbox_cage_x_[1]),
        static_cast<float>(bbox_cage_y_[0]), static_cast<float>(bbox_cage_y_[1]),
        static_cast<float>(bbox_cage_z_[0]), static_cast<float>(bbox_cage_z_[1]));

      uint32_t n_filtered = filtered.width * filtered.height;
      RCLCPP_DEBUG(get_logger(),
        "filtered: %u → %u points (removed %u)",
        n_merged, n_filtered, n_merged - n_filtered);

      pub_filtered_->publish(filtered);
    }
  }

  // Parameters
  std::string target_frame_;
  double      sync_tolerance_;
  std::vector<double> bbox_chassis_x_, bbox_chassis_y_, bbox_chassis_z_;
  std::vector<double> bbox_cage_x_,    bbox_cage_y_,    bbox_cage_z_;

  // TF
  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Publishers
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_full_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_filtered_;

  // Synchronized subscribers
  message_filters::Subscriber<PointCloud2>                    sub_hesai_;
  message_filters::Subscriber<PointCloud2>                    sub_rsairy_;
  std::shared_ptr<message_filters::Synchronizer<ApproxSync>>  sync_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CloudMergerNode>());
  rclcpp::shutdown();
  return 0;
}
