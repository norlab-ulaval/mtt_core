// cloud_merger_node.cpp — Dual-LiDAR temporal fusion for MTT-HL
//
// Subscribes independently to:
//   Hesai XT-32  /hesai_lidar/points  ~20 Hz
//   RS-Airy      /rsairy_ns/points    ~10 Hz
//
// Architecture: cache-based temporal pairing (replaces message_filters
// ApproximateTime, which drops half the Hesai frames because each RS-Airy
// scan can only be matched once).
//
// With anchor_sensor="hesai" (default):
//   - Publish one merged cloud per Hesai scan at ~20 Hz.
//   - Pair each Hesai frame with the nearest RS-Airy in the cache.
//   - The same RS-Airy scan may be reused for two consecutive Hesai frames
//     (allow_reuse_other_sensor=true). No Hesai temporal info is lost.
//
// Published topics:
//   /merged_points_raw        — always raw fusion, no self-filter
//   /merged_points            — alias (backward compat)
//   /merged_points_reliable   — optional raw fusion with reliable QoS for mappers
//   /merged_points_filtered   — only when publish_filtered=true
//   /hesai_in_base            — only when publish_debug_inputs=true
//   /rsairy_in_base           — only when publish_debug_inputs=true
//
// ── Fusion math ─────────────────────────────────────────────────────────────
//
// For each point p_L expressed in LiDAR frame L, the rigid-body transform to
// the target frame B (base_link) is:
//
//     p_B = R_BL * p_L + t_BL
//
// In homogeneous 4×4 form:
//
//     p_B_h = T_BL * p_L_h,     T_BL ∈ SE(3)
//
// where T_BL = [ R_BL | t_BL ]
//              [  0   |   1  ]
//
// T_BL is queried from the TF tree at the scan stamp via tf2::doTransform,
// which handles the full SE(3) rotation+translation internally.
//
// TODO(deskew): For rolling-shutter / spinning LiDARs each ring point i has
// its own acquisition timestamp t_i. A motion-corrected ("deskewed") transform
// uses a constant body twist xi ∈ se(3) (6-DOF velocity in body frame):
//
//     T(t_i) = T(t_0) * Exp(xi * delta_t_i),    delta_t_i = t_i - t_0
//
// where Exp is the matrix exponential on SE(3) (Rodrigues-style for
// the rotation part, exact for the translation).  Implement this step when
// per-point timestamps are available as a PointCloud2 field ("t" or "time").

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/header.hpp>

#include <atomic>
#include <cmath>
#include <cstring>
#include <deque>
#include <functional>
#include <limits>
#include <mutex>
#include <set>
#include <string>
#include <vector>

using PointCloud2 = sensor_msgs::msg::PointCloud2;

// ─── sensor identity ─────────────────────────────────────────────────────────

enum class SensorId : uint16_t { HESAI = 0, RSAIRY = 1 };

// ─── timestamped cache entry ─────────────────────────────────────────────────

struct CachedCloud {
  PointCloud2::ConstSharedPtr msg;
  rclcpp::Time                stamp;
  uint64_t                    seq_id;
};

// ─── free helpers ────────────────────────────────────────────────────────────

static inline uint32_t countPoints(const PointCloud2 & c)
{
  return c.width * c.height;
}

// Returns true and fills off_{x,y,z} if cloud has FLOAT32 x/y/z fields.
static bool validateXYZFields(const PointCloud2 & cloud,
                               uint32_t & off_x,
                               uint32_t & off_y,
                               uint32_t & off_z)
{
  constexpr uint32_t NONE = std::numeric_limits<uint32_t>::max();
  off_x = off_y = off_z = NONE;
  for (const auto & f : cloud.fields) {
    if (f.datatype != sensor_msgs::msg::PointField::FLOAT32 || f.count < 1) continue;
    // FLOAT32 is 4 bytes — verify the field fits inside one point record
    if (f.offset + 4u > cloud.point_step) continue;
    if      (f.name == "x") off_x = f.offset;
    else if (f.name == "y") off_y = f.offset;
    else if (f.name == "z") off_z = f.offset;
  }
  return (off_x != NONE && off_y != NONE && off_z != NONE);
}

// True when both clouds have an identical field descriptor set (name, offset,
// datatype, count) and the same point_step.  Guarantees that a raw byte-copy
// concatenation is lossless and produces a valid PointCloud2.
static bool fieldsCompatible(const PointCloud2 & a, const PointCloud2 & b)
{
  if (a.is_bigendian  != b.is_bigendian  ||
      a.point_step    != b.point_step    ||
      a.fields.size() != b.fields.size()) return false;
  for (size_t i = 0; i < a.fields.size(); ++i) {
    const auto & fa = a.fields[i];
    const auto & fb = b.fields[i];
    if (fa.name     != fb.name     ||
        fa.offset   != fb.offset   ||
        fa.datatype != fb.datatype ||
        fa.count    != fb.count)    return false;
  }
  return true;
}

// ── Path A: fast compatible concat ───────────────────────────────────────────
//
// Both clouds have an identical field layout.  Allocates the output buffer
// once and copies both data payloads with two memcpy calls.
// All original fields are preserved; no PCL conversion is performed.
static PointCloud2 concatCompatible(const PointCloud2 & a,
                                    const PointCloud2 & b,
                                    const std_msgs::msg::Header & out_header)
{
  PointCloud2 out;
  out.header = out_header;  // stamp = anchor scan time, frame_id = target_frame
  out.height          = 1;
  out.width           = countPoints(a) + countPoints(b);
  out.fields          = a.fields;
  out.is_bigendian    = a.is_bigendian;
  out.point_step      = a.point_step;
  out.row_step        = out.point_step * out.width;
  out.is_dense        = a.is_dense && b.is_dense;

  // Single allocation
  out.data.resize(static_cast<size_t>(out.row_step));
  std::memcpy(out.data.data(),                 a.data.data(), a.data.size());
  std::memcpy(out.data.data() + a.data.size(), b.data.data(), b.data.size());
  return out;
}

// ── Intensity field descriptor ────────────────────────────────────────────────
//
// Carries the validated offset and datatype for an intensity field so the
// per-point reader can dispatch correctly without re-scanning the field list.
struct IntensityField {
  uint32_t offset   = std::numeric_limits<uint32_t>::max();
  uint8_t  datatype = 0;
  bool     valid    = false;
};

// Find and validate the intensity field in a PointCloud2.
// Accepts UINT8/UINT16/UINT32/FLOAT32.  Rejects the field if its byte range
// would extend past point_step (unsafe memory read guard).
static IntensityField findIntensityField(const PointCloud2 & cloud)
{
  for (const auto & f : cloud.fields) {
    if (f.name != "intensity" || f.count < 1) continue;
    uint32_t dtype_size = 0;
    switch (f.datatype) {
      case sensor_msgs::msg::PointField::UINT8:   dtype_size = 1; break;
      case sensor_msgs::msg::PointField::UINT16:  dtype_size = 2; break;
      case sensor_msgs::msg::PointField::UINT32:  dtype_size = 4; break;
      case sensor_msgs::msg::PointField::FLOAT32: dtype_size = 4; break;
      default: continue;  // unsupported datatype — skip silently
    }
    if (f.offset + dtype_size > cloud.point_step) continue;  // unsafe — skip
    return IntensityField{f.offset, f.datatype, true};
  }
  return IntensityField{};  // no valid intensity field
}

// Read intensity from a raw point byte-pointer, converting to float.
// Called only when IntensityField::valid == true.
static inline float readIntensity(const uint8_t * p, const IntensityField & fi)
{
  switch (fi.datatype) {
    case sensor_msgs::msg::PointField::UINT8:  {
      uint8_t v; std::memcpy(&v, p + fi.offset, 1);
      return static_cast<float>(v);
    }
    case sensor_msgs::msg::PointField::UINT16: {
      uint16_t v; std::memcpy(&v, p + fi.offset, 2);
      return static_cast<float>(v);
    }
    case sensor_msgs::msg::PointField::UINT32: {
      uint32_t v; std::memcpy(&v, p + fi.offset, 4);
      return static_cast<float>(v);
    }
    case sensor_msgs::msg::PointField::FLOAT32: {
      float v; std::memcpy(&v, p + fi.offset, 4);
      return v;
    }
    default: return 0.0f;
  }
}

// ── Path B: manual normalized concat ─────────────────────────────────────────
//
// Used when field layouts differ, or when normalize_fields=true is set.
// Extracts x/y/z/intensity from raw PointCloud2 bytes without going through
// PCL.  Adds a sensor_id field (UINT16: 0=Hesai, 1=RS-Airy).
//
// Output field layout (point_step = 20 bytes, 4-byte aligned):
//   offset  0 : x         FLOAT32
//   offset  4 : y         FLOAT32
//   offset  8 : z         FLOAT32
//   offset 12 : intensity FLOAT32   (0.0 if source has no intensity field)
//   offset 16 : sensor_id UINT16    (0=Hesai, 1=RS-Airy)
//   offset 18 : (2 bytes padding)
//
// The output buffer is pre-allocated for the maximum possible point count
// and written with raw pointer arithmetic — no vector insert in a loop.
static PointCloud2 concatNormalized(const PointCloud2 & a, SensorId id_a,
                                    const PointCloud2 & b, SensorId id_b,
                                    const std_msgs::msg::Header & out_header,
                                    bool remove_invalid)
{
  constexpr uint32_t OFF_X      = 0;
  constexpr uint32_t OFF_Y      = 4;
  constexpr uint32_t OFF_Z      = 8;
  constexpr uint32_t OFF_INT    = 12;
  constexpr uint32_t OFF_SID    = 16;
  constexpr uint32_t POINT_STEP = 20;

  // Validate source x/y/z layout (checks FLOAT32 + safe offset)
  uint32_t ax, ay, az, bx, by, bz;
  const bool a_ok = validateXYZFields(a, ax, ay, az);
  const bool b_ok = validateXYZFields(b, bx, by, bz);

  // Find validated intensity fields (UINT8/UINT16/UINT32/FLOAT32, offset-safe)
  const IntensityField a_int = findIntensityField(a);
  const IntensityField b_int = findIntensityField(b);

  const uint32_t na = a_ok ? countPoints(a) : 0u;
  const uint32_t nb = b_ok ? countPoints(b) : 0u;

  // Build output descriptor
  PointCloud2 out;
  out.header     = out_header;  // stamp = anchor scan time, frame_id = target_frame
  out.height     = 1;
  out.point_step = POINT_STEP;
  out.is_bigendian = false;
  out.is_dense   = false;

  {
    auto add_field = [&](const std::string & name, uint8_t dtype, uint32_t off) {
      sensor_msgs::msg::PointField pf;
      pf.name = name; pf.offset = off; pf.datatype = dtype; pf.count = 1;
      out.fields.push_back(pf);
    };
    add_field("x",         sensor_msgs::msg::PointField::FLOAT32, OFF_X);
    add_field("y",         sensor_msgs::msg::PointField::FLOAT32, OFF_Y);
    add_field("z",         sensor_msgs::msg::PointField::FLOAT32, OFF_Z);
    add_field("intensity", sensor_msgs::msg::PointField::FLOAT32, OFF_INT);
    add_field("sensor_id", sensor_msgs::msg::PointField::UINT16,  OFF_SID);
  }

  // Pre-allocate for the maximum possible number of points (single allocation)
  out.data.resize(static_cast<size_t>(POINT_STEP) * (na + nb), 0u);
  uint8_t * dst = out.data.data();
  uint32_t  n_written = 0;

  // Write one source cloud into the output buffer via raw pointer arithmetic
  auto write_src = [&](const PointCloud2 & src,
                        uint32_t n_pts,
                        uint32_t ox, uint32_t oy, uint32_t oz,
                        const IntensityField & int_field,
                        uint16_t sid)
  {
    const uint8_t * p    = src.data.data();
    const uint32_t  step = src.point_step;
    for (uint32_t i = 0; i < n_pts; ++i, p += step) {
      float x, y, z;
      std::memcpy(&x, p + ox, 4);
      std::memcpy(&y, p + oy, 4);
      std::memcpy(&z, p + oz, 4);

      if (remove_invalid && (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)))
        continue;

      // Read intensity — supports UINT8/UINT16/UINT32/FLOAT32; 0.0f if absent
      const float intensity = int_field.valid ? readIntensity(p, int_field) : 0.0f;

      std::memcpy(dst + OFF_X,   &x,         4);
      std::memcpy(dst + OFF_Y,   &y,         4);
      std::memcpy(dst + OFF_Z,   &z,         4);
      std::memcpy(dst + OFF_INT, &intensity, 4);
      std::memcpy(dst + OFF_SID, &sid,       2);
      dst += POINT_STEP;
      ++n_written;
    }
  };

  if (a_ok) write_src(a, na, ax, ay, az, a_int, static_cast<uint16_t>(id_a));
  if (b_ok) write_src(b, nb, bx, by, bz, b_int, static_cast<uint16_t>(id_b));

  // Trim buffer if remove_invalid skipped any points
  out.width    = n_written;
  out.row_step = POINT_STEP * n_written;
  out.data.resize(static_cast<size_t>(POINT_STEP) * n_written);
  return out;
}

// ── Bounding-box self-filter (in-place) ──────────────────────────────────────
//
// Removes points strictly inside [x_min,x_max] × [y_min,y_max] × [z_min,z_max].
// Operates directly on the serialized buffer — no PCL conversion.
// Must only be called on a copy of the merged cloud; never on the raw output.
static void bboxFilterInplace(PointCloud2 & cloud,
                               float x_min, float x_max,
                               float y_min, float y_max,
                               float z_min, float z_max)
{
  uint32_t off_x, off_y, off_z;
  if (!validateXYZFields(cloud, off_x, off_y, off_z)) return;

  const uint32_t step = cloud.point_step;
  const uint32_t n_in = countPoints(cloud);

  // Pre-allocate to avoid repeated reallocations inside the loop
  std::vector<uint8_t> buf_out(cloud.data.size());
  uint8_t * dst = buf_out.data();
  uint32_t  n_out = 0;

  const uint8_t * src = cloud.data.data();
  for (uint32_t i = 0; i < n_in; ++i, src += step) {
    float x, y, z;
    std::memcpy(&x, src + off_x, 4);
    std::memcpy(&y, src + off_y, 4);
    std::memcpy(&z, src + off_z, 4);

    const bool inside = (x > x_min && x < x_max) &&
                        (y > y_min && y < y_max) &&
                        (z > z_min && z < z_max);
    if (!inside) {
      std::memcpy(dst, src, step);
      dst  += step;
      ++n_out;
    }
  }
  buf_out.resize(static_cast<size_t>(step) * n_out);
  cloud.data     = std::move(buf_out);
  cloud.width    = n_out;
  cloud.height   = 1;
  cloud.row_step = step * n_out;
}

static PointCloud2 strideSampleCloud(const PointCloud2 & cloud, int stride)
{
  if (stride <= 1 || cloud.point_step == 0) return cloud;

  const uint32_t n_in = countPoints(cloud);
  const uint32_t n_out = (n_in + static_cast<uint32_t>(stride) - 1u) /
                         static_cast<uint32_t>(stride);

  PointCloud2 out = cloud;
  out.height = 1;
  out.width = n_out;
  out.row_step = out.point_step * out.width;
  out.data.resize(static_cast<size_t>(out.row_step));

  const uint8_t * src = cloud.data.data();
  uint8_t * dst = out.data.data();
  uint32_t written = 0;
  for (uint32_t i = 0; i < n_in; i += static_cast<uint32_t>(stride)) {
    std::memcpy(dst, src + static_cast<size_t>(i) * cloud.point_step, cloud.point_step);
    dst += cloud.point_step;
    ++written;
  }
  out.width = written;
  out.row_step = out.point_step * out.width;
  out.data.resize(static_cast<size_t>(out.row_step));
  return out;
}

// ─── node ─────────────────────────────────────────────────────────────────────

class CloudMergerNode : public rclcpp::Node
{
public:
  CloudMergerNode()
  : Node("cloud_merger_node"),
    hesai_seq_(0), rsairy_seq_(0),
    last_used_other_seq_(0),
    hesai_rx_count_(0), rsairy_rx_count_(0),
    published_count_(0), no_pair_drop_count_(0),
    tf_drop_count_(0), reuse_count_(0),
    pair_dt_sum_(0.0), pair_dt_max_(0.0)
  {
    // ── Parameters ─────────────────────────────────────────────────────────
    target_frame_             = declare_parameter<std::string>("target_frame",           "base_link");
    anchor_sensor_str_        = declare_parameter<std::string>("anchor_sensor",          "hesai");
    max_pair_dt_              = declare_parameter<double>     ("max_pair_dt",             0.075);
    tf_timeout_               = declare_parameter<double>     ("tf_timeout",              0.05);
    max_cache_size_           = declare_parameter<int>        ("max_cache_size",          20);
    allow_reuse_other_sensor_ = declare_parameter<bool>       ("allow_reuse_other_sensor", true);
    publish_filtered_         = declare_parameter<bool>       ("publish_filtered",         false);
    publish_reliable_raw_     = declare_parameter<bool>       ("publish_reliable_raw_for_mapping", false);
    publish_debug_inputs_     = declare_parameter<bool>       ("publish_debug_inputs",     false);
    normalize_fields_         = declare_parameter<bool>       ("normalize_fields",         true);
    remove_invalid_points_    = declare_parameter<bool>       ("remove_invalid_points",    false);
    enable_self_bbox_filter_  = declare_parameter<bool>       ("enable_self_bbox_filter",  true);
    hesai_stride_             = declare_parameter<int>        ("hesai_stride",             1);
    rsairy_stride_            = declare_parameter<int>        ("rsairy_stride",            1);
    rsairy_inject_every_n_    = declare_parameter<int>        ("rsairy_inject_every_n",    1);

    // Bounding-box self-filter parameters
    // (Only affect /merged_points_filtered — raw output is never touched)
    bbox_chassis_x_ = declare_parameter<std::vector<double>>("bbox_chassis_x", {-0.90, 0.70});
    bbox_chassis_y_ = declare_parameter<std::vector<double>>("bbox_chassis_y", {-0.40, 0.40});
    bbox_chassis_z_ = declare_parameter<std::vector<double>>("bbox_chassis_z", {-0.10, 0.80});
    bbox_cage_x_    = declare_parameter<std::vector<double>>("bbox_cage_x",    {-0.25, 0.15});
    bbox_cage_y_    = declare_parameter<std::vector<double>>("bbox_cage_y",    {-0.20, 0.20});
    bbox_cage_z_    = declare_parameter<std::vector<double>>("bbox_cage_z",    { 0.70, 1.10});
    enable_trailer_bbox_filter_ = declare_parameter<bool>("enable_trailer_bbox_filter", false);
    bbox_trailer_x_ = declare_parameter<std::vector<double>>("bbox_trailer_x", {-4.20, -0.75});
    bbox_trailer_y_ = declare_parameter<std::vector<double>>("bbox_trailer_y", {-1.35, 1.35});
    bbox_trailer_z_ = declare_parameter<std::vector<double>>("bbox_trailer_z", {-0.35, 2.50});

    // Parse anchor mode
    if      (anchor_sensor_str_ == "hesai")  anchor_mode_ = AnchorMode::HESAI;
    else if (anchor_sensor_str_ == "rsairy") anchor_mode_ = AnchorMode::RSAIRY;
    else if (anchor_sensor_str_ == "either") anchor_mode_ = AnchorMode::EITHER;
    else {
      RCLCPP_WARN(get_logger(),
        "[cloud_merger] Unknown anchor_sensor '%s' — falling back to 'hesai'",
        anchor_sensor_str_.c_str());
      anchor_mode_       = AnchorMode::HESAI;
      anchor_sensor_str_ = "hesai";
    }

    // ── TF ─────────────────────────────────────────────────────────────────
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ── QoS ────────────────────────────────────────────────────────────────
    // Raw fusion output: best-effort so a slow subscriber never blocks fusion.
    // Filtered output: reliable for mapper nodes that require it.
    const auto sensor_qos   = rclcpp::SensorDataQoS();
    const auto reliable_qos = rclcpp::QoS(10).reliable();

    // ── Publishers ─────────────────────────────────────────────────────────
    pub_raw_  = create_publisher<PointCloud2>("merged_points_raw", sensor_qos);
    pub_full_ = create_publisher<PointCloud2>("merged_points",     sensor_qos);  // compat alias

    if (publish_reliable_raw_)
      pub_reliable_raw_ = create_publisher<PointCloud2>("merged_points_reliable", reliable_qos);

    if (publish_filtered_)
      pub_filtered_ = create_publisher<PointCloud2>("merged_points_filtered", reliable_qos);

    if (publish_debug_inputs_) {
      pub_hesai_in_base_  = create_publisher<PointCloud2>("hesai_in_base",  sensor_qos);
      pub_rsairy_in_base_ = create_publisher<PointCloud2>("rsairy_in_base", sensor_qos);
    }

    // ── Callback groups ────────────────────────────────────────────────────
    // Separate groups so Hesai and RS-Airy callbacks can execute concurrently
    // on the MultiThreadedExecutor without blocking each other.
    cb_group_hesai_  = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    cb_group_rsairy_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions opt_h, opt_r;
    opt_h.callback_group = cb_group_hesai_;
    opt_r.callback_group = cb_group_rsairy_;

    sub_hesai_ = create_subscription<PointCloud2>(
      "/hesai_lidar/points", sensor_qos,
      std::bind(&CloudMergerNode::hesaiCallback, this, std::placeholders::_1),
      opt_h);

    sub_rsairy_ = create_subscription<PointCloud2>(
      "/rsairy_ns/points", sensor_qos,
      std::bind(&CloudMergerNode::rsairyCallback, this, std::placeholders::_1),
      opt_r);

    RCLCPP_INFO(get_logger(),
      "[cloud_merger] ready — target_frame=%s anchor=%s "
      "max_pair_dt=%.3fs tf_timeout=%.3fs max_cache=%d "
      "reuse=%s normalize=%s filtered=%s reliable_raw=%s self_filter=%s "
      "debug_inputs=%s remove_invalid=%s stride(H=%d R=%d) rsairy_inject_every_n=%d",
      target_frame_.c_str(), anchor_sensor_str_.c_str(),
      max_pair_dt_, tf_timeout_, max_cache_size_,
      allow_reuse_other_sensor_ ? "true" : "false",
      normalize_fields_         ? "true" : "false",
      publish_filtered_         ? "true" : "false",
      publish_reliable_raw_     ? "true" : "false",
      enable_self_bbox_filter_  ? "true" : "false",
      publish_debug_inputs_     ? "true" : "false",
      remove_invalid_points_    ? "true" : "false",
      hesai_stride_, rsairy_stride_, rsairy_inject_every_n_);
  }

private:
  enum class AnchorMode { HESAI, RSAIRY, EITHER };

  // ── Subscription callbacks ────────────────────────────────────────────────

  void hesaiCallback(const PointCloud2::ConstSharedPtr msg)
  {
    uint64_t seq;
    {
      std::lock_guard<std::mutex> lk(hesai_mtx_);
      seq = ++hesai_seq_;
      hesai_cache_.push_back({msg, rclcpp::Time(msg->header.stamp), seq});
      while (hesai_cache_.size() > static_cast<size_t>(max_cache_size_))
        hesai_cache_.pop_front();
    }
    ++hesai_rx_count_;

    if (anchor_mode_ == AnchorMode::HESAI || anchor_mode_ == AnchorMode::EITHER)
      tryFuseFromAnchor(msg, SensorId::HESAI, seq);
  }

  void rsairyCallback(const PointCloud2::ConstSharedPtr msg)
  {
    uint64_t seq;
    {
      std::lock_guard<std::mutex> lk(rsairy_mtx_);
      seq = ++rsairy_seq_;
      rsairy_cache_.push_back({msg, rclcpp::Time(msg->header.stamp), seq});
      while (rsairy_cache_.size() > static_cast<size_t>(max_cache_size_))
        rsairy_cache_.pop_front();
    }
    ++rsairy_rx_count_;

    if (anchor_mode_ == AnchorMode::RSAIRY || anchor_mode_ == AnchorMode::EITHER)
      tryFuseFromAnchor(msg, SensorId::RSAIRY, seq);
  }

  // ── Core fusion driver ────────────────────────────────────────────────────
  //
  // Called for each anchor scan.  Looks up the nearest scan from the opposite
  // sensor, transforms both to target_frame, concatenates, and publishes.
  void tryFuseFromAnchor(const PointCloud2::ConstSharedPtr anchor_msg,
                          SensorId anchor_id,
                          uint64_t anchor_seq)
  {
    const rclcpp::Time anchor_stamp(anchor_msg->header.stamp);
    const bool request_rsairy =
      (anchor_id != SensorId::HESAI) ||
      (rsairy_inject_every_n_ <= 1) ||
      (anchor_seq % static_cast<uint64_t>(rsairy_inject_every_n_) == 0);

    // ── 1. Find nearest matching cloud from the other sensor ───────────────
    CachedCloud other;
    bool has_other = findNearestInCache(anchor_stamp, anchor_id, other);
    if (!has_other && (anchor_id != SensorId::HESAI || request_rsairy)) {
      ++no_pair_drop_count_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "[cloud_merger] no pair within %.3fs for anchor=%s t=%.3f — using anchor only when possible "
        "(no_pair_drops=%lu)",
        max_pair_dt_,
        anchor_id == SensorId::HESAI ? "hesai" : "rsairy",
        anchor_stamp.seconds(),
        no_pair_drop_count_.load());
    }
    if (!has_other && anchor_id != SensorId::HESAI) return;

    bool include_rsairy = (anchor_id == SensorId::HESAI) && request_rsairy && has_other;
    const double dt = has_other ? std::abs((anchor_stamp - other.stamp).seconds()) : 0.0;

    // ── 2. Deduplicate in "either" mode ────────────────────────────────────
    // Build a canonical pair hash: high 32 bits = hesai seq, low 32 = rsairy seq.
    // Prevents publishing the same physical pair twice when both sensors trigger.
    if (has_other && anchor_mode_ == AnchorMode::EITHER) {
      const uint64_t h_seq = (anchor_id == SensorId::HESAI) ? anchor_seq : other.seq_id;
      const uint64_t r_seq = (anchor_id == SensorId::RSAIRY) ? anchor_seq : other.seq_id;
      const uint64_t pair_hash = (h_seq << 32u) | (r_seq & 0xFFFF'FFFFull);
      {
        std::lock_guard<std::mutex> lk(published_pairs_mtx_);
        if (!published_pairs_.insert(pair_hash).second) return;  // already published
        if (published_pairs_.size() > 128)
          published_pairs_.erase(published_pairs_.begin());
      }
    }

    // ── 3. Reuse tracking (single-anchor modes only) ───────────────────────
    bool reused_other = false;
    if (has_other && include_rsairy && anchor_mode_ != AnchorMode::EITHER) {
      std::lock_guard<std::mutex> lk(reuse_mtx_);
      if (last_used_other_seq_ == other.seq_id) {
        reused_other = true;
        if (!allow_reuse_other_sensor_) {
          RCLCPP_DEBUG(get_logger(),
            "[cloud_merger] other-sensor seq=%lu already used — reuse disabled, skipping",
            other.seq_id);
          return;
        }
        ++reuse_count_;
      }
      last_used_other_seq_ = other.seq_id;
    }

    // ── 4. Resolve which pointer is Hesai and which is RS-Airy ────────────
    const PointCloud2::ConstSharedPtr & hesai_msg  =
      (anchor_id == SensorId::HESAI || !has_other) ? anchor_msg : other.msg;
    const PointCloud2::ConstSharedPtr rsairy_msg =
      include_rsairy ? ((anchor_id == SensorId::RSAIRY) ? anchor_msg : other.msg) : nullptr;

    // ── 5. Transform both into target_frame ───────────────────────────────
    // SE(3) transform: p_B = R_BL * p_L + t_BL  (see file header for full math)
    PointCloud2 hesai_tf, rsairy_tf;
    if (!transformCloud(*hesai_msg, hesai_tf)) {
      ++tf_drop_count_;
      return;  // transformCloud already emitted a throttled warning
    }
    if (include_rsairy && (!rsairy_msg || !transformCloud(*rsairy_msg, rsairy_tf))) {
      include_rsairy = false;
      ++tf_drop_count_;
    }
    if (hesai_stride_ > 1) hesai_tf = strideSampleCloud(hesai_tf, hesai_stride_);
    if (include_rsairy && rsairy_stride_ > 1)
      rsairy_tf = strideSampleCloud(rsairy_tf, rsairy_stride_);

    // ── 6. Concatenate ─────────────────────────────────────────────────────
    // Build the output header once: stamp = anchor scan time (always correct
    // regardless of which sensor is the anchor), frame_id = target_frame.
    std_msgs::msg::Header out_header = anchor_msg->header;
    out_header.frame_id = target_frame_;

    PointCloud2 merged;
    const bool compatible = include_rsairy && fieldsCompatible(hesai_tf, rsairy_tf);

    if (!include_rsairy) {
      merged = hesai_tf;
      merged.header = out_header;
    } else if (compatible && !normalize_fields_) {
      // Path A: field-identical sources — single-allocation byte copy.
      // All original fields (ring, timestamp, etc.) are preserved.
      merged = concatCompatible(hesai_tf, rsairy_tf, out_header);
    } else {
      // Path B: normalized output with sensor_id field.
      // Triggered when fields differ OR normalize_fields=true.
      if (!compatible) {
        RCLCPP_WARN_ONCE(get_logger(),
          "[cloud_merger] field layouts differ (hesai step=%u  rsairy step=%u) "
          "— using normalized path",
          hesai_tf.point_step, rsairy_tf.point_step);
      }
      merged = concatNormalized(hesai_tf, SensorId::HESAI,
                                rsairy_tf, SensorId::RSAIRY,
                                out_header, remove_invalid_points_);
    }

    // ── 7. Diagnostics ─────────────────────────────────────────────────────
    const uint32_t n_hesai  = countPoints(hesai_tf);
    const uint32_t n_rsairy = include_rsairy ? countPoints(rsairy_tf) : 0u;
    const uint32_t n_merged = countPoints(merged);

    uint64_t pub_snap, np_snap, tf_snap, ru_snap;
    double   avg_dt_snap, max_dt_snap;
    {
      std::lock_guard<std::mutex> lk(diag_mtx_);
      if (include_rsairy) {
        pair_dt_sum_ += dt;
        if (dt > pair_dt_max_) pair_dt_max_ = dt;
      }
      pub_snap    = ++published_count_;
      np_snap     = no_pair_drop_count_.load();
      tf_snap     = tf_drop_count_.load();
      ru_snap     = reuse_count_.load();
      avg_dt_snap = (pub_snap > 0) ? pair_dt_sum_ / static_cast<double>(pub_snap) : 0.0;
      max_dt_snap = pair_dt_max_;
    }

    // One INFO line at 5 s throttle (matches the format requested in the spec)
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
      "[cloud_merger] H=%u R=%u merged=%u dt=%.3fs anchor=%s reused_other=%s | "
      "rx(H=%lu R=%lu) pub=%lu no_pair=%lu tf_drop=%lu reuse=%lu "
      "avg_dt=%.3fs max_dt=%.3fs",
      n_hesai, n_rsairy, n_merged, dt,
      anchor_sensor_str_.c_str(),
      reused_other ? "true" : "false",
      hesai_rx_count_.load(), rsairy_rx_count_.load(),
      pub_snap, np_snap, tf_snap, ru_snap,
      avg_dt_snap, max_dt_snap);

    // ── 8. Publish raw merged cloud ────────────────────────────────────────
    // The raw cloud is never filtered.  Publish to both /merged_points_raw and
    // /merged_points (backward-compat alias).  Check subscriber count to
    // avoid serialization overhead when nobody is listening.
    if (pub_raw_->get_subscription_count() > 0)
      pub_raw_->publish(merged);
    if (pub_full_->get_subscription_count() > 0)
      pub_full_->publish(merged);
    if (pub_reliable_raw_ && pub_reliable_raw_->get_subscription_count() > 0)
      pub_reliable_raw_->publish(merged);

    // ── 9. Debug: individual transformed clouds ────────────────────────────
    if (publish_debug_inputs_) {
      if (pub_hesai_in_base_ && pub_hesai_in_base_->get_subscription_count() > 0)
        pub_hesai_in_base_->publish(hesai_tf);
      if (pub_rsairy_in_base_ && pub_rsairy_in_base_->get_subscription_count() > 0)
        pub_rsairy_in_base_->publish(rsairy_tf);
    }

    // ── 10. Self-filtered cloud (optional, never touches raw) ─────────────
    if (publish_filtered_ && pub_filtered_ &&
        pub_filtered_->get_subscription_count() > 0) {
      PointCloud2 filtered = merged;  // deep copy — raw output remains clean

      if (enable_self_bbox_filter_) {
        // BB1: tractor chassis (x=forward, y=left, z=up in base_link).
        // The trailer is further in -x and is not removed by this box.
        bboxFilterInplace(filtered,
          static_cast<float>(bbox_chassis_x_[0]), static_cast<float>(bbox_chassis_x_[1]),
          static_cast<float>(bbox_chassis_y_[0]), static_cast<float>(bbox_chassis_y_[1]),
          static_cast<float>(bbox_chassis_z_[0]), static_cast<float>(bbox_chassis_z_[1]));

        // BB2: LiDAR cage / mounting structure (tight box at sensor cluster height)
        bboxFilterInplace(filtered,
          static_cast<float>(bbox_cage_x_[0]), static_cast<float>(bbox_cage_x_[1]),
          static_cast<float>(bbox_cage_y_[0]), static_cast<float>(bbox_cage_y_[1]),
          static_cast<float>(bbox_cage_z_[0]), static_cast<float>(bbox_cage_z_[1]));

        // BB3: optional trailer/self-articulation mask for mapping input.
        if (enable_trailer_bbox_filter_) {
          bboxFilterInplace(filtered,
            static_cast<float>(bbox_trailer_x_[0]), static_cast<float>(bbox_trailer_x_[1]),
            static_cast<float>(bbox_trailer_y_[0]), static_cast<float>(bbox_trailer_y_[1]),
            static_cast<float>(bbox_trailer_z_[0]), static_cast<float>(bbox_trailer_z_[1]));
        }
      }

      RCLCPP_DEBUG(get_logger(),
        "[cloud_merger] filtered: %u → %u pts (removed %u)",
        n_merged, countPoints(filtered), n_merged - countPoints(filtered));

      pub_filtered_->publish(filtered);
    }
  }

  // ── Cache lookup ──────────────────────────────────────────────────────────
  //
  // Finds the entry in the cache of the sensor opposite to anchor_id whose
  // stamp is closest to anchor_stamp, provided abs(dt) <= max_pair_dt_.
  bool findNearestInCache(const rclcpp::Time & anchor_stamp,
                           SensorId             anchor_id,
                           CachedCloud        & result)
  {
    std::deque<CachedCloud> * cache;
    std::mutex               * mtx;
    if (anchor_id == SensorId::HESAI) {
      cache = &rsairy_cache_;
      mtx   = &rsairy_mtx_;
    } else {
      cache = &hesai_cache_;
      mtx   = &hesai_mtx_;
    }

    std::lock_guard<std::mutex> lk(*mtx);
    if (cache->empty()) return false;

    double             best_dt = std::numeric_limits<double>::max();
    const CachedCloud * best   = nullptr;
    for (const auto & cc : *cache) {
      const double d = std::abs((anchor_stamp - cc.stamp).seconds());
      if (d < best_dt) { best_dt = d; best = &cc; }
    }
    if (!best || best_dt > max_pair_dt_) return false;
    result = *best;
    return true;
  }

  // ── SE(3) transform helper ────────────────────────────────────────────────
  //
  // Queries T_{target_frame ← in.frame_id} from the TF tree at in.stamp, then
  // applies the rigid-body transform to every point:
  //
  //     p_B = R_BL * p_L + t_BL
  //
  // tf2::doTransform handles the SE(3) math and PointCloud2 field iteration.
  bool transformCloud(const PointCloud2 & in, PointCloud2 & out)
  {
    // Skip transform if cloud is already in the target frame
    if (in.header.frame_id == target_frame_) {
      out = in;
      return true;
    }

    // Basic sanity: buffer must be large enough to hold declared points
    if (in.point_step > 0 &&
        in.data.size() < static_cast<size_t>(in.point_step) * countPoints(in)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "[cloud_merger] malformed cloud from %s: buffer too small (%zu < %u) — skipping",
        in.header.frame_id.c_str(),
        in.data.size(),
        in.point_step * countPoints(in));
      return false;
    }

    try {
      const auto tf = tf_buffer_->lookupTransform(
        target_frame_,
        in.header.frame_id,
        in.header.stamp,
        rclcpp::Duration::from_seconds(tf_timeout_));
      tf2::doTransform(in, out, tf);
      return true;
    }
    catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "[cloud_merger] TF lookup failed (%s → %s): %s",
        in.header.frame_id.c_str(), target_frame_.c_str(), ex.what());
      return false;
    }
  }

  // ── Member variables ──────────────────────────────────────────────────────

  // Parameters
  std::string         target_frame_;
  std::string         anchor_sensor_str_;
  AnchorMode          anchor_mode_;
  double              max_pair_dt_;
  double              tf_timeout_;
  int                 max_cache_size_;
  bool                allow_reuse_other_sensor_;
  bool                publish_filtered_;
  bool                publish_reliable_raw_;
  bool                publish_debug_inputs_;
  bool                normalize_fields_;
  bool                remove_invalid_points_;
  bool                enable_self_bbox_filter_;
  int                 hesai_stride_;
  int                 rsairy_stride_;
  int                 rsairy_inject_every_n_;
  std::vector<double> bbox_chassis_x_, bbox_chassis_y_, bbox_chassis_z_;
  std::vector<double> bbox_cage_x_,    bbox_cage_y_,    bbox_cage_z_;
  bool enable_trailer_bbox_filter_{false};
  std::vector<double> bbox_trailer_x_, bbox_trailer_y_, bbox_trailer_z_;

  // TF
  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Publishers
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_raw_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_full_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_reliable_raw_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_filtered_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_hesai_in_base_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_rsairy_in_base_;

  // Independent subscriptions
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_hesai_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_rsairy_;

  // Callback groups (allow Hesai + RS-Airy to execute in parallel)
  rclcpp::CallbackGroup::SharedPtr cb_group_hesai_;
  rclcpp::CallbackGroup::SharedPtr cb_group_rsairy_;

  // Caches: guarded by individual mutexes so producers/consumers don't block
  std::deque<CachedCloud> hesai_cache_;
  std::deque<CachedCloud> rsairy_cache_;
  std::mutex              hesai_mtx_;
  std::mutex              rsairy_mtx_;
  uint64_t                hesai_seq_;
  uint64_t                rsairy_seq_;

  // Reuse tracking (single-anchor modes)
  uint64_t   last_used_other_seq_;
  std::mutex reuse_mtx_;

  // "Either" mode pair deduplication: set of canonical (hesai_seq<<32)|rsairy_seq hashes
  std::set<uint64_t> published_pairs_;
  std::mutex         published_pairs_mtx_;

  // Diagnostics — counters are atomic for lock-free increment from both callbacks;
  // floating-point accumulators are guarded by diag_mtx_.
  std::atomic<uint64_t> hesai_rx_count_;
  std::atomic<uint64_t> rsairy_rx_count_;
  uint64_t              published_count_;     // under diag_mtx_
  std::atomic<uint64_t> no_pair_drop_count_;
  std::atomic<uint64_t> tf_drop_count_;
  std::atomic<uint64_t> reuse_count_;
  double                pair_dt_sum_;         // under diag_mtx_
  double                pair_dt_max_;         // under diag_mtx_
  std::mutex            diag_mtx_;
};

// ─── main ─────────────────────────────────────────────────────────────────────

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CloudMergerNode>();

  // MultiThreadedExecutor lets the Hesai and RS-Airy callbacks run concurrently
  // on separate threads, which is safe because they have independent callback
  // groups and all shared state is mutex-guarded.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
