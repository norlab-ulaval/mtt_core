// ISAM2-based Factor Graph state estimator for MTT.
// Fuses: IMU (preintegrated), track odom, GPS position, GPS heading,
// LiDAR odom, visual odom, and articulation (encoder + trailer LiDAR pose).
//
// Extended state at keyframe k:
//   X(k) = Pose3   — tractor pose in map
//   V(k) = Vector3 — tractor velocity
//   B(k) = ImuBias — IMU bias
//   H(k) = double  — hitch yaw angle φ   (articulation, symbol 'h')
//   P(k) = double  — hitch pitch angle α  (articulation, symbol 'p', if use_pitch_state)
//
// Articulation factors:
//   1. EncoderFactor      : PriorFactor<double> on H(k)    from hardware yaw encoder
//   2. ArticulationDynYaw : BetweenFactor<double> H(k-1)→H(k)  (φ random-walk)
//   3. PitchEncoder       : PriorFactor<double> on P(k)    from pitch potentiometer (ADC1)
//   4. ArticulationDynPitch: BetweenFactor<double> P(k-1)→P(k)  (α random-walk, slow)
//   5. TrailerPoseFactorFull: NoiseModelFactor2<double,double> on (H(k),P(k))
//      from trailer/pose LiDAR — 6D residual constrains both φ and α simultaneously
//
// Bayesian mutual aid (no circular dependency):
//   • Tractor sensors (IMU/GPS/odom) constrain X(k) independently.
//   • Encoder + trailer LiDAR constrain H(k) and P(k) independently.
//   • The joint posterior p(X(k), H(k), P(k) | all_z) is the product of independent
//     likelihoods — no measurement appears twice.
//
// Publishes:
//   localization/odom               — nav_msgs/Odometry  (map frame, with Σ from smoother)
//   localization/articulation_angle — std_msgs/Float64   (optimised φ for downstream)
//   localization/articulation_pitch — std_msgs/Float64   (optimised α for downstream)
//
// Out-of-sequence measurements (OOSM):
//   Backend is a gtsam::IncrementalFixedLagSmoother (not raw ISAM2) so a late-arriving,
//   higher-accuracy measurement (LiDAR-ICP odom, ZED/visual odom, future Isaac VSLAM
//   odom — all can lag real time by hundreds of ms) is attached to the EXISTING keyframe
//   whose timestamp is closest to when the measurement was actually captured
//   (keyForStamp()), instead of always binding to "now". The smoother then relinearizes
//   the affected window and the correction propagates forward through the existing
//   IMU/odom BetweenFactor chain to the present. Keys older than smoother_lag_seconds
//   are marginalized automatically. The X/V/B (IMU) chain itself is unaffected by this —
//   it remains a strictly sequential prev_key→curr_key chain every tick, since
//   CombinedImuFactor requires adjacent keyframes with actual preintegrated IMU data
//   between them; only single-key factors (GPS, heading, articulation priors,
//   TrailerPoseFactorFull) and relative BetweenFactor sources (track/lidar/visual odom)
//   get OOSM-routed to a historical key via keyForStamp().

#include <chrono>
#include <memory>
#include <mutex>
#include <deque>
#include <map>
#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Quaternion.h"

#include "mtt_msgs/msg/mtt_articulation_state.hpp"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ImuFactor.h>
// IncrementalFixedLagSmoother lives in gtsam_unstable in the packaged GTSAM 4.2.0
// (ros-jazzy-gtsam apt) used here — later upstream versions moved it to
// gtsam/nonlinear, but that path doesn't exist in this install.
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/inference/Symbol.h>

#include "mtt_localization/state.hpp"
#include "mtt_localization/trailer_pose_factor.hpp"

using namespace std::chrono_literals;
using gtsam::symbol_shorthand::X;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::B;

/// Hitch yaw angle symbol — letter 'h' avoids collision with X/V/B
inline gtsam::Key H_key(uint64_t i) { return gtsam::Symbol('h', i); }
/// Hitch pitch angle symbol
inline gtsam::Key P_key(uint64_t i) { return gtsam::Symbol('p', i); }

/// Wrap to (-pi, pi]. SO(2) residual — required whenever differencing two
/// angles (e.g. yaw/articulation innovations); a naive subtraction is wrong
/// across the +-pi branch cut and silently corrupts anything downstream.
inline double normalizeAngle(double a) { return std::atan2(std::sin(a), std::cos(a)); }

// WGS84
static constexpr double kEarthRadius = 6378137.0;
static constexpr double kDeg2Rad = M_PI / 180.0;

class FactorGraphNode : public rclcpp::Node {
public:
  FactorGraphNode() : Node("factor_graph_node") {
    declare_parameters();
    load_parameters();
    setup_smoother();
    setup_imu_preintegration();
    setup_publishers();
    setup_subscribers();

    double rate = get_parameter("publish_rate").as_double();
    opt_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / rate),
        std::bind(&FactorGraphNode::optimize_and_publish, this));

    RCLCPP_INFO(get_logger(),
        "Factor Graph node started (IncrementalFixedLagSmoother, lag=%.1fs, SE(3) + φ articulation)",
        smoother_lag_seconds_);
  }

private:
  // ─── Parameter declarations ──
  void declare_parameters() {
    declare_parameter("use_imu_primary", true);
    declare_parameter("use_track_odom", true);
    declare_parameter("use_gps", true);
    declare_parameter("use_gps_heading", true);
    // Absolute PriorFactor<Pose3> on X(k) from /mapping/icp_measurement (the
    // ICP mapper's own accepted-registration pose, never dead-reckoned, with
    // real registration-derived covariance -- mapper_node.cpp:176/:3484).
    // Re-anchors this node's "map" frame onto the MAPPER's "map" frame,
    // instead of the independent GPS/ENU origin GPSFactor anchors it to.
    // Fixes the root cause of the map->odom double-publisher bug at its
    // source (see broadcast_tf comment below): before this, mapper_node and
    // this node published TWO DIFFERENT frames that happened to share the
    // name "map" (mapper's is anchored at its initial robot pose; this
    // node's is anchored at gps_origin_lat/lon). With this on, they are the
    // SAME frame, so /localization/odom becomes directly usable wherever
    // /mapping/icp_odom is today -- including WILN's LiDAR teach & repeat,
    // which was previously in a different, incompatible frame.
    //
    // MUTUALLY EXCLUSIVE with use_gps: both are absolute position priors on
    // the same X(k) in DIFFERENT frames (mapper-local vs GPS/ENU) -- running
    // both fights the graph over what "map" means and is refused at startup
    // (see the throw in load_parameters()), not just discouraged by comment.
    // Re-anchoring to the mapper's frame also means /localization/odom stops
    // being geo-referenced/cross-session-comparable, which real GPS teach &
    // repeat needs -- so this is an explicit per-scenario choice, not a
    // universal upgrade: leave false (default, unchanged behaviour) for any
    // GPS-anchored session; set true (with use_gps:false) for GPS-less
    // sessions (indoor, ice rink -- see demos/data_collection/config/
    // ice_experiment_profiles.yaml) where LiDAR-only T&R is what matters.
    declare_parameter("use_map_anchor", false);
    declare_parameter("map_anchor_topic", "mapping/icp_measurement");
    declare_parameter("use_lidar_odom", true);
    declare_parameter("use_visual_odom", false);
    declare_parameter("use_articulation", true);
    declare_parameter("use_trailer_pose", true);
    declare_parameter("use_pitch_state", true);  // add P(k) pitch angle to ISAM2

    declare_parameter("map_frame", "map");
    declare_parameter("odom_frame", "odom");
    declare_parameter("base_frame", "base_footprint");
    // false: mapper_node (norlab_icp_mapper_ros) also broadcasts map->odom
    // (mapTfPublisherLoop, 50Hz) and is the frame's rightful owner -- it owns
    // the point cloud that DEFINES "map" (mapper's map is anchored at the
    // initial robot pose, anchor_map_at_initial_robot_pose default false;
    // this node's map is anchored at a surveyed GPS/ENU origin, see
    // gps_origin_lat/lon below -- two DIFFERENT frames sharing one name).
    // Both are default services in demos/live_robot/compose.yaml, so before
    // this flag both broadcast simultaneously: harmless jitter without GPS
    // (no GPS factors added, the two origins stay near-coincident) but a
    // real divergence with GPS enabled. wiln_obstacle_node accumulates
    // against the mapper's cloud (wiln.yaml target_frame: map), so the
    // mapper must be the sole owner. publish_tf() is kept, not deleted, but
    // stays off even when use_map_anchor (below) is true: at that point this
    // node's map->odom describes the SAME frame as the mapper's, but a
    // second broadcaster of a near-identical-but-not-bit-identical value
    // (the smoother's fused estimate vs the mapper's raw ICP pose) would
    // reintroduce jitter, just smaller. The refined estimate stays available
    // via the /localization/odom TOPIC either way.
    declare_parameter("broadcast_tf", false);
    declare_parameter("imu_topic", "mti100/data");
    declare_parameter("track_odom_topic", "mtt_odometry");
    declare_parameter("gps_fix_topic", "/gps_front/fix");
    declare_parameter("gps_heading_topic", "gps/heading");
    declare_parameter("lidar_odom_topic", "mapping/icp_odom");
    declare_parameter("visual_odom_topic", "zed/zed_node/odom");
    // Empty (default): status gate disabled -- the ZED path has no such
    // topic. NOTE: this must be std_msgs/UInt8 (a plain vo_state byte), NOT
    // isaac_ros_visual_slam_interfaces/VisualSlamStatus directly -- this
    // node is built and run from mtt_workspace:devel
    // (demos/live_robot/compose.yaml localization service), which never has
    // isaac_ros_visual_slam_interfaces installed (only the separate
    // mtt_workspace:isaac sidecar image does, docker/Dockerfile.isaac).
    // Wiring the real Isaac status topic here requires a small republishing
    // bridge running inside the isaac_vslam container (same pattern as
    // scripts/isaac_vslam_frame_bridge.py) that extracts vo_state and
    // republishes it as std_msgs/UInt8 -- not yet written; do this before
    // Stage 5 (switching visual_odom_topic to Isaac).
    declare_parameter("visual_status_topic", "");
    // A status message older than this is treated as stale -> gate fails
    // closed (measurements dropped) rather than trusting a frozen "Success".
    declare_parameter("visual_status_timeout_seconds", 0.5);
    // Kinematic rejection bounds for the visual delta between consecutive
    // messages, applied to the BASE-frame (post-extrinsic-conjugation)
    // translation/rotation. A relocalization jump (isaac_vslam_vio.yaml has
    // enable_localization_n_mapping: true) produces a huge, physically
    // implausible delta that the last/pending differencing scheme would
    // otherwise turn into one bogus BetweenFactor -- Huber softens but does
    // not immunise against this. Defaults are generous for this vehicle
    // (tracked, articulated, ice/snow terrain) -- loose enough to never
    // reject real motion, tight enough to catch a relocalization snap.
    declare_parameter("visual_max_speed_mps", 4.0);
    declare_parameter("visual_max_yaw_rate_rps", 3.0);
    declare_parameter("visual_kinematic_margin", 2.0);
    declare_parameter("articulation_topic", "/mtt/articulation_state");
    declare_parameter("trailer_pose_topic", "/trailer/pose");
    declare_parameter("trailer_confidence_topic", "/trailer/pose_confidence");
    declare_parameter("publish_rate", 50.0);
    declare_parameter("min_imu_samples_per_update", 10);
    declare_parameter("extract_covariance", true);

    declare_parameter("isam2_relinearize_threshold", 0.1);
    declare_parameter("isam2_relinearize_skip", 10);
    // Fixed-lag smoother: keys older than this are marginalized. Must exceed the
    // largest expected sensor latency (ICP/ZED/VSLAM odom) with margin.
    declare_parameter("smoother_lag_seconds", 3.0);
    // Warn if a single update() call takes longer than this fraction of the tick
    // period — a cheap signal for whether the optimizer needs to move off the
    // subscription-callback mutex onto a dedicated thread (not done yet; see plan).
    declare_parameter("update_duration_warn_ratio", 0.5);

    // IMU noise
    declare_parameter("imu_accel_noise", 0.01);
    declare_parameter("imu_gyro_noise", 0.0001);
    declare_parameter("imu_accel_walk", 0.0001);
    declare_parameter("imu_gyro_walk", 1e-6);
    // Odometry noise
    declare_parameter("odom_linear_noise", 0.1);
    declare_parameter("odom_angular_noise", 0.05);
    // Not yet consumed by the track-odom BetweenFactor (see :~844, which
    // still applies odom_linear_noise to both x and y) -- declared now so
    // it is visible/tunable ahead of the day mtt_odometry carries a real
    // lateral-slip term. Defaults equal to odom_linear_noise: zero
    // behaviour change until that day and until the factor is updated to
    // use it.
    declare_parameter("odom_lateral_noise", 0.1);
    // LiDAR/visual odometry relative-pose noise, GTSAM Pose3 ordering
    // (rx, ry, rz, tx, ty, tz) -- rad, rad, rad, m, m, m. Previously
    // hardcoded gtsam::Vector6 members in state.hpp (no YAML/CLI override,
    // recompile required to tune); defaults below are unchanged from there.
    declare_parameter("lidar_odom_noise",
        std::vector<double>{0.05, 0.05, 0.05, 0.01, 0.01, 0.01});
    declare_parameter("visual_odom_noise",
        std::vector<double>{0.10, 0.10, 0.10, 0.02, 0.02, 0.02});
    // GPS noise
    declare_parameter("gps_position_noise_xy", 1.0);
    declare_parameter("gps_position_noise_z", 2.0);
    declare_parameter("gps_heading_noise", 0.05);
    // GPS origin
    declare_parameter("gps_origin_lat", 0.0);
    declare_parameter("gps_origin_lon", 0.0);
    declare_parameter("gps_origin_alt", 0.0);
    // Articulation yaw noise
    declare_parameter("phi_sigma_hardware", 0.008);    // rad — encoder fresh
    declare_parameter("phi_sigma_model", 0.035);       // rad — model/stale
    declare_parameter("phi_sigma_dynamics", 0.015);    // rad — random-walk σ per keyframe
    declare_parameter("phi_prior_sigma", 0.5);         // rad — initial prior on φ
    // Articulation pitch noise (P(k) state)
    declare_parameter("pitch_sigma_hardware", 0.020);  // rad — pitch potentiometer (ADC1 8-bit)
    declare_parameter("pitch_sigma_timon", 0.060);     // rad — 3D line ACP on timon (future)
    declare_parameter("pitch_sigma_dynamics", 0.008);  // rad — pitch changes slowly between KFs
    declare_parameter("pitch_prior_sigma", 0.20);      // rad — flat-terrain prior at k=0
    // Trailer LiDAR factor noise
    declare_parameter("trailer_sigma_rot", 0.04);      // rad  base rotation noise
    declare_parameter("trailer_sigma_trans", 0.10);    // m    base translation noise
    declare_parameter("trailer_min_confidence", 0.20); // [0,1] skip below this
  }

  void load_parameters() {
    use_imu_ = get_parameter("use_imu_primary").as_bool();
    use_odom_ = get_parameter("use_track_odom").as_bool();
    use_gps_ = get_parameter("use_gps").as_bool();
    use_gps_heading_ = get_parameter("use_gps_heading").as_bool();
    use_map_anchor_ = get_parameter("use_map_anchor").as_bool();
    map_anchor_topic_ = get_parameter("map_anchor_topic").as_string();
    if (use_gps_ && use_map_anchor_) {
      throw std::runtime_error(
          "use_gps and use_map_anchor are mutually exclusive: both add an "
          "absolute PriorFactor on X(k) anchoring \"map\" to a DIFFERENT "
          "origin (GPS/ENU vs the ICP mapper's own frame). Running both "
          "fights the graph over what \"map\" means. Pick one: use_gps for "
          "GPS-anchored/cross-session sessions, use_map_anchor for GPS-less "
          "sessions where /localization/odom must share the mapper's frame "
          "(e.g. for WILN LiDAR teach & repeat).");
    }
    use_lidar_odom_ = get_parameter("use_lidar_odom").as_bool();
    use_visual_odom_ = get_parameter("use_visual_odom").as_bool();
    use_articulation_ = get_parameter("use_articulation").as_bool();
    use_trailer_pose_ = get_parameter("use_trailer_pose").as_bool();
    use_pitch_state_  = get_parameter("use_pitch_state").as_bool();
    extract_covariance_ = get_parameter("extract_covariance").as_bool();

    map_frame_ = get_parameter("map_frame").as_string();
    odom_frame_ = get_parameter("odom_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    broadcast_tf_ = get_parameter("broadcast_tf").as_bool();
    imu_topic_ = get_parameter("imu_topic").as_string();
    track_odom_topic_ = get_parameter("track_odom_topic").as_string();
    gps_fix_topic_ = get_parameter("gps_fix_topic").as_string();
    gps_heading_topic_ = get_parameter("gps_heading_topic").as_string();
    lidar_odom_topic_ = get_parameter("lidar_odom_topic").as_string();
    visual_odom_topic_ = get_parameter("visual_odom_topic").as_string();
    visual_status_topic_ = get_parameter("visual_status_topic").as_string();
    visual_status_timeout_seconds_ =
        get_parameter("visual_status_timeout_seconds").as_double();
    visual_max_speed_mps_ = get_parameter("visual_max_speed_mps").as_double();
    visual_max_yaw_rate_rps_ = get_parameter("visual_max_yaw_rate_rps").as_double();
    visual_kinematic_margin_ = get_parameter("visual_kinematic_margin").as_double();
    articulation_topic_ = get_parameter("articulation_topic").as_string();
    trailer_pose_topic_ = get_parameter("trailer_pose_topic").as_string();
    trailer_confidence_topic_ = get_parameter("trailer_confidence_topic").as_string();
    min_imu_samples_per_update_ =
        static_cast<int>(get_parameter("min_imu_samples_per_update").as_int());

    noise_.accel_noise_density = get_parameter("imu_accel_noise").as_double();
    noise_.gyro_noise_density = get_parameter("imu_gyro_noise").as_double();
    noise_.accel_random_walk = get_parameter("imu_accel_walk").as_double();
    noise_.gyro_random_walk = get_parameter("imu_gyro_walk").as_double();
    noise_.odom_linear_noise = get_parameter("odom_linear_noise").as_double();
    noise_.odom_angular_noise = get_parameter("odom_angular_noise").as_double();
    odom_lateral_noise_ = get_parameter("odom_lateral_noise").as_double();
    noise_.lidar_odom_noise = loadNoise6("lidar_odom_noise");
    noise_.visual_odom_noise = loadNoise6("visual_odom_noise");
    noise_.gps_position_noise_xy = get_parameter("gps_position_noise_xy").as_double();
    noise_.gps_position_noise_z = get_parameter("gps_position_noise_z").as_double();
    noise_.gps_heading_noise = get_parameter("gps_heading_noise").as_double();
    gps_origin_lat_ = get_parameter("gps_origin_lat").as_double();
    gps_origin_lon_ = get_parameter("gps_origin_lon").as_double();
    gps_origin_alt_ = get_parameter("gps_origin_alt").as_double();
    noise_.phi_sigma_hardware = get_parameter("phi_sigma_hardware").as_double();
    noise_.phi_sigma_model = get_parameter("phi_sigma_model").as_double();
    noise_.phi_sigma_dynamics = get_parameter("phi_sigma_dynamics").as_double();
    noise_.phi_prior_sigma = get_parameter("phi_prior_sigma").as_double();
    noise_.pitch_sigma_hardware = get_parameter("pitch_sigma_hardware").as_double();
    noise_.pitch_sigma_timon = get_parameter("pitch_sigma_timon").as_double();
    noise_.pitch_sigma_dynamics = get_parameter("pitch_sigma_dynamics").as_double();
    noise_.pitch_prior_sigma = get_parameter("pitch_prior_sigma").as_double();
    noise_.trailer_sigma_rot = get_parameter("trailer_sigma_rot").as_double();
    noise_.trailer_sigma_trans = get_parameter("trailer_sigma_trans").as_double();
    trailer_min_confidence_ = get_parameter("trailer_min_confidence").as_double();
    smoother_lag_seconds_ = get_parameter("smoother_lag_seconds").as_double();
    update_duration_warn_ratio_ = get_parameter("update_duration_warn_ratio").as_double();
  }

  // ─── Fixed-lag smoother setup ──
  gtsam::ISAM2Params makeIsam2Params() const {
    gtsam::ISAM2Params params;
    params.relinearizeThreshold =
        get_parameter("isam2_relinearize_threshold").as_double();
    params.relinearizeSkip =
        static_cast<int>(get_parameter("isam2_relinearize_skip").as_int());
    return params;
  }

  // Adds priors + initial values for key 0 (X/V/B, and H/P if articulation is
  // enabled) into graph_/initial_values_, anchored at the given state. Used both
  // at startup (identity/zero) and by resetSmootherAfterFailure() (last known
  // good state, so recovery doesn't teleport the published pose back to origin).
  void seedKeyZero(
      const gtsam::Pose3 & pose, const gtsam::Vector3 & vel,
      const gtsam::imuBias::ConstantBias & bias, double phi, double alpha) {
    auto pose_noise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector6() << 0.1, 0.1, 0.1, 0.5, 0.5, 0.5).finished());
    auto vel_noise = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
    auto bias_noise = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3);

    graph_.addPrior(X(0), pose, pose_noise);
    graph_.addPrior(V(0), vel, vel_noise);
    graph_.addPrior(B(0), bias, bias_noise);
    initial_values_.insert(X(0), pose);
    initial_values_.insert(V(0), vel);
    initial_values_.insert(B(0), bias);

    if (use_articulation_) {
      auto phi_prior_noise = gtsam::noiseModel::Isotropic::Sigma(
          1, noise_.phi_prior_sigma);
      graph_.addPrior(H_key(0), phi, phi_prior_noise);
      initial_values_.insert(H_key(0), phi);
      current_state_.trailer_angle = phi;

      if (use_pitch_state_) {
        auto alpha_prior_noise = gtsam::noiseModel::Isotropic::Sigma(
            1, noise_.pitch_prior_sigma);
        graph_.addPrior(P_key(0), alpha, alpha_prior_noise);
        initial_values_.insert(P_key(0), alpha);
        current_alpha_ = alpha;
      }
    }

    current_state_.pose = pose;
    current_state_.velocity = vel;
    current_state_.imu_bias = bias;
    current_state_.key_index = 0;
  }

  // Recreates the smoother from scratch after an unrecoverable GTSAM exception
  // (ISAM2/fixed-lag internal state is not transactional — once an update()
  // call throws partway through, the Bayes tree is left inconsistent and every
  // subsequent update() fails the same way forever, which is what "Smoother
  // update failed" spamming every tick means). Re-seeds key 0 at the LAST
  // published state (not identity) so map→odom does not jump on recovery —
  // only the smoother's internal history is lost, not the live output pose.
  // `stamp`: the current tick's own timestamp (caller already computed one) —
  // reused as key 0's anchor so the new key 0 is immediately newer than nothing
  // and never looks stale to the smoother.
  void resetSmootherAfterFailure(double stamp) {
    // Last-resort guard: if current_state_ is ITSELF already non-finite or
    // implausible (e.g. a corrupted state from before isSaneNavState() was
    // added, or any future bug that still slips a bad value through), reseed
    // from identity instead of re-injecting the poison. Confirmed necessary
    // on the live robot 2026-07-28: without this, resetSmootherAfterFailure()
    // reseeding from an already-corrupted current_state_ is EXACTLY what made
    // 5310+ resets over ~10 minutes never self-heal -- the isSaneNavState()
    // check at the update() call site only stops NEW corruption from being
    // accepted, it does nothing for state that is already bad by the time
    // this function runs.
    if (!isSaneNavState(current_state_.pose, current_state_.velocity, current_state_.imu_bias)) {
      RCLCPP_ERROR(get_logger(),
          "current_state_ is non-finite/implausible (|t|=%.3g) going into "
          "reset -- falling back to identity instead of re-poisoning the "
          "smoother with it. map->odom WILL jump this one time; that is "
          "correct, the pre-jump value was already garbage.",
          current_state_.pose.translation().norm());
      current_state_.pose = gtsam::Pose3::Identity();
      current_state_.velocity = gtsam::Vector3::Zero();
      current_state_.imu_bias = gtsam::imuBias::ConstantBias();
      current_state_.trailer_angle = 0.0;
      current_alpha_ = 0.0;
    }
    RCLCPP_ERROR(get_logger(),
        "Resetting smoother from scratch, re-anchored at the last published "
        "state (pose/vel/bias/φ/α). Smoother history within the lag window is "
        "lost; the published pose itself does not jump.");
    graph_.resize(0);
    initial_values_.clear();
    smoother_ = std::make_unique<gtsam::IncrementalFixedLagSmoother>(
        smoother_lag_seconds_, makeIsam2Params());
    stamp_to_key_.clear();
    key0_timestamped_ = false;
    has_covariance_ = false;
    seedKeyZero(current_state_.pose, current_state_.velocity,
        current_state_.imu_bias, current_state_.trailer_angle, current_alpha_);

    // Flush key 0 into the smoother RIGHT NOW, inside this function — the
    // caller (optimize_and_publish's catch block) is followed unconditionally
    // by graph_.resize(0)/initial_values_.clear() at the end of every tick,
    // which would otherwise wipe this seed before it ever reached the smoother
    // (exactly what caused every reset to immediately fail again with
    // "key b0 does not exist in the Values" — the CombinedImuFactor at the
    // next tick referenced a key 0 that was queued but never actually inserted).
    gtsam::FixedLagSmoother::KeyTimestampMap key0_timestamps;
    key0_timestamps[X(0)] = stamp;
    key0_timestamps[V(0)] = stamp;
    key0_timestamps[B(0)] = stamp;
    if (use_articulation_) {
      key0_timestamps[H_key(0)] = stamp;
      if (use_pitch_state_) key0_timestamps[P_key(0)] = stamp;
    }
    try {
      smoother_->update(graph_, initial_values_, key0_timestamps);
      stamp_to_key_[stamp] = 0;
      key0_timestamped_ = true;
    } catch (const std::exception & e) {
      // A fresh smoother failing on a single prior-only key 0 would indicate a
      // deeper problem (bad noise model, NaN in current_state_, ...) that a
      // reset loop cannot fix. Log loudly; key0_timestamped_ stays false so the
      // NEXT tick's catch block will try resetSmootherAfterFailure() again
      // rather than silently running with an inconsistent key0_timestamped_/
      // smoother state.
      RCLCPP_ERROR(get_logger(),
          "Smoother reset itself failed on key 0: %s — will retry next tick.",
          e.what());
    }
    graph_.resize(0);
    initial_values_.clear();
  }

  void setup_smoother() {
    smoother_ = std::make_unique<gtsam::IncrementalFixedLagSmoother>(
        smoother_lag_seconds_, makeIsam2Params());
    seedKeyZero(gtsam::Pose3::Identity(), gtsam::Vector3::Zero(),
        gtsam::imuBias::ConstantBias(), 0.0, 0.0);

    RCLCPP_INFO(get_logger(), "Smoother initialised (tractor SE(3) + articulation φ%s)",
        (use_articulation_ && use_pitch_state_) ? "+α" : "");
  }

  void setup_imu_preintegration() {
    auto p = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(9.81);
    p->accelerometerCovariance =
        gtsam::I_3x3 * std::pow(noise_.accel_noise_density, 2);
    p->gyroscopeCovariance =
        gtsam::I_3x3 * std::pow(noise_.gyro_noise_density, 2);
    p->biasAccCovariance =
        gtsam::I_3x3 * std::pow(noise_.accel_random_walk, 2);
    p->biasOmegaCovariance =
        gtsam::I_3x3 * std::pow(noise_.gyro_random_walk, 2);
    p->integrationCovariance = gtsam::I_3x3 * 1e-8;

    imu_preint_ = std::make_unique<gtsam::PreintegratedCombinedMeasurements>(
        p, current_state_.imu_bias);
  }

  // ─── Publishers ──
  void setup_publishers() {
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("localization/odom", 10);
    // Separate topic, NOT a replacement for localization/odom (consumed by
    // mtt_gps_recorder_node.cpp:17, mtt_gps_path_follower_node.cpp:32,
    // mtt_gps_path_server_node.cpp:31, trailer_localizer_node.cpp:247 --
    // changing that contract would ripple). This is a higher-rate (~IMU
    // rate, up to 100Hz) but lower-fidelity companion: published from
    // imu_callback() by propagating the LAST optimised keyframe through the
    // preintegrator's current (not-yet-committed) state, for consumers that
    // need a fresher pose than the ~10Hz effective optimize_and_publish()
    // tick (min_imu_samples_per_update) can give -- e.g. a controller. See
    // publish_odom_fast().
    odom_fast_pub_ = create_publisher<nav_msgs::msg::Odometry>("localization/odom_fast", 10);
    articulation_pub_ = create_publisher<std_msgs::msg::Float64>(
        "localization/articulation_angle", 10);
    if (use_pitch_state_) {
      pitch_pub_ = create_publisher<std_msgs::msg::Float64>(
          "localization/articulation_pitch", 10);
    }
    // Innovation diagnostics (measurement vs pre-update predicted state, whitened
    // by the noise sigma actually assigned to that factor). This is NOT a full
    // NEES/NIS against the smoother's joint covariance (querying ISAM2 marginals
    // every tick is expensive); it is a cheap per-measurement z-score that still
    // catches the class of bug this was added for — a noise model silently
    // trusting a degraded measurement (see docs/roadmap/POSE_ESTIMATION_PLAN.md P2.1).
    // Layout (22 elements as of 2026-07-26, was 14 -- breaking change, grep
    // "innovation_diagnostics" before adding a new field: any Foxglove panel
    // indexing this array by position needs updating):
    //   [gps_valid, gps_innov_xy_m, gps_zscore_xy,
    //    art_valid, art_innov_rad, art_zscore,
    //    lidar_odom_valid, lidar_odom_innov_trans_m, lidar_odom_innov_rot_rad, lidar_odom_zscore,
    //    trailer_valid, trailer_innov_trans_m, trailer_innov_rot_rad, trailer_zscore,
    //    visual_odom_valid, visual_odom_innov_trans_m, visual_odom_innov_rot_rad, visual_odom_zscore,
    //    map_anchor_valid, map_anchor_innov_trans_m, map_anchor_innov_rot_rad, map_anchor_zscore]
    // lidar_odom/trailer/visual_odom/map_anchor channels are Lie-algebra
    // residuals (Pose3::Logmap), valid only when the factor was actually
    // added this tick against a clean (no-OOSM-lag) prev/curr keyframe pair
    // — see the comments at each BetweenFactor/PriorFactor site for the
    // exact formula. gps_valid and map_anchor_valid are never both 1 in the
    // same run (use_gps/use_map_anchor are mutually exclusive, enforced at
    // startup).
    innovation_diag_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "localization/factor_graph/innovation_diagnostics", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // For rotating raw IMU accel/gyro from their sensor frame (msg header
    // frame_id, e.g. imu_link) into base_frame_ before preintegration — see
    // imu_callback(). The IMU is very often NOT mounted Z-up/axis-aligned with
    // the body frame (this robot's imu_link is rolled 180° + yawed 90° per
    // calib_v2.xacro); integrating raw sensor-frame accel/gyro as if they were
    // already in the body frame silently inverts gravity for the preintegrator,
    // which diverges within seconds and was the root cause of persistent
    // IndeterminantLinearSystemException crashes on X/V/B regardless of
    // GPS/ICP/track-odom configuration.
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  // ─── Subscribers ──
  void setup_subscribers() {
    if (use_imu_) {
      imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
          imu_topic_, rclcpp::SensorDataQoS(),
          std::bind(&FactorGraphNode::imu_callback, this, std::placeholders::_1));
    }
    if (use_odom_) {
      odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
          track_odom_topic_, 10,
          std::bind(&FactorGraphNode::odom_callback, this, std::placeholders::_1));
    }
    if (use_gps_) {
      gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
          gps_fix_topic_, 10,
          std::bind(&FactorGraphNode::gps_callback, this, std::placeholders::_1));
    }
    if (use_map_anchor_) {
      map_anchor_sub_ = create_subscription<nav_msgs::msg::Odometry>(
          map_anchor_topic_, 10,
          std::bind(&FactorGraphNode::map_anchor_callback, this, std::placeholders::_1));
    }
    if (use_gps_heading_) {
      heading_sub_ = create_subscription<geometry_msgs::msg::QuaternionStamped>(
          gps_heading_topic_, 10,
          std::bind(&FactorGraphNode::heading_callback, this, std::placeholders::_1));
    }
    if (use_lidar_odom_) {
      lidar_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
          lidar_odom_topic_, 10,
          std::bind(&FactorGraphNode::lidar_odom_callback, this, std::placeholders::_1));
    }
    if (use_visual_odom_) {
      visual_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
          visual_odom_topic_, 10,
          std::bind(&FactorGraphNode::visual_odom_callback, this, std::placeholders::_1));
      if (!visual_status_topic_.empty()) {
        visual_status_sub_ = create_subscription<std_msgs::msg::UInt8>(
            visual_status_topic_, rclcpp::SensorDataQoS(),
            std::bind(&FactorGraphNode::visual_status_callback, this, std::placeholders::_1));
      }
    }
    if (use_articulation_) {
      articulation_sub_ = create_subscription<mtt_msgs::msg::MttArticulationState>(
          articulation_topic_, rclcpp::SensorDataQoS(),
          std::bind(&FactorGraphNode::articulation_callback, this, std::placeholders::_1));
    }
    if (use_articulation_ && use_trailer_pose_) {
      trailer_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
          trailer_pose_topic_, rclcpp::SensorDataQoS(),
          std::bind(&FactorGraphNode::trailer_pose_callback, this, std::placeholders::_1));
      trailer_confidence_sub_ = create_subscription<std_msgs::msg::Float64>(
          trailer_confidence_topic_, rclcpp::SensorDataQoS(),
          std::bind(&FactorGraphNode::trailer_confidence_callback, this, std::placeholders::_1));
    }

    RCLCPP_INFO(get_logger(),
        "Subscriptions: imu=%d odom=%d gps=%d heading=%d lidar_odom=%d "
        "visual_odom=%d map_anchor=%d articulation=%d trailer_pose=%d",
        use_imu_, use_odom_, use_gps_, use_gps_heading_, use_lidar_odom_,
        use_visual_odom_, use_map_anchor_,
        use_articulation_, use_articulation_ && use_trailer_pose_);
    if (use_map_anchor_) {
      RCLCPP_INFO(get_logger(),
          "map frame is anchored to the ICP mapper's frame via %s -- NOT "
          "GPS/ENU. /localization/odom is session-relative, not "
          "geo-referenced, while this is active.",
          map_anchor_topic_.c_str());
    }
  }

  // ─── Sensor callbacks ──
  // Lazily looks up the static rotation from the IMU's own sensor frame
  // (msg header frame_id) into base_frame_, caching it after the first success
  // (mirrors norlab_icp_mapper_ros's tryLookupImuSensorExtrinsic pattern — the
  // TF may not be published yet on the very first few callbacks at startup).
  bool ensureImuExtrinsic(const std::string & imu_frame) {
    if (imu_extrinsic_ready_) return true;
    try {
      const auto tf = tf_buffer_->lookupTransform(base_frame_, imu_frame, rclcpp::Time(0));
      const auto & q = tf.transform.rotation;
      R_base_imu_ = gtsam::Rot3::Quaternion(q.w, q.x, q.y, q.z);
      imu_extrinsic_ready_ = true;
      RCLCPP_INFO(get_logger(), "IMU extrinsic %s → %s acquired for preintegration.",
          imu_frame.c_str(), base_frame_.c_str());
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "IMU extrinsic %s → %s not yet available: %s — dropping IMU samples "
          "until TF is up (robot_state_publisher/description).",
          imu_frame.c_str(), base_frame_.c_str(), ex.what());
      return false;
    }
  }

  // Lazily looks up and caches the static rigid extrinsic base_frame_ ->
  // sensor_frame (a full Pose3, unlike ensureImuExtrinsic's rotation-only
  // R_base_imu_), for conjugating a relative-pose measurement expressed in
  // sensor_frame into base_frame_ before it is used as a BetweenFactor on
  // X(k) (which are base_frame_ poses). Needed because Isaac VSLAM/ZED
  // publish the pose of zed_camera_link, not base_frame_ -- differencing
  // two zed_camera_link poses directly and applying that delta to X(k)
  // ignores the lever arm between them (~0.605m on this robot via the
  // hesai_lidar chain), which is fine on straights but introduces a
  // heading-correlated translation error of several cm on every pivot
  // (confirmed by hand from the URDF chain, 2026-07-26 — see
  // visual_odom_callback for the conjugation itself).
  // Same caching pattern as ensureImuExtrinsic: try every call until the
  // first success (TF may not be up yet at startup), then reuse forever
  // (extrinsics are rigid, not expected to change at runtime).
  bool ensureSensorExtrinsic(const std::string & sensor_frame, gtsam::Pose3 & T_base_sensor) {
    const auto cached = sensor_extrinsics_.find(sensor_frame);
    if (cached != sensor_extrinsics_.end()) {
      T_base_sensor = cached->second;
      return true;
    }
    if (sensor_frame == base_frame_) {
      // Degenerate but valid: a source already publishing base_frame_
      // directly needs no conjugation. Cache identity so this branch is
      // only ever taken once per frame name.
      T_base_sensor = gtsam::Pose3::Identity();
      sensor_extrinsics_[sensor_frame] = T_base_sensor;
      return true;
    }
    try {
      const auto tf = tf_buffer_->lookupTransform(base_frame_, sensor_frame, rclcpp::Time(0));
      const auto & t = tf.transform.translation;
      const auto & q = tf.transform.rotation;
      T_base_sensor = gtsam::Pose3(
          gtsam::Rot3::Quaternion(q.w, q.x, q.y, q.z), gtsam::Point3(t.x, t.y, t.z));
      sensor_extrinsics_[sensor_frame] = T_base_sensor;
      RCLCPP_INFO(get_logger(), "Sensor extrinsic %s → %s acquired.",
          base_frame_.c_str(), sensor_frame.c_str());
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "Sensor extrinsic %s → %s not yet available: %s — dropping "
          "measurements from this source until TF is up.",
          base_frame_.c_str(), sensor_frame.c_str(), ex.what());
      return false;
    }
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!ensureImuExtrinsic(msg->header.frame_id)) return;

    double t = rclcpp::Time(msg->header.stamp).seconds();
    if (last_imu_t_ < 0) { last_imu_t_ = t; return; }
    double dt = t - last_imu_t_;
    if (dt <= 0 || dt > 1.0) { last_imu_t_ = t; return; }

    // Raw accel/gyro are in the IMU's own sensor frame, which is frequently NOT
    // axis-aligned with the body frame (e.g. this robot's imu_link is rolled
    // 180° + yawed 90° relative to base_footprint, per calib_v2.xacro) — rotate
    // into base_frame_ before preintegration, or gravity/rotation axes are wrong
    // and the estimate diverges within seconds.
    const gtsam::Vector3 acc_raw(msg->linear_acceleration.x,
                       msg->linear_acceleration.y,
                       msg->linear_acceleration.z);
    const gtsam::Vector3 gyro_raw(msg->angular_velocity.x,
                        msg->angular_velocity.y,
                        msg->angular_velocity.z);
    const gtsam::Vector3 acc = R_base_imu_.rotate(acc_raw);
    const gtsam::Vector3 gyro = R_base_imu_.rotate(gyro_raw);
    imu_preint_->integrateMeasurement(acc, gyro, dt);
    last_imu_t_ = t;
    imu_data_count_++;
    publish_odom_fast(msg->header.stamp, t);
  }

  // Publishes localization/odom_fast at IMU rate (~100Hz): propagates the
  // LAST optimised keyframe (current_state_) through the preintegrator's
  // CURRENT, not-yet-committed state (imu_preint_, which accumulates
  // between keyframes and resets in optimize_and_publish() after each
  // commit) -- the same predict() call optimize_and_publish() itself uses
  // to seed initial_values_ for the next keyframe, just called every IMU
  // sample instead of once every ~10 samples. Caller (imu_callback) already
  // holds mtx_.
  void publish_odom_fast(const builtin_interfaces::msg::Time & stamp, double now_s) {
    const auto predicted = imu_preint_->predict(
        gtsam::NavState(current_state_.pose, current_state_.velocity),
        current_state_.imu_bias);

    nav_msgs::msg::Odometry msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = map_frame_;
    msg.child_frame_id = base_frame_;

    const auto & pt = predicted.pose().translation();
    const auto pq = predicted.pose().rotation().toQuaternion();
    msg.pose.pose.position.x = pt.x();
    msg.pose.pose.position.y = pt.y();
    msg.pose.pose.position.z = pt.z();
    msg.pose.pose.orientation.x = pq.x();
    msg.pose.pose.orientation.y = pq.y();
    msg.pose.pose.orientation.z = pq.z();
    msg.pose.pose.orientation.w = pq.w();

    const auto & v = predicted.velocity();
    msg.twist.twist.linear.x = v.x();
    msg.twist.twist.linear.y = v.y();
    msg.twist.twist.linear.z = v.z();

    // NOT a rigorous propagation of the smoother's marginal (that needs the
    // preintegration Jacobians, not just the noise density) -- a
    // conservative inflation of the last keyframe's real marginal, growing
    // with elapsed time since that keyframe. Documented as an
    // approximation deliberately, not presented as exact: a consumer that
    // needs calibrated covariance should read localization/odom (the
    // ~10Hz, ISAM2-marginal-backed topic) instead.
    if (has_covariance_) {
      const double elapsed_s = std::max(0.0, now_s - current_state_.timestamp);
      const double inflate = 1.0 + noise_.accel_noise_density * elapsed_s * elapsed_s;
      constexpr int perm[6] = {3, 4, 5, 0, 1, 2};  // GTSAM [rx,ry,rz,tx,ty,tz] -> ROS [x,y,z,rx,ry,rz]
      for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
          msg.pose.covariance[i * 6 + j] = cov_pose_(perm[i], perm[j]) * inflate;
        }
      }
    }

    odom_fast_pub_->publish(msg);
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    latest_odom_ = *msg;
    has_latest_odom_ = true;
    if (!has_last_odom_) { last_odom_ = *msg; has_last_odom_ = true; return; }
    pending_odom_ = *msg;
    has_pending_odom_ = true;
  }

  void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    if (msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) return;
    std::lock_guard<std::mutex> lock(mtx_);
    if (!gps_origin_set_) {
      // A configured horizontal origin keeps map X/Y stable across restarts,
      // which is required for persisted GPS Teach routes. Zero means
      // "initialize from the first fix" for backward compatibility. Altitude
      // may remain 0 in the field config until it is surveyed, so use the
      // first valid fix for Z in that case.
      if (std::abs(gps_origin_lat_) < 1e-9 && std::abs(gps_origin_lon_) < 1e-9) {
        gps_origin_lat_ = msg->latitude;
        gps_origin_lon_ = msg->longitude;
      }
      if (std::abs(gps_origin_alt_) < 1e-9) {
        gps_origin_alt_ = msg->altitude;
      }
      gps_origin_set_ = true;
      RCLCPP_INFO(get_logger(), "GPS origin set: lat=%.7f lon=%.7f alt=%.2f",
                  gps_origin_lat_, gps_origin_lon_, gps_origin_alt_);
    }
    pending_gps_ = *msg;
    has_pending_gps_ = true;
  }

  // /mapping/icp_measurement -- unlike /mapping/icp_odom, this is NEVER
  // dead-reckoned (mapper_node's localizationOnlyOdomBridge skips it), so
  // every message here is a genuine accepted ICP registration with real
  // covariance. Buffered like GPS (single pending slot, no last/pending
  // differencing -- this is an absolute prior, not a relative delta).
  void map_anchor_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    pending_map_anchor_ = *msg;
    has_pending_map_anchor_ = true;
  }

  void heading_callback(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    pending_heading_ = *msg;
    has_pending_heading_ = true;
  }

  void lidar_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!has_last_lidar_odom_) {
      last_lidar_odom_ = *msg; has_last_lidar_odom_ = true; return;
    }
    pending_lidar_odom_ = *msg;
    has_pending_lidar_odom_ = true;
  }

  // vo_state: 0=Unknown, 1=Success, 2=Failed
  // (isaac_ros_visual_slam_interfaces/VisualSlamStatus, republished as a
  // plain std_msgs/UInt8 -- see the visual_status_topic declare_parameter
  // comment for why this node can't subscribe to the real Isaac message
  // type directly).
  void visual_status_callback(const std_msgs::msg::UInt8::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    visual_status_ok_ = (msg->data == 1);
    visual_status_stamp_ = get_clock()->now();
    has_visual_status_ = true;
  }

  // True if the visual-odom gate currently passes: either no status topic is
  // configured (gate disabled -- the ZED path has none) or the latest status
  // says Success and is fresh. Fails CLOSED if a status topic IS configured
  // but nothing has arrived yet, or the last message is stale -- a dead
  // status topic must never be silently treated as "fine". Caller must hold
  // mtx_.
  bool visualStatusGateOk() const {
    if (visual_status_topic_.empty()) return true;
    if (!has_visual_status_) return false;
    const double age = (get_clock()->now() - visual_status_stamp_).seconds();
    if (age > visual_status_timeout_seconds_) return false;
    return visual_status_ok_;
  }

  void visual_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!visualStatusGateOk()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "Visual odom gate closed (vo_state not Success, or status stale) "
          "-- dropping measurement.");
      return;
    }
    if (!has_last_visual_odom_) {
      last_visual_odom_ = *msg; has_last_visual_odom_ = true; return;
    }

    const double dt = rclcpp::Time(msg->header.stamp).seconds() -
                       rclcpp::Time(last_visual_odom_.header.stamp).seconds();
    if (dt <= 0.0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "Visual odom message not newer than the last one (dt=%.4fs) -- dropped.", dt);
      return;
    }

    // Kinematic jump rejection -- catches a relocalization discontinuity
    // (isaac_vslam_vio.yaml: enable_localization_n_mapping true) before it
    // is ever buffered as a BetweenFactor measurement; Huber (see
    // robustNoise()) softens a single bad factor but does not immunise
    // against one this large. Uses the RAW (sensor-frame, not yet
    // extrinsic-conjugated) delta as a conservative proxy: rotation
    // magnitude is exact regardless of the extrinsic (R_base_sensor is
    // identity on this robot -- see ensureSensorExtrinsic's derivation
    // comment), and translation magnitude is off by at most the lever-arm
    // term (~0.6m on this robot), negligible next to an actual
    // relocalization jump (metres). The PRECISE, extrinsic-conjugated delta
    // is computed later, at the BetweenFactor construction site.
    const gtsam::Pose3 raw_delta =
        odom_to_pose3(last_visual_odom_).between(odom_to_pose3(*msg));
    const gtsam::Vector6 xi = gtsam::Pose3::Logmap(raw_delta);
    const double speed = xi.tail<3>().norm() / dt;
    const double yaw_rate = xi.head<3>().norm() / dt;
    const double max_speed = visual_max_speed_mps_ * visual_kinematic_margin_;
    const double max_yaw_rate = visual_max_yaw_rate_rps_ * visual_kinematic_margin_;
    if (speed > max_speed || yaw_rate > max_yaw_rate) {
      RCLCPP_WARN(get_logger(),
          "Visual odom jump rejected: speed=%.2fm/s (max %.2f) "
          "yaw_rate=%.2frad/s (max %.2f) over dt=%.3fs -- resetting baseline.",
          speed, max_speed, yaw_rate, max_yaw_rate, dt);
      has_last_visual_odom_ = false;  // re-seed fresh on the next message
      has_pending_visual_odom_ = false;
      return;
    }

    pending_visual_odom_ = *msg;
    has_pending_visual_odom_ = true;
  }

  void articulation_callback(
      const mtt_msgs::msg::MttArticulationState::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    pending_articulation_ = *msg;
    has_pending_articulation_ = true;
  }

  void trailer_pose_callback(
      const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    pending_trailer_pose_ = *msg;
    has_pending_trailer_pose_ = true;
  }

  void trailer_confidence_callback(
      const std_msgs::msg::Float64::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    latest_trailer_confidence_ = msg->data;
  }

  // ─── Helpers ──
  gtsam::Point3 gps_to_local(double lat, double lon, double alt) const {
    double lat_ref = gps_origin_lat_ * kDeg2Rad;
    double m_per_deg_lat = kEarthRadius * kDeg2Rad;
    double m_per_deg_lon = kEarthRadius * kDeg2Rad * std::cos(lat_ref);
    return {(lat - gps_origin_lat_) * m_per_deg_lat,
            (lon - gps_origin_lon_) * m_per_deg_lon,
            alt - gps_origin_alt_};
  }

  gtsam::Pose3 odom_to_pose3(const nav_msgs::msg::Odometry & msg) const {
    auto & p = msg.pose.pose.position;
    auto & q = msg.pose.pose.orientation;
    return {gtsam::Rot3::Quaternion(q.w, q.x, q.y, q.z),
            gtsam::Point3(p.x, p.y, p.z)};
  }

  // Rejects a non-finite or physically-implausible state BEFORE it is ever
  // accepted into current_state_ -- confirmed necessary 2026-07-28 on the
  // live robot: a single bad ISAM2 solve (root cause not yet pinned down --
  // leading suspect is an Inf-valued GPS position_covariance element, which
  // slips past the "var > 0.0" guard in the GPS factor block since Inf > 0.0
  // is true) produced a translation on the order of 1e12 m. That value was
  // accepted as current_state_, published on /localization/odom for however
  // long the fault lasted, AND fed straight back into
  // resetSmootherAfterFailure()'s reseed -- so every subsequent "recovery"
  // re-seeded the smoother with the SAME poisoned state and immediately
  // failed again. 5310+ resets over ~10 minutes, never self-healing, exactly
  // matching this function's own prior warning ("a fresh smoother failing on
  // a single prior-only key 0 indicates a deeper problem that a reset loop
  // cannot fix") -- the "deeper problem" was that the loop was reseeding
  // from corrupted data, not that GTSAM itself was broken.
  // kMaxSanePositionM is generous by any real-world margin for this vehicle
  // (never operates more than a few km from its start) -- it exists purely
  // to catch a numerical blowup, not to bound legitimate operation.
  static bool isSaneNavState(const gtsam::Pose3 & pose, const gtsam::Vector3 & vel,
                              const gtsam::imuBias::ConstantBias & bias) {
    constexpr double kMaxSanePositionM = 1.0e6;
    constexpr double kMaxSaneVelocityMps = 100.0;
    constexpr double kMaxSaneBias = 10.0;
    const auto & t = pose.translation();
    if (!t.allFinite() || t.norm() > kMaxSanePositionM) return false;
    if (!pose.rotation().matrix().allFinite()) return false;
    if (!vel.allFinite() || vel.norm() > kMaxSaneVelocityMps) return false;
    const auto bias_vec = bias.vector();
    if (!bias_vec.allFinite() || bias_vec.norm() > kMaxSaneBias) return false;
    return true;
  }

  // Wraps a base noise model in a Huber M-estimator so ONE occasional bad
  // relative-pose measurement (an ICP jump under the still-fragile force4DOF=0
  // mapper config, a track-odom slip spike, a ZED VIO glitch) gets down-weighted
  // instead of dominating/corrupting the linear system. Plain Gaussian
  // BetweenFactors with a tight sigma (e.g. lidar_odom_noise's 1cm translation)
  // treat every discrepancy as equally significant, so a single outlier can — and
  // was observed to — throw IndeterminantLinearSystemException. k=1.345 is the
  // standard Huber tuning for ~95% efficiency under nominal Gaussian noise.
  static gtsam::SharedNoiseModel robustNoise(const gtsam::SharedNoiseModel & base) {
    return gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(1.345), base);
  }

  // Reads a 6-element relative-pose noise sigma param (GTSAM Pose3 ordering:
  // rx,ry,rz,tx,ty,tz -- rad,rad,rad,m,m,m) and validates it strictly: a
  // wrong-length or zero/negative sigma array silently reproduces the
  // IndeterminantLinearSystemException documented above robustNoise(), so
  // this fails loudly at startup instead of at the first bad tick. Logs the
  // assembled vector with the ordering spelled out -- the most likely thing
  // for a future tuner to get backwards.
  gtsam::Vector6 loadNoise6(const std::string & param_name) {
    const auto values = get_parameter(param_name).as_double_array();
    if (values.size() != 6) {
      throw std::runtime_error(
          "Parameter '" + param_name + "' must have exactly 6 elements "
          "(rx,ry,rz,tx,ty,tz), got " + std::to_string(values.size()));
    }
    gtsam::Vector6 v;
    for (int i = 0; i < 6; ++i) {
      if (!(values[i] > 0.0)) {
        throw std::runtime_error(
            "Parameter '" + param_name + "' element " + std::to_string(i) +
            " must be > 0, got " + std::to_string(values[i]));
      }
      v(i) = values[i];
    }
    RCLCPP_INFO(get_logger(),
        "%s = [rx=%.4f ry=%.4f rz=%.4f rad, tx=%.4f ty=%.4f tz=%.4f m]",
        param_name.c_str(), v(0), v(1), v(2), v(3), v(4), v(5));
    return v;
  }

  gtsam::Pose3 stamped_to_pose3(const geometry_msgs::msg::PoseStamped & msg) const {
    auto & p = msg.pose.position;
    auto & q = msg.pose.orientation;
    return {gtsam::Rot3::Quaternion(q.w, q.x, q.y, q.z),
            gtsam::Point3(p.x, p.y, p.z)};
  }

  // ─── OOSM: map a measurement timestamp to the nearest existing keyframe ──
  // Used to attach late-arriving measurements (ICP/ZED/VSLAM odom, GPS, articulation,
  // trailer pose) to the historical keyframe closest to when they were actually
  // captured, instead of always binding to curr_key ("now"). Falls back to the latest
  // known key if stamp_to_key_ is empty (startup) or stamp lies outside the recorded
  // range on either side (clamped to nearest edge).
  uint64_t keyForStamp(double stamp) const {
    if (stamp_to_key_.empty()) return current_state_.key_index;
    auto it = stamp_to_key_.lower_bound(stamp);
    if (it == stamp_to_key_.begin()) return it->second;
    if (it == stamp_to_key_.end()) return std::prev(it)->second;
    const auto prev_it = std::prev(it);
    return (stamp - prev_it->first <= it->first - stamp) ? prev_it->second : it->second;
  }

  // ─── Main optimisation loop ──
  void optimize_and_publish() {
    std::lock_guard<std::mutex> lock(mtx_);

    const bool has_pending =
        has_pending_odom_ || has_pending_gps_ || has_pending_heading_ ||
        has_pending_lidar_odom_ || has_pending_visual_odom_ ||
        has_pending_articulation_ || has_pending_trailer_pose_ ||
        has_pending_map_anchor_;

    if (use_imu_) {
      if (imu_data_count_ < min_imu_samples_per_update_) return;
    } else if (!has_pending) {
      return;
    }

    const uint64_t prev_key = current_state_.key_index;
    const uint64_t curr_key = prev_key + 1;

    // Keyframe timestamp for the fixed-lag smoother's marginalization bookkeeping.
    // MUST NOT be this->now(): opt_timer_ is a wall-clock timer (ticks at real-time
    // 50 Hz regardless of use_sim_time), while this->now() returns SIM time under
    // bag replay — the bag's /clock does not necessarily advance once per tick (it
    // publishes at whatever rate the recorded reference topic had), so two
    // consecutive ticks can read the exact same sim time, or even go briefly
    // backwards. A duplicate/non-monotonic timestamp silently corrupts
    // stamp_to_key_/the smoother's KeyTimestampMap and manifests later as an
    // IndeterminantLinearSystemException once the lag window starts marginalizing
    // (observed: crash at ~154 ticks ≈ 3.0s = smoother_lag_seconds_).
    // Use the last actually-integrated IMU sample's own stamp instead — it's tied
    // to the real sensor timeline (consistent under sim time) and only advances
    // when imu_callback() actually processes a new message.
    double tick_stamp = (use_imu_ && last_imu_t_ >= 0.0) ? last_imu_t_ : this->now().seconds();
    if (!stamp_to_key_.empty()) {
      const double prev_stamp = std::prev(stamp_to_key_.end())->first;
      if (tick_stamp <= prev_stamp) {
        // Defensive monotonic clamp — FixedLagSmoother assumes strictly
        // increasing timestamps per new key.
        tick_stamp = prev_stamp + 1e-3;
      }
    }

    // ── KeyTimestampMap for the fixed-lag smoother ──
    // Only NEW keys inserted into initial_values_ THIS call need an entry — the
    // smoother remembers timestamps for keys from earlier calls internally.
    gtsam::FixedLagSmoother::KeyTimestampMap new_timestamps;
    if (!key0_timestamped_) {
      // First tick: key 0's prior was added in setup_smoother() but never given a
      // timestamp (initial_values_ for key 0 is still sitting unflushed until now).
      new_timestamps[X(0)] = tick_stamp;
      new_timestamps[V(0)] = tick_stamp;
      new_timestamps[B(0)] = tick_stamp;
      if (use_articulation_) {
        new_timestamps[H_key(0)] = tick_stamp;
        if (use_pitch_state_) new_timestamps[P_key(0)] = tick_stamp;
      }
      stamp_to_key_[tick_stamp] = 0;
      key0_timestamped_ = true;
    }
    new_timestamps[X(curr_key)] = tick_stamp;
    new_timestamps[V(curr_key)] = tick_stamp;
    new_timestamps[B(curr_key)] = tick_stamp;

    // ── IMU preintegration factor ──
    if (use_imu_ && imu_data_count_ > 0) {
      graph_.add(gtsam::CombinedImuFactor(
          X(prev_key), V(prev_key), X(curr_key), V(curr_key),
          B(prev_key), B(curr_key), *imu_preint_));
    }

    // Predict tractor state from IMU
    const auto predicted = (use_imu_ && imu_data_count_ > 0)
        ? imu_preint_->predict(
              gtsam::NavState(current_state_.pose, current_state_.velocity),
              current_state_.imu_bias)
        : gtsam::NavState(current_state_.pose, current_state_.velocity);

    initial_values_.insert(X(curr_key), predicted.pose());
    initial_values_.insert(V(curr_key), predicted.velocity());
    initial_values_.insert(B(curr_key), current_state_.imu_bias);

    // Innovation diagnostics — see setup_publishers() for the rationale/layout.
    bool gps_diag_valid = false, art_diag_valid = false;
    bool lidar_odom_diag_valid = false, trailer_diag_valid = false;
    bool visual_odom_diag_valid = false, map_anchor_diag_valid = false;
    double gps_innov_xy_m = 0.0, gps_zscore_xy = 0.0;
    double art_innov_rad = 0.0, art_zscore = 0.0;
    double lidar_odom_innov_trans_m = 0.0, lidar_odom_innov_rot_rad = 0.0, lidar_odom_zscore = 0.0;
    double trailer_innov_trans_m = 0.0, trailer_innov_rot_rad = 0.0, trailer_zscore = 0.0;
    double visual_odom_innov_trans_m = 0.0, visual_odom_innov_rot_rad = 0.0, visual_odom_zscore = 0.0;
    double map_anchor_innov_trans_m = 0.0, map_anchor_innov_rot_rad = 0.0, map_anchor_zscore = 0.0;

    // ── GPS position (OOSM: attach to keyframe nearest the fix's own stamp) ──
    if (has_pending_gps_ && gps_origin_set_) {
      auto local = gps_to_local(pending_gps_.latitude,
                                pending_gps_.longitude,
                                pending_gps_.altitude);
      // A non-finite fix (NaN/Inf lat/lon/alt from a flaky driver, or a
      // degenerate flat-Earth projection) would become the GPSFactor's
      // MEASUREMENT MEAN, not just its noise -- unlike the covariance-only
      // Inf case below, a non-finite mean poisons the linear system
      // directly. Drop the message outright rather than let it reach
      // graph_.add().
      if (!local.allFinite()) {
        RCLCPP_ERROR(get_logger(),
            "Dropping GPS fix: non-finite local position from lat=%.7f "
            "lon=%.7f alt=%.2f -- check the GPS driver.",
            pending_gps_.latitude, pending_gps_.longitude, pending_gps_.altitude);
        has_pending_gps_ = false;
      } else {
      // Prefer the driver's own position_covariance over the status enum: the
      // GPS driver maps both RTK Fix (~2 cm) and RTK Float (~10-15 cm) to
      // STATUS_GBAS_FIX, so the enum alone cannot tell them apart and a binary
      // branch on it would trust Float as if it were Fix. Same reasoning as
      // mtt_gps_teach_repeat/logic/gps_fix_quality.hpp, applied here.
      double noise_xy = noise_.gps_position_noise_xy;
      double noise_z  = noise_.gps_position_noise_z;
      if (pending_gps_.position_covariance_type !=
          sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
        const double var_x = pending_gps_.position_covariance[0];
        const double var_y = pending_gps_.position_covariance[4];
        const double var_z = pending_gps_.position_covariance[8];
        // "> 0.0" alone lets +Inf through (Inf > 0.0 is true) -- a driver
        // reporting Inf for "unknown/very poor" quality would give
        // sqrt(Inf)=Inf here, which std::max floors harmlessly (1/Inf=0
        // in the resulting noise model, i.e. this measurement contributes
        // no information -- not itself dangerous), but is still the wrong
        // semantics for what this block claims to check ("finite, sane
        // covariance") and worth excluding explicitly rather than relying
        // on downstream floors to save it.
        if (std::isfinite(var_x) && std::isfinite(var_y) && std::isfinite(var_z) &&
            var_x > 0.0 && var_y > 0.0 && var_z > 0.0) {
          constexpr double kMinSigmaXy = 0.01;  // m — floor vs implausibly tight covariance
          constexpr double kMinSigmaZ  = 0.02;  // m
          noise_xy = std::max(std::sqrt(std::max(var_x, var_y)), kMinSigmaXy);
          noise_z  = std::max(std::sqrt(var_z), kMinSigmaZ);
        }
      }
      // Innovation vs the IMU-predicted position, whitened by the noise sigma
      // just assigned above (not a full NIS against joint covariance — see
      // setup_publishers()). A z-score consistently >> 3 means this factor is
      // fighting the rest of the graph; consistently << 1 means the noise
      // model is too loose to matter.
      {
        const auto & pred_t = predicted.pose().translation();
        const double dx = local.x() - pred_t.x();
        const double dy = local.y() - pred_t.y();
        gps_innov_xy_m = std::sqrt(dx * dx + dy * dy);
        gps_zscore_xy = gps_innov_xy_m / std::max(noise_xy, 1e-6);
        gps_diag_valid = true;
      }

      const uint64_t gps_key =
          keyForStamp(rclcpp::Time(pending_gps_.header.stamp).seconds());
      graph_.add(gtsam::GPSFactor(X(gps_key), local,
          gtsam::noiseModel::Diagonal::Sigmas(
              gtsam::Vector3(noise_xy, noise_xy, noise_z))));
      has_pending_gps_ = false;
      }
    }

    // ── Map anchor (OOSM): absolute PriorFactor from /mapping/icp_measurement,
    // re-anchoring this node's "map" onto the ICP mapper's own frame instead
    // of GPS/ENU -- see the use_map_anchor declare_parameter comment. ──
    if (has_pending_map_anchor_) {
      const gtsam::Pose3 T_meas = odom_to_pose3(pending_map_anchor_);
      // Real registration-derived covariance from the mapper (diagonal only
      // -- mapper_node.cpp fills indices 0,7,14,21,28,35, i.e. exactly the
      // diag of a row-major 6x6 [x,y,z,roll,pitch,yaw] REP-103 layout), not
      // a hand-tuned sigma. Reordered into GTSAM's (rx,ry,rz,tx,ty,tz).
      const auto & cov = pending_map_anchor_.pose.covariance;
      constexpr double kMinSigma = 1e-4;  // floor vs a degenerate zero covariance
      const gtsam::Vector6 sigmas = (gtsam::Vector6() <<
          std::max(std::sqrt(std::max(cov[21], 0.0)), kMinSigma),   // roll
          std::max(std::sqrt(std::max(cov[28], 0.0)), kMinSigma),   // pitch
          std::max(std::sqrt(std::max(cov[35], 0.0)), kMinSigma),   // yaw
          std::max(std::sqrt(std::max(cov[0], 0.0)), kMinSigma),    // x
          std::max(std::sqrt(std::max(cov[7], 0.0)), kMinSigma),    // y
          std::max(std::sqrt(std::max(cov[14], 0.0)), kMinSigma))   // z
          .finished();

      const uint64_t anchor_key =
          keyForStamp(rclcpp::Time(pending_map_anchor_.header.stamp).seconds());
      graph_.addPrior(X(anchor_key), T_meas,
          gtsam::noiseModel::Diagonal::Sigmas(sigmas));

      if (anchor_key == curr_key) {
        const gtsam::Vector6 xi =
            gtsam::Pose3::Logmap(T_meas.between(predicted.pose()));
        map_anchor_innov_rot_rad = xi.head<3>().norm();
        map_anchor_innov_trans_m = xi.tail<3>().norm();
        const double trans_sigma = std::sqrt(
            (sigmas(3) * sigmas(3) + sigmas(4) * sigmas(4)) / 2.0);
        map_anchor_zscore = map_anchor_innov_trans_m / std::max(trans_sigma, 1e-6);
        map_anchor_diag_valid = true;
      }
      has_pending_map_anchor_ = false;
    }

    // ── GPS heading (OOSM) ──
    if (has_pending_heading_) {
      auto & q = pending_heading_.quaternion;
      auto heading_noise = gtsam::noiseModel::Diagonal::Sigmas(
          (gtsam::Vector6() << 99.0, 99.0, noise_.gps_heading_noise,
                               99.0, 99.0, 99.0).finished());
      const uint64_t heading_key =
          keyForStamp(rclcpp::Time(pending_heading_.header.stamp).seconds());
      // Translation part of the prior is a don't-care (sigma=99): reuse the
      // predicted pose's translation only when priming curr_key; for a historical
      // key just anchor rotation and let the existing translation stand.
      const gtsam::Point3 anchor_t = (heading_key == curr_key)
          ? predicted.pose().translation()
          : gtsam::Point3(0, 0, 0);
      graph_.addPrior(X(heading_key),
          gtsam::Pose3(gtsam::Rot3::Quaternion(q.w, q.x, q.y, q.z), anchor_t),
          heading_noise);
      has_pending_heading_ = false;
    }

    // ── Track odometry (OOSM) ──
    if (has_pending_odom_ && has_last_odom_) {
      const uint64_t from_key =
          keyForStamp(rclcpp::Time(last_odom_.header.stamp).seconds());
      const uint64_t to_key =
          keyForStamp(rclcpp::Time(pending_odom_.header.stamp).seconds());
      if (from_key != to_key) {
        gtsam::Pose3 delta =
            odom_to_pose3(last_odom_).between(odom_to_pose3(pending_odom_));
        graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(X(from_key), X(to_key), delta,
            robustNoise(gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector6() << 99.0, 99.0, noise_.odom_angular_noise,
                                    noise_.odom_linear_noise,
                                    noise_.odom_linear_noise, 99.0).finished()))));
      }
      last_odom_ = pending_odom_;
      has_pending_odom_ = false;
    }

    // ── LiDAR odometry (OOSM — typically the primary late/high-accuracy source) ──
    // No extrinsic conjugation needed here, unlike the visual block below:
    // /mapping/icp_odom is published as map->base_footprint directly
    // (mapper_node.cpp), already in base_frame_ -- do not "fix" this to
    // match the visual pattern, there is no lever-arm error to correct.
    if (has_pending_lidar_odom_ && has_last_lidar_odom_) {
      const uint64_t from_key =
          keyForStamp(rclcpp::Time(last_lidar_odom_.header.stamp).seconds());
      const uint64_t to_key =
          keyForStamp(rclcpp::Time(pending_lidar_odom_.header.stamp).seconds());
      if (from_key != to_key) {
        gtsam::Pose3 delta =
            odom_to_pose3(last_lidar_odom_).between(odom_to_pose3(pending_lidar_odom_));
        graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(X(from_key), X(to_key), delta,
            robustNoise(gtsam::noiseModel::Diagonal::Sigmas(noise_.lidar_odom_noise))));

        // Innovation, proper SE(3) Lie-algebra residual — only meaningful for
        // the common no-OOSM-lag case (from_key/to_key are this tick's own
        // prev/curr keyframes), where "predicted relative motion" is directly
        // comparable to the measured delta:
        //   xi = Logmap( delta^-1 . predicted_relative ),  predicted_relative
        //   = X(prev)_opt.between(X(curr)_IMU-predicted)
        // Same formula shape as TrailerPoseFactor's own residual (see
        // trailer_pose_factor.hpp) — deliberately reused for consistency.
        if (from_key == prev_key && to_key == curr_key) {
          const gtsam::Pose3 predicted_relative =
              current_state_.pose.between(predicted.pose());
          const gtsam::Vector6 xi =
              gtsam::Pose3::Logmap(delta.inverse().compose(predicted_relative));
          lidar_odom_innov_rot_rad = xi.head<3>().norm();
          lidar_odom_innov_trans_m = xi.tail<3>().norm();
          const double trans_sigma = std::sqrt(
              (noise_.lidar_odom_noise(3) * noise_.lidar_odom_noise(3) +
               noise_.lidar_odom_noise(4) * noise_.lidar_odom_noise(4)) / 2.0);
          lidar_odom_zscore = lidar_odom_innov_trans_m / std::max(trans_sigma, 1e-6);
          lidar_odom_diag_valid = true;
        }
      }
      last_lidar_odom_ = pending_lidar_odom_;
      has_pending_lidar_odom_ = false;
    }

    // ── Visual odometry (OOSM — ZED today, Isaac VSLAM the same way) ──
    if (has_pending_visual_odom_ && has_last_visual_odom_) {
      const uint64_t from_key =
          keyForStamp(rclcpp::Time(last_visual_odom_.header.stamp).seconds());
      const uint64_t to_key =
          keyForStamp(rclcpp::Time(pending_visual_odom_.header.stamp).seconds());
      // child_frame_id is the frame whose pose is reported (Isaac VSLAM:
      // zed_camera_link; ZED wrapper: same). Both messages in a pair come
      // from the same publisher/topic so this is constant across them —
      // reading it off `pending` rather than caching a separate config
      // value keeps this correct if the source's frame ever changes without
      // a restart.
      gtsam::Pose3 T_base_sensor;
      if (from_key != to_key &&
          ensureSensorExtrinsic(pending_visual_odom_.child_frame_id, T_base_sensor)) {
        const gtsam::Pose3 delta_sensor =
            odom_to_pose3(last_visual_odom_).between(odom_to_pose3(pending_visual_odom_));
        // Conjugate the sensor-frame delta into base_frame_: for a rigid
        // body, T_wc(k) = T_wb(k)·T_bc  =>  ΔT_base = T_bc·ΔT_cam·T_bc⁻¹.
        // Differencing sensor-frame poses directly (the old behaviour) and
        // applying that delta straight to X(k) (a base_frame_ pose) ignores
        // the lever arm — see ensureSensorExtrinsic's comment for the
        // magnitude on this robot. On this robot T_bc's rotation happens to
        // be identity (zed_camera_link is reached from base_footprint via
        // an even number of 180°-class yaws), so this conjugation only
        // corrects translation here — but is written generally, since
        // that's a property of THIS URDF, not something to assume holds
        // for every mount.
        const gtsam::Pose3 delta =
            T_base_sensor.compose(delta_sensor).compose(T_base_sensor.inverse());
        graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(X(from_key), X(to_key), delta,
            robustNoise(gtsam::noiseModel::Diagonal::Sigmas(noise_.visual_odom_noise))));

        // Innovation — same formula/caveats as the lidar_odom block above.
        if (from_key == prev_key && to_key == curr_key) {
          const gtsam::Pose3 predicted_relative =
              current_state_.pose.between(predicted.pose());
          const gtsam::Vector6 xi =
              gtsam::Pose3::Logmap(delta.inverse().compose(predicted_relative));
          visual_odom_innov_rot_rad = xi.head<3>().norm();
          visual_odom_innov_trans_m = xi.tail<3>().norm();
          const double trans_sigma = std::sqrt(
              (noise_.visual_odom_noise(3) * noise_.visual_odom_noise(3) +
               noise_.visual_odom_noise(4) * noise_.visual_odom_noise(4)) / 2.0);
          visual_odom_zscore = visual_odom_innov_trans_m / std::max(trans_sigma, 1e-6);
          visual_odom_diag_valid = true;
        }
      }
      // Roll last=pending and clear the pending flag unconditionally, even
      // when the extrinsic wasn't ready or from_key==to_key: this measurement
      // is consumed either way (as a factor, or dropped) — never resubmitted.
      last_visual_odom_ = pending_visual_odom_;
      has_pending_visual_odom_ = false;
    }

    // ── Articulation factors: H(curr_key)=φ, P(curr_key)=α ──
    if (use_articulation_) {
      // Initial value for H(curr_key): propagate from previous optimised φ
      initial_values_.insert(H_key(curr_key), current_state_.trailer_angle);
      new_timestamps[H_key(curr_key)] = tick_stamp;

      // Initial value for P(curr_key): propagate from previous optimised α
      if (use_pitch_state_) {
        initial_values_.insert(P_key(curr_key), current_alpha_);
        new_timestamps[P_key(curr_key)] = tick_stamp;
      }

      // ── Yaw dynamics: BetweenFactor H(prev)→H(curr) ──
      // Penalises large φ changes between keyframes (random-walk prior).
      {
        auto dyn_noise = gtsam::noiseModel::Isotropic::Sigma(
            1, noise_.phi_sigma_dynamics);
        graph_.add(gtsam::BetweenFactor<double>(
            H_key(prev_key), H_key(curr_key), 0.0, dyn_noise));
      }

      // ── Pitch dynamics: BetweenFactor P(prev)→P(curr) ──
      // Pitch changes slowly (terrain slope); σ_dynamics is tight (~0.5°/KF).
      if (use_pitch_state_) {
        auto pitch_dyn_noise = gtsam::noiseModel::Isotropic::Sigma(
            1, noise_.pitch_sigma_dynamics);
        graph_.add(gtsam::BetweenFactor<double>(
            P_key(prev_key), P_key(curr_key), 0.0, pitch_dyn_noise));
      }

      if (has_pending_articulation_) {
        const auto & art = pending_articulation_;
        const uint64_t art_key =
            keyForStamp(rclcpp::Time(art.header.stamp).seconds());

        // ── Yaw encoder: PriorFactor on H(art_key) (OOSM) ──
        const double phi_meas =
            art.hardware_fresh ? art.hardware_rad : art.effective_rad;
        const double sigma_enc =
            art.hardware_fresh ? noise_.phi_sigma_hardware : noise_.phi_sigma_model;

        // Innovation vs the last optimised phi (SO(2) residual — wrapped, see
        // normalizeAngle(); a raw subtraction is wrong across the +-pi branch
        // cut and would falsely flag a huge innovation right where phi wraps).
        art_innov_rad = std::abs(normalizeAngle(phi_meas - current_state_.trailer_angle));
        art_zscore = art_innov_rad / std::max(sigma_enc, 1e-6);
        art_diag_valid = true;

        graph_.addPrior(H_key(art_key), phi_meas,
            gtsam::noiseModel::Isotropic::Sigma(1, sigma_enc));

        // ── Pitch encoder: PriorFactor on P(art_key) (OOSM) ──
        // Only when potentiometer reading is fresh (ADC1, 8-bit).
        if (use_pitch_state_ && art.pitch_fresh) {
          graph_.addPrior(P_key(art_key), art.pitch_rad,
              gtsam::noiseModel::Isotropic::Sigma(1, noise_.pitch_sigma_hardware));
        }

        // ── Trailer LiDAR pose factor (OOSM: keyed by its own stamp, may differ
        // from art_key since it comes from a separate topic/node) ──
        // TrailerPoseFactorFull (joint φ+α) when pitch state enabled,
        // TrailerPoseFactor (φ only) when not.
        // Noise scaled by 1/√confidence — bad detections contribute little.
        if (use_trailer_pose_ && has_pending_trailer_pose_) {
          const double conf = std::clamp(latest_trailer_confidence_, 0.01, 1.0);

          if (conf >= trailer_min_confidence_) {
            const double scale = 1.0 / std::sqrt(conf);
            const double sr = noise_.trailer_sigma_rot   * scale;
            const double st = noise_.trailer_sigma_trans * scale;

            auto trailer_noise = gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector6() << sr, sr, sr, st, st, st).finished());

            const gtsam::Pose3 T_meas = stamped_to_pose3(pending_trailer_pose_);
            const uint64_t trailer_key = keyForStamp(
                rclcpp::Time(pending_trailer_pose_.header.stamp).seconds());

            // Innovation — literally the same residual TrailerPoseFactor(Full)
            // computes internally (see trailer_pose_factor.hpp), evaluated here
            // against the last optimised (phi, alpha) as a cheap pre-update
            // check: e = Logmap( Delta(phi,alpha)^-1 . T_measured ).
            {
              const gtsam::Pose3 predicted_trailer = mtt_loc::hitch_kinematics::computeDelta(
                  current_state_.trailer_angle, use_pitch_state_ ? current_alpha_ : 0.0);
              const gtsam::Vector6 xi =
                  gtsam::Pose3::Logmap(predicted_trailer.inverse().compose(T_meas));
              trailer_innov_rot_rad = xi.head<3>().norm();
              trailer_innov_trans_m = xi.tail<3>().norm();
              trailer_zscore = trailer_innov_trans_m / std::max(st, 1e-6);
              trailer_diag_valid = true;
            }

            if (use_pitch_state_) {
              // 6D residual constrains both φ and α simultaneously
              graph_.add(mtt_loc::TrailerPoseFactorFull(
                  H_key(trailer_key), P_key(trailer_key), T_meas, trailer_noise));
            } else {
              graph_.add(mtt_loc::TrailerPoseFactor(
                  H_key(trailer_key), T_meas, trailer_noise));
            }
          }
          has_pending_trailer_pose_ = false;
        }

        has_pending_articulation_ = false;
      }
    }

    // Publish innovation diagnostics unconditionally (per-field valid flags
    // let subscribers distinguish "no measurement this tick" from "innovation
    // was zero"); cheap, so no reason to gate it behind a parameter.
    {
      std_msgs::msg::Float64MultiArray diag_msg;
      diag_msg.data = {
          gps_diag_valid ? 1.0 : 0.0, gps_innov_xy_m, gps_zscore_xy,
          art_diag_valid ? 1.0 : 0.0, art_innov_rad, art_zscore,
          lidar_odom_diag_valid ? 1.0 : 0.0, lidar_odom_innov_trans_m,
              lidar_odom_innov_rot_rad, lidar_odom_zscore,
          trailer_diag_valid ? 1.0 : 0.0, trailer_innov_trans_m,
              trailer_innov_rot_rad, trailer_zscore,
          visual_odom_diag_valid ? 1.0 : 0.0, visual_odom_innov_trans_m,
              visual_odom_innov_rot_rad, visual_odom_zscore,
          map_anchor_diag_valid ? 1.0 : 0.0, map_anchor_innov_trans_m,
              map_anchor_innov_rot_rad, map_anchor_zscore,
      };
      innovation_diag_pub_->publish(diag_msg);
    }

    // ── Fixed-lag smoother update ──
    const auto update_start = std::chrono::steady_clock::now();
    bool update_ok = false;
    try {
      smoother_->update(graph_, initial_values_, new_timestamps);
      auto result = smoother_->calculateEstimate();

      const gtsam::Pose3 new_pose = result.at<gtsam::Pose3>(X(curr_key));
      const gtsam::Vector3 new_vel = result.at<gtsam::Vector3>(V(curr_key));
      const gtsam::imuBias::ConstantBias new_bias =
          result.at<gtsam::imuBias::ConstantBias>(B(curr_key));
      // Reject BEFORE accepting into current_state_ -- see isSaneNavState()
      // comment for why this is load-bearing, not defensive fluff. A GTSAM
      // solve that doesn't throw can still return a numerically blown-up
      // result; treating that the same as a thrown exception (same catch
      // block, same reset path) is what stops it from ever reaching
      // current_state_, /localization/odom, or a future reseed.
      if (!isSaneNavState(new_pose, new_vel, new_bias)) {
        throw std::runtime_error(
            "ISAM2 solve returned a non-finite or implausible state "
            "(|t|=" + std::to_string(new_pose.translation().norm()) +
            "m) -- rejecting before it reaches current_state_.");
      }
      current_state_.pose     = new_pose;
      current_state_.velocity = new_vel;
      current_state_.imu_bias = new_bias;
      current_state_.key_index = curr_key;
      current_state_.timestamp = tick_stamp;  // last committed keyframe -- see publish_odom_fast()
      stamp_to_key_[tick_stamp] = curr_key;

      if (use_articulation_) {
        current_state_.trailer_angle = result.at<double>(H_key(curr_key));
        if (use_pitch_state_) {
          current_alpha_ = result.at<double>(P_key(curr_key));
        }
      }
      update_ok = true;

      // ── Extract marginal covariances ──
      // smoother_->marginalCovariance(key) runs back-substitution on the
      // internal Bayes tree — O(n) but cheap for a single key at 50 Hz.
      if (extract_covariance_) {
        try {
          // Tractor pose covariance (6×6)
          cov_pose_ = smoother_->marginalCovariance(X(curr_key));
          // Articulation variances (1×1 each)
          if (use_articulation_) {
            cov_phi_ = smoother_->marginalCovariance(H_key(curr_key));
            if (use_pitch_state_) {
              cov_alpha_ = smoother_->marginalCovariance(P_key(curr_key));
            }
          }
          has_covariance_ = true;
        } catch (const std::exception & e) {
          RCLCPP_WARN_ONCE(get_logger(),
              "Covariance extraction failed: %s — using zeros", e.what());
          has_covariance_ = false;
        }
      }

    } catch (const std::exception & e) {
      // GTSAM's ISAM2/fixed-lag internal state is not transactional: once
      // update() throws partway through (e.g. IndeterminantLinearSystemException),
      // the Bayes tree is left inconsistent and every subsequent update() call
      // fails the same way forever (observed: "Smoother update failed" spamming
      // every tick with no recovery). Reset from scratch instead of limping on.
      RCLCPP_ERROR(get_logger(), "Smoother update failed: %s", e.what());
      resetSmootherAfterFailure(tick_stamp);
    }

    // Warn if this update took long enough to risk blocking sensor callbacks
    // (they share mtx_) for a meaningful fraction of the tick period — a signal
    // that the dedicated optimization thread (planned, not yet implemented) is
    // needed. Cheap to check every tick; not a functional change.
    {
      const double update_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - update_start).count();
      const double rate = get_parameter("publish_rate").as_double();
      const double tick_period_ms = 1000.0 / std::max(rate, 1.0);
      if (update_ms > update_duration_warn_ratio_ * tick_period_ms) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "Smoother update took %.1f ms (> %.0f%% of the %.1f ms tick period) — "
            "consider moving optimization off the sensor-callback mutex onto a "
            "dedicated thread if this recurs.",
            update_ms, update_duration_warn_ratio_ * 100.0, tick_period_ms);
      }
    }

    // Prune our own timestamp bookkeeping using the smoother's OWN
    // KeyTimestampMap as ground truth (not a locally recomputed cutoff) —
    // guarantees keyForStamp() can never return a key the smoother has already
    // marginalized, regardless of any off-by-one/boundary mismatch between our
    // own cutoff arithmetic and the smoother's internal marginalization
    // decision. Referencing an already-marginalized key is exactly the kind of
    // bug that throws mid-update and requires the reset above — this closes
    // that gap structurally instead of by careful arithmetic.
    if (update_ok) {
      const auto & live = smoother_->timestamps();
      for (auto it = stamp_to_key_.begin(); it != stamp_to_key_.end(); ) {
        it = (live.find(X(it->second)) == live.end())
            ? stamp_to_key_.erase(it) : std::next(it);
      }
      if (stamp_to_key_.empty()) {
        stamp_to_key_[tick_stamp] = curr_key;
      }
    }

    // Clear pending data and reset IMU
    graph_.resize(0);
    initial_values_.clear();
    imu_preint_->resetIntegrationAndSetBias(current_state_.imu_bias);
    imu_data_count_ = 0;

    publish_odometry();
    publish_articulation();
    publish_pitch();
    publish_tf();
  }

  // ─── Publish tractor odometry (with covariance from ISAM2) ──
  void publish_odometry() {
    nav_msgs::msg::Odometry msg;
    msg.header.stamp    = has_latest_odom_ ? latest_odom_.header.stamp
                                           : static_cast<builtin_interfaces::msg::Time>(now());
    msg.header.frame_id = map_frame_;
    msg.child_frame_id  = base_frame_;

    const auto & t = current_state_.pose.translation();
    const auto   q = current_state_.pose.rotation().toQuaternion();

    msg.pose.pose.position.x    = t.x();
    msg.pose.pose.position.y    = t.y();
    msg.pose.pose.position.z    = t.z();
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    msg.pose.pose.orientation.w = q.w();

    msg.twist.twist.linear.x = current_state_.velocity.x();
    msg.twist.twist.linear.y = current_state_.velocity.y();
    msg.twist.twist.linear.z = current_state_.velocity.z();

    // Populate 6×6 covariance from ISAM2 marginals (GTSAM: [ω;v] → ROS: [v;ω])
    // GTSAM Pose3 covariance is in the order [rx,ry,rz,tx,ty,tz]
    // ROS Odometry covariance is in the order [x,y,z,rx,ry,rz]
    // We permute: ROS[i][j] = GTSAM[perm[i]][perm[j]], perm = {3,4,5,0,1,2}
    if (has_covariance_) {
      constexpr int perm[6] = {3, 4, 5, 0, 1, 2};  // GTSAM→ROS reorder
      for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
          msg.pose.covariance[i * 6 + j] = cov_pose_(perm[i], perm[j]);
        }
      }
    }

    odom_pub_->publish(msg);
  }

  // ─── Publish optimised yaw angle ──
  void publish_articulation() {
    if (!use_articulation_) return;
    std_msgs::msg::Float64 msg;
    msg.data = current_state_.trailer_angle;
    articulation_pub_->publish(msg);
  }

  // ─── Publish optimised pitch angle ──
  void publish_pitch() {
    if (!use_articulation_ || !use_pitch_state_ || !pitch_pub_) return;
    std_msgs::msg::Float64 msg;
    msg.data = current_alpha_;
    pitch_pub_->publish(msg);
  }

  // ─── TF broadcast: map → odom ──
  void publish_tf() {
    // mapper_node owns map->odom by default -- see broadcast_tf declare_parameter
    // comment above. Kept computable/loggable via has_latest_odom_ regardless,
    // just not sent to /tf unless explicitly re-enabled.
    if (!broadcast_tf_) return;
    if (!has_latest_odom_) return;
    gtsam::Pose3 map_to_base = current_state_.pose;
    gtsam::Pose3 odom_to_base = odom_to_pose3(latest_odom_);
    gtsam::Pose3 map_to_odom = map_to_base.compose(odom_to_base.inverse());

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp    = latest_odom_.header.stamp;
    tf.header.frame_id = map_frame_;
    tf.child_frame_id  = odom_frame_;
    const auto & t = map_to_odom.translation();
    const auto   q = map_to_odom.rotation().toQuaternion();
    tf.transform.translation.x = t.x();
    tf.transform.translation.y = t.y();
    tf.transform.translation.z = t.z();
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(tf);
  }

  // ─── Members ──
  std::mutex mtx_;

  // Fixed-lag smoother
  std::unique_ptr<gtsam::IncrementalFixedLagSmoother> smoother_;
  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values initial_values_;
  gtsam::Matrix66 cov_pose_  = gtsam::Matrix66::Zero();
  gtsam::Matrix11 cov_phi_   = gtsam::Matrix11::Zero();
  gtsam::Matrix11 cov_alpha_ = gtsam::Matrix11::Zero();
  bool has_covariance_{false};
  double smoother_lag_seconds_{3.0};
  double update_duration_warn_ratio_{0.5};

  // OOSM bookkeeping: ROS-time seconds → keyframe index, for keyForStamp().
  // Pruned each tick to match the smoother's own marginalization window.
  std::map<double, uint64_t> stamp_to_key_;
  bool key0_timestamped_{false};

  // IMU
  std::unique_ptr<gtsam::PreintegratedCombinedMeasurements> imu_preint_;
  double last_imu_t_{-1.0};
  int imu_data_count_{0};

  // Optimised state
  mtt_loc::NavState current_state_;
  mtt_loc::SensorNoiseParams noise_;
  // Declared, loaded, logged -- not yet consumed by any factor. See the
  // odom_lateral_noise declare_parameter comment.
  double odom_lateral_noise_{0.1};
  double current_alpha_{0.0};           ///< optimised pitch angle α (from P(k))
  double trailer_min_confidence_{0.20};

  // Pending sensor data (tractor)
  sensor_msgs::msg::NavSatFix pending_gps_;
  geometry_msgs::msg::QuaternionStamped pending_heading_;
  nav_msgs::msg::Odometry pending_odom_, last_odom_, latest_odom_;
  nav_msgs::msg::Odometry pending_lidar_odom_, last_lidar_odom_;
  nav_msgs::msg::Odometry pending_visual_odom_, last_visual_odom_;
  nav_msgs::msg::Odometry pending_map_anchor_;

  bool has_pending_gps_{false}, has_pending_heading_{false};
  bool has_pending_odom_{false}, has_last_odom_{false}, has_latest_odom_{false};
  bool has_pending_lidar_odom_{false}, has_last_lidar_odom_{false};
  bool has_pending_visual_odom_{false}, has_last_visual_odom_{false};
  bool has_pending_map_anchor_{false};

  // Pending sensor data (articulation)
  mtt_msgs::msg::MttArticulationState pending_articulation_;
  geometry_msgs::msg::PoseStamped pending_trailer_pose_;
  double latest_trailer_confidence_{0.0};
  bool has_pending_articulation_{false};
  bool has_pending_trailer_pose_{false};

  // GPS origin
  double gps_origin_lat_{0.0}, gps_origin_lon_{0.0}, gps_origin_alt_{0.0};
  bool gps_origin_set_{false};

  // Feature flags
  bool use_imu_, use_odom_, use_gps_, use_gps_heading_;
  bool use_map_anchor_{false};
  std::string map_anchor_topic_;
  bool use_lidar_odom_, use_visual_odom_;
  bool use_articulation_, use_trailer_pose_, use_pitch_state_;
  bool extract_covariance_;

  // Frame IDs / topics
  std::string map_frame_, odom_frame_, base_frame_;
  bool broadcast_tf_{false};
  std::string imu_topic_, track_odom_topic_, gps_fix_topic_;
  std::string gps_heading_topic_, lidar_odom_topic_, visual_odom_topic_;
  std::string articulation_topic_, trailer_pose_topic_, trailer_confidence_topic_;
  std::string visual_status_topic_;
  double visual_status_timeout_seconds_{0.5};
  double visual_max_speed_mps_{4.0};
  double visual_max_yaw_rate_rps_{3.0};
  double visual_kinematic_margin_{2.0};
  int min_imu_samples_per_update_{10};

  // ROS interfaces
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_fast_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr articulation_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pitch_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr innovation_diag_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  gtsam::Rot3 R_base_imu_;
  bool imu_extrinsic_ready_{false};
  // Full Pose3 extrinsics (vs. R_base_imu_'s rotation-only), cached per
  // source child_frame_id -- see ensureSensorExtrinsic(). One node can have
  // multiple visual sources over its lifetime (ZED today, Isaac later) that
  // happen to share a child_frame_id (both "zed_camera_link"), so a single
  // cached Pose3 would already cover both, but keying by frame keeps this
  // correct if that ever changes.
  std::map<std::string, gtsam::Pose3> sensor_extrinsics_;
  // vo_state gate: true until a status message says otherwise, so an empty
  // visual_status_topic_ (ZED path, no such topic) leaves the gate always
  // passing -- see the visual_status_topic declare_parameter comment for
  // why this can't yet be isaac_ros_visual_slam_interfaces/VisualSlamStatus
  // directly.
  bool visual_status_ok_{true};
  rclcpp::Time visual_status_stamp_;
  bool has_visual_status_{false};

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr map_anchor_sub_;
  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr heading_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lidar_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr visual_odom_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr visual_status_sub_;
  rclcpp::Subscription<mtt_msgs::msg::MttArticulationState>::SharedPtr articulation_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr trailer_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr trailer_confidence_sub_;

  rclcpp::TimerBase::SharedPtr opt_timer_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FactorGraphNode>());
  rclcpp::shutdown();
  return 0;
}
