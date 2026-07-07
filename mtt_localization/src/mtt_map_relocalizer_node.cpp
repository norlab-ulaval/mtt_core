/**
 * mtt_map_relocalizer_node — Map-relative teach-and-repeat relocalization.
 *
 * THEORY
 * ──────
 * A WILN route stores: trajectory poses in the map frame of the TEACH session,
 * and the map point cloud captured during teach (.vtk).
 *
 * At REPLAY time the mapper may have a different origin (fresh mapping session,
 * or localization started at a different initial pose).  To bridge the two
 * sessions, we compute the rigid body transform T_live_old ∈ SE(3) that aligns
 * the old teach map onto the current live map, then apply it to every pose in
 * the stored trajectory:
 *
 *   p_live = T_live_old · p_old      (action of SE(3) on homogeneous vectors)
 *
 * SE(3) MATHS
 * ──────────
 * SE(3) elements are represented as 4×4 homogeneous matrices:
 *   T = [ R  t ]   where R ∈ SO(3), t ∈ ℝ³
 *       [ 0  1 ]
 *
 * Composition :  T1 · T2  (matrix product)
 * Inverse     :  T⁻¹ = [ R'  -R't ]
 *                       [ 0      1 ]
 * Group action:  T · p = R·p_xyz + t  (transport of a pose)
 *
 * ICP is provided by libpointmatcher.  The registration is old_cloud → live_cloud
 * (old cloud = reading, live cloud = reference), producing T_live_old directly.
 *
 * PIPELINE (service /mtt_map_relocalizer/relocalize)
 * ────────────────────────────────────────────────────
 * 1. Load old map: PM::DataPoints::load("<route>/route.ltr.vtk")
 * 2. Snapshot live map from latest /mapping/map PointCloud2
 * 3. Initial guess T0 = T_robot_in_live_map  (from /mapping/icp_odom)
 * 4. Coarse+fine ICP with yaw-hypothesis sweep (0°, ±90°, 180°)
 * 5. Score overlap ratio; if < threshold → fallback identity + warn
 * 6. Transport trajectory: for each pose p, p_live = T_live_old · p_old
 *    Republish corrected PathSequence on /wiln/trajectory (transient_local)
 * 7. Seed /mapping/pose_in with corrected route-start pose (loose prior)
 */

#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>

// ROS messages
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

// WILN trajectory type
#include <norlab_controllers_msgs/msg/path_sequence.hpp>
#include <norlab_controllers_msgs/msg/directional_path.hpp>

// MTT service
#include <mtt_interfaces/srv/relocalize_route.hpp>

// libpointmatcher
#include <pointmatcher/PointMatcher.h>

namespace mtt_loc {

using PM          = PointMatcher<float>;
using DP          = PM::DataPoints;
using PathSeq     = norlab_controllers_msgs::msg::PathSequence;
using RelocSrv    = mtt_interfaces::srv::RelocalizeRoute;

// ──────────────────────────────────────────────────────────────────────────────
// SE(3) helpers (self-contained — avoids cross-package header dependency)
// ──────────────────────────────────────────────────────────────────────────────

/// geometry_msgs::Pose → 4×4 SE(3) matrix (double).
static Eigen::Matrix4d poseToSE3(const geometry_msgs::msg::Pose& p)
{
    Eigen::Quaterniond q(p.orientation.w, p.orientation.x,
                         p.orientation.y, p.orientation.z);
    q.normalize();
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.topLeftCorner<3,3>()  = q.toRotationMatrix();
    T(0,3) = p.position.x;
    T(1,3) = p.position.y;
    T(2,3) = p.position.z;
    return T;
}

/// 4×4 SE(3) matrix → geometry_msgs::Pose.
static geometry_msgs::msg::Pose SE3ToPose(const Eigen::Matrix4d& T)
{
    Eigen::Quaterniond q(T.topLeftCorner<3,3>());
    q.normalize();
    geometry_msgs::msg::Pose p;
    p.position.x    = T(0,3);
    p.position.y    = T(1,3);
    p.position.z    = T(2,3);
    p.orientation.x = q.x();
    p.orientation.y = q.y();
    p.orientation.z = q.z();
    p.orientation.w = q.w();
    return p;
}

/// sensor_msgs::PointCloud2 → libpointmatcher DataPoints (x,y,z + homogeneous pad).
static DP rosToDP(const sensor_msgs::msg::PointCloud2& msg)
{
    DP::Labels feat_labels;
    feat_labels.push_back(DP::Label("x",   1));
    feat_labels.push_back(DP::Label("y",   1));
    feat_labels.push_back(DP::Label("z",   1));
    feat_labels.push_back(DP::Label("pad", 1));

    const int N = static_cast<int>(msg.width * msg.height);
    DP dp(feat_labels, DP::Labels{}, N);

    sensor_msgs::PointCloud2ConstIterator<float> ix(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iy(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iz(msg, "z");

    int col = 0;
    for (; ix != ix.end() && col < N; ++ix, ++iy, ++iz, ++col) {
        if (!std::isfinite(*ix) || !std::isfinite(*iy) || !std::isfinite(*iz)) {
            dp.features(0,col) = dp.features(1,col) = dp.features(2,col) = 0.f;
        } else {
            dp.features(0,col) = *ix;
            dp.features(1,col) = *iy;
            dp.features(2,col) = *iz;
        }
        dp.features(3,col) = 1.f;
    }
    if (col < N) dp.conservativeResize(col);
    return dp;
}

/// Fraction of source points (already in reference frame) whose nearest-neighbour
/// in reference is within dist_thresh.
static float estimateOverlap(const DP& source_in_ref, const DP& reference,
                              float dist_thresh = 0.5f)
{
    if (source_in_ref.getNbPoints() == 0 || reference.getNbPoints() == 0) return 0.f;

    PM::Parameters knn_params{{"knn","1"},{"epsilon","0"},{"searchType","KDTree"}};
    auto matcher = PM::get().MatcherRegistrar.create("KDTreeMatcher", knn_params);
    matcher->init(reference);

    // findClosests returns a Matches struct with .dists and .ids matrices
    PM::Matches matches = matcher->findClosests(source_in_ref);

    const float thresh2 = dist_thresh * dist_thresh;
    int inliers = 0;
    for (int i = 0; i < matches.dists.cols(); ++i)
        if (matches.dists(0,i) < thresh2) ++inliers;

    return static_cast<float>(inliers) / static_cast<float>(source_in_ref.getNbPoints());
}

/// Eigen double 4×4 → PM float 4×4 transformation parameters.
static PM::TransformationParameters eigenToPM(const Eigen::Matrix4d& T)
{
    PM::TransformationParameters Tf(4,4);
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            Tf(r,c) = static_cast<float>(T(r,c));
    return Tf;
}

// ──────────────────────────────────────────────────────────────────────────────
// Node
// ──────────────────────────────────────────────────────────────────────────────

class MttMapRelocalizerNode : public rclcpp::Node
{
public:
    MttMapRelocalizerNode() : Node("mtt_map_relocalizer")
    {
        // ── Parameters ────────────────────────────────────────────────────────
        declare_parameter("icp_max_dist_coarse",  2.0);  // [m] coarse pass
        declare_parameter("icp_max_dist_fine",    0.5);  // [m] refinement pass
        declare_parameter("icp_max_iter",         50);
        declare_parameter("overlap_min_ratio",    0.25); // reject below this
        declare_parameter("yaw_sweep_step_deg",   90.0); // 0,90,180,270 by default
        declare_parameter("map_topic",            std::string("/mapping/map"));
        declare_parameter("odom_topic",           std::string("/mapping/icp_odom"));
        declare_parameter("trajectory_topic",     std::string("/wiln/trajectory"));
        declare_parameter("pose_prior_topic",     std::string("/mapping/pose_in"));

        icp_max_dist_coarse_ = get_parameter("icp_max_dist_coarse").as_double();
        icp_max_dist_fine_   = get_parameter("icp_max_dist_fine").as_double();
        icp_max_iter_        = get_parameter("icp_max_iter").as_int();
        overlap_min_ratio_   = static_cast<float>(get_parameter("overlap_min_ratio").as_double());
        yaw_sweep_step_deg_  = get_parameter("yaw_sweep_step_deg").as_double();

        const std::string map_topic   = get_parameter("map_topic").as_string();
        const std::string odom_topic  = get_parameter("odom_topic").as_string();
        const std::string traj_topic  = get_parameter("trajectory_topic").as_string();
        const std::string prior_topic = get_parameter("pose_prior_topic").as_string();

        // Latched (transient-local) QoS for map and trajectory
        auto latched = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

        // ── Subscribers ───────────────────────────────────────────────────────
        map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            map_topic, latched,
            [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(map_mutex_);
                latest_map_ = std::move(msg);
            });

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, rclcpp::QoS(10).best_effort(),
            [this](nav_msgs::msg::Odometry::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(odom_mutex_);
                latest_odom_ = std::move(msg);
            });

        // Trajectory subscription (persistent — read inside service handler)
        traj_sub_ = create_subscription<PathSeq>(
            traj_topic, latched,
            [this](PathSeq::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(traj_mutex_);
                latest_traj_ = std::move(msg);
            });

        // ── Publishers ────────────────────────────────────────────────────────
        traj_pub_  = create_publisher<PathSeq>(traj_topic, latched);
        prior_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            prior_topic, rclcpp::QoS(10));

        // ── Service ───────────────────────────────────────────────────────────
        relocalize_srv_ = create_service<RelocSrv>(
            "~/relocalize",
            [this](RelocSrv::Request::SharedPtr  req,
                   RelocSrv::Response::SharedPtr res) {
                handleRelocalize(req, res);
            });

        RCLCPP_INFO(get_logger(),
            "mtt_map_relocalizer ready — map=%s odom=%s traj=%s prior=%s",
            map_topic.c_str(), odom_topic.c_str(),
            traj_topic.c_str(), prior_topic.c_str());
    }

private:
    // ── State ────────────────────────────────────────────────────────────────
    double icp_max_dist_coarse_, icp_max_dist_fine_;
    int    icp_max_iter_;
    float  overlap_min_ratio_;
    double yaw_sweep_step_deg_;

    sensor_msgs::msg::PointCloud2::SharedPtr latest_map_;
    nav_msgs::msg::Odometry::SharedPtr       latest_odom_;
    PathSeq::SharedPtr                       latest_traj_;
    std::mutex map_mutex_, odom_mutex_, traj_mutex_;

    // ── ROS handles ──────────────────────────────────────────────────────────
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr       odom_sub_;
    rclcpp::Subscription<PathSeq>::SharedPtr                       traj_sub_;
    rclcpp::Publisher<PathSeq>::SharedPtr                          traj_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr prior_pub_;
    rclcpp::Service<RelocSrv>::SharedPtr                           relocalize_srv_;

    // ── ICP registration (old cloud → live cloud) ────────────────────────────

    /**
     * Run one ICP pass (old_cloud = reading → live_cloud = reference).
     * Returns the converged PM transformation parameters (T_init on failure).
     */
    PM::TransformationParameters runICP(const DP& old_cloud, const DP& live_cloud,
                                        const PM::TransformationParameters& T_init,
                                        double max_dist)
    {
        // Point-to-plane ICP, 4-DOF (ground vehicle: fix roll/pitch to ground)
        std::ostringstream yaml;
        yaml << "readingDataPointsFilters:\n"
             << "  - RandomSamplingDataPointsFilter:\n"
             << "      prob: 0.65\n"
             << "referenceDataPointsFilters:\n"
             << "  - SurfaceNormalDataPointsFilter:\n"
             << "      knn: 8\n"
             << "      keepNormals: 1\n"
             << "      keepDensities: 0\n"
             << "matcher:\n"
             << "  KDTreeMatcher:\n"
             << "    knn: 1\n"
             << "    maxDist: " << max_dist << "\n"
             << "outlierFilters:\n"
             << "  - TrimmedDistOutlierFilter:\n"
             << "      ratio: 0.75\n"
             << "errorMinimizer:\n"
             << "  PointToPlaneWithCovErrorMinimizer:\n"
             << "    force4DOF: 1\n"
             << "transformationCheckers:\n"
             << "  - CounterTransformationChecker:\n"
             << "      maxIterationCount: " << icp_max_iter_ << "\n"
             << "  - DifferentialTransformationChecker:\n"
             << "      minDiffRotErr: 0.001\n"
             << "      minDiffTransErr: 0.01\n"
             << "      smoothLength: 4\n"
             << "inspector: NullInspector\n"
             << "logger: NullLogger\n";

        std::istringstream iss(yaml.str());
        PM::ICP icp;
        try {
            icp.loadFromYaml(iss);
            return icp(old_cloud, live_cloud, T_init);
        } catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "ICP failed (maxDist=%.2f): %s", max_dist, e.what());
            return T_init;
        }
    }

    /**
     * Full registration pipeline: sweep yaw hypotheses, run coarse+fine ICP for each,
     * keep the best by overlap ratio.
     * Returns {T_live_old (Eigen double 4×4), best overlap ratio}.
     */
    std::pair<Eigen::Matrix4d, float>
    registerMaps(const DP& old_cloud, const DP& live_cloud, const Eigen::Matrix4d& T0)
    {
        const double step_rad = yaw_sweep_step_deg_ * M_PI / 180.0;
        std::vector<double> yaw_offsets;
        for (double y = 0.0; y < 2.0 * M_PI - 1e-6; y += step_rad)
            yaw_offsets.push_back(y);

        Eigen::Matrix4d best_T  = Eigen::Matrix4d::Identity();
        float           best_ov = -1.f;

        for (double yaw_off : yaw_offsets) {
            // Perturb the initial guess by this yaw offset around the z-axis
            Eigen::Matrix4d R_delta = Eigen::Matrix4d::Identity();
            R_delta(0,0) =  std::cos(yaw_off);
            R_delta(0,1) = -std::sin(yaw_off);
            R_delta(1,0) =  std::sin(yaw_off);
            R_delta(1,1) =  std::cos(yaw_off);
            PM::TransformationParameters T_init = eigenToPM(T0 * R_delta);

            // Coarse pass
            PM::TransformationParameters T_pm = runICP(old_cloud, live_cloud, T_init,
                                                        icp_max_dist_coarse_);
            // Fine pass
            T_pm = runICP(old_cloud, live_cloud, T_pm, icp_max_dist_fine_);

            // Convert to Eigen double
            Eigen::Matrix4d T_cand = Eigen::Matrix4d::Identity();
            for (int r = 0; r < 4; ++r)
                for (int c = 0; c < 4; ++c)
                    T_cand(r,c) = static_cast<double>(T_pm(r,c));

            // Measure overlap: transform old cloud into live frame, count inliers
            DP src_moved = old_cloud;
            src_moved.features = (eigenToPM(T_cand) * old_cloud.features);
            const float ov = estimateOverlap(src_moved, live_cloud, 0.5f);

            RCLCPP_DEBUG(get_logger(),
                "yaw_hyp=%.0f° → overlap=%.3f", yaw_off * 180.0 / M_PI, ov);

            if (ov > best_ov) {
                best_ov = ov;
                best_T  = T_cand;
                if (best_ov >= 0.80f) break;  // early exit — good enough
            }
        }
        return {best_T, best_ov};
    }

    // ── Service handler ───────────────────────────────────────────────────────
    void handleRelocalize(RelocSrv::Request::SharedPtr  req,
                          RelocSrv::Response::SharedPtr res)
    {
        const auto t_start = std::chrono::steady_clock::now();
        RCLCPP_INFO(get_logger(), "Relocalize request: route='%s' map='%s'",
            req->route_name.c_str(), req->map_file.c_str());

        // ── 1. Load old teach map ────────────────────────────────────────────
        DP old_cloud;
        try {
            old_cloud = DP::load(req->map_file);
        } catch (const std::exception& e) {
            res->success = false;
            res->message = std::string("cannot load map: ") + e.what();
            RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
            return;
        }

        // Ensure 4-row homogeneous features (x,y,z,1) — .vtk may have only 3 rows
        if (old_cloud.featureExists("x") && old_cloud.features.rows() < 4) {
            DP::Labels lbl;
            lbl.push_back(DP::Label("x",1));
            lbl.push_back(DP::Label("y",1));
            lbl.push_back(DP::Label("z",1));
            lbl.push_back(DP::Label("pad",1));
            DP dp4(lbl, DP::Labels{}, old_cloud.getNbPoints());
            dp4.features.topRows(3) = old_cloud.features.topRows(3);
            dp4.features.row(3).setOnes();
            old_cloud = std::move(dp4);
        }
        if (old_cloud.getNbPoints() < 10) {
            res->success = false;
            res->message = "old map has too few points (" +
                           std::to_string(old_cloud.getNbPoints()) + ")";
            return;
        }
        RCLCPP_INFO(get_logger(), "Old map: %d pts loaded", old_cloud.getNbPoints());

        // ── 2. Snapshot live map ─────────────────────────────────────────────
        sensor_msgs::msg::PointCloud2::SharedPtr map_msg;
        {
            std::lock_guard<std::mutex> lk(map_mutex_);
            map_msg = latest_map_;
        }
        if (!map_msg || (map_msg->width * map_msg->height) == 0u) {
            res->success = false;
            res->message = "no live map on " + get_parameter("map_topic").as_string()
                         + " — is the mapper running?";
            RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
            return;
        }
        DP live_cloud = rosToDP(*map_msg);
        RCLCPP_INFO(get_logger(), "Live map: %d pts", live_cloud.getNbPoints());

        // ── 3. Initial guess: robot's current pose in live map frame ─────────
        Eigen::Matrix4d T0 = Eigen::Matrix4d::Identity();
        {
            std::lock_guard<std::mutex> lk(odom_mutex_);
            if (latest_odom_) {
                T0 = poseToSE3(latest_odom_->pose.pose);
            } else {
                RCLCPP_WARN(get_logger(), "No odom available — using identity as T0");
            }
        }

        // ── 4 & 5. ICP with yaw-hypothesis sweep ────────────────────────────
        auto [T_live_old, overlap] = registerMaps(old_cloud, live_cloud, T0);

        const double elapsed_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t_start).count();
        RCLCPP_INFO(get_logger(), "Registration: overlap=%.3f min=%.2f dt=%.0fms",
            overlap, overlap_min_ratio_, elapsed_ms);

        // ── 6. Overlap gate ──────────────────────────────────────────────────
        res->overlap_ratio = overlap;
        res->transform_row_major.fill(0.0);

        if (overlap < overlap_min_ratio_) {
            RCLCPP_WARN(get_logger(),
                "Overlap %.3f < min=%.2f — trajectory NOT corrected. "
                "Make sure the robot is near the teach area and the map covers it.",
                overlap, overlap_min_ratio_);
            res->success = false;
            res->message = "overlap too low (" + std::to_string(overlap)
                         + " < " + std::to_string(overlap_min_ratio_) + ")";
            return;
        }

        // ── 7. Transport trajectory poses: p_live = T_live_old · p_old ───────
        PathSeq::SharedPtr traj;
        {
            std::lock_guard<std::mutex> lk(traj_mutex_);
            traj = latest_traj_;
        }
        if (!traj || traj->paths.empty()) {
            RCLCPP_WARN(get_logger(),
                "Overlap OK (%.3f) but no trajectory cached — cannot correct poses", overlap);
            res->success = false;
            res->message = "no trajectory received (load route before calling service)";
            return;
        }

        PathSeq corrected = *traj;  // deep copy
        size_t n_poses = 0;
        for (auto& seg : corrected.paths) {
            for (auto& sp : seg.poses) {
                sp.pose = SE3ToPose(T_live_old * poseToSE3(sp.pose));
                ++n_poses;
            }
        }
        traj_pub_->publish(corrected);
        RCLCPP_INFO(get_logger(),
            "Republished corrected trajectory: %zu poses / %zu segments",
            n_poses, corrected.paths.size());

        // ── 8. Seed ICP mapper with corrected start pose ─────────────────────
        if (!traj->paths.empty() && !traj->paths[0].poses.empty()) {
            const Eigen::Matrix4d T_start_live =
                T_live_old * poseToSE3(traj->paths[0].poses[0].pose);

            geometry_msgs::msg::PoseWithCovarianceStamped prior;
            prior.header.stamp    = now();
            prior.header.frame_id = "map";
            prior.pose.pose       = SE3ToPose(T_start_live);
            // Loose diagonal covariance — ICP will refine on the next scan
            prior.pose.covariance.fill(0.0);
            prior.pose.covariance[0]  = 1.0;  // x  σ = 1 m
            prior.pose.covariance[7]  = 1.0;  // y
            prior.pose.covariance[14] = 0.1;  // z
            prior.pose.covariance[21] = 0.1;  // roll
            prior.pose.covariance[28] = 0.1;  // pitch
            prior.pose.covariance[35] = 0.5;  // yaw  σ ≈ 40°

            // Publish 3× with 30 ms spacing (mapper uses the latest received)
            for (int i = 0; i < 3; ++i) {
                prior_pub_->publish(prior);
                rclcpp::sleep_for(std::chrono::milliseconds(30));
            }
        }

        // ── Response ─────────────────────────────────────────────────────────
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c)
                res->transform_row_major[r*4+c] = T_live_old(r,c);

        std::ostringstream msg;
        msg << std::fixed << std::setprecision(4)
            << "ok | overlap=" << overlap
            << " t=[" << T_live_old(0,3) << ","
                      << T_live_old(1,3) << ","
                      << T_live_old(2,3) << "]"
            << " " << n_poses << " poses corrected"
            << " " << static_cast<int>(elapsed_ms) << " ms";
        res->success = true;
        res->message = msg.str();
        RCLCPP_INFO(get_logger(), "Relocalization: %s", res->message.c_str());
    }
};

}  // namespace mtt_loc

// ──────────────────────────────────────────────────────────────────────────────
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mtt_loc::MttMapRelocalizerNode>());
    rclcpp::shutdown();
    return 0;
}
