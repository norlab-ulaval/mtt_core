// Offline MTT reference-state batch smoother.
//
// This executable is intentionally batch/offline, separate from the online
// factor_graph_node (which runs a causal IncrementalFixedLagSmoother). It
// consumes the CSVs produced by scripts/extract_v2_measurements.py (a
// read-only MCAP pass — no ros2 bag play, no DDS) and writes a full-rate
// SE(3)+velocity+bias+articulation reference trajectory plus a quality
// summary. It never touches ROS/DDS and can run alongside a live session.
//
// ─── Architecture ──
//
// States, one keyframe per IMU sample (the IMU CSV's own rate, ~100 Hz):
//   X(k) = Pose3               tractor pose, map frame
//   V(k) = Vector3             tractor velocity, WORLD frame (GTSAM convention)
//   B(k) = imuBias::ConstantBias
//   H(k) = double              hitch yaw φ   (articulation, symbol 'h')
//   P(k) = double              hitch pitch α (articulation, symbol 'p')
//
// Factors:
//   • CombinedImuFactor(X,V,B)(k)->(k+1) for every consecutive IMU pair —
//     this is what makes "100 Hz" a real optimized rate rather than a
//     Slerp/lerp fabrication: motion between sparse absolute updates is
//     constrained by measured accel/gyro, not merely interpolated.
//   • ICP: modelled as a DENSE PriorFactor<Pose3> (absolute, ~20 Hz), NOT as
//     a BetweenFactor chain. This is a deliberate departure from the online
//     factor_graph_node (which treats /mapping/icp_odom as relative motion,
//     appropriate for a causal, OOSM-routed estimator). Here it matters
//     because GT_icp was produced by norlab_icp_mapper — scan-to-PERSISTENT-
//     MAP registration, not frame-to-frame — so each ICP sample already is a
//     quasi-absolute measurement in the map frame, and its accuracy at a
//     revisit already reflects registration against the shared accumulated
//     map. Chaining icp.csv's own consecutive poses as BetweenFactors and
//     *also* manufacturing "loop-closure" BetweenFactors from the same
//     already-self-consistent trajectory would be circular — composing any
//     two absolute poses of one trajectory via .between() is algebraically
//     implied by chaining the intervening consecutive-sample factors, so it
//     contributes zero new information. Treating ICP as a dense absolute
//     prior avoids this fallacy and is what actually carries the mapper's
//     revisit-consistency into the graph, with no separate loop-closure
//     detector required. See report.md for the quantitative check of this
//     hypothesis (return-pose error at detected revisits).
//   • Track odometry (/mtt_odometry) and ZED odometry: genuinely independent,
//     causally-drifting relative-motion sources -> BetweenFactor chains
//     between consecutive samples, loosely weighted (xy/yaw only for track
//     odom — chenilles slip badly in z/roll/pitch, mirrors the online node's
//     sigma=99 convention; ZED further loosened, extrinsic to hesai_lidar is
//     unvalidated in this workspace per the audit — see report.md).
//   • Articulation: H(k)/P(k) random-walk BetweenFactor at every consecutive
//     keyframe pair (so H/P are never rank-deficient between measurements),
//     plus PriorFactor<double> wherever a real measurement lands (hardware
//     encoder always; LiDAR PCA only if lidar_detected — 0% in this bag, see
//     audit_report.md, so H(k) here is hardware-only, stated not hidden).
//     No /trailer/pose (6-DOF) exists in this bag -> TrailerPoseFactorFull is
//     not used; pose_trailer_gt_100hz.csv is instead produced downstream by
//     composing hitch_kinematics::computeDelta(phi_opt, alpha_opt) onto X(k).
//
// Gauge: the dense ICP prior stream gauge-fixes the whole run (any dense
// absolute reference suffices); a light prior at k=0 only seeds the ~1.7s
// gap before the first ICP sample arrives.
//
// ─── Solver backend ──
//
// A single one-shot batch elimination (LevenbergMarquardtOptimizer over the
// WHOLE ~273k-variable, ~54k-long sequential chain) reliably segfaults deep
// inside GTSAM's linearization/elimination, REGARDLESS of initial-value
// quality (verified: a pathological open-loop-IMU-drift init and a healthy
// ICP-seeded init both crash identically at the same 273030-variable scale,
// while a 50000-variable subset with the same structure solves cleanly in 3
// iterations). This is a scale/elimination-structure limit, not a numerics
// bug. The fix is to solve the SAME graph incrementally with gtsam::ISAM2 in
// a single forward pass over the whole pre-loaded dataset, in batches of
// `--isam2-batch-size` keyframes — standard practice for long sequential SLAM
// problems, and it never holds one giant elimination tree at once. This is
// still a full, non-causal pass (every key's final estimate is kept — no
// marginalization/lag window like the online node), just incrementally
// linearized.

#include <pthread.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <gtsam/base/Lie.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include "mtt_localization/trailer_pose_factor.hpp"

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

inline gtsam::Key H_key(uint64_t i) { return gtsam::Symbol('h', i); }
inline gtsam::Key P_key(uint64_t i) { return gtsam::Symbol('p', i); }

namespace {

double nan_d() { return std::numeric_limits<double>::quiet_NaN(); }
double normalize_angle(double a) { return std::atan2(std::sin(a), std::cos(a)); }

// ─── Args ──
struct Args {
  std::string imu_csv;
  std::string icp_csv;
  std::string artic_csv;
  std::string track_odom_csv;
  std::string zed_odom_csv;
  std::string isaac_vslam_csv;
  std::string output_dir;

  double icp_sigma_xy{0.05};      // m, overridden by measured jitter if computable
  double icp_sigma_z{0.08};       // m
  double icp_sigma_rot{0.02};     // rad
  bool icp_sigma_from_data{true};

  double track_odom_sigma_xy{0.15};   // m/s worth of drift per sample-to-sample step
  double track_odom_sigma_yaw{0.10};  // rad
  double zed_odom_sigma_xy{0.15};     // m — loosened vs state.hpp visual_odom_noise (0.02)
  double zed_odom_sigma_rot{0.20};    // rad — extrinsic unvalidated, see report.md
  // /isaac/vslam/odometry (cuVSLAM, local BA + optional ZED IMU fusion in VIO
  // mode). Tighter than raw zed_odom_sigma_* because cuVSLAM does local
  // bundle adjustment (not just frame-to-frame VO), but still loose: NOT YET
  // VALIDATED against real recorded data on this workspace (GPU passthrough
  // blocked as of this writing, see report.md / session notes) -- retune once
  // an actual isaac_vslam.csv exists to compare against GT_icp.
  double isaac_vslam_sigma_xy{0.08};  // m
  double isaac_vslam_sigma_rot{0.10}; // rad

  double phi_sigma_hardware{0.008};
  double phi_sigma_model{0.035};
  double phi_sigma_dynamics{0.015};
  double phi_prior_sigma{0.5};
  double pitch_sigma_hardware{0.020};
  double pitch_sigma_dynamics{0.008};
  double pitch_prior_sigma{0.20};

  double imu_accel_noise{0.01};
  double imu_gyro_noise{0.0001};
  // Bias random-walk PSD. NOT copied from factor_graph_node.cpp's online/causal
  // defaults: those are tuned for a 3s fixed-lag window where the bias state
  // never needs to explain more than a few seconds of drift. Here the bias is a
  // single state shared across the whole 546s batch. At the online defaults
  // (1e-4 / 1e-6) the B(0) prior (sigma=1e-3, see addPrior below) combined with
  // the tiny per-second walk variance pins the bias within ~2e-4 m/s^2 of zero
  // for the entire run, well below the ~0.046 m/s^2 gravity-projection residual
  // measured on the stationary window (audit/imu_static_check.yaml) -- i.e. the
  // bias state is structurally forbidden from absorbing a real, measured
  // systematic error. Verified empirically: at these defaults final
  // accel_bias_norm=2.1e-4 m/s^2; loosened to 0.01/1e-4 it converges to
  // 1.06e-1 m/s^2 (same order of magnitude as the measured static residual)
  // while pose_twist_residual_mean is unchanged (8.25mm/s in both cases) and
  // final_error drops slightly (12770 -> 12099). See report.md "IMU bias
  // random-walk sensitivity".
  double imu_accel_walk{0.01};
  double imu_gyro_walk{1e-4};

  double robust_k{1.345};
  int isam2_batch_size{200};
  int marginals_stride{20};  // compute covariance every Nth keyframe (perf)
  bool skip_marginals{false};

  double loop_closure_dist_m{1.5};
  double loop_closure_min_dt_s{15.0};
};

Args parse_args(int argc, char** argv) {
  Args a;
  for (int i = 1; i < argc; ++i) {
    std::string key = argv[i];
    auto req = [&](const std::string& name) -> std::string {
      if (i + 1 >= argc) throw std::runtime_error("missing value for " + name);
      return argv[++i];
    };
    if (key == "--imu") a.imu_csv = req(key);
    else if (key == "--icp") a.icp_csv = req(key);
    else if (key == "--artic") a.artic_csv = req(key);
    else if (key == "--track-odom") a.track_odom_csv = req(key);
    else if (key == "--zed-odom") a.zed_odom_csv = req(key);
    else if (key == "--isaac-vslam") a.isaac_vslam_csv = req(key);
    else if (key == "--output-dir") a.output_dir = req(key);
    else if (key == "--icp-sigma-xy") { a.icp_sigma_xy = std::stod(req(key)); a.icp_sigma_from_data = false; }
    else if (key == "--icp-sigma-z") a.icp_sigma_z = std::stod(req(key));
    else if (key == "--icp-sigma-rot") a.icp_sigma_rot = std::stod(req(key));
    else if (key == "--track-odom-sigma-xy") a.track_odom_sigma_xy = std::stod(req(key));
    else if (key == "--track-odom-sigma-yaw") a.track_odom_sigma_yaw = std::stod(req(key));
    else if (key == "--zed-odom-sigma-xy") a.zed_odom_sigma_xy = std::stod(req(key));
    else if (key == "--zed-odom-sigma-rot") a.zed_odom_sigma_rot = std::stod(req(key));
    else if (key == "--isaac-vslam-sigma-xy") a.isaac_vslam_sigma_xy = std::stod(req(key));
    else if (key == "--isaac-vslam-sigma-rot") a.isaac_vslam_sigma_rot = std::stod(req(key));
    else if (key == "--phi-sigma-hardware") a.phi_sigma_hardware = std::stod(req(key));
    else if (key == "--phi-sigma-model") a.phi_sigma_model = std::stod(req(key));
    else if (key == "--phi-sigma-dynamics") a.phi_sigma_dynamics = std::stod(req(key));
    else if (key == "--pitch-sigma-hardware") a.pitch_sigma_hardware = std::stod(req(key));
    else if (key == "--pitch-sigma-dynamics") a.pitch_sigma_dynamics = std::stod(req(key));
    else if (key == "--imu-accel-noise") a.imu_accel_noise = std::stod(req(key));
    else if (key == "--imu-gyro-noise") a.imu_gyro_noise = std::stod(req(key));
    else if (key == "--imu-accel-walk") a.imu_accel_walk = std::stod(req(key));
    else if (key == "--imu-gyro-walk") a.imu_gyro_walk = std::stod(req(key));
    else if (key == "--robust-k") a.robust_k = std::stod(req(key));
    else if (key == "--isam2-batch-size") a.isam2_batch_size = std::stoi(req(key));
    else if (key == "--marginals-stride") a.marginals_stride = std::stoi(req(key));
    else if (key == "--skip-marginals") a.skip_marginals = true;
    else if (key == "--no-track-odom") a.track_odom_csv.clear();
    else if (key == "--no-zed-odom") a.zed_odom_csv.clear();
    else if (key == "--no-isaac-vslam") a.isaac_vslam_csv.clear();
    else if (key == "--help" || key == "-h") {
      std::cout << "Usage: offline_reference_solver --imu imu.csv --icp icp.csv "
                   "--artic articulation_state.csv [--track-odom track_odom.csv] "
                   "[--zed-odom zed_odom.csv] [--isaac-vslam isaac_vslam.csv] "
                   "--output-dir DIR\n";
      std::exit(0);
    } else {
      throw std::runtime_error("unknown argument: " + key);
    }
  }
  if (a.imu_csv.empty() || a.icp_csv.empty() || a.output_dir.empty()) {
    throw std::runtime_error("--imu, --icp, and --output-dir are required");
  }
  return a;
}

// ─── Generic CSV reading ──
std::vector<std::string> split_csv_line(const std::string& line) {
  std::vector<std::string> out;
  std::string field;
  bool in_quotes = false;
  for (char c : line) {
    if (c == '"') in_quotes = !in_quotes;
    else if (c == ',' && !in_quotes) { out.push_back(field); field.clear(); }
    else field.push_back(c);
  }
  out.push_back(field);
  return out;
}

struct Table {
  std::vector<std::string> headers;
  std::unordered_map<std::string, std::size_t> index;
  std::vector<std::vector<std::string>> rows;

  double d(std::size_t row, const std::string& col, double fallback = nan_d()) const {
    auto it = index.find(col);
    if (it == index.end() || it->second >= rows[row].size()) return fallback;
    const auto& s = rows[row][it->second];
    if (s.empty()) return fallback;
    try { return std::stod(s); } catch (...) { return fallback; }
  }
  std::string s(std::size_t row, const std::string& col) const {
    auto it = index.find(col);
    if (it == index.end() || it->second >= rows[row].size()) return "";
    return rows[row][it->second];
  }
  std::size_t size() const { return rows.size(); }
};

// Strips a trailing '\r' left by std::getline() on CRLF-terminated files.
// Without this, the LAST field of every line (including the last header
// name) silently carries a trailing '\r' that never matches any lookup —
// Table::d()/s() then fall through to the NaN/"" fallback for that column
// on EVERY row, with no error raised. This was the actual root cause of a
// long-standing IndeterminantLinearSystemException: on a CRLF-terminated
// IMU CSV, the last column ("gz") was silently NaN on every row, corrupting
// every single CombinedImuFactor from the very first one (root-caused via
// direct diagnostic: input covariances were all finite, but every gyro
// measurement's z-component silently came back NaN from the CSV reader,
// not from GTSAM itself — ruled out CombinedImuFactor/ISAM2/elimination-
// method as the culprit by reproducing the identical failure with plain
// ImuFactor and with LevenbergMarquardt, then eliminating it entirely by
// fixing the parser; CombinedImuFactor was restored unchanged afterward).
void strip_cr(std::string& s) {
  if (!s.empty() && s.back() == '\r') s.pop_back();
}

Table read_csv(const std::string& path) {
  Table t;
  if (path.empty()) return t;
  std::ifstream stream(path);
  if (!stream) throw std::runtime_error("cannot open CSV: " + path);
  std::string line;
  if (!std::getline(stream, line)) throw std::runtime_error("empty CSV: " + path);
  strip_cr(line);
  t.headers = split_csv_line(line);
  for (std::size_t i = 0; i < t.headers.size(); ++i) t.index[t.headers[i]] = i;
  while (std::getline(stream, line)) {
    strip_cr(line);
    if (line.empty()) continue;
    t.rows.push_back(split_csv_line(line));
  }
  return t;
}

gtsam::Pose3 pose_from_row(const Table& t, std::size_t i) {
  return {gtsam::Rot3::Quaternion(t.d(i, "qw"), t.d(i, "qx"), t.d(i, "qy"), t.d(i, "qz")),
          gtsam::Point3(t.d(i, "x"), t.d(i, "y"), t.d(i, "z"))};
}

// Nearest-keyframe index for a given timestamp, given a sorted vector of keyframe times.
std::size_t nearest_index(const std::vector<double>& times, double t) {
  auto it = std::lower_bound(times.begin(), times.end(), t);
  if (it == times.begin()) return 0;
  if (it == times.end()) return times.size() - 1;
  std::size_t hi = std::distance(times.begin(), it);
  std::size_t lo = hi - 1;
  return (std::abs(times[hi] - t) < std::abs(times[lo] - t)) ? hi : lo;
}

auto robust(const gtsam::SharedNoiseModel& base, double k) {
  return gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(k), base);
}

}  // namespace

// Runs the whole solve. Executed on a dedicated large-stack thread by main()
// below — see the rationale there. Never called directly from main()'s own
// (small, OS-default) stack.
int run(int argc, char** argv) {
  // Force a flush after every '<<' — diagnostics matter more than the tiny
  // perf cost here, and a SIGSEGV loses any output still sitting in a
  // fully-buffered (non-tty, redirected) stdout buffer otherwise.
  std::cout << std::unitbuf;
  std::cerr << std::unitbuf;
  try {
    const Args args = parse_args(argc, argv);

    // ── Load tables ──
    std::cout << "Loading " << args.imu_csv << " ...\n";
    const Table imu = read_csv(args.imu_csv);
    std::cout << "  imu rows: " << imu.size() << "\n";
    std::cout << "Loading " << args.icp_csv << " ...\n";
    const Table icp = read_csv(args.icp_csv);
    std::cout << "  icp rows: " << icp.size() << "\n";
    std::cout << "Loading " << args.artic_csv << " ...\n";
    const Table artic = read_csv(args.artic_csv);
    std::cout << "  artic rows: " << artic.size() << "\n";
    std::cout << "Loading " << args.track_odom_csv << " ...\n";
    const Table track_odom = read_csv(args.track_odom_csv);
    std::cout << "  track_odom rows: " << track_odom.size() << "\n";
    std::cout << "Loading " << args.zed_odom_csv << " ...\n";
    const Table zed_odom = read_csv(args.zed_odom_csv);
    std::cout << "  zed_odom rows: " << zed_odom.size() << "\n";
    std::cout << "Loading " << args.isaac_vslam_csv << " ...\n";
    const Table isaac_vslam = read_csv(args.isaac_vslam_csv);
    std::cout << "  isaac_vslam rows: " << isaac_vslam.size() << "\n";

    if (imu.size() < 2) throw std::runtime_error("IMU CSV has fewer than 2 rows");
    if (icp.size() < 2) throw std::runtime_error("ICP CSV has fewer than 2 rows");

    const std::size_t N = imu.size();
    std::vector<double> imu_t(N);
    for (std::size_t k = 0; k < N; ++k) imu_t[k] = imu.d(k, "t");

    std::cout << "Keyframes (IMU rate): " << N << "\n";
    std::cout << "ICP samples: " << icp.size() << "\n";

    // ── Empirical ICP sigma from high-frequency jitter (documented proxy: the
    // ICP CSV's own covariance columns are unusable — pose covariance is all
    // zero, twist covariance is -1, see extraction audit). We estimate xy/z
    // registration noise as the residual of each sample against a short
    // moving-average of itself: low-order drift/curvature is not noise, but
    // sample-to-sample jitter around the local trend is a reasonable lower
    // bound for registration precision. ──
    double icp_sigma_xy = args.icp_sigma_xy;
    double icp_sigma_z = args.icp_sigma_z;
    if (args.icp_sigma_from_data && icp.size() > 11) {
      std::vector<double> res_xy, res_z;
      const int half_win = 5;
      for (std::size_t i = half_win; i + half_win < icp.size(); ++i) {
        double mx = 0, my = 0, mz = 0;
        for (int d = -half_win; d <= half_win; ++d) {
          mx += icp.d(i + d, "x"); my += icp.d(i + d, "y"); mz += icp.d(i + d, "z");
        }
        const int wsize = 2 * half_win + 1;
        mx /= wsize; my /= wsize; mz /= wsize;
        const double dx = icp.d(i, "x") - mx, dy = icp.d(i, "y") - my, dz = icp.d(i, "z") - mz;
        res_xy.push_back(std::sqrt(dx * dx + dy * dy));
        res_z.push_back(std::abs(dz));
      }
      auto rms = [](const std::vector<double>& v) {
        double s = 0; for (double x : v) s += x * x; return std::sqrt(s / std::max<std::size_t>(1, v.size()));
      };
      const double measured_jitter_xy = rms(res_xy);
      const double measured_jitter_z = rms(res_z);
      // Floored well above the measured jitter: jitter is the smoothness of an
      // already-registered, already-filtered trajectory, not registration
      // accuracy — using it directly turns ~10.9k ICP samples into near-hard
      // constraints. This is a documented assumption, not a measurement
      // (logged explicitly so it is never silently mistaken for one).
      icp_sigma_xy = std::max(0.04, measured_jitter_xy);
      icp_sigma_z = std::max(0.08, measured_jitter_z);
      std::cout << "ICP sigma: measured jitter xy=" << measured_jitter_xy << " z=" << measured_jitter_z
                << " (11-sample window, NOT registration accuracy) -> floored to xy=" << icp_sigma_xy
                << " z=" << icp_sigma_z << " (assumption, see report.md)\n";
    }

    // ── ICP -> IMU-grid interpolation, used ONLY to seed initial values (never
    // as a factor/output). This is the critical fix for a 546 s sequence: open-
    // loop double-integrating raw IMU accel from k=0 with zero initial bias
    // accumulates ~0.5*residual_specific_force*T^2 position error — at this
    // bag's measured gravity residual (~0.046 m/s^2, see audit) that is on the
    // order of KILOMETERS by the end of the run, against ICP priors at ~1 cm
    // sigma: a linearization point the optimizer cannot recover from. Seeding
    // X(k)/V(k) from the ICP trajectory itself keeps every FACTOR exactly as
    // designed (CombinedImuFactor still carries the physics) — only the
    // starting guess changes. ──
    std::vector<double> icp_t(icp.size());
    for (std::size_t i = 0; i < icp.size(); ++i) icp_t[i] = icp.d(i, "t");
    auto interp_pose_at = [&](double t) -> gtsam::Pose3 {
      auto it = std::lower_bound(icp_t.begin(), icp_t.end(), t);
      if (it == icp_t.begin()) return pose_from_row(icp, 0);
      if (it == icp_t.end()) return pose_from_row(icp, icp.size() - 1);
      const std::size_t hi = std::distance(icp_t.begin(), it);
      const std::size_t lo = hi - 1;
      const double t_lo = icp_t[lo], t_hi = icp_t[hi];
      double frac = (t_hi > t_lo) ? (t - t_lo) / (t_hi - t_lo) : 0.0;
      frac = std::clamp(frac, 0.0, 1.0);
      return gtsam::interpolate<gtsam::Pose3>(pose_from_row(icp, lo), pose_from_row(icp, hi), frac);
    };
    auto vel_world_guess_at = [&](double t, const gtsam::Rot3& r) -> gtsam::Vector3 {
      const std::size_t idx = nearest_index(icp_t, t);
      const gtsam::Vector3 v_body(icp.d(idx, "vx"), icp.d(idx, "vy"), icp.d(idx, "vz"));
      return r.rotate(v_body);
    };

    // Sanity-check the seed itself, before it ever reaches the optimizer —
    // this is exactly the check that would have caught the open-loop-drift
    // bug immediately instead of segfaulting deep inside Cholesky.
    {
      double max_pos_norm = 0.0, max_vel_norm = 0.0;
      for (std::size_t k = 0; k < N; k += std::max<std::size_t>(1, N / 200)) {
        const gtsam::Pose3 p = interp_pose_at(imu_t[k]);
        const gtsam::Vector3 v = vel_world_guess_at(imu_t[k], p.rotation());
        max_pos_norm = std::max(max_pos_norm, p.translation().norm());
        max_vel_norm = std::max(max_vel_norm, v.norm());
      }
      std::cout << "Initial-value sanity check (ICP-seeded, sampled): max |position| = " << max_pos_norm
                << " m, max |velocity| = " << max_vel_norm << " m/s\n";
      if (max_pos_norm > 500.0 || max_vel_norm > 30.0) {
        std::cerr << "WARNING: initial guess looks implausible for a rink-scale session "
                     "(position/velocity magnitude far too large) — check icp.csv units/frame "
                     "before trusting the solve.\n";
      }
    }

    const bool use_articulation = artic.size() > 0;

    // ── IMU preintegration params (shared across all per-step factors) ──
    auto imu_params = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(9.81);
    imu_params->accelerometerCovariance = gtsam::I_3x3 * std::pow(args.imu_accel_noise, 2);
    imu_params->gyroscopeCovariance = gtsam::I_3x3 * std::pow(args.imu_gyro_noise, 2);
    imu_params->biasAccCovariance = gtsam::I_3x3 * std::pow(args.imu_accel_walk, 2);
    imu_params->biasOmegaCovariance = gtsam::I_3x3 * std::pow(args.imu_gyro_walk, 2);
    imu_params->integrationCovariance = gtsam::I_3x3 * 1e-8;

    // ── Bucket non-IMU factors by the LATEST key they touch, so they can be
    // added to the incremental ISAM2 batch at (or after) the step where that
    // key is first created. ──
    std::vector<gtsam::NonlinearFactorGraph> factors_at_key(N);

    // ICP: dense absolute PriorFactor<Pose3>.
    const auto icp_prior_noise = robust(
        gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector6() << args.icp_sigma_rot, args.icp_sigma_rot, args.icp_sigma_rot,
                                 icp_sigma_xy, icp_sigma_xy, icp_sigma_z).finished()),
        args.robust_k);
    std::size_t icp_prior_count = 0;
    std::vector<std::size_t> icp_key_of_row(icp.size());
    for (std::size_t i = 0; i < icp.size(); ++i) {
      const std::size_t key = nearest_index(imu_t, icp.d(i, "t"));
      icp_key_of_row[i] = key;
      factors_at_key[key].add(gtsam::PriorFactor<gtsam::Pose3>(X(key), pose_from_row(icp, i), icp_prior_noise));
      ++icp_prior_count;
    }
    std::cout << "ICP (dense PriorFactor<Pose3>) factors: " << icp_prior_count << "\n";

    // Articulation measurements.
    std::size_t phi_hw_count = 0, phi_lidar_count = 0, pitch_count = 0;
    if (use_articulation) {
      for (std::size_t i = 0; i < artic.size(); ++i) {
        const std::size_t key = nearest_index(imu_t, artic.d(i, "t"));
        if (artic.s(i, "hardware_fresh") == "1") {
          factors_at_key[key].add(gtsam::PriorFactor<double>(H_key(key), artic.d(i, "hardware_rad"),
              gtsam::noiseModel::Isotropic::Sigma(1, args.phi_sigma_hardware)));
          ++phi_hw_count;
        }
        if (artic.s(i, "lidar_detected") == "1") {
          factors_at_key[key].add(gtsam::PriorFactor<double>(H_key(key), artic.d(i, "lidar_rad"),
              gtsam::noiseModel::Isotropic::Sigma(1, args.phi_sigma_model)));
          ++phi_lidar_count;
        }
        if (artic.s(i, "pitch_fresh") == "1") {
          factors_at_key[key].add(gtsam::PriorFactor<double>(P_key(key), artic.d(i, "pitch_rad"),
              gtsam::noiseModel::Isotropic::Sigma(1, args.pitch_sigma_hardware)));
          ++pitch_count;
        }
      }
    }
    std::cout << "Articulation: hardware=" << phi_hw_count << " lidar=" << phi_lidar_count
              << " pitch=" << pitch_count
              << (phi_lidar_count == 0 ? "  [LiDAR hitch stream never fired in this bag — see audit_report.md]" : "")
              << "\n";

    // Track odometry: consecutive-sample BetweenFactor, xy/yaw only. Bucketed
    // at curr_key — safe because prev_key <= curr_key always (nearest_index is
    // monotonic non-decreasing over sorted, increasing query times), so
    // prev_key's variable is guaranteed to already exist by the time curr_key
    // is reached in the forward ISAM2 pass below.
    std::size_t track_odom_count = 0;
    if (track_odom.size() > 1) {
      const auto noise = robust(
          gtsam::noiseModel::Diagonal::Sigmas(
              (gtsam::Vector6() << 99.0, 99.0, args.track_odom_sigma_yaw,
                                   args.track_odom_sigma_xy, args.track_odom_sigma_xy, 99.0).finished()),
          args.robust_k);
      std::size_t prev_key = nearest_index(imu_t, track_odom.d(0, "t"));
      gtsam::Pose3 prev_pose = pose_from_row(track_odom, 0);
      for (std::size_t i = 1; i < track_odom.size(); ++i) {
        const std::size_t curr_key = nearest_index(imu_t, track_odom.d(i, "t"));
        if (curr_key == prev_key) continue;
        const gtsam::Pose3 curr_pose = pose_from_row(track_odom, i);
        factors_at_key[curr_key].add(gtsam::BetweenFactor<gtsam::Pose3>(X(prev_key), X(curr_key),
            prev_pose.between(curr_pose), noise));
        prev_key = curr_key;
        prev_pose = curr_pose;
        ++track_odom_count;
      }
    }
    std::cout << "Track odom BetweenFactor: " << track_odom_count << "\n";

    // ZED odometry: consecutive-sample BetweenFactor, loose, full 6DOF.
    // Simplification, documented: the relative-pose delta is added in ZED's
    // own reported frame WITHOUT extrinsic rotation correction — the
    // hesai_lidar<->ZED extrinsic is contested in this workspace (URDF vs
    // calibration_zed.urdf_joint.xml target different frames and disagree,
    // see plan/report). Given the intentionally loose sigma below, the
    // resulting few-degree misalignment is absorbed by the noise model.
    std::size_t zed_odom_count = 0;
    if (zed_odom.size() > 1) {
      const auto noise = robust(
          gtsam::noiseModel::Diagonal::Sigmas(
              (gtsam::Vector6() << args.zed_odom_sigma_rot, args.zed_odom_sigma_rot, args.zed_odom_sigma_rot,
                                   args.zed_odom_sigma_xy, args.zed_odom_sigma_xy, args.zed_odom_sigma_xy).finished()),
          args.robust_k);
      std::size_t prev_key = nearest_index(imu_t, zed_odom.d(0, "t"));
      gtsam::Pose3 prev_pose = pose_from_row(zed_odom, 0);
      for (std::size_t i = 1; i < zed_odom.size(); ++i) {
        const std::size_t curr_key = nearest_index(imu_t, zed_odom.d(i, "t"));
        if (curr_key == prev_key) continue;
        const double dt = imu_t[curr_key] - imu_t[prev_key];
        if (dt > 3.0) { prev_key = curr_key; prev_pose = pose_from_row(zed_odom, i); continue; }  // camera dropout gap
        const gtsam::Pose3 curr_pose = pose_from_row(zed_odom, i);
        factors_at_key[curr_key].add(gtsam::BetweenFactor<gtsam::Pose3>(X(prev_key), X(curr_key),
            prev_pose.between(curr_pose), noise));
        prev_key = curr_key;
        prev_pose = curr_pose;
        ++zed_odom_count;
      }
    }
    std::cout << "ZED odom BetweenFactor: " << zed_odom_count << "\n";

    // Isaac ROS Visual SLAM (cuVSLAM) odometry: consecutive-sample BetweenFactor,
    // same structure as zed_odom above (full 6DOF, no extrinsic correction on the
    // relative delta -- the isaac_map/isaac_odom frame's own axes don't need to
    // align with map, only the RELATIVE motion between samples is used). Tighter
    // sigma than zed_odom (see isaac_vslam_sigma_xy/rot above) because cuVSLAM
    // does local bundle adjustment, not raw frame-to-frame VO -- but still
    // unvalidated against real data on this workspace as of this writing.
    std::size_t isaac_vslam_count = 0;
    if (isaac_vslam.size() > 1) {
      const auto noise = robust(
          gtsam::noiseModel::Diagonal::Sigmas(
              (gtsam::Vector6() << args.isaac_vslam_sigma_rot, args.isaac_vslam_sigma_rot, args.isaac_vslam_sigma_rot,
                                   args.isaac_vslam_sigma_xy, args.isaac_vslam_sigma_xy, args.isaac_vslam_sigma_xy).finished()),
          args.robust_k);
      std::size_t prev_key = nearest_index(imu_t, isaac_vslam.d(0, "t"));
      gtsam::Pose3 prev_pose = pose_from_row(isaac_vslam, 0);
      for (std::size_t i = 1; i < isaac_vslam.size(); ++i) {
        const std::size_t curr_key = nearest_index(imu_t, isaac_vslam.d(i, "t"));
        if (curr_key == prev_key) continue;
        const double dt = imu_t[curr_key] - imu_t[prev_key];
        if (dt > 3.0) { prev_key = curr_key; prev_pose = pose_from_row(isaac_vslam, i); continue; }  // tracking-loss gap
        const gtsam::Pose3 curr_pose = pose_from_row(isaac_vslam, i);
        factors_at_key[curr_key].add(gtsam::BetweenFactor<gtsam::Pose3>(X(prev_key), X(curr_key),
            prev_pose.between(curr_pose), noise));
        prev_key = curr_key;
        prev_pose = curr_pose;
        ++isaac_vslam_count;
      }
    }
    std::cout << "Isaac VSLAM BetweenFactor: " << isaac_vslam_count << "\n";

    // ── Revisit (loop) candidate detection — DIAGNOSTIC ONLY, not added as a
    // graph factor (see file header for why that would be circular here).
    // Reported so qualification can measure return-pose error at revisits. ──
    struct Revisit { std::size_t i, j; double dist_m, dt_s; };
    std::vector<Revisit> revisits;
    {
      const double d2_thresh = args.loop_closure_dist_m * args.loop_closure_dist_m;
      const double cell = args.loop_closure_dist_m;
      std::unordered_map<int64_t, std::vector<std::size_t>> grid;
      auto cell_key = [&](double x, double y) -> int64_t {
        int64_t ix = static_cast<int64_t>(std::floor(x / cell));
        int64_t iy = static_cast<int64_t>(std::floor(y / cell));
        return (ix << 32) ^ (iy & 0xffffffffLL);
      };
      // At a rink, a slow pass near a previously-visited spot matches dozens of
      // earlier rows at ~20 Hz — keep only the SINGLE closest earlier match per
      // later row (not every match in radius), and require the recorded events
      // themselves to be spaced apart so the CSV stays one-row-per-revisit
      // instead of one-row-per-ICP-sample-during-the-revisit.
      std::size_t last_recorded_i = 0;
      bool have_last = false;
      for (std::size_t i = 0; i < icp.size(); ++i) {
        const double xi = icp.d(i, "x"), yi = icp.d(i, "y"), ti = icp.d(i, "t");
        std::size_t best_j = 0;
        double best_d2 = d2_thresh;
        bool found = false;
        for (int dx = -1; dx <= 1; ++dx) {
          for (int dy = -1; dy <= 1; ++dy) {
            int64_t ix = static_cast<int64_t>(std::floor(xi / cell)) + dx;
            int64_t iy = static_cast<int64_t>(std::floor(yi / cell)) + dy;
            auto it = grid.find((ix << 32) ^ (iy & 0xffffffffLL));
            if (it == grid.end()) continue;
            for (std::size_t j : it->second) {
              const double tj = icp.d(j, "t");
              if (ti - tj < args.loop_closure_min_dt_s) continue;
              const double dxp = xi - icp.d(j, "x"), dyp = yi - icp.d(j, "y");
              const double d2 = dxp * dxp + dyp * dyp;
              if (d2 <= best_d2) { best_d2 = d2; best_j = j; found = true; }
            }
          }
        }
        if (found && (!have_last || i - last_recorded_i >= 20)) {
          revisits.push_back({best_j, i, std::sqrt(best_d2), ti - icp.d(best_j, "t")});
          last_recorded_i = i;
          have_last = true;
        }
        grid[cell_key(xi, yi)].push_back(i);
      }
    }
    std::cout << "Revisit candidates (diagnostic, deduplicated): " << revisits.size() << "\n";

    // ── Solve: incremental ISAM2, single forward pass in batches ──
    gtsam::ISAM2Params isam2_params;
    isam2_params.relinearizeThreshold = 0.1;
    isam2_params.relinearizeSkip = 10;
    gtsam::ISAM2 isam2(isam2_params);

    gtsam::NonlinearFactorGraph batch_factors;
    gtsam::Values batch_values;

    const gtsam::Pose3 pose0 = interp_pose_at(imu_t[0]);
    const gtsam::Vector3 vel0 = vel_world_guess_at(imu_t[0], pose0.rotation());
    // Light prior at k=0 only seeds the gap before the first ICP sample
    // arrives — the dense ICP PriorFactor<Pose3> stream gauge-fixes the rest
    // of the run (see module docstring's "Gauge" note).
    batch_factors.addPrior(X(0), pose0,
        gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector6() << 1.0, 1.0, 0.1, 0.3, 0.3, 0.3).finished()));
    batch_factors.addPrior(V(0), vel0, gtsam::noiseModel::Isotropic::Sigma(3, 0.5));
    batch_factors.addPrior(B(0), gtsam::imuBias::ConstantBias(), gtsam::noiseModel::Isotropic::Sigma(6, 1e-3));
    batch_values.insert(X(0), pose0);
    batch_values.insert(V(0), vel0);
    batch_values.insert(B(0), gtsam::imuBias::ConstantBias());
    if (use_articulation) {
      batch_factors.addPrior(H_key(0), 0.0, gtsam::noiseModel::Isotropic::Sigma(1, args.phi_prior_sigma));
      batch_factors.addPrior(P_key(0), 0.0, gtsam::noiseModel::Isotropic::Sigma(1, args.pitch_prior_sigma));
      batch_values.insert(H_key(0), 0.0);
      batch_values.insert(P_key(0), 0.0);
    }
    batch_factors.add(factors_at_key[0]);

    gtsam::imuBias::ConstantBias last_bias;
    double last_phi = 0.0, last_alpha = 0.0;
    std::size_t imu_factor_count = 0;
    std::size_t imu_gap_count = 0;
    const std::size_t batch_size = static_cast<std::size_t>(std::max(1, args.isam2_batch_size));
    std::size_t total_updates = 0;

    for (std::size_t k = 0; k + 1 < N; ++k) {
      const double dt = imu_t[k + 1] - imu_t[k];
      if (dt > 0.0 && dt <= 1.0) {
        gtsam::PreintegratedCombinedMeasurements preint(imu_params, last_bias);
        const gtsam::Vector3 acc(imu.d(k, "ax"), imu.d(k, "ay"), imu.d(k, "az"));
        const gtsam::Vector3 gyro(imu.d(k, "gx"), imu.d(k, "gy"), imu.d(k, "gz"));
        preint.integrateMeasurement(acc, gyro, dt);
        batch_factors.add(gtsam::CombinedImuFactor(X(k), V(k), X(k + 1), V(k + 1), B(k), B(k + 1), preint));
        ++imu_factor_count;
      } else {
        // Non-monotonic or gap — no IMU factor for this link.
        ++imu_gap_count;
      }
      // Seed X(k+1)/V(k+1) from ICP interpolation (never as a factor/output,
      // see the block above pose0/vel0 for rationale). X/V/B(k+1) and H/P(k+1)
      // below are still seeded and chained unconditionally regardless of the
      // IMU-factor branch above, so no key is ever left referenced-but-
      // uninitialized.
      const gtsam::Pose3 seed_pose = interp_pose_at(imu_t[k + 1]);
      const gtsam::Vector3 seed_vel = vel_world_guess_at(imu_t[k + 1], seed_pose.rotation());
      batch_values.insert(X(k + 1), seed_pose);
      batch_values.insert(V(k + 1), seed_vel);
      batch_values.insert(B(k + 1), last_bias);

      if (use_articulation) {
        batch_values.insert(H_key(k + 1), last_phi);
        batch_values.insert(P_key(k + 1), last_alpha);
        batch_factors.add(gtsam::BetweenFactor<double>(H_key(k), H_key(k + 1), 0.0,
            gtsam::noiseModel::Isotropic::Sigma(1, args.phi_sigma_dynamics)));
        batch_factors.add(gtsam::BetweenFactor<double>(P_key(k), P_key(k + 1), 0.0,
            gtsam::noiseModel::Isotropic::Sigma(1, args.pitch_sigma_dynamics)));
      }

      batch_factors.add(factors_at_key[k + 1]);

      const bool last_step = (k + 2 == N);
      if (((k + 1) % batch_size == 0) || last_step) {
        isam2.update(batch_factors, batch_values);
        ++total_updates;
        batch_factors = gtsam::NonlinearFactorGraph();
        batch_values.clear();
        last_bias = isam2.calculateEstimate<gtsam::imuBias::ConstantBias>(B(k + 1));
        if (use_articulation) {
          last_phi = isam2.calculateEstimate<double>(H_key(k + 1));
          last_alpha = isam2.calculateEstimate<double>(P_key(k + 1));
        }
        if (total_updates % 50 == 0) {
          std::cout << "  ISAM2 update " << total_updates << " (keyframe " << (k + 1) << "/" << N << ")\n";
        }
      }
    }
    std::cout << "IMU (CombinedImuFactor) factors: " << imu_factor_count
              << " (" << imu_gap_count << " links skipped: gap/non-monotonic dt)\n";
    std::cout << "ISAM2 updates: " << total_updates << " (batch size " << batch_size << ")\n";

    const gtsam::Values result = isam2.calculateEstimate();
    const gtsam::NonlinearFactorGraph& full_graph = isam2.getFactorsUnsafe();
    const double final_error = full_graph.error(result);
    std::cout << "Final error: " << final_error << " (factors=" << full_graph.size()
              << ", variables=" << result.size() << ")\n";

    // ── Marginals via ISAM2's own Bayes tree (cheap — reuses the already-
    // factored tree, unlike reconstructing gtsam::Marginals from scratch,
    // which is itself part of what made the one-shot approach expensive). ──
    bool marginals_ok = !args.skip_marginals;

    // ── Revisit return-error against the OPTIMIZED trajectory (real check) ──
    std::ofstream revisit_csv(args.output_dir + "/loop_closures.csv");
    revisit_csv << "icp_row_i,icp_row_j,t_i,t_j,dt_s,dist_m_icp_raw,"
                   "return_trans_err_m,return_rot_err_deg\n";
    revisit_csv << std::setprecision(12);
    for (const auto& r : revisits) {
      const std::size_t key_i = icp_key_of_row[r.i];
      const std::size_t key_j = icp_key_of_row[r.j];
      const gtsam::Pose3 pi = result.at<gtsam::Pose3>(X(key_i));
      const gtsam::Pose3 pj = result.at<gtsam::Pose3>(X(key_j));
      const gtsam::Vector6 xi = gtsam::Pose3::Logmap(pi.between(pj));
      revisit_csv << r.i << ',' << r.j << ',' << icp.d(r.i, "t") << ',' << icp.d(r.j, "t") << ','
                  << r.dt_s << ',' << r.dist_m << ','
                  << xi.tail<3>().norm() << ',' << (xi.head<3>().norm() * 180.0 / M_PI) << '\n';
    }
    revisit_csv.close();

    // ── optimized_trajectory.csv — native rate = IMU rate (no gap to interpolate) ──
    std::ofstream traj(args.output_dir + "/optimized_trajectory.csv");
    traj << std::setprecision(15);
    traj << "t,x,y,z,qx,qy,qz,qw,vx_world,vy_world,vz_world,vx_body,vy_body,vz_body,"
            "wx_body,wy_body,wz_body,bax,bay,baz,bgx,bgy,bgz,phi_rad,alpha_rad,"
            "sigma_x,sigma_y,sigma_z,sigma_roll,sigma_pitch,sigma_yaw,has_marginals\n";
    const std::size_t marg_stride = static_cast<std::size_t>(std::max(1, args.marginals_stride));
    for (std::size_t k = 0; k < N; ++k) {
      const gtsam::Pose3 pose = result.at<gtsam::Pose3>(X(k));
      const gtsam::Vector3 vel_world = result.at<gtsam::Vector3>(V(k));
      const gtsam::Vector3 vel_body = pose.rotation().unrotate(vel_world);
      const auto bias = result.at<gtsam::imuBias::ConstantBias>(B(k));
      const gtsam::Vector3 acc_bias = bias.accelerometer();
      const gtsam::Vector3 gyro_bias = bias.gyroscope();
      gtsam::Vector3 wb(0, 0, 0);
      if (k + 1 < N) {
        wb = gtsam::Vector3(imu.d(k, "gx"), imu.d(k, "gy"), imu.d(k, "gz")) - gyro_bias;
      } else if (k > 0) {
        wb = gtsam::Vector3(imu.d(k - 1, "gx"), imu.d(k - 1, "gy"), imu.d(k - 1, "gz")) - gyro_bias;
      }
      const double phi = use_articulation ? result.at<double>(H_key(k)) : nan_d();
      const double alpha = use_articulation ? result.at<double>(P_key(k)) : nan_d();

      double sx = nan_d(), sy = nan_d(), sz = nan_d(), sroll = nan_d(), spitch = nan_d(), syaw = nan_d();
      bool has_marg = false;
      if (marginals_ok && (k % marg_stride == 0)) {
        try {
          const gtsam::Matrix66 cov = isam2.marginalCovariance(X(k));
          sroll = std::sqrt(std::max(0.0, cov(0, 0)));
          spitch = std::sqrt(std::max(0.0, cov(1, 1)));
          syaw = std::sqrt(std::max(0.0, cov(2, 2)));
          sx = std::sqrt(std::max(0.0, cov(3, 3)));
          sy = std::sqrt(std::max(0.0, cov(4, 4)));
          sz = std::sqrt(std::max(0.0, cov(5, 5)));
          has_marg = true;
        } catch (const std::exception&) { /* leave NaN */ }
      }

      const auto q = pose.rotation().toQuaternion();
      const auto p = pose.translation();
      traj << imu_t[k] << ',' << p.x() << ',' << p.y() << ',' << p.z() << ','
           << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w() << ','
           << vel_world.x() << ',' << vel_world.y() << ',' << vel_world.z() << ','
           << vel_body.x() << ',' << vel_body.y() << ',' << vel_body.z() << ','
           << wb.x() << ',' << wb.y() << ',' << wb.z() << ','
           << acc_bias.x() << ',' << acc_bias.y() << ',' << acc_bias.z() << ','
           << gyro_bias.x() << ',' << gyro_bias.y() << ',' << gyro_bias.z() << ','
           << phi << ',' << alpha << ','
           << sx << ',' << sy << ',' << sz << ',' << sroll << ',' << spitch << ',' << syaw << ','
           << (has_marg ? 1 : 0) << '\n';
    }
    traj.close();

    // ── Trailer pose (kinematic only — no 6-DOF LiDAR trailer pose in this bag) ──
    std::ofstream trailer(args.output_dir + "/trailer_pose_kinematic.csv");
    trailer << std::setprecision(15);
    trailer << "t,x,y,z,qx,qy,qz,qw\n";
    if (use_articulation) {
      for (std::size_t k = 0; k < N; ++k) {
        const gtsam::Pose3 pose = result.at<gtsam::Pose3>(X(k));
        const double phi = result.at<double>(H_key(k));
        const double alpha = result.at<double>(P_key(k));
        const gtsam::Pose3 delta = mtt_loc::hitch_kinematics::computeDelta(phi, alpha);
        const gtsam::Pose3 trailer_pose = pose.compose(delta);
        const auto q = trailer_pose.rotation().toQuaternion();
        const auto p = trailer_pose.translation();
        trailer << imu_t[k] << ',' << p.x() << ',' << p.y() << ',' << p.z() << ','
                << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w() << '\n';
      }
    }
    trailer.close();

    // ── Summary ──
    std::ofstream summary(args.output_dir + "/solver_summary.yaml");
    summary << "status: ok\n";
    summary << "keyframe_count: " << N << "\n";
    summary << "keyframe_rate_hz: " << (N > 1 ? (N - 1) / (imu_t.back() - imu_t.front()) : 0.0) << "\n";
    summary << "solver_backend: isam2_incremental_single_forward_pass\n";
    summary << "isam2_batch_size: " << batch_size << "\n";
    summary << "isam2_update_calls: " << total_updates << "\n";
    summary << "factor_count: " << full_graph.size() << "\n";
    summary << "variable_count: " << result.size() << "\n";
    summary << "final_error: " << final_error << "\n";
    summary << "imu_factor_count: " << imu_factor_count << "\n";
    summary << "icp_prior_factor_count: " << icp_prior_count << "\n";
    summary << "icp_sigma_xy_m: " << icp_sigma_xy << "\n";
    summary << "icp_sigma_z_m: " << icp_sigma_z << "\n";
    summary << "icp_sigma_source: " << (args.icp_sigma_from_data ? "measured_jitter_floored" : "cli_override") << "\n";
    summary << "track_odom_between_factor_count: " << track_odom_count << "\n";
    summary << "zed_odom_between_factor_count: " << zed_odom_count << "\n";
    summary << "isaac_vslam_between_factor_count: " << isaac_vslam_count << "\n";
    summary << "phi_hardware_prior_count: " << phi_hw_count << "\n";
    summary << "phi_lidar_prior_count: " << phi_lidar_count << "\n";
    summary << "pitch_prior_count: " << pitch_count << "\n";
    summary << "revisit_candidate_count: " << revisits.size() << "\n";
    summary << "marginals_computed: " << (marginals_ok ? "true" : "false") << "\n";
    summary << "marginals_stride: " << marg_stride << "\n";
    summary << "notes: >\n";
    summary << "  Solved with gtsam::ISAM2 in a single forward pass over the whole bag (not\n";
    summary << "  causal/online) because one-shot batch LM segfaults at this scale regardless\n";
    summary << "  of initial-value quality (verified at 50k vs 273k variables). Initial values\n";
    summary << "  for X(k)/V(k) come from ICP interpolation, not open-loop IMU integration —\n";
    summary << "  see the sanity-check line in the run log. ICP is a dense absolute\n";
    summary << "  PriorFactor<Pose3> stream, not a BetweenFactor chain, because GT_icp is\n";
    summary << "  scan-to-persistent-map (not frame-to-frame); no separate loop-closure\n";
    summary << "  factors are added (would be circular against the same source) — see\n";
    summary << "  loop_closures.csv for the measured return-pose error at detected revisits.\n";

    std::cout << "offline_reference_solver OK -> " << args.output_dir << "\n";
    return 0;
  } catch (const std::exception& exc) {
    std::cerr << "offline_reference_solver ERROR: " << exc.what() << "\n";
    return 1;
  }
}

namespace {
struct ThreadArgs {
  int argc;
  char** argv;
  int result;
};

void* run_thread_entry(void* raw_args) {
  auto* targs = static_cast<ThreadArgs*>(raw_args);
  targs->result = run(targs->argc, targs->argv);
  return nullptr;
}
}  // namespace

// GTSAM's BayesTree operations (separatorMarginal, clique traversal, etc. —
// exercised by ISAM2::update()/marginalCovariance()) recurse to a depth
// proportional to the pose-chain length. This is a documented, reported
// GTSAM limitation for long sequential problems (users have hit stack
// overflows in these exact recursive clique operations on chains as short as
// a few thousand nodes on constrained stacks); at this executable's scale
// (~54k sequential keyframes for a 546 s bag at 100 Hz) it reliably overflows
// the OS-default ~8 MiB thread stack, confirmed via gdb: SIGSEGV inside
// gtsam::BayesTreeCliqueBase::separatorMarginal, tens of thousands of
// identical recursive frames deep.
//
// Rather than depend on every caller remembering to raise the shell's
// `ulimit -s` before invoking this binary, run the actual solve on a
// dedicated thread created with an explicit large stack. A 2 GiB stack is a
// virtual reservation (demand-paged on Linux), not resident memory paid
// upfront, so this costs nothing when unused and comfortably covers the
// deepest recursion this graph exercises (empirically verified against a
// 2 GiB ulimit on the full 54,606-keyframe bag before this was folded into
// the binary itself).
int main(int argc, char** argv) {
  constexpr std::size_t kWorkerStackBytes = 2ULL * 1024 * 1024 * 1024;  // 2 GiB

  pthread_attr_t attr;
  if (pthread_attr_init(&attr) != 0) {
    std::cerr << "offline_reference_solver ERROR: pthread_attr_init failed\n";
    return 1;
  }
  if (pthread_attr_setstacksize(&attr, kWorkerStackBytes) != 0) {
    std::cerr << "offline_reference_solver WARNING: pthread_attr_setstacksize failed, "
                 "continuing with the default stack size (may segfault on long bags).\n";
  }

  ThreadArgs targs{argc, argv, 1};
  pthread_t worker;
  const int create_rc = pthread_create(&worker, &attr, run_thread_entry, &targs);
  pthread_attr_destroy(&attr);

  if (create_rc != 0) {
    std::cerr << "offline_reference_solver WARNING: pthread_create failed (errno=" << create_rc
              << ") — running on the default-size main-thread stack instead; this WILL "
                 "segfault on long bags. Run with a raised `ulimit -s` if it does.\n";
    return run(argc, argv);
  }

  pthread_join(worker, nullptr);
  return targs.result;
}
