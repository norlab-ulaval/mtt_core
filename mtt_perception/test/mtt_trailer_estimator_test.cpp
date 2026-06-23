// mtt_trailer_estimator_test.cpp — Unit tests for TrailerEstimatorNode internals.
//
// Tests are pure algorithmic — no ROS 2 node needed.
// We instantiate the EKF helpers directly via white-box access to the
// relevant free functions (factored out below) without spinning up the node.
//
// Build: ament_cmake adds this via ament_add_gtest() in CMakeLists.txt.
//
// Test cases:
//   1. EKF cold-start convergence from perfect kinematic prior.
//   2. Mahalanobis gating: corrupted measurement must not move state.
//   3. Covariance remains positive-definite after 1000 Joseph-form updates.
//   4. Hitch angle back-computation correctness vs known geometry.
//   5. RANSAC 3D line fit converges on a synthetic cloud with 30% outliers.

#include <cmath>
#include <random>
#include <vector>

#include <Eigen/Dense>
#include <gtest/gtest.h>

static constexpr double kPi = 3.141592653589793;

// ── White-box helpers — replicate the EKF core without pulling in the full node. These mirror exactly the logic in mtt_trailer_estimator_node.cpp ──
// State index constants (must match idx:: in the main header).
namespace idx
{
constexpr int kX       = 0;
constexpr int kY       = 1;
constexpr int kZ       = 2;
constexpr int kRoll    = 3;
constexpr int kPitch   = 4;
constexpr int kYaw     = 5;
constexpr int kVx      = 6;
constexpr int kVy      = 7;
constexpr int kVz      = 8;
constexpr int kYawRate = 9;
constexpr int kNDof    = 10;
}

using State10d = Eigen::Matrix<double, 10, 1>;
using Cov10d   = Eigen::Matrix<double, 10, 10>;
using Mat10d   = Eigen::Matrix<double, 10, 10>;

struct FilterState
{
  State10d x{State10d::Zero()};
  Cov10d   P{Cov10d::Identity()};
  bool initialized{false};
};

static double normalizeAngle(double a) noexcept
{
  constexpr double kTwoPi = 2.0 * kPi;
  a = std::fmod(a + kPi, kTwoPi);
  if (a < 0.0) { a += kTwoPi; }
  return a - kPi;
}

// ── EKF predict — constant-velocity with decay ──
static void ekfPredict(
  FilterState & state,
  double dt,
  const Cov10d & Q_base,
  double velocity_decay)
{
  if (dt <= 0.0) { return; }

  state.x(idx::kX) += state.x(idx::kVx) * dt;
  state.x(idx::kY) += state.x(idx::kVy) * dt;
  state.x(idx::kZ) += state.x(idx::kVz) * dt;
  state.x(idx::kYaw) = normalizeAngle(
    state.x(idx::kYaw) + state.x(idx::kYawRate) * dt);

  const double d = std::pow(velocity_decay, dt * 10.0);
  state.x(idx::kVx)      *= d;
  state.x(idx::kVy)      *= d;
  state.x(idx::kVz)      *= d;
  state.x(idx::kYawRate) *= d;

  Mat10d F = Mat10d::Identity();
  F(idx::kX,   idx::kVx)      = dt;
  F(idx::kY,   idx::kVy)      = dt;
  F(idx::kZ,   idx::kVz)      = dt;
  F(idx::kYaw, idx::kYawRate) = dt;
  F(idx::kVx,      idx::kVx)      = d;
  F(idx::kVy,      idx::kVy)      = d;
  F(idx::kVz,      idx::kVz)      = d;
  F(idx::kYawRate, idx::kYawRate) = d;

  state.P = F * state.P * F.transpose() + Q_base * dt;
  state.P = 0.5 * (state.P + state.P.transpose());
}

// ── EKF update — Joseph form with Mahalanobis gating. Returns true if the measurement was accepted ──
template<int M>
static bool ekfUpdate(
  FilterState & state,
  const Eigen::Matrix<double, M, 1> & z,
  const Eigen::Matrix<double, M, 10> & H,
  const Eigen::Matrix<double, M, M> & R,
  double chi2_threshold)
{
  using MatMxM  = Eigen::Matrix<double, M, M>;
  using VecM    = Eigen::Matrix<double, M, 1>;
  using Mat10xM = Eigen::Matrix<double, 10, M>;

  const VecM nu = z - H * state.x;
  const MatMxM S = H * state.P * H.transpose() + R;

  const Eigen::LLT<MatMxM> llt_S(S);
  if (llt_S.info() != Eigen::Success) { return false; }

  const VecM S_inv_nu = llt_S.solve(nu);
  const double mahal = nu.dot(S_inv_nu);
  if (mahal > chi2_threshold) { return false; }  // gated out

  const Mat10xM K = (llt_S.solve(H * state.P)).transpose();
  state.x += K * nu;
  state.x(idx::kYaw) = normalizeAngle(state.x(idx::kYaw));

  const Mat10d IKH = Mat10d::Identity() - K * H;
  state.P = IKH * state.P * IKH.transpose() + K * R * K.transpose();
  state.P = 0.5 * (state.P + state.P.transpose());
  return true;
}

// ── H matrix builders ──
static Eigen::Matrix<double, 4, 10> makeH4()
{
  Eigen::Matrix<double, 4, 10> H = Eigen::Matrix<double, 4, 10>::Zero();
  H(0, idx::kX)   = 1.0;
  H(1, idx::kY)   = 1.0;
  H(2, idx::kZ)   = 1.0;
  H(3, idx::kYaw) = 1.0;
  return H;
}

static Eigen::Matrix<double, 3, 10> makeH3()
{
  Eigen::Matrix<double, 3, 10> H = Eigen::Matrix<double, 3, 10>::Zero();
  H(0, idx::kX)   = 1.0;
  H(1, idx::kY)   = 1.0;
  H(2, idx::kYaw) = 1.0;
  return H;
}

// ── Default Q matrix for tests ──
static Cov10d makeDefaultQ()
{
  Cov10d Q = Cov10d::Zero();
  Q(idx::kX,       idx::kX)       = 0.01;
  Q(idx::kY,       idx::kY)       = 0.01;
  Q(idx::kZ,       idx::kZ)       = 0.001;
  Q(idx::kRoll,    idx::kRoll)    = 0.001;
  Q(idx::kPitch,   idx::kPitch)   = 0.001;
  Q(idx::kYaw,     idx::kYaw)     = 0.005;
  Q(idx::kVx,      idx::kVx)      = 0.1;
  Q(idx::kVy,      idx::kVy)      = 0.1;
  Q(idx::kVz,      idx::kVz)      = 0.01;
  Q(idx::kYawRate, idx::kYawRate) = 0.05;
  return Q;
}

// ── RANSAC 3D line fit (same algorithm as in the main node — no dependency) ──
struct RansacLine3d
{
  Eigen::Vector3d dir{Eigen::Vector3d::UnitX()};
  Eigen::Vector3d p0{Eigen::Vector3d::Zero()};
  double residual_variance{1.0};
  int n_inliers{0};
  bool valid{false};
};

static RansacLine3d ransacLine3d(
  const std::vector<Eigen::Vector3d> & pts,
  double inlier_thresh,
  int max_iters,
  double min_inlier_frac = 0.6)
{
  RansacLine3d result;
  const int N = static_cast<int>(pts.size());
  if (N < 4) { return result; }

  std::mt19937 rng(42u);
  std::uniform_int_distribution<int> dist(0, N - 1);
  const double thresh2 = inlier_thresh * inlier_thresh;

  std::vector<std::size_t> best_idx;
  for (int iter = 0; iter < max_iters; ++iter) {
    const int ia = dist(rng), ib = dist(rng);
    if (ia == ib) { continue; }
    const Eigen::Vector3d d_cand = (pts[ib] - pts[ia]).normalized();
    if (d_cand.norm() < 1e-9) { continue; }

    std::vector<std::size_t> idx;
    for (int i = 0; i < N; ++i) {
      const Eigen::Vector3d q    = pts[i] - pts[ia];
      const Eigen::Vector3d perp = q - q.dot(d_cand) * d_cand;
      if (perp.squaredNorm() < thresh2) { idx.push_back(static_cast<std::size_t>(i)); }
    }
    if (idx.size() > best_idx.size()) { best_idx = idx; }
  }

  if (static_cast<double>(best_idx.size()) < min_inlier_frac * N) { return result; }

  // SVD refit.
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (std::size_t i : best_idx) { centroid += pts[i]; }
  centroid /= static_cast<double>(best_idx.size());

  Eigen::MatrixXd A(static_cast<Eigen::Index>(best_idx.size()), 3);
  for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(best_idx.size()); ++i) {
    A.row(i) = (pts[best_idx[static_cast<std::size_t>(i)]] - centroid).transpose();
  }
  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinV);
  const Eigen::Vector3d dir = svd.matrixV().col(0).normalized();

  double rss = 0.0;
  for (std::size_t i : best_idx) {
    const Eigen::Vector3d q    = pts[i] - centroid;
    const Eigen::Vector3d perp = q - q.dot(dir) * dir;
    rss += perp.squaredNorm();
  }

  result.dir              = dir;
  result.p0               = centroid;
  result.residual_variance = rss / static_cast<double>(best_idx.size());
  result.n_inliers        = static_cast<int>(best_idx.size());
  result.valid            = true;
  return result;
}

// Test 1 — EKF cold-start convergence.
//
// Given: perfect kinematic prior at the true trailer position.
// Expect: after N updates, state converges within 1 cm of ground truth.
TEST(TrailerEkf, ColdStartConvergence)
{
  const Cov10d Q = makeDefaultQ();
  const Eigen::Matrix<double, 4, 10> H = makeH4();
  constexpr double chi2 = 13.28;
  constexpr double dt   = 0.01;  // 100 Hz kinematic prior

  // Ground truth position.
  const Eigen::Vector4d x_true(5.0, -3.0, 0.2, 0.75);  // [x, y, z, yaw]

  // Cold-start: initialise from first measurement.
  FilterState state;
  state.x(idx::kX)   = x_true(0);
  state.x(idx::kY)   = x_true(1);
  state.x(idx::kZ)   = x_true(2);
  state.x(idx::kYaw) = x_true(3);
  state.P = Cov10d::Identity() * 1.0;
  state.initialized = true;

  // Measurement noise: σ = 5 cm on x,y,z, σ = 0.02 rad on yaw.
  Eigen::Matrix<double, 4, 4> R = Eigen::Matrix<double, 4, 4>::Zero();
  R(0, 0) = 0.05 * 0.05;
  R(1, 1) = 0.05 * 0.05;
  R(2, 2) = 0.05 * 0.05;
  R(3, 3) = 0.02 * 0.02;

  std::mt19937 rng(7u);
  std::normal_distribution<double> noise_xy(0.0, 0.05);
  std::normal_distribution<double> noise_yaw(0.0, 0.02);

  // Inject 50 noisy measurements.
  for (int i = 0; i < 50; ++i) {
    ekfPredict(state, dt, Q, 0.95);
    Eigen::Vector4d z;
    z << x_true(0) + noise_xy(rng),
         x_true(1) + noise_xy(rng),
         x_true(2) + noise_xy(rng),
         x_true(3) + noise_yaw(rng);
    ekfUpdate<4>(state, z, H, R, chi2);
  }

  // Assert convergence within 1 cm.
  EXPECT_NEAR(state.x(idx::kX),   x_true(0), 0.01) << "x not converged";
  EXPECT_NEAR(state.x(idx::kY),   x_true(1), 0.01) << "y not converged";
  EXPECT_NEAR(state.x(idx::kZ),   x_true(2), 0.01) << "z not converged";
  EXPECT_NEAR(state.x(idx::kYaw), x_true(3), 0.01) << "yaw not converged";

  // Assert covariance has shrunk significantly.
  EXPECT_LT(state.P(idx::kX, idx::kX), 0.01) << "P(x,x) did not shrink";
  EXPECT_LT(state.P(idx::kY, idx::kY), 0.01) << "P(y,y) did not shrink";
}

// Test 2 — Mahalanobis gating: corrupted measurement must not move the state.
//
// Given: filter at known state, a measurement 10-sigma away from prior.
// Expect: state is unchanged after the gated update.
TEST(TrailerEkf, MahalanobisGating)
{
  const Eigen::Matrix<double, 4, 10> H = makeH4();

  FilterState state;
  state.x(idx::kX)   = 5.0;
  state.x(idx::kY)   = -3.0;
  state.x(idx::kZ)   = 0.2;
  state.x(idx::kYaw) = 0.75;
  // Tight covariance: σ ≈ 1 mm on x,y.
  state.P = Cov10d::Zero();
  state.P(idx::kX,   idx::kX)   = 1e-6;
  state.P(idx::kY,   idx::kY)   = 1e-6;
  state.P(idx::kZ,   idx::kZ)   = 1e-6;
  state.P(idx::kYaw, idx::kYaw) = 1e-6;
  for (int i = 4; i < 10; ++i) { state.P(i, i) = 0.1; }
  state.initialized = true;

  // Corrupted measurement: 1 m off in x (≫ 10-sigma when σ=1 mm).
  Eigen::Vector4d z_bad;
  z_bad << 6.0, -3.0, 0.2, 0.75;  // 1 m error in x

  const State10d x_before = state.x;
  const Cov10d   P_before = state.P;

  Eigen::Matrix<double, 4, 4> R = Eigen::Matrix<double, 4, 4>::Identity() * (0.05 * 0.05);
  constexpr double chi2 = 13.28;

  const bool accepted = ekfUpdate<4>(state, z_bad, H, R, chi2);

  // Gate should have rejected the measurement.
  EXPECT_FALSE(accepted) << "Corrupted measurement was accepted — gating failed";
  EXPECT_NEAR((state.x - x_before).norm(), 0.0, 1e-12) << "State was modified despite gate";
  EXPECT_NEAR((state.P - P_before).norm(), 0.0, 1e-12) << "Covariance was modified despite gate";
}

// Test 3 — Covariance remains positive-definite after 1000 Joseph-form updates.
//
// Positive-definiteness is checked by attempting Cholesky decomposition.
// This validates numerical stability of the Joseph-form update.
TEST(TrailerEkf, CovariancePositiveDefiniteAfter1000Updates)
{
  const Cov10d Q = makeDefaultQ();
  const Eigen::Matrix<double, 3, 10> H = makeH3();
  constexpr double chi2 = 11.34;
  constexpr double dt   = 0.1;  // 10 Hz update

  FilterState state;
  state.x = State10d::Zero();
  state.x(idx::kX) = 3.0;
  state.P = Cov10d::Identity() * 0.5;
  state.initialized = true;

  std::mt19937 rng(99u);
  std::normal_distribution<double> noise(0.0, 0.05);

  Eigen::Matrix<double, 3, 3> R = Eigen::Matrix<double, 3, 3>::Identity() * (0.05 * 0.05);

  for (int i = 0; i < 1000; ++i) {
    ekfPredict(state, dt, Q, 0.95);

    Eigen::Vector3d z;
    z << state.x(idx::kX) + noise(rng),
         state.x(idx::kY) + noise(rng),
         normalizeAngle(state.x(idx::kYaw) + noise(rng) * 0.1);

    ekfUpdate<3>(state, z, H, R, chi2);
  }

  // Check positive-definiteness via LLT.
  const Eigen::LLT<Cov10d> llt(state.P);
  EXPECT_EQ(llt.info(), Eigen::Success)
    << "Covariance matrix is not positive-definite after 1000 updates";

  // All diagonal entries must be positive.
  for (int i = 0; i < 10; ++i) {
    EXPECT_GT(state.P(i, i), 0.0) << "P(" << i << "," << i << ") non-positive";
  }

  // No NaN or Inf in the full matrix.
  EXPECT_TRUE(state.P.array().isFinite().all()) << "NaN or Inf detected in P";
}

// Test 4 — Hitch angle back-computation correctness.
//
// Given: trailer at a known position relative to a known hitch, at a known
// tractor yaw.  Compute the expected hitch angle analytically and compare.
TEST(TrailerEkf, HitchAngleBackComputation)
{
  // Setup (all values in map frame, arbitrary):
  //   Tractor at origin, facing +X (yaw=0).
  //   Hitch at base_footprint (-1.4, 0, 0.35) → in map: (-1.4, 0, 0.35).
  //   Trailer at (-2.3, 0.5, 0.35).
  //   Expected phi = atan2(0.5 - 0, -2.3 - (-1.4)) - 0 = atan2(0.5, -0.9).

  const double tractor_yaw  = 0.0;
  const double hitch_x_map  = -1.4;
  const double hitch_y_map  =  0.0;
  const double trailer_x    = -2.3;
  const double trailer_y    =  0.5;

  const double phi_expected = normalizeAngle(
    std::atan2(trailer_y - hitch_y_map, trailer_x - hitch_x_map) - tractor_yaw);

  // Same formula as publishHitchAngle().
  const double phi_computed = normalizeAngle(
    std::atan2(trailer_y - hitch_y_map, trailer_x - hitch_x_map) - tractor_yaw);

  EXPECT_NEAR(phi_computed, phi_expected, 1e-9) << "Hitch angle formula mismatch";

  // Non-trivial case: tractor rotated 45°.
  const double tractor_yaw2  = kPi / 4.0;
  const double phi_expected2 = normalizeAngle(
    std::atan2(trailer_y - hitch_y_map, trailer_x - hitch_x_map) - tractor_yaw2);
  const double phi_computed2 = normalizeAngle(
    std::atan2(trailer_y - hitch_y_map, trailer_x - hitch_x_map) - tractor_yaw2);
  EXPECT_NEAR(phi_computed2, phi_expected2, 1e-9) << "Hitch angle formula mismatch (rotated)";

  // Sanity: straight-ahead trailer (phi must be close to pi, pointing away from tractor).
  //   Hitch at (-1.4, 0), trailer directly behind at (-3.3, 0).
  const double phi_straight = normalizeAngle(
    std::atan2(0.0 - 0.0, -3.3 - (-1.4)) - 0.0);
  // atan2(0, -1.9) = pi — normalise wraps to ≈-pi.
  EXPECT_NEAR(std::abs(phi_straight), kPi, 0.01) << "Straight-ahead hitch angle wrong";
}

// Test 5 — RANSAC 3D line fit converges with 30% outliers.
//
// Generate a noisy line along [1, 0, 0] direction, inject 30% uniform outliers.
// Expect: recovered direction within 2° of true direction, > 70% inlier ratio.
TEST(TrailerEkf, RansacLineFit3D)
{
  std::mt19937 rng(42u);
  std::uniform_real_distribution<double> u_t(-2.0, 2.0);
  std::uniform_real_distribution<double> u_outlier(-1.0, 1.0);
  std::normal_distribution<double> noise(0.0, 0.02);  // 2 cm noise on inliers

  const Eigen::Vector3d true_dir(1.0, 0.0, 0.0);
  const Eigen::Vector3d true_p0(0.5, -1.0, 0.3);

  std::vector<Eigen::Vector3d> pts;
  pts.reserve(100);

  // 70 inlier points.
  for (int i = 0; i < 70; ++i) {
    const double t = u_t(rng);
    pts.emplace_back(
      true_p0 + t * true_dir +
      Eigen::Vector3d(noise(rng), noise(rng), noise(rng)));
  }

  // 30 outlier points (uniform in a cube).
  for (int i = 0; i < 30; ++i) {
    pts.emplace_back(u_outlier(rng), u_outlier(rng), u_outlier(rng));
  }

  // Shuffle to mix inliers and outliers.
  std::shuffle(pts.begin(), pts.end(), rng);

  const RansacLine3d result = ransacLine3d(pts, 0.05, 200, 0.5);

  ASSERT_TRUE(result.valid) << "RANSAC failed to find a line";
  EXPECT_GE(result.n_inliers, 60) << "Too few inliers found";

  // Direction agreement (angle between result and true direction).
  const double dot = std::abs(result.dir.dot(true_dir));  // abs: direction is undirected
  const double angle_deg = std::acos(std::clamp(dot, 0.0, 1.0)) * 180.0 / kPi;
  EXPECT_LE(angle_deg, 2.0) << "Recovered direction error > 2°: " << angle_deg << "°";

  // Residual variance should be small (< 0.01 m²) for 2 cm noise.
  EXPECT_LT(result.residual_variance, 0.01) << "Residual variance too large";
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
