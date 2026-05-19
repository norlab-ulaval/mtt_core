/// TrailerPoseFactor — Custom GTSAM unary factor on the hitch yaw angle φ.
///
/// Mathematical formulation
/// ────────────────────────
/// State variable : φ ∈ ℝ  (hitch yaw angle, symbol 'h')
///
/// Kinematic model (same URDF chain as TrailerLocalizerNode):
///   Δ(φ) = A · Rz(π/2 + φ) · B   ∈ SE(3)
///   where A, B are constant 4×4 matrices precomputed from the URDF.
///
/// Measurement : T_measured ∈ SE(3)  (trailer/pose in base_link frame,
///               from trailer_pose_node's LiDAR + kinematic refinement)
///
/// Residual (6-vector in se(3), GTSAM convention [ω; v] = [rx,ry,rz,tx,ty,tz]):
///   e(φ) = Pose3::Logmap( Δ(φ)⁻¹ · T_measured )
///
/// Jacobian de/dφ (6×1, computed via central differences):
///   H = [ e(φ+ε) − e(φ−ε) ] / (2ε)
///
/// Why is this elegant?
/// ────────────────────
/// The measurement T_measured is 6-dimensional but φ is 1-dimensional.
/// The system is overdetermined: all 6 components of the pose residual
/// constrain φ through the URDF kinematic chain.  Translation x/y primarily
/// constrain φ; translation z constrains pitch α (which we don't model yet
/// but the residual serves as a diagnostic); rotation rz maps directly to φ.
/// ISAM2 finds the φ that minimises the weighted sum-of-squares — the full
/// 6D information is used, not just yaw.
///
/// Noise model:
///   σ_rot  = sigma_rot_base  / √max(confidence, 0.05)
///   σ_trans = sigma_trans_base / √max(confidence, 0.05)
///   Below min_confidence threshold the factor is not added at all.

#pragma once

#include <cmath>

#include <boost/optional.hpp>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace mtt_loc
{

// ─────────────────────────────────────────────────────────────────────────────
// URDF kinematic constants (base_footprint → MTT_remorque)
// ─────────────────────────────────────────────────────────────────────────────
namespace hitch_kinematics
{

/// Build URDF joint origin transform: Trans(xyz) · RotRPY(roll,pitch,yaw)
/// URDF RPY convention: Rz(yaw)·Ry(pitch)·Rx(roll)
/// In GTSAM: Rot3::RzRyRx(yaw, pitch, roll)
inline gtsam::Pose3 urdfOrigin(
  double x, double y, double z,
  double roll, double pitch, double yaw)
{
  return {gtsam::Rot3::RzRyRx(yaw, pitch, roll), gtsam::Point3(x, y, z)};
}

/// Full revolute joint (axis = z): T_origin · Rz(q)
inline gtsam::Pose3 urdfJoint(
  double x, double y, double z,
  double roll, double pitch, double yaw,
  double q)
{
  return urdfOrigin(x, y, z, roll, pitch, yaw).compose(
    gtsam::Pose3{gtsam::Rot3::Rz(q), gtsam::Point3::Zero()});
}

/// Δ(φ, α=0) — yaw-only, pitch at rest position (backward compat).
/// Δ(φ, α)   — full 3D: pitch joint deviated by α from rest (-π/2).
///
/// URDF chain (base_footprint → MTT_remorque):
///   T_bf_bl   : fixed,  xyz=(0,0,-0.1)
///   T_pitch(α): revolute xyz=(-1.0512,0.2125,0.3578) rpy=(-π/2,0,0) q=-π/2+α
///   T_yaw_origin: fixed origin, xyz=(0,0.0571,-0.2635) rpy=(-π/2,0,0)
///   Rz(π/2+φ): yaw rotation
///   T_roll    : revolute xyz=(0,0,-0.0571) rpy=(-π/2,0,-π/2) q=-π/2 (fixed at rest)
///
/// A_prefix = T_bf_bl · T_pitch_origin   (before pitch rotation)
/// A_suffix = T_yaw_origin               (between pitch and yaw rotations)
/// B        = T_roll                     (after yaw rotation, constant)
///
/// Δ(φ, α) = A_prefix · Rz(-π/2 + α) · A_suffix · Rz(π/2 + φ) · B
inline gtsam::Pose3 computeDelta(double phi, double alpha = 0.0)
{
  // base_footprint → base_link (fixed)
  static const gtsam::Pose3 T_bf_bl{gtsam::Rot3(), gtsam::Point3(0, 0, -0.1)};

  // Pitch joint origin (without rotation): Trans(xyz) · RotRPY(rpy)
  static const gtsam::Pose3 T_pitch_origin = urdfOrigin(
    -1.0511878376018575, 0.2125028310306423, 0.3577510511469179,
    -M_PI_2, 0.0, 0.0);

  // Yaw joint origin (without rotation): Trans(xyz) · RotRPY(rpy)
  static const gtsam::Pose3 T_yaw_origin = urdfOrigin(
    0.0, 0.05714999999950, -0.26352500000200,
    -M_PI_2, 0.0, 0.0);

  // Constant prefix: T_bf_bl · T_pitch_origin
  static const gtsam::Pose3 kA_prefix = T_bf_bl.compose(T_pitch_origin);

  // Constant suffix after yaw: roll at rest  xyz=(0,0,-0.0571) rpy=(-π/2,0,-π/2) q=-π/2
  static const gtsam::Pose3 kB = urdfJoint(
    0.0, 0.0, -0.05714999999985,
    -M_PI_2, 0.0, -M_PI_2, -M_PI_2);

  // Pitch rotation: Rz(-π/2 + α)  — rest = -π/2, deviation α from potentiometer
  const gtsam::Pose3 R_pitch{gtsam::Rot3::Rz(-M_PI_2 + alpha), gtsam::Point3::Zero()};

  // Yaw rotation: Rz(π/2 + φ)
  const gtsam::Pose3 R_yaw{gtsam::Rot3::Rz(M_PI_2 + phi), gtsam::Point3::Zero()};

  return kA_prefix
    .compose(R_pitch)
    .compose(T_yaw_origin)
    .compose(R_yaw)
    .compose(kB);
}

}  // namespace hitch_kinematics

// ─────────────────────────────────────────────────────────────────────────────
// TrailerPoseFactor
// ─────────────────────────────────────────────────────────────────────────────
class TrailerPoseFactor : public gtsam::NoiseModelFactor1<double>
{
public:
  using Base = gtsam::NoiseModelFactor1<double>;

  TrailerPoseFactor(
    gtsam::Key key_phi,
    const gtsam::Pose3 & measured,
    const gtsam::SharedNoiseModel & model)
  : Base(model, key_phi), measured_(measured)
  {}

  ~TrailerPoseFactor() override = default;

  /// Residual: e(φ) = Pose3::Logmap( Δ(φ)⁻¹ · T_measured )
  gtsam::Vector evaluateError(
    const double & phi,
    boost::optional<gtsam::Matrix &> H = boost::none) const override
  {
    using namespace hitch_kinematics;

    // Predicted pose from kinematics
    const gtsam::Pose3 predicted = computeDelta(phi);

    // Residual in se(3)
    const gtsam::Pose3 err_pose = predicted.inverse().compose(measured_);
    const gtsam::Vector6 error = gtsam::Pose3::Logmap(err_pose);

    if (H) {
      // Numerical Jacobian 6×1 via central differences
      constexpr double kEps = 1e-5;
      const gtsam::Pose3 pred_p = computeDelta(phi + kEps);
      const gtsam::Pose3 pred_m = computeDelta(phi - kEps);
      const gtsam::Vector6 e_p = gtsam::Pose3::Logmap(pred_p.inverse().compose(measured_));
      const gtsam::Vector6 e_m = gtsam::Pose3::Logmap(pred_m.inverse().compose(measured_));
      // H is 6×1 (Matrix with 6 rows, 1 column)
      *H = (e_p - e_m) / (2.0 * kEps);
    }

    return error;
  }

  /// Allow GTSAM to clone this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override
  {
    return gtsam::NonlinearFactor::shared_ptr(new TrailerPoseFactor(*this));
  }

private:
  gtsam::Pose3 measured_;
};

// ─────────────────────────────────────────────────────────────────────────────
// TrailerPoseFactorFull — joint factor on (φ, α)
// ─────────────────────────────────────────────────────────────────────────────
/// 2-variable factor constraining BOTH hitch yaw H(k)=φ and pitch P(k)=α
/// from a single 6D trailer/pose measurement.
///
/// State variables:
///   key_phi   → H(k)  hitch yaw   (symbol 'h')
///   key_alpha → P(k)  hitch pitch (symbol 'p')
///
/// Residual: e(φ, α) = Pose3::Logmap( Δ(φ, α)⁻¹ · T_measured )
///
/// Jacobian: 6×2 matrix
///   H_phi   = ∂e/∂φ   (column 0) — central differences
///   H_alpha = ∂e/∂α   (column 1) — central differences
///
/// This is more informative than TrailerPoseFactor (1-variable) because the
/// 6D measurement constrains pitch α through the longitudinal/vertical residual
/// components — the z translation and rx/ry rotation carry pitch information.
class TrailerPoseFactorFull : public gtsam::NoiseModelFactor2<double, double>
{
public:
  using Base = gtsam::NoiseModelFactor2<double, double>;

  TrailerPoseFactorFull(
    gtsam::Key key_phi,
    gtsam::Key key_alpha,
    const gtsam::Pose3 & measured,
    const gtsam::SharedNoiseModel & model)
  : Base(model, key_phi, key_alpha), measured_(measured)
  {}

  ~TrailerPoseFactorFull() override = default;

  /// Residual and optional 6×2 Jacobian (two 6×1 columns, one per variable)
  gtsam::Vector evaluateError(
    const double & phi,
    const double & alpha,
    boost::optional<gtsam::Matrix &> H_phi   = boost::none,
    boost::optional<gtsam::Matrix &> H_alpha = boost::none) const override
  {
    using namespace hitch_kinematics;

    const gtsam::Pose3 pred  = computeDelta(phi, alpha);
    const gtsam::Vector6 err = gtsam::Pose3::Logmap(pred.inverse().compose(measured_));

    constexpr double kEps = 1e-5;
    if (H_phi) {
      const gtsam::Vector6 e_p =
        gtsam::Pose3::Logmap(computeDelta(phi + kEps, alpha).inverse().compose(measured_));
      const gtsam::Vector6 e_m =
        gtsam::Pose3::Logmap(computeDelta(phi - kEps, alpha).inverse().compose(measured_));
      *H_phi = (e_p - e_m) / (2.0 * kEps);
    }
    if (H_alpha) {
      const gtsam::Vector6 e_p =
        gtsam::Pose3::Logmap(computeDelta(phi, alpha + kEps).inverse().compose(measured_));
      const gtsam::Vector6 e_m =
        gtsam::Pose3::Logmap(computeDelta(phi, alpha - kEps).inverse().compose(measured_));
      *H_alpha = (e_p - e_m) / (2.0 * kEps);
    }

    return err;
  }

  gtsam::NonlinearFactor::shared_ptr clone() const override
  {
    return gtsam::NonlinearFactor::shared_ptr(new TrailerPoseFactorFull(*this));
  }

private:
  gtsam::Pose3 measured_;
};

}  // namespace mtt_loc
