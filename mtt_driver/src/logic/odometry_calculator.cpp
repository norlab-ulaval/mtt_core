// MTT-154 Multi-Mode Odometry — implementation
// Ported from mtt_odometry_manager.py (SingleTrailerOdometry, DualDifferential, DualSerpentine).

#include "mtt_driver/logic/odometry_calculator.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace mtt::logic {

namespace {

double wrap_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace

// ──────────────────────────────────────────────────────────────────────
// Single Trailer
// ──────────────────────────────────────────────────────────────────────
SingleTrailerOdometry::SingleTrailerOdometry() = default;

OdometryOutput SingleTrailerOdometry::update(const OdometryInput& input)
{
  const double cur_abs_m = input.distance_km * 1000.0;
  const double heading_prev = heading_;
  const bool use_distance_delta = !input.synthetic_model_valid;

  // Encoder delta distance
  std::optional<double> delta_m{};
  if (last_abs_m_.has_value())
    delta_m = cur_abs_m - *last_abs_m_;

  last_abs_m_ = cur_abs_m;

  // Signed speed
  double speed_ms = input.speed_ms;
  if (use_distance_delta && delta_m && input.dt > 0.001)
    speed_ms = (*delta_m * input.direction_sign) / input.dt;

  const double ds = (use_distance_delta && delta_m)
    ? (*delta_m * input.direction_sign)
    : (speed_ms * input.dt);
  const double commanded_phi = input.synthetic_model_valid
    ? input.articulation_command_rad
    : VehicleParams::normalized_steer_to_articulation_rad(input.steer_cmd);
  const double effective_phi_target = input.synthetic_model_valid
    ? input.articulation_effective_rad
    : commanded_phi;
  const double articulation_alpha = std::clamp(articulation_response_gain_ * input.dt, 0.0, 1.0);
  articulation_angle_ += (effective_phi_target - articulation_angle_) * articulation_alpha;
  articulation_angle_ = std::clamp(
    articulation_angle_,
    -VehicleParams::max_articulation_rad,
    VehicleParams::max_articulation_rad);

  const double yaw_rate = input.synthetic_model_valid
    ? input.yaw_rate_effective_rad_s
    : input.angular_velocity;
  const double dtheta = yaw_rate * input.dt;

  double final_heading = heading_;
  if (use_imu_ && input.imu_heading.has_value()) {
    final_heading = wrap_angle(*input.imu_heading);
  } else {
    final_heading = wrap_angle(heading_ + dtheta);
  }

  const double heading_mid = (use_imu_ && input.imu_heading.has_value())
    ? final_heading
    : wrap_angle(heading_ + 0.5 * dtheta);
  x_ += ds * std::cos(heading_mid);
  y_ += ds * std::sin(heading_mid);
  heading_ = final_heading;

  // Covariance increases with speed and articulation
  constexpr double max_v = VehicleParams::max_speed_ms;
  const double speed_factor = std::abs(speed_ms) / std::max(max_v, 1e-6);
  const double artic_factor =
    std::abs(articulation_angle_) / std::max(VehicleParams::max_articulation_rad, 1e-6);

  OdometryOutput out;
  out.x                 = x_;
  out.y                 = y_;
  out.heading           = heading_;
  out.vx                = speed_ms;
  out.wz                = input.dt > 1e-6
    ? wrap_angle(heading_ - heading_prev) / input.dt
    : yaw_rate;
  out.articulation_angle = articulation_angle_;
  out.pos_cov           = 0.02 * (1.0 + speed_factor + artic_factor);
  out.heading_cov       = 0.05 * (1.0 + 2.0 * artic_factor);
  out.vel_cov           = 0.15 * (1.0 + speed_factor);
  return out;
}

void SingleTrailerOdometry::reset()
{
  x_ = 0.0;
  y_ = 0.0;
  heading_ = 0.0;
  articulation_angle_ = 0.0;
  last_abs_m_.reset();
}

OdometryPose SingleTrailerOdometry::export_pose() const
{
  return {
    x_,
    y_,
    heading_,
    articulation_angle_,
    last_abs_m_,
  };
}

void SingleTrailerOdometry::import_pose(const OdometryPose& pose)
{
  x_ = pose.x;
  y_ = pose.y;
  heading_ = pose.heading;
  articulation_angle_ = pose.articulation_angle;
  last_abs_m_ = pose.last_abs_m;
}

// ──────────────────────────────────────────────────────────────────────
// Dual Differential
// ──────────────────────────────────────────────────────────────────────
DualDifferentialOdometry::DualDifferentialOdometry(double track_width_m)
  : track_width_m_(track_width_m) {}

OdometryOutput DualDifferentialOdometry::update(const OdometryInput& input)
{
  const double abs_m = input.distance_km * 1000.0;
  const bool use_distance_delta = !input.synthetic_model_valid;

  if (!last_abs_m_) { last_abs_m_ = abs_m; }

  const double delta = abs_m - *last_abs_m_;
  last_abs_m_  = abs_m;

  const double speed_ms = use_distance_delta && input.dt > 1e-6
    ? (delta * input.direction_sign) / input.dt
    : input.speed_ms;
  const double ds = use_distance_delta
    ? delta * input.direction_sign
    : speed_ms * input.dt;
  const double dtheta = input.angular_velocity * input.dt;
  const double heading_mid = theta_ + 0.5 * dtheta;

  x_ += ds * std::cos(heading_mid);
  y_ += ds * std::sin(heading_mid);
  theta_ = std::atan2(std::sin(theta_ + dtheta), std::cos(theta_ + dtheta));

  OdometryOutput out;
  out.x       = x_;
  out.y       = y_;
  out.heading = theta_;
  out.vx      = speed_ms;
  out.wz      = input.angular_velocity;
  return out;
}

void DualDifferentialOdometry::reset()
{
  x_ = y_ = theta_ = 0.0;
  last_abs_m_.reset();
}

OdometryPose DualDifferentialOdometry::export_pose() const
{
  return {x_, y_, theta_, 0.0, last_abs_m_};
}

void DualDifferentialOdometry::import_pose(const OdometryPose& pose)
{
  x_ = pose.x; y_ = pose.y; theta_ = pose.heading;
  last_abs_m_ = pose.last_abs_m;
}

// ──────────────────────────────────────────────────────────────────────
// Dual Serpentine
// ──────────────────────────────────────────────────────────────────────
DualSerpentineOdometry::DualSerpentineOdometry(double wheelbase_m)
  : wheelbase_m_(wheelbase_m) {}

OdometryOutput DualSerpentineOdometry::update(const OdometryInput& input)
{
  const double cur_abs = input.distance_km * 1000.0;
  const bool use_distance_delta = !input.synthetic_model_valid;

  double ds = 0.0;
  if (!last_abs_m_) {
    last_abs_m_ = cur_abs;
  } else {
    if (use_distance_delta) {
      ds = (cur_abs - *last_abs_m_) * input.direction_sign;
    }
    last_abs_m_ = cur_abs;
  }
  const double speed_ms = use_distance_delta
    ? (input.direction_sign * std::abs(input.speed_ms))
    : input.speed_ms;
  if (!use_distance_delta) {
    ds = speed_ms * input.dt;
  }

  const double articulation = input.synthetic_model_valid
    ? input.articulation_effective_rad
    : VehicleParams::normalized_steer_to_articulation_rad(input.steer_cmd);
  double dtheta = 0.0;

  if (input.synthetic_model_valid) {
    dtheta = input.yaw_rate_effective_rad_s * input.dt;
    const double heading_mid = th_ + 0.5 * dtheta;
    x_ += ds * std::cos(heading_mid);
    y_ += ds * std::sin(heading_mid);
    th_ = std::atan2(std::sin(th_ + dtheta), std::cos(th_ + dtheta));
  } else if (std::abs(articulation) < 1e-9 || wheelbase_m_ < 1e-9) {
    x_ += ds * std::cos(th_);
    y_ += ds * std::sin(th_);
  } else {
    dtheta = std::tan(articulation) / wheelbase_m_ * ds;
    if (std::abs(dtheta) > 1e-9) {
      const double radius = ds / dtheta;
      x_ += radius * (std::sin(th_ + dtheta) - std::sin(th_));
      y_ -= radius * (std::cos(th_ + dtheta) - std::cos(th_));
    } else {
      x_ += ds * std::cos(th_);
      y_ += ds * std::sin(th_);
    }
    th_ = std::atan2(std::sin(th_ + dtheta), std::cos(th_ + dtheta));
  }

  OdometryOutput out;
  out.x       = x_;
  out.y       = y_;
  out.heading = th_;
  out.vx      = speed_ms;
  out.wz      = input.synthetic_model_valid
    ? input.yaw_rate_effective_rad_s
    : (input.dt > 1e-6 ? dtheta / input.dt : 0.0);
  return out;
}

void DualSerpentineOdometry::reset()
{
  x_ = y_ = th_ = 0.0;
  last_abs_m_.reset();
}

OdometryPose DualSerpentineOdometry::export_pose() const
{
  return {x_, y_, th_, 0.0, last_abs_m_};
}

void DualSerpentineOdometry::import_pose(const OdometryPose& pose)
{
  x_ = pose.x; y_ = pose.y; th_ = pose.heading;
  last_abs_m_ = pose.last_abs_m;
}

// ──────────────────────────────────────────────────────────────────────
// Factory
// ──────────────────────────────────────────────────────────────────────
std::unique_ptr<IOdometryCalculator> OdometryFactory::create(
  DrivingMode mode, double track_width_m, double wheelbase_m)
{
  switch (mode) {
    case DrivingMode::SingleTrailer:    return std::make_unique<SingleTrailerOdometry>();
    case DrivingMode::DualDifferential: return std::make_unique<DualDifferentialOdometry>(track_width_m);
    case DrivingMode::DualSerpentine:   return std::make_unique<DualSerpentineOdometry>(wheelbase_m);
    default: throw std::invalid_argument("Unknown driving mode");
  }
}

}  // namespace mtt::logic
