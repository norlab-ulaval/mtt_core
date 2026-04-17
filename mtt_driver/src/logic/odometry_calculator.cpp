// MTT-154 Multi-Mode Odometry — implementation
// Ported from mtt_odometry_manager.py (SingleTrailerOdometry, DualDifferential, DualSerpentine).

#include "mtt_driver/logic/odometry_calculator.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace mtt::logic {

// ──────────────────────────────────────────────────────────────────────
// Single Trailer
// ──────────────────────────────────────────────────────────────────────
SingleTrailerOdometry::SingleTrailerOdometry() = default;

OdometryOutput SingleTrailerOdometry::update(const OdometryInput& input)
{
  const double cur_abs_m = input.distance_km * 1000.0;

  // Encoder delta distance
  std::optional<double> delta_m{};
  if (last_abs_m_.has_value())
    delta_m = cur_abs_m - *last_abs_m_;

  last_abs_m_ = cur_abs_m;

  // Signed speed
  double speed_ms = 0.0;
  if (delta_m && input.dt > 0.001)
    speed_ms = (*delta_m * input.direction_sign) / input.dt;
  else
    speed_ms = input.speed_ms;

  // Throttle for dynamics ([-1, 1])
  constexpr double max_v = VehicleParams::max_speed_ms;
  double throttle = std::clamp(speed_ms / max_v, -1.0, 1.0);

  // Save previous pose to integrate encoder distance
  double x_prev = dynamics_.x();
  double y_prev = dynamics_.y();

  auto [x, y, heading] = dynamics_.update(throttle, input.steer_cmd, input.dt);

  // Override heading with IMU when available
  double final_heading = (use_imu_ && input.imu_heading) ? *input.imu_heading : heading;

  // Integrate position along encoder delta
  double x_enc = x_prev, y_enc = y_prev;
  if (delta_m) {
    double ds = *delta_m * input.direction_sign;
    x_enc = x_prev + ds * std::cos(final_heading);
    y_enc = y_prev + ds * std::sin(final_heading);
  }

  // Sync dynamics internal state
  dynamics_.set_x(x_enc);
  dynamics_.set_y(y_enc);
  dynamics_.set_heading(final_heading);

  // Covariance increases with speed and articulation
  const double phi = dynamics_.articulation_angle();
  double speed_factor = std::abs(speed_ms) / max_v;
  double artic_factor = std::abs(phi) / VehicleParams::max_articulation_rad;

  OdometryOutput out;
  out.x                 = x_enc;
  out.y                 = y_enc;
  out.heading           = final_heading;
  out.vx                = speed_ms;
  out.wz                = input.angular_velocity;
  out.articulation_angle = phi;
  out.pos_cov           = 0.02 * (1.0 + speed_factor + artic_factor);
  out.heading_cov       = 0.05 * (1.0 + 2.0 * artic_factor);
  out.vel_cov           = 0.15 * (1.0 + speed_factor);
  return out;
}

void SingleTrailerOdometry::reset()
{
  dynamics_.reset();
  last_abs_m_.reset();
}

OdometryPose SingleTrailerOdometry::export_pose() const
{
  return {dynamics_.x(), dynamics_.y(), dynamics_.heading(), last_abs_m_};
}

void SingleTrailerOdometry::import_pose(const OdometryPose& pose)
{
  dynamics_.set_state(pose.x, pose.y, pose.heading);
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

  if (!last_abs_m_) { last_abs_m_ = abs_m; }

  double delta = abs_m - *last_abs_m_;
  last_abs_m_  = abs_m;

  // Integrate with commanded angular velocity
  constexpr double dt = 0.02;  // nominal; real dt passed in input.dt
  theta_ += input.angular_velocity * input.dt;
  x_ += delta * input.direction_sign * std::cos(theta_);
  y_ += delta * input.direction_sign * std::sin(theta_);

  OdometryOutput out;
  out.x       = x_;
  out.y       = y_;
  out.heading = theta_;
  out.vx      = input.speed_ms;
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
  return {x_, y_, theta_, last_abs_m_};
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

  double ds = 0.0;
  if (!last_abs_m_) {
    last_abs_m_ = cur_abs;
  } else {
    ds = (cur_abs - *last_abs_m_) * input.direction_sign;
    last_abs_m_ = cur_abs;
  }

  const double steer = input.steer_cmd;

  if (std::abs(steer) < 1e-9 || wheelbase_m_ < 1e-9) {
    x_ += ds * std::cos(th_);
    y_ += ds * std::sin(th_);
  } else {
    double dth = std::tan(steer) / wheelbase_m_ * ds;
    if (std::abs(dth) > 1e-9) {
      double R = ds / dth;
      x_ += R * (std::sin(th_ + dth) - std::sin(th_));
      y_ -= R * (std::cos(th_ + dth) - std::cos(th_));
    } else {
      x_ += ds * std::cos(th_);
      y_ += ds * std::sin(th_);
    }
    th_ += dth;
  }

  double v = input.direction_sign * std::abs(input.speed_ms);

  OdometryOutput out;
  out.x       = x_;
  out.y       = y_;
  out.heading = th_;
  out.vx      = v;
  out.wz      = (wheelbase_m_ > 1e-6 && std::abs(steer) > 0)
                  ? (v / wheelbase_m_) * std::tan(steer)
                  : 0.0;
  return out;
}

void DualSerpentineOdometry::reset()
{
  x_ = y_ = th_ = 0.0;
  last_abs_m_.reset();
}

OdometryPose DualSerpentineOdometry::export_pose() const
{
  return {x_, y_, th_, last_abs_m_};
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
