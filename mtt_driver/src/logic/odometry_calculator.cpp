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

bool usable_tachometer_delta(double raw_delta_m, const OdometryInput& input)
{
  if (!std::isfinite(raw_delta_m) || !std::isfinite(input.dt) || input.dt <= 1e-6) {
    return false;
  }

  // The CAN distance field is expected to be an unsigned cumulative encoder
  // count. Reject resets, wraps, and samples that are incompatible with the
  // instantaneous speed from the same tachometer message.
  if (raw_delta_m < 0.0) {
    return false;
  }

  constexpr double kMinAllowedSpeedMs = 8.0;
  constexpr double kSpeedMarginMs = 1.0;
  const double abs_delta_m = std::abs(raw_delta_m);
  const double abs_speed_ms = std::abs(input.speed_ms);
  constexpr double kMovingSpeedThresholdMs = 0.05;
  constexpr double kStuckDistanceEpsilonM = 1.0e-6;
  if (abs_speed_ms > kMovingSpeedThresholdMs && abs_delta_m <= kStuckDistanceEpsilonM) {
    return false;
  }

  const double implied_speed_ms = abs_delta_m / input.dt;
  const double max_allowed_speed_ms = std::max(
    kMinAllowedSpeedMs,
    std::max(VehicleParams::max_speed_ms * 1.5, abs_speed_ms + kSpeedMarginMs));
  if (implied_speed_ms > max_allowed_speed_ms) {
    return false;
  }

  constexpr double kAbsDeltaMarginM = 0.05;
  constexpr double kRelativeDeltaMargin = 1.5;
  const double expected_delta_m = abs_speed_ms * input.dt;
  const double delta_margin_m = std::max(kAbsDeltaMarginM, kRelativeDeltaMargin * expected_delta_m);
  return std::abs(abs_delta_m - expected_delta_m) <= delta_margin_m;
}

struct DistanceStep {
  double ds{0.0};
  double speed_ms{0.0};
};

DistanceStep resolve_distance_step(
  const OdometryInput& input,
  const std::optional<double>& raw_delta_m,
  bool use_distance_delta)
{
  DistanceStep step;
  step.speed_ms = input.speed_ms;
  step.ds = std::isfinite(input.dt) && input.dt > 0.0
    ? input.speed_ms * input.dt
    : 0.0;

  if (use_distance_delta && raw_delta_m && usable_tachometer_delta(*raw_delta_m, input)) {
    step.ds = *raw_delta_m * static_cast<double>(input.direction_sign);
    step.speed_ms = input.dt > 1e-6 ? step.ds / input.dt : input.speed_ms;
  }

  return step;
}

}  // namespace

// ── Single Trailer ──
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

  const DistanceStep distance_step = resolve_distance_step(input, delta_m, use_distance_delta);
  const double speed_ms = distance_step.speed_ms;
  const double ds = distance_step.ds;
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

  // Covariance increases with speed and articulation.
  // heading_cov is additionally scaled by yaw rate source quality:
  //   IMU_BLEND             → 0.4× (best: direct measurement + model)
  //   HARDWARE_CLOSED_LOOP  → 0.6× (hardware articulation feedback)
  //   IMU_ONLY              → 0.5× (IMU alone, no model cross-check)
  //   MODEL_ONLY            → 1.0× (worst: open-loop command estimate)
  constexpr double max_v = VehicleParams::max_speed_ms;
  const double speed_factor = std::abs(speed_ms) / std::max(max_v, 1e-6);
  const double artic_factor =
    std::abs(articulation_angle_) / std::max(VehicleParams::max_articulation_rad, 1e-6);

  double source_quality = 1.0;
  switch (input.yaw_rate_source) {
    case YawRateSource::IMU_BLEND:            source_quality = 0.4; break;
    case YawRateSource::HARDWARE_CLOSED_LOOP: source_quality = 0.6; break;
    case YawRateSource::IMU_ONLY:             source_quality = 0.5; break;
    case YawRateSource::MODEL_ONLY:           source_quality = 1.0; break;
  }

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
  out.heading_cov       = 0.05 * (1.0 + 2.0 * artic_factor) * source_quality;
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

// ── Dual Differential ──
DualDifferentialOdometry::DualDifferentialOdometry(double track_width_m)
  : track_width_m_(track_width_m) {}

OdometryOutput DualDifferentialOdometry::update(const OdometryInput& input)
{
  const double abs_m = input.distance_km * 1000.0;
  const bool use_distance_delta = !input.synthetic_model_valid;

  if (!last_abs_m_) { last_abs_m_ = abs_m; }

  const double delta = abs_m - *last_abs_m_;
  last_abs_m_  = abs_m;

  const DistanceStep distance_step = resolve_distance_step(input, std::optional<double>{delta}, use_distance_delta);
  const double speed_ms = distance_step.speed_ms;
  const double ds = distance_step.ds;
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

// ── Dual Serpentine ──
DualSerpentineOdometry::DualSerpentineOdometry(double wheelbase_m)
  : wheelbase_m_(wheelbase_m) {}

OdometryOutput DualSerpentineOdometry::update(const OdometryInput& input)
{
  const double cur_abs = input.distance_km * 1000.0;
  const bool use_distance_delta = !input.synthetic_model_valid;

  double ds = 0.0;
  std::optional<double> delta_m{};
  if (!last_abs_m_) {
    last_abs_m_ = cur_abs;
  } else {
    delta_m = cur_abs - *last_abs_m_;
    last_abs_m_ = cur_abs;
  }
  const DistanceStep distance_step = resolve_distance_step(input, delta_m, use_distance_delta);
  ds = distance_step.ds;
  const double speed_ms = distance_step.speed_ms;

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

// ── Factory ──
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
