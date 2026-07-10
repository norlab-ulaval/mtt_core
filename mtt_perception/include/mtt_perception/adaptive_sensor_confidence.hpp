#pragma once

#include <algorithm>
#include <cmath>
#include <optional>

namespace mtt_perception
{

struct AdaptiveSensorConfidenceParams
{
  double initial{0.5};
  double recovery_gain{0.08};
  double disagreement_penalty{0.30};
  double stuck_penalty{0.35};
  double stale_decay{0.75};
  double good_residual_rad{0.04};
  double bad_residual_rad{0.20};
  double lidar_motion_threshold_rad{0.025};
  double sensor_motion_epsilon_rad{0.003};
};

/// Confidence estimator for a secondary articulation sensor.
///
/// LiDAR remains the primary measurement. A coherent STM measurement slowly earns
/// confidence, while disagreement, stale data, or a sensor that stays fixed when
/// LiDAR moves loses confidence quickly. The asymmetric rates are intentional:
/// failure must be fast and recovery must require repeated agreement.
class AdaptiveSensorConfidence
{
public:
  explicit AdaptiveSensorConfidence(
    const AdaptiveSensorConfidenceParams & params = AdaptiveSensorConfidenceParams{})
  : params_(params), confidence_(std::clamp(params.initial, 0.0, 1.0)) {}

  void set_params(const AdaptiveSensorConfidenceParams & params)
  {
    params_ = params;
    reset();
  }

  void reset()
  {
    confidence_ = std::clamp(params_.initial, 0.0, 1.0);
    previous_lidar_.reset();
    previous_sensor_.reset();
  }

  double update(double sensor_rad, double lidar_rad)
  {
    const double residual = angular_distance(sensor_rad, lidar_rad);
    if (residual <= params_.good_residual_rad) {
      confidence_ += params_.recovery_gain * (1.0 - confidence_);
    } else {
      const double span = std::max(
        params_.bad_residual_rad - params_.good_residual_rad, 1e-6);
      const double severity = std::clamp(
        (residual - params_.good_residual_rad) / span, 0.0, 1.0);
      confidence_ -= params_.disagreement_penalty * severity;
    }

    if (previous_lidar_ && previous_sensor_) {
      const double lidar_motion = angular_distance(lidar_rad, *previous_lidar_);
      const double sensor_motion = angular_distance(sensor_rad, *previous_sensor_);
      if (lidar_motion >= params_.lidar_motion_threshold_rad &&
          sensor_motion <= params_.sensor_motion_epsilon_rad)
      {
        confidence_ -= params_.stuck_penalty;
      }
    }

    previous_lidar_ = lidar_rad;
    previous_sensor_ = sensor_rad;
    confidence_ = std::clamp(confidence_, 0.0, 1.0);
    return confidence_;
  }

  double mark_stale()
  {
    confidence_ = std::clamp(confidence_ * params_.stale_decay, 0.0, 1.0);
    return confidence_;
  }

  double confidence() const { return confidence_; }

private:
  static double angular_distance(double a, double b)
  {
    return std::abs(std::atan2(std::sin(a - b), std::cos(a - b)));
  }

  AdaptiveSensorConfidenceParams params_;
  double confidence_{0.5};
  std::optional<double> previous_lidar_;
  std::optional<double> previous_sensor_;
};

}  // namespace mtt_perception
