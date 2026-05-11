// SE(3) state definition for the Factor Graph.
// Wraps GTSAM types for the MTT robot state vector.

#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/base/Vector.h>

namespace mtt_loc {

// State at a single keyframe
struct NavState {
  gtsam::Pose3 pose;
  gtsam::Vector3 velocity;
  gtsam::imuBias::ConstantBias imu_bias;
  double trailer_angle{0.0};
  double timestamp{0.0};
  uint64_t key_index{0};
};

// Sensor noise parameters (loaded from YAML)
struct SensorNoiseParams {
  // IMU
  double accel_noise_density{0.01};      // m/s²/√Hz
  double gyro_noise_density{0.0001};     // rad/s/√Hz
  double accel_random_walk{0.0001};      // m/s³/√Hz
  double gyro_random_walk{1e-6};         // rad/s²/√Hz

  // Track odometry
  double odom_linear_noise{0.1};         // m/s
  double odom_angular_noise{0.05};       // rad/s

  // GPS
  double gps_position_noise_xy{1.0};     // meters (SPP)
  double gps_position_noise_z{2.0};      // meters
  double gps_heading_noise{0.05};        // radians

  // LiDAR odometry
  gtsam::Vector6 lidar_odom_noise =
      (gtsam::Vector6() << 0.05, 0.05, 0.05, 0.01, 0.01, 0.01).finished();

  // Visual odometry
  gtsam::Vector6 visual_odom_noise =
      (gtsam::Vector6() << 0.1, 0.1, 0.1, 0.02, 0.02, 0.02).finished();

  // Articulation (hitch yaw φ)
  double phi_sigma_hardware{0.008};  // rad — encoder fresh (≈ ±0.5°)
  double phi_sigma_model{0.035};     // rad — model / stale encoder
  double phi_sigma_dynamics{0.015};  // rad — random-walk per keyframe
  double phi_prior_sigma{0.5};       // rad — initial prior on φ at k=0

  // Trailer LiDAR pose factor (TrailerPoseFactor)
  double trailer_sigma_rot{0.04};    // rad base noise (scaled by 1/√confidence)
  double trailer_sigma_trans{0.10};  // m   base noise
};

}  // namespace mtt_loc
