// MTT-154 Multi-Mode Odometry Calculators
// Ported from mtt_odometry_manager.py — Strategy pattern, no ROS dependency.
// Provides: SingleTrailerOdometry, DualDifferentialOdometry, DualSerpentineOdometry.

#pragma once

#include <cmath>
#include <memory>
#include <optional>
#include <string>

#include "mtt_driver/logic/vehicle_params.hpp"

namespace mtt::logic {

// ── Driving modes ──
enum class DrivingMode : uint8_t {
  SingleTrailer      = 0,
  DualDifferential   = 1,
  DualSerpentine     = 2,
};

// ── Yaw rate source quality — used to scale heading covariance ──
enum class YawRateSource : uint8_t {
  MODEL_ONLY            = 0,  // open-loop command model, highest uncertainty
  HARDWARE_CLOSED_LOOP  = 1,  // hardware articulation angle in closed-loop
  IMU_ONLY              = 2,  // IMU when model is unavailable
  IMU_BLEND             = 3,  // complementary filter: IMU + model (best)
};

// ── Input snapshot passed to each odometry calculator ──
struct OdometryInput {
  double   distance_km{0.0};        // Absolute cumulative distance from tachometer
  double   speed_ms{0.0};           // Instantaneous speed (signed with direction)
  double   steer_cmd{0.0};          // Raw steering command [-1, +1]
  double   angular_velocity{0.0};   // Commanded angular velocity (rad/s)
  int      direction_sign{1};       // +1 forward, -1 reverse
  double   dt{0.02};                // Integration time step (s)
  std::optional<double> imu_heading{};  // IMU heading (rad) if available
  bool synthetic_model_valid{false};
  double articulation_command_rad{0.0};
  double articulation_effective_rad{0.0};
  bool articulation_measurement_valid{false};
  double curvature_nominal_m_inv{0.0};
  double curvature_effective_m_inv{0.0};
  double yaw_rate_nominal_rad_s{0.0};
  double yaw_rate_effective_rad_s{0.0};
  YawRateSource yaw_rate_source{YawRateSource::MODEL_ONLY};
};

// ── Output pose from each calculator ──
struct OdometryOutput {
  double x{0.0};
  double y{0.0};
  double heading{0.0};   // rad, normalized to [-π, +π]
  double vx{0.0};        // linear velocity (m/s)
  double wz{0.0};        // angular velocity (rad/s)
  double articulation_angle{0.0};  // only meaningful for SingleTrailer mode

  // Covariance hints for ROS node to fill nav_msgs/Odometry
  double pos_cov{0.01};
  double heading_cov{0.05};
  double vel_cov{0.15};
};

// ── Serializable pose for mode-switch state preservation ──
struct OdometryPose {
  double x{0.0}, y{0.0}, heading{0.0};
  double articulation_angle{0.0};
  std::optional<double> last_abs_m{};
};

// ── Abstract calculator ──
class IOdometryCalculator {
public:
  virtual ~IOdometryCalculator() = default;
  virtual OdometryOutput update(const OdometryInput& input) = 0;
  virtual void reset() = 0;
  virtual OdometryPose export_pose() const = 0;
  virtual void import_pose(const OdometryPose& pose) = 0;
  virtual std::string mode_name() const = 0;
};

// ── Single Trailer — articulated dynamics + encoder integration ──
class SingleTrailerOdometry final : public IOdometryCalculator {
public:
  explicit SingleTrailerOdometry();

  OdometryOutput update(const OdometryInput& input) override;
  void reset() override;
  OdometryPose export_pose() const override;
  void import_pose(const OdometryPose& pose) override;
  std::string mode_name() const override { return "SingleTrailer"; }

  void set_imu_feedback(bool enabled) { use_imu_ = enabled; }

private:
  double x_{0.0};
  double y_{0.0};
  double heading_{0.0};
  double articulation_angle_{0.0};
  double articulation_response_gain_{VehicleParams::articulation_response};
  std::optional<double> last_abs_m_{};
  bool use_imu_{true};
};

// ── Dual Differential — skid-steer fallback ──
class DualDifferentialOdometry final : public IOdometryCalculator {
public:
  explicit DualDifferentialOdometry(double track_width_m = VehicleParams::track_width);

  OdometryOutput update(const OdometryInput& input) override;
  void reset() override;
  OdometryPose export_pose() const override;
  void import_pose(const OdometryPose& pose) override;
  std::string mode_name() const override { return "DualDifferential"; }

private:
  double x_{0.0}, y_{0.0}, theta_{0.0};
  double track_width_m_;
  std::optional<double> last_abs_m_{};
};

// ── Dual Serpentine — bicycle model ──
class DualSerpentineOdometry final : public IOdometryCalculator {
public:
  explicit DualSerpentineOdometry(double wheelbase_m = VehicleParams::total_wheelbase());

  OdometryOutput update(const OdometryInput& input) override;
  void reset() override;
  OdometryPose export_pose() const override;
  void import_pose(const OdometryPose& pose) override;
  std::string mode_name() const override { return "DualSerpentine"; }

private:
  double x_{0.0}, y_{0.0}, th_{0.0};
  double wheelbase_m_;
  std::optional<double> last_abs_m_{};
};

// ── Factory ──
class OdometryFactory {
public:
  static std::unique_ptr<IOdometryCalculator> create(
    DrivingMode mode,
    double track_width_m = VehicleParams::track_width,
    double wheelbase_m   = VehicleParams::total_wheelbase());
};

}  // namespace mtt::logic
