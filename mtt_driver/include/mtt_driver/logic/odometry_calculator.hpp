// MTT-154 Multi-Mode Odometry Calculators
// Ported from mtt_odometry_manager.py — Strategy pattern, no ROS dependency.
// Provides: SingleTrailerOdometry, DualDifferentialOdometry, DualSerpentineOdometry.

#pragma once

#include <cmath>
#include <memory>
#include <optional>
#include <string>

#include "mtt_driver/logic/articulated_model.hpp"
#include "mtt_driver/logic/vehicle_params.hpp"

namespace mtt::logic {

// ── Driving modes ─────────────────────────────────────────────────────
enum class DrivingMode : uint8_t {
  SingleTrailer      = 0,
  DualDifferential   = 1,
  DualSerpentine     = 2,
};

// ── Input snapshot passed to each odometry calculator ────────────────
struct OdometryInput {
  double   distance_km{0.0};        // Absolute cumulative distance from tachometer
  double   speed_ms{0.0};           // Instantaneous speed (signed with direction)
  double   steer_cmd{0.0};          // Raw steering command [-1, +1]
  double   angular_velocity{0.0};   // Commanded angular velocity (rad/s)
  int      direction_sign{1};       // +1 forward, -1 reverse
  double   dt{0.02};                // Integration time step (s)
  std::optional<double> imu_heading{};  // IMU heading (rad) if available
};

// ── Output pose from each calculator ─────────────────────────────────
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

// ── Serializable pose for mode-switch state preservation ─────────────
struct OdometryPose {
  double x{0.0}, y{0.0}, heading{0.0};
  std::optional<double> last_abs_m{};
};

// ── Abstract calculator ───────────────────────────────────────────────
class IOdometryCalculator {
public:
  virtual ~IOdometryCalculator() = default;
  virtual OdometryOutput update(const OdometryInput& input) = 0;
  virtual void reset() = 0;
  virtual OdometryPose export_pose() const = 0;
  virtual void import_pose(const OdometryPose& pose) = 0;
  virtual std::string mode_name() const = 0;
};

// ── Single Trailer — articulated dynamics + encoder integration ───────
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
  ArticulatedVehicleDynamics dynamics_;
  std::optional<double> last_abs_m_{};
  bool use_imu_{true};
};

// ── Dual Differential — skid-steer fallback ───────────────────────────
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

// ── Dual Serpentine — bicycle model ───────────────────────────────────
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

// ── Factory ───────────────────────────────────────────────────────────
class OdometryFactory {
public:
  static std::unique_ptr<IOdometryCalculator> create(
    DrivingMode mode,
    double track_width_m = VehicleParams::track_width,
    double wheelbase_m   = VehicleParams::total_wheelbase());
};

}  // namespace mtt::logic
