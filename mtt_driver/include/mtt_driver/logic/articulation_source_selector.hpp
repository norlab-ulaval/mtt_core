#pragma once

namespace mtt::logic
{

enum class ArticulationMeasurementSource
{
  NONE,
  LIDAR,
  HARDWARE,
  EXTERNAL_STATE,
};

struct ArticulationMeasurementSelection
{
  bool valid{false};
  double angle_rad{0.0};
  ArticulationMeasurementSource source{ArticulationMeasurementSource::NONE};
};

inline ArticulationMeasurementSelection select_articulation_measurement(
  bool prefer_lidar,
  bool lidar_valid,
  double lidar_rad,
  bool hardware_valid,
  double hardware_rad,
  bool external_state_valid,
  double external_state_rad)
{
  if (prefer_lidar && lidar_valid) {
    return {true, lidar_rad, ArticulationMeasurementSource::LIDAR};
  }
  if (hardware_valid) {
    return {true, hardware_rad, ArticulationMeasurementSource::HARDWARE};
  }
  if (lidar_valid) {
    return {true, lidar_rad, ArticulationMeasurementSource::LIDAR};
  }
  if (external_state_valid) {
    return {true, external_state_rad, ArticulationMeasurementSource::EXTERNAL_STATE};
  }
  return {};
}

}  // namespace mtt::logic
