// MTT-154 Tachometer data structures and speed/distance calculations
// Ported from TachometerData dataclass + _calculate_gear_ratio/_calculate_absolute_distance

#pragma once

#include <chrono>
#include <cstdint>

#include "mtt_driver/logic/vehicle_params.hpp"

namespace mtt {

// Raw decoded reading from a single 0x2FF CAN frame
struct TachometerReading {
  int8_t   temperature_a{0};
  int8_t   temperature_b{0};
  uint16_t instant_rps{0};          // Instantaneous encoder ticks per second
  uint32_t cumulative_ticks{0};     // Cumulative distance in encoder ticks
};

// Stateful tachometer with timestamp and freshness tracking
struct TachometerState {
  TachometerReading reading{};
  std::chrono::steady_clock::time_point last_update{};
  bool has_data{false};

  // Update from a new CAN frame
  void update(const TachometerReading& r) {
    reading = r;
    last_update = std::chrono::steady_clock::now();
    has_data = true;
  }

  // Check if data is fresh enough to trust
  bool is_fresh(std::chrono::milliseconds timeout) const {
    if (!has_data) return false;
    auto age = std::chrono::steady_clock::now() - last_update;
    return age <= timeout;
  }

  // Age in milliseconds since last update
  double age_ms() const {
    if (!has_data) return 0.0;
    auto age = std::chrono::steady_clock::now() - last_update;
    return std::chrono::duration<double, std::milli>(age).count();
  }

  // ── Speed calculations ──

  // Speed in m/s from raw RPS (unsigned)
  double speed_ms() const {
    constexpr double ratio = VehicleParams::encoder_final_ratio();
    if (ratio == 0.0) return 0.0;
    double speed_kmh = (static_cast<double>(reading.instant_rps) / ratio)
                       * VehicleParams::track_length_km * 3600.0;
    return speed_kmh / 3.6;
  }

  // Speed in km/h from raw RPS (unsigned)
  double speed_kmh() const {
    constexpr double ratio = VehicleParams::encoder_final_ratio();
    if (ratio == 0.0) return 0.0;
    return (static_cast<double>(reading.instant_rps) / ratio)
           * VehicleParams::track_length_km * 3600.0;
  }

  // ── Distance calculations ──

  // Absolute distance in meters from cumulative ticks
  double absolute_distance_m() const {
    if (reading.cumulative_ticks == 0) return 0.0;
    constexpr double ratio = VehicleParams::encoder_final_ratio();
    return (static_cast<double>(reading.cumulative_ticks) / ratio)
           * VehicleParams::track_length_m;
  }
};

}  // namespace mtt
