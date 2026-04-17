// MTT-154 Vehicle Parameters — compile-time constants from real vehicle measurements
// Single source of truth for vehicle geometry, drivetrain, and steering.
// Ported from mtt_vehicle_params.py

#pragma once

#include <cmath>
#include <cstdint>

namespace mtt {

struct VehicleParams {
  // ── Drivetrain ──────────────────────────────────────────────────────
  static constexpr double l_f = 0.9;              // Tractor: center-of-track → hitch pin (m)
  static constexpr double l_r = 1.5;              // Trailer: hitch pin → trailer axle (m)
  static constexpr double r_sprocket_eff = 0.0202; // Effective sprocket radius (m)

  // Gear train (per mtt_encoder_methodology.md)
  static constexpr int gear1 = 16;
  static constexpr int gear2 = 36;
  static constexpr int gear3 = 15;
  static constexpr int gear4 = 32;
  static constexpr int gear_drive = 8;
  static constexpr int gear_track = 54;
  static constexpr int tacho_teeth = 10;  // Tachometer disc: 10 slots per motor rev

  // Track
  static constexpr double track_length_cm = 393.0;
  static constexpr double track_length_m  = track_length_cm / 100.0;
  static constexpr double track_length_km = track_length_cm / 100000.0;

  // ── Computed drivetrain ─────────────────────────────────────────────
  static constexpr double mechanical_gear_ratio() {
    return (static_cast<double>(gear2) / gear1)
         * (static_cast<double>(gear4) / gear3)
         * (static_cast<double>(gear_track) / gear_drive);
  }

  // Canonical value: 324.0 ticks per sprocket revolution
  static constexpr double encoder_final_ratio() {
    return mechanical_gear_ratio() * tacho_teeth;
  }

  static constexpr double total_wheelbase() { return l_f + l_r; }
  static constexpr double wheel_radius()    { return r_sprocket_eff; }

  // ── Steering ────────────────────────────────────────────────────────
  // Physical articulation limit measured from URDF yaw joint (±1.047 rad = ±60°).
  // Previously 50° — increased to match the actual mechanical joint range.
  static constexpr double max_articulation_deg = 60.0;
  static constexpr double max_articulation_rad = max_articulation_deg * M_PI / 180.0;
  static constexpr double steering_deadband_deg = 0.5;
  static constexpr double steering_deadband_rad = steering_deadband_deg * M_PI / 180.0;
  static constexpr double steering_deadband_normalized = steering_deadband_deg / max_articulation_deg;

  static constexpr uint8_t steering_center_byte   = 127;
  static constexpr uint8_t steering_max_byte       = 255;
  static constexpr int     steering_halfspan_byte  = 100;

  // ── Vehicle dynamics ────────────────────────────────────────────────
  static constexpr double track_width          = 1.2;
  static constexpr double track_slip_coeff     = 0.05;
  static constexpr double track_grip_coeff     = 0.95;
  static constexpr double carving_factor       = 0.1;
  static constexpr double articulation_response = 0.8;
  static constexpr double max_yaw_rate_rad_s   = (45.0 * M_PI / 180.0) / 6.0;
  static constexpr double max_speed_ms         = 2.0;
  static constexpr double slip_speed_factor    = 0.02;
  static constexpr double min_speed_for_steering = 0.1;

  // ── CAN command frame limits ────────────────────────────────────────
  static constexpr uint8_t throttle_max = 230;
  static constexpr uint8_t brake_max    = 255;
};

}  // namespace mtt
