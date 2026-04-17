// GTest: tachometer speed and distance calculations.
// Verifies the canonical gear ratio = 324.0 and derived speed/distance values.

#include <gtest/gtest.h>
#include "mtt_driver/logic/tachometer.hpp"

using namespace mtt;

// ── Gear ratio ────────────────────────────────────────────────────────
TEST(Tachometer, GearRatioIs324)
{
  // Per mtt_encoder_methodology.md: G_theory = (36/16)*(32/15)*(54/8) = 32.4
  // Final ratio = G_theory * 10 ticks/rev = 324.0
  constexpr double ratio = VehicleParams::encoder_final_ratio();
  EXPECT_NEAR(ratio, 324.0, 1e-6);
}

TEST(Tachometer, MechanicalGearRatioIs32_4)
{
  constexpr double g = VehicleParams::mechanical_gear_ratio();
  EXPECT_NEAR(g, 32.4, 1e-6);
}

// ── Speed calculation ─────────────────────────────────────────────────
TEST(Tachometer, SpeedAtZeroRPS)
{
  TachometerState t;
  TachometerReading r;
  r.instant_rps = 0;
  t.update(r);
  EXPECT_DOUBLE_EQ(t.speed_ms(), 0.0);
  EXPECT_DOUBLE_EQ(t.speed_kmh(), 0.0);
}

TEST(Tachometer, SpeedConsistency)
{
  // speed_kmh should equal speed_ms * 3.6
  TachometerState t;
  TachometerReading r;
  r.instant_rps = 100;
  t.update(r);
  EXPECT_NEAR(t.speed_kmh(), t.speed_ms() * 3.6, 1e-9);
}

TEST(Tachometer, SpeedTrackLength)
{
  // Manual calculation: speed_ms = (instant_rps / 324.0) * 3.93 * 3600 / 3.6
  TachometerState t;
  TachometerReading r;
  r.instant_rps = 324;  // exactly 1 sprocket revolution per second
  t.update(r);
  double expected_speed = VehicleParams::track_length_m;  // 1 rev/s * 3.93m = 3.93 m/s
  EXPECT_NEAR(t.speed_ms(), expected_speed, 0.001);
}

// ── Distance calculation ──────────────────────────────────────────────
TEST(Tachometer, DistanceAtZeroTicks)
{
  TachometerState t;
  TachometerReading r;
  r.cumulative_ticks = 0;
  t.update(r);
  EXPECT_DOUBLE_EQ(t.absolute_distance_m(), 0.0);
}

TEST(Tachometer, DistanceOneFullRevolution)
{
  // 324 ticks = 1 sprocket revolution = 1 track_length_m distance
  TachometerState t;
  TachometerReading r;
  r.cumulative_ticks = 324;
  t.update(r);
  EXPECT_NEAR(t.absolute_distance_m(), VehicleParams::track_length_m, 0.001);
}

TEST(Tachometer, FreshnessTracking)
{
  TachometerState t;
  EXPECT_FALSE(t.is_fresh(std::chrono::milliseconds(500)));
  TachometerReading r;
  t.update(r);
  EXPECT_TRUE(t.is_fresh(std::chrono::milliseconds(500)));
}
