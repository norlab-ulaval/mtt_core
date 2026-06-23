// GTest: CAN frame codec round-trips.
// Tests every encode/decode operation in can_frame_codec.hpp.
// Runs entirely without ROS or hardware.

#include <gtest/gtest.h>
#include "mtt_driver/logic/can_frame_codec.hpp"

using namespace mtt;
using namespace mtt::can;

// ── CommandFrame defaults ──
TEST(CanFrameCodec, InitDefaultsCenter)
{
  CommandFrame f;
  f.init_defaults();
  EXPECT_EQ(f.steer_raw(), VehicleParams::steering_center_byte);
  EXPECT_EQ(f.throttle_raw(), 0u);
  EXPECT_EQ(f.brake_raw(), 0u);
}

// ── Throttle ──
TEST(CanFrameCodec, ThrottleNormalized)
{
  CommandFrame f;
  f.init_defaults();
  EXPECT_TRUE(f.set_throttle(0.0));
  EXPECT_EQ(f.throttle_raw(), 0u);
  EXPECT_TRUE(f.set_throttle(1.0));
  EXPECT_EQ(f.throttle_raw(), VehicleParams::throttle_max);
  // Half throttle (~round)
  EXPECT_TRUE(f.set_throttle(0.5));
  EXPECT_NEAR(static_cast<int>(f.throttle_raw()), static_cast<int>(VehicleParams::throttle_max / 2), 2);
}

TEST(CanFrameCodec, ThrottleOutOfRange)
{
  CommandFrame f;
  f.init_defaults();
  EXPECT_FALSE(f.set_throttle(-0.1));
  EXPECT_FALSE(f.set_throttle(1.1));
}

// ── Brake ──
TEST(CanFrameCodec, BrakeNormalized)
{
  CommandFrame f;
  f.init_defaults();
  EXPECT_TRUE(f.set_brake(0.0));
  EXPECT_EQ(f.brake_raw(), 0u);
  EXPECT_TRUE(f.set_brake(1.0));
  EXPECT_EQ(f.brake_raw(), VehicleParams::brake_max);
}

// ── Steering ──
TEST(CanFrameCodec, SteerCenter)
{
  CommandFrame f;
  f.init_defaults();
  EXPECT_TRUE(f.set_steer(0.0));  // center → deadband → 127
  EXPECT_EQ(f.steer_raw(), VehicleParams::steering_center_byte);
}

TEST(CanFrameCodec, SteerDeadband)
{
  CommandFrame f;
  f.init_defaults();
  // Values within deadband should snap to center
  EXPECT_TRUE(f.set_steer(0.005));
  EXPECT_EQ(f.steer_raw(), VehicleParams::steering_center_byte);
}

TEST(CanFrameCodec, SteerFullLeft)
{
  CommandFrame f;
  f.init_defaults();
  EXPECT_TRUE(f.set_steer(-1.0));  // full left → raw 0
  EXPECT_EQ(f.steer_raw(), 0u);
}

TEST(CanFrameCodec, SteerFullRight)
{
  CommandFrame f;
  f.init_defaults();
  EXPECT_TRUE(f.set_steer(1.0));  // full right → raw 255
  EXPECT_EQ(f.steer_raw(), VehicleParams::steering_max_byte);
}

// ── Direction bit ──
TEST(CanFrameCodec, DirectionToggle)
{
  CommandFrame f;
  f.init_defaults();
  EXPECT_EQ(f.get_direction(), Direction::Forward);
  f.set_direction(Direction::Reverse);
  EXPECT_EQ(f.get_direction(), Direction::Reverse);
  f.set_direction(Direction::Forward);
  EXPECT_EQ(f.get_direction(), Direction::Forward);
}

// ── Telemetry decoder ──
TEST(CanFrameCodec, TelemetryDecode)
{
  // Synthesize a 8-byte frame
  // temp_a = 25, temp_b = -10, instant = 0x0032 = 50 RPS, cumulative = 0x000000C8 = 200
  const uint8_t raw[8] = {25, 246, 0x00, 0x32, 0x00, 0x00, 0x00, 0xC8};
  auto result = TelemetryDecoder::decode(raw, 8);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->temperature_a, 25);
  EXPECT_EQ(result->temperature_b, -10);
  EXPECT_EQ(result->instant_rps, 50u);
  EXPECT_EQ(result->cumulative_ticks, 200u);
}

TEST(CanFrameCodec, TelemetryDecodeShortFrame)
{
  const uint8_t raw[4] = {0, 0, 0, 0};
  EXPECT_FALSE(TelemetryDecoder::decode(raw, 4).has_value());
}
