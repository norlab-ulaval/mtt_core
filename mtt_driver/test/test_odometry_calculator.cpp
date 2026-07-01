// GTest: odometry integration guards around tachometer cumulative distance.

#include <gtest/gtest.h>

#include "mtt_driver/logic/odometry_calculator.hpp"

using namespace mtt::logic;

namespace {

OdometryInput base_input(double speed_ms = 1.0)
{
  OdometryInput input;
  input.speed_ms = speed_ms;
  input.direction_sign = speed_ms < 0.0 ? -1 : 1;
  input.dt = 0.02;
  input.synthetic_model_valid = false;
  return input;
}

}  // namespace

TEST(OdometryCalculator, SingleTrailerRejectsImpossibleCumulativeDistanceJump)
{
  SingleTrailerOdometry odom;
  auto input = base_input(1.0);

  const auto out1 = odom.update(input);
  input.distance_km = 1.0 / 1000.0;  // 1 m in 20 ms would imply 50 m/s.
  const auto out2 = odom.update(input);

  EXPECT_NEAR(out2.x - out1.x, 0.02, 1e-9);
  EXPECT_NEAR(out2.vx, 1.0, 1e-9);
}

TEST(OdometryCalculator, SingleTrailerKeepsReverseSignWhenDistanceJumpIsRejected)
{
  SingleTrailerOdometry odom;
  auto input = base_input(-1.0);

  const auto out1 = odom.update(input);
  input.distance_km = 1.0 / 1000.0;
  const auto out2 = odom.update(input);

  EXPECT_NEAR(out2.x - out1.x, -0.02, 1e-9);
  EXPECT_NEAR(out2.vx, -1.0, 1e-9);
}

TEST(OdometryCalculator, SingleTrailerUsesPlausibleCumulativeDistanceDelta)
{
  SingleTrailerOdometry odom;
  auto input = base_input(1.0);

  const auto out1 = odom.update(input);
  input.distance_km = 0.02 / 1000.0;
  const auto out2 = odom.update(input);

  EXPECT_NEAR(out2.x - out1.x, 0.02, 1e-9);
  EXPECT_NEAR(out2.vx, 1.0, 1e-9);
}

TEST(OdometryCalculator, SingleTrailerRejectsStuckCumulativeDistanceWhileMoving)
{
  SingleTrailerOdometry odom;
  auto input = base_input(1.0);
  input.distance_km = 10.0 / 1000.0;

  const auto out1 = odom.update(input);
  const auto out2 = odom.update(input);

  EXPECT_NEAR(out2.x - out1.x, 0.02, 1e-9);
  EXPECT_NEAR(out2.vx, 1.0, 1e-9);
}

TEST(OdometryCalculator, SingleTrailerPublishesFreshMeasuredArticulation)
{
  SingleTrailerOdometry odom;
  auto input = base_input(0.0);
  input.steer_cmd = 0.0;
  input.articulation_effective_rad = -0.044;
  input.articulation_measurement_valid = true;

  const auto out = odom.update(input);

  EXPECT_NEAR(out.articulation_angle, -0.044, 1e-12);
}

TEST(OdometryCalculator, DualDifferentialRejectsImpossibleCumulativeDistanceJump)
{
  DualDifferentialOdometry odom;
  auto input = base_input(1.0);

  const auto out1 = odom.update(input);
  input.distance_km = 1.0 / 1000.0;
  const auto out2 = odom.update(input);

  EXPECT_NEAR(out2.x - out1.x, 0.02, 1e-9);
  EXPECT_NEAR(out2.vx, 1.0, 1e-9);
}

TEST(OdometryCalculator, DualSerpentineRejectsImpossibleCumulativeDistanceJump)
{
  DualSerpentineOdometry odom;
  auto input = base_input(1.0);

  const auto out1 = odom.update(input);
  input.distance_km = 1.0 / 1000.0;
  const auto out2 = odom.update(input);

  EXPECT_NEAR(out2.x - out1.x, 0.02, 1e-9);
  EXPECT_NEAR(out2.vx, 1.0, 1e-9);
}
