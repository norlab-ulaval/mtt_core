#include <gtest/gtest.h>

#include <cmath>

#include "mtt_control/logic/articulated_command_model.hpp"

using mtt_control::logic::ArticulatedCommandModel;
using mtt_control::logic::ArticulatedCommandParams;

TEST(ArticulatedCommandModel, ForwardInverseRoundTripIncludesSlip)
{
  ArticulatedCommandModel model;
  const double speed = 0.70;
  const double articulation = 0.30;
  const double yaw_rate = model.yaw_rate(speed, articulation);
  EXPECT_NEAR(model.articulation_from_yaw_rate(speed, yaw_rate), articulation, 1e-6);
}

TEST(ArticulatedCommandModel, ReverseNaturallyFlipsArticulationForSameYawRate)
{
  ArticulatedCommandModel model;
  const double forward = model.articulation_from_yaw_rate(0.60, 0.12);
  const double reverse = model.articulation_from_yaw_rate(-0.60, 0.12);
  EXPECT_GT(forward, 0.0);
  EXPECT_LT(reverse, 0.0);
  EXPECT_NEAR(forward, -reverse, 1e-6);
}

TEST(ArticulatedCommandModel, SaturatesImpossibleCurvature)
{
  ArticulatedCommandParams params;
  params.max_articulation_rad = 0.733;
  ArticulatedCommandModel model(params);
  EXPECT_DOUBLE_EQ(model.articulation_from_yaw_rate(0.5, 100.0), 0.733);
  EXPECT_DOUBLE_EQ(model.articulation_from_yaw_rate(0.5, -100.0), -0.733);
}

TEST(ArticulatedCommandModel, RateLimitIsSymmetric)
{
  ArticulatedCommandParams params;
  params.max_articulation_rate_rad_s = 0.5;
  ArticulatedCommandModel model(params);
  EXPECT_NEAR(model.rate_limited_articulation(0.7, 0.0, 0.1), 0.05, 1e-12);
  EXPECT_NEAR(model.rate_limited_articulation(-0.7, 0.0, 0.1), -0.05, 1e-12);
}

TEST(ArticulatedCommandModel, ZeroSpeedCannotRequestArticulationFromYawRate)
{
  ArticulatedCommandModel model;
  EXPECT_DOUBLE_EQ(model.articulation_from_yaw_rate(0.0, 0.3), 0.0);
}
