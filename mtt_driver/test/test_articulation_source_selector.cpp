#include <gtest/gtest.h>

#include "mtt_driver/logic/articulation_source_selector.hpp"

namespace mtt::logic
{

TEST(ArticulationSourceSelector, PrefersDetectedLidar)
{
  const auto result = select_articulation_measurement(
    true, true, 0.30, true, 0.10, false, 0.0);
  EXPECT_TRUE(result.valid);
  EXPECT_EQ(result.source, ArticulationMeasurementSource::LIDAR);
  EXPECT_DOUBLE_EQ(result.angle_rad, 0.30);
}

TEST(ArticulationSourceSelector, FallsBackToFreshHardware)
{
  const auto result = select_articulation_measurement(
    true, false, 0.0, true, -0.20, false, 0.0);
  EXPECT_TRUE(result.valid);
  EXPECT_EQ(result.source, ArticulationMeasurementSource::HARDWARE);
}

TEST(ArticulationSourceSelector, RejectsModelWhenBothSensorsInvalid)
{
  const auto result = select_articulation_measurement(
    true, false, 0.0, false, 0.0, false, 0.0);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.source, ArticulationMeasurementSource::NONE);
}

}  // namespace mtt::logic
