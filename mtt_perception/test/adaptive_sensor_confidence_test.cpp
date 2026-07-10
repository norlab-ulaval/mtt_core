#include <gtest/gtest.h>

#include "mtt_perception/adaptive_sensor_confidence.hpp"

namespace mtt_perception
{

TEST(AdaptiveSensorConfidence, CoherentSensorRecoversGradually)
{
  AdaptiveSensorConfidence confidence;
  const double initial = confidence.confidence();
  for (int i = 0; i < 10; ++i) {
    confidence.update(0.20 + i * 0.001, 0.20 + i * 0.0015);
  }
  EXPECT_GT(confidence.confidence(), initial);
  EXPECT_LT(confidence.confidence(), 1.0);
}

TEST(AdaptiveSensorConfidence, LargeDisagreementLowersConfidenceQuickly)
{
  AdaptiveSensorConfidence confidence;
  confidence.update(0.0, 0.0);
  const double before = confidence.confidence();
  confidence.update(0.0, 0.35);
  EXPECT_LT(confidence.confidence(), before - 0.20);
}

TEST(AdaptiveSensorConfidence, FrozenSensorIsRejectedWhenLidarMoves)
{
  AdaptiveSensorConfidence confidence;
  confidence.update(0.10, 0.10);
  const double before = confidence.confidence();
  confidence.update(0.10, 0.14);
  EXPECT_LT(confidence.confidence(), before - 0.20);
}

TEST(AdaptiveSensorConfidence, StaleSensorDecays)
{
  AdaptiveSensorConfidence confidence;
  const double before = confidence.confidence();
  confidence.mark_stale();
  EXPECT_LT(confidence.confidence(), before);
}

}  // namespace mtt_perception
