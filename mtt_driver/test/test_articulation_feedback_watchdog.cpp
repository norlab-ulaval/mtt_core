#include <gtest/gtest.h>

#include "mtt_driver/logic/articulation_servo.hpp"

namespace mtt::logic
{

TEST(ArticulationFeedbackWatchdog, TripsOnFrozenFeedbackUnderEffort)
{
  ArticulationFeedbackWatchdog watchdog;
  bool stuck = false;
  for (int i = 0; i < 50; ++i) {
    stuck = watchdog.update(0.1, 0.2, 0.8, 0.02);
  }
  EXPECT_TRUE(stuck);
}

TEST(ArticulationFeedbackWatchdog, DoesNotTripWhenFeedbackMoves)
{
  ArticulationFeedbackWatchdog watchdog;
  bool stuck = false;
  for (int i = 0; i < 100; ++i) {
    stuck = watchdog.update(0.001 * i, 0.2, 0.8, 0.02);
  }
  EXPECT_FALSE(stuck);
}

TEST(ArticulationFeedbackWatchdog, ClearsWhenErrorIsSmall)
{
  ArticulationFeedbackWatchdog watchdog;
  for (int i = 0; i < 30; ++i) {
    watchdog.update(0.0, 0.2, 0.8, 0.02);
  }
  EXPECT_FALSE(watchdog.update(0.0, 0.01, 0.8, 0.02));
}

}  // namespace mtt::logic
