#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>

#include <nlohmann/json.hpp>

#include "mtt_gps_teach_repeat/logic/articulated_gps_controller.hpp"
#include "mtt_gps_teach_repeat/logic/gps_waypoint_recorder.hpp"

namespace
{

mtt_gps_teach_repeat::logic::ArticulatedGpsController make_controller()
{
  mtt_gps_teach_repeat::logic::ArticulatedGpsControllerParams params;
  params.min_speed_ms = 0.35;
  params.max_speed_ms = 1.60;
  return {params, mtt_control::logic::ArticulatedCommandModel{}};
}

}  // namespace

TEST(ArticulatedGpsController, ZeroDesiredSpeedStaysStopped)
{
  auto controller = make_controller();
  mtt_gps_teach_repeat::logic::ArticulatedGpsControllerInput input;
  input.desired_speed_ms = 0.0;
  input.previous_articulation_rad = 0.2;
  input.dt_s = 0.1;
  const auto output = controller.compute(input);
  EXPECT_DOUBLE_EQ(output.speed_ms, 0.0);
  EXPECT_DOUBLE_EQ(output.articulation_rad, 0.2);
}

TEST(ArticulatedGpsController, PreservesForwardAndReverseDirection)
{
  auto controller = make_controller();
  mtt_gps_teach_repeat::logic::ArticulatedGpsControllerInput input;
  input.dt_s = 0.1;
  input.desired_speed_ms = 0.8;
  EXPECT_GT(controller.compute(input).speed_ms, 0.0);
  input.desired_speed_ms = -0.8;
  EXPECT_LT(controller.compute(input).speed_ms, 0.0);
}

TEST(GpsWaypointRecorder, ResetPreventsTeachSessionConcatenation)
{
  mtt_gps_teach_repeat::logic::GpsWaypointRecorder recorder;
  recorder.configure(0.1, 0.1, 5.0, 0);
  EXPECT_EQ(recorder.add_sample(0.0, 0.0, 0.5, 0.0),
    mtt_gps_teach_repeat::logic::GpsSampleOutcome::Added);
  EXPECT_EQ(recorder.add_sample(1.0, 0.0, 0.5, 2.0),
    mtt_gps_teach_repeat::logic::GpsSampleOutcome::Added);
  recorder.reset();
  EXPECT_TRUE(recorder.empty());
  EXPECT_EQ(recorder.section_count(), 0U);
  EXPECT_EQ(recorder.add_sample(10.0, 2.0, 0.5, 10.0),
    mtt_gps_teach_repeat::logic::GpsSampleOutcome::Added);
  ASSERT_EQ(recorder.point_count(), 1U);
  EXPECT_DOUBLE_EQ(recorder.sections().front().front().x, 10.0);
}

TEST(GpsWaypointRecorder, SavesForwardAndReverseAsSeparateSections)
{
  mtt_gps_teach_repeat::logic::GpsWaypointRecorder recorder;
  recorder.configure(0.1, 0.1, 5.0, 0);
  recorder.add_sample(0.0, 0.0, 0.5, 0.0);
  recorder.add_sample(1.0, 0.0, 0.5, 1.0);
  recorder.add_sample(0.8, 0.0, -0.4, 2.0);

  const auto output = std::filesystem::temp_directory_path() /
    "mtt_gps_teach_repeat_logic_test.traj";
  recorder.save(output.string(), {46.778879, -71.277157, 0.0});
  std::ifstream stream(output);
  nlohmann::json data;
  stream >> data;
  std::filesystem::remove(output);

  ASSERT_EQ(data["sections"].size(), 2U);
  EXPECT_EQ(data["sections"][0], 0);
  EXPECT_EQ(data["sections"][1], 2);
  EXPECT_LT(data["points"]["values"][2][2].get<double>(), 0.0);
}
