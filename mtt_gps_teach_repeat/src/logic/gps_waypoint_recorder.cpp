#include "mtt_gps_teach_repeat/logic/gps_waypoint_recorder.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <stdexcept>

#include <nlohmann/json.hpp>

namespace mtt_gps_teach_repeat
{
namespace logic
{

void GpsWaypointRecorder::configure(
  double min_dist_between_points,
  double min_speed,
  double max_plausible_speed_ms,
  int smoothing_window_size)
{
  min_dist_between_points_ = min_dist_between_points;
  min_speed_ = min_speed;
  max_plausible_speed_ms_ = max_plausible_speed_ms;
  smoothing_window_size_ = smoothing_window_size;
}

void GpsWaypointRecorder::reset()
{
  sections_.clear();
  current_sign_ = 0;
  has_last_stamp_ = false;
  rejected_jump_count_ = 0;
}

GpsSampleOutcome GpsWaypointRecorder::add_sample(double x, double y, double speed, double stamp_s)
{
  if (std::abs(speed) < min_speed_) {
    return GpsSampleOutcome::BelowMinSpeed;
  }

  const int sign = speed >= 0.0 ? 1 : -1;
  if (sections_.empty() || sign != current_sign_) {
    sections_.emplace_back();
    current_sign_ = sign;
  }

  auto & section = sections_.back();
  if (!section.empty()) {
    const auto & last = section.back();
    const double dx = x - last.x;
    const double dy = y - last.y;
    const double dist = std::hypot(dx, dy);

    if (has_last_stamp_) {
      const double dt = stamp_s - last_stamp_s_;
      if (dt > 1e-3 && dist / dt > max_plausible_speed_ms_) {
        // Position jumped further than the vehicle could plausibly have
        // moved: a GPS/localization glitch, not a real waypoint. Do not
        // advance last_stamp_s_/section state so a later, plausible sample
        // is still checked against the last GOOD point.
        ++rejected_jump_count_;
        return GpsSampleOutcome::RejectedJump;
      }
    }

    if (dist < min_dist_between_points_) {
      return GpsSampleOutcome::TooClose;
    }
  }

  section.push_back({x, y, speed});
  last_stamp_s_ = stamp_s;
  has_last_stamp_ = true;
  return GpsSampleOutcome::Added;
}

void GpsWaypointRecorder::smooth()
{
  if (smoothing_window_size_ <= 0) {
    return;
  }
  const int half_window = smoothing_window_size_;

  for (auto & section : sections_) {
    const std::size_t n = section.size();
    if (n < 3) {
      continue;
    }
    std::vector<Waypoint> smoothed = section;
    for (std::size_t i = 0; i < n; ++i) {
      const std::size_t lo = (i >= static_cast<std::size_t>(half_window)) ?
        i - static_cast<std::size_t>(half_window) : 0;
      const std::size_t hi = std::min(n - 1, i + static_cast<std::size_t>(half_window));

      double sum_x = 0.0;
      double sum_y = 0.0;
      std::size_t count = 0;
      for (std::size_t j = lo; j <= hi; ++j) {
        sum_x += section[j].x;
        sum_y += section[j].y;
        ++count;
      }
      smoothed[i].x = sum_x / static_cast<double>(count);
      smoothed[i].y = sum_y / static_cast<double>(count);
    }
    section = std::move(smoothed);
  }
}

std::size_t GpsWaypointRecorder::rejected_jump_count() const
{
  return rejected_jump_count_;
}

bool GpsWaypointRecorder::empty() const
{
  return point_count() == 0;
}

std::size_t GpsWaypointRecorder::point_count() const
{
  std::size_t count = 0;
  for (const auto & section : sections_) {
    count += section.size();
  }
  return count;
}

std::size_t GpsWaypointRecorder::section_count() const
{
  return sections_.size();
}

void GpsWaypointRecorder::save(
  const std::string & filename, const std::array<double, 3> & anchor_wgs84) const
{
  // PathFile v2 format: "sections" lists the start index of each section and
  // must begin with 0.
  nlohmann::json data;
  data["version"] = "2";
  data["origin"]["type"] = "WGS84";
  data["origin"]["coordinates"] = {anchor_wgs84[0], anchor_wgs84[1], anchor_wgs84[2]};
  data["points"]["columns"] = {"x", "y", "speed"};

  auto & values = data["points"]["values"];
  values = nlohmann::json::array();
  auto & section_indexes = data["sections"];
  section_indexes = nlohmann::json::array();

  std::size_t index = 0;
  for (const auto & section : sections_) {
    section_indexes.push_back(index);
    for (const auto & point : section) {
      values.push_back({point.x, point.y, point.speed});
      ++index;
    }
  }
  data["annotations"] = nlohmann::json::array();

  const auto parent = std::filesystem::path(filename).parent_path();
  if (!parent.empty()) {
    std::filesystem::create_directories(parent);
  }

  const std::filesystem::path output_path(filename);
  const std::filesystem::path temporary_path = output_path.string() + ".tmp";
  std::error_code ec;
  std::filesystem::remove(temporary_path, ec);

  std::ofstream file(temporary_path);
  if (!file.is_open()) {
    throw std::runtime_error(
            "cannot open temporary trajectory file for writing: " + temporary_path.string());
  }
  file << data.dump(1);
  file.flush();
  if (!file.good()) {
    file.close();
    std::filesystem::remove(temporary_path, ec);
    throw std::runtime_error("failed while writing trajectory file: " + filename);
  }
  file.close();

  std::filesystem::rename(temporary_path, output_path, ec);
  if (ec) {
    std::filesystem::remove(temporary_path, ec);
    throw std::runtime_error("cannot atomically replace trajectory file: " + filename);
  }
}

}  // namespace logic
}  // namespace mtt_gps_teach_repeat
