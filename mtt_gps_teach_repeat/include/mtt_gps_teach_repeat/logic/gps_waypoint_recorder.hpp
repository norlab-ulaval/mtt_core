#pragma once

#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace mtt_gps_teach_repeat
{
namespace logic
{

/** Outcome of GpsWaypointRecorder::add_sample, so the node can count and
 * report why a GPS reading did not become a waypoint. */
enum class GpsSampleOutcome
{
  Added,
  TooClose,
  BelowMinSpeed,
  RejectedJump,
};

/** Accumulate (x, y, speed) waypoints in the localization ENU frame and save
 * them as a ROMEA `.traj` v2 file (readable by romea_core_path::PathFile).
 *
 * A new section starts whenever the longitudinal speed changes sign, so
 * forward and reverse legs stay separately interpolable. Samples implying an
 * implausible speed relative to the previous point (a GPS/localization
 * "jump" — reacquisition, multipath, ISAM2 relocalization) are rejected
 * rather than recorded, since a single bad point would otherwise bend the
 * whole taught path once ROMEA fits a curve through it.
 */
class GpsWaypointRecorder
{
public:
  struct Waypoint
  {
    double x{0.0};
    double y{0.0};
    double speed{0.0};
  };

  void configure(
    double min_dist_between_points,
    double min_speed,
    double max_plausible_speed_ms,
    int smoothing_window_size);
  void reset();

  /// stamp_s should be monotonically non-decreasing (e.g. message header time).
  GpsSampleOutcome add_sample(double x, double y, double speed, double stamp_s);

  /// Centered moving-average smoothing of recorded positions, per section,
  /// with the configured half-window. Call once before save(), after
  /// recording stops — mirrors WILN's teach smoothing_window_size.
  void smooth();

  bool empty() const;
  std::size_t point_count() const;
  std::size_t section_count() const;
  std::size_t rejected_jump_count() const;
  const std::vector<std::vector<Waypoint>> & sections() const {return sections_;}

  /// anchor_wgs84 = [latitude_deg, longitude_deg, altitude_m]
  /// The file is written through a temporary sibling and atomically renamed,
  /// so a crash cannot leave a partially-written active route.
  /// Throws std::runtime_error on I/O failure.
  void save(const std::string & filename, const std::array<double, 3> & anchor_wgs84) const;

private:
  double min_dist_between_points_{0.10};
  double min_speed_{0.10};
  double max_plausible_speed_ms_{5.0};
  int smoothing_window_size_{0};
  int current_sign_{0};
  bool has_last_stamp_{false};
  double last_stamp_s_{0.0};
  std::size_t rejected_jump_count_{0};
  std::vector<std::vector<Waypoint>> sections_;
};

}  // namespace logic
}  // namespace mtt_gps_teach_repeat
