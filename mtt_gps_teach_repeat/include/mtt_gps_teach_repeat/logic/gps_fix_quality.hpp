#pragma once

#include <cmath>
#include <string>

namespace mtt_gps_teach_repeat
{
namespace logic
{

/** Coarse precision bands derived from NavSatFix horizontal accuracy.
 *
 * The standard sensor_msgs/NavSatStatus enum cannot distinguish RTK Fix from
 * RTK Float (mtt_gps_driver maps both fix_quality 4 and 5 to STATUS_GBAS_FIX)
 * — only position_covariance carries that distinction. Thresholds below match
 * the covariance values reach_driver_node assigns per NMEA GGA fix quality:
 * RTK Fix ~1cm, RTK Float ~10cm, DGPS ~50cm, SPP HDOP-based (a few meters).
 */
enum class GpsFixQuality
{
  NoFix,
  Spp,
  Dgps,
  RtkFloat,
  RtkFix,
};

inline GpsFixQuality classify_gps_fix_quality(double horizontal_accuracy_m)
{
  if (!std::isfinite(horizontal_accuracy_m)) {
    return GpsFixQuality::NoFix;
  }
  if (horizontal_accuracy_m <= 0.03) {
    return GpsFixQuality::RtkFix;
  }
  if (horizontal_accuracy_m <= 0.15) {
    return GpsFixQuality::RtkFloat;
  }
  if (horizontal_accuracy_m <= 0.75) {
    return GpsFixQuality::Dgps;
  }
  if (horizontal_accuracy_m <= 15.0) {
    return GpsFixQuality::Spp;
  }
  return GpsFixQuality::NoFix;
}

inline const char * to_string(GpsFixQuality quality)
{
  switch (quality) {
    case GpsFixQuality::RtkFix: return "RTK_FIX";
    case GpsFixQuality::RtkFloat: return "RTK_FLOAT";
    case GpsFixQuality::Dgps: return "DGPS";
    case GpsFixQuality::Spp: return "SPP";
    default: return "NO_FIX";
  }
}

/// Ordinal rank for threshold comparisons (higher = better precision).
inline int rank(GpsFixQuality quality)
{
  switch (quality) {
    case GpsFixQuality::RtkFix: return 4;
    case GpsFixQuality::RtkFloat: return 3;
    case GpsFixQuality::Dgps: return 2;
    case GpsFixQuality::Spp: return 1;
    default: return 0;
  }
}

inline GpsFixQuality gps_fix_quality_from_string(const std::string & name)
{
  if (name == "rtk_fix") {return GpsFixQuality::RtkFix;}
  if (name == "rtk_float") {return GpsFixQuality::RtkFloat;}
  if (name == "dgps") {return GpsFixQuality::Dgps;}
  if (name == "spp") {return GpsFixQuality::Spp;}
  return GpsFixQuality::NoFix;
}

}  // namespace logic
}  // namespace mtt_gps_teach_repeat
