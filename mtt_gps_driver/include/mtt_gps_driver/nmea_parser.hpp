// NMEA sentence parser for Emlid Reach RS GPS receivers.
// Parses GGA, RMC, HDT, VTG sentences into structured data.

#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>


namespace mtt_gps {

struct GgaData {
  double timestamp_utc;
  double latitude_deg;
  double longitude_deg;
  double altitude_m;
  int fix_quality;     // 0=invalid, 1=GPS, 2=DGPS, 4=RTK Fix, 5=RTK Float
  int num_satellites;
  double hdop;
  double geoid_separation_m;
};

struct RmcData {
  double timestamp_utc;
  double latitude_deg;
  double longitude_deg;
  double speed_knots;
  double course_deg;
  bool valid;
};

struct HdtData {
  double heading_deg;   // True heading from dual antenna
  bool valid;
};

// Parse a latitude string like "4807.038" with hemisphere 'N'/'S'
double parse_lat(const std::string& field, const std::string& hemisphere);

// Parse a longitude string like "01131.000" with hemisphere 'E'/'W'
double parse_lon(const std::string& field, const std::string& hemisphere);

// Parse UTC time string "hhmmss.ss" to seconds since midnight
double parse_utc_time(const std::string& field);

// Verify NMEA checksum
bool verify_checksum(const std::string& sentence);

// Split NMEA sentence into fields
std::vector<std::string> split_fields(const std::string& sentence);

// Parse specific sentence types
std::optional<GgaData> parse_gga(const std::string& sentence);
std::optional<RmcData> parse_rmc(const std::string& sentence);
std::optional<HdtData> parse_hdt(const std::string& sentence);

}  // namespace mtt_gps
