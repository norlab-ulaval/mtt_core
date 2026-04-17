#include "mtt_gps_driver/nmea_parser.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <sstream>
#include <vector>

namespace mtt_gps {

double parse_lat(const std::string& field, const std::string& hemisphere) {
  if (field.empty()) return 0.0;
  // Format: ddmm.mmmm
  double raw = std::stod(field);
  int degrees = static_cast<int>(raw / 100.0);
  double minutes = raw - degrees * 100.0;
  double result = degrees + minutes / 60.0;
  if (hemisphere == "S") result = -result;
  return result;
}

double parse_lon(const std::string& field, const std::string& hemisphere) {
  if (field.empty()) return 0.0;
  // Format: dddmm.mmmm
  double raw = std::stod(field);
  int degrees = static_cast<int>(raw / 100.0);
  double minutes = raw - degrees * 100.0;
  double result = degrees + minutes / 60.0;
  if (hemisphere == "W") result = -result;
  return result;
}

double parse_utc_time(const std::string& field) {
  if (field.size() < 6) return 0.0;
  double hours = std::stod(field.substr(0, 2));
  double mins  = std::stod(field.substr(2, 2));
  double secs  = std::stod(field.substr(4));
  return hours * 3600.0 + mins * 60.0 + secs;
}

bool verify_checksum(const std::string& sentence) {
  auto star = sentence.find('*');
  if (star == std::string::npos || star + 2 >= sentence.size()) return false;

  uint8_t computed = 0;
  // XOR everything between $ and *
  for (size_t i = 1; i < star; ++i) {
    computed ^= static_cast<uint8_t>(sentence[i]);
  }

  auto expected_str = sentence.substr(star + 1, 2);
  auto expected = static_cast<uint8_t>(std::stoi(expected_str, nullptr, 16));
  return computed == expected;
}

std::vector<std::string> split_fields(const std::string& sentence) {
  std::vector<std::string> fields;
  // Strip leading $ and trailing *XX\r\n
  auto start = sentence.find('$');
  auto star  = sentence.find('*');
  if (start == std::string::npos) start = 0; else start += 1;
  if (star == std::string::npos) star = sentence.size();

  std::string body = sentence.substr(start, star - start);
  std::istringstream ss(body);
  std::string field;
  while (std::getline(ss, field, ',')) {
    fields.push_back(field);
  }
  return fields;
}

std::optional<GgaData> parse_gga(const std::string& sentence) {
  if (!verify_checksum(sentence)) return std::nullopt;
  auto fields = split_fields(sentence);
  if (fields.size() < 15) return std::nullopt;
  if (fields[0].find("GGA") == std::string::npos) return std::nullopt;

  GgaData data{};
  data.timestamp_utc = parse_utc_time(fields[1]);
  data.latitude_deg  = parse_lat(fields[2], fields[3]);
  data.longitude_deg = parse_lon(fields[4], fields[5]);
  data.fix_quality   = fields[6].empty() ? 0 : std::stoi(fields[6]);
  data.num_satellites = fields[7].empty() ? 0 : std::stoi(fields[7]);
  data.hdop          = fields[8].empty() ? 99.0 : std::stod(fields[8]);
  data.altitude_m    = fields[9].empty() ? 0.0 : std::stod(fields[9]);
  data.geoid_separation_m = fields[11].empty() ? 0.0 : std::stod(fields[11]);
  return data;
}

std::optional<RmcData> parse_rmc(const std::string& sentence) {
  if (!verify_checksum(sentence)) return std::nullopt;
  auto fields = split_fields(sentence);
  if (fields.size() < 12) return std::nullopt;
  if (fields[0].find("RMC") == std::string::npos) return std::nullopt;

  RmcData data{};
  data.timestamp_utc = parse_utc_time(fields[1]);
  data.valid = (fields[2] == "A");
  data.latitude_deg  = parse_lat(fields[3], fields[4]);
  data.longitude_deg = parse_lon(fields[5], fields[6]);
  data.speed_knots   = fields[7].empty() ? 0.0 : std::stod(fields[7]);
  data.course_deg    = fields[8].empty() ? 0.0 : std::stod(fields[8]);
  return data;
}

std::optional<HdtData> parse_hdt(const std::string& sentence) {
  if (!verify_checksum(sentence)) return std::nullopt;
  auto fields = split_fields(sentence);
  if (fields.size() < 3) return std::nullopt;
  if (fields[0].find("HDT") == std::string::npos) return std::nullopt;

  HdtData data{};
  data.heading_deg = fields[1].empty() ? 0.0 : std::stod(fields[1]);
  data.valid = !fields[1].empty();
  return data;
}

}  // namespace mtt_gps
