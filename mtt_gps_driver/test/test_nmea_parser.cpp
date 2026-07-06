#include <gtest/gtest.h>

#include "mtt_gps_driver/nmea_parser.hpp"

TEST(NmeaParser, PreservesTrailingEmptyField)
{
  const auto fields = mtt_gps::split_fields(
    "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47");

  ASSERT_EQ(fields.size(), 15U);
  EXPECT_EQ(fields[0], "GPGGA");
  EXPECT_TRUE(fields[14].empty());
}

TEST(NmeaParser, ParsesStandardGgaWithEmptyStationId)
{
  const auto gga = mtt_gps::parse_gga(
    "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47");

  ASSERT_TRUE(gga.has_value());
  EXPECT_NEAR(gga->timestamp_utc, 45319.0, 1e-9);
  EXPECT_NEAR(gga->latitude_deg, 48.1173, 1e-6);
  EXPECT_NEAR(gga->longitude_deg, 11.5166667, 1e-6);
  EXPECT_EQ(gga->fix_quality, 1);
  EXPECT_EQ(gga->num_satellites, 8);
  EXPECT_NEAR(gga->altitude_m, 545.4, 1e-9);
}
