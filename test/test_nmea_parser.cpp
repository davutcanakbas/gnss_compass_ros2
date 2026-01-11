// Copyright (c) 2026 Davut Can Akbas
// SPDX-License-Identifier: MIT

#include "gnss_compass_driver/protocol/nmea_parser.hpp"

#include <gtest/gtest.h>
#include <cmath>
#include <sstream>
#include <iomanip>

namespace gnss_compass
{
namespace test
{

// Helper function to calculate NMEA checksum
std::string create_nmea_sentence(const std::string& content)
{
  uint8_t checksum = 0;
  for (char c : content) {
    checksum ^= static_cast<uint8_t>(c);
  }
  
  std::ostringstream oss;
  oss << "$" << content << "*" << std::uppercase << std::hex 
      << std::setw(2) << std::setfill('0') << static_cast<int>(checksum) << "\r\n";
  return oss.str();
}

class NmeaParserTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    parser_ = std::make_unique<NmeaParser>();
  }

  std::unique_ptr<NmeaParser> parser_;
};

// =============================================================================
// GGA Message Tests
// =============================================================================

TEST_F(NmeaParserTest, ParseValidGGA)
{
  NmeaGGA received_msg;
  bool callback_called = false;
  
  parser_->set_gga_callback([&](const NmeaGGA& msg) {
    received_msg = msg;
    callback_called = true;
  });
  
  // Valid GGA sentence with RTK fixed
  std::string sentence = create_nmea_sentence(
    "GNGGA,120000.00,4807.038,N,01131.000,E,4,12,0.9,545.4,M,47.0,M,1.0,0000");
  
  parser_->process(reinterpret_cast<const uint8_t*>(sentence.data()), sentence.size());
  
  ASSERT_TRUE(callback_called);
  EXPECT_TRUE(received_msg.valid);
  EXPECT_NEAR(received_msg.latitude_deg, 48.1173, 0.0001);
  EXPECT_NEAR(received_msg.longitude_deg, 11.5167, 0.0001);
  EXPECT_EQ(received_msg.fix_quality, 4);  // RTK fixed
  EXPECT_EQ(received_msg.satellites_used, 12);
  EXPECT_NEAR(received_msg.hdop, 0.9f, 0.01f);
  EXPECT_NEAR(received_msg.altitude_m, 545.4, 0.1);
}

TEST_F(NmeaParserTest, ParseGGANoFix)
{
  NmeaGGA received_msg;
  bool callback_called = false;
  
  parser_->set_gga_callback([&](const NmeaGGA& msg) {
    received_msg = msg;
    callback_called = true;
  });
  
  // GGA with no fix (quality = 0)
  std::string sentence = create_nmea_sentence(
    "GNGGA,120000.00,,,,,0,00,99.99,,,,,,");
  
  parser_->process(reinterpret_cast<const uint8_t*>(sentence.data()), sentence.size());
  
  ASSERT_TRUE(callback_called);
  EXPECT_FALSE(received_msg.valid);
  EXPECT_EQ(received_msg.fix_quality, 0);
}

// =============================================================================
// RMC Message Tests
// =============================================================================

TEST_F(NmeaParserTest, ParseValidRMC)
{
  NmeaRMC received_msg;
  bool callback_called = false;
  
  parser_->set_rmc_callback([&](const NmeaRMC& msg) {
    received_msg = msg;
    callback_called = true;
  });
  
  std::string sentence = create_nmea_sentence(
    "GNRMC,120000.00,A,4807.038,N,01131.000,E,0.5,54.7,110126,,,D");
  
  parser_->process(reinterpret_cast<const uint8_t*>(sentence.data()), sentence.size());
  
  ASSERT_TRUE(callback_called);
  EXPECT_TRUE(received_msg.valid);
  EXPECT_TRUE(received_msg.status_valid);
  EXPECT_NEAR(received_msg.latitude_deg, 48.1173, 0.0001);
  EXPECT_NEAR(received_msg.longitude_deg, 11.5167, 0.0001);
  EXPECT_NEAR(received_msg.speed_knots, 0.5, 0.01);
  EXPECT_NEAR(received_msg.course_deg, 54.7, 0.1);
}

// =============================================================================
// HDT Message Tests (Heading)
// =============================================================================

TEST_F(NmeaParserTest, ParseValidHDT)
{
  NmeaHDT received_msg;
  bool callback_called = false;
  
  parser_->set_hdt_callback([&](const NmeaHDT& msg) {
    received_msg = msg;
    callback_called = true;
  });
  
  std::string sentence = create_nmea_sentence("GNHDT,123.45,T");
  
  parser_->process(reinterpret_cast<const uint8_t*>(sentence.data()), sentence.size());
  
  ASSERT_TRUE(callback_called);
  EXPECT_TRUE(received_msg.valid);
  EXPECT_NEAR(received_msg.heading_deg, 123.45, 0.01);
}

// =============================================================================
// VTG Message Tests
// =============================================================================

TEST_F(NmeaParserTest, ParseValidVTG)
{
  NmeaVTG received_msg;
  bool callback_called = false;
  
  parser_->set_vtg_callback([&](const NmeaVTG& msg) {
    received_msg = msg;
    callback_called = true;
  });
  
  std::string sentence = create_nmea_sentence(
    "GNVTG,054.7,T,034.4,M,005.5,N,010.2,K,D");
  
  parser_->process(reinterpret_cast<const uint8_t*>(sentence.data()), sentence.size());
  
  ASSERT_TRUE(callback_called);
  EXPECT_TRUE(received_msg.valid);
  EXPECT_NEAR(received_msg.course_true_deg, 54.7, 0.1);
  EXPECT_NEAR(received_msg.speed_knots, 5.5, 0.1);
  EXPECT_NEAR(received_msg.speed_kmh, 10.2, 0.1);
}

// =============================================================================
// GST Message Tests (Accuracy)
// =============================================================================

TEST_F(NmeaParserTest, ParseValidGST)
{
  NmeaGST received_msg;
  bool callback_called = false;
  
  parser_->set_gst_callback([&](const NmeaGST& msg) {
    received_msg = msg;
    callback_called = true;
  });
  
  std::string sentence = create_nmea_sentence(
    "GNGST,120000.00,1.0,0.015,0.012,45.0,0.010,0.008,0.020");
  
  parser_->process(reinterpret_cast<const uint8_t*>(sentence.data()), sentence.size());
  
  ASSERT_TRUE(callback_called);
  EXPECT_TRUE(received_msg.valid);
  EXPECT_NEAR(received_msg.lat_error_m, 0.010, 0.001);
  EXPECT_NEAR(received_msg.lon_error_m, 0.008, 0.001);
  EXPECT_NEAR(received_msg.alt_error_m, 0.020, 0.001);
}

// =============================================================================
// Checksum Tests
// =============================================================================

TEST_F(NmeaParserTest, RejectInvalidChecksum)
{
  bool callback_called = false;
  
  parser_->set_gga_callback([&](const NmeaGGA&) {
    callback_called = true;
  });
  
  // Manually create sentence with wrong checksum
  std::string sentence = "$GNGGA,120000.00,4807.038,N,01131.000,E,4,12,0.9,545.4,M,47.0,M,1.0,0000*FF\r\n";
  
  parser_->process(reinterpret_cast<const uint8_t*>(sentence.data()), sentence.size());
  
  EXPECT_FALSE(callback_called);
  EXPECT_EQ(parser_->stats().checksum_errors, 1u);
}

// =============================================================================
// Stream Processing Tests
// =============================================================================

TEST_F(NmeaParserTest, ProcessPartialData)
{
  NmeaGGA received_msg;
  bool callback_called = false;
  
  parser_->set_gga_callback([&](const NmeaGGA& msg) {
    received_msg = msg;
    callback_called = true;
  });
  
  // Create valid sentence
  std::string full = create_nmea_sentence(
    "GNGGA,120000.00,4807.038,N,01131.000,E,4,12,0.9,545.4,M,47.0,M,1.0,0000");
  
  // Send data in chunks
  size_t half = full.size() / 2;
  std::string part1 = full.substr(0, half);
  std::string part2 = full.substr(half);
  
  parser_->process(reinterpret_cast<const uint8_t*>(part1.data()), part1.size());
  EXPECT_FALSE(callback_called);
  
  parser_->process(reinterpret_cast<const uint8_t*>(part2.data()), part2.size());
  EXPECT_TRUE(callback_called);
}

TEST_F(NmeaParserTest, ProcessMultipleSentences)
{
  int gga_count = 0;
  int rmc_count = 0;
  
  parser_->set_gga_callback([&](const NmeaGGA&) { gga_count++; });
  parser_->set_rmc_callback([&](const NmeaRMC&) { rmc_count++; });
  
  std::string data = 
    create_nmea_sentence("GNGGA,120000.00,4807.038,N,01131.000,E,4,12,0.9,545.4,M,47.0,M,1.0,0000") +
    create_nmea_sentence("GNRMC,120000.00,A,4807.038,N,01131.000,E,0.5,54.7,110126,,,D") +
    create_nmea_sentence("GNGGA,120001.00,4807.038,N,01131.000,E,4,12,0.9,545.4,M,47.0,M,1.0,0000");
  
  parser_->process(reinterpret_cast<const uint8_t*>(data.data()), data.size());
  
  EXPECT_EQ(gga_count, 2);
  EXPECT_EQ(rmc_count, 1);
}

// =============================================================================
// Coordinate Parsing Tests
// =============================================================================

TEST_F(NmeaParserTest, ParseSouthWestCoordinates)
{
  NmeaGGA received_msg;
  bool callback_called = false;
  
  parser_->set_gga_callback([&](const NmeaGGA& msg) {
    received_msg = msg;
    callback_called = true;
  });
  
  // Southern hemisphere, Western hemisphere
  std::string sentence = create_nmea_sentence(
    "GNGGA,120000.00,3356.123,S,07012.456,W,4,12,0.9,100.0,M,0.0,M,1.0,0000");
  
  parser_->process(reinterpret_cast<const uint8_t*>(sentence.data()), sentence.size());
  
  ASSERT_TRUE(callback_called);
  EXPECT_LT(received_msg.latitude_deg, 0.0);   // South = negative
  EXPECT_LT(received_msg.longitude_deg, 0.0);  // West = negative
}

// =============================================================================
// Statistics Tests
// =============================================================================

TEST_F(NmeaParserTest, StatisticsTracking)
{
  parser_->set_gga_callback([](const NmeaGGA&) {});
  
  // Process a valid sentence
  std::string valid = create_nmea_sentence(
    "GNGGA,120000.00,4807.038,N,01131.000,E,4,12,0.9,545.4,M,47.0,M,1.0,0000");
  parser_->process(reinterpret_cast<const uint8_t*>(valid.data()), valid.size());
  
  // Process invalid checksum
  std::string invalid = "$GNGGA,120000.00,,,,,0,00,99.99,,,,,,*FF\r\n";
  parser_->process(reinterpret_cast<const uint8_t*>(invalid.data()), invalid.size());
  
  auto stats = parser_->stats();
  EXPECT_EQ(stats.sentences_parsed, 1u);
  EXPECT_EQ(stats.checksum_errors, 1u);
}

}  // namespace test
}  // namespace gnss_compass
