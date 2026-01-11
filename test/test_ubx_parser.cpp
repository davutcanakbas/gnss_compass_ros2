// Copyright (c) 2026 Davut Can Akbas
// SPDX-License-Identifier: MIT

#include "gnss_compass_driver/protocol/ubx_parser.hpp"

#include <gtest/gtest.h>
#include <cstring>

namespace gnss_compass
{
namespace test
{

class UbxParserTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    parser_ = std::make_unique<UbxParser>();
  }

  std::unique_ptr<UbxParser> parser_;
};

// =============================================================================
// Message Creation Tests
// =============================================================================

TEST_F(UbxParserTest, CreateEmptyMessage)
{
  auto msg = UbxParser::create_message(0x01, 0x07);  // NAV-PVT poll
  
  ASSERT_EQ(msg.size(), 8u);  // Header (6) + Checksum (2)
  EXPECT_EQ(msg[0], 0xB5);    // Sync 1
  EXPECT_EQ(msg[1], 0x62);    // Sync 2
  EXPECT_EQ(msg[2], 0x01);    // Class
  EXPECT_EQ(msg[3], 0x07);    // ID
  EXPECT_EQ(msg[4], 0x00);    // Length low
  EXPECT_EQ(msg[5], 0x00);    // Length high
}

TEST_F(UbxParserTest, CreateMessageWithPayload)
{
  // CFG-MSG: NAV-PVT, rate=1
  auto msg = UbxParser::create_cfg_msg(0x01, 0x07, 0x01);
  
  ASSERT_GE(msg.size(), 11u);  // Header (6) + Payload (3) + Checksum (2)
  EXPECT_EQ(msg[0], 0xB5);
  EXPECT_EQ(msg[1], 0x62);
  EXPECT_EQ(msg[2], 0x06);     // CFG class
  EXPECT_EQ(msg[3], 0x01);     // MSG ID
  EXPECT_EQ(msg[4], 0x03);     // Length = 3
  EXPECT_EQ(msg[5], 0x00);
  EXPECT_EQ(msg[6], 0x01);     // NAV class
  EXPECT_EQ(msg[7], 0x07);     // PVT ID
  EXPECT_EQ(msg[8], 0x01);     // Rate = 1
}

TEST_F(UbxParserTest, CreateCfgRate)
{
  auto msg = UbxParser::create_cfg_rate(100, 1);  // 100ms = 10Hz
  
  ASSERT_GE(msg.size(), 14u);  // Header (6) + Payload (6) + Checksum (2)
  EXPECT_EQ(msg[2], 0x06);     // CFG class
  EXPECT_EQ(msg[3], 0x08);     // RATE ID
  EXPECT_EQ(msg[4], 0x06);     // Length = 6
  EXPECT_EQ(msg[6], 100);      // measRate low byte
  EXPECT_EQ(msg[7], 0);        // measRate high byte
}

// =============================================================================
// NAV-PVT Parsing Tests
// =============================================================================

TEST_F(UbxParserTest, ParseNavPvt)
{
  ubx::NavPvt received_msg;
  bool callback_called = false;
  
  parser_->set_nav_pvt_callback([&](const ubx::NavPvt& msg) {
    received_msg = msg;
    callback_called = true;
  });
  
  // Create a NAV-PVT message
  ubx::NavPvt pvt{};
  pvt.lat = 481173000;        // 48.1173 degrees
  pvt.lon = 115167000;        // 11.5167 degrees
  pvt.height = 545400;        // 545.4 meters
  pvt.hAcc = 1500;            // 1.5 meters
  pvt.vAcc = 3000;            // 3.0 meters
  pvt.fixType = 3;            // 3D fix
  pvt.flags = 0x01 | (0x02 << 6);  // gnssFixOk, carrier=RTK fixed
  pvt.numSV = 12;
  pvt.velN = 1000;            // 1.0 m/s north
  pvt.velE = 500;             // 0.5 m/s east
  pvt.velD = -100;            // 0.1 m/s up
  
  auto msg = UbxParser::create_message(
    static_cast<uint8_t>(ubx::MessageClass::NAV),
    static_cast<uint8_t>(ubx::NavMessageId::PVT),
    reinterpret_cast<const uint8_t*>(&pvt),
    sizeof(pvt));
  
  parser_->process(msg.data(), msg.size());
  
  ASSERT_TRUE(callback_called);
  EXPECT_NEAR(received_msg.latitude_deg(), 48.1173, 0.0001);
  EXPECT_NEAR(received_msg.longitude_deg(), 11.5167, 0.0001);
  EXPECT_NEAR(received_msg.altitude_m(), 545.4, 0.1);
  EXPECT_NEAR(received_msg.horizontal_accuracy_m(), 1.5, 0.01);
  EXPECT_EQ(received_msg.fixType, 3);
  EXPECT_TRUE(received_msg.gnss_fix_ok());
  EXPECT_EQ(received_msg.carrier_soln(), 2);  // RTK fixed
}

// =============================================================================
// NAV-RELPOSNED Parsing Tests
// =============================================================================

TEST_F(UbxParserTest, ParseNavRelPosNed)
{
  ubx::NavRelPosNed received_msg;
  bool callback_called = false;
  
  parser_->set_nav_relposned_callback([&](const ubx::NavRelPosNed& msg) {
    received_msg = msg;
    callback_called = true;
  });
  
  // Create a NAV-RELPOSNED message
  ubx::NavRelPosNed relpos{};
  relpos.version = 0x01;
  relpos.relPosN = 100;           // 1.0 meter north
  relpos.relPosE = 0;             // 0 meters east  
  relpos.relPosD = -10;           // 0.1 meter down (antenna tilt)
  relpos.relPosLength = 100;      // 1.0 meter baseline
  relpos.relPosHeading = 4500000; // 45.0 degrees
  relpos.accHeading = 100000;     // 1.0 degree accuracy
  relpos.flags = 0x01 | 0x04 | (0x02 << 3) | 0x100;  // gnssFixOk, relPosValid, carrierFixed, headingValid
  
  auto msg = UbxParser::create_message(
    static_cast<uint8_t>(ubx::MessageClass::NAV),
    static_cast<uint8_t>(ubx::NavMessageId::RELPOSNED),
    reinterpret_cast<const uint8_t*>(&relpos),
    sizeof(relpos));
  
  parser_->process(msg.data(), msg.size());
  
  ASSERT_TRUE(callback_called);
  EXPECT_NEAR(received_msg.heading_deg(), 45.0, 0.01);
  EXPECT_NEAR(received_msg.baseline_length_m(), 1.0, 0.01);
  EXPECT_NEAR(received_msg.heading_accuracy_deg(), 1.0, 0.01);
  EXPECT_TRUE(received_msg.rel_pos_valid());
  EXPECT_TRUE(received_msg.rel_pos_heading_valid());
  EXPECT_EQ(received_msg.carrier_soln(), 2);  // RTK fixed
}

// =============================================================================
// Checksum Tests
// =============================================================================

TEST_F(UbxParserTest, RejectInvalidChecksum)
{
  bool callback_called = false;
  
  parser_->set_nav_pvt_callback([&](const ubx::NavPvt&) {
    callback_called = true;
  });
  
  // Create valid message then corrupt checksum
  ubx::NavPvt pvt{};
  auto msg = UbxParser::create_message(
    static_cast<uint8_t>(ubx::MessageClass::NAV),
    static_cast<uint8_t>(ubx::NavMessageId::PVT),
    reinterpret_cast<const uint8_t*>(&pvt),
    sizeof(pvt));
  
  // Corrupt the last byte (checksum B)
  msg.back() ^= 0xFF;
  
  parser_->process(msg.data(), msg.size());
  
  EXPECT_FALSE(callback_called);
  EXPECT_EQ(parser_->stats().checksum_errors, 1u);
}

// =============================================================================
// Stream Processing Tests
// =============================================================================

TEST_F(UbxParserTest, ProcessPartialData)
{
  bool callback_called = false;
  
  parser_->set_nav_pvt_callback([&](const ubx::NavPvt&) {
    callback_called = true;
  });
  
  ubx::NavPvt pvt{};
  auto msg = UbxParser::create_message(
    static_cast<uint8_t>(ubx::MessageClass::NAV),
    static_cast<uint8_t>(ubx::NavMessageId::PVT),
    reinterpret_cast<const uint8_t*>(&pvt),
    sizeof(pvt));
  
  // Split message in half
  size_t half = msg.size() / 2;
  
  parser_->process(msg.data(), half);
  EXPECT_FALSE(callback_called);
  
  parser_->process(msg.data() + half, msg.size() - half);
  EXPECT_TRUE(callback_called);
}

TEST_F(UbxParserTest, IgnoreGarbageBeforeSync)
{
  bool callback_called = false;
  
  parser_->set_nav_pvt_callback([&](const ubx::NavPvt&) {
    callback_called = true;
  });
  
  ubx::NavPvt pvt{};
  auto msg = UbxParser::create_message(
    static_cast<uint8_t>(ubx::MessageClass::NAV),
    static_cast<uint8_t>(ubx::NavMessageId::PVT),
    reinterpret_cast<const uint8_t*>(&pvt),
    sizeof(pvt));
  
  // Prepend garbage
  std::vector<uint8_t> data = {0x00, 0xFF, 0x12, 0x34, 0xAB, 0xCD};
  data.insert(data.end(), msg.begin(), msg.end());
  
  parser_->process(data.data(), data.size());
  
  EXPECT_TRUE(callback_called);
}

TEST_F(UbxParserTest, ProcessMixedWithNmea)
{
  bool ubx_callback_called = false;
  
  parser_->set_nav_pvt_callback([&](const ubx::NavPvt&) {
    ubx_callback_called = true;
  });
  
  ubx::NavPvt pvt{};
  auto ubx_msg = UbxParser::create_message(
    static_cast<uint8_t>(ubx::MessageClass::NAV),
    static_cast<uint8_t>(ubx::NavMessageId::PVT),
    reinterpret_cast<const uint8_t*>(&pvt),
    sizeof(pvt));
  
  // Mix NMEA and UBX data
  std::string nmea = "$GNGGA,120000.00,4807.038,N,01131.000,E,4,12,0.9,545.4,M,47.0,M,1.0,0000*47\r\n";
  
  std::vector<uint8_t> mixed_data(nmea.begin(), nmea.end());
  mixed_data.insert(mixed_data.end(), ubx_msg.begin(), ubx_msg.end());
  
  parser_->process(mixed_data.data(), mixed_data.size());
  
  EXPECT_TRUE(ubx_callback_called);
}

// =============================================================================
// Reset Tests
// =============================================================================

TEST_F(UbxParserTest, ResetClearsState)
{
  // Start parsing a message
  uint8_t partial[] = {0xB5, 0x62, 0x01, 0x07};
  parser_->process(partial, sizeof(partial));
  
  // Reset
  parser_->reset();
  
  // Now a complete different message should work
  bool callback_called = false;
  parser_->set_nav_dop_callback([&](const ubx::NavDop&) {
    callback_called = true;
  });
  
  ubx::NavDop dop{};
  auto msg = UbxParser::create_message(
    static_cast<uint8_t>(ubx::MessageClass::NAV),
    static_cast<uint8_t>(ubx::NavMessageId::DOP),
    reinterpret_cast<const uint8_t*>(&dop),
    sizeof(dop));
  
  parser_->process(msg.data(), msg.size());
  EXPECT_TRUE(callback_called);
}

}  // namespace test
}  // namespace gnss_compass

