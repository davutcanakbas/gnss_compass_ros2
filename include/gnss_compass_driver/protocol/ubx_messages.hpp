// Copyright (c) 2026 Davut Can Akbas
// SPDX-License-Identifier: MIT

#ifndef GNSS_COMPASS_DRIVER__PROTOCOL__UBX_MESSAGES_HPP_
#define GNSS_COMPASS_DRIVER__PROTOCOL__UBX_MESSAGES_HPP_

#include <cstddef>
#include <cstdint>

namespace gnss_compass
{
namespace ubx
{

// =============================================================================
// UBX Protocol Constants
// =============================================================================

constexpr uint8_t SYNC_CHAR_1 = 0xB5;
constexpr uint8_t SYNC_CHAR_2 = 0x62;

constexpr size_t HEADER_SIZE = 6;      // Sync (2) + Class (1) + ID (1) + Length (2)
constexpr size_t CHECKSUM_SIZE = 2;
constexpr size_t MIN_MESSAGE_SIZE = HEADER_SIZE + CHECKSUM_SIZE;
constexpr size_t MAX_PAYLOAD_SIZE = 8192;

// =============================================================================
// UBX Message Classes
// =============================================================================

enum class MessageClass : uint8_t
{
  NAV = 0x01,    // Navigation Results
  RXM = 0x02,    // Receiver Manager
  INF = 0x04,    // Information
  ACK = 0x05,    // Acknowledgement
  CFG = 0x06,    // Configuration
  UPD = 0x09,    // Firmware Update
  MON = 0x0A,    // Monitoring
  AID = 0x0B,    // AssistNow Aiding (deprecated)
  TIM = 0x0D,    // Timing
  ESF = 0x10,    // External Sensor Fusion
  MGA = 0x13,    // Multiple GNSS Assistance
  LOG = 0x21,    // Logging
  SEC = 0x27,    // Security
  HNR = 0x28,    // High Rate Navigation
};

// =============================================================================
// NAV Message IDs
// =============================================================================

enum class NavMessageId : uint8_t
{
  PVT = 0x07,           // Navigation Position Velocity Time Solution
  STATUS = 0x03,        // Receiver Navigation Status
  POSLLH = 0x02,        // Geodetic Position Solution
  VELNED = 0x12,        // Velocity Solution in NED frame
  TIMEUTC = 0x21,       // UTC Time Solution
  RELPOSNED = 0x3C,     // Relative Positioning (heading!)
  HPPOSLLH = 0x14,      // High Precision Position Solution
  DOP = 0x04,           // Dilution of Precision
  SAT = 0x35,           // Satellite Information
  SIG = 0x43,           // Signal Information
  COV = 0x36,           // Covariance Matrices
};

// =============================================================================
// CFG Message IDs
// =============================================================================

enum class CfgMessageId : uint8_t
{
  MSG = 0x01,           // Message Configuration
  PRT = 0x00,           // Port Configuration
  RATE = 0x08,          // Navigation/Measurement Rate
  NAV5 = 0x24,          // Navigation Engine Settings
  NAVX5 = 0x23,         // Navigation Engine Expert Settings
  VALSET = 0x8A,        // Configuration Set
  VALGET = 0x8B,        // Configuration Get
  VALDEL = 0x8C,        // Configuration Delete
};

// =============================================================================
// Packed Structures for UBX Messages
// =============================================================================

#pragma pack(push, 1)

/**
 * UBX-NAV-PVT: Navigation Position Velocity Time Solution
 * Most comprehensive navigation message
 */
struct NavPvt
{
  uint32_t iTOW;          // GPS time of week (ms)
  uint16_t year;          // Year (UTC)
  uint8_t month;          // Month (1-12)
  uint8_t day;            // Day (1-31)
  uint8_t hour;           // Hour (0-23)
  uint8_t min;            // Minute (0-59)
  uint8_t sec;            // Second (0-60)
  uint8_t valid;          // Validity flags
  uint32_t tAcc;          // Time accuracy estimate (ns)
  int32_t nano;           // Fraction of second (-1e9 to 1e9)
  uint8_t fixType;        // GNSS fix type
  uint8_t flags;          // Fix status flags
  uint8_t flags2;         // Additional flags
  uint8_t numSV;          // Number of satellites used
  int32_t lon;            // Longitude (1e-7 deg)
  int32_t lat;            // Latitude (1e-7 deg)
  int32_t height;         // Height above ellipsoid (mm)
  int32_t hMSL;           // Height above mean sea level (mm)
  uint32_t hAcc;          // Horizontal accuracy estimate (mm)
  uint32_t vAcc;          // Vertical accuracy estimate (mm)
  int32_t velN;           // NED north velocity (mm/s)
  int32_t velE;           // NED east velocity (mm/s)
  int32_t velD;           // NED down velocity (mm/s)
  int32_t gSpeed;         // Ground speed (mm/s)
  int32_t headMot;        // Heading of motion (1e-5 deg)
  uint32_t sAcc;          // Speed accuracy estimate (mm/s)
  uint32_t headAcc;       // Heading accuracy estimate (1e-5 deg)
  uint16_t pDOP;          // Position DOP (0.01)
  uint8_t flags3;         // Additional flags
  uint8_t reserved1[5];   // Reserved
  int32_t headVeh;        // Heading of vehicle (1e-5 deg)
  int16_t magDec;         // Magnetic declination (1e-2 deg)
  uint16_t magAcc;        // Magnetic declination accuracy (1e-2 deg)
  
  // Helper methods
  double latitude_deg() const { return lat * 1e-7; }
  double longitude_deg() const { return lon * 1e-7; }
  double altitude_m() const { return height * 1e-3; }
  double altitude_msl_m() const { return hMSL * 1e-3; }
  double horizontal_accuracy_m() const { return hAcc * 1e-3; }
  double vertical_accuracy_m() const { return vAcc * 1e-3; }
  double vel_north_mps() const { return velN * 1e-3; }
  double vel_east_mps() const { return velE * 1e-3; }
  double vel_down_mps() const { return velD * 1e-3; }
  double ground_speed_mps() const { return gSpeed * 1e-3; }
  double heading_motion_deg() const { return headMot * 1e-5; }
  double heading_vehicle_deg() const { return headVeh * 1e-5; }
  double speed_accuracy_mps() const { return sAcc * 1e-3; }
  double heading_accuracy_deg() const { return headAcc * 1e-5; }
  double pdop() const { return pDOP * 0.01; }
  
  // Validity flags
  bool valid_date() const { return (valid & 0x01) != 0; }
  bool valid_time() const { return (valid & 0x02) != 0; }
  bool fully_resolved() const { return (valid & 0x04) != 0; }
  bool valid_mag() const { return (valid & 0x08) != 0; }
  
  // Fix flags
  bool gnss_fix_ok() const { return (flags & 0x01) != 0; }
  bool diff_soln() const { return (flags & 0x02) != 0; }
  uint8_t carrier_soln() const { return (flags >> 6) & 0x03; }  // 0=none, 1=float, 2=fixed
  bool head_veh_valid() const { return (flags & 0x20) != 0; }
};

/**
 * UBX-NAV-RELPOSNED: Relative Positioning Information
 * Critical for dual-antenna heading calculation!
 */
struct NavRelPosNed
{
  uint8_t version;        // Message version (0x01 for 9th gen)
  uint8_t reserved1;
  uint16_t refStationId;  // Reference station ID
  uint32_t iTOW;          // GPS time of week (ms)
  int32_t relPosN;        // North component (cm)
  int32_t relPosE;        // East component (cm)
  int32_t relPosD;        // Down component (cm)
  int32_t relPosLength;   // Length of relative position vector (cm)
  int32_t relPosHeading;  // Heading of relative position (1e-5 deg)
  uint8_t reserved2[4];
  int8_t relPosHPN;       // High-precision North (0.1mm)
  int8_t relPosHPE;       // High-precision East (0.1mm)
  int8_t relPosHPD;       // High-precision Down (0.1mm)
  int8_t relPosHPLength;  // High-precision length (0.1mm)
  uint32_t accN;          // North accuracy (0.1mm)
  uint32_t accE;          // East accuracy (0.1mm)
  uint32_t accD;          // Down accuracy (0.1mm)
  uint32_t accLength;     // Length accuracy (0.1mm)
  uint32_t accHeading;    // Heading accuracy (1e-5 deg)
  uint8_t reserved3[4];
  uint32_t flags;         // Status flags
  
  // Helper methods
  double rel_pos_n_m() const { return relPosN * 0.01 + relPosHPN * 0.0001; }
  double rel_pos_e_m() const { return relPosE * 0.01 + relPosHPE * 0.0001; }
  double rel_pos_d_m() const { return relPosD * 0.01 + relPosHPD * 0.0001; }
  double baseline_length_m() const { return relPosLength * 0.01 + relPosHPLength * 0.0001; }
  double heading_deg() const { return relPosHeading * 1e-5; }
  double accuracy_n_m() const { return accN * 0.0001; }
  double accuracy_e_m() const { return accE * 0.0001; }
  double accuracy_d_m() const { return accD * 0.0001; }
  double heading_accuracy_deg() const { return accHeading * 1e-5; }
  
  // Flags
  bool gnss_fix_ok() const { return (flags & 0x01) != 0; }
  bool diff_soln() const { return (flags & 0x02) != 0; }
  bool rel_pos_valid() const { return (flags & 0x04) != 0; }
  uint8_t carrier_soln() const { return (flags >> 3) & 0x03; }  // 0=none, 1=float, 2=fixed
  bool is_moving() const { return (flags & 0x20) != 0; }
  bool ref_pos_miss() const { return (flags & 0x40) != 0; }
  bool ref_obs_miss() const { return (flags & 0x80) != 0; }
  bool rel_pos_heading_valid() const { return (flags & 0x100) != 0; }
  bool rel_pos_normalized() const { return (flags & 0x200) != 0; }
};

/**
 * UBX-NAV-STATUS: Receiver Navigation Status
 */
struct NavStatus
{
  uint32_t iTOW;          // GPS time of week (ms)
  uint8_t gpsFix;         // GPS fix type
  uint8_t flags;          // Navigation status flags
  uint8_t fixStat;        // Fix status information
  uint8_t flags2;         // Additional flags
  uint32_t ttff;          // Time to first fix (ms)
  uint32_t msss;          // Milliseconds since startup/reset
  
  bool gps_fix_ok() const { return (flags & 0x01) != 0; }
  bool diff_soln() const { return (flags & 0x02) != 0; }
  bool week_set() const { return (flags & 0x04) != 0; }
  bool tow_set() const { return (flags & 0x08) != 0; }
};

/**
 * UBX-NAV-DOP: Dilution of Precision
 */
struct NavDop
{
  uint32_t iTOW;          // GPS time of week (ms)
  uint16_t gDOP;          // Geometric DOP (0.01)
  uint16_t pDOP;          // Position DOP (0.01)
  uint16_t tDOP;          // Time DOP (0.01)
  uint16_t vDOP;          // Vertical DOP (0.01)
  uint16_t hDOP;          // Horizontal DOP (0.01)
  uint16_t nDOP;          // Northing DOP (0.01)
  uint16_t eDOP;          // Easting DOP (0.01)
  
  double gdop() const { return gDOP * 0.01; }
  double pdop() const { return pDOP * 0.01; }
  double tdop() const { return tDOP * 0.01; }
  double vdop() const { return vDOP * 0.01; }
  double hdop() const { return hDOP * 0.01; }
};

/**
 * UBX-ACK-ACK/NAK: Acknowledgement
 */
struct AckAck
{
  uint8_t clsID;          // Class ID of acknowledged message
  uint8_t msgID;          // Message ID of acknowledged message
};

#pragma pack(pop)

// =============================================================================
// Message Size Validation
// =============================================================================

constexpr size_t NAV_PVT_SIZE = sizeof(NavPvt);
constexpr size_t NAV_RELPOSNED_SIZE = sizeof(NavRelPosNed);
constexpr size_t NAV_STATUS_SIZE = sizeof(NavStatus);
constexpr size_t NAV_DOP_SIZE = sizeof(NavDop);
constexpr size_t ACK_ACK_SIZE = sizeof(AckAck);

static_assert(NAV_PVT_SIZE == 92, "NavPvt size mismatch");
static_assert(NAV_RELPOSNED_SIZE == 64, "NavRelPosNed size mismatch");
static_assert(NAV_STATUS_SIZE == 16, "NavStatus size mismatch");
static_assert(NAV_DOP_SIZE == 18, "NavDop size mismatch");

}  // namespace ubx
}  // namespace gnss_compass

#endif  // GNSS_COMPASS_DRIVER__PROTOCOL__UBX_MESSAGES_HPP_

