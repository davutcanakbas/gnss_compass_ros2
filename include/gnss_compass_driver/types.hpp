// Copyright (c) 2026 Davut Can Akbas
// SPDX-License-Identifier: MIT

#ifndef GNSS_COMPASS_DRIVER__TYPES_HPP_
#define GNSS_COMPASS_DRIVER__TYPES_HPP_

#include <cstdint>
#include <chrono>
#include <optional>
#include <string>

namespace gnss_compass
{

// =============================================================================
// Time Types
// =============================================================================

using Clock = std::chrono::steady_clock;
using TimePoint = Clock::time_point;
using Duration = Clock::duration;

// =============================================================================
// GNSS Fix Types
// =============================================================================

enum class FixType : uint8_t
{
  NO_FIX = 0,
  DEAD_RECKONING = 1,
  FIX_2D = 2,
  FIX_3D = 3,
  GNSS_DEAD_RECKONING = 4,
  TIME_ONLY = 5
};

enum class RtkStatus : uint8_t
{
  NONE = 0,
  FLOAT = 1,
  FIXED = 2
};

// =============================================================================
// Position Data
// =============================================================================

struct GnssPosition
{
  double latitude_deg{0.0};   // Degrees, positive = North
  double longitude_deg{0.0};  // Degrees, positive = East
  double altitude_m{0.0};     // Meters above ellipsoid
  double undulation_m{0.0};   // Geoid separation

  // Covariance (meters^2)
  double position_covariance[9]{0.0};
  
  // Accuracy estimates (1-sigma, meters)
  double horizontal_accuracy_m{0.0};
  double vertical_accuracy_m{0.0};

  // Quality
  FixType fix_type{FixType::NO_FIX};
  RtkStatus rtk_status{RtkStatus::NONE};
  uint8_t satellites_used{0};
  
  // Dilution of precision
  float hdop{99.9f};
  float vdop{99.9f};
  float pdop{99.9f};
  
  // Timestamp
  TimePoint timestamp;
  bool valid{false};
};

// =============================================================================
// Velocity Data
// =============================================================================

struct GnssVelocity
{
  // NED frame velocities (m/s)
  double vel_north_mps{0.0};
  double vel_east_mps{0.0};
  double vel_down_mps{0.0};
  
  // Ground speed and course
  double ground_speed_mps{0.0};
  double course_deg{0.0};  // True course over ground
  
  // Accuracy (1-sigma, m/s)
  double speed_accuracy_mps{0.0};
  double course_accuracy_deg{0.0};
  
  TimePoint timestamp;
  bool valid{false};
};

// =============================================================================
// Heading Data (Dual Antenna)
// =============================================================================

struct GnssHeading
{
  // True heading from dual antenna (degrees, 0-360)
  double heading_deg{0.0};
  
  // Pitch from antenna baseline (degrees)
  double pitch_deg{0.0};
  
  // Relative position NED (meters) - rover relative to base antenna
  double rel_pos_n_m{0.0};
  double rel_pos_e_m{0.0};
  double rel_pos_d_m{0.0};
  
  // Baseline length between antennas
  double baseline_length_m{0.0};
  
  // Accuracy estimates (1-sigma)
  double heading_accuracy_deg{0.0};
  
  // Quality flags
  bool heading_valid{false};
  bool rel_pos_valid{false};
  bool baseline_fixed{false};  // RTK fixed between antennas
  bool carrier_solution_valid{false};
  
  TimePoint timestamp;
};

// =============================================================================
// Time Data
// =============================================================================

struct GnssTime
{
  uint16_t year{0};
  uint8_t month{0};
  uint8_t day{0};
  uint8_t hour{0};
  uint8_t minute{0};
  double second{0.0};  // Includes fractional seconds
  
  // GPS time
  uint16_t gps_week{0};
  double gps_tow_s{0.0};  // Time of week in seconds
  
  // UTC offset
  int8_t utc_offset_s{0};
  
  // Validity
  bool date_valid{false};
  bool time_valid{false};
  bool fully_resolved{false};
  
  TimePoint timestamp;
};

// =============================================================================
// Combined GNSS Data
// =============================================================================

struct GnssData
{
  GnssPosition position;
  GnssVelocity velocity;
  GnssHeading heading;
  GnssTime time;
  
  // Additional status
  float diff_age_s{0.0};  // Age of differential corrections
  uint8_t satellites_visible{0};
  
  TimePoint last_update;
};

// =============================================================================
// String Conversion Utilities
// =============================================================================

inline const char* to_string(FixType type)
{
  switch (type) {
    case FixType::NO_FIX: return "NO_FIX";
    case FixType::DEAD_RECKONING: return "DEAD_RECKONING";
    case FixType::FIX_2D: return "2D_FIX";
    case FixType::FIX_3D: return "3D_FIX";
    case FixType::GNSS_DEAD_RECKONING: return "GNSS_DEAD_RECKONING";
    case FixType::TIME_ONLY: return "TIME_ONLY";
    default: return "UNKNOWN";
  }
}

inline const char* to_string(RtkStatus status)
{
  switch (status) {
    case RtkStatus::NONE: return "NONE";
    case RtkStatus::FLOAT: return "FLOAT";
    case RtkStatus::FIXED: return "FIXED";
    default: return "UNKNOWN";
  }
}

}  // namespace gnss_compass

#endif  // GNSS_COMPASS_DRIVER__TYPES_HPP_

