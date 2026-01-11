// Copyright (c) 2026 Davut Can Akbas
// SPDX-License-Identifier: MIT

#ifndef GNSS_COMPASS_DRIVER__PROTOCOL__UNICORE_PARSER_HPP_
#define GNSS_COMPASS_DRIVER__PROTOCOL__UNICORE_PARSER_HPP_

#include "gnss_compass_driver/types.hpp"

#include <cstdint>
#include <functional>
#include <string>
#include <string_view>
#include <vector>

namespace gnss_compass
{

// =============================================================================
// Unicore Solution Status
// =============================================================================

enum class UniCoreSolutionStatus : uint8_t
{
  SOL_COMPUTED = 0,
  INSUFFICIENT_OBS = 1,
  NO_CONVERGENCE = 2,
  SINGULARITY = 3,
  COV_TRACE = 4,
  TEST_DIST = 5,
  COLD_START = 6,
  V_H_LIMIT = 7,
  VARIANCE = 8,
  RESIDUALS = 9,
  INTEGRITY_WARNING = 13,
  PENDING = 18,
  INVALID_FIX = 19,
  UNAUTHORIZED = 20,
  INVALID_RATE = 22
};

enum class UniCorePosType : uint8_t
{
  NONE = 0,
  FIXEDPOS = 1,
  FIXEDHEIGHT = 2,
  DOPPLER_VELOCITY = 8,
  SINGLE = 16,
  PSRDIFF = 17,
  WAAS = 18,
  PROPAGATED = 19,
  L1_FLOAT = 32,
  IONOFREE_FLOAT = 33,
  NARROW_FLOAT = 34,
  L1_INT = 48,
  WIDE_INT = 49,
  NARROW_INT = 50,
  RTK_DIRECT_INS = 51,
  INS_SBAS = 52,
  INS_PSRSP = 53,
  INS_PSRDIFF = 54,
  INS_RTKFLOAT = 55,
  INS_RTKFIXED = 56,
  PPP_CONVERGING = 68,
  PPP = 69
};

// =============================================================================
// Parsed Unicore Messages
// =============================================================================

/**
 * UNIHEADINGA - Dual antenna heading message
 * Format: #UNIHEADINGA,header;sol_stat,pos_type,baseline,heading,pitch,reserved,
 *         hdg_std,pitch_std,rover_id,num_sv,num_soln,num_obs,num_multi,
 *         reserved,ext_sol_stat,reserved2,sig_mask*checksum
 */
struct UniCoreHeading
{
  UniCoreSolutionStatus sol_status{UniCoreSolutionStatus::INVALID_FIX};
  UniCorePosType pos_type{UniCorePosType::NONE};
  
  double baseline_length_m{0.0};   // Baseline length in meters
  double heading_deg{0.0};         // Heading in degrees (0-360)
  double pitch_deg{0.0};           // Pitch in degrees
  
  double heading_std_deg{0.0};     // Heading standard deviation
  double pitch_std_deg{0.0};       // Pitch standard deviation
  
  uint8_t num_satellites{0};       // Number of satellites tracked
  uint8_t num_soln_satellites{0};  // Number of satellites in solution
  uint8_t num_observations{0};     // Number of observations
  
  TimePoint timestamp;
  bool valid{false};
};

/**
 * BESTNAVA - Best available position and velocity
 * Format: #BESTNAVA,header;pos_sol_stat,pos_type,lat,lon,hgt,undulation,
 *         datum,lat_std,lon_std,hgt_std,stn_id,diff_age,sol_age,
 *         num_sv,num_soln,num_ggL1,num_ggL1L2,reserved,ext_sol_stat,
 *         gal_bds_sig,gps_glo_sig,
 *         vel_sol_stat,vel_type,latency,age,hor_spd,trk_gnd,vert_spd,reserved2*checksum
 */
struct UniCoreBestNav
{
  // Position
  UniCoreSolutionStatus pos_sol_status{UniCoreSolutionStatus::INVALID_FIX};
  UniCorePosType pos_type{UniCorePosType::NONE};
  
  double latitude_deg{0.0};
  double longitude_deg{0.0};
  double height_m{0.0};            // Height above ellipsoid
  double undulation_m{0.0};        // Geoid undulation
  
  double lat_std_m{0.0};
  double lon_std_m{0.0};
  double hgt_std_m{0.0};
  
  float diff_age_s{0.0};
  float sol_age_s{0.0};
  
  uint8_t num_satellites{0};
  uint8_t num_soln_satellites{0};
  
  // Velocity
  UniCoreSolutionStatus vel_sol_status{UniCoreSolutionStatus::INVALID_FIX};
  UniCorePosType vel_type{UniCorePosType::NONE};
  
  double horizontal_speed_mps{0.0};
  double track_ground_deg{0.0};    // Track over ground (course)
  double vertical_speed_mps{0.0};
  
  TimePoint timestamp;
  bool valid{false};
};

// =============================================================================
// Unicore Parser Class
// =============================================================================

class UniCoreParser
{
public:
  using HeadingCallback = std::function<void(const UniCoreHeading&)>;
  using BestNavCallback = std::function<void(const UniCoreBestNav&)>;
  using ErrorCallback = std::function<void(const std::string&)>;

  UniCoreParser();
  ~UniCoreParser() = default;

  // ==========================================================================
  // Data Processing
  // ==========================================================================
  
  /**
   * @brief Process incoming data bytes
   * @param data Pointer to data buffer
   * @param size Number of bytes
   */
  void process(const uint8_t* data, size_t size);
  
  /**
   * @brief Reset parser state
   */
  void reset();

  // ==========================================================================
  // Callbacks
  // ==========================================================================
  
  void set_heading_callback(HeadingCallback cb) { heading_callback_ = std::move(cb); }
  void set_bestnav_callback(BestNavCallback cb) { bestnav_callback_ = std::move(cb); }
  void set_error_callback(ErrorCallback cb) { error_callback_ = std::move(cb); }

  // ==========================================================================
  // Statistics
  // ==========================================================================
  
  struct Stats
  {
    uint64_t messages_parsed{0};
    uint64_t checksum_errors{0};
    uint64_t parse_errors{0};
  };
  
  Stats stats() const { return stats_; }
  void reset_stats() { stats_ = Stats{}; }

private:
  bool parse_message(const std::string& message);
  bool validate_checksum(const std::string& message);
  uint32_t calculate_crc32(const uint8_t* data, size_t len);
  
  std::vector<std::string_view> split_fields(std::string_view data, char delimiter);
  
  void parse_heading(const std::vector<std::string_view>& header_fields,
                     const std::vector<std::string_view>& data_fields);
  void parse_bestnav(const std::vector<std::string_view>& header_fields,
                     const std::vector<std::string_view>& data_fields);
  
  static double parse_double(std::string_view field, double default_value = 0.0);
  static int parse_int(std::string_view field, int default_value = 0);
  static UniCoreSolutionStatus parse_sol_status(std::string_view field);
  static UniCorePosType parse_pos_type(std::string_view field);
  
  // Buffer for accumulating data
  std::string buffer_;
  static constexpr size_t MAX_BUFFER_SIZE = 2048;
  
  // Callbacks
  HeadingCallback heading_callback_;
  BestNavCallback bestnav_callback_;
  ErrorCallback error_callback_;
  
  // Statistics
  Stats stats_;
  
  // CRC32 lookup table
  static const uint32_t crc32_table_[256];
};

// =============================================================================
// String Conversion Utilities
// =============================================================================

inline const char* to_string(UniCoreSolutionStatus status)
{
  switch (status) {
    case UniCoreSolutionStatus::SOL_COMPUTED: return "SOL_COMPUTED";
    case UniCoreSolutionStatus::INSUFFICIENT_OBS: return "INSUFFICIENT_OBS";
    case UniCoreSolutionStatus::COLD_START: return "COLD_START";
    case UniCoreSolutionStatus::INVALID_FIX: return "INVALID_FIX";
    default: return "UNKNOWN";
  }
}

inline const char* to_string(UniCorePosType type)
{
  switch (type) {
    case UniCorePosType::NONE: return "NONE";
    case UniCorePosType::SINGLE: return "SINGLE";
    case UniCorePosType::PSRDIFF: return "PSRDIFF";
    case UniCorePosType::L1_FLOAT: return "L1_FLOAT";
    case UniCorePosType::NARROW_FLOAT: return "NARROW_FLOAT";
    case UniCorePosType::L1_INT: return "L1_INT";
    case UniCorePosType::NARROW_INT: return "NARROW_INT";
    case UniCorePosType::PPP: return "PPP";
    default: return "UNKNOWN";
  }
}

}  // namespace gnss_compass

#endif  // GNSS_COMPASS_DRIVER__PROTOCOL__UNICORE_PARSER_HPP_

