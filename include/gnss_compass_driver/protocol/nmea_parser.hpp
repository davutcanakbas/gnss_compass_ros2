// Copyright (c) 2026 Davut Can Akbas
// SPDX-License-Identifier: MIT

#ifndef GNSS_COMPASS_DRIVER__PROTOCOL__NMEA_PARSER_HPP_
#define GNSS_COMPASS_DRIVER__PROTOCOL__NMEA_PARSER_HPP_

#include "gnss_compass_driver/types.hpp"

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace gnss_compass
{

// =============================================================================
// NMEA Message Types
// =============================================================================

enum class NmeaMessageType
{
  UNKNOWN,
  GGA,   // Global Positioning System Fix Data
  RMC,   // Recommended Minimum Navigation Information
  VTG,   // Course Over Ground and Ground Speed
  GSA,   // DOP and Active Satellites
  GSV,   // Satellites in View
  GLL,   // Geographic Position - Latitude/Longitude
  GST,   // GNSS Pseudorange Error Statistics
  HDT,   // True Heading
  THS,   // True Heading and Status (u-blox proprietary)
  ZDA    // Time and Date
};

// =============================================================================
// Parsed NMEA Messages
// =============================================================================

struct NmeaGGA
{
  double utc_time{0.0};        // HHMMSS.sss
  double latitude_deg{0.0};    // Decimal degrees
  double longitude_deg{0.0};   // Decimal degrees
  uint8_t fix_quality{0};      // 0=invalid, 1=GPS, 2=DGPS, 4=RTK fixed, 5=RTK float
  uint8_t satellites_used{0};
  float hdop{99.9f};
  double altitude_m{0.0};      // Above MSL
  double geoid_sep_m{0.0};     // Geoid separation
  float diff_age_s{0.0};       // Age of differential corrections
  uint16_t diff_station_id{0}; // Reference station ID
  bool valid{false};
};

struct NmeaRMC
{
  double utc_time{0.0};        // HHMMSS.sss
  bool status_valid{false};    // A=valid, V=warning
  double latitude_deg{0.0};
  double longitude_deg{0.0};
  double speed_knots{0.0};     // Speed over ground
  double course_deg{0.0};      // Course over ground
  uint32_t date{0};            // DDMMYY
  double mag_var_deg{0.0};     // Magnetic variation
  char mode{'N'};              // A=autonomous, D=differential, E=estimated, N=invalid
  bool valid{false};
};

struct NmeaVTG
{
  double course_true_deg{0.0};
  double course_mag_deg{0.0};
  double speed_knots{0.0};
  double speed_kmh{0.0};
  char mode{'N'};
  bool valid{false};
};

struct NmeaGST
{
  double utc_time{0.0};
  double rms_residual{0.0};
  double semi_major_m{0.0};    // Error ellipse semi-major
  double semi_minor_m{0.0};    // Error ellipse semi-minor
  double orientation_deg{0.0}; // Error ellipse orientation
  double lat_error_m{0.0};     // Latitude error (1-sigma)
  double lon_error_m{0.0};     // Longitude error (1-sigma)
  double alt_error_m{0.0};     // Altitude error (1-sigma)
  bool valid{false};
};

struct NmeaHDT
{
  double heading_deg{0.0};     // True heading
  bool valid{false};
};

struct NmeaTHS
{
  double heading_deg{0.0};     // True heading
  char status{'V'};            // A=autonomous, E=estimated, M=manual, S=simulator, V=invalid
  bool valid{false};
};

struct NmeaZDA
{
  double utc_time{0.0};        // HHMMSS.sss
  uint8_t day{0};
  uint8_t month{0};
  uint16_t year{0};
  int8_t local_zone_hours{0};
  int8_t local_zone_minutes{0};
  bool valid{false};
};

// =============================================================================
// NMEA Parser Class
// =============================================================================

class NmeaParser
{
public:
  // Callbacks for parsed messages
  using GGACallback = std::function<void(const NmeaGGA&)>;
  using RMCCallback = std::function<void(const NmeaRMC&)>;
  using VTGCallback = std::function<void(const NmeaVTG&)>;
  using GSTCallback = std::function<void(const NmeaGST&)>;
  using HDTCallback = std::function<void(const NmeaHDT&)>;
  using THSCallback = std::function<void(const NmeaTHS&)>;
  using ZDACallback = std::function<void(const NmeaZDA&)>;
  using ErrorCallback = std::function<void(const std::string&)>;

  NmeaParser();
  ~NmeaParser() = default;

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
   * @brief Process a complete NMEA sentence
   * @param sentence Complete NMEA sentence including $ and checksum
   * @return true if sentence was valid and parsed
   */
  bool parse_sentence(const std::string& sentence);
  
  /**
   * @brief Reset parser state
   */
  void reset();

  // ==========================================================================
  // Callbacks
  // ==========================================================================
  
  void set_gga_callback(GGACallback callback) { gga_callback_ = std::move(callback); }
  void set_rmc_callback(RMCCallback callback) { rmc_callback_ = std::move(callback); }
  void set_vtg_callback(VTGCallback callback) { vtg_callback_ = std::move(callback); }
  void set_gst_callback(GSTCallback callback) { gst_callback_ = std::move(callback); }
  void set_hdt_callback(HDTCallback callback) { hdt_callback_ = std::move(callback); }
  void set_ths_callback(THSCallback callback) { ths_callback_ = std::move(callback); }
  void set_zda_callback(ZDACallback callback) { zda_callback_ = std::move(callback); }
  void set_error_callback(ErrorCallback callback) { error_callback_ = std::move(callback); }

  // ==========================================================================
  // Statistics
  // ==========================================================================
  
  struct Stats
  {
    uint64_t sentences_parsed{0};
    uint64_t checksum_errors{0};
    uint64_t parse_errors{0};
  };
  
  Stats stats() const { return stats_; }
  void reset_stats() { stats_ = Stats{}; }

private:
  // Parsing helpers
  static bool validate_checksum(const std::string& sentence);
  static uint8_t calculate_checksum(std::string_view data);
  static std::vector<std::string_view> split_fields(std::string_view sentence);
  static NmeaMessageType identify_message(std::string_view talker_type);
  
  // Coordinate conversion
  static double parse_latitude(std::string_view value, std::string_view direction);
  static double parse_longitude(std::string_view value, std::string_view direction);
  static double parse_utc_time(std::string_view value);
  
  // Field parsing utilities
  static double parse_double(std::string_view field, double default_value = 0.0);
  static int parse_int(std::string_view field, int default_value = 0);
  static char parse_char(std::string_view field, char default_value = ' ');
  
  // Message parsers
  void parse_gga(const std::vector<std::string_view>& fields);
  void parse_rmc(const std::vector<std::string_view>& fields);
  void parse_vtg(const std::vector<std::string_view>& fields);
  void parse_gst(const std::vector<std::string_view>& fields);
  void parse_hdt(const std::vector<std::string_view>& fields);
  void parse_ths(const std::vector<std::string_view>& fields);
  void parse_zda(const std::vector<std::string_view>& fields);
  
  // Buffer for accumulating data
  std::string buffer_;
  static constexpr size_t MAX_BUFFER_SIZE = 1024;
  
  // Callbacks
  GGACallback gga_callback_;
  RMCCallback rmc_callback_;
  VTGCallback vtg_callback_;
  GSTCallback gst_callback_;
  HDTCallback hdt_callback_;
  THSCallback ths_callback_;
  ZDACallback zda_callback_;
  ErrorCallback error_callback_;
  
  // Statistics
  Stats stats_;
};

}  // namespace gnss_compass

#endif  // GNSS_COMPASS_DRIVER__PROTOCOL__NMEA_PARSER_HPP_

