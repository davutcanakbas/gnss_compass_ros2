// Copyright (c) 2026 Davut Can Akbas
// SPDX-License-Identifier: MIT

#include "gnss_compass_driver/protocol/nmea_parser.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <stdexcept>

namespace gnss_compass
{

// =============================================================================
// Construction
// =============================================================================

NmeaParser::NmeaParser()
{
  buffer_.reserve(MAX_BUFFER_SIZE);
}

// =============================================================================
// Data Processing
// =============================================================================

void NmeaParser::process(const uint8_t* data, size_t size)
{
  for (size_t i = 0; i < size; ++i) {
    char c = static_cast<char>(data[i]);
    
    if (c == '$' || c == '!') {
      // Start of new sentence
      buffer_.clear();
      buffer_ += c;
    } else if (c == '\r' || c == '\n') {
      // End of sentence
      if (!buffer_.empty() && (buffer_[0] == '$' || buffer_[0] == '!')) {
        parse_sentence(buffer_);
      }
      buffer_.clear();
    } else if (buffer_.size() < MAX_BUFFER_SIZE) {
      buffer_ += c;
    } else {
      // Buffer overflow - reset
      buffer_.clear();
    }
  }
}

bool NmeaParser::parse_sentence(const std::string& sentence)
{
  if (sentence.size() < 10) {  // Minimum valid sentence length
    return false;
  }
  
  // Validate checksum
  if (!validate_checksum(sentence)) {
    stats_.checksum_errors++;
    if (error_callback_) {
      error_callback_("Checksum error in sentence: " + sentence.substr(0, 20));
    }
    return false;
  }
  
  // Find the asterisk (checksum delimiter)
  size_t asterisk_pos = sentence.find('*');
  if (asterisk_pos == std::string::npos) {
    return false;
  }
  
  // Extract the data portion (between $ and *)
  std::string_view data_part(sentence.data() + 1, asterisk_pos - 1);
  
  // Split into fields
  auto fields = split_fields(data_part);
  if (fields.empty()) {
    return false;
  }
  
  // Identify message type (first field contains talker ID + message type)
  std::string_view first_field = fields[0];
  if (first_field.size() < 5) {
    return false;
  }
  
  // Extract message type (last 3 characters)
  std::string_view msg_type = first_field.substr(first_field.size() - 3);
  NmeaMessageType type = identify_message(msg_type);
  
  // Parse based on type
  try {
    switch (type) {
      case NmeaMessageType::GGA:
        parse_gga(fields);
        break;
      case NmeaMessageType::RMC:
        parse_rmc(fields);
        break;
      case NmeaMessageType::VTG:
        parse_vtg(fields);
        break;
      case NmeaMessageType::GST:
        parse_gst(fields);
        break;
      case NmeaMessageType::HDT:
        parse_hdt(fields);
        break;
      case NmeaMessageType::THS:
        parse_ths(fields);
        break;
      case NmeaMessageType::ZDA:
        parse_zda(fields);
        break;
      default:
        // Unknown message type - ignore
        return false;
    }
    
    stats_.sentences_parsed++;
    return true;
    
  } catch (const std::exception& e) {
    stats_.parse_errors++;
    if (error_callback_) {
      error_callback_("Parse error: " + std::string(e.what()));
    }
    return false;
  }
}

void NmeaParser::reset()
{
  buffer_.clear();
}

// =============================================================================
// Checksum Validation
// =============================================================================

bool NmeaParser::validate_checksum(const std::string& sentence)
{
  size_t asterisk_pos = sentence.find('*');
  if (asterisk_pos == std::string::npos || asterisk_pos + 2 >= sentence.size()) {
    return false;
  }
  
  // Calculate checksum of data between $ and *
  if (sentence[0] != '$' && sentence[0] != '!') {
    return false;
  }
  
  std::string_view data(sentence.data() + 1, asterisk_pos - 1);
  uint8_t calculated = calculate_checksum(data);
  
  // Parse provided checksum (2 hex digits after *)
  std::string_view checksum_str(sentence.data() + asterisk_pos + 1, 2);
  uint8_t provided = 0;
  auto result = std::from_chars(checksum_str.data(), checksum_str.data() + 2, provided, 16);
  
  if (result.ec != std::errc()) {
    return false;
  }
  
  return calculated == provided;
}

uint8_t NmeaParser::calculate_checksum(std::string_view data)
{
  uint8_t checksum = 0;
  for (char c : data) {
    checksum ^= static_cast<uint8_t>(c);
  }
  return checksum;
}

// =============================================================================
// Field Parsing
// =============================================================================

std::vector<std::string_view> NmeaParser::split_fields(std::string_view sentence)
{
  std::vector<std::string_view> fields;
  fields.reserve(20);  // Most NMEA sentences have fewer than 20 fields
  
  size_t start = 0;
  size_t pos = 0;
  
  while (pos <= sentence.size()) {
    if (pos == sentence.size() || sentence[pos] == ',') {
      fields.push_back(sentence.substr(start, pos - start));
      start = pos + 1;
    }
    ++pos;
  }
  
  return fields;
}

NmeaMessageType NmeaParser::identify_message(std::string_view type)
{
  if (type == "GGA") return NmeaMessageType::GGA;
  if (type == "RMC") return NmeaMessageType::RMC;
  if (type == "VTG") return NmeaMessageType::VTG;
  if (type == "GSA") return NmeaMessageType::GSA;
  if (type == "GSV") return NmeaMessageType::GSV;
  if (type == "GLL") return NmeaMessageType::GLL;
  if (type == "GST") return NmeaMessageType::GST;
  if (type == "HDT") return NmeaMessageType::HDT;
  if (type == "THS") return NmeaMessageType::THS;
  if (type == "ZDA") return NmeaMessageType::ZDA;
  return NmeaMessageType::UNKNOWN;
}

double NmeaParser::parse_double(std::string_view field, double default_value)
{
  if (field.empty()) {
    return default_value;
  }
  
  double value = default_value;
  std::from_chars(field.data(), field.data() + field.size(), value);
  return value;
}

int NmeaParser::parse_int(std::string_view field, int default_value)
{
  if (field.empty()) {
    return default_value;
  }
  
  int value = default_value;
  std::from_chars(field.data(), field.data() + field.size(), value);
  return value;
}

char NmeaParser::parse_char(std::string_view field, char default_value)
{
  return field.empty() ? default_value : field[0];
}

double NmeaParser::parse_utc_time(std::string_view value)
{
  return parse_double(value, 0.0);
}

double NmeaParser::parse_latitude(std::string_view value, std::string_view direction)
{
  if (value.empty()) {
    return 0.0;
  }
  
  // Format: DDMM.MMMMM
  // First 2 digits are degrees, rest is minutes
  if (value.size() < 4) {
    return 0.0;
  }
  
  double raw = parse_double(value, 0.0);
  int degrees = static_cast<int>(raw / 100);
  double minutes = raw - (degrees * 100);
  double decimal = degrees + (minutes / 60.0);
  
  if (!direction.empty() && (direction[0] == 'S' || direction[0] == 's')) {
    decimal = -decimal;
  }
  
  return decimal;
}

double NmeaParser::parse_longitude(std::string_view value, std::string_view direction)
{
  if (value.empty()) {
    return 0.0;
  }
  
  // Format: DDDMM.MMMMM
  // First 3 digits are degrees, rest is minutes
  if (value.size() < 5) {
    return 0.0;
  }
  
  double raw = parse_double(value, 0.0);
  int degrees = static_cast<int>(raw / 100);
  double minutes = raw - (degrees * 100);
  double decimal = degrees + (minutes / 60.0);
  
  if (!direction.empty() && (direction[0] == 'W' || direction[0] == 'w')) {
    decimal = -decimal;
  }
  
  return decimal;
}

// =============================================================================
// Message Parsers
// =============================================================================

void NmeaParser::parse_gga(const std::vector<std::string_view>& fields)
{
  // $xxGGA,time,lat,N/S,lon,E/W,quality,sats,hdop,alt,M,geoid,M,age,station*cs
  // Index:  0    1   2   3   4   5    6      7    8   9  10  11  12  13    14
  
  if (fields.size() < 15) {
    return;
  }
  
  NmeaGGA msg;
  msg.utc_time = parse_utc_time(fields[1]);
  msg.latitude_deg = parse_latitude(fields[2], fields[3]);
  msg.longitude_deg = parse_longitude(fields[4], fields[5]);
  msg.fix_quality = static_cast<uint8_t>(parse_int(fields[6], 0));
  msg.satellites_used = static_cast<uint8_t>(parse_int(fields[7], 0));
  msg.hdop = static_cast<float>(parse_double(fields[8], 99.9));
  msg.altitude_m = parse_double(fields[9], 0.0);
  msg.geoid_sep_m = parse_double(fields[11], 0.0);
  msg.diff_age_s = static_cast<float>(parse_double(fields[13], 0.0));
  msg.diff_station_id = static_cast<uint16_t>(parse_int(fields[14], 0));
  msg.valid = (msg.fix_quality > 0);
  
  if (gga_callback_) {
    gga_callback_(msg);
  }
}

void NmeaParser::parse_rmc(const std::vector<std::string_view>& fields)
{
  // $xxRMC,time,status,lat,N/S,lon,E/W,speed,course,date,magvar,E/W,mode*cs
  // Index:  0    1      2    3   4   5   6     7      8     9     10   11
  
  if (fields.size() < 12) {
    return;
  }
  
  NmeaRMC msg;
  msg.utc_time = parse_utc_time(fields[1]);
  msg.status_valid = (parse_char(fields[2], 'V') == 'A');
  msg.latitude_deg = parse_latitude(fields[3], fields[4]);
  msg.longitude_deg = parse_longitude(fields[5], fields[6]);
  msg.speed_knots = parse_double(fields[7], 0.0);
  msg.course_deg = parse_double(fields[8], 0.0);
  msg.date = static_cast<uint32_t>(parse_int(fields[9], 0));
  msg.mag_var_deg = parse_double(fields[10], 0.0);
  
  if (!fields[11].empty() && (fields[11][0] == 'W' || fields[11][0] == 'w')) {
    msg.mag_var_deg = -msg.mag_var_deg;
  }
  
  msg.mode = (fields.size() > 12) ? parse_char(fields[12], 'N') : 'N';
  msg.valid = msg.status_valid;
  
  if (rmc_callback_) {
    rmc_callback_(msg);
  }
}

void NmeaParser::parse_vtg(const std::vector<std::string_view>& fields)
{
  // $xxVTG,course_t,T,course_m,M,speed_n,N,speed_k,K,mode*cs
  // Index:    0      1    2    3    4    5    6    7   8
  
  if (fields.size() < 9) {
    return;
  }
  
  NmeaVTG msg;
  msg.course_true_deg = parse_double(fields[1], 0.0);
  msg.course_mag_deg = parse_double(fields[3], 0.0);
  msg.speed_knots = parse_double(fields[5], 0.0);
  msg.speed_kmh = parse_double(fields[7], 0.0);
  msg.mode = (fields.size() > 9) ? parse_char(fields[9], 'N') : 'N';
  msg.valid = (msg.mode != 'N');
  
  if (vtg_callback_) {
    vtg_callback_(msg);
  }
}

void NmeaParser::parse_gst(const std::vector<std::string_view>& fields)
{
  // $xxGST,time,rms,smajor,sminor,orient,lat_err,lon_err,alt_err*cs
  // Index:  0    1    2       3      4      5       6       7
  
  if (fields.size() < 8) {
    return;
  }
  
  NmeaGST msg;
  msg.utc_time = parse_utc_time(fields[1]);
  msg.rms_residual = parse_double(fields[2], 0.0);
  msg.semi_major_m = parse_double(fields[3], 0.0);
  msg.semi_minor_m = parse_double(fields[4], 0.0);
  msg.orientation_deg = parse_double(fields[5], 0.0);
  msg.lat_error_m = parse_double(fields[6], 0.0);
  msg.lon_error_m = parse_double(fields[7], 0.0);
  msg.alt_error_m = (fields.size() > 8) ? parse_double(fields[8], 0.0) : 0.0;
  msg.valid = true;
  
  if (gst_callback_) {
    gst_callback_(msg);
  }
}

void NmeaParser::parse_hdt(const std::vector<std::string_view>& fields)
{
  // $xxHDT,heading,T*cs
  // Index:    0     1
  
  if (fields.size() < 2) {
    return;
  }
  
  NmeaHDT msg;
  msg.heading_deg = parse_double(fields[1], 0.0);
  msg.valid = !fields[1].empty();
  
  if (hdt_callback_) {
    hdt_callback_(msg);
  }
}

void NmeaParser::parse_ths(const std::vector<std::string_view>& fields)
{
  // $xxTHS,heading,status*cs
  // Index:    0      1
  
  if (fields.size() < 3) {
    return;
  }
  
  NmeaTHS msg;
  msg.heading_deg = parse_double(fields[1], 0.0);
  msg.status = parse_char(fields[2], 'V');
  msg.valid = (msg.status == 'A');
  
  if (ths_callback_) {
    ths_callback_(msg);
  }
}

void NmeaParser::parse_zda(const std::vector<std::string_view>& fields)
{
  // $xxZDA,time,day,month,year,ltzh,ltzn*cs
  // Index:  0    1    2     3    4    5
  
  if (fields.size() < 5) {
    return;
  }
  
  NmeaZDA msg;
  msg.utc_time = parse_utc_time(fields[1]);
  msg.day = static_cast<uint8_t>(parse_int(fields[2], 0));
  msg.month = static_cast<uint8_t>(parse_int(fields[3], 0));
  msg.year = static_cast<uint16_t>(parse_int(fields[4], 0));
  msg.local_zone_hours = (fields.size() > 5) ? static_cast<int8_t>(parse_int(fields[5], 0)) : 0;
  msg.local_zone_minutes = (fields.size() > 6) ? static_cast<int8_t>(parse_int(fields[6], 0)) : 0;
  msg.valid = (msg.year > 0);
  
  if (zda_callback_) {
    zda_callback_(msg);
  }
}

}  // namespace gnss_compass

