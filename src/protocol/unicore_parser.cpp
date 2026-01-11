// Copyright (c) 2026 Davut Can Akbas
// SPDX-License-Identifier: MIT

#include "gnss_compass_driver/protocol/unicore_parser.hpp"

#include <charconv>
#include <cstring>

namespace gnss_compass
{

// =============================================================================
// CRC32 Lookup Table (Unicore uses standard CRC-32)
// =============================================================================

const uint32_t UniCoreParser::crc32_table_[256] = {
  0x00000000, 0x77073096, 0xEE0E612C, 0x990951BA, 0x076DC419, 0x706AF48F,
  0xE963A535, 0x9E6495A3, 0x0EDB8832, 0x79DCB8A4, 0xE0D5E91E, 0x97D2D988,
  0x09B64C2B, 0x7EB17CBD, 0xE7B82D07, 0x90BF1D91, 0x1DB71064, 0x6AB020F2,
  0xF3B97148, 0x84BE41DE, 0x1ADAD47D, 0x6DDDE4EB, 0xF4D4B551, 0x83D385C7,
  0x136C9856, 0x646BA8C0, 0xFD62F97A, 0x8A65C9EC, 0x14015C4F, 0x63066CD9,
  0xFA0F3D63, 0x8D080DF5, 0x3B6E20C8, 0x4C69105E, 0xD56041E4, 0xA2677172,
  0x3C03E4D1, 0x4B04D447, 0xD20D85FD, 0xA50AB56B, 0x35B5A8FA, 0x42B2986C,
  0xDBBBC9D6, 0xACBCF940, 0x32D86CE3, 0x45DF5C75, 0xDCD60DCF, 0xABD13D59,
  0x26D930AC, 0x51DE003A, 0xC8D75180, 0xBFD06116, 0x21B4F4B5, 0x56B3C423,
  0xCFBA9599, 0xB8BDA50F, 0x2802B89E, 0x5F058808, 0xC60CD9B2, 0xB10BE924,
  0x2F6F7C87, 0x58684C11, 0xC1611DAB, 0xB6662D3D, 0x76DC4190, 0x01DB7106,
  0x98D220BC, 0xEFD5102A, 0x71B18589, 0x06B6B51F, 0x9FBFE4A5, 0xE8B8D433,
  0x7807C9A2, 0x0F00F934, 0x9609A88E, 0xE10E9818, 0x7F6A0DBB, 0x086D3D2D,
  0x91646C97, 0xE6635C01, 0x6B6B51F4, 0x1C6C6162, 0x856530D8, 0xF262004E,
  0x6C0695ED, 0x1B01A57B, 0x8208F4C1, 0xF50FC457, 0x65B0D9C6, 0x12B7E950,
  0x8BBEB8EA, 0xFCB9887C, 0x62DD1DDF, 0x15DA2D49, 0x8CD37CF3, 0xFBD44C65,
  0x4DB26158, 0x3AB551CE, 0xA3BC0074, 0xD4BB30E2, 0x4ADFA541, 0x3DD895D7,
  0xA4D1C46D, 0xD3D6F4FB, 0x4369E96A, 0x346ED9FC, 0xAD678846, 0xDA60B8D0,
  0x44042D73, 0x33031DE5, 0xAA0A4C5F, 0xDD0D7CC9, 0x5005713C, 0x270241AA,
  0xBE0B1010, 0xC90C2086, 0x5768B525, 0x206F85B3, 0xB966D409, 0xCE61E49F,
  0x5EDEF90E, 0x29D9C998, 0xB0D09822, 0xC7D7A8B4, 0x59B33D17, 0x2EB40D81,
  0xB7BD5C3B, 0xC0BA6CAD, 0xEDB88320, 0x9ABFB3B6, 0x03B6E20C, 0x74B1D29A,
  0xEAD54739, 0x9DD277AF, 0x04DB2615, 0x73DC1683, 0xE3630B12, 0x94643B84,
  0x0D6D6A3E, 0x7A6A5AA8, 0xE40ECF0B, 0x9309FF9D, 0x0A00AE27, 0x7D079EB1,
  0xF00F9344, 0x8708A3D2, 0x1E01F268, 0x6906C2FE, 0xF762575D, 0x806567CB,
  0x196C3671, 0x6E6B06E7, 0xFED41B76, 0x89D32BE0, 0x10DA7A5A, 0x67DD4ACC,
  0xF9B9DF6F, 0x8EBEEFF9, 0x17B7BE43, 0x60B08ED5, 0xD6D6A3E8, 0xA1D1937E,
  0x38D8C2C4, 0x4FDFF252, 0xD1BB67F1, 0xA6BC5767, 0x3FB506DD, 0x48B2364B,
  0xD80D2BDA, 0xAF0A1B4C, 0x36034AF6, 0x41047A60, 0xDF60EFC3, 0xA867DF55,
  0x316E8EEF, 0x4669BE79, 0xCB61B38C, 0xBC66831A, 0x256FD2A0, 0x5268E236,
  0xCC0C7795, 0xBB0B4703, 0x220216B9, 0x5505262F, 0xC5BA3BBE, 0xB2BD0B28,
  0x2BB45A92, 0x5CB36A04, 0xC2D7FFA7, 0xB5D0CF31, 0x2CD99E8B, 0x5BDEAE1D,
  0x9B64C2B0, 0xEC63F226, 0x756AA39C, 0x026D930A, 0x9C0906A9, 0xEB0E363F,
  0x72076785, 0x05005713, 0x95BF4A82, 0xE2B87A14, 0x7BB12BAE, 0x0CB61B38,
  0x92D28E9B, 0xE5D5BE0D, 0x7CDCEFB7, 0x0BDBDF21, 0x86D3D2D4, 0xF1D4E242,
  0x68DDB3F8, 0x1FDA836E, 0x81BE16CD, 0xF6B9265B, 0x6FB077E1, 0x18B74777,
  0x88085AE6, 0xFF0F6A70, 0x66063BCA, 0x11010B5C, 0x8F659EFF, 0xF862AE69,
  0x616BFFD3, 0x166CCF45, 0xA00AE278, 0xD70DD2EE, 0x4E048354, 0x3903B3C2,
  0xA7672661, 0xD06016F7, 0x4969474D, 0x3E6E77DB, 0xAED16A4A, 0xD9D65ADC,
  0x40DF0B66, 0x37D83BF0, 0xA9BCAE53, 0xDEBB9EC5, 0x47B2CF7F, 0x30B5FFE9,
  0xBDBDF21C, 0xCABAC28A, 0x53B39330, 0x24B4A3A6, 0xBAD03605, 0xCDD706B3,
  0x54DE5729, 0x23D967BF, 0xB3667A2E, 0xC4614AB8, 0x5D681B02, 0x2A6F2B94,
  0xB40BBE37, 0xC30C8EA1, 0x5A05DF1B, 0x2D02EF8D
};

// =============================================================================
// Construction
// =============================================================================

UniCoreParser::UniCoreParser()
{
  buffer_.reserve(MAX_BUFFER_SIZE);
}

// =============================================================================
// Data Processing
// =============================================================================

void UniCoreParser::process(const uint8_t* data, size_t size)
{
  for (size_t i = 0; i < size; ++i) {
    char c = static_cast<char>(data[i]);
    
    if (c == '#') {
      // Start of new Unicore message
      buffer_.clear();
      buffer_ += c;
    } else if (c == '\r' || c == '\n') {
      // End of message
      if (!buffer_.empty() && buffer_[0] == '#') {
        parse_message(buffer_);
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

void UniCoreParser::reset()
{
  buffer_.clear();
}

// =============================================================================
// Message Parsing
// =============================================================================

bool UniCoreParser::parse_message(const std::string& message)
{
  // Format: #MSGNAME,header;data*checksum
  // Find the asterisk (checksum delimiter)
  size_t asterisk_pos = message.find('*');
  if (asterisk_pos == std::string::npos || asterisk_pos + 8 > message.size()) {
    stats_.parse_errors++;
    return false;
  }
  
  // Validate checksum (skip for now, some messages have different format)
  // TODO: Implement proper CRC32 validation
  
  // Find the semicolon (header/data delimiter)
  size_t semicolon_pos = message.find(';');
  if (semicolon_pos == std::string::npos) {
    stats_.parse_errors++;
    return false;
  }
  
  // Extract header (between # and ;)
  std::string_view header_part(message.data() + 1, semicolon_pos - 1);
  
  // Extract data (between ; and *)
  std::string_view data_part(message.data() + semicolon_pos + 1, 
                              asterisk_pos - semicolon_pos - 1);
  
  // Split header into fields
  auto header_fields = split_fields(header_part, ',');
  if (header_fields.empty()) {
    stats_.parse_errors++;
    return false;
  }
  
  // Get message name (first header field)
  std::string_view msg_name = header_fields[0];
  
  // Split data into fields
  auto data_fields = split_fields(data_part, ',');
  
  try {
    if (msg_name == "UNIHEADINGA") {
      parse_heading(header_fields, data_fields);
      stats_.messages_parsed++;
      return true;
    } else if (msg_name == "BESTNAVA") {
      parse_bestnav(header_fields, data_fields);
      stats_.messages_parsed++;
      return true;
    }
    // Unknown message type - ignore
    return false;
    
  } catch (const std::exception& e) {
    stats_.parse_errors++;
    if (error_callback_) {
      error_callback_("Unicore parse error: " + std::string(e.what()));
    }
    return false;
  }
}

std::vector<std::string_view> UniCoreParser::split_fields(std::string_view data, char delimiter)
{
  std::vector<std::string_view> fields;
  fields.reserve(30);
  
  size_t start = 0;
  size_t pos = 0;
  
  while (pos <= data.size()) {
    if (pos == data.size() || data[pos] == delimiter) {
      fields.push_back(data.substr(start, pos - start));
      start = pos + 1;
    }
    ++pos;
  }
  
  return fields;
}

// =============================================================================
// Field Parsing Utilities
// =============================================================================

double UniCoreParser::parse_double(std::string_view field, double default_value)
{
  if (field.empty()) {
    return default_value;
  }
  
  double value = default_value;
  std::from_chars(field.data(), field.data() + field.size(), value);
  return value;
}

int UniCoreParser::parse_int(std::string_view field, int default_value)
{
  if (field.empty()) {
    return default_value;
  }
  
  int value = default_value;
  std::from_chars(field.data(), field.data() + field.size(), value);
  return value;
}

UniCoreSolutionStatus UniCoreParser::parse_sol_status(std::string_view field)
{
  if (field == "SOL_COMPUTED") return UniCoreSolutionStatus::SOL_COMPUTED;
  if (field == "INSUFFICIENT_OBS") return UniCoreSolutionStatus::INSUFFICIENT_OBS;
  if (field == "NO_CONVERGENCE") return UniCoreSolutionStatus::NO_CONVERGENCE;
  if (field == "COLD_START") return UniCoreSolutionStatus::COLD_START;
  if (field == "PENDING") return UniCoreSolutionStatus::PENDING;
  return UniCoreSolutionStatus::INVALID_FIX;
}

UniCorePosType UniCoreParser::parse_pos_type(std::string_view field)
{
  if (field == "NONE") return UniCorePosType::NONE;
  if (field == "SINGLE") return UniCorePosType::SINGLE;
  if (field == "PSRDIFF") return UniCorePosType::PSRDIFF;
  if (field == "WAAS") return UniCorePosType::WAAS;
  if (field == "L1_FLOAT") return UniCorePosType::L1_FLOAT;
  if (field == "IONOFREE_FLOAT") return UniCorePosType::IONOFREE_FLOAT;
  if (field == "NARROW_FLOAT") return UniCorePosType::NARROW_FLOAT;
  if (field == "L1_INT") return UniCorePosType::L1_INT;
  if (field == "WIDE_INT") return UniCorePosType::WIDE_INT;
  if (field == "NARROW_INT") return UniCorePosType::NARROW_INT;
  if (field == "DOPPLER_VELOCITY") return UniCorePosType::DOPPLER_VELOCITY;
  if (field == "PPP") return UniCorePosType::PPP;
  return UniCorePosType::NONE;
}

// =============================================================================
// Message Parsers
// =============================================================================

void UniCoreParser::parse_heading(const std::vector<std::string_view>& /*header_fields*/,
                                   const std::vector<std::string_view>& data_fields)
{
  // UNIHEADINGA data format:
  // 0: sol_status, 1: pos_type, 2: baseline, 3: heading, 4: pitch, 5: reserved,
  // 6: hdg_std, 7: pitch_std, 8: rover_id, 9: num_sv, 10: num_soln, 11: num_obs, ...
  
  if (data_fields.size() < 12) {
    return;
  }
  
  UniCoreHeading msg;
  msg.sol_status = parse_sol_status(data_fields[0]);
  msg.pos_type = parse_pos_type(data_fields[1]);
  msg.baseline_length_m = parse_double(data_fields[2], 0.0);
  msg.heading_deg = parse_double(data_fields[3], 0.0);
  msg.pitch_deg = parse_double(data_fields[4], 0.0);
  msg.heading_std_deg = parse_double(data_fields[6], 0.0);
  msg.pitch_std_deg = parse_double(data_fields[7], 0.0);
  msg.num_satellites = static_cast<uint8_t>(parse_int(data_fields[9], 0));
  msg.num_soln_satellites = static_cast<uint8_t>(parse_int(data_fields[10], 0));
  msg.num_observations = static_cast<uint8_t>(parse_int(data_fields[11], 0));
  
  msg.valid = (msg.sol_status == UniCoreSolutionStatus::SOL_COMPUTED);
  msg.timestamp = Clock::now();
  
  if (heading_callback_) {
    heading_callback_(msg);
  }
}

void UniCoreParser::parse_bestnav(const std::vector<std::string_view>& /*header_fields*/,
                                   const std::vector<std::string_view>& data_fields)
{
  // BESTNAVA data format:
  // Position: 0: sol_stat, 1: pos_type, 2: lat, 3: lon, 4: hgt, 5: undulation,
  //           6: datum, 7: lat_std, 8: lon_std, 9: hgt_std, 10: stn_id,
  //           11: diff_age, 12: sol_age, 13: num_sv, 14: num_soln, ...
  // Velocity: 20: vel_sol_stat, 21: vel_type, 22: latency, 23: age,
  //           24: hor_spd, 25: trk_gnd, 26: vert_spd, ...
  
  if (data_fields.size() < 28) {
    return;
  }
  
  UniCoreBestNav msg;
  
  // Position
  msg.pos_sol_status = parse_sol_status(data_fields[0]);
  msg.pos_type = parse_pos_type(data_fields[1]);
  msg.latitude_deg = parse_double(data_fields[2], 0.0);
  msg.longitude_deg = parse_double(data_fields[3], 0.0);
  msg.height_m = parse_double(data_fields[4], 0.0);
  msg.undulation_m = parse_double(data_fields[5], 0.0);
  msg.lat_std_m = parse_double(data_fields[7], 0.0);
  msg.lon_std_m = parse_double(data_fields[8], 0.0);
  msg.hgt_std_m = parse_double(data_fields[9], 0.0);
  msg.diff_age_s = static_cast<float>(parse_double(data_fields[11], 0.0));
  msg.sol_age_s = static_cast<float>(parse_double(data_fields[12], 0.0));
  msg.num_satellites = static_cast<uint8_t>(parse_int(data_fields[13], 0));
  msg.num_soln_satellites = static_cast<uint8_t>(parse_int(data_fields[14], 0));
  
  // Velocity (indices start at 21 after position fields)
  msg.vel_sol_status = parse_sol_status(data_fields[21]);
  msg.vel_type = parse_pos_type(data_fields[22]);
  msg.horizontal_speed_mps = parse_double(data_fields[25], 0.0);
  msg.track_ground_deg = parse_double(data_fields[26], 0.0);
  msg.vertical_speed_mps = parse_double(data_fields[27], 0.0);
  
  msg.valid = (msg.pos_sol_status == UniCoreSolutionStatus::SOL_COMPUTED);
  msg.timestamp = Clock::now();
  
  if (bestnav_callback_) {
    bestnav_callback_(msg);
  }
}

// =============================================================================
// CRC32 Calculation
// =============================================================================

uint32_t UniCoreParser::calculate_crc32(const uint8_t* data, size_t len)
{
  uint32_t crc = 0;
  for (size_t i = 0; i < len; ++i) {
    crc = crc32_table_[(crc ^ data[i]) & 0xFF] ^ (crc >> 8);
  }
  return crc;
}

bool UniCoreParser::validate_checksum(const std::string& message)
{
  // Find asterisk
  size_t asterisk_pos = message.find('*');
  if (asterisk_pos == std::string::npos || asterisk_pos + 8 > message.size()) {
    return false;
  }
  
  // Calculate CRC32 of data (excluding # and *)
  uint32_t calculated = calculate_crc32(
    reinterpret_cast<const uint8_t*>(message.data() + 1),
    asterisk_pos - 1);
  
  // Parse provided checksum (8 hex digits after *)
  std::string_view checksum_str(message.data() + asterisk_pos + 1, 8);
  uint32_t provided = 0;
  auto result = std::from_chars(checksum_str.data(), checksum_str.data() + 8, provided, 16);
  
  if (result.ec != std::errc()) {
    return false;
  }
  
  return calculated == provided;
}

}  // namespace gnss_compass

