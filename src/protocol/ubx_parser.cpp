// Copyright (c) 2026 Davut Can Akbas
// SPDX-License-Identifier: MIT

#include "gnss_compass_driver/protocol/ubx_parser.hpp"

#include <cstring>

namespace gnss_compass
{

// =============================================================================
// Construction
// =============================================================================

UbxParser::UbxParser()
{
  payload_buffer_.reserve(ubx::MAX_PAYLOAD_SIZE);
}

// =============================================================================
// Data Processing
// =============================================================================

void UbxParser::process(const uint8_t* data, size_t size)
{
  for (size_t i = 0; i < size; ++i) {
    process_byte(data[i]);
  }
}

void UbxParser::process_byte(uint8_t byte)
{
  switch (state_) {
    case State::SYNC1:
      if (byte == ubx::SYNC_CHAR_1) {
        state_ = State::SYNC2;
      }
      break;
      
    case State::SYNC2:
      if (byte == ubx::SYNC_CHAR_2) {
        state_ = State::CLASS;
        ck_a_ = 0;
        ck_b_ = 0;
      } else if (byte == ubx::SYNC_CHAR_1) {
        // Stay in SYNC2 state - might be repeated sync byte
        state_ = State::SYNC2;
      } else {
        state_ = State::SYNC1;
      }
      break;
      
    case State::CLASS:
      msg_class_ = byte;
      update_checksum(byte);
      state_ = State::ID;
      break;
      
    case State::ID:
      msg_id_ = byte;
      update_checksum(byte);
      state_ = State::LENGTH1;
      break;
      
    case State::LENGTH1:
      payload_length_ = byte;
      update_checksum(byte);
      state_ = State::LENGTH2;
      break;
      
    case State::LENGTH2:
      payload_length_ |= static_cast<uint16_t>(byte) << 8;
      update_checksum(byte);
      
      if (payload_length_ > ubx::MAX_PAYLOAD_SIZE) {
        // Invalid length
        state_ = State::SYNC1;
        stats_.parse_errors++;
        if (error_callback_) {
          error_callback_("UBX payload too large: " + std::to_string(payload_length_));
        }
      } else if (payload_length_ == 0) {
        state_ = State::CHECKSUM1;
      } else {
        payload_buffer_.resize(payload_length_);
        payload_index_ = 0;
        state_ = State::PAYLOAD;
      }
      break;
      
    case State::PAYLOAD:
      payload_buffer_[payload_index_++] = byte;
      update_checksum(byte);
      
      if (payload_index_ >= payload_length_) {
        state_ = State::CHECKSUM1;
      }
      break;
      
    case State::CHECKSUM1:
      received_ck_a_ = byte;
      state_ = State::CHECKSUM2;
      break;
      
    case State::CHECKSUM2:
      if (ck_a_ == received_ck_a_ && ck_b_ == byte) {
        process_message();
        stats_.messages_parsed++;
      } else {
        stats_.checksum_errors++;
        if (error_callback_) {
          error_callback_("UBX checksum error");
        }
      }
      state_ = State::SYNC1;
      break;
  }
}

void UbxParser::reset()
{
  state_ = State::SYNC1;
  payload_buffer_.clear();
  payload_index_ = 0;
  ck_a_ = 0;
  ck_b_ = 0;
}

void UbxParser::update_checksum(uint8_t byte)
{
  ck_a_ += byte;
  ck_b_ += ck_a_;
}

void UbxParser::calculate_checksum(const uint8_t* data, size_t len, uint8_t& ck_a, uint8_t& ck_b)
{
  ck_a = 0;
  ck_b = 0;
  for (size_t i = 0; i < len; ++i) {
    ck_a += data[i];
    ck_b += ck_a;
  }
}

// =============================================================================
// Message Processing
// =============================================================================

void UbxParser::process_message()
{
  auto msg_class = static_cast<ubx::MessageClass>(msg_class_);
  
  switch (msg_class) {
    case ubx::MessageClass::NAV: {
      auto msg_id = static_cast<ubx::NavMessageId>(msg_id_);
      
      switch (msg_id) {
        case ubx::NavMessageId::PVT:
          if (payload_length_ >= ubx::NAV_PVT_SIZE && nav_pvt_callback_) {
            ubx::NavPvt msg;
            std::memcpy(&msg, payload_buffer_.data(), sizeof(msg));
            nav_pvt_callback_(msg);
          }
          break;
          
        case ubx::NavMessageId::RELPOSNED:
          if (payload_length_ >= ubx::NAV_RELPOSNED_SIZE && nav_relposned_callback_) {
            ubx::NavRelPosNed msg;
            std::memcpy(&msg, payload_buffer_.data(), sizeof(msg));
            nav_relposned_callback_(msg);
          }
          break;
          
        case ubx::NavMessageId::STATUS:
          if (payload_length_ >= ubx::NAV_STATUS_SIZE && nav_status_callback_) {
            ubx::NavStatus msg;
            std::memcpy(&msg, payload_buffer_.data(), sizeof(msg));
            nav_status_callback_(msg);
          }
          break;
          
        case ubx::NavMessageId::DOP:
          if (payload_length_ >= ubx::NAV_DOP_SIZE && nav_dop_callback_) {
            ubx::NavDop msg;
            std::memcpy(&msg, payload_buffer_.data(), sizeof(msg));
            nav_dop_callback_(msg);
          }
          break;
          
        default:
          stats_.unknown_messages++;
          break;
      }
      break;
    }
    
    case ubx::MessageClass::ACK: {
      if (payload_length_ >= ubx::ACK_ACK_SIZE && ack_callback_) {
        ubx::AckAck msg;
        std::memcpy(&msg, payload_buffer_.data(), sizeof(msg));
        bool is_ack = (msg_id_ == 0x01);  // 0x01 = ACK-ACK, 0x00 = ACK-NAK
        ack_callback_(msg.clsID, msg.msgID, is_ack);
      }
      break;
    }
    
    default:
      stats_.unknown_messages++;
      break;
  }
}

// =============================================================================
// Message Generation
// =============================================================================

std::vector<uint8_t> UbxParser::create_message(
  uint8_t cls, uint8_t id,
  const uint8_t* payload, size_t payload_len)
{
  std::vector<uint8_t> msg;
  msg.reserve(ubx::HEADER_SIZE + payload_len + ubx::CHECKSUM_SIZE);
  
  // Sync bytes
  msg.push_back(ubx::SYNC_CHAR_1);
  msg.push_back(ubx::SYNC_CHAR_2);
  
  // Class and ID
  msg.push_back(cls);
  msg.push_back(id);
  
  // Length (little endian)
  msg.push_back(static_cast<uint8_t>(payload_len & 0xFF));
  msg.push_back(static_cast<uint8_t>((payload_len >> 8) & 0xFF));
  
  // Payload
  if (payload && payload_len > 0) {
    msg.insert(msg.end(), payload, payload + payload_len);
  }
  
  // Calculate checksum (over class, id, length, and payload)
  uint8_t ck_a, ck_b;
  calculate_checksum(msg.data() + 2, msg.size() - 2, ck_a, ck_b);
  
  msg.push_back(ck_a);
  msg.push_back(ck_b);
  
  return msg;
}

std::vector<uint8_t> UbxParser::create_cfg_msg(
  uint8_t msg_class, uint8_t msg_id, uint8_t rate)
{
  // CFG-MSG payload: msgClass, msgID, rate (for current port)
  uint8_t payload[3] = {msg_class, msg_id, rate};
  return create_message(
    static_cast<uint8_t>(ubx::MessageClass::CFG),
    static_cast<uint8_t>(ubx::CfgMessageId::MSG),
    payload, sizeof(payload));
}

std::vector<uint8_t> UbxParser::create_cfg_rate(
  uint16_t meas_rate_ms, uint16_t nav_rate)
{
  // CFG-RATE payload: measRate (2), navRate (2), timeRef (2)
  uint8_t payload[6];
  payload[0] = static_cast<uint8_t>(meas_rate_ms & 0xFF);
  payload[1] = static_cast<uint8_t>((meas_rate_ms >> 8) & 0xFF);
  payload[2] = static_cast<uint8_t>(nav_rate & 0xFF);
  payload[3] = static_cast<uint8_t>((nav_rate >> 8) & 0xFF);
  payload[4] = 0x01;  // GPS time reference
  payload[5] = 0x00;
  
  return create_message(
    static_cast<uint8_t>(ubx::MessageClass::CFG),
    static_cast<uint8_t>(ubx::CfgMessageId::RATE),
    payload, sizeof(payload));
}

}  // namespace gnss_compass

