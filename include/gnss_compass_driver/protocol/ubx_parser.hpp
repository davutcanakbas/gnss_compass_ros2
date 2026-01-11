// Copyright (c) 2026 Davut Can Akbas
// SPDX-License-Identifier: MIT

#ifndef GNSS_COMPASS_DRIVER__PROTOCOL__UBX_PARSER_HPP_
#define GNSS_COMPASS_DRIVER__PROTOCOL__UBX_PARSER_HPP_

#include "gnss_compass_driver/protocol/ubx_messages.hpp"
#include "gnss_compass_driver/types.hpp"

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace gnss_compass
{

// =============================================================================
// UBX Parser Class
// =============================================================================

class UbxParser
{
public:
  // Callbacks for parsed messages
  using NavPvtCallback = std::function<void(const ubx::NavPvt&)>;
  using NavRelPosNedCallback = std::function<void(const ubx::NavRelPosNed&)>;
  using NavStatusCallback = std::function<void(const ubx::NavStatus&)>;
  using NavDopCallback = std::function<void(const ubx::NavDop&)>;
  using AckCallback = std::function<void(uint8_t cls, uint8_t id, bool ack)>;
  using ErrorCallback = std::function<void(const std::string&)>;

  UbxParser();
  ~UbxParser() = default;

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
  // Message Generation
  // ==========================================================================
  
  /**
   * @brief Create a UBX message buffer
   * @param cls Message class
   * @param id Message ID
   * @param payload Payload data
   * @param payload_len Payload length
   * @return Complete UBX message with header and checksum
   */
  static std::vector<uint8_t> create_message(
    uint8_t cls, uint8_t id,
    const uint8_t* payload = nullptr, size_t payload_len = 0);
  
  /**
   * @brief Create CFG-MSG message to enable/disable a message
   * @param msg_class Message class to configure
   * @param msg_id Message ID to configure
   * @param rate Output rate (0 = disabled)
   * @return CFG-MSG message
   */
  static std::vector<uint8_t> create_cfg_msg(
    uint8_t msg_class, uint8_t msg_id, uint8_t rate);
  
  /**
   * @brief Create CFG-RATE message to set navigation rate
   * @param meas_rate_ms Measurement rate in milliseconds
   * @param nav_rate Navigation rate (measurements per solution)
   * @return CFG-RATE message
   */
  static std::vector<uint8_t> create_cfg_rate(
    uint16_t meas_rate_ms, uint16_t nav_rate = 1);

  // ==========================================================================
  // Callbacks
  // ==========================================================================
  
  void set_nav_pvt_callback(NavPvtCallback cb) { nav_pvt_callback_ = std::move(cb); }
  void set_nav_relposned_callback(NavRelPosNedCallback cb) { nav_relposned_callback_ = std::move(cb); }
  void set_nav_status_callback(NavStatusCallback cb) { nav_status_callback_ = std::move(cb); }
  void set_nav_dop_callback(NavDopCallback cb) { nav_dop_callback_ = std::move(cb); }
  void set_ack_callback(AckCallback cb) { ack_callback_ = std::move(cb); }
  void set_error_callback(ErrorCallback cb) { error_callback_ = std::move(cb); }

  // ==========================================================================
  // Statistics
  // ==========================================================================
  
  struct Stats
  {
    uint64_t messages_parsed{0};
    uint64_t checksum_errors{0};
    uint64_t parse_errors{0};
    uint64_t unknown_messages{0};
  };
  
  Stats stats() const { return stats_; }
  void reset_stats() { stats_ = Stats{}; }

private:
  // Parser state machine
  enum class State
  {
    SYNC1,        // Looking for first sync byte (0xB5)
    SYNC2,        // Looking for second sync byte (0x62)
    CLASS,        // Reading message class
    ID,           // Reading message ID
    LENGTH1,      // Reading length low byte
    LENGTH2,      // Reading length high byte
    PAYLOAD,      // Reading payload
    CHECKSUM1,    // Reading checksum A
    CHECKSUM2     // Reading checksum B
  };
  
  void process_byte(uint8_t byte);
  void process_message();
  void update_checksum(uint8_t byte);
  static void calculate_checksum(const uint8_t* data, size_t len, uint8_t& ck_a, uint8_t& ck_b);
  
  // Parser state
  State state_{State::SYNC1};
  uint8_t msg_class_{0};
  uint8_t msg_id_{0};
  uint16_t payload_length_{0};
  uint16_t payload_index_{0};
  uint8_t ck_a_{0};
  uint8_t ck_b_{0};
  uint8_t received_ck_a_{0};
  
  std::vector<uint8_t> payload_buffer_;
  
  // Callbacks
  NavPvtCallback nav_pvt_callback_;
  NavRelPosNedCallback nav_relposned_callback_;
  NavStatusCallback nav_status_callback_;
  NavDopCallback nav_dop_callback_;
  AckCallback ack_callback_;
  ErrorCallback error_callback_;
  
  // Statistics
  Stats stats_;
};

}  // namespace gnss_compass

#endif  // GNSS_COMPASS_DRIVER__PROTOCOL__UBX_PARSER_HPP_

