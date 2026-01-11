// Copyright (c) 2026 Davut Can Akbas
// SPDX-License-Identifier: MIT

#ifndef GNSS_COMPASS_DRIVER__SERIAL__SERIAL_PORT_HPP_
#define GNSS_COMPASS_DRIVER__SERIAL__SERIAL_PORT_HPP_

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace gnss_compass
{

// =============================================================================
// Serial Port Configuration
// =============================================================================

enum class Parity
{
  NONE,
  ODD,
  EVEN
};

enum class StopBits
{
  ONE,
  TWO
};

enum class FlowControl
{
  NONE,
  HARDWARE,
  SOFTWARE
};

struct SerialConfig
{
  std::string port{"/dev/ttyUSB0"};
  uint32_t baudrate{460800};
  uint8_t data_bits{8};
  Parity parity{Parity::NONE};
  StopBits stop_bits{StopBits::ONE};
  FlowControl flow_control{FlowControl::NONE};
  
  // Read settings
  uint32_t read_timeout_ms{100};
  size_t read_buffer_size{4096};
  
  // Reconnection settings
  bool auto_reconnect{true};
  uint32_t reconnect_delay_ms{1000};
  uint32_t max_reconnect_attempts{10};
};

// =============================================================================
// Serial Port Statistics
// =============================================================================

struct SerialStats
{
  uint64_t bytes_received{0};
  uint64_t bytes_sent{0};
  uint64_t read_errors{0};
  uint64_t write_errors{0};
  uint64_t reconnect_count{0};
  std::chrono::steady_clock::time_point last_receive_time;
};

// =============================================================================
// Serial Port Class
// =============================================================================

class SerialPort
{
public:
  using DataCallback = std::function<void(const uint8_t*, size_t)>;
  using ErrorCallback = std::function<void(const std::string&)>;
  using ConnectedCallback = std::function<void(bool)>;

  SerialPort();
  ~SerialPort();

  // Non-copyable, movable
  SerialPort(const SerialPort&) = delete;
  SerialPort& operator=(const SerialPort&) = delete;
  SerialPort(SerialPort&&) noexcept;
  SerialPort& operator=(SerialPort&&) noexcept;

  // ==========================================================================
  // Configuration
  // ==========================================================================
  
  /**
   * @brief Configure the serial port
   * @param config Serial port configuration
   */
  void configure(const SerialConfig& config);
  
  /**
   * @brief Get current configuration
   */
  const SerialConfig& config() const { return config_; }

  // ==========================================================================
  // Connection Management
  // ==========================================================================
  
  /**
   * @brief Open the serial port and start reading
   * @return true if successful
   */
  bool open();
  
  /**
   * @brief Close the serial port and stop reading
   */
  void close();
  
  /**
   * @brief Check if port is open
   */
  bool is_open() const;
  
  /**
   * @brief Check if port is connected and receiving data
   */
  bool is_connected() const;

  // ==========================================================================
  // Data Transfer
  // ==========================================================================
  
  /**
   * @brief Write data to serial port
   * @param data Pointer to data buffer
   * @param size Number of bytes to write
   * @return Number of bytes written, -1 on error
   */
  ssize_t write(const uint8_t* data, size_t size);
  
  /**
   * @brief Write string to serial port
   * @param str String to write
   * @return Number of bytes written, -1 on error
   */
  ssize_t write(const std::string& str);

  // ==========================================================================
  // Callbacks
  // ==========================================================================
  
  /**
   * @brief Set callback for received data
   * @param callback Function called when data is received
   */
  void set_data_callback(DataCallback callback);
  
  /**
   * @brief Set callback for errors
   * @param callback Function called on error
   */
  void set_error_callback(ErrorCallback callback);
  
  /**
   * @brief Set callback for connection state changes
   * @param callback Function called when connection state changes
   */
  void set_connected_callback(ConnectedCallback callback);

  // ==========================================================================
  // Statistics
  // ==========================================================================
  
  /**
   * @brief Get serial port statistics
   */
  SerialStats stats() const;
  
  /**
   * @brief Reset statistics counters
   */
  void reset_stats();

private:
  // Implementation details
  void read_thread_func();
  bool configure_port();
  void handle_reconnect();
  void update_stats_rx(size_t bytes);
  void notify_error(const std::string& error);

  SerialConfig config_;
  int fd_{-1};
  
  std::atomic<bool> running_{false};
  std::atomic<bool> connected_{false};
  std::thread read_thread_;
  
  mutable std::mutex callback_mutex_;
  DataCallback data_callback_;
  ErrorCallback error_callback_;
  ConnectedCallback connected_callback_;
  
  mutable std::mutex stats_mutex_;
  SerialStats stats_;
  
  std::vector<uint8_t> read_buffer_;
};

}  // namespace gnss_compass

#endif  // GNSS_COMPASS_DRIVER__SERIAL__SERIAL_PORT_HPP_

