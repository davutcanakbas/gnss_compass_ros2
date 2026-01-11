// Copyright (c) 2026 Davut Can Akbas
// SPDX-License-Identifier: MIT

#include "gnss_compass_driver/serial/serial_port.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <cerrno>
#include <cstring>

namespace gnss_compass
{

// =============================================================================
// Helper Functions
// =============================================================================

namespace
{

speed_t baudrate_to_speed(uint32_t baudrate)
{
  switch (baudrate) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 500000: return B500000;
    case 576000: return B576000;
    case 921600: return B921600;
    case 1000000: return B1000000;
    case 1152000: return B1152000;
    case 1500000: return B1500000;
    case 2000000: return B2000000;
    default: return B115200;
  }
}

}  // namespace

// =============================================================================
// Construction / Destruction
// =============================================================================

SerialPort::SerialPort()
  : read_buffer_(4096)
{
}

SerialPort::~SerialPort()
{
  close();
}

SerialPort::SerialPort(SerialPort&& other) noexcept
  : config_(std::move(other.config_)),
    fd_(other.fd_),
    running_(other.running_.load()),
    connected_(other.connected_.load()),
    read_buffer_(std::move(other.read_buffer_))
{
  other.fd_ = -1;
  other.running_ = false;
  other.connected_ = false;
  
  // Note: Thread and callbacks need special handling
  // For now, don't allow move while running
}

SerialPort& SerialPort::operator=(SerialPort&& other) noexcept
{
  if (this != &other) {
    close();
    
    config_ = std::move(other.config_);
    fd_ = other.fd_;
    running_ = other.running_.load();
    connected_ = other.connected_.load();
    read_buffer_ = std::move(other.read_buffer_);
    
    other.fd_ = -1;
    other.running_ = false;
    other.connected_ = false;
  }
  return *this;
}

// =============================================================================
// Configuration
// =============================================================================

void SerialPort::configure(const SerialConfig& config)
{
  config_ = config;
  read_buffer_.resize(config_.read_buffer_size);
}

// =============================================================================
// Connection Management
// =============================================================================

bool SerialPort::open()
{
  if (is_open()) {
    return true;
  }
  
  // Open the serial port
  fd_ = ::open(config_.port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    notify_error("Failed to open " + config_.port + ": " + std::strerror(errno));
    return false;
  }
  
  // Configure the port
  if (!configure_port()) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }
  
  // Clear any pending data
  tcflush(fd_, TCIOFLUSH);
  
  // Start read thread
  running_ = true;
  connected_ = true;
  read_thread_ = std::thread(&SerialPort::read_thread_func, this);
  
  // Notify connection
  {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    if (connected_callback_) {
      connected_callback_(true);
    }
  }
  
  return true;
}

void SerialPort::close()
{
  running_ = false;
  
  if (read_thread_.joinable()) {
    read_thread_.join();
  }
  
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  
  if (connected_.exchange(false)) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    if (connected_callback_) {
      connected_callback_(false);
    }
  }
}

bool SerialPort::is_open() const
{
  return fd_ >= 0;
}

bool SerialPort::is_connected() const
{
  return connected_.load();
}

// =============================================================================
// Port Configuration
// =============================================================================

bool SerialPort::configure_port()
{
  struct termios tty;
  
  if (tcgetattr(fd_, &tty) != 0) {
    notify_error("Failed to get terminal attributes: " + std::string(std::strerror(errno)));
    return false;
  }
  
  // Set baudrate
  speed_t speed = baudrate_to_speed(config_.baudrate);
  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);
  
  // Control modes
  tty.c_cflag &= ~CSIZE;
  switch (config_.data_bits) {
    case 5: tty.c_cflag |= CS5; break;
    case 6: tty.c_cflag |= CS6; break;
    case 7: tty.c_cflag |= CS7; break;
    case 8: default: tty.c_cflag |= CS8; break;
  }
  
  // Parity
  switch (config_.parity) {
    case Parity::NONE:
      tty.c_cflag &= ~PARENB;
      break;
    case Parity::ODD:
      tty.c_cflag |= PARENB;
      tty.c_cflag |= PARODD;
      break;
    case Parity::EVEN:
      tty.c_cflag |= PARENB;
      tty.c_cflag &= ~PARODD;
      break;
  }
  
  // Stop bits
  if (config_.stop_bits == StopBits::TWO) {
    tty.c_cflag |= CSTOPB;
  } else {
    tty.c_cflag &= ~CSTOPB;
  }
  
  // Flow control
  switch (config_.flow_control) {
    case FlowControl::NONE:
      tty.c_cflag &= ~CRTSCTS;
      tty.c_iflag &= ~(IXON | IXOFF | IXANY);
      break;
    case FlowControl::HARDWARE:
      tty.c_cflag |= CRTSCTS;
      tty.c_iflag &= ~(IXON | IXOFF | IXANY);
      break;
    case FlowControl::SOFTWARE:
      tty.c_cflag &= ~CRTSCTS;
      tty.c_iflag |= (IXON | IXOFF);
      break;
  }
  
  // Enable receiver, ignore modem control lines
  tty.c_cflag |= (CLOCAL | CREAD);
  
  // Input modes - disable special handling
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
  
  // Output modes - disable special handling
  tty.c_oflag &= ~OPOST;
  tty.c_oflag &= ~ONLCR;
  
  // Local modes - raw input
  tty.c_lflag &= ~(ECHO | ECHOE | ECHONL | ICANON | ISIG | IEXTEN);
  
  // Read settings
  tty.c_cc[VMIN] = 0;   // Non-blocking read
  tty.c_cc[VTIME] = 1;  // 100ms timeout
  
  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    notify_error("Failed to set terminal attributes: " + std::string(std::strerror(errno)));
    return false;
  }
  
  return true;
}

// =============================================================================
// Data Transfer
// =============================================================================

ssize_t SerialPort::write(const uint8_t* data, size_t size)
{
  if (!is_open()) {
    return -1;
  }
  
  ssize_t written = ::write(fd_, data, size);
  
  if (written < 0) {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    stats_.write_errors++;
    notify_error("Write error: " + std::string(std::strerror(errno)));
  } else {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    stats_.bytes_sent += static_cast<uint64_t>(written);
  }
  
  return written;
}

ssize_t SerialPort::write(const std::string& str)
{
  return write(reinterpret_cast<const uint8_t*>(str.data()), str.size());
}

// =============================================================================
// Read Thread
// =============================================================================

void SerialPort::read_thread_func()
{
  uint32_t reconnect_attempts = 0;
  
  while (running_) {
    if (fd_ < 0) {
      // Try to reconnect
      if (config_.auto_reconnect && reconnect_attempts < config_.max_reconnect_attempts) {
        std::this_thread::sleep_for(std::chrono::milliseconds(config_.reconnect_delay_ms));
        
        fd_ = ::open(config_.port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ >= 0 && configure_port()) {
          tcflush(fd_, TCIOFLUSH);
          connected_ = true;
          reconnect_attempts = 0;
          
          std::lock_guard<std::mutex> lock(stats_mutex_);
          stats_.reconnect_count++;
          
          {
            std::lock_guard<std::mutex> cb_lock(callback_mutex_);
            if (connected_callback_) {
              connected_callback_(true);
            }
          }
        } else {
          if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
          }
          reconnect_attempts++;
        }
      }
      continue;
    }
    
    // Use select for timeout-based reading
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(fd_, &read_fds);
    
    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = config_.read_timeout_ms * 1000;
    
    int ret = select(fd_ + 1, &read_fds, nullptr, nullptr, &timeout);
    
    if (ret < 0) {
      if (errno == EINTR) {
        continue;
      }
      notify_error("Select error: " + std::string(std::strerror(errno)));
      handle_reconnect();
      continue;
    }
    
    if (ret == 0) {
      // Timeout - no data available
      continue;
    }
    
    // Data available - read it
    ssize_t bytes_read = ::read(fd_, read_buffer_.data(), read_buffer_.size());
    
    if (bytes_read < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        continue;
      }
      notify_error("Read error: " + std::string(std::strerror(errno)));
      
      std::lock_guard<std::mutex> lock(stats_mutex_);
      stats_.read_errors++;
      
      handle_reconnect();
      continue;
    }
    
    if (bytes_read == 0) {
      // Connection closed or device removed
      handle_reconnect();
      continue;
    }
    
    // Update statistics
    update_stats_rx(static_cast<size_t>(bytes_read));
    
    // Call data callback
    {
      std::lock_guard<std::mutex> lock(callback_mutex_);
      if (data_callback_) {
        data_callback_(read_buffer_.data(), static_cast<size_t>(bytes_read));
      }
    }
  }
}

void SerialPort::handle_reconnect()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  
  if (connected_.exchange(false)) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    if (connected_callback_) {
      connected_callback_(false);
    }
  }
}

// =============================================================================
// Callbacks
// =============================================================================

void SerialPort::set_data_callback(DataCallback callback)
{
  std::lock_guard<std::mutex> lock(callback_mutex_);
  data_callback_ = std::move(callback);
}

void SerialPort::set_error_callback(ErrorCallback callback)
{
  std::lock_guard<std::mutex> lock(callback_mutex_);
  error_callback_ = std::move(callback);
}

void SerialPort::set_connected_callback(ConnectedCallback callback)
{
  std::lock_guard<std::mutex> lock(callback_mutex_);
  connected_callback_ = std::move(callback);
}

// =============================================================================
// Statistics
// =============================================================================

SerialStats SerialPort::stats() const
{
  std::lock_guard<std::mutex> lock(stats_mutex_);
  return stats_;
}

void SerialPort::reset_stats()
{
  std::lock_guard<std::mutex> lock(stats_mutex_);
  stats_ = SerialStats{};
}

void SerialPort::update_stats_rx(size_t bytes)
{
  std::lock_guard<std::mutex> lock(stats_mutex_);
  stats_.bytes_received += bytes;
  stats_.last_receive_time = std::chrono::steady_clock::now();
}

void SerialPort::notify_error(const std::string& error)
{
  std::lock_guard<std::mutex> lock(callback_mutex_);
  if (error_callback_) {
    error_callback_(error);
  }
}

}  // namespace gnss_compass

