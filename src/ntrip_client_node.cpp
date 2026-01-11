/**
 * @file ntrip_client_node.cpp
 * @brief Simple NTRIP client for RTK corrections
 * @author Davut Can Akbas <akbasdavutcan@gmail.com>
 * @copyright Copyright (c) 2024-2026 Davut Can Akbas
 * @license Apache-2.0
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <cstring>
#include <thread>
#include <atomic>
#include <vector>
#include <sstream>
#include <iomanip>

namespace gnss_compass {

// Base64 encoding table
static const char base64_chars[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

std::string base64_encode(const std::string& input) {
  std::string ret;
  int i = 0;
  unsigned char char_array_3[3];
  unsigned char char_array_4[4];
  size_t in_len = input.size();
  const char* bytes = input.c_str();

  while (in_len--) {
    char_array_3[i++] = *(bytes++);
    if (i == 3) {
      char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
      char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
      char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
      char_array_4[3] = char_array_3[2] & 0x3f;

      for (i = 0; i < 4; i++)
        ret += base64_chars[char_array_4[i]];
      i = 0;
    }
  }

  if (i) {
    for (int j = i; j < 3; j++)
      char_array_3[j] = '\0';

    char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
    char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
    char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);

    for (int j = 0; j < i + 1; j++)
      ret += base64_chars[char_array_4[j]];

    while (i++ < 3)
      ret += '=';
  }

  return ret;
}

class NtripClientNode : public rclcpp::Node
{
public:
  NtripClientNode()
  : Node("ntrip_client")
  {
    // Declare parameters
    declare_parameter("host", "rtk2go.com");
    declare_parameter("port", 2101);
    declare_parameter("mountpoint", "");
    declare_parameter("username", "");  // Your email (required for RTK2GO)
    declare_parameter("password", "");
    declare_parameter("rtcm_topic", "rtcm");
    declare_parameter("reconnect_delay_sec", 5.0);
    
    // Send GGA position to NTRIP caster for VRS
    declare_parameter("send_gga", true);
    declare_parameter("gga_update_rate_sec", 10.0);
    declare_parameter("latitude", 0.0);
    declare_parameter("longitude", 0.0);
    declare_parameter("altitude", 0.0);
    
    // Load parameters
    host_ = get_parameter("host").as_string();
    port_ = get_parameter("port").as_int();
    mountpoint_ = get_parameter("mountpoint").as_string();
    username_ = get_parameter("username").as_string();
    password_ = get_parameter("password").as_string();
    rtcm_topic_ = get_parameter("rtcm_topic").as_string();
    reconnect_delay_sec_ = get_parameter("reconnect_delay_sec").as_double();
    send_gga_ = get_parameter("send_gga").as_bool();
    gga_update_rate_sec_ = get_parameter("gga_update_rate_sec").as_double();
    latitude_ = get_parameter("latitude").as_double();
    longitude_ = get_parameter("longitude").as_double();
    altitude_ = get_parameter("altitude").as_double();
    
    // Create publisher
    pub_rtcm_ = create_publisher<std_msgs::msg::UInt8MultiArray>(rtcm_topic_, 10);
    
    RCLCPP_INFO(get_logger(), "NTRIP Client Configuration:");
    RCLCPP_INFO(get_logger(), "  Host: %s:%d", host_.c_str(), port_);
    RCLCPP_INFO(get_logger(), "  Mountpoint: %s", mountpoint_.c_str());
    RCLCPP_INFO(get_logger(), "  RTCM Topic: %s", rtcm_topic_.c_str());
    
    if (mountpoint_.empty()) {
      RCLCPP_ERROR(get_logger(), "Mountpoint is required! Set the 'mountpoint' parameter.");
      return;
    }
    
    // Start connection thread
    running_ = true;
    connection_thread_ = std::thread(&NtripClientNode::connection_loop, this);
  }
  
  ~NtripClientNode()
  {
    running_ = false;
    if (connection_thread_.joinable()) {
      connection_thread_.join();
    }
    if (socket_fd_ >= 0) {
      close(socket_fd_);
    }
  }

private:
  void connection_loop()
  {
    while (running_ && rclcpp::ok()) {
      if (!connect_to_caster()) {
        RCLCPP_WARN(get_logger(), "Connection failed, retrying in %.1f seconds...", 
          reconnect_delay_sec_);
        std::this_thread::sleep_for(
          std::chrono::milliseconds(static_cast<int>(reconnect_delay_sec_ * 1000)));
        continue;
      }
      
      RCLCPP_INFO(get_logger(), "Connected to NTRIP caster");
      
      // Read RTCM data
      read_rtcm_data();
      
      // Connection lost
      if (socket_fd_ >= 0) {
        close(socket_fd_);
        socket_fd_ = -1;
      }
      
      if (running_) {
        RCLCPP_WARN(get_logger(), "Connection lost, reconnecting in %.1f seconds...",
          reconnect_delay_sec_);
        std::this_thread::sleep_for(
          std::chrono::milliseconds(static_cast<int>(reconnect_delay_sec_ * 1000)));
      }
    }
  }
  
  bool connect_to_caster()
  {
    // Resolve hostname
    struct hostent* server = gethostbyname(host_.c_str());
    if (!server) {
      RCLCPP_ERROR(get_logger(), "Failed to resolve hostname: %s", host_.c_str());
      return false;
    }
    
    // Create socket
    socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "Failed to create socket");
      return false;
    }
    
    // Set socket timeout
    struct timeval timeout;
    timeout.tv_sec = 10;
    timeout.tv_usec = 0;
    setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(socket_fd_, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
    
    // Connect
    struct sockaddr_in serv_addr;
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    memcpy(&serv_addr.sin_addr.s_addr, server->h_addr, server->h_length);
    serv_addr.sin_port = htons(port_);
    
    if (connect(socket_fd_, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
      RCLCPP_ERROR(get_logger(), "Failed to connect to %s:%d", host_.c_str(), port_);
      close(socket_fd_);
      socket_fd_ = -1;
      return false;
    }
    
    // Build NTRIP request
    std::stringstream request;
    request << "GET /" << mountpoint_ << " HTTP/1.0\r\n";
    request << "Host: " << host_ << "\r\n";
    request << "Ntrip-Version: Ntrip/2.0\r\n";
    request << "User-Agent: NTRIP gnss_compass_driver/1.0.0 (" << username_ << ")\r\n";
    
    if (!username_.empty()) {
      std::string auth = username_ + ":" + password_;
      request << "Authorization: Basic " << base64_encode(auth) << "\r\n";
    }
    
    request << "\r\n";
    
    std::string request_str = request.str();
    if (send(socket_fd_, request_str.c_str(), request_str.size(), 0) < 0) {
      RCLCPP_ERROR(get_logger(), "Failed to send NTRIP request");
      close(socket_fd_);
      socket_fd_ = -1;
      return false;
    }
    
    // Read response
    char buffer[1024];
    ssize_t n = recv(socket_fd_, buffer, sizeof(buffer) - 1, 0);
    if (n <= 0) {
      RCLCPP_ERROR(get_logger(), "No response from NTRIP caster");
      close(socket_fd_);
      socket_fd_ = -1;
      return false;
    }
    
    buffer[n] = '\0';
    std::string response(buffer);
    
    // Check for ICY 200 OK or HTTP/1.x 200 OK
    if (response.find("ICY 200 OK") == std::string::npos &&
        response.find("200 OK") == std::string::npos) {
      RCLCPP_ERROR(get_logger(), "NTRIP caster rejected connection: %s", 
        response.substr(0, 100).c_str());
      close(socket_fd_);
      socket_fd_ = -1;
      return false;
    }
    
    RCLCPP_INFO(get_logger(), "NTRIP authentication successful");
    
    // Send initial GGA if configured
    if (send_gga_ && (latitude_ != 0.0 || longitude_ != 0.0)) {
      send_gga_message();
    }
    
    return true;
  }
  
  void read_rtcm_data()
  {
    std::vector<uint8_t> buffer(4096);
    auto last_gga_time = std::chrono::steady_clock::now();
    
    while (running_ && rclcpp::ok()) {
      ssize_t n = recv(socket_fd_, buffer.data(), buffer.size(), 0);
      
      if (n <= 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          // Timeout - send GGA if needed
          if (send_gga_) {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
              now - last_gga_time).count();
            if (elapsed >= gga_update_rate_sec_) {
              send_gga_message();
              last_gga_time = now;
            }
          }
          continue;
        }
        // Connection closed
        break;
      }
      
      // Publish RTCM data
      auto msg = std::make_unique<std_msgs::msg::UInt8MultiArray>();
      msg->data.assign(buffer.begin(), buffer.begin() + n);
      pub_rtcm_->publish(std::move(msg));
      
      bytes_received_ += n;
      messages_received_++;
      
      RCLCPP_DEBUG(get_logger(), "Received %zd bytes of RTCM data (total: %zu)", 
        n, bytes_received_);
      
      // Send GGA periodically
      if (send_gga_) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
          now - last_gga_time).count();
        if (elapsed >= gga_update_rate_sec_) {
          send_gga_message();
          last_gga_time = now;
        }
      }
    }
  }
  
  void send_gga_message()
  {
    if (socket_fd_ < 0) return;
    
    // Build GGA sentence
    std::string gga = build_gga_sentence(latitude_, longitude_, altitude_);
    
    if (send(socket_fd_, gga.c_str(), gga.size(), 0) > 0) {
      RCLCPP_DEBUG(get_logger(), "Sent GGA: %s", gga.c_str());
    }
  }
  
  std::string build_gga_sentence(double lat, double lon, double alt)
  {
    // Get current UTC time
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    auto tm_utc = *gmtime(&time_t_now);
    
    // Convert decimal degrees to NMEA format (ddmm.mmmm)
    char lat_dir = (lat >= 0) ? 'N' : 'S';
    char lon_dir = (lon >= 0) ? 'E' : 'W';
    lat = std::abs(lat);
    lon = std::abs(lon);
    
    int lat_deg = static_cast<int>(lat);
    double lat_min = (lat - lat_deg) * 60.0;
    int lon_deg = static_cast<int>(lon);
    double lon_min = (lon - lon_deg) * 60.0;
    
    std::stringstream ss;
    ss << "$GPGGA,";
    ss << std::setfill('0') << std::setw(2) << tm_utc.tm_hour;
    ss << std::setfill('0') << std::setw(2) << tm_utc.tm_min;
    ss << std::setfill('0') << std::setw(2) << tm_utc.tm_sec << ".00,";
    ss << std::setfill('0') << std::setw(2) << lat_deg;
    ss << std::fixed << std::setprecision(5) << lat_min << "," << lat_dir << ",";
    ss << std::setfill('0') << std::setw(3) << lon_deg;
    ss << std::fixed << std::setprecision(5) << lon_min << "," << lon_dir << ",";
    ss << "1,12,1.0,";  // Fix quality, num sats, HDOP
    ss << std::fixed << std::setprecision(1) << alt << ",M,";
    ss << "0.0,M,,";  // Geoid separation, age of diff, station ID
    
    // Calculate checksum
    std::string sentence = ss.str();
    uint8_t checksum = 0;
    for (size_t i = 1; i < sentence.size(); i++) {
      checksum ^= static_cast<uint8_t>(sentence[i]);
    }
    
    std::stringstream result;
    result << sentence << "*" << std::hex << std::uppercase 
           << std::setfill('0') << std::setw(2) << static_cast<int>(checksum) << "\r\n";
    
    return result.str();
  }
  
  // Parameters
  std::string host_;
  int port_;
  std::string mountpoint_;
  std::string username_;
  std::string password_;
  std::string rtcm_topic_;
  double reconnect_delay_sec_;
  bool send_gga_;
  double gga_update_rate_sec_;
  double latitude_;
  double longitude_;
  double altitude_;
  
  // Publisher
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pub_rtcm_;
  
  // Connection state
  int socket_fd_{-1};
  std::atomic<bool> running_{false};
  std::thread connection_thread_;
  
  // Stats
  size_t bytes_received_{0};
  size_t messages_received_{0};
};

}  // namespace gnss_compass

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<gnss_compass::NtripClientNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

