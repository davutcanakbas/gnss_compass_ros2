// Copyright (c) 2026 Davut Can Akbas
// SPDX-License-Identifier: MIT

#include "gnss_compass_driver/gnss_compass_driver_node.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>
#include <chrono>
#include <thread>
#include <iomanip>
#include <sstream>

namespace gnss_compass
{

// =============================================================================
// Construction / Destruction
// =============================================================================

GnssCompassDriverNode::GnssCompassDriverNode(const rclcpp::NodeOptions& options)
  : rclcpp_lifecycle::LifecycleNode("gnss_compass_driver", options)
{
  declare_parameters();
  
  RCLCPP_INFO(get_logger(), "GNSS Compass Driver created");
}

GnssCompassDriverNode::~GnssCompassDriverNode()
{
  if (serial_port_ && serial_port_->is_open()) {
    serial_port_->close();
  }
}

// =============================================================================
// Parameter Handling
// =============================================================================

void GnssCompassDriverNode::declare_parameters()
{
  // Serial port parameters
  declare_parameter("port", "/dev/ttyUSB0");
  declare_parameter("baudrate", 460800);
  
  // Frame IDs
  declare_parameter("frame_id", "gnss_link");
  
  // Protocol settings
  declare_parameter("use_ubx", true);
  declare_parameter("use_nmea", true);
  declare_parameter("use_unicore", true);
  
  // Output configuration
  declare_parameter("publish_heading", true);
  declare_parameter("publish_velocity", true);
  declare_parameter("publish_time_ref", true);
  declare_parameter("publish_diagnostics", true);
  
  // Antenna configuration
  declare_parameter("antenna_baseline", 1.0);
  
  // Rates
  declare_parameter("diagnostics_rate_hz", 1.0);
  declare_parameter("output_rate_hz", 20.0);  // GNSS output rate (1-20 Hz for heading)
  
  // RTCM correction input
  declare_parameter("rtcm_topic", "rtcm");
}

void GnssCompassDriverNode::load_parameters()
{
  params_.port = get_parameter("port").as_string();
  params_.baudrate = get_parameter("baudrate").as_int();
  params_.frame_id = get_parameter("frame_id").as_string();
  params_.use_ubx = get_parameter("use_ubx").as_bool();
  params_.use_nmea = get_parameter("use_nmea").as_bool();
  params_.use_unicore = get_parameter("use_unicore").as_bool();
  params_.publish_heading = get_parameter("publish_heading").as_bool();
  params_.publish_velocity = get_parameter("publish_velocity").as_bool();
  params_.publish_time_ref = get_parameter("publish_time_ref").as_bool();
  params_.publish_diagnostics = get_parameter("publish_diagnostics").as_bool();
  params_.antenna_baseline = get_parameter("antenna_baseline").as_double();
  params_.diagnostics_rate_hz = get_parameter("diagnostics_rate_hz").as_double();
  params_.output_rate_hz = get_parameter("output_rate_hz").as_double();
  params_.rtcm_topic = get_parameter("rtcm_topic").as_string();
  
  // Clamp output rate to valid range (1-20 Hz for dual antenna heading)
  if (params_.output_rate_hz < 1.0) params_.output_rate_hz = 1.0;
  if (params_.output_rate_hz > 20.0) params_.output_rate_hz = 20.0;
  
  RCLCPP_INFO(get_logger(), "Parameters loaded:");
  RCLCPP_INFO(get_logger(), "  port: %s", params_.port.c_str());
  RCLCPP_INFO(get_logger(), "  baudrate: %d", params_.baudrate);
  RCLCPP_INFO(get_logger(), "  frame_id: %s", params_.frame_id.c_str());
  RCLCPP_INFO(get_logger(), "  use_ubx: %s", params_.use_ubx ? "true" : "false");
  RCLCPP_INFO(get_logger(), "  use_nmea: %s", params_.use_nmea ? "true" : "false");
  RCLCPP_INFO(get_logger(), "  use_unicore: %s", params_.use_unicore ? "true" : "false");
  RCLCPP_INFO(get_logger(), "  antenna_baseline: %.3f m", params_.antenna_baseline);
  RCLCPP_INFO(get_logger(), "  output_rate_hz: %.0f Hz", params_.output_rate_hz);
}

// =============================================================================
// Lifecycle Callbacks
// =============================================================================

GnssCompassDriverNode::CallbackReturn 
GnssCompassDriverNode::on_configure(const rclcpp_lifecycle::State& /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring...");
  
  load_parameters();
  
  // Create publishers
  pub_fix_ = create_publisher<sensor_msgs::msg::NavSatFix>("gnss/fix", 10);
  
  if (params_.publish_heading) {
    pub_heading_quat_ = create_publisher<geometry_msgs::msg::QuaternionStamped>(
      "gnss/heading/quaternion", 10);
    pub_heading_deg_ = create_publisher<std_msgs::msg::Float64>(
      "gnss/heading/degrees", 10);
  }
  
  if (params_.publish_velocity) {
    pub_velocity_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      "gnss/velocity", 10);
  }
  
  if (params_.publish_time_ref) {
    pub_time_ref_ = create_publisher<sensor_msgs::msg::TimeReference>(
      "gnss/time_reference", 10);
  }
  
  if (params_.publish_diagnostics) {
    pub_diagnostics_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10);
  }
  
  // Create RTCM correction subscriber
  sub_rtcm_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
    params_.rtcm_topic, 10,
    [this](const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
      on_rtcm_received(msg);
    });
  
  RCLCPP_INFO(get_logger(), "Subscribed to RTCM corrections on: %s", params_.rtcm_topic.c_str());
  
  // Create serial port
  serial_port_ = std::make_unique<SerialPort>();
  
  SerialConfig serial_config;
  serial_config.port = params_.port;
  serial_config.baudrate = static_cast<uint32_t>(params_.baudrate);
  serial_config.auto_reconnect = true;
  serial_config.reconnect_delay_ms = 1000;
  serial_config.max_reconnect_attempts = 0;  // Infinite retries
  
  serial_port_->configure(serial_config);
  
  serial_port_->set_data_callback(
    [this](const uint8_t* data, size_t size) {
      on_serial_data(data, size);
    });
  
  serial_port_->set_error_callback(
    [this](const std::string& error) {
      on_serial_error(error);
    });
  
  serial_port_->set_connected_callback(
    [this](bool connected) {
      on_serial_connected(connected);
    });
  
  // Create parsers
  nmea_parser_ = std::make_unique<NmeaParser>();
  ubx_parser_ = std::make_unique<UbxParser>();
  unicore_parser_ = std::make_unique<UniCoreParser>();
  
  // Setup NMEA callbacks
  if (params_.use_nmea) {
    nmea_parser_->set_gga_callback([this](const NmeaGGA& msg) { on_nmea_gga(msg); });
    nmea_parser_->set_rmc_callback([this](const NmeaRMC& msg) { on_nmea_rmc(msg); });
    nmea_parser_->set_vtg_callback([this](const NmeaVTG& msg) { on_nmea_vtg(msg); });
    nmea_parser_->set_gst_callback([this](const NmeaGST& msg) { on_nmea_gst(msg); });
    nmea_parser_->set_hdt_callback([this](const NmeaHDT& msg) { on_nmea_hdt(msg); });
    nmea_parser_->set_ths_callback([this](const NmeaTHS& msg) { on_nmea_ths(msg); });
    nmea_parser_->set_zda_callback([this](const NmeaZDA& msg) { on_nmea_zda(msg); });
  }
  
  // Setup UBX callbacks
  if (params_.use_ubx) {
    ubx_parser_->set_nav_pvt_callback([this](const ubx::NavPvt& msg) { on_ubx_nav_pvt(msg); });
    ubx_parser_->set_nav_relposned_callback([this](const ubx::NavRelPosNed& msg) { on_ubx_nav_relposned(msg); });
    ubx_parser_->set_nav_status_callback([this](const ubx::NavStatus& msg) { on_ubx_nav_status(msg); });
    ubx_parser_->set_nav_dop_callback([this](const ubx::NavDop& msg) { on_ubx_nav_dop(msg); });
    ubx_parser_->set_ack_callback([this](uint8_t cls, uint8_t id, bool ack) { on_ubx_ack(cls, id, ack); });
  }
  
  // Setup UniCore callbacks
  if (params_.use_unicore) {
    unicore_parser_->set_heading_callback([this](const UniCoreHeading& msg) { on_unicore_heading(msg); });
    unicore_parser_->set_bestnav_callback([this](const UniCoreBestNav& msg) { on_unicore_bestnav(msg); });
  }
  
  RCLCPP_INFO(get_logger(), "Configuration complete");
  return CallbackReturn::SUCCESS;
}

GnssCompassDriverNode::CallbackReturn 
GnssCompassDriverNode::on_activate(const rclcpp_lifecycle::State& /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating...");
  
  // Activate publishers
  pub_fix_->on_activate();
  if (pub_heading_quat_) pub_heading_quat_->on_activate();
  if (pub_heading_deg_) pub_heading_deg_->on_activate();
  if (pub_velocity_) pub_velocity_->on_activate();
  if (pub_time_ref_) pub_time_ref_->on_activate();
  if (pub_diagnostics_) pub_diagnostics_->on_activate();
  
  // Open serial port (configure_receiver is called automatically via on_serial_connected callback)
  if (!serial_port_->open()) {
    RCLCPP_ERROR(get_logger(), "Failed to open serial port: %s", params_.port.c_str());
    return CallbackReturn::FAILURE;
  }
  
  // Start diagnostics timer
  if (params_.publish_diagnostics && params_.diagnostics_rate_hz > 0) {
    auto period = std::chrono::duration<double>(1.0 / params_.diagnostics_rate_hz);
    diagnostics_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { publish_diagnostics(); });
  }
  
  RCLCPP_INFO(get_logger(), "Activated");
  return CallbackReturn::SUCCESS;
}

GnssCompassDriverNode::CallbackReturn 
GnssCompassDriverNode::on_deactivate(const rclcpp_lifecycle::State& /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating...");
  
  // Stop timers
  if (diagnostics_timer_) {
    diagnostics_timer_->cancel();
    diagnostics_timer_.reset();
  }
  
  // Close serial port
  if (serial_port_) {
    serial_port_->close();
  }
  
  // Deactivate publishers
  pub_fix_->on_deactivate();
  if (pub_heading_quat_) pub_heading_quat_->on_deactivate();
  if (pub_heading_deg_) pub_heading_deg_->on_deactivate();
  if (pub_velocity_) pub_velocity_->on_deactivate();
  if (pub_time_ref_) pub_time_ref_->on_deactivate();
  if (pub_diagnostics_) pub_diagnostics_->on_deactivate();
  
  RCLCPP_INFO(get_logger(), "Deactivated");
  return CallbackReturn::SUCCESS;
}

GnssCompassDriverNode::CallbackReturn 
GnssCompassDriverNode::on_cleanup(const rclcpp_lifecycle::State& /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up...");
  
  serial_port_.reset();
  nmea_parser_.reset();
  ubx_parser_.reset();
  unicore_parser_.reset();
  
  sub_rtcm_.reset();
  pub_fix_.reset();
  pub_heading_quat_.reset();
  pub_heading_deg_.reset();
  pub_velocity_.reset();
  pub_time_ref_.reset();
  pub_diagnostics_.reset();
  
  RCLCPP_INFO(get_logger(), "Cleanup complete");
  return CallbackReturn::SUCCESS;
}

GnssCompassDriverNode::CallbackReturn 
GnssCompassDriverNode::on_shutdown(const rclcpp_lifecycle::State& /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down...");
  
  if (serial_port_ && serial_port_->is_open()) {
    serial_port_->close();
  }
  
  return CallbackReturn::SUCCESS;
}

// =============================================================================
// Serial Port Callbacks
// =============================================================================

void GnssCompassDriverNode::on_serial_data(const uint8_t* data, size_t size)
{
  last_message_time_ = Clock::now();
  
  // Process through all parsers - they will ignore non-matching data
  if (params_.use_nmea) {
    nmea_parser_->process(data, size);
  }
  
  if (params_.use_ubx) {
    ubx_parser_->process(data, size);
  }
  
  if (params_.use_unicore) {
    unicore_parser_->process(data, size);
  }
}

void GnssCompassDriverNode::on_serial_error(const std::string& error)
{
  RCLCPP_WARN(get_logger(), "Serial error: %s", error.c_str());
}

void GnssCompassDriverNode::on_serial_connected(bool connected)
{
  is_connected_ = connected;
  
  if (connected) {
    RCLCPP_INFO(get_logger(), "Serial port connected");
    // Reconfigure receiver on reconnection
    configure_receiver();
  } else {
    RCLCPP_WARN(get_logger(), "Serial port disconnected");
  }
}

// =============================================================================
// Receiver Configuration
// =============================================================================

void GnssCompassDriverNode::configure_receiver()
{
  if (!serial_port_ || !serial_port_->is_open()) {
    return;
  }
  
  RCLCPP_INFO(get_logger(), "Configuring receiver...");
  
  // Calculate output interval from rate (e.g., 20 Hz = 0.05 sec)
  double interval = 1.0 / params_.output_rate_hz;
  
  // Configure Unicore receiver (UM982)
  if (params_.use_unicore) {
    // Wait for receiver to be ready
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    char buf[128];
    std::string cmd;
    
    // First, try to wake up the command interface
    cmd = "\r\n";
    serial_port_->write(reinterpret_cast<const uint8_t*>(cmd.c_str()), cmd.size());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Configure GPGGA output at desired rate (most reliable command)
    snprintf(buf, sizeof(buf), "GPGGA %.2f\r\n", interval);
    cmd = buf;
    RCLCPP_INFO(get_logger(), "Sending: %s", buf);
    serial_port_->write(reinterpret_cast<const uint8_t*>(cmd.c_str()), cmd.size());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Configure BESTNAVA output at desired rate
    snprintf(buf, sizeof(buf), "LOG BESTNAVA ONTIME %.2f\r\n", interval);
    cmd = buf;
    serial_port_->write(reinterpret_cast<const uint8_t*>(cmd.c_str()), cmd.size());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Configure UNIHEADINGA output at desired rate
    snprintf(buf, sizeof(buf), "LOG UNIHEADINGA ONTIME %.2f\r\n", interval);
    cmd = buf;
    serial_port_->write(reinterpret_cast<const uint8_t*>(cmd.c_str()), cmd.size());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Save config to flash (optional - comment out if you don't want persistent config)
    // cmd = "SAVECONFIG\r\n";
    // serial_port_->write(reinterpret_cast<const uint8_t*>(cmd.c_str()), cmd.size());
    // std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    RCLCPP_INFO(get_logger(), "Unicore receiver configured at %.0f Hz", params_.output_rate_hz);
  }
  
  // Configure u-blox receiver (ZED-F9P)
  if (params_.use_ubx) {
    uint8_t rate = static_cast<uint8_t>(params_.output_rate_hz);
    
    // Enable NAV-PVT message
    auto msg = UbxParser::create_cfg_msg(
      static_cast<uint8_t>(ubx::MessageClass::NAV),
      static_cast<uint8_t>(ubx::NavMessageId::PVT),
      rate);
    serial_port_->write(msg.data(), msg.size());
    
    // Enable NAV-RELPOSNED message (critical for heading)
    msg = UbxParser::create_cfg_msg(
      static_cast<uint8_t>(ubx::MessageClass::NAV),
      static_cast<uint8_t>(ubx::NavMessageId::RELPOSNED),
      rate);
    serial_port_->write(msg.data(), msg.size());
    
    // Enable NAV-DOP
    msg = UbxParser::create_cfg_msg(
      static_cast<uint8_t>(ubx::MessageClass::NAV),
      static_cast<uint8_t>(ubx::NavMessageId::DOP),
      rate);
    serial_port_->write(msg.data(), msg.size());
    
    RCLCPP_INFO(get_logger(), "u-blox receiver configured at %d Hz", rate);
  }
  
  RCLCPP_INFO(get_logger(), "Receiver configuration complete");
}

// =============================================================================
// NMEA Message Handlers
// =============================================================================

void GnssCompassDriverNode::on_nmea_gga(const NmeaGGA& msg)
{
  if (!msg.valid) return;
  
  std::lock_guard<std::mutex> lock(data_mutex_);
  
  gnss_data_.position.latitude_deg = msg.latitude_deg;
  gnss_data_.position.longitude_deg = msg.longitude_deg;
  gnss_data_.position.altitude_m = msg.altitude_m + msg.geoid_sep_m;  // Height above ellipsoid
  gnss_data_.position.undulation_m = msg.geoid_sep_m;
  gnss_data_.position.satellites_used = msg.satellites_used;
  gnss_data_.position.hdop = msg.hdop;
  gnss_data_.diff_age_s = msg.diff_age_s;
  
  // Map GGA quality indicator to fix type and RTK status
  switch (msg.fix_quality) {
    case 0:
      gnss_data_.position.fix_type = FixType::NO_FIX;
      gnss_data_.position.rtk_status = RtkStatus::NONE;
      break;
    case 1:
      gnss_data_.position.fix_type = FixType::FIX_3D;
      gnss_data_.position.rtk_status = RtkStatus::NONE;
      break;
    case 2:
      gnss_data_.position.fix_type = FixType::FIX_3D;
      gnss_data_.position.rtk_status = RtkStatus::NONE;  // DGPS
      break;
    case 4:
      gnss_data_.position.fix_type = FixType::FIX_3D;
      gnss_data_.position.rtk_status = RtkStatus::FIXED;
      break;
    case 5:
      gnss_data_.position.fix_type = FixType::FIX_3D;
      gnss_data_.position.rtk_status = RtkStatus::FLOAT;
      break;
    default:
      gnss_data_.position.fix_type = FixType::NO_FIX;
      gnss_data_.position.rtk_status = RtkStatus::NONE;
  }
  
  gnss_data_.position.valid = true;
  gnss_data_.position.timestamp = Clock::now();
  position_updated_ = true;
  
  publish_nav_sat_fix();
}

void GnssCompassDriverNode::on_nmea_rmc(const NmeaRMC& msg)
{
  if (!msg.valid) return;
  
  std::lock_guard<std::mutex> lock(data_mutex_);
  
  // Update velocity from RMC
  gnss_data_.velocity.ground_speed_mps = msg.speed_knots * 0.514444;  // knots to m/s
  gnss_data_.velocity.course_deg = msg.course_deg;
  gnss_data_.velocity.valid = true;
  gnss_data_.velocity.timestamp = Clock::now();
  velocity_updated_ = true;
  
  // Update time from RMC
  if (msg.date > 0) {
    gnss_data_.time.day = static_cast<uint8_t>((msg.date / 10000) % 100);
    gnss_data_.time.month = static_cast<uint8_t>((msg.date / 100) % 100);
    gnss_data_.time.year = static_cast<uint16_t>(2000 + (msg.date % 100));
    gnss_data_.time.date_valid = true;
  }
  
  gnss_data_.time.timestamp = Clock::now();
}

void GnssCompassDriverNode::on_nmea_vtg(const NmeaVTG& msg)
{
  if (!msg.valid) return;
  
  std::lock_guard<std::mutex> lock(data_mutex_);
  
  gnss_data_.velocity.ground_speed_mps = msg.speed_kmh / 3.6;
  gnss_data_.velocity.course_deg = msg.course_true_deg;
  gnss_data_.velocity.valid = true;
  gnss_data_.velocity.timestamp = Clock::now();
  velocity_updated_ = true;
  
  if (params_.publish_velocity) {
    publish_velocity();
  }
}

void GnssCompassDriverNode::on_nmea_gst(const NmeaGST& msg)
{
  if (!msg.valid) return;
  
  std::lock_guard<std::mutex> lock(data_mutex_);
  
  gnss_data_.position.horizontal_accuracy_m = std::sqrt(
    msg.lat_error_m * msg.lat_error_m + msg.lon_error_m * msg.lon_error_m);
  gnss_data_.position.vertical_accuracy_m = msg.alt_error_m;
  
  update_covariance();
}

void GnssCompassDriverNode::on_nmea_hdt(const NmeaHDT& msg)
{
  if (!msg.valid) return;
  
  std::lock_guard<std::mutex> lock(data_mutex_);
  
  gnss_data_.heading.heading_deg = msg.heading_deg;
  gnss_data_.heading.heading_valid = true;
  gnss_data_.heading.timestamp = Clock::now();
  heading_updated_ = true;
  
  if (params_.publish_heading) {
    publish_heading();
  }
}

void GnssCompassDriverNode::on_nmea_ths(const NmeaTHS& msg)
{
  if (!msg.valid) return;
  
  std::lock_guard<std::mutex> lock(data_mutex_);
  
  gnss_data_.heading.heading_deg = msg.heading_deg;
  gnss_data_.heading.heading_valid = true;
  gnss_data_.heading.timestamp = Clock::now();
  heading_updated_ = true;
  
  if (params_.publish_heading) {
    publish_heading();
  }
}

void GnssCompassDriverNode::on_nmea_zda(const NmeaZDA& msg)
{
  if (!msg.valid) return;
  
  std::lock_guard<std::mutex> lock(data_mutex_);
  
  gnss_data_.time.year = msg.year;
  gnss_data_.time.month = msg.month;
  gnss_data_.time.day = msg.day;
  gnss_data_.time.date_valid = true;
  gnss_data_.time.time_valid = true;
  gnss_data_.time.timestamp = Clock::now();
  time_updated_ = true;
  
  if (params_.publish_time_ref) {
    publish_time_reference();
  }
}

// =============================================================================
// UBX Message Handlers
// =============================================================================

void GnssCompassDriverNode::on_ubx_nav_pvt(const ubx::NavPvt& msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  
  // Position
  gnss_data_.position.latitude_deg = msg.latitude_deg();
  gnss_data_.position.longitude_deg = msg.longitude_deg();
  gnss_data_.position.altitude_m = msg.altitude_m();
  gnss_data_.position.horizontal_accuracy_m = msg.horizontal_accuracy_m();
  gnss_data_.position.vertical_accuracy_m = msg.vertical_accuracy_m();
  gnss_data_.position.satellites_used = msg.numSV;
  gnss_data_.position.pdop = static_cast<float>(msg.pdop());
  
  // Map fix type
  switch (msg.fixType) {
    case 0: gnss_data_.position.fix_type = FixType::NO_FIX; break;
    case 1: gnss_data_.position.fix_type = FixType::DEAD_RECKONING; break;
    case 2: gnss_data_.position.fix_type = FixType::FIX_2D; break;
    case 3: gnss_data_.position.fix_type = FixType::FIX_3D; break;
    case 4: gnss_data_.position.fix_type = FixType::GNSS_DEAD_RECKONING; break;
    case 5: gnss_data_.position.fix_type = FixType::TIME_ONLY; break;
    default: gnss_data_.position.fix_type = FixType::NO_FIX;
  }
  
  // RTK status from carrier solution
  switch (msg.carrier_soln()) {
    case 0: gnss_data_.position.rtk_status = RtkStatus::NONE; break;
    case 1: gnss_data_.position.rtk_status = RtkStatus::FLOAT; break;
    case 2: gnss_data_.position.rtk_status = RtkStatus::FIXED; break;
    default: gnss_data_.position.rtk_status = RtkStatus::NONE;
  }
  
  gnss_data_.position.valid = msg.gnss_fix_ok();
  gnss_data_.position.timestamp = Clock::now();
  
  // Velocity
  gnss_data_.velocity.vel_north_mps = msg.vel_north_mps();
  gnss_data_.velocity.vel_east_mps = msg.vel_east_mps();
  gnss_data_.velocity.vel_down_mps = msg.vel_down_mps();
  gnss_data_.velocity.ground_speed_mps = msg.ground_speed_mps();
  gnss_data_.velocity.course_deg = msg.heading_motion_deg();
  gnss_data_.velocity.speed_accuracy_mps = msg.speed_accuracy_mps();
  gnss_data_.velocity.course_accuracy_deg = msg.heading_accuracy_deg();
  gnss_data_.velocity.valid = msg.gnss_fix_ok();
  gnss_data_.velocity.timestamp = Clock::now();
  
  // Time
  gnss_data_.time.year = msg.year;
  gnss_data_.time.month = msg.month;
  gnss_data_.time.day = msg.day;
  gnss_data_.time.hour = msg.hour;
  gnss_data_.time.minute = msg.min;
  gnss_data_.time.second = msg.sec + msg.nano * 1e-9;
  gnss_data_.time.date_valid = msg.valid_date();
  gnss_data_.time.time_valid = msg.valid_time();
  gnss_data_.time.fully_resolved = msg.fully_resolved();
  gnss_data_.time.timestamp = Clock::now();
  
  update_covariance();
  position_updated_ = true;
  velocity_updated_ = true;
  time_updated_ = true;
  
  publish_nav_sat_fix();
  
  if (params_.publish_velocity) {
    publish_velocity();
  }
  
  if (params_.publish_time_ref) {
    publish_time_reference();
  }
}

void GnssCompassDriverNode::on_ubx_nav_relposned(const ubx::NavRelPosNed& msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  
  gnss_data_.heading.rel_pos_n_m = msg.rel_pos_n_m();
  gnss_data_.heading.rel_pos_e_m = msg.rel_pos_e_m();
  gnss_data_.heading.rel_pos_d_m = msg.rel_pos_d_m();
  gnss_data_.heading.baseline_length_m = msg.baseline_length_m();
  gnss_data_.heading.heading_deg = msg.heading_deg();
  gnss_data_.heading.heading_accuracy_deg = msg.heading_accuracy_deg();
  
  // Calculate pitch from relative position
  if (msg.baseline_length_m() > 0.01) {
    gnss_data_.heading.pitch_deg = std::asin(-msg.rel_pos_d_m() / msg.baseline_length_m()) 
                                   * 180.0 / M_PI;
  }
  
  gnss_data_.heading.rel_pos_valid = msg.rel_pos_valid();
  gnss_data_.heading.heading_valid = msg.rel_pos_heading_valid();
  gnss_data_.heading.baseline_fixed = (msg.carrier_soln() == 2);
  gnss_data_.heading.carrier_solution_valid = (msg.carrier_soln() > 0);
  gnss_data_.heading.timestamp = Clock::now();
  
  heading_updated_ = true;
  
  if (params_.publish_heading && msg.rel_pos_heading_valid()) {
    publish_heading();
  }
  
  RCLCPP_DEBUG(get_logger(), 
    "RELPOSNED: heading=%.2f° acc=%.2f° baseline=%.3fm carrier=%d",
    msg.heading_deg(), msg.heading_accuracy_deg(), 
    msg.baseline_length_m(), msg.carrier_soln());
}

void GnssCompassDriverNode::on_ubx_nav_status(const ubx::NavStatus& msg)
{
  RCLCPP_DEBUG(get_logger(), "NAV-STATUS: fix=%d flags=0x%02X ttff=%d ms",
    msg.gpsFix, msg.flags, msg.ttff);
}

void GnssCompassDriverNode::on_ubx_nav_dop(const ubx::NavDop& msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  
  gnss_data_.position.hdop = static_cast<float>(msg.hdop());
  gnss_data_.position.vdop = static_cast<float>(msg.vdop());
  gnss_data_.position.pdop = static_cast<float>(msg.pdop());
}

void GnssCompassDriverNode::on_ubx_ack(uint8_t cls, uint8_t id, bool ack)
{
  RCLCPP_DEBUG(get_logger(), "UBX %s: class=0x%02X id=0x%02X",
    ack ? "ACK" : "NAK", cls, id);
}

// =============================================================================
// UniCore Message Handlers
// =============================================================================

void GnssCompassDriverNode::on_unicore_heading(const UniCoreHeading& msg)
{
  if (!msg.valid) return;
  
  std::lock_guard<std::mutex> lock(data_mutex_);
  
  gnss_data_.heading.heading_deg = msg.heading_deg;
  gnss_data_.heading.pitch_deg = msg.pitch_deg;
  gnss_data_.heading.baseline_length_m = msg.baseline_length_m;
  gnss_data_.heading.heading_accuracy_deg = msg.heading_std_deg;
  gnss_data_.heading.heading_valid = true;
  
  // Map UniCore solution status to our flags
  gnss_data_.heading.baseline_fixed = (msg.pos_type == UniCorePosType::L1_INT ||
                                        msg.pos_type == UniCorePosType::NARROW_INT ||
                                        msg.pos_type == UniCorePosType::WIDE_INT);
  gnss_data_.heading.carrier_solution_valid = (msg.pos_type == UniCorePosType::L1_FLOAT ||
                                                msg.pos_type == UniCorePosType::NARROW_FLOAT ||
                                                gnss_data_.heading.baseline_fixed);
  
  gnss_data_.heading.timestamp = Clock::now();
  heading_updated_ = true;
  
  if (params_.publish_heading) {
    publish_heading();
  }
  
  RCLCPP_DEBUG(get_logger(), 
    "UNIHEADINGA: heading=%.2f° pitch=%.2f° baseline=%.3fm type=%s",
    msg.heading_deg, msg.pitch_deg, msg.baseline_length_m, 
    to_string(msg.pos_type));
}

void GnssCompassDriverNode::on_unicore_bestnav(const UniCoreBestNav& msg)
{
  if (!msg.valid) return;
  
  std::lock_guard<std::mutex> lock(data_mutex_);
  
  // Update position
  gnss_data_.position.latitude_deg = msg.latitude_deg;
  gnss_data_.position.longitude_deg = msg.longitude_deg;
  gnss_data_.position.altitude_m = msg.height_m;
  gnss_data_.position.undulation_m = msg.undulation_m;
  gnss_data_.position.horizontal_accuracy_m = std::sqrt(
    msg.lat_std_m * msg.lat_std_m + msg.lon_std_m * msg.lon_std_m);
  gnss_data_.position.vertical_accuracy_m = msg.hgt_std_m;
  gnss_data_.position.satellites_used = msg.num_soln_satellites;
  gnss_data_.diff_age_s = msg.diff_age_s;
  
  // Map UniCore position type to our fix type
  gnss_data_.position.fix_type = FixType::FIX_3D;
  
  // Map to RTK status
  if (msg.pos_type == UniCorePosType::L1_INT || 
      msg.pos_type == UniCorePosType::NARROW_INT ||
      msg.pos_type == UniCorePosType::WIDE_INT) {
    gnss_data_.position.rtk_status = RtkStatus::FIXED;
  } else if (msg.pos_type == UniCorePosType::L1_FLOAT ||
             msg.pos_type == UniCorePosType::NARROW_FLOAT ||
             msg.pos_type == UniCorePosType::IONOFREE_FLOAT) {
    gnss_data_.position.rtk_status = RtkStatus::FLOAT;
  } else {
    gnss_data_.position.rtk_status = RtkStatus::NONE;
  }
  
  gnss_data_.position.valid = true;
  gnss_data_.position.timestamp = Clock::now();
  
  // Update velocity
  gnss_data_.velocity.ground_speed_mps = msg.horizontal_speed_mps;
  gnss_data_.velocity.course_deg = msg.track_ground_deg;
  gnss_data_.velocity.vel_down_mps = -msg.vertical_speed_mps;  // Convert up to down
  gnss_data_.velocity.valid = (msg.vel_sol_status == UniCoreSolutionStatus::SOL_COMPUTED);
  gnss_data_.velocity.timestamp = Clock::now();
  
  update_covariance();
  position_updated_ = true;
  velocity_updated_ = true;
  
  // BESTNAVA provides complete data, but NMEA GGA might be more frequent
  // Only publish if we haven't recently published from NMEA
  // For now, always publish
  publish_nav_sat_fix();
  
  if (params_.publish_velocity && gnss_data_.velocity.valid) {
    publish_velocity();
  }
  
  RCLCPP_DEBUG(get_logger(), 
    "BESTNAVA: lat=%.7f lon=%.7f hgt=%.2f spd=%.2f type=%s",
    msg.latitude_deg, msg.longitude_deg, msg.height_m,
    msg.horizontal_speed_mps, to_string(msg.pos_type));
}

// =============================================================================
// Publishing Methods
// =============================================================================

void GnssCompassDriverNode::publish_nav_sat_fix()
{
  if (!pub_fix_->is_activated()) return;
  
  auto msg = std::make_unique<sensor_msgs::msg::NavSatFix>();
  
  msg->header.stamp = now();
  msg->header.frame_id = params_.frame_id;
  
  msg->status = create_nav_sat_status();
  
  msg->latitude = gnss_data_.position.latitude_deg;
  msg->longitude = gnss_data_.position.longitude_deg;
  msg->altitude = gnss_data_.position.altitude_m;
  
  // Covariance (ENU order in NavSatFix)
  msg->position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  
  double h_var = gnss_data_.position.horizontal_accuracy_m * gnss_data_.position.horizontal_accuracy_m;
  double v_var = gnss_data_.position.vertical_accuracy_m * gnss_data_.position.vertical_accuracy_m;
  
  // ENU covariance matrix
  msg->position_covariance[0] = h_var;  // East variance
  msg->position_covariance[4] = h_var;  // North variance
  msg->position_covariance[8] = v_var;  // Up variance
  
  pub_fix_->publish(std::move(msg));
}

void GnssCompassDriverNode::publish_heading()
{
  // Publish as quaternion
  if (pub_heading_quat_ && pub_heading_quat_->is_activated()) {
    auto msg = std::make_unique<geometry_msgs::msg::QuaternionStamped>();
    
    msg->header.stamp = now();
    msg->header.frame_id = params_.frame_id;
    
    // Convert heading to quaternion (rotation around Z axis)
    // Heading is clockwise from North, quaternion rotation is counter-clockwise
    double heading_rad = gnss_data_.heading.heading_deg * M_PI / 180.0;
    
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, -heading_rad + M_PI/2);  // ENU convention
    msg->quaternion = tf2::toMsg(q);
    
    pub_heading_quat_->publish(std::move(msg));
  }
  
  // Publish as degrees
  if (pub_heading_deg_ && pub_heading_deg_->is_activated()) {
    auto msg = std::make_unique<std_msgs::msg::Float64>();
    msg->data = gnss_data_.heading.heading_deg;
    pub_heading_deg_->publish(std::move(msg));
  }
}

void GnssCompassDriverNode::publish_velocity()
{
  if (!pub_velocity_ || !pub_velocity_->is_activated()) return;
  
  auto msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
  
  msg->header.stamp = now();
  msg->header.frame_id = params_.frame_id;
  
  // Linear velocity in ENU frame
  msg->twist.linear.x = gnss_data_.velocity.vel_east_mps;
  msg->twist.linear.y = gnss_data_.velocity.vel_north_mps;
  msg->twist.linear.z = -gnss_data_.velocity.vel_down_mps;  // Convert to up
  
  // Angular velocity not available from GNSS
  msg->twist.angular.x = 0.0;
  msg->twist.angular.y = 0.0;
  msg->twist.angular.z = 0.0;
  
  pub_velocity_->publish(std::move(msg));
}

void GnssCompassDriverNode::publish_time_reference()
{
  if (!pub_time_ref_ || !pub_time_ref_->is_activated()) return;
  
  if (!gnss_data_.time.time_valid) return;
  
  auto msg = std::make_unique<sensor_msgs::msg::TimeReference>();
  
  msg->header.stamp = now();
  msg->header.frame_id = params_.frame_id;
  
  // Convert GPS time to ROS time
  // This is a simplified conversion - proper implementation would use GPS week/TOW
  struct tm time_info = {};
  time_info.tm_year = gnss_data_.time.year - 1900;
  time_info.tm_mon = gnss_data_.time.month - 1;
  time_info.tm_mday = gnss_data_.time.day;
  time_info.tm_hour = gnss_data_.time.hour;
  time_info.tm_min = gnss_data_.time.minute;
  time_info.tm_sec = static_cast<int>(gnss_data_.time.second);
  
  time_t unix_time = timegm(&time_info);
  double frac_sec = gnss_data_.time.second - static_cast<int>(gnss_data_.time.second);
  
  msg->time_ref.sec = static_cast<int32_t>(unix_time);
  msg->time_ref.nanosec = static_cast<uint32_t>(frac_sec * 1e9);
  
  msg->source = "gnss";
  
  pub_time_ref_->publish(std::move(msg));
}

void GnssCompassDriverNode::publish_diagnostics()
{
  if (!pub_diagnostics_ || !pub_diagnostics_->is_activated()) return;
  
  auto msg = std::make_unique<diagnostic_msgs::msg::DiagnosticArray>();
  msg->header.stamp = now();
  
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "ArduSimple GNSS";
  status.hardware_id = params_.port;
  
  // Determine overall status
  if (!is_connected_) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "Serial port disconnected";
  } else if (!gnss_data_.position.valid) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "No GNSS fix";
  } else if (gnss_data_.position.rtk_status == RtkStatus::FIXED) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "RTK Fixed";
  } else if (gnss_data_.position.rtk_status == RtkStatus::FLOAT) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "RTK Float";
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "GNSS " + std::string(to_string(gnss_data_.position.fix_type));
  }
  
  // Add key-value pairs
  auto add_kv = [&status](const std::string& key, const std::string& value) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    kv.value = value;
    status.values.push_back(kv);
  };
  
  std::lock_guard<std::mutex> lock(data_mutex_);
  
  add_kv("Fix Type", to_string(gnss_data_.position.fix_type));
  add_kv("RTK Status", to_string(gnss_data_.position.rtk_status));
  add_kv("Satellites", std::to_string(gnss_data_.position.satellites_used));
  
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(2);
  
  oss.str(""); oss << gnss_data_.position.hdop;
  add_kv("HDOP", oss.str());
  
  oss.str(""); oss << gnss_data_.position.horizontal_accuracy_m << " m";
  add_kv("Horizontal Accuracy", oss.str());
  
  oss.str(""); oss << gnss_data_.position.vertical_accuracy_m << " m";
  add_kv("Vertical Accuracy", oss.str());
  
  if (gnss_data_.heading.heading_valid) {
    oss.str(""); oss << gnss_data_.heading.heading_deg << "°";
    add_kv("Heading", oss.str());
    
    oss.str(""); oss << gnss_data_.heading.heading_accuracy_deg << "°";
    add_kv("Heading Accuracy", oss.str());
    
    oss.str(""); oss << gnss_data_.heading.baseline_length_m << " m";
    add_kv("Baseline Length", oss.str());
    
    add_kv("Baseline Fixed", gnss_data_.heading.baseline_fixed ? "Yes" : "No");
  }
  
  // Parser statistics
  auto nmea_stats = nmea_parser_->stats();
  auto ubx_stats = ubx_parser_->stats();
  auto unicore_stats = unicore_parser_->stats();
  
  add_kv("NMEA Sentences", std::to_string(nmea_stats.sentences_parsed));
  add_kv("UBX Messages", std::to_string(ubx_stats.messages_parsed));
  add_kv("UniCore Messages", std::to_string(unicore_stats.messages_parsed));
  add_kv("Checksum Errors", std::to_string(
    nmea_stats.checksum_errors + ubx_stats.checksum_errors + unicore_stats.checksum_errors));
  
  msg->status.push_back(status);
  pub_diagnostics_->publish(std::move(msg));
}

// =============================================================================
// Helper Methods
// =============================================================================

sensor_msgs::msg::NavSatStatus GnssCompassDriverNode::create_nav_sat_status() const
{
  sensor_msgs::msg::NavSatStatus status;
  
  if (!gnss_data_.position.valid) {
    status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
  } else if (gnss_data_.position.rtk_status == RtkStatus::FIXED) {
    status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
  } else if (gnss_data_.position.rtk_status == RtkStatus::FLOAT) {
    status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;  // Closest match
  } else {
    status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  }
  
  // Service flags - assume GPS at minimum
  status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS |
                   sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS |
                   sensor_msgs::msg::NavSatStatus::SERVICE_GALILEO;
  
  return status;
}

void GnssCompassDriverNode::update_covariance()
{
  double h_var = gnss_data_.position.horizontal_accuracy_m * 
                 gnss_data_.position.horizontal_accuracy_m;
  double v_var = gnss_data_.position.vertical_accuracy_m * 
                 gnss_data_.position.vertical_accuracy_m;
  
  // Diagonal covariance matrix (ENU)
  gnss_data_.position.position_covariance[0] = h_var;  // East
  gnss_data_.position.position_covariance[4] = h_var;  // North
  gnss_data_.position.position_covariance[8] = v_var;  // Up
}

void GnssCompassDriverNode::on_rtcm_received(
    const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
{
  if (!serial_port_ || !serial_port_->is_open()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "RTCM data received but serial port is not open");
    return;
  }
  
  if (msg->data.empty()) {
    return;
  }
  
  // Write RTCM data to GNSS receiver
  ssize_t bytes_written = serial_port_->write(msg->data.data(), msg->data.size());
  
  if (bytes_written > 0) {
    RCLCPP_DEBUG(get_logger(), "Sent %zd bytes of RTCM data to receiver",
      bytes_written);
  } else {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "Failed to write RTCM data to receiver");
  }
}

}  // namespace gnss_compass

// Register component
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(gnss_compass::GnssCompassDriverNode)

