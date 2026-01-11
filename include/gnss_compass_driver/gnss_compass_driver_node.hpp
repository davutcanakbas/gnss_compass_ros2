// Copyright (c) 2026 Davut Can Akbas
// SPDX-License-Identifier: MIT

#ifndef GNSS_COMPASS_DRIVER__GNSS_COMPASS_DRIVER_NODE_HPP_
#define GNSS_COMPASS_DRIVER__GNSS_COMPASS_DRIVER_NODE_HPP_

#include "gnss_compass_driver/types.hpp"
#include "gnss_compass_driver/serial/serial_port.hpp"
#include "gnss_compass_driver/protocol/nmea_parser.hpp"
#include "gnss_compass_driver/protocol/ubx_parser.hpp"
#include "gnss_compass_driver/protocol/unicore_parser.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <sensor_msgs/msg/time_reference.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include <memory>
#include <mutex>
#include <string>

namespace gnss_compass
{

/**
 * @brief ROS2 Lifecycle Node for ArduSimple simpleRTK3B Compass GNSS receiver
 * 
 * This node reads data from the GNSS receiver via serial port,
 * parses NMEA and UBX messages, and publishes ROS2 messages.
 */
class GnssCompassDriverNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit GnssCompassDriverNode(const rclcpp::NodeOptions& options);
  ~GnssCompassDriverNode() override;

  // Lifecycle callbacks
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  
  CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

private:
  // ==========================================================================
  // Parameter Declaration & Loading
  // ==========================================================================
  
  void declare_parameters();
  void load_parameters();

  // ==========================================================================
  // Serial Port Callbacks
  // ==========================================================================
  
  void on_serial_data(const uint8_t* data, size_t size);
  void on_serial_error(const std::string& error);
  void on_serial_connected(bool connected);

  // ==========================================================================
  // NMEA Message Handlers
  // ==========================================================================
  
  void on_nmea_gga(const NmeaGGA& msg);
  void on_nmea_rmc(const NmeaRMC& msg);
  void on_nmea_vtg(const NmeaVTG& msg);
  void on_nmea_gst(const NmeaGST& msg);
  void on_nmea_hdt(const NmeaHDT& msg);
  void on_nmea_ths(const NmeaTHS& msg);
  void on_nmea_zda(const NmeaZDA& msg);

  // ==========================================================================
  // UBX Message Handlers
  // ==========================================================================
  
  void on_ubx_nav_pvt(const ubx::NavPvt& msg);
  void on_ubx_nav_relposned(const ubx::NavRelPosNed& msg);
  void on_ubx_nav_status(const ubx::NavStatus& msg);
  void on_ubx_nav_dop(const ubx::NavDop& msg);
  void on_ubx_ack(uint8_t cls, uint8_t id, bool ack);

  // ==========================================================================
  // Publishing Methods
  // ==========================================================================
  
  void publish_nav_sat_fix();
  void publish_heading();
  void publish_velocity();
  void publish_time_reference();
  void publish_diagnostics();

  // ==========================================================================
  // Helper Methods
  // ==========================================================================
  
  void configure_receiver();
  sensor_msgs::msg::NavSatStatus create_nav_sat_status() const;
  void update_covariance();

  // ==========================================================================
  // Parameters
  // ==========================================================================
  
  struct Parameters
  {
    // Serial port
    std::string port{"/dev/ttyUSB0"};
    int baudrate{460800};
    
    // Frame IDs
    std::string frame_id{"gnss_link"};
    
    // Protocol settings
    bool use_ubx{true};
    bool use_nmea{true};
    bool use_unicore{true};
    
    // Output configuration
    bool publish_heading{true};
    bool publish_velocity{true};
    bool publish_time_ref{true};
    bool publish_diagnostics{true};
    
    // Antenna baseline (meters)
    double antenna_baseline{1.0};
    
    // Diagnostic rate
    double diagnostics_rate_hz{1.0};
    
    // Output rate (Hz) - max 20 Hz for dual-antenna, 50 Hz for single
    double output_rate_hz{20.0};
    
    // RTCM correction input
    std::string rtcm_topic{"rtcm"};
  };
  
  Parameters params_;

  // ==========================================================================
  // UniCore Message Handlers
  // ==========================================================================
  
  void on_unicore_heading(const UniCoreHeading& msg);
  void on_unicore_bestnav(const UniCoreBestNav& msg);

  // ==========================================================================
  // RTCM Correction Input
  // ==========================================================================
  
  void on_rtcm_received(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);

  // ==========================================================================
  // Serial Port & Parsers
  // ==========================================================================
  
  std::unique_ptr<SerialPort> serial_port_;
  std::unique_ptr<NmeaParser> nmea_parser_;
  std::unique_ptr<UbxParser> ubx_parser_;
  std::unique_ptr<UniCoreParser> unicore_parser_;

  // ==========================================================================
  // Current State (Protected by mutex)
  // ==========================================================================
  
  mutable std::mutex data_mutex_;
  GnssData gnss_data_;
  bool position_updated_{false};
  bool heading_updated_{false};
  bool velocity_updated_{false};
  bool time_updated_{false};

  // ==========================================================================
  // Publishers
  // ==========================================================================
  
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_fix_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr pub_heading_quat_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr pub_heading_deg_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_velocity_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::TimeReference>::SharedPtr pub_time_ref_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diagnostics_;

  // ==========================================================================
  // Subscriptions
  // ==========================================================================
  
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr sub_rtcm_;

  // ==========================================================================
  // Timers
  // ==========================================================================
  
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;

  // ==========================================================================
  // Connection State
  // ==========================================================================
  
  std::atomic<bool> is_connected_{false};
  TimePoint last_message_time_;
};

}  // namespace gnss_compass

#endif  // GNSS_COMPASS_DRIVER__GNSS_COMPASS_DRIVER_NODE_HPP_

