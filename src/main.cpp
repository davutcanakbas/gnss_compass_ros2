// Copyright (c) 2026 Davut Can Akbas
// SPDX-License-Identifier: MIT

#include "gnss_compass_driver/gnss_compass_driver_node.hpp"

#include <rclcpp/rclcpp.hpp>
#include <lifecycle_msgs/msg/transition.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  rclcpp::executors::SingleThreadedExecutor executor;
  
  auto node = std::make_shared<gnss_compass::GnssCompassDriverNode>(rclcpp::NodeOptions());
  
  executor.add_node(node->get_node_base_interface());
  
  // Automatically transition to active state
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}

