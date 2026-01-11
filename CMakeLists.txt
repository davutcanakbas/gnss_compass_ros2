cmake_minimum_required(VERSION 3.14)
project(gnss_compass_driver VERSION 1.0.0 LANGUAGES C CXX)

# ==============================================================================
# Compiler Settings
# ==============================================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# ==============================================================================
# Dependencies
# ==============================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Threads for async serial handling
find_package(Threads REQUIRED)

# ==============================================================================
# Custom Messages
# ==============================================================================
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GnssStatus.msg"
  "msg/Heading.msg"
  DEPENDENCIES std_msgs builtin_interfaces
)

# ==============================================================================
# Library: ardusimple_core (protocol parsers, serial handler)
# ==============================================================================
add_library(ardusimple_core SHARED
  src/serial/serial_port.cpp
  src/protocol/nmea_parser.cpp
  src/protocol/ubx_parser.cpp
  src/protocol/ubx_messages.cpp
  src/protocol/unicore_parser.cpp
)

target_include_directories(ardusimple_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(ardusimple_core
  Threads::Threads
)

ament_target_dependencies(ardusimple_core
  rclcpp
)

# ==============================================================================
# Library: ardusimple_node (ROS2 node implementation)
# ==============================================================================
add_library(ardusimple_node SHARED
  src/gnss_compass_driver_node.cpp
)

target_include_directories(ardusimple_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(ardusimple_node
  ardusimple_core
)

ament_target_dependencies(ardusimple_node
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  std_msgs
  sensor_msgs
  geometry_msgs
  diagnostic_msgs
  tf2
  tf2_geometry_msgs
)

# Link generated messages
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(ardusimple_node ${cpp_typesupport_target})

# Register as component
rclcpp_components_register_node(ardusimple_node
  PLUGIN "gnss_compass::GnssCompassDriverNode"
  EXECUTABLE gnss_compass_driver_node
)

# ==============================================================================
# Standalone Executable
# ==============================================================================
add_executable(gnss_compass_driver_exec
  src/main.cpp
)

target_link_libraries(gnss_compass_driver_exec
  ardusimple_node
)

ament_target_dependencies(gnss_compass_driver_exec
  rclcpp
)

# ==============================================================================
# NTRIP Client Executable
# ==============================================================================
add_executable(ntrip_client
  src/ntrip_client_node.cpp
)

ament_target_dependencies(ntrip_client
  rclcpp
  std_msgs
)

# ==============================================================================
# Installation
# ==============================================================================
install(TARGETS
  ardusimple_core
  ardusimple_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Set output name for executable
set_target_properties(gnss_compass_driver_exec PROPERTIES
  OUTPUT_NAME gnss_compass_driver
)

install(TARGETS gnss_compass_driver_exec ntrip_client
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  config
  scripts
  DESTINATION share/${PROJECT_NAME}
)

# Make scripts executable
install(PROGRAMS
  scripts/find_rtk2go_stations.sh
  DESTINATION lib/${PROJECT_NAME}
)

# ==============================================================================
# Testing
# ==============================================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  
  ament_lint_auto_find_test_dependencies()
  
  # NMEA Parser tests
  ament_add_gtest(test_nmea_parser
    test/test_nmea_parser.cpp
  )
  target_link_libraries(test_nmea_parser ardusimple_core)
  
  # UBX Parser tests
  ament_add_gtest(test_ubx_parser
    test/test_ubx_parser.cpp
  )
  target_link_libraries(test_ubx_parser ardusimple_core)
endif()

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install scripts
install(PROGRAMS scripts/find_rtk2go_stations.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(ardusimple_core ardusimple_node)
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  geometry_msgs
  diagnostic_msgs
)

ament_package()

