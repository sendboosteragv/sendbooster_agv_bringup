cmake_minimum_required(VERSION 3.8)
project(sendbooster_agv_bringup)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(serial REQUIRED)

# Include directories
include_directories(include)

# Motor driver library (STATIC to link with serial)
add_library(motor_driver STATIC
  src/motor_driver.cpp
)
target_include_directories(motor_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(motor_driver
  serial
)

# Motor driver node executable
add_executable(motor_driver_node
  src/motor_driver_node.cpp
)
target_link_libraries(motor_driver_node
  motor_driver
)
ament_target_dependencies(motor_driver_node
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  diagnostic_msgs
  tf2
  tf2_ros
  serial
)

# Laser scan merger executable
add_executable(laser_scan_merger
  src/laser_scan_merger.cpp
)
ament_target_dependencies(laser_scan_merger
  rclcpp
  sensor_msgs
  message_filters
)

# Install targets
install(TARGETS
  motor_driver
  motor_driver_node
  laser_scan_merger
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(motor_driver)
ament_export_dependencies(
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  diagnostic_msgs
  tf2
  tf2_ros
  serial
)

ament_package()
