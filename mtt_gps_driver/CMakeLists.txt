cmake_minimum_required(VERSION 3.14)
project(mtt_gps_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

# --- Reach Driver Node (TCP or USB Serial, configurable at runtime) ---
add_executable(reach_driver_node
  src/reach_driver_node.cpp
  src/nmea_parser.cpp
)
ament_target_dependencies(reach_driver_node
  rclcpp sensor_msgs geometry_msgs std_msgs nmea_msgs geographic_msgs tf2 tf2_ros
)

# --- Dual Antenna Heading Node ---
add_executable(dual_antenna_heading_node
  src/dual_antenna_heading_node.cpp
)
ament_target_dependencies(dual_antenna_heading_node
  rclcpp sensor_msgs geometry_msgs std_msgs tf2 tf2_ros
)

install(TARGETS
  reach_driver_node
  dual_antenna_heading_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
