cmake_minimum_required(VERSION 3.14)
project(mtt_perception)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(pcl_conversions REQUIRED)

include_directories(include ${PCL_INCLUDE_DIRS})

# --- Trailer Detector Node ---
add_executable(trailer_detector_node
  src/trailer_detector_node.cpp
)
ament_target_dependencies(trailer_detector_node
  rclcpp sensor_msgs geometry_msgs std_msgs visualization_msgs
  tf2 tf2_ros pcl_conversions
)
target_link_libraries(trailer_detector_node
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)

# --- Cloud Merger Node (dual-LiDAR fusion: Hesai + RS-Airy → base_link) ---
add_executable(cloud_merger_node
  src/cloud_merger_node.cpp
)
ament_target_dependencies(cloud_merger_node
  rclcpp sensor_msgs tf2 tf2_ros tf2_sensor_msgs message_filters pcl_conversions
)
target_link_libraries(cloud_merger_node ${PCL_LIBRARIES})

install(TARGETS
  trailer_detector_node
  cloud_merger_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
