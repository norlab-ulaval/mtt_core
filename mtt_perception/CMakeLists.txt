cmake_minimum_required(VERSION 3.14)
project(mtt_perception)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters segmentation)
find_package(pcl_conversions REQUIRED)

include_directories(include ${PCL_INCLUDE_DIRS})

# --- Trailer Detector Node (LiDAR-based trailer angle estimation) ---
add_executable(trailer_detector_node
  src/trailer_detector_node.cpp
)
ament_target_dependencies(trailer_detector_node
  rclcpp sensor_msgs geometry_msgs std_msgs tf2 tf2_ros tf2_geometry_msgs pcl_conversions
)
target_link_libraries(trailer_detector_node ${PCL_LIBRARIES})

install(TARGETS
  trailer_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
