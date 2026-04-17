cmake_minimum_required(VERSION 3.14)
project(mtt_localization)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(GTSAM REQUIRED)

include_directories(include ${GTSAM_INCLUDE_DIR})

add_executable(factor_graph_node
  src/factor_graph_node.cpp
)
ament_target_dependencies(factor_graph_node
  rclcpp sensor_msgs geometry_msgs nav_msgs std_msgs
  tf2 tf2_ros tf2_geometry_msgs
)
target_link_libraries(factor_graph_node gtsam)

install(TARGETS factor_graph_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
