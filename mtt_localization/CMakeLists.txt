cmake_minimum_required(VERSION 3.14)
project(mtt_localization)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(mtt_msgs REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include ${GTSAM_INCLUDE_DIR})

add_executable(factor_graph_node
  src/factor_graph_node.cpp
)
ament_target_dependencies(factor_graph_node
  rclcpp sensor_msgs geometry_msgs nav_msgs std_msgs
  tf2 tf2_ros tf2_geometry_msgs mtt_msgs
)
target_link_libraries(factor_graph_node gtsam)

add_executable(offline_reference_solver
  src/offline_reference_solver.cpp
)
target_link_libraries(offline_reference_solver gtsam)

# ────────────────────────────────────────────────────────────────────
# COMPONENT: mtt_trailer_localizer_node
# Computes trailer pose in map frame via SE(3) kinematics from URDF chain.
# Subscribes: localization/odom + /mtt/articulation_state
# Publishes:  trailer/odom, trailer/pose_in_map, TF map→trailer_body
# ────────────────────────────────────────────────────────────────────
add_library(mtt_trailer_localizer_node SHARED
  src/trailer_localizer_node.cpp
)
target_include_directories(mtt_trailer_localizer_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(mtt_trailer_localizer_node Eigen3::Eigen)
ament_target_dependencies(mtt_trailer_localizer_node
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs
  tf2_ros
  mtt_msgs
)
rclcpp_components_register_node(mtt_trailer_localizer_node
  PLUGIN "mtt_loc::TrailerLocalizerNode"
  EXECUTABLE mtt_trailer_localizer_node_exe
)

install(TARGETS factor_graph_node offline_reference_solver mtt_trailer_localizer_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
