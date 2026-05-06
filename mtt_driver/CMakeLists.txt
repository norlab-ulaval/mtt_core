cmake_minimum_required(VERSION 3.22)
project(mtt_driver)

# ── C++ standard ──────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Strict warnings in debug builds
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion)
endif()

# ── ROS 2 dependencies ────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(mtt_msgs REQUIRED)
find_package(mtt_interfaces REQUIRED)
find_package(std_srvs REQUIRED)

# ────────────────────────────────────────────────────────────────────
# LIBRARY 1: mtt_logic  (pure C++17 — NO ROS, fully unit-testable)
# Layers: vehicle_params.hpp (header-only), can_frame_codec.hpp (header-only),
#         tachometer.hpp (header-only), articulated_model, odometry_calculator
# ────────────────────────────────────────────────────────────────────
add_library(mtt_logic SHARED
  src/logic/command_motion_model.cpp
  src/logic/articulated_model.cpp
  src/logic/odometry_calculator.cpp
)
target_include_directories(mtt_logic PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# No ROS deps — intentionally pure C++17

# ────────────────────────────────────────────────────────────────────
# LIBRARY 2: mtt_hardware  (Linux SocketCAN)
# ────────────────────────────────────────────────────────────────────
add_library(mtt_hardware SHARED
  src/hardware/linux_socket_can.cpp
)
target_include_directories(mtt_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# No extra deps — uses linux/can.h from kernel headers (always present on ROS robot)

# ────────────────────────────────────────────────────────────────────
# COMPONENT: mtt_can_node  (ROS 2 node — CAN driver + publisher)
# ────────────────────────────────────────────────────────────────────
add_library(mtt_can_node SHARED
  src/components/mtt_can_node.cpp
)
target_include_directories(mtt_can_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(mtt_can_node mtt_logic mtt_hardware)
ament_target_dependencies(mtt_can_node
  rclcpp
  rclcpp_components
  std_msgs
  geometry_msgs
  sensor_msgs
  mtt_msgs
  mtt_interfaces
)
rclcpp_components_register_node(mtt_can_node
  PLUGIN "mtt::MttCanNode"
  EXECUTABLE mtt_can_node_exe
)

# ────────────────────────────────────────────────────────────────────
# COMPONENT: mtt_odometry_node  (ROS 2 node — odometry + TF)
# ────────────────────────────────────────────────────────────────────
add_library(mtt_odometry_node SHARED
  src/components/mtt_odometry_node.cpp
)
target_include_directories(mtt_odometry_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(mtt_odometry_node mtt_logic)
ament_target_dependencies(mtt_odometry_node
  rclcpp
  rclcpp_components
  nav_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf2_ros
  mtt_msgs
  mtt_interfaces
)
rclcpp_components_register_node(mtt_odometry_node
  PLUGIN "mtt::MttOdometryNode"
  EXECUTABLE mtt_odometry_node_exe
)

# ────────────────────────────────────────────────────────────────────
# COMPONENT: mtt_health_monitor_node  (ROS 2 node — health + fallback)
# ────────────────────────────────────────────────────────────────────
add_library(mtt_health_monitor_node SHARED
  src/components/mtt_health_monitor_node.cpp
)
target_include_directories(mtt_health_monitor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(mtt_health_monitor_node mtt_logic)
ament_target_dependencies(mtt_health_monitor_node
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs
  mtt_msgs
)
rclcpp_components_register_node(mtt_health_monitor_node
  PLUGIN "mtt::MttHealthMonitorNode"
  EXECUTABLE mtt_health_monitor_node_exe
)

# ────────────────────────────────────────────────────────────────────
# COMPONENT: mtt_teleop_joy_node (ROS 2 node — Teleop Joystick)
# ────────────────────────────────────────────────────────────────────
add_library(mtt_teleop_joy_node SHARED
  src/components/mtt_teleop_joy_node.cpp
)
target_include_directories(mtt_teleop_joy_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mtt_teleop_joy_node
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  std_msgs
  mtt_msgs
)
rclcpp_components_register_node(mtt_teleop_joy_node
  PLUGIN "mtt::MttTeleopJoyNode"
  EXECUTABLE mtt_teleop_joy
)

# ────────────────────────────────────────────────────────────────────
# COMPONENT: teleop_cmd_smoother_node (ROS 2 node — Command Smoother)
# ────────────────────────────────────────────────────────────────────
add_library(teleop_cmd_smoother_node SHARED
  src/components/teleop_cmd_smoother_node.cpp
)
target_include_directories(teleop_cmd_smoother_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(teleop_cmd_smoother_node
  rclcpp
  rclcpp_components
  geometry_msgs
)
rclcpp_components_register_node(teleop_cmd_smoother_node
  PLUGIN "mtt::TeleopCmdSmootherNode"
  EXECUTABLE teleop_cmd_smoother_node_exe
)

# ────────────────────────────────────────────────────────────────────
# COMPONENT: mtt_synthetic_tachometer_node (ROS 2 node — Simulation mode)
# Converts cmd_vel to synthetic tachometer data for unified real/sim testing
# ────────────────────────────────────────────────────────────────────
add_library(mtt_synthetic_tachometer_node SHARED
  src/components/mtt_synthetic_tachometer_node.cpp
)
target_include_directories(mtt_synthetic_tachometer_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(mtt_synthetic_tachometer_node mtt_logic)
ament_target_dependencies(mtt_synthetic_tachometer_node
  rclcpp
  rclcpp_components
  geometry_msgs
  mtt_msgs
)
rclcpp_components_register_node(mtt_synthetic_tachometer_node
  PLUGIN "mtt::MttSyntheticTachometerNode"
  EXECUTABLE mtt_synthetic_tachometer
)

# ────────────────────────────────────────────────────────────────────
# COMPONENT: mtt_joint_state_builder_node (ROS 2 node — shared articulation view)
# Builds a consistent joint-state stream for replay, live runtime, and simulation.
# ────────────────────────────────────────────────────────────────────
add_library(mtt_joint_state_builder_node SHARED
  src/components/mtt_joint_state_builder_node.cpp
)
target_include_directories(mtt_joint_state_builder_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mtt_joint_state_builder_node
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  mtt_msgs
)
rclcpp_components_register_node(mtt_joint_state_builder_node
  PLUGIN "mtt::MttJointStateBuilderNode"
  EXECUTABLE mtt_joint_state_builder_node_exe
)

# ────────────────────────────────────────────────────────────────────
# Install
# ────────────────────────────────────────────────────────────────────
install(TARGETS mtt_logic mtt_hardware mtt_can_node mtt_odometry_node mtt_health_monitor_node mtt_teleop_joy_node teleop_cmd_smoother_node mtt_synthetic_tachometer_node mtt_joint_state_builder_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../scripts/mtt_health_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

# ────────────────────────────────────────────────────────────────────
# Tests (GTest — no ROS required)
# ────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_can_frame_codec test/test_can_frame_codec.cpp)
  target_link_libraries(test_can_frame_codec mtt_logic)

  ament_add_gtest(test_tachometer test/test_tachometer.cpp)
  target_link_libraries(test_tachometer mtt_logic)

  # Odometry test added in Step 5
endif()

ament_package()
