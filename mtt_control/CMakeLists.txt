cmake_minimum_required(VERSION 3.22)
project(mtt_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(mtt_msgs REQUIRED)
find_package(mtt_interfaces REQUIRED)

add_library(mtt_control_core SHARED
  src/common/joystick_state.cpp
)
target_include_directories(mtt_control_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mtt_control_core
  rclcpp
  sensor_msgs
)

add_executable(mtt_operator_input_node src/nodes/mtt_operator_input_node.cpp)
target_link_libraries(mtt_operator_input_node mtt_control_core)
target_include_directories(mtt_operator_input_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mtt_operator_input_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  mtt_msgs
  mtt_interfaces
)

add_executable(mtt_manual_cmd_filter_node src/nodes/mtt_manual_cmd_filter_node.cpp)
target_link_libraries(mtt_manual_cmd_filter_node mtt_control_core)
target_include_directories(mtt_manual_cmd_filter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mtt_manual_cmd_filter_node
  rclcpp
  geometry_msgs
  std_msgs
)

add_executable(mtt_mode_manager_node src/nodes/mtt_mode_manager_node.cpp)
target_link_libraries(mtt_mode_manager_node mtt_control_core)
target_include_directories(mtt_mode_manager_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mtt_mode_manager_node
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
)

add_executable(mtt_cmd_arbiter_node src/nodes/mtt_cmd_arbiter_node.cpp)
target_link_libraries(mtt_cmd_arbiter_node mtt_control_core)
target_include_directories(mtt_cmd_arbiter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mtt_cmd_arbiter_node
  rclcpp
  geometry_msgs
  std_msgs
  mtt_interfaces
)

add_executable(mtt_autonomy_mux_node src/nodes/mtt_autonomy_mux_node.cpp)
target_include_directories(mtt_autonomy_mux_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mtt_autonomy_mux_node
  rclcpp
  geometry_msgs
  std_msgs
)

add_executable(mtt_cmd_vel_bridge_node src/nodes/mtt_cmd_vel_bridge_node.cpp)
target_include_directories(mtt_cmd_vel_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mtt_cmd_vel_bridge_node
  rclcpp
  geometry_msgs
)

add_executable(
  mtt_articulated_nav_adapter_node
  src/nodes/mtt_articulated_nav_adapter_node.cpp
)
target_include_directories(mtt_articulated_nav_adapter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mtt_articulated_nav_adapter_node
  rclcpp
  geometry_msgs
  std_msgs
)

add_executable(
  mtt_articulated_command_dispatcher_node
  src/nodes/mtt_articulated_command_dispatcher_node.cpp
)
target_include_directories(mtt_articulated_command_dispatcher_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mtt_articulated_command_dispatcher_node
  rclcpp
  geometry_msgs
  std_msgs
)

add_executable(mtt_ice_slip_detector_node src/nodes/mtt_ice_slip_detector_node.cpp)
target_include_directories(mtt_ice_slip_detector_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mtt_ice_slip_detector_node
  rclcpp
  geometry_msgs
  mtt_msgs
  nav_msgs
  sensor_msgs
  std_msgs
)

# --- COM switch variants (button-triggered / speed-threshold-triggered) ---
add_executable(mtt_com_button_switch_node src/nodes/mtt_com_button_switch_node.cpp)
target_link_libraries(mtt_com_button_switch_node mtt_control_core)
target_include_directories(mtt_com_button_switch_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mtt_com_button_switch_node
  rclcpp
  sensor_msgs
  std_msgs
)

add_executable(mtt_com_speed_switch_node src/nodes/mtt_com_speed_switch_node.cpp)
target_include_directories(mtt_com_speed_switch_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mtt_com_speed_switch_node
  rclcpp
  std_msgs
)
# --- end COM switch variants ---

install(TARGETS
  mtt_control_core
  EXPORT mtt_control_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  mtt_operator_input_node
  mtt_manual_cmd_filter_node
  mtt_mode_manager_node
  mtt_cmd_arbiter_node
  mtt_autonomy_mux_node
  mtt_cmd_vel_bridge_node
  mtt_articulated_nav_adapter_node
  mtt_articulated_command_dispatcher_node
  mtt_ice_slip_detector_node
  # COM switch variants added alongside mtt_ice_slip_detector_node
  mtt_com_button_switch_node
  mtt_com_speed_switch_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

# Export include/ and the core library so other packages (e.g.
# mtt_gps_teach_repeat) can reuse logic::ArticulatedCommandModel via
# find_package(mtt_control) instead of duplicating it.
ament_export_include_directories(include)
ament_export_libraries(mtt_control_core)
ament_export_targets(mtt_control_targets)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  std_srvs
  mtt_msgs
  mtt_interfaces
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(
    test_articulated_command_model
    test/test_articulated_command_model.cpp
  )
  if(TARGET test_articulated_command_model)
    target_include_directories(test_articulated_command_model PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
  endif()
endif()

ament_package()
