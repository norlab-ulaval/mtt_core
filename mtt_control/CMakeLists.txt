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
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(mtt_msgs REQUIRED)

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
)

install(TARGETS
  mtt_control_core
  mtt_operator_input_node
  mtt_manual_cmd_filter_node
  mtt_mode_manager_node
  mtt_cmd_arbiter_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
