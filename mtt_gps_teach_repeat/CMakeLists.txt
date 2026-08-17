cmake_minimum_required(VERSION 3.22)
project(mtt_gps_teach_repeat)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(mtt_control REQUIRED)
find_package(mtt_interfaces REQUIRED)
find_package(romea_core_common REQUIRED)
find_package(romea_core_path REQUIRED)
find_package(romea_core_path_matching REQUIRED)
find_package(nlohmann_json REQUIRED)

add_library(mtt_gps_teach_repeat_core SHARED
  src/logic/gps_waypoint_recorder.cpp
  src/logic/articulated_gps_controller.cpp
)
target_include_directories(mtt_gps_teach_repeat_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(mtt_gps_teach_repeat_core
  nlohmann_json::nlohmann_json
)
ament_target_dependencies(mtt_gps_teach_repeat_core
  mtt_control
)

add_executable(mtt_gps_recorder_node src/nodes/mtt_gps_recorder_node.cpp)
target_link_libraries(mtt_gps_recorder_node mtt_gps_teach_repeat_core)
target_include_directories(mtt_gps_recorder_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mtt_gps_recorder_node
  rclcpp
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
)

add_executable(mtt_gps_path_server_node src/nodes/mtt_gps_path_server_node.cpp)
target_link_libraries(mtt_gps_path_server_node
  romea_core_common::romea_core_common
  romea_core_path::romea_core_path
  romea_core_path_matching::romea_core_path_matching
)
target_include_directories(mtt_gps_path_server_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mtt_gps_path_server_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  std_srvs
  mtt_interfaces
)

add_executable(mtt_gps_path_follower_node src/nodes/mtt_gps_path_follower_node.cpp)
target_link_libraries(mtt_gps_path_follower_node mtt_gps_teach_repeat_core)
target_include_directories(mtt_gps_path_follower_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mtt_gps_path_follower_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  std_srvs
)

install(TARGETS
  mtt_gps_teach_repeat_core
  mtt_gps_recorder_node
  mtt_gps_path_server_node
  mtt_gps_path_follower_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(
    test_gps_teach_repeat_logic
    test/test_gps_teach_repeat_logic.cpp
  )
  if(TARGET test_gps_teach_repeat_logic)
    target_link_libraries(test_gps_teach_repeat_logic mtt_gps_teach_repeat_core)
    target_include_directories(test_gps_teach_repeat_logic PRIVATE include)
  endif()
endif()

ament_package()
