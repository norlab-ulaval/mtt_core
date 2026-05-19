cmake_minimum_required(VERSION 3.8)
project(mtt_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Python
ament_python_install_package(${PROJECT_NAME})


# Install config (data files — copied once, rarely edited)
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# Install launch files as PROGRAMS → colcon --symlink-install creates symlinks,
# so edits in src/ are immediately visible without recompiling.
file(GLOB_RECURSE LAUNCH_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "launch/*.py")
install(PROGRAMS ${LAUNCH_FILES}
  DESTINATION share/${PROJECT_NAME}/launch
)



install(PROGRAMS
  ${PROJECT_NAME}/odom_publisher_simul.py
  ${PROJECT_NAME}/odom_to_pose_node.py
  ${PROJECT_NAME}/mtt_controller_interface.py
  ${PROJECT_NAME}/mtt_path_follower.py
  ${PROJECT_NAME}/mtt_repeat_supervisor.py
  ${PROJECT_NAME}/mtt_route_manager.py
  DESTINATION lib/${PROJECT_NAME})
ament_package()
