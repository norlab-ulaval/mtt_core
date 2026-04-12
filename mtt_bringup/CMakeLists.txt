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


install(
  DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)


install(PROGRAMS
  ${PROJECT_NAME}/odom_publisher_simul.py
  ${PROJECT_NAME}/mtt_controller_interface.py
  DESTINATION lib/${PROJECT_NAME})
ament_package()