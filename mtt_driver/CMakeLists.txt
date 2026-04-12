cmake_minimum_required(VERSION 3.8)
project(mtt_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(joy REQUIRED)


# Python
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/mtt_driver.py
  ${PROJECT_NAME}/mtt_vehicle_params.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
