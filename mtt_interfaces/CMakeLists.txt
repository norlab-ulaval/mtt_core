cmake_minimum_required(VERSION 3.8)
project(mtt_interfaces)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AvoidanceStatus.msg"
  "srv/RouteCommand.srv"
  "srv/RouteList.srv"
  "srv/RouteStatus.srv"
  "srv/SetBrakeSrv.srv"
  "srv/SetDirectionModeSrv.srv"
  "srv/SetDirectionSrv.srv"
  "srv/GetVehiculeTypeSrv.srv"
  "srv/SetLightStateSrv.srv"
  "srv/SetSecuritySwitchSrv.srv"
  "srv/SetSteerControlMode.srv"
  "srv/SetSteerSrv.srv"
  "srv/SetThrottleSrv.srv"
  "srv/SetVehiculeTypeSrv.srv"
  "srv/SetWinchStateSrv.srv"
  "srv/RelocalizeRoute.srv"
  "srv/ValidateCompositePath.srv"
  DEPENDENCIES nav_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()



