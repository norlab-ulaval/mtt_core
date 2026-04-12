cmake_minimum_required(VERSION 3.8)
project(mtt_msgs)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MttAuxCommand.msg"
  "msg/MttTachometerData.msg"
  "msg/MttVehicleStatus.msg"
  "msg/MttDrivingMode.msg"
  DEPENDENCIES std_msgs
)

ament_package()
