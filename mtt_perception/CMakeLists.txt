cmake_minimum_required(VERSION 3.14)
project(mtt_perception)

# NOTE: C++20 is applied only to mtt_trailer_estimator_node (see target below).
# Do NOT set CMAKE_CXX_STANDARD globally — PCL-based nodes (trailer_detector_node,
# trailer_pose_node) use PCL headers that are not C++20-compatible and crash at
# runtime if compiled with -ffinite-math-only (part of -ffast-math in C++20 builds).

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(message_filters REQUIRED)
find_package(mtt_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(pcl_conversions REQUIRED)

include_directories(include ${PCL_INCLUDE_DIRS})

# ── Trailer Detector Node (V1.5 — adaptive LiDAR/STM articulation fusion) ───
add_executable(trailer_detector_node
  src/trailer_detector_node.cpp
)
ament_target_dependencies(trailer_detector_node
  rclcpp sensor_msgs geometry_msgs std_msgs visualization_msgs
  tf2 tf2_ros pcl_conversions
)
target_link_libraries(trailer_detector_node
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)

# ── Cloud Merger Node (dual-LiDAR fusion: Hesai + RS-Airy → base_link) ───────
add_executable(cloud_merger_node
  src/cloud_merger_node.cpp
)
ament_target_dependencies(cloud_merger_node
  rclcpp sensor_msgs geometry_msgs tf2 tf2_ros tf2_sensor_msgs
)

# ── Trailer Pose Node (V4.0 — kept compilable for A/B comparison) ────────────
add_executable(trailer_pose_node
  src/trailer_pose_node.cpp
)
ament_target_dependencies(trailer_pose_node
  rclcpp sensor_msgs geometry_msgs std_msgs visualization_msgs
  tf2 tf2_ros tf2_geometry_msgs pcl_conversions
)
target_link_libraries(trailer_pose_node
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)

# ── MTT Trailer Estimator Node (V1.0 — unified EKF, replaces trailer_pose_node) ──
add_executable(mtt_trailer_estimator_node
  src/mtt_trailer_estimator_node.cpp
  src/mtt_trailer_estimator_main.cpp
)
ament_target_dependencies(mtt_trailer_estimator_node
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  mtt_msgs
  pcl_conversions
)
target_link_libraries(mtt_trailer_estimator_node
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)
# C++20 and optimisation flags scoped to the estimator only.
# -ffast-math is intentionally EXCLUDED: it disables IEEE 754 NaN semantics
# which breaks isfinite() checks used in the point-cloud pipeline.
target_compile_features(mtt_trailer_estimator_node PUBLIC cxx_std_20)
target_compile_options(mtt_trailer_estimator_node PRIVATE
  $<$<CONFIG:Release>:-O3 -march=native>
  $<$<CONFIG:RelWithDebInfo>:-O3 -march=native>
)
target_compile_definitions(mtt_trailer_estimator_node PRIVATE
  EIGEN_NO_AUTOMATIC_RESIZING
)

# ── Install ────────────────────────────────────────────────────────────────
install(TARGETS
  trailer_detector_node
  cloud_merger_node
  trailer_pose_node
  mtt_trailer_estimator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

# ── Tests ──────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(mtt_trailer_estimator_test
    test/mtt_trailer_estimator_test.cpp
  )
  target_include_directories(mtt_trailer_estimator_test PUBLIC include)
  target_link_libraries(mtt_trailer_estimator_test Eigen3::Eigen)

  ament_add_gtest(adaptive_sensor_confidence_test
    test/adaptive_sensor_confidence_test.cpp
  )
  target_include_directories(adaptive_sensor_confidence_test PUBLIC include)
endif()

ament_package()
