#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gps_share = get_package_share_directory("mtt_gps_teach_repeat")
    localization_share = get_package_share_directory("mtt_localization")

    recorder_default = os.path.join(gps_share, "config", "gps_recorder.yaml")
    server_default = os.path.join(gps_share, "config", "gps_path_server.yaml")
    follower_default = os.path.join(gps_share, "config", "gps_path_follower.yaml")
    obstacle_default = os.path.join(
        get_package_share_directory("mtt_bringup"),
        "config", "mtt_front_obstacle_monitor.yaml"
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(localization_share, "launch", "localization.launch.py")
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "use_gps": LaunchConfiguration("use_gps"),
            # A single front antenna gives position, not dual-antenna heading.
            "use_gps_heading": LaunchConfiguration("use_gps_heading"),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_gps", default_value="true"),
        DeclareLaunchArgument("use_gps_heading", default_value="false"),
        DeclareLaunchArgument("gps_recorder_params_file", default_value=recorder_default),
        DeclareLaunchArgument("gps_path_server_params_file", default_value=server_default),
        DeclareLaunchArgument("gps_path_follower_params_file", default_value=follower_default),
        DeclareLaunchArgument("front_obstacle_params_file", default_value=obstacle_default),
        localization,
        Node(
            package="mtt_bringup",
            # mtt_bringup is an ament_cmake package and installs this Python
            # program with its .py suffix (see mtt_bringup/CMakeLists.txt).
            executable="mtt_front_obstacle_monitor.py",
            # Keep the configured node name so the ROS parameter file applies.
            name="mtt_front_obstacle_monitor",
            output="screen",
            parameters=[LaunchConfiguration("front_obstacle_params_file")],
        ),
        Node(
            package="mtt_gps_teach_repeat",
            executable="mtt_gps_recorder_node",
            name="mtt_gps_recorder_node",
            output="screen",
            parameters=[LaunchConfiguration("gps_recorder_params_file")],
        ),
        Node(
            package="mtt_gps_teach_repeat",
            executable="mtt_gps_path_server_node",
            name="mtt_gps_path_server_node",
            output="screen",
            parameters=[LaunchConfiguration("gps_path_server_params_file")],
        ),
        Node(
            package="mtt_gps_teach_repeat",
            executable="mtt_gps_path_follower_node",
            name="mtt_gps_path_follower_node",
            output="screen",
            parameters=[LaunchConfiguration("gps_path_follower_params_file")],
        ),
    ])
