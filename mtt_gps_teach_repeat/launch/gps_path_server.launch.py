#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushROSNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("mtt_gps_teach_repeat").find("mtt_gps_teach_repeat")
    default_params = os.path.join(package_share, "config", "gps_path_server.yaml")
    robot_namespace = LaunchConfiguration("robot_namespace")
    use_namespace = LaunchConfiguration("use_namespace")

    return LaunchDescription([
        DeclareLaunchArgument("robot_namespace", default_value=""),
        DeclareLaunchArgument("use_namespace", default_value="false"),
        DeclareLaunchArgument("gps_path_server_params_file", default_value=default_params),
        GroupAction(actions=[
            PushROSNamespace(condition=IfCondition(use_namespace), namespace=robot_namespace),
            Node(
                package="mtt_gps_teach_repeat",
                executable="mtt_gps_path_server_node",
                name="mtt_gps_path_server_node",
                output="screen",
                parameters=[LaunchConfiguration("gps_path_server_params_file")],
            ),
        ]),
    ])
