#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushROSNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Runs the GPS path server + follower together. This chain publishes on
    # the same AUTO command chain (/mtt_articulation_setpoint, controller/cmd_vel)
    # as WILN's wiln_path_follower — do not run both followers at once.
    package_share = FindPackageShare("mtt_gps_teach_repeat").find("mtt_gps_teach_repeat")
    default_server_params = os.path.join(package_share, "config", "gps_path_server.yaml")
    default_follower_params = os.path.join(package_share, "config", "gps_path_follower.yaml")
    robot_namespace = LaunchConfiguration("robot_namespace")
    use_namespace = LaunchConfiguration("use_namespace")

    return LaunchDescription([
        DeclareLaunchArgument("robot_namespace", default_value=""),
        DeclareLaunchArgument("use_namespace", default_value="false"),
        DeclareLaunchArgument("gps_path_server_params_file", default_value=default_server_params),
        DeclareLaunchArgument(
            "gps_path_follower_params_file", default_value=default_follower_params),
        GroupAction(actions=[
            PushROSNamespace(condition=IfCondition(use_namespace), namespace=robot_namespace),
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
        ]),
    ])
