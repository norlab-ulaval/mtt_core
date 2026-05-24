#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushROSNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("mtt_control").find("mtt_control")
    default_params = os.path.join(package_share, "config", "control_defaults.yaml")
    robot_namespace = LaunchConfiguration("robot_namespace")
    use_namespace = LaunchConfiguration("use_namespace")

    return LaunchDescription([
        DeclareLaunchArgument("robot_namespace", default_value=""),
        DeclareLaunchArgument("use_namespace", default_value="false"),
        DeclareLaunchArgument("joy_device", default_value="/dev/input/js0"),
        DeclareLaunchArgument("joy_deadzone", default_value="0.15"),
        DeclareLaunchArgument("control_params_file", default_value=default_params),
        GroupAction(actions=[
            PushROSNamespace(condition=IfCondition(use_namespace), namespace=robot_namespace),
            Node(
                package="joy_linux",
                executable="joy_linux_node",
                name="joy_node",
                parameters=[{
                    "deadzone": LaunchConfiguration("joy_deadzone"),
                    "dev": LaunchConfiguration("joy_device"),
                    "autorepeat_rate": 20.0,
                }],
                output="screen",
            ),
            Node(
                package="mtt_control",
                executable="mtt_operator_input_node",
                name="mtt_operator_input_node",
                parameters=[LaunchConfiguration("control_params_file")],
                output="screen",
            ),
            Node(
                package="mtt_control",
                executable="mtt_manual_cmd_filter_node",
                name="mtt_manual_cmd_filter_node",
                parameters=[LaunchConfiguration("control_params_file")],
                output="screen",
            ),
            Node(
                package="mtt_control",
                executable="mtt_mode_manager_node",
                name="mtt_mode_manager_node",
                parameters=[LaunchConfiguration("control_params_file")],
                output="screen",
            ),
        ]),
    ])
