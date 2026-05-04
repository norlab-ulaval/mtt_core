#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    control_share = FindPackageShare("mtt_control").find("mtt_control")
    control_launch = os.path.join(control_share, "launch", "mtt_operator_control.launch.py")
    control_params = os.path.join(control_share, "config", "control_defaults.yaml")

    arguments = [
        DeclareLaunchArgument(
            "robot_namespace",
            default_value="",
            description="Top-level namespace for the operator teleop stack.",
        ),
        DeclareLaunchArgument(
            "use_namespace",
            default_value="false",
            description="Whether to apply robot_namespace to the teleop stack.",
        ),
        DeclareLaunchArgument(
            "joy_device",
            default_value="/dev/input/js0",
            description="Joystick device on the operator computer.",
        ),
        DeclareLaunchArgument(
            "joy_deadzone",
            default_value="0.15",
            description="Joystick deadzone for joy_linux.",
        ),
        DeclareLaunchArgument(
            "control_params_file",
            default_value=control_params,
            description="Operator control parameter file.",
        ),
    ]

    include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(control_launch),
        launch_arguments={
            "robot_namespace": LaunchConfiguration("robot_namespace"),
            "use_namespace": LaunchConfiguration("use_namespace"),
            "joy_device": LaunchConfiguration("joy_device"),
            "joy_deadzone": LaunchConfiguration("joy_deadzone"),
            "control_params_file": LaunchConfiguration("control_params_file"),
        }.items(),
    )

    return LaunchDescription(arguments + [include])
