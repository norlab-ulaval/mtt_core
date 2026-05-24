#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    control_share = get_package_share_directory("mtt_control")
    control_params_path = os.path.join(control_share, "config", "control_defaults.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    control_params_file = LaunchConfiguration("control_params_file")
    enable_joystick = LaunchConfiguration("enable_joystick")
    joy_device = LaunchConfiguration("joy_device")
    joy_deadzone = LaunchConfiguration("joy_deadzone")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation or bag replay clock",
        ),
        DeclareLaunchArgument(
            "control_params_file",
            default_value=control_params_path,
            description="ROS parameter file for the MTT operator control stack",
        ),
        DeclareLaunchArgument(
            "enable_joystick",
            default_value="true",
            description="Enable joystick input node",
        ),
        DeclareLaunchArgument(
            "joy_device",
            default_value="/dev/input/js0",
            description="Joystick device path",
        ),
        DeclareLaunchArgument(
            "joy_deadzone",
            default_value="0.15",
            description="Joystick deadzone",
        ),
        Node(
            package="joy_linux",
            executable="joy_linux_node",
            name="joy_node",
            parameters=[{
                "use_sim_time": use_sim_time,
                "deadzone": joy_deadzone,
                "dev": joy_device,
                "autorepeat_rate": 20.0,
            }],
            output="screen",
            condition=IfCondition(enable_joystick),
            respawn=True,
        ),
        Node(
            package="mtt_control",
            executable="mtt_operator_input_node",
            name="mtt_operator_input_node",
            parameters=[
                control_params_file,
                {"use_sim_time": use_sim_time},
            ],
            output="screen",
            respawn=True,
            respawn_delay=2.0,
        ),
        Node(
            package="mtt_control",
            executable="mtt_manual_cmd_filter_node",
            name="mtt_manual_cmd_filter_node",
            parameters=[
                control_params_file,
                {"use_sim_time": use_sim_time},
            ],
            output="screen",
            respawn=True,
            respawn_delay=2.0,
        ),
        Node(
            package="mtt_control",
            executable="mtt_mode_manager_node",
            name="mtt_mode_manager_node",
            parameters=[
                control_params_file,
                {"use_sim_time": use_sim_time},
            ],
            output="screen",
            respawn=True,
            respawn_delay=2.0,
        ),
        Node(
            package="mtt_control",
            executable="mtt_cmd_arbiter_node",
            name="mtt_cmd_arbiter_node",
            parameters=[
                control_params_file,
                {"use_sim_time": use_sim_time},
            ],
            output="screen",
            respawn=True,
            respawn_delay=2.0,
        ),
    ])
