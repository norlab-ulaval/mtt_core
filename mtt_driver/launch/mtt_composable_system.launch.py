#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    driver_share = FindPackageShare("mtt_driver").find("mtt_driver")
    control_share = FindPackageShare("mtt_control").find("mtt_control")
    description_share = FindPackageShare("mtt_description").find("mtt_description")
    canonical_launch = os.path.join(driver_share, "launch", "mtt.launch.py")

    arguments = [
        DeclareLaunchArgument("setup_vcan", default_value="false"),
        DeclareLaunchArgument("setup_real_can", default_value="false"),
        DeclareLaunchArgument("robot_namespace", default_value=""),
        DeclareLaunchArgument("use_namespace", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("can_interface", default_value="can0"),
        DeclareLaunchArgument("can_bitrate", default_value="250000"),
        DeclareLaunchArgument("can_id", default_value="1"),
        DeclareLaunchArgument(
            "driver_params_file",
            default_value=os.path.join(driver_share, "config", "mtt_driver_params.yaml"),
        ),
        DeclareLaunchArgument(
            "health_params_file",
            default_value=os.path.join(driver_share, "config", "mtt_health_monitor.yaml"),
        ),
        DeclareLaunchArgument(
            "control_params_file",
            default_value=os.path.join(control_share, "config", "control_defaults.yaml"),
        ),
        DeclareLaunchArgument("driver_log_level", default_value="INFO"),
        DeclareLaunchArgument("control_frequency_hz", default_value="50.0"),
        DeclareLaunchArgument("can_frame_frequency_hz", default_value="20.0"),
        DeclareLaunchArgument("telemetry_timeout_seconds", default_value="0.5"),
        DeclareLaunchArgument("command_timeout_seconds", default_value="0.5"),
        DeclareLaunchArgument("enable_teleop", default_value="true"),
        DeclareLaunchArgument("enable_joystick", default_value="true"),
        DeclareLaunchArgument("joy_device", default_value="/dev/input/js0"),
        DeclareLaunchArgument("joy_deadzone", default_value="0.15"),
        DeclareLaunchArgument("publish_description", default_value="true"),
        DeclareLaunchArgument("base_frame", default_value="base_footprint"),
        DeclareLaunchArgument("odom_frame", default_value="odom"),
        DeclareLaunchArgument("odometry_broadcast_tf", default_value="true"),
        DeclareLaunchArgument(
            "articulation_serial_port",
            default_value="/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066EFF373146363143225155-if02",
        ),
        DeclareLaunchArgument("publish_runtime_joint_states", default_value="false"),
        DeclareLaunchArgument("runtime_joint_states_topic", default_value="runtime_joint_states"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=os.path.join(description_share, "rviz", "urdf_config.rviz"),
        ),
    ]

    forwarded = [
        "setup_vcan",
        "setup_real_can",
        "robot_namespace",
        "use_namespace",
        "use_sim_time",
        "can_interface",
        "can_bitrate",
        "can_id",
        "driver_params_file",
        "health_params_file",
        "control_params_file",
        "driver_log_level",
        "control_frequency_hz",
        "can_frame_frequency_hz",
        "telemetry_timeout_seconds",
        "command_timeout_seconds",
        "enable_teleop",
        "enable_joystick",
        "joy_device",
        "joy_deadzone",
        "publish_description",
        "base_frame",
        "odom_frame",
        "odometry_broadcast_tf",
        "articulation_serial_port",
        "publish_runtime_joint_states",
        "runtime_joint_states_topic",
        "use_rviz",
        "rviz_config",
    ]

    include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(canonical_launch),
        launch_arguments={name: LaunchConfiguration(name) for name in forwarded}.items(),
    )

    return LaunchDescription(arguments + [include])
