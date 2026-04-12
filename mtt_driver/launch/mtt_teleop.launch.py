#!/usr/bin/env python3
"""
MTT Teleop Launch File

This launch file starts the complete MTT teleoperation system including:
- Joystick input (joy_linux)
- MTT driver wrapper with configurable logging
- Teleop controller

Usage examples:
  # Default setup for real hardware
  ros2 launch mtt_driver mtt_teleop.launch.py

  # Test mode with virtual CAN
  ros2 launch mtt_driver mtt_teleop.launch.py test_mode:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, PushROSNamespace


def generate_launch_description():
    robot_namespace = LaunchConfiguration('robot_namespace')
    use_namespace = LaunchConfiguration('use_namespace')

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_namespace',
            default_value='',
            description='Top-level namespace for the teleop stack'
        ),
        DeclareLaunchArgument(
            'use_namespace',
            default_value='false',
            description='Whether to apply robot_namespace to the teleop stack'
        ),
        DeclareLaunchArgument(
            'can_interface',
            default_value='can0',
            description='CAN interface name'
        ),
        DeclareLaunchArgument(
            'test_mode',
            default_value='false',
            description='Enable test mode (uses vcan0)'
        ),
        DeclareLaunchArgument(
            'driver_log_level',
            default_value='INFO',
            description='Driver logging level (DEBUG, INFO, WARNING, ERROR)'
        ),

        GroupAction(actions=[
            PushROSNamespace(condition=IfCondition(use_namespace), namespace=robot_namespace),
            Node(
                package='joy_linux',
                executable='joy_linux_node',
                name='joy_node',
                parameters=[{
                    'deadzone': 0.15,
                    'device_name': '/dev/input/js0'
                }],
                output='screen'
            ),

            Node(
                package='mtt_driver',
                executable='mtt_ros_wrapper',
                name='mtt_driver_node',
                parameters=[{
                    'can_interface': LaunchConfiguration('can_interface'),
                    'test_mode': LaunchConfiguration('test_mode'),
                    'driver_log_level': LaunchConfiguration('driver_log_level'),
                }],
                output='screen'
            ),

            Node(
                package='mtt_driver',
                executable='mtt_teleop_joy',
                name='mtt_teleop_joy_node',
                remappings=[('cmd_vel_raw', 'cmd_vel/teleop')],
                output='screen'
            ),

            Node(
                package='mtt_driver',
                executable='teleop_cmd_smoother',
                name='teleop_cmd_smoother',
                parameters=[{
                    'input_topic': 'cmd_vel/teleop',
                    'output_topic': 'cmd_vel',
                    'max_accel_linear': 1.5,
                    'max_accel_angular': 1.5,
                    'rate_hz': 50.0,
                }],
                output='screen'
            ),

            Node(
                package='mtt_driver',
                executable='mtt_odometry_manager',
                name='mtt_odometry_manager',
                arguments=['--ros-args', '--log-level', LaunchConfiguration('driver_log_level')],
                output='screen',
                respawn=True,
                respawn_delay=2.0
            ),
        ]),
    ])
