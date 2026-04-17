#!/usr/bin/env python3
"""
MTT Teleop Launch File

Starts the minimal teleoperation stack:
- Joystick input (joy_linux)
- MTT CAN driver + Odometry (C++ composable, MultiThreadedExecutor, intra-process)
- Teleop controller

Usage:
  ros2 launch mtt_driver mtt_teleop.launch.py
  ros2 launch mtt_driver mtt_teleop.launch.py can_interface:=vcan0  # test avec vcan
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushROSNamespace, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_namespace = LaunchConfiguration('robot_namespace')
    use_namespace = LaunchConfiguration('use_namespace')

    return LaunchDescription([
        DeclareLaunchArgument('robot_namespace', default_value='',
                              description='Top-level namespace'),
        DeclareLaunchArgument('use_namespace', default_value='false',
                              description='Apply robot_namespace'),
        DeclareLaunchArgument('can_interface', default_value='can0',
                              description='CAN interface (can0 or vcan0)'),
        DeclareLaunchArgument('can_id', default_value='1',
                      description='Command CAN arbitration ID (decimal)'),
        DeclareLaunchArgument('control_frequency_hz', default_value='50.0',
                              description='Driver control loop frequency'),
        DeclareLaunchArgument('can_frame_frequency_hz', default_value='20.0',
                              description='CAN frame send frequency'),
        DeclareLaunchArgument('base_frame', default_value='base_footprint',
                              description='Base frame of the robot'),
        DeclareLaunchArgument('odom_frame', default_value='odom',
                              description='Odometry frame'),
        DeclareLaunchArgument('enable_joystick', default_value='true',
                              description='Enable joystick input node'),

        GroupAction(actions=[
            PushROSNamespace(condition=IfCondition(use_namespace), namespace=robot_namespace),

            # ── Joystick ───────────────────────────────────────────────
            Node(
                package='joy_linux',
                executable='joy_linux_node',
                name='joy_node',
                parameters=[{'deadzone': 0.15, 'device_name': '/dev/input/js0', 'autorepeat_rate': 20.0}],
                output='screen',
                condition=IfCondition(LaunchConfiguration('enable_joystick')),
                respawn=True
            ),

            # ── MTT Core: C++ MultiThreaded composable container ────────
            ComposableNodeContainer(
                name='mtt_core_container',
                namespace='',
                package='rclcpp_components',
                executable='component_container_mt',
                composable_node_descriptions=[

                    ComposableNode(
                        package='mtt_driver',
                        plugin='mtt::MttCanNode',
                        name='mtt_can_node',
                        parameters=[{
                            'can_interface': LaunchConfiguration('can_interface'),
                            'can_id': ParameterValue(LaunchConfiguration('can_id'), value_type=int),
                            'control_frequency_hz': LaunchConfiguration('control_frequency_hz'),
                            'can_frame_frequency_hz': LaunchConfiguration('can_frame_frequency_hz'),
                            'telemetry_timeout_ms': 500.0,
                            'command_timeout_seconds': 0.5,
                            'base_frame': LaunchConfiguration('base_frame'),
                        }],
                        extra_arguments=[{'use_intra_process_comms': True}],
                    ),

                    ComposableNode(
                        package='mtt_driver',
                        plugin='mtt::MttOdometryNode',
                        name='mtt_odometry_node',
                        parameters=[{
                            'base_frame': LaunchConfiguration('base_frame'),
                            'odom_frame': LaunchConfiguration('odom_frame'),
                            'broadcast_tf': True,
                            'steer_control_mode': 'closed_loop',
                            'pivot_turn_enabled': False,
                            'min_turn_speed_ms': 0.05,
                            'yaw_slip_factor': 0.6,
                        }],
                        extra_arguments=[{'use_intra_process_comms': True}],
                    ),
                ],
                output='screen',
                emulate_tty=True,
            ),

            # ── Teleop joystick controller ─────────────────────────────
            Node(
                package='mtt_driver',
                executable='mtt_teleop_joy',
                name='mtt_teleop_joy_node',
                parameters=[
                    os.path.join(
                        FindPackageShare(package='mtt_driver').find('mtt_driver'),
                        'config',
                        'mtt_teleop_joy.yaml'
                    )
                ],
                remappings=[('cmd_vel_raw', 'cmd_vel/teleop')],
                output='screen',
                respawn=True
            ),
        ]),
    ])
