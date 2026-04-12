#!/usr/bin/env python3
"""
MTT Composable System Launch File

This launch file starts the complete MTT composable architecture including:
- MTT driver wrapper (hardware abstraction + ROS integration + safety)
- MTT odometry node (dedicated composable odometry calculations)
- Joystick input (optional)
- Teleop controller (optional)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node, PushROSNamespace
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_share = FindPackageShare(package='mtt_description').find('mtt_description')
    driver_share = FindPackageShare(package='mtt_driver').find('mtt_driver')
    robot_namespace = LaunchConfiguration('robot_namespace')
    use_namespace = LaunchConfiguration('use_namespace')

    # --- optional host-side CAN bring-up ---
    setup_vcan_process = ExecuteProcess(
        cmd=[
            'bash', '-lc',
            'IFACE="${CAN_IFACE}"; '
            'sudo modprobe vcan; '
            'sudo ip link add dev "${IFACE}" type vcan 2>/dev/null || true; '
            'sudo ip link set "${IFACE}" up; '
            'echo "[can] ${IFACE} VCAN up."'
        ],
        additional_env={
            'CAN_IFACE': LaunchConfiguration('can_interface'),
        },
        name='setup_vcan',
        output='screen',
        condition=IfCondition(LaunchConfiguration('setup_vcan')),
    )

    setup_real_can_process = ExecuteProcess(
        cmd=[
            'bash', '-lc',
            'IFACE="$(echo $CAN_IFACE)"; RATE="$(echo $CAN_RATE)"; '
            'sudo ip link set "$IFACE" down 2>/dev/null || true; '
            'sudo ip link set "$IFACE" up type can bitrate "$RATE"; '
            'echo "[can] ${IFACE} UP @ ${RATE} bps."'
        ],
        additional_env={
            'CAN_IFACE': LaunchConfiguration('can_interface'),
            'CAN_RATE':  LaunchConfiguration('can_bitrate'),
        },
        name='setup_real_can',
        output='screen',
        condition=IfCondition(LaunchConfiguration('setup_real_can')),
    )

    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(description_share, 'launch', 'mtt_description.launch.py')),
        launch_arguments={
            'use_rviz': LaunchConfiguration('use_rviz'),
            'use_joint_state_gui': 'false',
        }.items(),
        condition=IfCondition(LaunchConfiguration('publish_description')),
    )

    return LaunchDescription([
        # -------- Arguments --------
        DeclareLaunchArgument(
            'setup_vcan',
            default_value='false',
            description='Bring up vcan0 (Docker/testing - uses sudo).'
        ),
        DeclareLaunchArgument(
            'robot_namespace',
            default_value='',
            description='Top-level namespace for the MTT runtime'
        ),
        DeclareLaunchArgument(
            'use_namespace',
            default_value='false',
            description='Whether to apply robot_namespace to the MTT runtime'
        ),
        DeclareLaunchArgument(
            'setup_real_can',
            default_value='false',
            description='Bring up real CAN interface with bitrate (host sudo).'
        ),
        DeclareLaunchArgument(
            'can_interface',
            default_value='can0',
            description='CAN interface name for real hardware'
        ),
        DeclareLaunchArgument(
            'can_bitrate',
            default_value='250000',
            description='Bitrate for real CAN interface (e.g., 250000, 500000).'
        ),
        DeclareLaunchArgument(
            'can_id',
            default_value='1',
            description='Command CAN arbitration ID as a decimal integer (1 means 0x001).'
        ),
        DeclareLaunchArgument(
            'driver_log_level',
            default_value='INFO',
            description='Driver logging level (DEBUG, INFO, WARNING, ERROR)'
        ),
        DeclareLaunchArgument(
            'control_frequency_hz',
            default_value='50.0',
            description='Driver control loop frequency (Hz)'
        ),
        DeclareLaunchArgument(
            'can_frame_frequency_hz',
            default_value='20.0',
            description='CAN frame frequency (Hz)'
        ),
        DeclareLaunchArgument(
            'telemetry_timeout_seconds',
            default_value='0.5',
            description='Maximum age for 0x2FF telemetry before motion state is treated as stale'
        ),
        DeclareLaunchArgument(
            'command_timeout_seconds',
            default_value='0.5',
            description='Maximum age for cmd_vel before throttle and steering are neutralized'
        ),
        DeclareLaunchArgument(
            'enable_teleop',
            default_value='true',
            description='Enable teleoperation (joystick + teleop controller + smoother)'
        ),
        DeclareLaunchArgument(
            'enable_joystick',
            default_value='true',
            description='Enable joystick input node'
        ),
        DeclareLaunchArgument(
            'publish_description',
            default_value='true',
            description='Publish robot_state_publisher for real hardware run'
        ),
        DeclareLaunchArgument(
            'base_frame',
            default_value='base_footprint',
            description='Base frame of the robot (child of odom)'
        ),
        DeclareLaunchArgument(
            'odom_frame',
            default_value='odom',
            description='Odom frame (parent of base frame)'
        ),
        DeclareLaunchArgument(
            'enable_map_frame',
            default_value='false',
            description='Publish static map->odom identity transform'
        ),
        DeclareLaunchArgument(
            'odometry_broadcast_tf',
            default_value='true',
            description='Whether mtt_odometry_manager should publish odom->base TF'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz for visualization'
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(description_share, 'rviz', 'urdf_config.rviz'),
            description='RViz config file path'
        ),

        # -------- Actions / Nodes --------

        # 0) Optional host-side CAN interface setup
        setup_vcan_process,
        setup_real_can_process,

        # 1) Robot description / TF tree
        robot_description,

        # 2) Joint controller (cmd_vel → joints) and joint_states
        # Node(
        #     package='mtt_driver',
        #     executable='mtt_joint_controller',
        #     name='mtt_joint_controller',
        #     # No remapping - should receive final muxed commands from twist_mux
        #     output='screen'
        # ),

        # 3) Optional static map->odom identity TF (RViz dead-reckoning)
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_map_odom',
        #     arguments=['0','0','0','0','0','0','map', LaunchConfiguration('odom_frame')],
        #     condition=IfCondition(LaunchConfiguration('enable_map_frame'))
        # ),

        GroupAction(actions=[
            PushROSNamespace(condition=IfCondition(use_namespace), namespace=robot_namespace),
            Node(
                package='mtt_driver',
                executable='mtt_ros_wrapper',
                name='mtt_ros_wrapper',
                parameters=[{
                    'can_interface': LaunchConfiguration('can_interface'),
                    'can_id': ParameterValue(LaunchConfiguration('can_id'), value_type=int),
                    'driver_log_level': LaunchConfiguration('driver_log_level'),
                    'control_frequency_hz': LaunchConfiguration('control_frequency_hz'),
                    'can_frame_frequency_hz': LaunchConfiguration('can_frame_frequency_hz'),
                    'telemetry_timeout_seconds': LaunchConfiguration('telemetry_timeout_seconds'),
                    'command_timeout_seconds': LaunchConfiguration('command_timeout_seconds'),
                    'base_frame': LaunchConfiguration('base_frame'),
                }],
                output='screen',
                emulate_tty=True,
                respawn=True,
                respawn_delay=2.0
            ),


        # Node(
        #     package='mtt_driver',
        #     executable='mtt_odometry_manager',
        #     name='mtt_odometry_manager',
        #     arguments=['--ros-args', '--log-level', LaunchConfiguration('driver_log_level')],
        #     parameters=[{
        #         'base_frame': LaunchConfiguration('base_frame'),
        #         'odom_frame': LaunchConfiguration('odom_frame'),
        #         # Odometry now listens to the final muxed cmd_vel topic
        #         'cmd_vel_topic': 'cmd_vel',
        #         # Toggle TF broadcasting via launch arg
        #         'broadcast_tf': LaunchConfiguration('odometry_broadcast_tf'),
        #         # Steering control and anti-drift tuning - using centralized vehicle parameters
        #         'steer_control_mode': 'closed_loop',
        #         # max_articulation_angle and max_yaw_rate now come from centralized vehicle parameters
        #         'pivot_turn_enabled': False,
        #         'min_turn_speed_ms': 0.05,
        #         'yaw_slip_factor': 0.6,
        #     }],
        #     output='screen',
        #     emulate_tty=True,
        #     respawn=True,
        #     respawn_delay=2.0
        # ),

        # 4.5) Twist Mux - Command multiplexer (replaces internal muxing)
            Node(
                package='twist_mux',
                executable='twist_mux',
                name='twist_mux',
                parameters=[os.path.join(driver_share, 'config', 'twist_mux.yaml')],
                remappings=[('cmd_vel_out', 'cmd_vel')],
                output='screen',
                respawn=True,
                respawn_delay=2.0
            ),

        # 6) Joystick (joy_linux)
            Node(
                package='joy_linux',
                executable='joy_linux_node',
                name='joy_node',
                parameters=[{
                    'deadzone': 0.15,
                    'device_name': '/dev/input/js0'
                }],
                output='screen',
                condition=IfCondition(LaunchConfiguration('enable_joystick')),
                respawn=True
            ),

        # # 6.5) Teleop command smoother (decays to zero on input inactivity)
        # Node(
        #     package='mtt_driver',
        #     executable='teleop_cmd_smoother',
        #     name='teleop_cmd_smoother',
        #     parameters=[{
        #         'input_topic': 'cmd_vel/teleop',
        #         'output_topic': 'cmd_vel/teleop_smoothed',
        #         'input_timeout': 0.5,
        #         'rate_hz': 50.0,
        #     }],
        #     output='screen',
        #     condition=IfCondition(LaunchConfiguration('enable_teleop')),
        #     respawn=True
        # ),

        # 7) Teleop
            Node(
                package='mtt_driver',
                executable='mtt_teleop_joy',
                name='mtt_teleop_joy_node',
                remappings=[('cmd_vel_raw', 'cmd_vel/teleop')],
                output='screen',
                condition=IfCondition(LaunchConfiguration('enable_teleop')),
                respawn=True
            ),
        ]),

    
    ])
