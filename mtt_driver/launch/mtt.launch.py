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
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node, PushROSNamespace, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_share = FindPackageShare(package='mtt_description').find('mtt_description')
    urdf_path = os.path.join(description_share, 'urdf', 'robot.urdf.xacro')
    robot_namespace = LaunchConfiguration('robot_namespace')
    use_namespace = LaunchConfiguration('use_namespace')

    # --- real CAN bring-up (with bitrate) ---
    # All commands end with || true so a failure (e.g. sudo not available in the
    # container, or interface already up) never causes the launch to shut down.
    setup_real_can_process = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'IFACE="$(echo $CAN_IFACE)"; RATE="$(echo $CAN_RATE)"; '
            'sudo ip link set "$IFACE" down 2>/dev/null || true; '
            'sudo ip link set "$IFACE" up type can bitrate "$RATE" 2>/dev/null || true; '
            'echo "[can] ${IFACE} bring-up attempted @ ${RATE} bps (errors are non-fatal)."'
        ],
        additional_env={
            'CAN_IFACE': LaunchConfiguration('can_interface'),
            'CAN_RATE':  LaunchConfiguration('can_bitrate'),
        },
        name='setup_real_can',
        output='screen',
        condition=IfCondition(LaunchConfiguration('setup_real_can')),
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
            description='Whether mtt_odometry_node should publish odom->base TF'
        ),
        DeclareLaunchArgument(
            'max_articulation_deg',
            default_value='60.0',
            description='Maximum articulation angle (degrees). Matches URDF yaw joint ±60°.'
        ),
        DeclareLaunchArgument(
            'max_linear_speed_ms',
            default_value='0.6',
            description='Maximum linear speed fed to the drive controller (m/s).'
        ),
        DeclareLaunchArgument(
            'throttle_deadband',
            default_value='0.05',
            description='Normalized throttle deadband (0–1). Tune to remove mechanical slop.'
        ),
        DeclareLaunchArgument(
            'steer_deadband',
            default_value='0.05',
            description='Normalized steering deadband (0–1). Tune to remove mechanical slop.'
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

        # 0b) (Optional) Ensure real CAN is up with bitrate BEFORE anything else
        setup_real_can_process,

        # 1) Robot description (URDF → TF tree for real runs)
        

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

            # ── MTT Core: MultiThreaded composable container ────────────
            # We use an OpaqueFunction to conditionally exclude the CAN node in simulation
            OpaqueFunction(function=lambda context: [
                ComposableNodeContainer(
                    name='mtt_core_container',
                    namespace='',
                    package='rclcpp_components',
                    executable='component_container_mt',
                    composable_node_descriptions=(
                        [
                            ComposableNode(
                                package='mtt_driver',
                                plugin='mtt::MttCanNode',
                                name='mtt_can_node',
                                parameters=[{
                                    'can_interface': LaunchConfiguration('can_interface'),
                                    'can_id': ParameterValue(LaunchConfiguration('can_id'), value_type=int),
                                    'control_frequency_hz': LaunchConfiguration('control_frequency_hz'),
                                    'can_frame_frequency_hz': LaunchConfiguration('can_frame_frequency_hz'),
                                    'max_linear_speed_ms': LaunchConfiguration('max_linear_speed_ms'),
                                    'telemetry_timeout_ms': ParameterValue(
                                        PythonExpression(['1000.0 * ', LaunchConfiguration('telemetry_timeout_seconds')]),
                                        value_type=float,
                                    ),
                                    'command_timeout_seconds': LaunchConfiguration('command_timeout_seconds'),
                                    'base_frame': LaunchConfiguration('base_frame'),
                                    'throttle_deadband': LaunchConfiguration('throttle_deadband'),
                                    'steer_deadband': LaunchConfiguration('steer_deadband'),
                                }],
                                extra_arguments=[{'use_intra_process_comms': True}],
                            )
                        ]
                        if LaunchConfiguration('use_sim_time').perform(context).lower() != 'true'
                        else []
                    ) + [
                        ComposableNode(
                            package='mtt_driver',
                            plugin='mtt::MttOdometryNode',
                            name='mtt_odometry_node',
                            parameters=[{
                                'base_frame': LaunchConfiguration('base_frame'),
                                'odom_frame': LaunchConfiguration('odom_frame'),
                                'broadcast_tf': LaunchConfiguration('odometry_broadcast_tf'),
                                'steer_control_mode': 'closed_loop',
                                'pivot_turn_enabled': False,
                                'min_turn_speed_ms': 0.05,
                                'yaw_slip_factor': 0.6,
                                'max_articulation_deg': LaunchConfiguration('max_articulation_deg'),
                            }],
                            extra_arguments=[{'use_intra_process_comms': True}],
                        )
                    ],
                    output='screen',
                    emulate_tty=True,
                )
            ]),

            Node(
                package='twist_mux',
                executable='twist_mux',
                name='twist_mux',
                parameters=[os.path.join(FindPackageShare(package='mtt_driver').find('mtt_driver'), 'config', 'twist_mux.yaml')],
                remappings=[('cmd_vel_out', 'cmd_vel')],
                output='screen',
                respawn=True,
                respawn_delay=2.0
            ),
        # Node(
        #     package='twist_stamper',
        #     executable='twist_stamper',
        #     name='twist_stamper',
        #     parameters=[{
        #         'input_topic': 'cmd_vel',
        #         'output_topic': 'cmd_vel_stamped',
        #         'frame_id': 'base_link'
        #     }],
        #     output='screen',
        #     respawn=True,
        #     respawn_delay=2.0
        # ),


        # 6) Joystick (joy_linux)
        # Node(
        #     package='joy_linux',
        #     executable='joy_linux_node',
        #     name='joy_node',
        #     parameters=[{
        #         'deadzone': 0.15,
        #         'device_name': '/dev/input/js0'
        #     }],
        #     output='screen',
        #     condition=IfCondition(LaunchConfiguration('enable_joystick')),
        #     respawn=True
        # ),
            Node(
                package='joy_linux',
                executable='joy_linux_node',
                name='joy_node',
                parameters=[{
                    'deadzone': 0.15,
                    'device_name': '/dev/input/js0',
                    'autorepeat_rate': 20.0,
                }],
                output='screen',
                condition=IfCondition(LaunchConfiguration('enable_joystick')),
                respawn=True
            ),

        # 6.5) Teleop command smoother — rate-limits acceleration and decays on timeout
        #      Edit config/teleop_smoother.yaml to tune max_accel_linear/angular.
            Node(
                package='mtt_driver',
                executable='teleop_cmd_smoother_node_exe',
                name='teleop_cmd_smoother_node',
                parameters=[os.path.join(
                    FindPackageShare(package='mtt_driver').find('mtt_driver'),
                    'config', 'teleop_smoother.yaml')],
                output='screen',
                respawn=True,
                respawn_delay=2.0,
            ),

        # 7) Teleop
            Node(
                package='mtt_driver',
                executable='mtt_teleop_joy',
                name='mtt_teleop_joy_node',
                parameters=[os.path.join(
                    FindPackageShare(package='mtt_driver').find('mtt_driver'),
                    'config', 'mtt_teleop_joy.yaml')],
                remappings=[('cmd_vel_raw', 'cmd_vel/teleop')],
                output='screen',
                condition=IfCondition(LaunchConfiguration('enable_teleop')),
                respawn=True
            ),
        ]),
    ])
