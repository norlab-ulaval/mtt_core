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
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node, PushROSNamespace, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_share = FindPackageShare(package='mtt_description').find('mtt_description')
    driver_share = FindPackageShare(package='mtt_driver').find('mtt_driver')
    control_share = FindPackageShare(package='mtt_control').find('mtt_control')
    driver_params_path = os.path.join(driver_share, 'config', 'mtt_driver_params.yaml')
    health_params_path = os.path.join(driver_share, 'config', 'mtt_health_monitor.yaml')
    control_params_path = os.path.join(control_share, 'config', 'control_defaults.yaml')
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
            'use_sim_time',
            default_value='false',
            description='Use simulation or bag replay clock'
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
            'driver_params_file',
            default_value=driver_params_path,
            description='ROS parameter file for mtt_can_node and mtt_odometry_node'
        ),
        DeclareLaunchArgument(
            'health_params_file',
            default_value=health_params_path,
            description='ROS parameter file for mtt_health_monitor_node'
        ),
        DeclareLaunchArgument(
            'control_params_file',
            default_value=control_params_path,
            description='ROS parameter file for the MTT operator input, manual filter, mode manager, and cmd arbiter'
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
            'joy_device',
            default_value='/dev/input/js0',
            description='Joystick device path for the built-in robot teleop'
        ),
        DeclareLaunchArgument(
            'joy_deadzone',
            default_value='0.15',
            description='Joystick deadzone for the built-in robot teleop'
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
            'publish_runtime_joint_states',
            default_value='false',
            description='Publish articulation joint states on the live /joint_states topic'
        ),
        DeclareLaunchArgument(
            'runtime_joint_states_topic',
            default_value='joint_states',
            description='JointState topic used to drive the articulated URDF'
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
                                parameters=[LaunchConfiguration('driver_params_file'), {
                                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                                    'can_interface': LaunchConfiguration('can_interface'),
                                    'can_id': ParameterValue(LaunchConfiguration('can_id'), value_type=int),
                                    'control_frequency_hz': LaunchConfiguration('control_frequency_hz'),
                                    'can_frame_frequency_hz': LaunchConfiguration('can_frame_frequency_hz'),
                                    'telemetry_timeout_ms': ParameterValue(
                                        PythonExpression(['1000.0 * ', LaunchConfiguration('telemetry_timeout_seconds')]),
                                        value_type=float,
                                    ),
                                    'command_timeout_seconds': LaunchConfiguration('command_timeout_seconds'),
                                    'base_frame': LaunchConfiguration('base_frame'),
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
                            parameters=[LaunchConfiguration('driver_params_file'), {
                                'use_sim_time': LaunchConfiguration('use_sim_time'),
                                'base_frame': LaunchConfiguration('base_frame'),
                                'odom_frame': LaunchConfiguration('odom_frame'),
                                'broadcast_tf': LaunchConfiguration('odometry_broadcast_tf'),
                                'publish_runtime_joint_states': LaunchConfiguration('publish_runtime_joint_states'),
                                'runtime_joint_states_topic': LaunchConfiguration('runtime_joint_states_topic'),
                            }],
                            extra_arguments=[{'use_intra_process_comms': True}],
                        ),
                        ComposableNode(
                            package='mtt_driver',
                            plugin='mtt::MttHealthMonitorNode',
                            name='mtt_health_monitor_node',
                            parameters=[LaunchConfiguration('health_params_file'), {
                                'use_sim_time': LaunchConfiguration('use_sim_time'),
                                'base_frame': LaunchConfiguration('base_frame'),
                                'odom_frame': LaunchConfiguration('odom_frame'),
                                'cmd_vel_timeout_seconds': LaunchConfiguration('command_timeout_seconds'),
                            }],
                            extra_arguments=[{'use_intra_process_comms': True}],
                        ),
                    ],
                    output='screen',
                    emulate_tty=True,
                )
            ]),

            Node(
                package='joy_linux',
                executable='joy_linux_node',
                name='joy_node',
                parameters=[{
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'deadzone': LaunchConfiguration('joy_deadzone'),
                    'device_name': LaunchConfiguration('joy_device'),
                    'autorepeat_rate': 20.0,
                }],
                output='screen',
                condition=IfCondition(LaunchConfiguration('enable_joystick')),
                respawn=True
            ),
            Node(
                package='mtt_control',
                executable='mtt_operator_input_node',
                name='mtt_operator_input_node',
                parameters=[
                    LaunchConfiguration('control_params_file'),
                    {'use_sim_time': LaunchConfiguration('use_sim_time')},
                ],
                output='screen',
                respawn=True,
                respawn_delay=2.0,
            ),
            Node(
                package='mtt_control',
                executable='mtt_manual_cmd_filter_node',
                name='mtt_manual_cmd_filter_node',
                parameters=[
                    LaunchConfiguration('control_params_file'),
                    {'use_sim_time': LaunchConfiguration('use_sim_time')},
                ],
                output='screen',
                respawn=True,
                respawn_delay=2.0,
            ),
            Node(
                package='mtt_control',
                executable='mtt_mode_manager_node',
                name='mtt_mode_manager_node',
                parameters=[
                    LaunchConfiguration('control_params_file'),
                    {'use_sim_time': LaunchConfiguration('use_sim_time')},
                ],
                output='screen',
                respawn=True,
                respawn_delay=2.0,
            ),
            Node(
                package='mtt_control',
                executable='mtt_cmd_arbiter_node',
                name='mtt_cmd_arbiter_node',
                parameters=[
                    LaunchConfiguration('control_params_file'),
                    {'use_sim_time': LaunchConfiguration('use_sim_time')},
                ],
                output='screen',
                respawn=True,
                respawn_delay=2.0,
            ),
        ]),
    ])
