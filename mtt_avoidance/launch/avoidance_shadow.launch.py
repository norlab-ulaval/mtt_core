#!/usr/bin/env python3
"""Optional avoidance stack. No node starts unless enabled:=true."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    share = get_package_share_directory("mtt_avoidance")
    default_avoidance = os.path.join(share, "config", "avoidance.yaml")
    default_nav2 = os.path.join(share, "config", "nav2_avoidance.yaml")

    enabled = LaunchConfiguration("enabled")
    shadow_mode = LaunchConfiguration("shadow_mode")
    execution_enabled = LaunchConfiguration("execution_enabled")
    reverse_enabled = LaunchConfiguration("reverse_recovery_enabled")
    use_sim_time = LaunchConfiguration("use_sim_time")
    avoidance_params = LaunchConfiguration("avoidance_params_file")
    nav2_params = LaunchConfiguration("nav2_params_file")
    command_output = LaunchConfiguration("command_output_topic")
    hardware_output_enabled = LaunchConfiguration("hardware_output_enabled")
    speed_output = LaunchConfiguration("speed_output_topic")
    articulation_output = LaunchConfiguration("articulation_output_topic")

    configured_nav2 = ParameterFile(
        RewrittenYaml(
            source_file=nav2_params,
            root_key="mtt_avoidance",
            param_rewrites={"use_sim_time": use_sim_time},
            convert_types=True,
        ),
        allow_substs=True,
    )
    common_nav2 = [configured_nav2]
    tf_remaps = [("/tf", "/tf"), ("/tf_static", "/tf_static")]
    lifecycle_nodes = [
        "planner_server",
        "controller_server",
        "velocity_smoother",
        "behavior_server",
        "collision_monitor",
    ]

    stack = GroupAction(
        condition=IfCondition(enabled),
        actions=[
            LogInfo(
                msg=[
                    "Starting isolated MTT avoidance stack: shadow=",
                    shadow_mode,
                    ", execution=",
                    execution_enabled,
                    ", output=",
                    command_output,
                ]
            ),
            Node(
                package="wiln",
                executable="wiln_obstacle_node",
                name="mtt_avoidance_obstacle_filter",
                output="both",
                parameters=[
                    avoidance_params,
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                ],
                remappings=tf_remaps,
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                namespace="mtt_avoidance",
                name="planner_server",
                output="both",
                parameters=common_nav2,
                remappings=tf_remaps,
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                namespace="mtt_avoidance",
                name="controller_server",
                output="both",
                parameters=common_nav2,
                remappings=tf_remaps + [("cmd_vel", "/mtt_avoidance/nav_cmd_controller")],
            ),
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                namespace="mtt_avoidance",
                name="velocity_smoother",
                output="both",
                parameters=common_nav2,
                remappings=tf_remaps
                + [
                    ("cmd_vel", "/mtt_avoidance/nav_cmd_controller"),
                    ("cmd_vel_smoothed", "/mtt_avoidance/nav_cmd_smoothed"),
                ],
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                namespace="mtt_avoidance",
                name="behavior_server",
                output="both",
                parameters=common_nav2,
                remappings=tf_remaps + [("cmd_vel", "/mtt_avoidance/nav_cmd_controller")],
            ),
            Node(
                package="nav2_collision_monitor",
                executable="collision_monitor",
                namespace="mtt_avoidance",
                name="collision_monitor",
                output="both",
                parameters=common_nav2,
                remappings=tf_remaps,
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                namespace="mtt_avoidance",
                name="lifecycle_manager_avoidance",
                output="both",
                parameters=[
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                    {"autostart": True, "node_names": lifecycle_nodes},
                ],
            ),
            Node(
                package="mtt_avoidance",
                executable="mtt_composite_path_validator",
                name="mtt_composite_path_validator",
                output="both",
                parameters=[
                    avoidance_params,
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                ],
            ),
            Node(
                package="mtt_avoidance",
                executable="mtt_rejoin_planner",
                name="mtt_rejoin_planner",
                output="both",
                parameters=[
                    avoidance_params,
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                ],
            ),
            Node(
                package="mtt_avoidance",
                executable="mtt_avoidance_executor",
                name="mtt_avoidance_executor",
                output="both",
                parameters=[
                    avoidance_params,
                    {
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "execution_enabled": ParameterValue(execution_enabled, value_type=bool),
                        "shadow_mode": ParameterValue(shadow_mode, value_type=bool),
                    },
                ],
            ),
            Node(
                package="mtt_avoidance",
                executable="mtt_recovery_manager",
                name="mtt_recovery_manager",
                output="both",
                parameters=[
                    avoidance_params,
                    {
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "reverse_enabled": ParameterValue(reverse_enabled, value_type=bool),
                    },
                ],
            ),
            Node(
                package="mtt_avoidance",
                executable="mtt_avoidance_supervisor",
                name="mtt_avoidance_supervisor",
                output="both",
                parameters=[
                    avoidance_params,
                    {
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "enabled": True,
                        "shadow_mode": ParameterValue(shadow_mode, value_type=bool),
                        "execution_enabled": ParameterValue(
                            execution_enabled, value_type=bool
                        ),
                        "reverse_recovery_enabled": ParameterValue(
                            reverse_enabled, value_type=bool
                        ),
                    },
                ],
            ),
            Node(
                package="mtt_control",
                executable="mtt_articulated_nav_adapter_node",
                name="mtt_avoidance_nav_adapter",
                output="both",
                parameters=[
                    avoidance_params,
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                ],
            ),
            Node(
                package="mtt_avoidance",
                executable="mtt_avoidance_command_mux",
                name="mtt_avoidance_command_mux",
                output="both",
                parameters=[
                    avoidance_params,
                    {
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "output_topic": command_output,
                    },
                ],
            ),
            Node(
                package="mtt_control",
                executable="mtt_articulated_command_dispatcher_node",
                name="mtt_avoidance_command_dispatcher",
                output="both",
                parameters=[
                    avoidance_params,
                    {
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "enabled": ParameterValue(execution_enabled, value_type=bool),
                        "shadow_mode": ParameterValue(shadow_mode, value_type=bool),
                        "hardware_output_enabled": ParameterValue(
                            hardware_output_enabled, value_type=bool
                        ),
                        "input_topic": command_output,
                        "speed_output_topic": speed_output,
                        "articulation_output_topic": articulation_output,
                    },
                ],
            ),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("enabled", default_value="false"),
            DeclareLaunchArgument("shadow_mode", default_value="true"),
            DeclareLaunchArgument("execution_enabled", default_value="false"),
            DeclareLaunchArgument("reverse_recovery_enabled", default_value="false"),
            DeclareLaunchArgument("hardware_output_enabled", default_value="false"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("avoidance_params_file", default_value=default_avoidance),
            DeclareLaunchArgument("nav2_params_file", default_value=default_nav2),
            DeclareLaunchArgument(
                "command_output_topic",
                default_value="/mtt_avoidance/selected_articulated_cmd",
            ),
            DeclareLaunchArgument(
                "speed_output_topic",
                default_value="/mtt_avoidance/speed_cmd_preview",
            ),
            DeclareLaunchArgument(
                "articulation_output_topic",
                default_value="/mtt_avoidance/articulation_setpoint_preview",
            ),
            stack,
        ]
    )
