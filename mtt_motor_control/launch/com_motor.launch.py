"""
com_motor.launch.py
-------------------
Combined launch: EtherCAT driver + COM position controller.

Usage:
  ros2 launch mtt_motor_control com_motor.launch.py
  ros2 launch mtt_motor_control com_motor.launch.py dry_run:=true

The driver node (mtt_motor_driver) starts first.
The position controller starts immediately after (it waits for /motor/joint_state
to arrive before accepting commands, so timing is not critical).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _make_nodes(context, *args, **kwargs):
    """OpaqueFunction that builds the node list once all substitutions are resolved."""
    dry_run     = LaunchConfiguration("dry_run").perform(context)
    auto_enable = LaunchConfiguration("auto_enable").perform(context)
    iface       = LaunchConfiguration("ethercat_ifname").perform(context)
    drv_cfg     = LaunchConfiguration("driver_params").perform(context)
    ctrl_cfg    = LaunchConfiguration("control_params").perform(context)
    com_params  = LaunchConfiguration("com_params").perform(context)

    # Build parameter list for each node.
    # com_params overrides per-node defaults only when provided (non-empty path).
    drv_params  = [drv_cfg]
    ctrl_params = [ctrl_cfg]
    if com_params:
        drv_params.append(com_params)
        ctrl_params.append(com_params)

    drv_params.append({
        "dry_run":         dry_run.lower() == "true",
        "auto_enable":     auto_enable.lower() == "true",
        "ethercat_ifname": iface,
    })

    return [
        Node(
            package="mtt_motor_driver",
            executable="motor_driver_node",
            name="motor_driver_node",
            output="screen",
            parameters=drv_params,
        ),
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package="mtt_motor_control",
                    executable="com_position_node",
                    name="com_position_node",
                    output="screen",
                    parameters=ctrl_params,
                ),
            ],
        ),
    ]


def generate_launch_description():
    ctrl_share = get_package_share_directory("mtt_motor_control")
    drv_share  = get_package_share_directory("mtt_motor_driver")

    ctrl_cfg = os.path.join(ctrl_share, "config", "motor_control.yaml")
    drv_cfg  = os.path.join(drv_share,  "config", "motor_driver.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("dry_run",         default_value="false"),
        DeclareLaunchArgument("auto_enable",     default_value="true"),
        DeclareLaunchArgument("ethercat_ifname", default_value=""),
        DeclareLaunchArgument("driver_params",   default_value=drv_cfg),
        DeclareLaunchArgument("control_params",  default_value=ctrl_cfg),
        # Operator calibration override: position limits and home.
        # Contains both motor_driver_node and com_position_node namespaces.
        # Leave empty to use package defaults only.
        DeclareLaunchArgument(
            "com_params", default_value="",
            description="Operator YAML with position_limit_min/max and home_position_counts"),
        OpaqueFunction(function=_make_nodes),
    ])
