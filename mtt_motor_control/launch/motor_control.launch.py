"""
motor_control.launch.py
-----------------------
Launches only the COM position controller node.
The driver must be running separately (see com_motor.launch.py for the combined launch).
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("mtt_motor_control")
    default_cfg = os.path.join(pkg_share, "config", "motor_control.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_cfg),

        Node(
            package="mtt_motor_control",
            executable="com_position_node",
            name="com_position_node",
            output="screen",
            parameters=[LaunchConfiguration("params_file")],
        ),
    ])
