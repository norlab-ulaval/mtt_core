import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('mtt_gps_driver')

    gps_mode_arg = DeclareLaunchArgument(
        'gps_mode',
        default_value='serial',
        description='GPS connection mode: "serial" (USB/ttyACM) or "tcp" (WiFi, requires TCP output enabled in ReachView3)'
    )
    gps_mode = LaunchConfiguration('gps_mode')

    def _make_nodes(context, *args, **kwargs):
        mode = context.perform_substitution(gps_mode)
        if mode not in ('tcp', 'serial'):
            raise ValueError(f"gps_mode must be 'tcp' or 'serial', got: '{mode}'")

        config = os.path.join(pkg, 'config', f'gps_{mode}.yaml')

        left_node = Node(
            package='mtt_gps_driver',
            executable='reach_driver_node',
            name='reach_driver_left',
            namespace='gps_left',
            output='both',
            respawn=True,
            respawn_delay=3.0,
            parameters=[config],
        )

        right_node = Node(
            package='mtt_gps_driver',
            executable='reach_driver_node',
            name='reach_driver_right',
            namespace='gps_right',
            output='both',
            respawn=True,
            respawn_delay=3.0,
            parameters=[config],
        )

        heading_node = Node(
            package='mtt_gps_driver',
            executable='dual_antenna_heading_node',
            name='dual_antenna_heading',
            output='both',
            respawn=True,
            parameters=[config],
            remappings=[
                ('gps_left/fix',  '/gps_left/fix'),
                ('gps_right/fix', '/gps_right/fix'),
            ],
        )

        return [left_node, right_node, heading_node]

    from launch.actions import OpaqueFunction
    return LaunchDescription([
        gps_mode_arg,
        OpaqueFunction(function=_make_nodes),
    ])
