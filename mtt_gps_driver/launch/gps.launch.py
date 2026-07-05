import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('mtt_gps_driver')

    gps_mode_arg = DeclareLaunchArgument(
        'gps_mode',
        default_value='serial',
        description='Connection mode: "serial" (USB) or "tcp" (WiFi/Ethernet)'
    )
    gps_antennas_arg = DeclareLaunchArgument(
        'gps_antennas',
        default_value='single',
        description=(
            '"single" — one rover on robot + external base station (RTK corrections via LoRa/TCP). '
            '"dual"   — two antennas on robot for dual-antenna heading (legacy setup). '
            '"front"  — one Emlid M+ in front (over ZED) without mixing with right/left.'
        )
    )
    gps_mode      = LaunchConfiguration('gps_mode')
    gps_antennas  = LaunchConfiguration('gps_antennas')

    def _make_nodes(context, *args, **kwargs):
        mode     = context.perform_substitution(gps_mode)
        antennas = context.perform_substitution(gps_antennas)

        if mode not in ('tcp', 'serial'):
            raise ValueError(f"gps_mode must be 'tcp' or 'serial', got: '{mode}'")
        if antennas not in ('single', 'dual', 'front'):
            raise ValueError(f"gps_antennas must be 'single', 'dual', or 'front', got: '{antennas}'")

        if antennas == 'single':
            # ── Single rover + external base station (RTK via LoRa/TCP) ──
            # Config files: gps_rover_serial.yaml or gps_rover_tcp.yaml
            # The driver reads from the rover NMEA stream only.
            # RTK corrections flow directly between the two Reach RS units
            # (LoRa radio or TCP) — the ROS layer does not touch corrections.
            # Topics: /gps/fix  /gps/nmea_sentence  /gps/time_reference
            config = os.path.join(pkg, 'config', f'gps_rover_{mode}.yaml')

            rover_node = Node(
                package='mtt_gps_driver',
                executable='reach_driver_node',
                name='reach_driver_rover',
                namespace='gps',
                output='both',
                respawn=True,
                respawn_delay=3.0,
                parameters=[config],
            )
            return [rover_node]

        elif antennas == 'front':
            # ── Single front rover ──
            # Used for the newly added Emlid M+ placed on the ZED camera.
            # Outputs to /gps_front/fix to avoid mixing with gps_left/gps_right.
            config = os.path.join(pkg, 'config', f'gps_front_{mode}.yaml')

            front_node = Node(
                package='mtt_gps_driver',
                executable='reach_driver_node',
                name='reach_driver_front',
                namespace='gps_front',
                output='both',
                respawn=True,
                respawn_delay=3.0,
                parameters=[config],
            )
            return [front_node]

        else:
            # ── Dual antenna — two rovers on robot, heading from baseline ──
            # Legacy setup: two Reach RS on the robot, ~1.1m apart.
            # Gives GPS heading in addition to position.
            # Topics: /gps_left/fix  /gps_right/fix  /gps/heading  /gps/heading_imu
            if mode == 'serial':
                config = os.path.join(pkg, 'config', 'gps_serial.yaml')
            else:
                config = os.path.join(pkg, 'config', 'gps_tcp.yaml')

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

    return LaunchDescription([
        gps_mode_arg,
        gps_antennas_arg,
        OpaqueFunction(function=_make_nodes),
    ])
