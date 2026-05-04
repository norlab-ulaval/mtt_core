import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('mtt_perception'), 'config', 'perception.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    trailer_detector = Node(
        package='mtt_perception',
        executable='trailer_detector_node',
        name='trailer_detector_node',
        output='both',
        parameters=[config, {'use_sim_time': use_sim_time}],
    )

    cloud_merger = Node(
        package='mtt_perception',
        executable='cloud_merger_node',
        name='cloud_merger_node',
        output='both',
        parameters=[config, {'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation or bag replay clock',
        ),
        cloud_merger,
        trailer_detector,
    ])
