import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('mtt_perception'), 'config', 'perception.yaml')

    trailer_detector = Node(
        package='mtt_perception',
        executable='trailer_detector_node',
        name='trailer_detector_node',
        output='both',
        parameters=[config],
    )

    return LaunchDescription([trailer_detector])
