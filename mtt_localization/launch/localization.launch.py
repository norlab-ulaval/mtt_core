import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('mtt_localization'), 'config', 'localization.yaml')

    factor_graph = Node(
        package='mtt_localization',
        executable='factor_graph_node',
        name='factor_graph_node',
        output='both',
        parameters=[config],
    )

    return LaunchDescription([factor_graph])
