import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('mtt_lidar_camera_colorizer'),
        'config', 'colorizer.yaml')

    use_sim_time     = LaunchConfiguration('use_sim_time')
    image_transport  = LaunchConfiguration('image_transport')
    use_depth_check  = LaunchConfiguration('use_depth_check')
    max_range        = LaunchConfiguration('max_range')
    min_range        = LaunchConfiguration('min_range')
    image_timeout    = LaunchConfiguration('image_timeout')

    node = Node(
        package='mtt_lidar_camera_colorizer',
        executable='lidar_camera_colorizer_node',
        name='lidar_camera_colorizer_node',
        output='both',
        parameters=[
            config,
            {
                'use_sim_time':    use_sim_time,
                'image_transport': image_transport,
                'use_depth_check': ParameterValue(use_depth_check, value_type=bool),
                'max_range':       ParameterValue(max_range,       value_type=float),
                'min_range':       ParameterValue(min_range,       value_type=float),
                'image_timeout':   ParameterValue(image_timeout,   value_type=float),
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use bag replay clock (/clock)',
        ),
        DeclareLaunchArgument(
            'image_transport',
            default_value='compressed',
            description='image_transport hint: compressed, zstd, or raw',
        ),
        DeclareLaunchArgument(
            'use_depth_check',
            default_value='false',
            description='Enable ZED depth-based occlusion filtering',
        ),
        DeclareLaunchArgument(
            'max_range',
            default_value='50.0',
            description='Ignore LiDAR points beyond this range (meters)',
        ),
        DeclareLaunchArgument(
            'min_range',
            default_value='0.5',
            description='Ignore LiDAR points closer than this range (meters)',
        ),
        DeclareLaunchArgument(
            'image_timeout',
            default_value='0.5',
            description='Maximum age of cached image relative to cloud stamp (seconds)',
        ),
        node,
    ])
