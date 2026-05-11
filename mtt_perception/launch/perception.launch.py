import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('mtt_perception'), 'config', 'perception.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    publish_filtered = LaunchConfiguration('publish_filtered')
    cloud_merger_anchor_sensor = LaunchConfiguration('cloud_merger_anchor_sensor')
    cloud_merger_max_pair_dt = LaunchConfiguration('cloud_merger_max_pair_dt')
    cloud_merger_tf_timeout = LaunchConfiguration('cloud_merger_tf_timeout')
    cloud_merger_publish_debug_inputs = LaunchConfiguration('cloud_merger_publish_debug_inputs')
    cloud_merger_remove_invalid_points = LaunchConfiguration('cloud_merger_remove_invalid_points')
    cloud_merger_enable_trailer_bbox_filter = LaunchConfiguration('cloud_merger_enable_trailer_bbox_filter')

    trailer_detector = Node(
        package='mtt_perception',
        executable='trailer_detector_node',
        name='trailer_detector_node',
        output='both',
        parameters=[config, {'use_sim_time': use_sim_time}],
    )

    trailer_pose = Node(
        package='mtt_perception',
        executable='trailer_pose_node',
        name='trailer_pose_node',
        output='both',
        parameters=[config, {'use_sim_time': use_sim_time}],
    )

    cloud_merger = Node(
        package='mtt_perception',
        executable='cloud_merger_node',
        name='cloud_merger_node',
        output='both',
        parameters=[
            config,
            {
                'use_sim_time': use_sim_time,
                'publish_filtered': ParameterValue(publish_filtered, value_type=bool),
                'anchor_sensor': cloud_merger_anchor_sensor,
                'max_pair_dt': ParameterValue(cloud_merger_max_pair_dt, value_type=float),
                'tf_timeout': ParameterValue(cloud_merger_tf_timeout, value_type=float),
                'publish_debug_inputs': ParameterValue(cloud_merger_publish_debug_inputs, value_type=bool),
                'remove_invalid_points': ParameterValue(cloud_merger_remove_invalid_points, value_type=bool),
                'enable_trailer_bbox_filter': ParameterValue(
                    cloud_merger_enable_trailer_bbox_filter, value_type=bool),
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation or bag replay clock',
        ),
        DeclareLaunchArgument(
            'publish_filtered',
            default_value='false',
            description='Publish /merged_points_filtered from cloud_merger_node',
        ),
        DeclareLaunchArgument(
            'cloud_merger_anchor_sensor',
            default_value='hesai',
            description='Cloud merger anchor sensor: hesai, rsairy, or either',
        ),
        DeclareLaunchArgument(
            'cloud_merger_max_pair_dt',
            default_value='0.075',
            description='Maximum Hesai/RS-Airy pairing time difference in seconds',
        ),
        DeclareLaunchArgument(
            'cloud_merger_tf_timeout',
            default_value='0.05',
            description='TF lookup timeout per cloud in seconds',
        ),
        DeclareLaunchArgument(
            'cloud_merger_publish_debug_inputs',
            default_value='false',
            description='Publish transformed per-sensor debug clouds',
        ),
        DeclareLaunchArgument(
            'cloud_merger_remove_invalid_points',
            default_value='false',
            description='Drop invalid XYZ points from normalized merged output',
        ),
        DeclareLaunchArgument(
            'cloud_merger_enable_trailer_bbox_filter',
            default_value='false',
            description='Remove trailer body from /merged_points_filtered for mapping/replay',
        ),
        cloud_merger,
        trailer_detector,
        trailer_pose,
    ])
