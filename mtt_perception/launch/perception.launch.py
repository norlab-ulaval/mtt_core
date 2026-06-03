import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('mtt_perception'), 'config', 'perception.yaml')

    use_sim_time                       = LaunchConfiguration('use_sim_time')
    enable_new_trailer_estimator       = LaunchConfiguration('enable_new_trailer_estimator')
    publish_filtered                   = LaunchConfiguration('publish_filtered')
    enable_cloud_merger                = LaunchConfiguration('enable_cloud_merger')
    cloud_merger_anchor_sensor         = LaunchConfiguration('cloud_merger_anchor_sensor')
    cloud_merger_max_pair_dt           = LaunchConfiguration('cloud_merger_max_pair_dt')
    cloud_merger_tf_timeout            = LaunchConfiguration('cloud_merger_tf_timeout')
    cloud_merger_publish_reliable_raw  = LaunchConfiguration('cloud_merger_publish_reliable_raw')
    cloud_merger_publish_debug_inputs  = LaunchConfiguration('cloud_merger_publish_debug_inputs')
    cloud_merger_remove_invalid_points = LaunchConfiguration('cloud_merger_remove_invalid_points')
    cloud_merger_hesai_stride          = LaunchConfiguration('cloud_merger_hesai_stride')
    cloud_merger_rsairy_stride         = LaunchConfiguration('cloud_merger_rsairy_stride')
    cloud_merger_rsairy_inject_every_n = LaunchConfiguration('cloud_merger_rsairy_inject_every_n')
    cloud_merger_enable_self_bbox_filter    = LaunchConfiguration('cloud_merger_enable_self_bbox_filter')
    cloud_merger_enable_trailer_bbox_filter = LaunchConfiguration('cloud_merger_enable_trailer_bbox_filter')

    # ── trailer_detector_node (V1.5) — ALWAYS launched ──────────────────────
    # Articulation angle KF: fast, proven, independent.
    # Feeds /trailer/articulation_angle to both legacy and new estimator.
    trailer_detector = Node(
        package='mtt_perception',
        executable='trailer_detector_node',
        name='trailer_detector_node',
        output='both',
        parameters=[config, {'use_sim_time': use_sim_time}],
    )

    # ── NEW: mtt_trailer_estimator_node (V1.0) — unified EKF ──────────────────
    # Enabled when enable_new_trailer_estimator=true (default).
    # Subscribes to /trailer/articulation_angle from trailer_detector_node.
    # Publishes: /trailer/pose, /trailer/pose_in_map, /trailer/odom,
    #            /trailer/body_markers, /trailer/trailer_roi_cloud,
    #            /trailer/articulation_angle (back-computed), TF map→trailer_body.
    trailer_estimator = Node(
        package='mtt_perception',
        executable='mtt_trailer_estimator_node',
        name='mtt_trailer_estimator_node',
        output='both',
        condition=IfCondition(enable_new_trailer_estimator),
        parameters=[config, {'use_sim_time': use_sim_time}],
    )

    # ── LEGACY: trailer_pose_node (V4.0) — for A/B comparison only ──────────
    # Enabled when enable_new_trailer_estimator=false.
    # Do not run both simultaneously — they publish to the same topics.
    trailer_pose = Node(
        package='mtt_perception',
        executable='trailer_pose_node',
        name='trailer_pose_node',
        output='both',
        condition=UnlessCondition(enable_new_trailer_estimator),
        parameters=[config, {'use_sim_time': use_sim_time}],
    )

    # ── cloud_merger_node — unchanged ─────────────────────────────────────────
    cloud_merger = Node(
        package='mtt_perception',
        executable='cloud_merger_node',
        name='cloud_merger_node',
        output='both',
        condition=IfCondition(enable_cloud_merger),
        parameters=[
            config,
            {
                'use_sim_time': use_sim_time,
                'publish_filtered': ParameterValue(publish_filtered, value_type=bool),
                'anchor_sensor': cloud_merger_anchor_sensor,
                'max_pair_dt': ParameterValue(cloud_merger_max_pair_dt, value_type=float),
                'tf_timeout': ParameterValue(cloud_merger_tf_timeout, value_type=float),
                'publish_reliable_raw_for_mapping': ParameterValue(
                    cloud_merger_publish_reliable_raw, value_type=bool),
                'publish_debug_inputs': ParameterValue(
                    cloud_merger_publish_debug_inputs, value_type=bool),
                'remove_invalid_points': ParameterValue(
                    cloud_merger_remove_invalid_points, value_type=bool),
                'hesai_stride': ParameterValue(cloud_merger_hesai_stride, value_type=int),
                'rsairy_stride': ParameterValue(cloud_merger_rsairy_stride, value_type=int),
                'rsairy_inject_every_n': ParameterValue(
                    cloud_merger_rsairy_inject_every_n, value_type=int),
                'enable_self_bbox_filter': ParameterValue(
                    cloud_merger_enable_self_bbox_filter, value_type=bool),
                'enable_trailer_bbox_filter': ParameterValue(
                    cloud_merger_enable_trailer_bbox_filter, value_type=bool),
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation or bag replay clock'),
        DeclareLaunchArgument(
            'enable_new_trailer_estimator', default_value='true',
            description=(
                'true  → launch mtt_trailer_estimator_node (EKF V1.0, unified).\n'
                'false → launch legacy trailer_pose_node (V4.0) for A/B comparison.\n'
                'trailer_detector_node is always launched regardless of this flag.'
            )),
        DeclareLaunchArgument(
            'enable_cloud_merger', default_value='true',
            description='Launch cloud_merger_node'),
        DeclareLaunchArgument(
            'publish_filtered', default_value='true',
            description='Publish /merged_points_filtered from cloud_merger_node'),
        DeclareLaunchArgument(
            'cloud_merger_anchor_sensor', default_value='hesai',
            description='Cloud merger anchor sensor: hesai, rsairy, or either'),
        DeclareLaunchArgument(
            'cloud_merger_max_pair_dt', default_value='0.075',
            description='Max Hesai/RS-Airy pairing time difference (s)'),
        DeclareLaunchArgument(
            'cloud_merger_tf_timeout', default_value='0.05',
            description='TF lookup timeout per cloud (s)'),
        DeclareLaunchArgument(
            'cloud_merger_publish_reliable_raw', default_value='false',
            description='Publish /merged_points_reliable with reliable QoS'),
        DeclareLaunchArgument(
            'cloud_merger_publish_debug_inputs', default_value='false',
            description='Publish transformed per-sensor debug clouds'),
        DeclareLaunchArgument(
            'cloud_merger_remove_invalid_points', default_value='false',
            description='Drop invalid XYZ from normalized merged output'),
        DeclareLaunchArgument(
            'cloud_merger_hesai_stride', default_value='1',
            description='Keep one out of N Hesai points'),
        DeclareLaunchArgument(
            'cloud_merger_rsairy_stride', default_value='1',
            description='Keep one out of N RS-Airy points'),
        DeclareLaunchArgument(
            'cloud_merger_rsairy_inject_every_n', default_value='1',
            description='Inject RS-Airy into one out of N Hesai-anchored frames'),
        DeclareLaunchArgument(
            'cloud_merger_enable_self_bbox_filter', default_value='true',
            description='Apply chassis/cage/trailer bbox filters'),
        DeclareLaunchArgument(
            'cloud_merger_enable_trailer_bbox_filter', default_value='true',
            description='Remove trailer body from /merged_points_filtered'),
        cloud_merger,
        trailer_detector,
        trailer_estimator,
        trailer_pose,
    ])
