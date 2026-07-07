import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('mtt_localization'), 'config', 'localization.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    track_odom_topic = LaunchConfiguration('track_odom_topic')
    articulation_topic = LaunchConfiguration('articulation_topic')
    use_gps = LaunchConfiguration('use_gps')
    use_gps_heading = LaunchConfiguration('use_gps_heading')

    # ── ISAM2 Factor Graph — tractor SE(3) pose + optimised φ ──
    # Subscribes: IMU, track odom, GPS, LiDAR odom, /mtt/articulation_state,
    #             /trailer/pose (base_link frame, from perception)
    # Publishes:  localization/odom (with 6×6 covariance from ISAM2 marginals)
    #             localization/articulation_angle (optimised φ, std_msgs/Float64)
    factor_graph = Node(
        package='mtt_localization',
        executable='factor_graph_node',
        name='factor_graph_node',
        output='both',
        parameters=[config, {
            'use_sim_time': use_sim_time,
            'track_odom_topic': track_odom_topic,
            'articulation_topic': articulation_topic,
            'use_gps': use_gps,
            'use_gps_heading': use_gps_heading,
        }],
    )

    # ── Trailer Localizer — trailer SE(3) pose in map ──
    # Subscribes: localization/odom (T_map_tractor from factor_graph_node)
    #             /mtt/articulation_state (raw φ fallback)
    #             localization/articulation_angle (ISAM2 φ, preferred when fresh)
    # Publishes:  trailer/odom  (nav_msgs/Odometry, map frame, 6×6 covariance)
    #             trailer/pose_in_map (PoseWithCovarianceStamped, map frame)
    #             TF: map → trailer_body
    # Math: T_map_trailer = T_map_base_footprint · A·Rz(yaw_rest_rad+φ)·B
    #       (yaw_rest_rad/pitch_rest_rad/roll_rest_rad are params in localization.yaml,
    #        must match mtt_joint_state_builder_node's rest angles — see that file)
    #       Σ_trailer = Ad(Δ⁻¹)·Σ_tractor·Ad(Δ⁻¹)ᵀ + J_φ·σ²_φ·J_φᵀ
    trailer_localizer = Node(
        package='mtt_localization',
        executable='mtt_trailer_localizer_node_exe',
        name='mtt_trailer_localizer_node',
        output='both',
        parameters=[config, {
            'use_sim_time': use_sim_time,
            'articulation_topic': articulation_topic,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (bag replay) clock',
        ),
        DeclareLaunchArgument(
            'track_odom_topic',
            default_value='mtt_odometry',
            description=(
                'Live default is the raw driver topic. Bag replay MUST override to '
                'mtt_odometry/runtime (the recomputed, replay-consistent stream) — the '
                'bare topic is stale/inconsistent bag-recorded data under replay.'
            ),
        ),
        DeclareLaunchArgument(
            'articulation_topic',
            default_value='/mtt/articulation_state',
            description=(
                'Live default is the raw driver topic. Bag replay MUST override to '
                'mtt/articulation_state/runtime, matching '
                'mtt_joint_state_builder_node — same staleness caveat as '
                'track_odom_topic.'
            ),
        ),
        DeclareLaunchArgument(
            'use_gps',
            default_value='true',
            description='Fuse GPS position (GPSFactor). Disable for bags with no/bad GPS fix.',
        ),
        DeclareLaunchArgument(
            'use_gps_heading',
            default_value='true',
            description=(
                'Fuse GPS dual-antenna heading as an absolute rotation prior. Requires '
                'gps_antennas=dual — on a single-antenna session (see session_info.yaml) '
                'the recorded /gps/heading is not a valid heading measurement. Because '
                'this is a tight PriorFactor on rotation (not just a soft residual), '
                'feeding it garbage on a single-antenna bag injects contradictory '
                'absolute-orientation constraints across keyframes as the robot turns, '
                'which manifests as IndeterminantLinearSystemException near the IMU bias '
                '— set to false for single-antenna sessions.'
            ),
        ),
        factor_graph,
        trailer_localizer,
    ])
