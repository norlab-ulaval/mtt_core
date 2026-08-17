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
    use_visual_odom = LaunchConfiguration('use_visual_odom')
    visual_odom_topic = LaunchConfiguration('visual_odom_topic')

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
            'use_visual_odom': use_visual_odom,
            'visual_odom_topic': visual_odom_topic,
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
            default_value='false',
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
        DeclareLaunchArgument(
            'use_visual_odom',
            default_value='true',
            description=(
                'Fuse a visual-odometry BetweenFactor. Enabled 2026-07-27 on the ZED '
                'onboard-VIO path (visual_odom_topic default below), after the '
                'extrinsic-conjugation fix in factor_graph_node.cpp (corrects for the '
                '~0.605m lever arm between base_footprint and zed_camera_link, which the '
                'raw delta ignored) landed with the vo_state gate + kinematic jump '
                'rejection also in place. Empirical A/B validation on the ice-rink bag '
                '(2026-07-27) was inconclusive on THIS bag specifically -- the correction\'s '
                'per-step magnitude was swamped by ~10-25cm of ZED VIO/ICP noise because '
                'that session\'s rotation rates were modest -- but the extrinsic math was '
                'independently confirmed exact against the bag\'s own /tf_static (0.605m, '
                '0deg rotation), and the failure modes are gated (see vo_state gate + '
                'kinematic rejection in visual_odom_callback). Enabled deliberately despite '
                'the inconclusive A/B result, not because it was proven decisively better. '
                'Isaac ROS Visual SLAM (/isaac/vslam/odometry, GPU-accelerated) is a '
                'higher-quality alternative gated behind the live_robot isaac_vslam '
                'compose profile plus a still-missing status-republishing bridge (see '
                'visual_status_topic in factor_graph_node.cpp) -- not switched to yet.\n'
                'IMPORTANT: this launch argument\'s default, not localization.yaml\'s '
                'use_visual_odom value, is authoritative here -- Node(parameters=[config, '
                '{...}]) below lists the launch-arg dict AFTER the YAML, so it always wins '
                '(same override-precedence class of bug root-caused and fixed for ZED\'s '
                'publish_imu_tf on 2026-07-26; do not let the two drift out of sync).'
            ),
        ),
        DeclareLaunchArgument(
            'visual_odom_topic',
            default_value='isaac/vslam/odometry',
            description=(
                'Topic for the visual-odometry BetweenFactor (only read when '
                'use_visual_odom=true). Default is Isaac ROS Visual SLAM (2026-07-27, '
                'isaac_vslam is now a default compose service, not just opt-in profile) -- '
                'GPU-accelerated, ~60Hz, real loop-closure SLAM (vs the ZED SDK\'s own onboard '
                'VIO at ~13Hz). If isaac_vslam is not running (e.g. bag replay without an '
                'isaac-profile recording, or a machine without the GPU sidecar image), this '
                'topic simply never publishes -- factor_graph_node degrades gracefully to no '
                'visual factor, matching every other optional source in this graph. Override '
                'to zed/zed_node/odom for the ZED\'s own onboard VIO instead (this is what was '
                'empirically validated via bag replay 2026-07-27 -- Isaac itself has not been '
                'validated end-to-end through this exact code path yet, only VIO tracking in '
                'isolation and the extrinsic math via the ZED path, which shares the same '
                'zed_camera_link child_frame_id and lever arm).'
            ),
        ),
        factor_graph,
        trailer_localizer,
    ])
