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
        parameters=[config, {'use_sim_time': use_sim_time}],
    )

    # ── Trailer Localizer — trailer SE(3) pose in map ──
    # Subscribes: localization/odom (T_map_tractor from factor_graph_node)
    #             /mtt/articulation_state (raw φ fallback)
    #             localization/articulation_angle (ISAM2 φ, preferred when fresh)
    # Publishes:  trailer/odom  (nav_msgs/Odometry, map frame, 6×6 covariance)
    #             trailer/pose_in_map (PoseWithCovarianceStamped, map frame)
    #             TF: map → trailer_body
    # Math: T_map_trailer = T_map_base_footprint · A·Rz(π/2+φ)·B
    #       Σ_trailer = Ad(Δ⁻¹)·Σ_tractor·Ad(Δ⁻¹)ᵀ + J_φ·σ²_φ·J_φᵀ
    trailer_localizer = Node(
        package='mtt_localization',
        executable='mtt_trailer_localizer_node_exe',
        name='mtt_trailer_localizer_node',
        output='both',
        parameters=[config, {'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (bag replay) clock',
        ),
        factor_graph,
        trailer_localizer,
    ])
