from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node, PushROSNamespace
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import os



def generate_launch_description():
    pkg_share = FindPackageShare(package='mtt_description').find('mtt_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'urdf_config.rviz')

    robot_namespace = LaunchConfiguration('robot_namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_rviz = LaunchConfiguration('use_rviz')
    use_joint_state_gui = LaunchConfiguration('use_joint_state_gui')

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_namespace',
            default_value='',
            description='Top-level namespace for the MTT robot'
        ),
        DeclareLaunchArgument(
            'use_namespace',
            default_value='false',
            description='Whether to push robot_namespace onto all nodes'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz for robot visualization.'
        ),
        DeclareLaunchArgument(
            'use_joint_state_gui',
            default_value='true',
            description='Launch the joint_state_publisher_gui for interactive joint inspection.'
        ),

        GroupAction([
            PushROSNamespace(condition=IfCondition(use_namespace), namespace=robot_namespace),

            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                parameters=[{
                    # ParameterValue with value_type=str prevents ROS2 Jazzy from
                    # trying to parse the xacro XML output as YAML (which fails).
                    'robot_description': ParameterValue(
                        Command(['xacro ', urdf_path]),
                        value_type=str
                    )
                }]
            ),

            Node(
                package='joint_state_publisher_gui',
                executable='joint_state_publisher_gui',
                condition=IfCondition(use_joint_state_gui)
            ),

            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', rviz_config_path],
                output='screen',
                condition=IfCondition(use_rviz)
            ),
        ]),
    ])
