import launch
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
)

import launch_ros
import os


def generate_launch_description():
    # absolute package path
    packageName = 'mtt_bringup'

    pkgPath = launch_ros.substitutions.FindPackageShare(package=packageName).find(packageName)
    ros2controlRelativePath = 'config/control_config.yaml'

    # controller config file
    ros2controlPath = os.path.join(pkgPath, ros2controlRelativePath)


    # Declare arguments 
    declared_arguments = [] 
    declared_arguments.append(
        launch.actions.DeclareLaunchArgument (name="gui", default_value="true", description="Start the RViz2 GUI."))
    declared_arguments.append(
        launch.actions.DeclareLaunchArgument(
            name="enable_joint_state_broadcaster",
            default_value="true",
            description="Spawn the Gazebo joint_state_broadcaster",
        )
    )
    
    # Initialize Arguments
    gui = LaunchConfiguration("gui")
    enable_joint_state_broadcaster = LaunchConfiguration("enable_joint_state_broadcaster")


    # joint state broadcaster
    joint_state_broadcaster_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        condition=launch.conditions.IfCondition(enable_joint_state_broadcaster),
    )

    robot_controller_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["wheel_group_controller", "--param-file", ros2controlPath],
        output="screen"
    )

    yaw_controller_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["yaw_controller", "--param-file", ros2controlPath],
        output="screen"
    )

    mtt_controller_interface = launch_ros.actions.Node(
        package='mtt_bringup',
        executable='mtt_controller_interface.py',
        name='mtt_controller_interface',
        output='screen'
    )

    
    ld = launch.LaunchDescription()
    ld.add_action(declared_arguments[0])
    ld.add_action(declared_arguments[1])

    # Only necessary when this launch is launched alone
    # ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_broadcaster_spawner)


    # ld.add_action(gazebo)
    # ld.add_action(gazebo_headless)
    # ld.add_action(gazebo_bridge)
    # ld.add_action(gz_spawn_entity)
    # ld.add_action(rviz_node)
    # ld.add_action(control_node)
    ld.add_action(robot_controller_spawner)
    ld.add_action(mtt_controller_interface)
    ld.add_action(yaw_controller_spawner)
    
    return ld
