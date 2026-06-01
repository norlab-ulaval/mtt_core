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
    declared_arguments.append(
        launch.actions.DeclareLaunchArgument(
            name="spawn_motion_controllers",
            default_value="true",
            description="Spawn wheel and yaw controllers if they are not already managed by Gazebo startup",
        )
    )
    declared_arguments.append(
        launch.actions.DeclareLaunchArgument(
            name="controller_gear_ratio",
            default_value="1.0",
            description="Velocity scale applied by the Gazebo cmd_vel to wheel command bridge",
        )
    )
    declared_arguments.append(
        launch.actions.DeclareLaunchArgument(
            name="controller_wheel_command_sign",
            default_value="-1.0",
            description="Wheel command sign applied by the Gazebo cmd_vel bridge",
        )
    )
    declared_arguments.append(
        launch.actions.DeclareLaunchArgument(
            name="controller_yaw_command_sign",
            default_value="1.0",
            description="Yaw/articulation command sign applied by the Gazebo cmd_vel bridge",
        )
    )
    declared_arguments.append(
        launch.actions.DeclareLaunchArgument(
            name="controller_yaw_command_mode",
            default_value="position_servo",
            description="How the Gazebo bridge drives the yaw velocity controller: position_servo or velocity",
        )
    )
    declared_arguments.append(
        launch.actions.DeclareLaunchArgument(
            name="controller_yaw_position_kp",
            default_value="5.0",
            description="P gain from desired articulation angle to yaw joint velocity command",
        )
    )
    declared_arguments.append(
        launch.actions.DeclareLaunchArgument(
            name="controller_yaw_velocity_limit_rad_s",
            default_value="2.5",
            description="Yaw joint velocity command limit for position_servo mode",
        )
    )
    
    # Initialize Arguments
    gui = LaunchConfiguration("gui")
    enable_joint_state_broadcaster = LaunchConfiguration("enable_joint_state_broadcaster")
    spawn_motion_controllers = LaunchConfiguration("spawn_motion_controllers")
    controller_gear_ratio = LaunchConfiguration("controller_gear_ratio")
    controller_wheel_command_sign = LaunchConfiguration("controller_wheel_command_sign")
    controller_yaw_command_sign = LaunchConfiguration("controller_yaw_command_sign")
    controller_yaw_command_mode = LaunchConfiguration("controller_yaw_command_mode")
    controller_yaw_position_kp = LaunchConfiguration("controller_yaw_position_kp")
    controller_yaw_velocity_limit_rad_s = LaunchConfiguration("controller_yaw_velocity_limit_rad_s")


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
        output="screen",
        condition=launch.conditions.IfCondition(spawn_motion_controllers),
    )

    yaw_controller_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["yaw_controller", "--param-file", ros2controlPath],
        output="screen",
        condition=launch.conditions.IfCondition(spawn_motion_controllers),
    )

    mtt_controller_interface = launch_ros.actions.Node(
        package='mtt_bringup',
        executable='mtt_controller_interface.py',
        name='mtt_controller_interface',
        output='screen',
        parameters=[{
            "gear_ratio": controller_gear_ratio,
            "wheel_command_sign": controller_wheel_command_sign,
            "yaw_command_sign": controller_yaw_command_sign,
            "yaw_command_mode": controller_yaw_command_mode,
            "yaw_position_kp": controller_yaw_position_kp,
            "yaw_velocity_limit_rad_s": controller_yaw_velocity_limit_rad_s,
        }],
    )

    
    ld = launch.LaunchDescription()
    ld.add_action(declared_arguments[0])
    ld.add_action(declared_arguments[1])
    ld.add_action(declared_arguments[2])
    ld.add_action(declared_arguments[3])
    ld.add_action(declared_arguments[4])
    ld.add_action(declared_arguments[5])
    ld.add_action(declared_arguments[6])
    ld.add_action(declared_arguments[7])
    ld.add_action(declared_arguments[8])

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
