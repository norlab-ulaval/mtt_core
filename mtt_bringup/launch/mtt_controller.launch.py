import launch
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart
from launch.actions import (
    DeclareLaunchArgument,
)

import launch_ros
import os
import xacro

def generate_launch_description():
    rvizRelativePath = "config/config.rviz"

    # absolute package path
    packageName = 'mtt_bringup'
    mtt_description_package = 'mtt_description'

    pkgPath = launch_ros.substitutions.FindPackageShare(package=packageName).find(packageName)
    description_path = launch_ros.substitutions.FindPackageShare(package=mtt_description_package).find(mtt_description_package)

    # absolute Xacro model path
    xacroModelPath = os.path.join(description_path, 'urdf', 'robot.urdf.xacro')
    ros2controlRelativePath = 'config/control_config.yaml'


    # absolute rviz config file path
    rvizConfigPath=os.path.join(pkgPath, rvizRelativePath)

    # controller config file
    ros2controlPath=os.path.join(pkgPath, ros2controlRelativePath)

    # here, for verification, print the xacro model path 
    print(xacroModelPath)

    # get the robot description from the xacro model file 
    robot_desc = xacro.process_file(xacroModelPath).toxml()

    # define a parameter with the robot xacro description 
    robot_description = {'robot_description': robot_desc}

    # Declare arguments 
    declared_arguments = [] 
    declared_arguments.append(
        launch.actions.DeclareLaunchArgument (name="gui", default_value="true", description="Start the RViz2 GUI."))
    
    # Initialize Arguments
    gui = LaunchConfiguration("gui")


    # for starting Gazebo
    gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [launch_ros.substitutions.FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments=[("gz_args", " -r -v 3 empty.sdf")],
        condition=launch.conditions.IfCondition(gui))
    


    gazebo_headless = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [launch_ros.substitutions.FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
            ),
            launch_arguments=[("gz_args", ["--headless-rendering -s -r -v 3 empty.sdf"])], 
            condition=launch.conditions.UnlessCondition (gui))
    

    # Gazebo bridge
    gazebo_bridge = launch_ros.actions.Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen")
    
    gz_spawn_entity = launch_ros.actions.Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "/robot_description",
            "-name",
            "robot_system_position",
            "-allow_renaming",
            "true"])
    

    # TODO: put the one from mtt_simulation here instead
    # robot state publisher node
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher', 
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description])
    

    # rviz node
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rvizConfigPath])
    
    
    # ros2_control node
    # control_node = launch_ros.actions.Node(
    #     package="controller_manager", 
    #     executable="ros2_control_node",
    #     parameters=[robot_description, ros2controlPath],
    #     output="both",
    # )

    # joint state broadcaster
    joint_state_broadcaster_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
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