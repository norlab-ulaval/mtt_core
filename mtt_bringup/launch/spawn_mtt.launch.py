import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.substitutions.command import Command
from launch.substitutions.find_executable import FindExecutable

from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration('robot_name').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context)
    robot_sdf = LaunchConfiguration('robot_sdf')
    pose = {'x': LaunchConfiguration('x_pose'),
            'y': LaunchConfiguration('y_pose'),
            'z': LaunchConfiguration('z_pose'),
            'R': LaunchConfiguration('roll'),
            'P': LaunchConfiguration('pitch'),
            'Y': LaunchConfiguration('yaw')}

    bridge_args = [
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        f'/world/default/model/{robot_name}/pose@geometry_msgs/msg/Pose[gz.msgs.Pose',
        f'/world/default/model/{robot_name}/link/center_lidar_link/sensor/center_lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        f'/world/default/model/{robot_name}/link/front_lidar_link/sensor/front_lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        f'/world/default/model/{robot_name}/link/rear_lidar_link/sensor/rear_lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
    ]

    remappings = [
        (f'/world/default/model/{robot_name}/pose', 'gz_pose'),
        (f'/world/default/model/{robot_name}/link/center_lidar_link/sensor/center_lidar/scan', 'center_lidar/scan'),
        (f'/world/default/model/{robot_name}/link/front_lidar_link/sensor/front_lidar/scan', 'front_lidar/scan'),
        (f'/world/default/model/{robot_name}/link/rear_lidar_link/sensor/rear_lidar/scan', 'rear_lidar/scan'),
    ]

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        namespace=namespace,
        parameters=[{
            'expand_gz_topic_names': True,
            'use_sim_time': True,
        }],
        arguments=bridge_args,
        remappings=remappings,
        output='screen',
    )

    spawn_model = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        namespace=namespace,
        arguments=[
            '-name', robot_name,
            '-string', Command([
                FindExecutable(name='xacro'), ' ', 'namespace:=',
                LaunchConfiguration('namespace'), ' ', robot_sdf]),
            '-x', pose['x'], '-y', pose['y'], '-z', pose['z'],
            '-R', pose['R'], '-P', pose['P'], '-Y', pose['Y']]
    )

    return [bridge, spawn_model]


def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_minimal_tb3_sim')
    mtt_description_dir = get_package_share_directory('mtt_description')

    declare_namespace_cmd = DeclareLaunchArgument('namespace', default_value='', description='Top-level namespace')
    declare_robot_name_cmd = DeclareLaunchArgument('robot_name', default_value='mtt_robot', description='name of the robot')
    declare_robot_sdf_cmd = DeclareLaunchArgument(
        'robot_sdf',
        default_value=os.path.join(mtt_description_dir, 'urdf', 'robot_less_collision.urdf.xacro'),
        description='Full path to the robot xacro file used to spawn the robot in Gazebo')
    
    pose_args = [
        DeclareLaunchArgument('x_pose', default_value='-2.00'),
        DeclareLaunchArgument('y_pose', default_value='-0.50'),
        DeclareLaunchArgument('z_pose', default_value='0.01'),
        DeclareLaunchArgument('roll', default_value='0.00'),
        DeclareLaunchArgument('pitch', default_value='0.00'),
        DeclareLaunchArgument('yaw', default_value='0.00'),
    ]

    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(bringup_dir, 'models'))
    set_env_vars_resources2 = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', str(Path(os.path.join(bringup_dir)).parent.resolve()))

    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_robot_sdf_cmd)
    for arg in pose_args:
        ld.add_action(arg)

    ld.add_action(set_env_vars_resources)
    ld.add_action(set_env_vars_resources2)

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
