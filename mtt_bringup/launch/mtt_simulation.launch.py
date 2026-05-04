import os
import re
import tempfile

import logging
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    AppendEnvironmentVariable,

)
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnShutdown, OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
import xacro


def launch_sim_description(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_sdf_path = LaunchConfiguration('robot_sdf').perform(context)

    robot_description_content = xacro.process_file(
        robot_sdf_path,
        mappings={'namespace': namespace},
    ).toxml()
    robot_description_content = re.sub(
        r'file://[^"]*share/mtt_description/',
        'package://mtt_description/',
        robot_description_content,
    )

    robot_description = ParameterValue(robot_description_content, value_type=str)

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=namespace,
            output='screen',
            parameters=[
                {
                    'use_sim_time': use_sim_time,
                    'robot_description': robot_description,
                    'frame_prefix': PythonExpression(
                        ["'", namespace, "/' if '", namespace, "' else ''"]
                    ),
                }
            ],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        )
    ]


def generate_launch_description():
    # logging.getLogger().setLevel(logging.WARN)

    # TODO: test if this is still necessary
    # Without this, the world sdf file has trouble finding gz_ros2_control
    os.environ["GZ_SIM_SYSTEM_PLUGIN_PATH"] = os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "") + ":/opt/ros/jazzy/lib"
    
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    mtt_description_dir = get_package_share_directory('mtt_description')
    mtt_bringup_dir = get_package_share_directory('mtt_bringup')
    mtt_bringup_launch_dir = os.path.join(mtt_bringup_dir, 'launch')

    use_rviz = LaunchConfiguration('use_rviz', default='True')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    use_sim_time = LaunchConfiguration('use_sim_time')
    base_frame = LaunchConfiguration('base_frame')
    odom_frame = LaunchConfiguration('odom_frame')
    odom_topic = LaunchConfiguration('odom_topic')
    world = LaunchConfiguration('world')
    pose = {
        'x': LaunchConfiguration('x_pose', default='-2.00'),
        'y': LaunchConfiguration('y_pose', default='-0.50'),
        'z': LaunchConfiguration('z_pose', default='0.01'),
        'R': LaunchConfiguration('roll', default='0.00'),
        'P': LaunchConfiguration('pitch', default='0.00'),
        'Y': LaunchConfiguration('yaw', default='0.00'),
    }
    robot_name = LaunchConfiguration('robot_name')
    robot_sdf = LaunchConfiguration('robot_sdf')

    log_level = LaunchConfiguration("log_level")

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="warn", description="Logging level"
    )
    ######### Temp zone

    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    slam = LaunchConfiguration('slam')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')

    use_respawn = LaunchConfiguration('use_respawn')

    # 
    
    # Launch configuration variables specific to simulation

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack',
    )

    declare_slam_cmd = DeclareLaunchArgument(
        'slam', default_value='False', description='Whether run a SLAM'
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(nav2_bringup_dir, 'maps', 'tb3_sandbox.yaml'),
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(nav2_bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack',
    )
    
    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='True',
        description='Whether to use composed bringup',
    )

    # 
    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.',
    )
    
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='True', description='Whether to start RVIZ'
    )
    ###########


    # TODO: to remove? 
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    
    # TODO: to remove
    headless = LaunchConfiguration('headless')

    # TODO: to remove
    use_simulator = LaunchConfiguration('use_simulator')

    
    # TODO: to remove
    declare_simulator_cmd = DeclareLaunchArgument(
        'headless', default_value='False', description='Whether to execute gzclient)'
    )
    
    # TODO: to remove
    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator',
        default_value='True',
        description='Whether to start the simulator',
    )
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_base_frame_cmd = DeclareLaunchArgument(
        'base_frame',
        default_value='base_footprint',
        description='Base frame used as the child of odom in simulation TF',
    )

    declare_odom_frame_cmd = DeclareLaunchArgument(
        'odom_frame',
        default_value='odom',
        description='Odom frame published by the simulation odom bridge',
    )

    declare_odom_topic_cmd = DeclareLaunchArgument(
        'odom_topic',
        default_value='mtt_odometry',
        description='Odometry topic published from Gazebo ground-truth pose',
    )
    
    # TODO: check if other config isnt better
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(mtt_description_dir, 'rviz', 'urdf_config.rviz'),
        description='Full path to the RVIZ config file to use',
    )
    
    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher',
    )
    
    # Setting the robot name
    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name', default_value='mtt_robot', description='name of the robot'
    )

    # tip:  mtt sdf: gz sdf -p robot.urdf.xacro > robot.sdf
    declare_robot_sdf_cmd = DeclareLaunchArgument(
        'robot_sdf',
        default_value=os.path.join(mtt_description_dir, 'urdf', 'robot_less_collision.urdf.xacro'),
        # default_value=os.path.join(mtt_description_dir, 'urdf', 'robot_no_collision.urdf.xacro'),
        # default_value=os.path.join(mtt_description_dir, 'urdf', 'robot.urdf.xacro'),

        description='Full path to the robot xacro file used to spawn the robot in Gazebo',
    )

    # RVIZ
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(mtt_bringup_launch_dir, 'mtt_rviz.launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': use_namespace,
            'use_sim_time': use_sim_time,
            'rviz_config': rviz_config_file,
            'log_level': log_level,
        }.items(),
    )

    model_path = os.path.join(
        get_package_share_directory("mtt_description"), 'models'
    )

    model_path = os.path.abspath(model_path)

    gz_sim_environment = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f'{model_path}:{os.environ.get("GZ_SIM_RESOURCE_PATH", "")}'
    )
    # TODO: change to a world used for the mtt
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(mtt_description_dir, 'worlds', 'test_world.sdf'),
        description='Full path to world model file to load',
    )      
    # The SDF file for the world is a xacro file because we wanted to
    # conditionally load the SceneBroadcaster plugin based on wheter we're
    # running in headless mode. But currently, the Gazebo command line doesn't
    # take SDF strings for worlds, so the output of xacro needs to be saved into
    # a temporary file and passed to Gazebo.

    # something in those lines for test_word incapacitate the lidar
    world_sdf = tempfile.mktemp(prefix='nav2_', suffix='.sdf')
    world_sdf_xacro = ExecuteProcess(
        cmd=['xacro', '-o', world_sdf, ['headless:=', headless], world], output='screen')

    start_gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', os.path.join(mtt_description_dir, 'worlds', 'test_world.sdf')],
        output='screen',
        condition=IfCondition(use_simulator)
    )

    gazebo_server = RegisterEventHandler(
        OnProcessExit(
            target_action=world_sdf_xacro,
            on_exit=[start_gz_sim]
        )
    )
    
    remove_temp_sdf_file = RegisterEventHandler(event_handler=OnShutdown(
        on_shutdown=[
            OpaqueFunction(function=lambda _: os.remove(world_sdf) if os.path.exists(world_sdf) else None)
        ]))

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                        'launch',
                        'gz_sim.launch.py')
        ),
        condition=IfCondition(PythonExpression(
            [use_simulator, ' and not ', headless])),
        launch_arguments={'gz_args': ['-v4 -g ']}.items(),
    )


    gz_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mtt_bringup_dir, 'launch', 'spawn_mtt.launch.py')),
            # os.path.join(sim_dir, 'launch', 'spawn_tb3.launch.py')),
        launch_arguments={'namespace': namespace,
                          'use_sim_time': use_sim_time,
                          'robot_name': robot_name,
                          'robot_sdf': robot_sdf,
                          'x_pose': pose['x'],
                          'y_pose': pose['y'],
                          'z_pose': pose['z'],
                          'roll': pose['R'],
                          'pitch': pose['P'],
                          'yaw': pose['Y']}.items())
    


    mtt_controller_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(mtt_bringup_dir, 'launch', 'mtt_controller.launch.py')),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': use_namespace,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    simulation_odom_cmd = Node(
        package='mtt_bringup',
        executable='odom_publisher_simul.py',
        name='odom_publisher_simul',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'base_frame': base_frame,
                'odom_frame': odom_frame,
                'odom_topic': odom_topic,
                'pose_topic': 'gz_pose',
                'source_child_frame': robot_name,
                'source_frame': 'default',
            }
        ],
    )

    ld = LaunchDescription()
    ld.add_action(declare_log_level_cmd)

    # Optionnals
    # temp zone
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    # 

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_base_frame_cmd)
    ld.add_action(declare_odom_frame_cmd)
    ld.add_action(declare_odom_topic_cmd)

    # temp zone
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    # 
    
    

    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd) # check if order important

    ld.add_action(gz_sim_environment)
    # temp zone
    ld.add_action(declare_use_rviz_cmd)
    # 

    # TODO: to remove
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_robot_sdf_cmd)

    # temp zone
    ld.add_action(declare_use_respawn_cmd)
    # 
    
    ld.add_action(world_sdf_xacro)
    ld.add_action(remove_temp_sdf_file)
    ld.add_action(OpaqueFunction(function=launch_sim_description))
    ld.add_action(gz_robot)
    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)
    ld.add_action(simulation_odom_cmd)

    ld.add_action(mtt_controller_cmd)

    return ld
    
