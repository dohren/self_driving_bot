from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessStart
import os
import tempfile
from launch.actions import TimerAction
from pathlib import Path


def generate_launch_description():
    # Directories and files
    sim_dir = get_package_share_directory('nav2_minimal_tb4_sim')
    self_driving_dir = get_package_share_directory('self_driving_bot')
    urdf_file = os.path.join(self_driving_dir, 'urdf', 'ikea_table.urdf')
    servo_controller_config = os.path.join(self_driving_dir, 'config', 'servo_controllers.yaml')

    # Launch configurations
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_simulator = LaunchConfiguration('use_simulator')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')
    pose = {
        'x': LaunchConfiguration('x_pose', default='-8.00'),
        'y': LaunchConfiguration('y_pose', default='0.00'),
        'z': LaunchConfiguration('z_pose', default='0.01'),
        'R': LaunchConfiguration('roll', default='0.00'),
        'P': LaunchConfiguration('pitch', default='0.00'),
        'Y': LaunchConfiguration('yaw', default='0.00'),
    }
    robot_name = LaunchConfiguration('robot_name')
    robot_sdf = LaunchConfiguration('robot_sdf')

    # Remappings for tf topics
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # Declare launch arguments (same as before)
    declare_namespace_cmd = DeclareLaunchArgument('namespace', default_value='', description='Top-level namespace')
    declare_use_sim_time_cmd = DeclareLaunchArgument('use_sim_time', default_value='True', description='Use simulation clock')
    declare_rviz_config_file_cmd = DeclareLaunchArgument('rviz_config_file',
                                                        default_value=os.path.join(self_driving_dir, 'rviz', 'config.rviz'),
                                                        description='Full path to the RVIZ config file to use')
    declare_use_simulator_cmd = DeclareLaunchArgument('use_simulator', default_value='True', description='Whether to start the simulator')
    declare_use_robot_state_pub_cmd = DeclareLaunchArgument('use_robot_state_pub', default_value='True', description='Start robot_state_publisher?')
    declare_simulator_cmd = DeclareLaunchArgument('headless', default_value='False', description='Whether to execute gzclient)')
    declare_world_cmd = DeclareLaunchArgument('world', default_value=os.path.join( self_driving_dir, 'worlds', 'depot.sdf'), description='Full path to world model file')
    declare_robot_name_cmd = DeclareLaunchArgument('robot_name', default_value='nav2_turtlebot4', description='Robot name')
    declare_robot_sdf_cmd = DeclareLaunchArgument('robot_sdf', default_value=os.path.join(self_driving_dir, 'urdf', 'ikea_table.urdf'), description='Robot sdf file')

    # Nodes

    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': Path(urdf_file).read_text()}], # Command(['xacro', ' ', urdf_file])}],
        remappings=remappings,
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[servo_controller_config],
        output='screen'
    )


    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )
    
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=remappings,
    )

    # Temp SDF for world
    world_sdf = tempfile.mktemp(prefix='nav2_', suffix='.sdf')
    world_sdf_xacro = ExecuteProcess(
        cmd=['xacro', '-o', world_sdf, ['headless:=', headless], world])

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r -s ', world_sdf]}.items(),
        condition=IfCondition(use_simulator))

    remove_temp_sdf_file = RegisterEventHandler(
        event_handler=OnShutdown(on_shutdown=[OpaqueFunction(function=lambda _: os.remove(world_sdf))])
    )

    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(sim_dir, 'worlds')
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])),
        launch_arguments={'gz_args': ['-v4 -g ']}.items(),
    )

    gz_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(self_driving_dir, 'launch', 'spawn_friedbert.launch.py')),
        launch_arguments={
            'namespace': namespace,
            'use_simulator': use_simulator,
            'use_sim_time': use_sim_time,
            'robot_name': robot_name,
            'robot_sdf': robot_sdf,
            'x_pose': pose['x'],
            'y_pose': pose['y'],
            'z_pose': pose['z'],
            'roll': pose['R'],
            'pitch': pose['P'],
            'yaw': pose['Y'],
        }.items()
    )

    # LaunchDescription
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_robot_sdf_cmd)

    # Add environment and gazebo launch
    ld.add_action(set_env_vars_resources)
    ld.add_action(world_sdf_xacro)
    ld.add_action(remove_temp_sdf_file)
    ld.add_action(gz_robot)
    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)

    # Add ros2_control_node and robot_state_publisher
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(ros2_control_node)
    ld.add_action(joint_state_broadcaster_spawner)

    # RViz zuletzt starten
    #ld.add_action(rviz_cmd)

    return ld
