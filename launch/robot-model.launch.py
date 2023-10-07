import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    urdf = get_package_share_directory("self_driving_bot") + '/urdf/ikea_table.urdf'
    launch_dir = os.path.join(get_package_share_directory("self_driving_bot") , 'launch')

     # Specify the actions
    start_gazebo_server_cmd = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'],
        cwd=[launch_dir], output='screen')

    start_gazebo_client_cmd = ExecuteProcess(
        cmd=['gzclient'],
        cwd=[launch_dir], output='screen')

    start_gazebo_spawner_cmd =  Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            output='screen',
            arguments=['-entity', 'my_robot', '-file', urdf])
        
    with open(urdf, 'r') as infp:
        robot_description = infp.read()

    ld = LaunchDescription()
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(start_gazebo_spawner_cmd)

    return ld

if __name__ == '__main__':
    generate_launch_description()