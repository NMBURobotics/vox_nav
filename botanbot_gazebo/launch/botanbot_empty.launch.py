import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    DeclareLaunchArgument('gui', default_value='true',
                          description='Set to "false" to run headless.'),

    DeclareLaunchArgument('debug', default_value='true',
                          description='Set to "false" not to run gzserver.'),

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file_name = 'empty_world/empty_world.model'
    world = os.path.join(get_package_share_directory(
        'botanbot_gazebo'), 'worlds', world_file_name)
    launch_file_dir = os.path.join(
        get_package_share_directory('botanbot_gazebo'), 'launch')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world,
                 '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [launch_file_dir, '/robot_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
    ])
