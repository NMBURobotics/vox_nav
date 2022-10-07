from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import GroupAction
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    share_dir = get_package_share_directory('vox_nav_cupoch')

    params = LaunchConfiguration('params')

    decleare_params = DeclareLaunchArgument(
        'params',
        default_value=os.path.join(
            share_dir, 'config', 'cupoch_gpu_icp_params.yaml'),
        description='Path to the vox_nav parameters file.')

    node = Node(package='vox_nav_cupoch',
                executable='cupoch_gpu_icp',
                name='cupoch_gpu_icp',
                output='screen',
                #prefix=['xterm -e gdb -ex run --args'],
                #prefix=['valgrind --tool=callgrind --dump-instr=yes -v --instr-atstart=no'],
                remappings=[('/ouster/points', '/ouster/points'),
                            ('vox_nav/map_server/octomap_pointcloud',
                             'vox_nav/map_server/octomap_pointcloud'),
                            ('odometry/gps', 'odometry/global')],
                parameters=[params]
                )

    return LaunchDescription([
        decleare_params,
        node
    ])
