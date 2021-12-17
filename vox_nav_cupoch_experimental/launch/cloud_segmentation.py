from launch import LaunchDescription
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(package='vox_nav_cupoch_experimental',
                                executable='cloud_segmentation',
                                name='cloud_segmentation',
                                output='screen',
                                 prefix=['xterm -e gdb -ex run --args'],
                                #prefix=['valgrind --tool=callgrind --dump-instr=yes -v --instr-atstart=no'],
                                remappings=[('points', '/ouster/points/segmented'),
                                            ('odom', '/odometry/global'),
                                            ('imu', '/ouster/imu')],
                                parameters=[{'sequence_horizon': 2},
                                            {'dt': 0.01},
                                            {'sensor_height': 0.0}])
    ])
