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
        launch_ros.actions.Node(package='vox_nav_utilities',
                                executable='dynamic_points_node',
                                name='dynamic_points_node',
                                output='screen',
                                #prefix=['xterm -e gdb -ex run --args'],
                                remappings=[('points', '/ouster/points'),
                                            ('odom', '/odometry/global'),
                                            ('imu', '/xsens/imu')],
                                parameters=[{'sequence_horizon': 2},
                                            {'dt': 5.5},
                                            {'sensor_height': -1.9}])
    ])
