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
        launch_ros.actions.Node(package='botanbot_utilities',
                                executable='gps_waypoint_collector_node',
                                name='gps_waypoint_collector_rclcpp_node',
                                output='screen',
                                remappings=[('/gps/fix', '/gps/fix'),
                                            ('/gps/fix', '/imu/data')])
    ])
