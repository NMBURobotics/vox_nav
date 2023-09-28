from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import GroupAction
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    fix_ouster_pointtype_node = Node(
        package="vox_nav_misc",
        executable="fix_ouster_pointtype_node",
        name="fix_ouster_pointtype_node",
        output="screen",
        remappings=[
            ("points_in", "/AGV0/dobbie/sensing/lidar/top/pointcloud_raw_ex"),
            ("points_out", "AGV0/dobbie/sensing/lidar/top/pointcloud_raw_correct"),
        ],
    )

    ld = LaunchDescription()
    ld.add_action(fix_ouster_pointtype_node)

    return ld
