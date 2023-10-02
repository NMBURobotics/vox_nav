from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import GroupAction
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    stick_imu_to_inertial_frame = Node(
        package="vox_nav_misc",
        executable="stick_imu_to_inertial_frame",
        name="stick_imu_to_inertial_frame",
        output="screen",
        remappings=[
            ("imu", "/AGV0/dobbie/sensing/imu/fjr/imu_raw"),
            ("imu_corrected", "/imu/data/corrected"),
        ],
    )

    ld = LaunchDescription()
    ld.add_action(stick_imu_to_inertial_frame)

    return ld
