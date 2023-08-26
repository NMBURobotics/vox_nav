from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import GroupAction
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share_dir = get_package_share_directory("vox_nav_misc")

    ouster_correction_node = Node(
        package="vox_nav_misc",
        executable="ouster_correction_node",
        name="ouster_correction_node",
        output="screen",
        remappings=[
            ("points_in", "AGV0/dobbie/sensing/lidar/top/pointcloud_raw_ex"),
            ("points_out", "AGV0/dobbie/sensing/lidar/top/pointcloud_raw_correct"),
        ],
    )

    traversablity_integrator_node = Node(
        package="vox_nav_misc",
        executable="traversablity_integrator_node",
        name="traversablity_integrator_node",
        output="screen",
        remappings=[
            ("map_points_in", "lio_sam/mapping/map_local"),
            ("traversability_cloud_in", "pointnet/traversability/map_local"),
            ("traversability_map_points_out", "pointnet/traversability/map"),
        ],
        # prefix=["xterm -e gdb -ex run --args"],
    )

    ld = LaunchDescription()
    ld.add_action(traversablity_integrator_node)
    ld.add_action(ouster_correction_node)

    return ld
