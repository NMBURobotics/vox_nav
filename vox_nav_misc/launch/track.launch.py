from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import GroupAction
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share_dir = get_package_share_directory("vox_nav_misc")

    tracker_params = LaunchConfiguration("tracker_params")

    decleare_tracker_params = DeclareLaunchArgument(
        "tracker_params",
        default_value=os.path.join(share_dir, "config", "ukf_tracker.yaml"),
        description="Path to the vox_nav parameters file.",
    )

    tracking_node = Node(
        package="vox_nav_misc",
        executable="ukf_tracker",
        name="ukf_tracker_node",
        output="screen",
        remappings=[
            (
                "detections",
                "/vox_nav/lidar_apollo_instance_segmentation/detections",
            ),  # input
            ("tracks", "/vox_nav/ukf_tracker/tracks"),  # output
        ],
        parameters=[tracker_params],
    )

    ld = LaunchDescription()
    ld.add_action(decleare_tracker_params)
    ld.add_action(tracking_node)

    return ld
