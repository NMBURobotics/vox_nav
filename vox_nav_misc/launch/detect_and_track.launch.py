from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import GroupAction
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share_dir = get_package_share_directory("vox_nav_misc")

    detector_params = LaunchConfiguration("detector_params")
    tracker_params = LaunchConfiguration("tracker_params")

    decleare_detector_params = DeclareLaunchArgument(
        "detector_params",
        default_value=os.path.join(share_dir, "config", "naive_lidar_clustering.yaml"),
        description="Path to the vox_nav parameters file.",
    )
    decleare_tracker_params = DeclareLaunchArgument(
        "tracker_params",
        default_value=os.path.join(share_dir, "config", "ukf_tracker.yaml"),
        description="Path to the vox_nav parameters file.",
    )

    detection_node = Node(
        package="vox_nav_misc",
        executable="naive_lidar_clustering",
        name="naive_lidar_clustering_node",
        output="screen",
        # prefix=['xterm -e gdb -ex run --args'],
        # prefix=['valgrind --tool=callgrind --dump-instr=yes -v --instr-atstart=no'],
        # remappings=[('points', '/velodyne_points')],
        remappings=[
            ("points", "/ouster/points"),
            ("detections", "/vox_nav/naive_lidar_clustering/detections"),
        ],
        parameters=[detector_params],
    )

    tracking_node = Node(
        package="vox_nav_misc",
        executable="ukf_tracker",
        name="ukf_tracker_node",
        output="screen",
        remappings=[
            ("detections", "/vox_nav/naive_lidar_clustering/detections"),
            ("tracks", "/vox_nav/naive_lidar_tracking/tracks"),
        ],
        parameters=[tracker_params],
    )

    ld = LaunchDescription()
    ld.add_action(decleare_detector_params)
    ld.add_action(detection_node)
    ld.add_action(decleare_tracker_params)
    ld.add_action(tracking_node)

    return ld
