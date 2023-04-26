from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import GroupAction
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share_dir = get_package_share_directory("vox_nav_misc")

    traversablity_estimator_params = LaunchConfiguration(
        "traversablity_estimator_params"
    )

    decleare_traversablity_estimator_params = DeclareLaunchArgument(
        "traversablity_estimator_params",
        default_value=os.path.join(share_dir, "config", "traversablity_estimator.yaml"),
        description="Path to the vox_nav parameters file.",
    )

    traversablity_estimator_node = Node(
        package="vox_nav_misc",
        executable="traversablity_estimator",
        name="traversablity_estimator_node",
        output="screen",
        remappings=[
            ("points", "/lio_sam/mapping/map_local"),
            ("segmenents", "/vox_nav/ukf_tracker/tracks"),
        ],
        parameters=[traversablity_estimator_params],
    )

    ld = LaunchDescription()
    ld.add_action(decleare_traversablity_estimator_params)
    ld.add_action(traversablity_estimator_node)

    return ld
