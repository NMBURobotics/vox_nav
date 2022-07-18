import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Find the grid_map_demos package share directory
    package_dir = get_package_share_directory('vox_nav_planning')

    planner_benchmarking_node_config = LaunchConfiguration(
        'planner_benchmarking_node_config')

    planner_benchmarking_node_config_cmd = DeclareLaunchArgument(
        'planner_benchmarking_node_config',
        default_value=os.path.join(package_dir, 'config',
                                   'control_planner_benchmarking.yaml'),
        description='Full path to the pcl2octomap rclcpp node config')

    # Declare node actions
    control_planners_benchmark = Node(
        package='vox_nav_planning',
        executable='control_planners_benchmark',
        name='control_planners_benchmark',
        output='screen',
        prefix=['xterm -e gdb -ex run --args'],
        parameters=[planner_benchmarking_node_config])

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(planner_benchmarking_node_config_cmd)
    ld.add_action(control_planners_benchmark)

    return ld
