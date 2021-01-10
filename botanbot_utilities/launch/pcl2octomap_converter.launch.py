import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Find the grid_map_demos package share directory
    grid_map_demos_dir = get_package_share_directory('botanbot_utilities')

    pcl2octomap_converter_node_config = LaunchConfiguration(
        'pcl2octomap_converter_node_config')

    pcl2octomap_converter_node_config_cmd = DeclareLaunchArgument(
        'pcl2octomap_converter_node_config',
        default_value=os.path.join(grid_map_demos_dir, 'config',
                                   'pcl2octomap_converter_node_config.yaml'),
        description='Full path to the pcl2octomap rclcpp node config')

    # Declare node actions
    pcl2octomap_converter_rclcpp_node = Node(
        package='botanbot_utilities',
        executable='pcl2octomap_converter',
        name='pcl2octomap_converter_rclcpp_node',
        output='screen',
        parameters=[pcl2octomap_converter_node_config])

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(pcl2octomap_converter_node_config_cmd)
    ld.add_action(pcl2octomap_converter_rclcpp_node)

    return ld
