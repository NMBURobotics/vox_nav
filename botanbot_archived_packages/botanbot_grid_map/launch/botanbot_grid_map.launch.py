import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Find the grid_map_demos package share directory
    grid_map_demos_dir = get_package_share_directory('botanbot_grid_map')

    # Declare launch configuration variables that can access the launch arguments values
    visualization_config_file = LaunchConfiguration('visualization_config')
    node_config_file = LaunchConfiguration('node_config')
    rviz_config_file = LaunchConfiguration('rviz_config')

    # Declare launch arguments
    declare_visualization_config_file_cmd = DeclareLaunchArgument(
        'visualization_config',
        default_value=os.path.join(grid_map_demos_dir, 'config',
                                   'grid_map_visualization_config.yaml'),
        description='Full path to the Gridmap visualization config file to use'
    )
    declare_grid_map_node_config_file_cmd = DeclareLaunchArgument(
        'node_config',
        default_value=os.path.join(grid_map_demos_dir, 'config',
                                   'grid_map_node_config.yaml'),
        description='Full path to the Gridmap visualization config file to use'
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(grid_map_demos_dir, 'rviz',
                                   'botanbot_grid_map.rviz'),
        description='Full path to the RVIZ config file to use')

    # Declare node actions
    tutorial_demo_node = Node(package='botanbot_grid_map',
                              executable='botanbot_grid_map_node',
                              name='botanbot_grid_map_node',
                              output='screen',
                              parameters=[node_config_file])

    grid_map_visualization_node = Node(package='grid_map_visualization',
                                       executable='grid_map_visualization',
                                       name='grid_map_visualization',
                                       output='screen',
                                       parameters=[visualization_config_file])

    tf = Node(package='tf2_ros',
              executable='static_transform_publisher',
              arguments=[
                  '0.0', '0.0', '0.0', '0.0', '0.0', '1.0', '1', 'odom',
                  'grid_map'
              ])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add launch arguments to the launch description
    ld.add_action(declare_visualization_config_file_cmd)
    ld.add_action(declare_grid_map_node_config_file_cmd)
    ld.add_action(declare_rviz_config_file_cmd)

    # Add node actions to the launch description
    ld.add_action(tutorial_demo_node)
    ld.add_action(grid_map_visualization_node)
    ld.add_action(tf)

    return ld
