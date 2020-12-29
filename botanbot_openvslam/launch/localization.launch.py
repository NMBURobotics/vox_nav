import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():

    this_package_dir = get_package_share_directory('botanbot_openvslam')
    rviz_config_dir = os.path.join(this_package_dir, 'rviz', 'openvslam.rviz')

    config_file = os.path.join(this_package_dir, 'config',
                               'openvslam_node_params.yaml')
    orb_vocab_file = os.path.join(this_package_dir, 'config',
                                  'orb_vocab.dbow2')

    output_map_filename = LaunchConfiguration('output_map_filename',
                                              default='~/output_map.msg')

    return LaunchDescription([
        DeclareLaunchArgument('output_map_filename',
                              default_value=output_map_filename,
                              description='Full path to output map msg file'),
        Node(
            package='botanbot_openvslam',
            executable='run_localization',
            name='run_localization_rclcpp_node',
            remappings=[('camera/color/image_raw', 'camera/color/image_raw'),
                        ('camera/depth/image_raw',
                         'camera/aligned_depth_to_color/image_raw')],
            output='screen',
            parameters=[config_file],
        ),
    ])
