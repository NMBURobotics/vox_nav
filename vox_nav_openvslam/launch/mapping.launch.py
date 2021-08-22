import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    this_package_dir = get_package_share_directory('vox_nav_openvslam')
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
            package='vox_nav_openvslam',
            executable='run_slam',
            name='run_slam_rclcpp_node',
            remappings=[('camera/color/image_raw', '/camera/color/image_raw'),
                        ('camera/depth/image_raw', '/camera/aligned_depth_to_color/image_raw'),
                        ('camera/left/image_raw', '/zed/zed_node/left/image_rect_color'),
                        ('camera/right/image_raw', '/zed/zed_node/right/image_rect_color')],
            output='screen',
            parameters=[config_file]
        ),
    ])
