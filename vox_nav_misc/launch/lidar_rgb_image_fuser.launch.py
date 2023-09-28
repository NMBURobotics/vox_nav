from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    lidar_rgb_image_fuser = Node(
        package="vox_nav_misc",
        executable="lidar_rgb_image_fuser",
        name="lidar_rgb_image_fuser",
        output="screen",
        remappings=[
            ("image", "/camera/color/image_raw"),
            ("points", "/ouster/points"),
        ],
    )

    ld = LaunchDescription()

    ld.add_action(lidar_rgb_image_fuser)

    return ld
