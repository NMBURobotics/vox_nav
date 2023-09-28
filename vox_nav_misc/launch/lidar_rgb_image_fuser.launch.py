from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    lidar_rgb_image_fuser = Node(
        package="vox_nav_misc",
        executable="lidar_rgb_image_fuser",
        name="lidar_rgb_image_fuser",
        output="screen",
        remappings=[
            # ("image", "/camera/color/image_raw"),
            # ("points", "/ouster/points"),
            ("image", "/AGV0/dobbie/sensing/camera/traffic_light/image_raw"),
            ("points", "/AGV0/dobbie/sensing/lidar/top/pointcloud_raw_correct"),
        ],
    )

    ld = LaunchDescription()

    ld.add_action(lidar_rgb_image_fuser)

    return ld
