# Copyright (c) 2020 Fetullah Atas, Norwegian University of Life Sciences
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import GroupAction
from launch.actions import DeclareLaunchArgument

import os


def generate_launch_description():

    share_dir = get_package_share_directory('vox_nav_bringup')

    params = LaunchConfiguration('params')
    localization_params = LaunchConfiguration('localization_params')

    decleare_params = DeclareLaunchArgument(
        'params',
        default_value=os.path.join(share_dir, 'params', 'params.yaml'),
        description='Path to the vox_nav parameters file.')

    decleare_localization_params = DeclareLaunchArgument(
        'localization_params',
        default_value=os.path.join(
            share_dir, 'params', 'localization_params.yaml'),
        description='Path to the localization parameters file.')

    planner_server_node = Node(
        package='vox_nav_planning',
        executable='planner_server',
        name='vox_nav_planner_server_rclcpp_node',
        namespace='',
        output='screen',
        #prefix=['xterm -e gdb -ex run --args'],
        # prefix=['valgrind --tool=callgrind --dump-instr=yes -v --instr-atstart=no'],
        parameters=[params],
    )
    controller_server_node = Node(
        package='vox_nav_control',
        executable='vox_nav_controller_server',
        name='vox_nav_controller_server_rclcpp_node',
        namespace='',
        output='screen',
        # prefix=['xterm -e gdb -ex run --args'],
        parameters=[params],
    )
    map_server_node = Node(
        package='vox_nav_map_server',
        executable='map_manager',
        name='vox_nav_map_server_rclcpp_node',
        namespace='',
        output='screen',
        parameters=[params],
    )
    navigate_to_pose_server_node = Node(
        package='vox_nav_navigators',
        executable='navigate_to_pose_server_node',
        name='navigate_to_pose_server_node',
        namespace='',
        output='screen',
        parameters=[params],
    )
    navigate_through_poses_server_node = Node(
        package='vox_nav_navigators',
        executable='navigate_through_poses_server_node',
        name='navigate_through_poses_server_node',
        namespace='',
        output='screen',
        parameters=[params],
    )
    navigate_through_gps_poses_server_node = Node(
        package='vox_nav_navigators',
        executable='navigate_through_gps_poses_server_node',
        name='navigate_through_gps_poses_server_node',
        namespace='',
        output='screen',
        parameters=[params],
    )

    ekf_local_filter_node = Node(package='robot_localization',
                                 executable='ekf_node',
                                 name='ekf_local_filter_node',
                                 # output='screen',
                                 output={'both': 'log'},
                                 parameters=[localization_params],
                                 remappings=[('odometry/filtered', 'odometry/local')])

    ekf_global_filter_node = Node(package='robot_localization',
                                  executable='ekf_node',
                                  name='ekf_global_filter_node',
                                  # output='screen',
                                  output={'both': 'log'},
                                  parameters=[localization_params],
                                  remappings=[('odometry/filtered', 'odometry/global')])

    navsat_transform_node = Node(package='robot_localization',
                                 executable='navsat_transform_node',
                                 name='navsat_transform_node',
                                 # change it to screen if you wanna see RVIZ output in terminal
                                 # output={'both': 'log'},
                                 output='screen',
                                 parameters=[localization_params],
                                 remappings=[('imu', 'imu/absolute'),
                                             ('gps/fix', 'fix'),
                                             ('gps/filtered', 'gps/filtered'),
                                             ('odometry/gps', 'odometry/gps'),
                                             ('odometry/filtered', 'odometry/global')])

    return LaunchDescription([
        decleare_params,
        decleare_localization_params,
        ekf_local_filter_node,
        ekf_global_filter_node,
        navsat_transform_node,
        planner_server_node,
        controller_server_node,
        map_server_node,
        navigate_to_pose_server_node,
        navigate_through_poses_server_node,
        navigate_through_gps_poses_server_node
    ])
