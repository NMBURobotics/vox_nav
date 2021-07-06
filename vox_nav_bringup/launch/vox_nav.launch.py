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
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import EmitEvent
from launch.actions import RegisterEventHandler
from launch_ros.events.lifecycle import ChangeState
from launch_ros.events.lifecycle import matches_node_name
from launch_ros.event_handlers import OnStateTransition
from launch.actions import LogInfo
from launch.events import matches_action
from launch.event_handlers.on_shutdown import OnShutdown

import lifecycle_msgs.msg
import os


def generate_launch_description():

    share_dir = get_package_share_directory('vox_nav_bringup')

    params = LaunchConfiguration('params')

    decleare_params = DeclareLaunchArgument(
        'params',
        default_value=os.path.join(share_dir, 'params', 'params.yaml'),
        description='Path to the vox_nav parameters file.')

    planner_server_node = Node(
        package='vox_nav_planning',
        executable='planner_server',
        name='vox_nav_planner_server_rclcpp_node',
        namespace='',
        output='screen',
        prefix=['xterm -e gdb -ex run --args'],
        parameters=[params],
    )

    controller_server_node = Node(
        package='vox_nav_control',
        executable='vox_nav_controller_server',
        name='vox_nav_controller_server_rclcpp_node',
        namespace='',
        output='screen',
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
        package='vox_nav_pose_navigator',
        executable='navigate_to_pose_server_node',
        name='navigate_to_pose_server_node',
        namespace='',
        output='screen',
        parameters=[params],
    )

    return LaunchDescription([
        decleare_params,
        planner_server_node,
        controller_server_node,
        map_server_node,
        navigate_to_pose_server_node
    ])
