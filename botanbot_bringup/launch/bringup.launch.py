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
    share_dir = get_package_share_directory('botanbot_bringup')

    parameter_file = LaunchConfiguration('params_file')
    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(share_dir, 'params', 'params.yaml'),
        description='FPath to the ROS2 parameters file to use.')

    planner_server_node = Node(
        package='botanbot_planning',
        executable='botanbot_planner_server',
        name='botanbot_planner_server_rclcpp_node',
        namespace='',
        #prefix=['xterm -e gdb -ex run --args'],
        output='screen',
        parameters=[parameter_file],
    )

    controller_server_node = Node(
        package='botanbot_control',
        executable='botanbot_controller_server',
        name='botanbot_controller_server_rclcpp_node',
        namespace='',
        output='screen',
        parameters=[parameter_file],
    )

    map_server_node = Node(
        package='botanbot_map_server',
        executable='botanbot_map_manager',
        name='botanbot_map_server_rclcpp_node',
        namespace='',
        output='screen',
        parameters=[parameter_file],
    )

    navigate_to_pose_server_node = Node(
        package='botanbot_pose_navigator',
        executable='navigate_to_pose_server_node',
        name='navigate_to_pose_server_node',
        namespace='',
        output='screen',
        parameters=[parameter_file],
    )

    return LaunchDescription([
        params_declare, planner_server_node, controller_server_node,
        map_server_node, navigate_to_pose_server_node
    ])
