# Copyright 2018 Open Source Robotics Foundation, Inc.
# Copyright 2020 Fetullah Atas, Norwegian University of Life Sciences
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

    vox_nav_bringup_dir = get_package_share_directory(
        'vox_nav_bringup')

    localization_params = LaunchConfiguration('localization_params')
    output_final_position = LaunchConfiguration('output_final_position')
    output_location = LaunchConfiguration('output_location')

    decleare_localization_params = DeclareLaunchArgument('localization_params',
                                                         default_value=os.path.join(
                                                             vox_nav_bringup_dir, 'params', 'localization_params.yaml'),
                                                         description='Path to the localization parameters file.')

    declare_output_final_position = DeclareLaunchArgument('output_final_position',
                                                          default_value='false')

    declare_output_location = DeclareLaunchArgument('output_location',
                                                    default_value='~/localization_params.txt')

    ekf_local_filter_node = Node(package='robot_localization',
                                 executable='ekf_node',
                                 name='ekf_local_filter_node',
                                 output='screen',
                                 parameters=[localization_params],
                                 remappings=[('odometry/filtered', 'odometry/local')])

    ekf_global_filter_node = Node(package='robot_localization',
                                  executable='ekf_node',
                                  name='ekf_global_filter_node',
                                  output='screen',
                                  parameters=[localization_params],
                                  remappings=[('odometry/filtered', 'odometry/global')])

    navsat_transform_node = Node(package='robot_localization',
                                 executable='navsat_transform_node',
                                 name='navsat_transform_node',
                                 output='screen',
                                 parameters=[localization_params],
                                 remappings=[('imu/data', 'imu/data'),
                                             ('gps/fix', 'gps/fix'),
                                             ('gps/filtered', 'gps/filtered'),
                                             ('odometry/gps', 'odometry/gps'),
                                             ('odometry/filtered', 'odometry/global')])

    return LaunchDescription([
        decleare_localization_params,
        declare_output_final_position,
        declare_output_location,
        ekf_local_filter_node,
        ekf_global_filter_node,
        navsat_transform_node
    ])
