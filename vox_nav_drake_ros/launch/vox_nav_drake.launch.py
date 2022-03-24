# Copyright (c) 2022 Fetullah Atas, Norwegian University of Life Sciences
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

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Declare node actions
    drake_ros_node = Node(
        package='vox_nav_drake_ros',
        executable='drake_ros_node',
        name='drake_ros_node',
        output='screen',
        #prefix=['xterm -e gdb -ex run --args'],
        parameters=[],
        remappings=[('drake_test_node/cmd_vel', 'vox_nav/cmd_vel')])

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(drake_ros_node)

    return ld
