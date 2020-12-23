# Copyright 2019 Open Source Robotics Foundation, Inc.
#
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

# Author: Darby Lim

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

    config_file = os.path.join(this_package_dir, 'config', 'config.yaml')
    orb_vocab_file = os.path.join(this_package_dir, 'config', 'orb_vocab.dbow2')

    output_map_filename = LaunchConfiguration('output_map_filename', default=os.path.join(this_package_dir, 'scripts', 'map.msg'))

    return LaunchDescription([
        DeclareLaunchArgument(
            'output_map_filename',
            default_value=output_map_filename,
            description='Full path to output map msg file'),
        Node(
            package='botanbot_openvslam',
            executable='run_localization',
            name='run_localization',
            remappings=[('camera/image_raw', 'camera/color/image_raw')],           
            output='screen',
            arguments=['-v', orb_vocab_file, '-c', config_file, '--map-db', output_map_filename]),

    ])
