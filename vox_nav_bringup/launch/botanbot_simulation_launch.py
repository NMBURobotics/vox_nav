# Copyright (c) 2018 Intel Corporation
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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

GAZEBO_WORLD = os.environ['GAZEBO_WORLD']


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('vox_nav_bringup')
    localization_dir = get_package_share_directory(
        'vox_nav_robot_localization')

    launch_dir = os.path.join(bringup_dir, 'launch')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Launch configuration variables specific to simulation
    use_simulator = LaunchConfiguration('use_simulator')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(bringup_dir, 'rviz',
                                   'vox_nav_default_view.rviz'),
        description='Full path to the RVIZ config file to use')

    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator',
        default_value='True',
        description='Whether to start the simulator')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='True', description='Whether to start RVIZ')

    declare_simulator_cmd = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Whether to execute gzclient)')
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(
            get_package_share_directory('vox_nav_gazebo'), 'worlds/',
            GAZEBO_WORLD, GAZEBO_WORLD + '.world'),
        description='Full path to world model file to load')

    # Specify the actions
    start_gazebo_server_cmd = ExecuteProcess(
        condition=IfCondition(use_simulator),
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so', world],
        cwd=[launch_dir],
        output='screen')

    start_gazebo_client_cmd = ExecuteProcess(condition=IfCondition(
        PythonExpression([use_simulator, ' and not ', headless])),
                                             cmd=['gzclient'],
                                             cwd=[launch_dir],
                                             output='screen')

    urdf = os.path.join(get_package_share_directory('vox_nav_description'),
                        'urdf/vox_nav.urdf')

    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        remappings=remappings,
        arguments=[urdf])

    rviz_cmd = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(launch_dir, 'rviz_launch.py')),
                                        condition=IfCondition(use_rviz),
                                        launch_arguments={
                                            'namespace': '',
                                            'use_namespace': 'False',
                                        }.items())

    bringup_cmd = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(launch_dir, 'bringup.launch.py')),
                                           launch_arguments={
                                               'namespace': namespace,
                                               'use_namespace': use_namespace,
                                               'use_sim_time': use_sim_time,
                                           }.items())

    localization_cmd = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(localization_dir, 'launch',
                     'dual_ekf_navsat_localization.launch.py')),
                                                launch_arguments={
                                                    'namespace': namespace,
                                                    'use_namespace':
                                                    use_namespace,
                                                    'use_sim_time':
                                                    use_sim_time
                                                }.items())
    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)

    # Add any conditioned actions
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(bringup_cmd)
    ld.add_action(localization_cmd)

    return ld
