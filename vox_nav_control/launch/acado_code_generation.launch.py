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
    # Find the grid_map_demos package share directory
    vox_nav_bringup_share_dir = get_package_share_directory("vox_nav_bringup")
    vox_nav_map_server_share_dir = get_package_share_directory("vox_nav_map_server")

    vox_nav_params = LaunchConfiguration("vox_nav_params")

    # vox_nav_params_cmd = DeclareLaunchArgument(
    #    "vox_nav_params",
    #    default_value=os.path.join(
    #        vox_nav_bringup_share_dir, "params", "vox_nav_default_params.yaml"
    #    ),
    #    description="Full path to the vox_nav params",
    # )
    vox_nav_params_cmd = DeclareLaunchArgument(
        "vox_nav_params",
        default_value=os.path.join(
            vox_nav_map_server_share_dir, "config", "osm_map.yaml"
        ),
        description="Full path to the vox_nav params",
    )

    # Declare node actions
    mpc_controller_acado_code_gen_node = Node(
        package="vox_nav_control",
        executable="mpc_controller_acado_code_gen",
        # set identical name to controller server to acesss params
        name="vox_nav_controller_server_rclcpp_node",
        output="screen",
        # prefix=['xterm -e gdb -ex run --args'],
        parameters=[vox_nav_params],
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(vox_nav_params_cmd)
    ld.add_action(mpc_controller_acado_code_gen_node)

    return ld
