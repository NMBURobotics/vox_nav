#!/bin/bash
current_dir="$(dirname "${BASH_SOURCE[0]}")"  # get the directory name
current_dir="$(realpath "${current_dir}")"    # resolve its full path if
source /opt/ros/foxy/setup.bash
source $current_dir/../../../../install/setup.bash
source ~/.bashrc
gnome-terminal -x sh -c "ros2 launch botanbot_navigation2 rviz_launch.py; bash" 
