#!/bin/bash
current_dir="$(dirname "${BASH_SOURCE[0]}")"  # get the directory name
current_dir="$(realpath "${current_dir}")"    # resolve its full path if
source $current_dir/../../../../install/setup.bash
source ~/.bashrc
killall gzserver
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$current_dir/../../../../src/OUTDOOR_NAV2/botanbot_gazebo/models
ros2_launch_command="ros2 launch botanbot_gazebo "
launch_extension=".launch.py; bash"
gazebo_world="botanbot_$1$launch_extension"
gnome-terminal -- sh -c "$ros2_launch_command$gazebo_world"