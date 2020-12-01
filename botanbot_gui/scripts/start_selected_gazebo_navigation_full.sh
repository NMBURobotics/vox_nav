#!/bin/bash
current_dir="$(dirname "${BASH_SOURCE[0]}")"  # get the directory name
current_dir="$(realpath "${current_dir}")"    # resolve its full path if
source /opt/ros/foxy/setup.bash
source $current_dir/../../../../install/setup.bash
source ~/.bashrc
killall gzserver
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH$current_dir/../../../src/OUTDOOR_NAV2/botanbot_gazebo/models
export GAZEBO_WORLD=$GAZEBO_WORLD$1
ros2_launch_command="ros2 launch botanbot_navigation2 botanbot_simulation_launch.py; bash"
gnome-terminal -- sh -c "$ros2_launch_command"
echo "$ros2_launch_command"
echo "$GAZEBO_WORLD"