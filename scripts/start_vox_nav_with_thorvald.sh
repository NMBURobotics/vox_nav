#!/bin/bash

source /opt/ros/foxy/setup.bash
source ~/colcon_ws/install/setup.bash

ouster_sync_command="sudo ptpd -i enp6s0 -M -V; bash"
gnome-terminal -- sh -c "$ouster_sync_command"

ros2_launch_command="ros2 launch sensor_drivers_bringup bringup.launch.py; bash"
gnome-terminal -- sh -c "$ros2_launch_command"

ros2_launch_command="ros2 launch thorvald_vox_nav vox_nav.launch.py ; bash"
gnome-terminal -- sh -c "$ros2_launch_command"

ros2_record_command="bash record.sh; bash"
gnome-terminal -- sh -c "$ros2_record_command"
