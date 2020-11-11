source ~/colcon_ws/install/setup.bash
source ~/.bashrc
gnome-terminal -x sh -c "ros2 launch botanbot_bringup rviz_launch.py; bash" 
