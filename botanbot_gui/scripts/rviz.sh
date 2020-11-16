source ~/colcon_ws/install/setup.bash
source ~/.bashrc
gnome-terminal -x sh -c "ros2 launch botanbot_navigation2 rviz_launch.py; bash" 
