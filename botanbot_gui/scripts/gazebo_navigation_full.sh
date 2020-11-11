killall gzserver
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/atas/colcon_ws/src/OUTDOOR_NAV2/botanbot_gazebo/models
source ~/colcon_ws/install/setup.bash
source ~/.bashrc
gnome-terminal -x sh -c "ros2 launch botanbot_navigation2 botanbot_simulation_launch.py; bash" 
