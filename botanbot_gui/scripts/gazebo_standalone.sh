killall gzserver
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/atas/colcon_ws/src/OUTDOOR_NAV2/botanbot_gazebo/models
source ~/colcon_ws/install/setup.bash
source ~/.bashrc

gnome-terminal -x sh -c "ros2 launch botanbot_gazebo botanbot_city.launch.py; bash" 
