# OUTDOOR_NAV2
A project to develop/adapt a navigation system for outdoor robotics particularly aiming for use-cases in agriculture. 

## Quick Start

* Install ROS2 foxy. 
This is latest LTS of ROS2 distros , so it makes a lot of sense to start from this version. 
Deb installation is strongly recomended. Detailed steps to install ROS2 Foxy can be found [here](https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Install-Debians/)

* Install Navigation2.
ROS way to control mobile robots with different physical models(e.g differantial, ackermann, omnidirection). 
This can be quickly done with; 

`sudo apt-get install ros-foxy-navigation`

* Install OUTDOOR_NAV2 packages

> mkdir -p colcon_ws/src

> cd colcon_ws/src

> https://github.com/jediofgever/OUTDOOR_NAV2.git && cd ..

* Install dependencies with; 

> rosdep install -y -r -q --from-paths src --ignore-src --rosdistro foxy

> colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

* Additional linux terminal command utility package

> sudo apt-get install xdotool

## Run project

THe project has a RGT GUI plugin that lets you to interact with robot. To start with this plugin make sure in previous step you built project 
successfully. 

* source your colcon_ws and start the project with;

> source install/setup.bash

> rqt --force-discover

![.](images/3.png)

The rqt window should open as above. You should now find our plugin under; 

Plugins -> Visualization -> Control Plugin. 

Click on Control Plugin and you would be able to see; 

![.](images/4.png)

***

## Botanbot
Botanbot is a simple 4 wheeled , ackermann drived moile robot. It is simulated under Gazebo with all required essential sensors in order to do outdoor navigation. The following table shows currently supported sensors. 
### Sensor support for Botanbot
| Sensor type | Topic Name(s) | Message Type | Update Rate |
| :---: | :---: | :---: | :---: |
| LIDAR | /velodyne_points | sensor_msgs::msg::PointCloud2 | 30 |
| RealSense D435 COLOR CAMERA | /camera/color/image_raw | sensor_msgs::msg::Image | 30 |
| RealSense D435 DEPTH CAMERA | /camera/aligned_depth_to_color/image_raw | sensor_msgs::msg::Image | 30 |
| RealSense D435 IR1 CAMERA | /camera/infra1/image_raw | sensor_msgs::msg::Image | 1 |
| RealSense D435 IR2 CAMERA | /camera/infra2/image_raw | sensor_msgs::msg::Image | 1 |
| GPS | /gps/data | sensor_msgs::msg::NavSatFix | 30 |
| IMU | /imu | sensor_msgs::msg::Imu | 30 |

### Botanbot in Gazebo
![.](docs/botanbot_0.jpg)

![.](docs/botanbot_1.jpg)
