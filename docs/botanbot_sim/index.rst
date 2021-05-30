.. OUTDOOR_NAV2 documentation master file, created by
   sphinx-quickstart on Tue Dec 22 16:24:53 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Botanbot Sim
========================================

Botanbot is a simple 4 wheeled , ackermann drived mobile robot.
It is simulated under Gazebo with all required essential sensors in order to do outdoor navigation. The following table shows currently supported sensors. 


**1. Sensor support for Botanbot**


+-------------------------------+--------------------------+----------------------------------+-----------------------+
|      Sensor Type              |       Topic              |       MSG TYPE                   | Update Rate           |
+===============================+==========================+==================================+=======================+
|  LIDAR                        | /velodyne_points         | sensor_msgs::msg::PointCloud2    | 30                    |
|                               |                          |                                  |                       |
+-------------------------------+--------------------------+----------------------------------+-----------------------+
|  RealSense D435 COLOR CAMERA  | /camera/color/image_raw  | sensor_msgs::msg::Image          |  30                   |
|                               |                          |                                  |                       |
+-------------------------------+--------------------------+----------------------------------+-----------------------+
|  RealSense D435 DEPTH CAMERA  | /camera/                 | sensor_msgs::msg::Image          |  30                   |
|                               | aligned_depth_to_color/  |                                  |                       |
|                               | image_raw                |                                  |                       |
+-------------------------------+--------------------------+----------------------------------+-----------------------+
|  RealSense D435 IR1   CAMERA  | /camera/infra1/image_raw | sensor_msgs::msg::Image          |  1                    |
|                               |                          |                                  |                       |
+-------------------------------+--------------------------+----------------------------------+-----------------------+
|  RealSense D435 IR2   CAMERA  | /camera/infra2/image_raw | sensor_msgs::msg::Image          |  1                    |
|                               |                          |                                  |                       |
+-------------------------------+--------------------------+----------------------------------+-----------------------+
|  GPS                          | /gps/fix                 | sensor_msgs::msg::NavSatFix      |  30                   |
|                               |                          |                                  |                       |
+-------------------------------+--------------------------+----------------------------------+-----------------------+
|  IMU                          | /imu/data                | sensor_msgs::msg::IMU            |  1                    |
|                               |                          |                                  |                       |
+-------------------------------+--------------------------+----------------------------------+-----------------------+


* Botanbot navigation in farming world

.. image:: /images/botanbot_2.png
   :width: 700px
   :align: center
   :alt: rqt landing screen


* Botanbot in Hilly Gazebo world

.. image:: /images/botanbot_0.jpg
   :width: 700px
   :align: center
   :alt: rqt landing screen

.. image:: /images/botanbot_1.jpg
   :width: 700px
   :align: center
   :alt: rqt landing screen


(botanbot_sim)[https://github.com/jediofgever/botanbot_sim] is configured use vox_nav system for 3D navigation. If you are trying to setup your robot for use with 
vox_nav, botanbot_sim is  right place to look for.
A RQT based gui is also provided in botanbot_gui, through this gui it is possible to send goal commands to action server(NavigateToPose) of vox_nav. If using simulation, 
botanbot_gazebo includes sensor plugins.
   