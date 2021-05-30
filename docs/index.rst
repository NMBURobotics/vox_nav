.. vox_nav documentation master file, created by
   sphinx-quickstart on Tue Dec 22 16:24:53 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to vox_nav's documentation!
========================================

A project to develop/adapt a navigation system for outdoor robotics particularly aiming for use-cases in agriculture. 

* We call this `vox_nav` since it relies on voxel-based 3D occupancy grid(Octomap) in order to navigate a mobile robot. 

* This project's main use case is navigation of a mobile robot in rough outdoor environments. 

* At the moment, environment reprsenation is done with Octomap, Based on the 3D topology of map we overlay cost layers onto octomap, this bases on several critics such as elevation, roughness etc.

* Planning of vox_net relies on OMPL, we provide SE2, SE2.5 planners. SE2 planners can also be constrained to output plans for Ackermaan type robots with DUBINS AND REEDS-SHEEP.

* SE2.5 planner computes valid plans that can go through ramps and hills, giving the robot chance to traverse through them, we this as one of the main difference of vox_net over existing navigation frameworks.

* Octomaps are usually acquired from a pointcloud map of the envoirnment that you would like to navigate in.

* Map server manages pointclouds maps, a pointcloud map is required to have a datum(GPS coordinates and orientation), see the configuration section.


This branch(foxy) is aiming for ROS2 Foxy distro. 

.. toctree::
   :hidden:

   getting_started/index.rst
   configuration/index.rst
   botanbot_sim/index.rst
   running_project/index.rst
   openvslam/index.rst
   vox_nav_archived/index.rst


