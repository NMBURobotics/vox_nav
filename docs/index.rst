.. vox_nav documentation master file, created by
   sphinx-quickstart on Tue Dec 22 16:24:53 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to vox_nav's documentation!
========================================

A project to develop/adapt a navigation system for outdoor robotics particularly aiming for use-cases in agriculture. 

* We call this `vox_nav` since it relies on voxel-based 3D occupacy grid(Octomap) in order to navigate a robot. 

* This project's main use case is navigation of robot in rough outdoor envoirnmets. 

* At the moment envoirnment reprsenation is done with Octomap, Based on the 3D topology of map we overlap costs to octomap.

* Planning of vox_net relies on OMPL, we provide SE2, SE2.5 , planners. SE2 planners can also be constrained to output plans for Ackermaan type robots with DUBINS AND REEDS-SHEEP.

* SE2.5 planner compute valid plans that go through ramps and hills, giving the robot chance to traverse though them.




This branch(foxy) is aiming for ROS2 Foxy distro. 

.. toctree::
   :hidden:

   roadmap_milestones/index.rst
   getting_started/index.rst
   running_project/index.rst
   botanbot/index.rst
   slam/index.rst
   grid_map/index.rst


