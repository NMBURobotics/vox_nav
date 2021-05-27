.. OUTDOOR_NAV2 documentation master file, created by
   sphinx-quickstart on Tue Dec 22 16:24:53 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Getting Started
========================================

* Install ROS2 foxy. 
Deb installation is strongly recomended. 
Detailed steps to install ROS2 Foxy can be found [here](https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Install-Debians/)

A few helper packages we use for gui and installation;

.. code-block:: bash

   sudo apt-get install python3-vcstool
   sudo apt-get install xdotool

Finally get the project repository and dependecy repositories and build; 

.. code-block:: bash

   source /opt/ros/foxy/setup.bash
   mkdir -p ~/colcon_ws/src
   cd ~/colcon_ws
   wget https://raw.githubusercontent.com/jediofgever/vox_nav/foxy/underlay.repos
   vcs import src < underlay.repos
   cd ~/colcon_ws
   rosdep install -y -r -q --from-paths src --ignore-src --rosdistro foxy
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-skip-regex archived_


.. note::
   Pay attention that we have disabled the build of archived_ packages. Some packages(e.g vox_net_openvslam) has quite some 
   amount of deps. If you like to use please refer to SLAM section , there you will find instructions
   to install vox_net_openvslam deps. After that you can remove `--packages-skip vox_net_openvslam` part from 
   colcon build command.

.. code-block:: bash

   cd ~/colcon_ws
   vcs import src < underlay.repos
   vcs pull src

We need the source build of some dependencies(e.g `pcl_ros`),

