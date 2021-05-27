.. OUTDOOR_NAV2 documentation master file, created by
   sphinx-quickstart on Tue Dec 22 16:24:53 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Getting Started
========================================

* Install ROS2 foxy. 
This is latest LTS of ROS2 distros , so it makes a lot of sense to start from this version. 
Deb installation is strongly recomended. Detailed steps to install ROS2 Foxy can be found [here](https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Install-Debians/)

A few helper packages we use fro gui and installation;

.. code-block:: bash

   sudo apt-get install python3-vcstool
   sudo apt-get install xdotool

Finally get the project repository and dependecy repositories and build; 

.. code-block:: bash

   source /opt/ros/foxy/setup.bash
   mkdir -p ~/colcon_ws/src
   cd ~/colcon_ws
   wget https://raw.githubusercontent.com/jediofgever/OUTDOOR_NAV2_UNDERLAY_REPOS/main/underlay.repos
   vcs import src < underlay.repos
   cd ~/colcon_ws
   rosdep install -y -r -q --from-paths src --ignore-src --rosdistro foxy
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-skip botanbot_openvslam

You will be asked to enter your github credentials, enter them correctly, since this repo is private at the moment. 
`rosdep` might take quite long to install all required depenedencies , so please wait for that command to finish. 

You can update dependencies that are build from source such as `navigation2` or `teb_local_planner`
with this convient commands; 

.. note::
   Pay attention that we have disabled the build of botanbot_openvslam. The package has quite some 
   amount of deps. If you like to use please refer to SLAM section , there you will find instructions
   to install botanbot_openvslam deps. After that you ca remove `--packages-skip botanbot_openvslam` part from 
   colcon build command.

.. code-block:: bash

   cd ~/colcon_ws
   vcs import src < underlay.repos
   vcs pull src

We need the source build of some dependencies(e.g `navigation2`), sometimes we need to add/modify functionalities, overall it gives more control for the development. For future we might remove all source built dependencies, but for now the existing 3 better stay as it is since some of them are not avaliable in from debian installations. 
