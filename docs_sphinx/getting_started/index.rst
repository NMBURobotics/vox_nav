.. OUTDOOR_NAV2 documentation master file, created by
   sphinx-quickstart on Tue Dec 22 16:24:53 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Getting Started
========================================

* Install ROS2 foxy. 
Deb installation is strongly recomended. 
You can always find updated step to install [here](https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Install-Debians/)

To install ROS2 foxy desktop ;

.. code-block:: bash

   sudo locale-gen en_US en_US.UTF-8
   sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
   export LANG=en_US.UTF-8
   sudo apt update && sudo apt install curl gnupg2 lsb-release
   curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
   sudo sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
   sudo apt update
   sudo apt install ros-foxy-desktop
   source /opt/ros/foxy/setup.bash

* Install all deps and vox_nav;

.. code-block:: bash

   mkdir -p ~/ros2_ws/src
   source /opt/ros/foxy/setup.bash
   sudo apt install python3-colcon-common-extensions
   sudo apt install -y python3-rosdep2
   sudo apt-get install python3-vcstool
   sudo apt-get install xdotool
   sudo apt-get install coinor-libipopt-dev
   rosdep update
   cd ~/ros2_ws
   wget https://raw.githubusercontent.com/jediofgever/vox_nav/foxy/underlay.repos
   vcs import src < underlay.repos --recursive   
   rosdep install -y -r -q --from-paths src --ignore-src --rosdistro foxy  
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DACADOS_WITH_QPOASES=ON -DACADO_CODE_IS_READY=ON -DWITH_IPOPT=true --packages-select ompl casadi; \
   sudo cp install/ompl/lib/libompl.so* /usr/local/lib/
   sudo cp install/casadi/lib/libcasadi.so* /usr/local/lib/ 
   source install/setup.bash  
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DACADOS_WITH_QPOASES=ON -DACADO_CODE_IS_READY=ON -DWITH_IPOPT=true --packages-skip-regex archive --packages-skip vox_nav_control vox_nav_misc; \
   source /opt/ros/foxy/setup.bash
   cd ~/ros2_ws
   source build/ACADO/acado_env.sh
   source install/setup.bash 
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DACADOS_WITH_QPOASES=ON -DACADO_CODE_IS_READY=ON -DWITH_IPOPT=true --packages-select vox_nav_control; \


There are dependency libraries that needs source build perception_pcl ,OMPL and casadi etc..
The above sript will build and install them and remove the source code as its not needed.


.. note::
   Pay attention that we have disabled the build of archived_ packages and vox_nav_misc. The archived package are not in use and not needed but are kept for convenience.
   The cupoch package is really meaty and it is used for ICP, NDT localization algos. 

You can update code by vcstool, you can also us classic git pull. 

.. code-block:: bash

   cd ~/ros2_ws
   vcs import src < underlay.repos
   vcs pull src


.. note::
   If for some reason you could not build vox_nav, a good place to seek for a solution is the github actions file that we have.
   After each pull-push github actions is setup to build the vox_nav on remote to ensure stability of builds. 
   Find a recent successful build and see the commands in .github/workflows/main.yml. The commands should more or less look as in this page.
