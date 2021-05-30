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

* A few helper packages we use for gui and installation;

.. code-block:: bash

   source /opt/ros/foxy/setup.bash
   sudo apt install python3-colcon-common-extensions
   sudo apt install -y python3-rosdep2
   sudo apt-get install python3-vcstool
   sudo apt-get install xdotool
   rosdep update

* Get the project repository, source build deps and build deps first; 

.. code-block:: bash

   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws
   source /opt/ros/foxy/setup.bash
   wget https://raw.githubusercontent.com/jediofgever/vox_nav/foxy/underlay.repos
   vcs import src < underlay.repos     
   rosdep install -y -r -q --from-paths src --ignore-src --rosdistro foxy   
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DWITH_IPOPT=true --packages-select casadi ompl
   sudo cp install/ompl/lib/libompl.so* /usr/local/lib/
   sudo cp install/casadi/lib/libcasadi.so* /usr/local/lib/
   sudo rm -rf src/ompl/
   sudo rm -rf src/casadi/

There are essentially 2-3 dependency libraries that needs source build. 
perception_pcl ,OMPL and casadi. The above sript will build and install them and remove the source code as its not needed.

* With above we only built dependencies, now lest build vox_nav itself

.. code-block:: bash

   source /opt/ros/foxy/setup.bash
   cd ~/ros2_ws
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-skip-regex archived --packages-skip ompl casadi vox_nav_openvslam
   source ~/ros2_ws/install/setup.bash

.. note::
   Pay attention that we have disabled the build of archived_ packages. Some packages(e.g vox_net_openvslam) has quite some 
   amount of deps. If you like to use please refer to SLAM vox_net_openvslam section , there you will find instructions
   to install vox_net_openvslam deps. After that you can remove `--packages-skip vox_net_openvslam` part from 
   colcon build command.

You can update code by vcstool, you can also us classic git pull. 

.. code-block:: bash

   cd ~/ros2_ws
   vcs import src < underlay.repos
   vcs pull src


.. note::
   If for some reason you could not build vox_nav, a good place to seek for a solution is the github actions file that we have.
   After each pull-push github actions is setup to build the vox_nav on remote to ensure stability of builds. 
   Find a recent successful build and see the commands in .github/workflows/main.yml. The commands should more or less look as in this page.
