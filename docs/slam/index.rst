.. OUTDOOR_NAV2 documentation master file, created by
   sphinx-quickstart on Tue Dec 22 16:24:53 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

SLAM
========================================

------------ 

 **1. Related to Google Cartographer 3D SLAM**

Unfourtunately Cartographer has not been ported to ROS2 completely ATM. 
We can still create maps and visualize results, the `botanbot_cartogrpaher` is correctly configured and tested. 
But the node named `cartographer_asset_writter` has not been ported,this node is used to 
recieve actual 3D map in .pcd or other formats, therefore we cannot exploit created map now. 
Hopefully soon enough this will be available. For exploting the actual maps created see the next SLAM options below this section.
We have added configuration package(`botanbot_cartogrpaher`) in order to build 3D maps using google cartogrpaher.


In order to use the provided configuration package, 
one will ned to create ros2 bag files with required topics. 
For now we use 1 LIDAR and 1 IMU(Can be changed to 2 LIDAR in future). 
In order to record a bag file this the required topics, do the following; 

.. code-block:: bash
   
   ros2 bag record /velodyne_points /imu/data

A bag file will be created. Cartogrpaher expects that you define the rigid body 
transforms between sensor links and robot body frame(base_link). 
This transforms are defined in `archived_botanbot_cartographer/urdf`. 
You might need to modify translation and roation between velodyne sensor and imu sensor 
for different setup. A strict calibration might not be necesarry between IMU and LIDAR. 

Also see the `cartogrpaher.launch.py` file and make sure the data topics are remapped correctly. 
After we have the bag file and configuration ready, we do the following to build the 3D map. 

.. code-block:: bash

   ros2 launch archived_botanbot_cartographer cartographer.launch.py use_sim_time:=true bag_file:=${HOME}/rosbag2_2020_12_18-10_25_37/rosbag2_2020_12_18-10_25_37_0.db3

Wait for cartogrpaher to finish and do optimizations on the map. 

------------ 

**2. Related to LIDAR SLAM ROS2 3D PACKAGE**

As second(but primary for now) SLAM package we have [lidarslam_ros2](https://github.com/jediofgever/lidarslam_ros2) package. 
The package is able to generate nice maps(in city like envoirnments) based on GICP/NDT. 

This package will be auomatically cloned to workspace with the vcs tool. 
Following same fashion with google cartogrpaher, 
we need to record bag files and generate maps with data inside this bags. 
Assuming that we have recorded bag file that contains point cloud and imu(optional) with topic names `velodyne_points`, `imu/data`, we can generate maps with ;

.. code-block:: bash

   ros2 bag play -r 0.5 rosbag2_2020_12_18-10_25_37/
   ros2 launch lidarslam lidarslam.launch.py
   rviz2 -d src/lidarslam_ros2/lidarslam/rviz/mapping.rviz

Note; if the sensor topic names are different then you need to recorrect them in tha launch file `lidarslam.launch.py`
execute all commands in seperate terminals. 
Here are a few example maps crreated when botanbot was taking a tour to gas station and retruning back. 

.. image:: ../images/slam_0.png
   :width: 700px
   :align: center
   :alt: rqt landing screen

.. image:: ../images/slam_1.png
   :width: 700px
   :align: center
   :alt: rqt landing screen

.. image:: ../images/slam_2.png
   :width: 700px
   :align: center
   :alt: rqt landing screen

**3. Related to OpenVSLAM**

OpenVSLAM[1] is a open ource Visual SLAM framework. It support several types of camera models in order to achieve SLAM only based on a camera image. 
In agri-fields the repetitive/featuresless , texturless envoirnments leads to poor results on LIDAR based SLAM. or instance Neither with Cartographer or LIDAR SLAM package 

We could build a reasonable map. However openvslam perfromed quite ok in the tomato_field world. 

.. image:: ../images/openvslam_0.png
   :width: 700px
   :align: center
   :alt: rqt landing screen

Installation and Usage of OpenVSLAM
========================================

Altough it is best to refer to their read the docs website here; https://openvslam.readthedocs.io/en/develop/installation.html
for an updated information related to installation,
we will still provide steps here to achieve instllation of this complcated software piece.

Most of openvslam dependencies will be already exstent in your Ubuntu system but make sure you have all following dependecises; 

.. code-block:: bash

   Eigen : version 3.3.0 or later.
   g2o : Please use the latest release. Tested on commit ID 9b41a4e.
   SuiteSparse : Required by g2o.
   DBoW2 : Please use the custom version of DBoW2 released in https://github.com/shinsumicco/DBoW2.
   yaml-cpp : version 0.6.0 or later.
   OpenCV : version 3.3.1 or later.
   Pangolin : Required for visualization and GUI.

yaml-cpp;

.. code-block:: bash

   sudo apt-get install libyaml-cpp-dev

Some of above deps will need a source build. Which can be done as ; 

DBoW2; 

.. code-block:: bash
    
   cd ~/
   git clone https://github.com/shinsumicco/DBoW2.git
   cd DBoW2
   mkdir build && cd build
   cmake \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/usr/local \
      ..
   make -j8
   sudo make install

g2o; 

.. code-block:: bash

   cd ~/
   git clone https://github.com/RainerKuemmerle/g2o.git
   cd g2o
   git checkout 9b41a4ea5ade8e1250b9c1b279f3a9c098811b5a
   mkdir build && cd build
   cmake \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/usr/local \
      -DCMAKE_CXX_FLAGS=-std=c++11 \
      -DBUILD_SHARED_LIBS=ON \
      -DBUILD_UNITTESTS=OFF \
      -DBUILD_WITH_MARCH_NATIVE=OFF \
      -DG2O_USE_CHOLMOD=OFF \
      -DG2O_USE_CSPARSE=ON \
      -DG2O_USE_OPENGL=OFF \
      -DG2O_USE_OPENMP=ON \
      ..
   make -j4
   sudo make install

Pangolin; 

.. code-block:: bash
   cd ~/
   git clone https://github.com/stevenlovegrove/Pangolin.git
   cd Pangolin
   git checkout ad8b5f83222291c51b4800d5a5873b0e90a0cf81
   mkdir build && cd build
   cmake \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/usr/local \
      ..
   make -j4
   sudo make install

Build openvslam

.. code-block:: bash

   cd ~/
   git clone https://github.com/xdspacelab/openvslam.git
   cd openvslam
   git checkout develop
   mkdir build && cd build
   cmake \
      -DBUILD_WITH_MARCH_NATIVE=OFF \
      -DUSE_PANGOLIN_VIEWER=ON \
      -DUSE_SOCKET_PUBLISHER=OFF \
      -DUSE_STACK_TRACE_LOGGER=ON \
      -DBOW_FRAMEWORK=DBoW2 \
      -DBUILD_TESTS=ON \
      ..
   make -j4
   sudo make install

After all of this `botanbot_openvslam` should compile fine.

In order to build a map with provided server package do following; 

.. code-block:: bash

   ros2 launch botanbot_openvslam openvslam_mapping.launch.py output_map_filename:=${HOME}/test_map.msg

Currently mono and RGBD cameras are suppoorted. RGBD is recomended and in the deafult settings we use ATM.
Mono images cannot be correctly scaled to real world unlike RGBD. See the mapping.launch.py under `botanbot_openvslam` and make sure 
the camera topics are corrctly remapped. 

.. note:: text
   Visual SLAM has difficulties dealing with pure rotations. So the robot needs at least some translation as well when taking sharp 
   turns. 


Jog the robot with rqt gui plugin and visualize the map with pangolin viewer. A map with extension of `.msg` will be dumped 
to the path you passed to output_map_filename. The scripts provided in `botanbot_openvslam` are able to visualize 
and convert this .msg to .pcd extension. 

In order to visualize the created map do the following

make sure to cd ino `botanbot_openvslam/scripts`
.. code-block:: bash

   python3 visualize_openvslam_map.py map.msg

.. image:: ../images/openvslam_0.png
   :width: 700px
   :align: center
   :alt: rqt landing screen

.. image:: ../images/openvslam_1.png
   :width: 700px
   :align: center
   :alt: rqt landing screen

You can convert .msg to .pcd with provided script; 

.. code-block:: bash

   python3 convert_msg_to_pcd.py map.msg out.pcd


Lastly Localization can be perfromed in a pre build map ; 

.. code-block:: bash
   ros2 launch botanbot_openvslam openvslam_localization.launch.py output_map_filename:=${HOME}/test_map.msg

where the argument is pull path to prebuild map in .msg format.

[1](https://github.com/xdspacelab/openvslam)


