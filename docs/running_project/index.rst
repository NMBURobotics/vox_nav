.. OUTDOOR_NAV2 documentation master file, created by
   sphinx-quickstart on Tue Dec 22 16:24:53 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Running Project
========================================

The project has a RQT GUI plugin that lets you to interact with robot. To start with this plugin make sure in previous sections you built project 
successfully. 

* source your colcon_ws and start the project with;

.. code-block:: bash

   source install/setup.bash
   rqt --force-discover

.. image:: /images/gui_1.png
   :width: 700px
   :align: center
   :alt: rqt landing screen

The rqt window should open as above. You should now find our plugin under; 

`Plugins -> Visualization -> Control Plugin. `

Click on Control Plugin and you would be able to see; 

.. image:: /images/gui_2.png
   :width: 700px
   :align: center
   :alt: rqt landing screen

Interact with GUI
========================================   

Select a world that you would like to run te robot in then Click on Start Gazebo Stand Alone, to start botanbot simulation. 
Note that the Gazebo worlds we use are large, so your computer needs to have an dedicated GPU,it takes apprx. 10 seconds for simulation to start in my case.
After a while you should see Gazebo starting. 

You may not see the Botanbot at first, At left side of Gazebo simulation, find the `models -> botanbot`
right click and then `follow` botanbot model. This should put the focus onto botanbot. 

You can also click on start RVIZ and you should be able to see sensor data and robot model in rviz ; 

.. image:: /images/rviz_1.png
   :width: 700px
   :align: center
   :alt: rqt landing screen

You can jog botanbot with RQT plugin, use `L/R` for giving angular speed and `D/R` for  linear speed. 
