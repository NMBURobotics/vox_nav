.. OUTDOOR_NAV2 documentation master file, created by
   sphinx-quickstart on Tue Dec 22 16:24:53 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Roadmap and Milestones
========================================

This section is aimed to keep track of development of major features.

- [x] Create an ackermann model robot under Gazebo and decorate with required sensor suite for outdoor navigation. 

- [x] Add different Gazebo worlds(farming like, city like, wild hilly world, empty world with simple shapes in it).

- [x] Add a GUI for interaction with Robot, jogging robot manually, starting shutting down nodes  and launch files, selecting the worlds to start.

- [x] Configure `robot_localization` to obtain map->odom->base_link chain.

- [x] Configure `teb_local_planner` for ackermann robot, as DWB does not work for ackermann.

- [x] Test with `navigation2`, set simple goals and get in goals wtih all functionalities enabled; planner, controller, recovery, obstacle avoidance.

- [x] Add GPS Waypoint following feature.

- [x] Test GPS Waypoint Following feature in a city like envoirment, [Youtube video](https://www.youtube.com/watch?v=DQGfRRn1DBQ&t=13s) .

- [X] Test GPS waypoint following in tomato field [Youtube Video](https://www.youtube.com/watch?v=afxouvL1JAk), this is not pure GPS waypoint following .

- [ ] Add GPS Waypoint following to `navigation2`, Here [nav2_gps_waypoint_follower](https://github.com/ros-planning/navigation2/pull/2111), Progress about 95% .

- [ ] Add tutorial to `navigation2`, on how to make use of `nav2_gps_waypoint_follower`, PR IS OPEN [HERE](https://github.com/ros-planning/navigation2_tutorials/pull/16), Progress about 95%,

- [ ] Consider adding 3D AMCL or another 3D SLAM to help `robot_localization`, see [here](https://answers.ros.org/question/218137/using-robot_localization-with-amcl/) 

- [ ] Can [grid_map](https://github.com/ANYbotics/grid_map/tree/ros2) be beneficial to here? 

- [ ] Test GPS waypoint following on real hardware.

- [ ] Add perception pipeline, at least describe a perception module. 

- [ ] Test multi-robot simulation. more than 1 botanbot doing some task collabratively.


