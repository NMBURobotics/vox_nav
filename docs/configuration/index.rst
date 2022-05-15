
Configuration
========================================

Botanbot sim is good reference to see how it is possible to configure vox_nav for your custom robot.
Be aware that at the moment we only cover Ackerman type robot. 
Although it is possible to configure MPC Controller's params, such that it works for 
Diff-Drive robots, the primary model is aimed for Ackerman type robots.

Please see botanbot_sim in this link; https://github.com/jediofgever/botanbot_sim

botanbot is a basic Ackermann type robot with all essential sensors(GPS, LIDAR, CAMERA, IMU). 
botanbot_bringup package includes sample parameters and launch files to run vox_nav.

Important nodes of vox_nav are; conroller_node, map_server_node, planning_node. 

parameters for these nodes are more or less as following;

.. code-block:: yaml

  vox_nav_planner_server_rclcpp_node:
    ros__parameters:
      planner_plugin: "ElevationPlanner"              # other options: "SE2Planner", "ElevationPlanner"
      expected_planner_frequency: 1.0
      planner_name: "PRMstar"                         # PRMstar,LazyPRMstar,RRTstar,RRTsharp,RRTXstatic,InformedRRTstar,BITstar, 
      interpolation_parameter: 20                     # ABITstar,AITstar,CForest,LBTRRT,SST,TRRT,SPARS,SPARStwo,FMT,AnytimePathShortening
      planner_timeout: 10.0
      octomap_voxel_size: 0.2
      robot_body_dimens:
        x: 1.2
        y: 0.9
        z: 0.8
      robot_mesh_path: "package://botanbot_description/meshes/base_simplified.stl" # leave empty if you do not have one, robot_mesh_path: ""
      SE2Planner:
        plugin: "vox_nav_planning::SE2Planner"        # CForest: Reccomended
        se2_space: "REEDS"                            # "DUBINS","REEDS", "SE2" ### PS. Use DUBINS OR REEDS for Ackermann
        z_elevation: 1.5                              # Elevation of robot from ground plane, robot should not collide with plane                                      
        rho: 2.5                                      # Curve radius form reeds and dubins only
        state_space_boundries: 
          minx: -50.0
          maxx: 50.0
          miny: -50.0
          maxy: 50.0
          minyaw: -3.14
          maxyaw: 3.14
      ElevationPlanner: 
        plugin: "vox_nav_planning::ElevationPlanner"    # PRMstar: Reccomended
        se2_space: "DUBINS"                             # "DUBINS","REEDS", "SE2" ### PS. Use DUBINS OR REEDS for Ackermann
        rho: 1.0                                        # Curve radius for reeds and dubins only
        state_space_boundries:
          minx: -50.0
          maxx: 50.0
          miny: -50.0
          maxy: 50.0
          minz: -2.0
          maxz: 12.0

  vox_nav_controller_server_rclcpp_node:
    ros__parameters:
        controller_plugin: "MPCControllerCasadiROS"                                   # other options: non
        controller_frequency: 15.0
        MPCControllerCasadiROS:
          plugin: "mpc_controller_casadi::MPCControllerCasadiROS"
          N: 8                                                                 # timesteps in MPC Horizon
          DT: 0.2                                                              # discretization time between timesteps(s)
          L_F: 0.66                                                            # distance from CoG to front axle(m)
          L_R: 0.66                                                            # distance from CoG to rear axle(m)
          V_MIN: -1.0                                                          # min / max velocity constraint(m / s)
          V_MAX: 2.0
          A_MIN: -2.0                                                          # min / max acceleration constraint(m / s ^ 2)
          A_MAX: 1.0
          DF_MIN: -1.5                                                         # min / max front steer angle constraint(rad)
          DF_MAX: 1.5
          A_DOT_MIN: -1.0                                                      # min / max jerk constraint(m / s ^ 3)
          A_DOT_MAX: 1.0
          DF_DOT_MIN: -0.8                                                     # min / max front steer angle rate constraint(rad / s)
          DF_DOT_MAX: 0.8
          Q: [10.0, 10.0, 0.1, 0.1]                                            # weights on x, y, psi, and v.
          R: [10.0, 100.0]                                                     # weights on jerk and slew rate(steering angle derivative)
          debug_mode: False                                                    # enable/disable debug messages
          params_configured: True

  vox_nav_map_server_rclcpp_node:
    ros__parameters:
      pcd_map_filename: /home/atas/colcon_ws/src/botanbot_sim/botanbot_bringup/maps/uneven_world.pcd
      # PCD PREPROCESS PARAMS
      pcd_map_downsample_voxel_size: 0.2                                        # Set to smaller if you do not want downsample
      pcd_map_transform:                                                        # Apply an optional rigid-body transrom to pcd file
        translation:
          x: 0.0
          y: 0.0
          z: 0.0 #1.0
        rotation:                                                               #intrinsic rotation X-Y-Z (r-p-y)sequence
          r: 0.0 #3.14
          p: 0.0 #1.57
          y: 0.0 #1.57
      apply_filters: False
      remove_outlier_mean_K: 50
      remove_outlier_stddev_threshold: 0.1
      remove_outlier_radius_search: 0.1
      remove_outlier_min_neighbors_in_radius: 1
      # COST REGRESSION CRITICS AND PARAMS
      cell_radius: 0.8                                                         # Works as resolution of cost regression onto map
      max_allowed_tilt: 0.6                                                    # 1st Cost critic Any angle(radians) higher than this is marked as NON-traversable
      max_allowed_point_deviation: 0.2                                         # 2nd Cost critic Point deviation from plane, this could be viewed as roughness of each cell 
      max_allowed_energy_gap: 0.2                                              # 3rd Cost critic Max Energy in each cell, this is detemined by max height differnce between edge points of cell
      node_elevation_distance: 1.8                                             # According to cell_radius, cell centers are sampled from original point cloud map, they are elevated from the original cloud
      plane_fit_threshold: 0.1                                                 # when fitting a flan to each cell, a plane_fit_threshold is considered from plane fitting f PCL
      robot_mass: 0.1                                                          # approximate robot mass considering cell_radius
      average_speed: 1.0                                                       # average robot speed(m/s) when calcuating kinetic energy m = 0.5 * (m * pow(v,2))
      cost_critic_weights: [0.6, 0.2, 0.2]                                     # Give weight to each cost critic wen calculating final cost
      # PCD MAP IS TRANSLATED TO OCTOMAP TO BE USED BY PLANNER
      octomap_voxel_size: 0.2
      octomap_publish_frequency: 1
      publish_octomap_visuals: true
      octomap_point_cloud_publish_topic: "octomap_pointcloud"                  # sensor_msgs::msg::PoinCloud2 that represents octomap
      octomap_markers_publish_topic: "octomap_markers"                         # visualization_msgs::msg::MarkeArray that represents octomap
      map_frame_id: "map"
      utm_frame_id: "utm"
      yaw_offset: 1.57                                                         # see navsat_transform_node from robot_localization, this offset is needed to recorrect orientation of static map
      map_datum:
        latitude: 49.89999996757017
        longitude: 8.899999997371747
        altitude: 1.6
        quaternion:
          x: -0.0001960611448920198
          y: -0.003682083159658604
          z: 4.672499893387009e-05
          w: 0.9999932007970892


vox_nav's skeleton is made by following ROS2 nodes; 

Some highlights of the fetaures for this nodes are as follows.

**1. vox_nav_planner_server_rclcpp_node**

You can select an available planner plugin(SE2Planner or SE3Planner), be sure to see through the parameters. 
SE2Planner can be configured such that kinematic constrains
of ackemann robots are respected. e.g select REEDS OR DUBINS spaces.
The planner plugins are interfaced with OMPL. Many of OMPL planners could be selected. 

The planners are Sampling-Based, they utilize a octomap of environment in order to perform collision checks.
You also need to provide a 3D volume box that represents body of your robot. 
see the robot_body_dimens params for that.

**2. vox_nav_controller_server_rclcpp_node**

Currently we have 2 MPC and 1 Lyapunov controller for uni-cycle robot models. 
MPC implementations are based on (Casadi)[https://github.com/casadi/casadi] and (Acado)[https://github.com/acado/acado] while Lyapunov controller is a simple cLf.  

**3. vox_nav_map_server_rclcpp_node**
 
You will need to provide a pre-built pcd map of environment for this node to consume. 
This map needs to have a datum of its origin(GPS coordinates and IMU acquired absolute heading). 
This is basically the pose where you initialize your SLAM algorithm to build your map. 
This is needed in order to geo-reference your map.
vox_nav_openvslam can help you with building such maps, these is also a helper node to dump map meta information including datum.
Refer to SLAM section to see more details. 
With this information the node is able to grab your pcd map and georeference it utilizing robot_localization package. 
The pcd map is converted to an octomap and published with configured voxel sizes and topic names. 
You should visualize topics in RVIZ, in order to make sure the map looks as expected.
visualizing as markers usually lags RVIZ, instead we recomend you to visualize pointcloud topic of octomap.

**4. vox_nav_navigators**

We currently have several behaviour tree nodes in this package. The most promeinent ones are; 

- navigate_to_pose
- navigate_through_poses
- navigate_thorugh_gps_poses

All of this action servers navigates robot to given pose(s). If you see (botanbot_gui)[https://github.com/NMBURobotics/botanbot_sim/tree/main/botanbot_gui], we have created simple interface to send a single goal pose
`navigate_to_pose` action server. For other two, things are more manual. Since there are more than one pose to navigate, we use YAML file to specify the poses, the action clients in 
(vox_nav_waypoint_nav_clients)[https://github.com/NMBURobotics/vox_nav/tree/foxy/vox_nav_waypoint_nav_clients] reads given poses and navigates robot through them. 

The gps poses are `[lat, lang]` format, while normal poses in map coordinate frames as `[x(meter), y(meter), yaw(radians)]`. 

Watch a shiny video of (Thorvald II)[https://sagarobotics.com/] robot navigating through gps poses with vox_nav below.

 .. raw:: html

  <iframe width="1046" height="294" src="https://www.youtube.com/embed/fe--px9K61A" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>