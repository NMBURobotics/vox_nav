
## Configuration


[Botanbot](https://github.com/jediofgever/botanbot_sim) is good reference to see how it is possible to configure vox_nav for your custom robot.

botanbot is a basic Ackermann type robot with all essential sensors(GPS, LIDAR, CAMERA, IMU). 
botanbot_bringup package includes sample parameters and launch files to run vox_nav.

A full featured parameter file for these nodes are more or less as following;

```yaml
  vox_nav_planner_server_rclcpp_node:
    ros__parameters:
      planner_plugin: "ElevationControlPlanner"          # other options: "SE2Planner", "ElevationPlanner", "OptimalElevationPlanner"
      expected_planner_frequency: 1.0
      planner_name: "PRMstar"                            # PRMstar,LazyPRMstar,RRTstar,RRTsharp,RRTXstatic,InformedRRTstar,BITstar, 
                                                        # ABITstar,AITstar,CForest,LBTRRT,SST,TRRT,SPARS,SPARStwo,FMT,AnytimePathShortening
      planner_timeout: 60.0
      interpolation_parameter: 0                         # set to 0 if you wanna disable interpolating and smooting, otherwise 25 is a good default value                    
      octomap_voxel_size: 0.4
      robot_body_dimens:
        x: 1.2
        y: 1.2
        z: 0.8
      #robot_mesh_path: "package://botanbot_description/meshes/base_simplified.stl" # leave empty if you do not have one, robot_mesh_path: ""
      robot_mesh_path: ""
      SE2Planner:
        plugin: "vox_nav_planning::SE2Planner"         # CForest: Reccomended
        se2_space: "REEDS"                             # "DUBINS","REEDS", "SE2" ### PS. Use DUBINS OR REEDS for Ackermann
        z_elevation: 1.5                               # Elevation of robot from ground plane, robot should not collide with plane                                      
        rho: 2.5                                       # Curve radius form reeds and dubins only
        state_space_boundries: 
          minx: -100.0
          maxx: 100.0
          miny: -100.0
          maxy: 100.0
          minyaw: -3.14
          maxyaw: 3.14
      ElevationPlanner: 
        plugin: "vox_nav_planning::ElevationPlanner"    # PRMstar: Reccomended
        se2_space: "REEDS"                             # "DUBINS","REEDS", "SE2" ### PS. Use DUBINS OR REEDS for Ackermann
        rho: 2.0                                        # Curve radius for reeds and dubins only
        state_space_boundries:
          minx: -100.0
          maxx:  100.0
          miny: -100.0
          maxy:  100.0
          minz: -5.0
          maxz:  10.0
      ElevationControlPlanner: 
        plugin: "vox_nav_planning::ElevationControlPlanner"    # PRMstar: Reccomended
        se2_space: "SE2"                              # "DUBINS","REEDS", "SE2" ### PS. Use DUBINS OR REEDS for Ackermann
        rho: 2.0                                        # Curve radius for reeds and dubins only
        state_space_boundries:
          minx: -100.0
          maxx:  100.0
          miny: -100.0
          maxy:  100.0
          minz: -5.0
          maxz:  10.0    
          minv: -1.5
          maxv: 1.5  
        control_boundries:
          minv: 0.5
          maxv: 0.8
          minw: -0.3
          maxw:  0.3  
      OptimalElevationPlanner: 
        plugin: "vox_nav_planning::OptimalElevationPlanner"    # Bases on Astar on SuperVoxelClustering
        se2_space: "SE2"                                     # "DUBINS","REEDS", "SE2" ### PS. Use DUBINS OR REEDS for Ackermann
        rho: 2.0
        graph_search_method: "astar"                           # Other options: astar , dijkstra
        supervoxel_disable_transform: false                    # set true for organized point clouds
        supervoxel_resolution: 0.4
        supervoxel_seed_resolution: 1.5
        supervoxel_color_importance: 0.0
        supervoxel_spatial_importance: 1.0
        supervoxel_normal_importance: 1.0
        distance_penalty_weight: 10.0
        elevation_penalty_weight: 0.0
        state_space_boundries:
          minx: -50.0
          maxx: 50.0
          miny: -50.0
          maxy: 50.0
          minz: -2.0
          maxz: 12.0

  vox_nav_controller_server_rclcpp_node:
    ros__parameters: 
        controller_plugin: "LyapunovControllerROS"                              # other options: non
        controller_frequency: 100.0                                             # acado is really fast(+1000.0Hz) casadi can deal up to just 20.0Hz maybe
        goal_tolerance_distance: 0.3                                            # in meters, once smaller than this one controller tries to minimize orientation error
        goal_tolerance_orientation: 0.3                                         # in radians, once smaller than this value,controller exits with success
        transform_timeout: 0.01                                                 # seconds, this is used to regulate lookUpTransfrom calls from tf2
        global_plan_look_ahead_distance: 4.5                                    # look this amount of meters from current robot pose to remaining global path
        ref_traj_se2_space: "REEDS"                                             # "DUBINS","REEDS", "SE2" ### PS. Use DUBINS OR REEDS for Ackermann
        rho: 3.0                                                                # Curve radius form reeds and dubins only
        robot_radius: 1.25                                                             
        MPCControllerCasadiROS:
          plugin: "mpc_controller_casadi::MPCControllerCasadiROS"
          N: 10                                                                # timesteps in MPC Horizon
          DT: 0.2                                                              # discretization time between timesteps(s)
          L_F: 0.67                                                            # distance from CoG to front axle(m)
          L_R: 0.67                                                            # distance from CoG to rear axle(m)
          V_MIN: -0.4                                                          # min / max velocity constraint(m / s)
          V_MAX: 1.0
          A_MIN: -0.1                                                          # min / max acceleration constraint(m / s ^ 2)
          A_MAX: 1.0
          DF_MIN: -0.8                                                         # min / max front steer angle constraint(rad)
          DF_MAX: 0.8
          A_DOT_MIN: -0.5                                                      # min / max jerk constraint(m / s ^ 3)
          A_DOT_MAX: 0.5
          DF_DOT_MIN: -0.5                                                     # min / max front steer angle rate constraint(rad / s)
          DF_DOT_MAX: 0.5
          Q: [10.0, 10.0, 0.1, 0.1]                                            # weights on x, y, psi, v.
          R: [10.0, 100.0]                                                     # weights on jerk and slew rate(steering angle derivative)
          debug_mode: False                                                    # enable/disable debug messages
          params_configured: True
          obstacle_cost: 50.0
          max_obstacles: 4

        MPCControllerAcadoROS:
          plugin: "mpc_controller_acado::MPCControllerAcadoROS"
          N: 20                                                                # timesteps in MPC Horizon
          Ni: 1
          DT: 0.2                                                              # discretization time between timesteps(s)
          L_F: 0.67                                                            # distance from CoG to front axle(m)
          L_R: 0.67                                                            # distance from CoG to rear axle(m)
          V_MIN: -0.2                                                          # min / max velocity constraint(m / s)
          V_MAX: 0.4
          A_MIN: -0.1                                                          # min / max acceleration constraint(m / s ^ 2)
          A_MAX: 0.5
          DF_MIN: -0.6                                                         # min / max front steer angle constraint(rad)
          DF_MAX: 0.6
          Q: [1.0, 1.0, 0.0, 0.0, 0.15]                                        # weights on x, y, psi, and v, obstacle_cost.
          R: [10.0, 10.0]                                                      # weights on input acc and df, acceleration and steering angle
          debug_mode: False                                                    # enable/disable debug messages
          params_configured: True
          max_obstacles: 6
          full_ackerman: True

        LyapunovControllerROS:
          plugin: "lyapunov_controller::LyapunovControllerROS"
          V_MIN: -0.5                                                          # min / max velocity constraint(m / s)
          V_MAX: 0.5
          DF_MIN: -0.5                                                         # min / max front steer angle constraint(rad)
          DF_MAX: 0.5
          k1: -1.0
          k2: 5.0
          lookahead_n_waypoints: 2

  vox_nav_map_server_rclcpp_node:
    ros__parameters:
      pcd_map_filename: /home/atas/colcon_ws/src/Thorvald/thorvald_vox_nav/maps/container_office_map.pcd # Provide a PCD format map

      # PCD PREPROCESS PARAMS
      pcd_map_downsample_voxel_size: 0.10                                       # Set to smaller if you do not want downsample pointclouds of the map
      pcd_map_transform:                                                        # Apply an OPTIONAL rigid-body transform to cloud, leave to all zeros if not wished
        translation:                                                            # Unit is meters
          x: 0.0
          y: 0.0
          z: 0.0
        rotation:                                                               # rotation X-Y-Z (r-p-y)sequence to apply to cloud, if you acquired map in camera frame and want to represent
                                                                                # it in base_link or lidar frame, you can specify the rotation here, Unit is Radians
          r: 0.0 #3.14
          p: 0.0 #1.57
          y: 0.0 #1.57
      apply_filters: True                                                       # Optional noise removal steps to apply to map
      remove_outlier_mean_K: 50                                                 # You can set it True and play with parameters if the map is noisy
      remove_outlier_stddev_threshold: 0.06
      remove_outlier_radius_search: 0.6
      remove_outlier_min_neighbors_in_radius: 5
      # COST REGRESSION CRITICS AND PARAMS
      uniform_sample_radius: 0.2
      surfel_radius: 0.6                                                       # Works as resolution of cost regression onto map
      max_allowed_tilt: 0.15                                                   # 1st Cost critic, Any angle(radians) higher than this is marked as NON-traversable
      max_allowed_point_deviation: 0.20                                        # 2nd Cost critic, Point deviation from plane, this could be viewed as roughness of each cell 
      max_allowed_energy_gap: 0.5                                              # 3rd Cost critic, Max Energy in each cell, this is detemined by max height difference between edge points of cell
      node_elevation_distance: 1.2                                             # According to cell_radius, cell centers are sampled from original point cloud map, they are elevated from the original cloud
      plane_fit_threshold: 0.1                                                 # when fitting a plane to each cell, a plane_fit_threshold is considered from plane fitting utility of PCL
      robot_mass: 0.1                                                          # approximate robot mass considering cell_radius, this isnt so important
      average_speed: 1.0                                                       # average robot speed(m/s) when calcuating kinetic energy m = 0.5 * (m * pow(v,2))
      cost_critic_weights: [0.45, 0.45, 0.1]                                   # Give weight to each cost critic when calculating final cost, see above 3 Cost Critic descriptions
      # PCD MAP IS CONVERTED TO OCTOMAP, THIS OCTOMAP IS THEN USED BY PLANNERS FOR
      # COLLISION CHECKING
      octomap_voxel_size: 0.4                                                  # determines resolution of Octomap
      octomap_publish_frequency: 1                                             # Used to determine publish frequncy of octomap visuals(pointclouds and markers)
      publish_octomap_visuals: true
      octomap_point_cloud_publish_topic: "vox_nav/map_server/octomap_pointcloud"                  # sensor_msgs::msg::PoinCloud2 that represents octomap
      octomap_markers_publish_topic: "vox_nav/map_server/octomap_markers"                         # visualization_msgs::msg::MarkeArray that represents octomap
      map_frame_id: "map"                                                      # This should be consistent with Gloabl EKF node , in robot_localization
      utm_frame_id: "utm"
      map_datum:                                                               # Datum coordinates of map is used to geo-reference the map 
                                                                               # If you are building a map with a SLAM method, you need to get a reading from GPS and absolute headed IMU
                                                                               # just before you start to build the map. See for example 
        latitude: 59.66424233333333         
        longitude: 10.76279866666667
        altitude: 0.9
        quaternion:
          x: 0.0
          y: 0.0
          z: 0.0
          w: 1.0
```

Ouch, that is 200 lines of just configuration... Allright I see your point. But keep this in mind, 

At the time of writing, the system incorporates 2 map plugins, 3 planner plugins, and 3 controller plugins. 
So considering you have figured perfect plugins for your robot, the above configuration will look a lot simpler. 
For instance for planner we have picked `ElevationControlPlanner` plugin so the other plugin parameters could have been removed. 
```yaml
planner_plugin: "ElevationControlPlanner" 
```

You will find the most up to date parameter configs under `vox_nav_bringup/params/vox_nav_default_params.yaml`

vox_nav's skeleton is made by following (ROS 2) nodes; 

Some highlights of the fetaures for this nodes are as follows.

### 1. vox_nav_planner_server_rclcpp_node

You can select an available planner plugin(SE2Planner or SE3Planner, etc.), be sure to see through the parameters. 
The planner plugins are interfaced with OMPL. Many of OMPL planners could be selected. 

The planners are Sampling-Based, they utilize a octomap of environment in order to perform collision checks.
You also need to provide a 3D volume box that represents body of your robot. 
see the `robot_body_dimens` params for that.

### 2. vox_nav_controller_server_rclcpp_node

Currently we have 2 MPC and 1 Lyapunov controller for uni-cycle robot models. 
MPC implementations are based on (Casadi)[https://github.com/casadi/casadi] and (Acado)[https://github.com/acado/acado] while Lyapunov controller is a simple [cLf](https://arxiv.org/abs/2210.02837v1).  

### 3. vox_nav_map_server_rclcpp_node
 
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

### 4. vox_nav_navigators

We currently have several behaviour tree nodes in `vox_nav_navigators` package. The most prominent ones are; 

- navigate_to_pose
- navigate_through_poses
- navigate_thorugh_gps_poses

All of this action servers navigate robot to given pose(s). If you see [botanbot_gui](https://github.com/NMBURobotics/botanbot_sim/tree/main/botanbot_gui), we have created simple interface to send a single goal pose
`navigate_to_pose` action server. 
For other two, things are more manual. Since there are more than one pose to navigate, we use YAML file to specify the poses, the action clients in 
[vox_nav_waypoint_nav_clients](https://github.com/NMBURobotics/vox_nav/tree/foxy/vox_nav_waypoint_nav_clients) reads given poses and navigates robot through them. 

The gps poses are `[lat, lang]` format, while normal poses in map coordinate frames as `[x(meter), y(meter), yaw(radians)]`. 

Watch a video of [Thorvald II](https://sagarobotics.com/) robot navigating through gps poses with vox_nav `navigate_thorugh_gps_poses` in this [link](https://www.youtube.com/embed/fe--px9K61A).

 
 