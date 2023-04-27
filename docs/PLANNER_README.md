## Planner Server

> **_NOTE:_** Under Construction!

The planner plugins are classified based on the state space in which the robot operates, such as SE2 or SE3. 
Various parameters can be specified for the plugins, including the collision body dimensions of the robot, state boundaries, and actual planning algorithms. 

Refer to bringup/params.yaml for configuring planner server. 

```yaml
vox_nav_planner_server_rclcpp_node:
  ros__parameters:
    planner_plugin: "PolyTunnelPlanner"          # other options: "SE2Planner", "ElevationPlanner", "OptimalElevationPlanner"
    expected_planner_frequency: 1.0
    planner_name: "LQRRRTStar"                            
    planner_timeout: 35.0
    interpolation_parameter: 0                         # set to 0 if you wanna disable interpolating and smooting, otherwise 25 is a good default value                    
    octomap_voxel_size: 0.4
    robot_body_dimens:
      x: 1.2
      y: 1.2
      z: 0.8
    #robot_mesh_path: "package://botanbot_description/meshes/base_simplified.stl" # leave empty if you do not have one, robot_mesh_path: ""
    robot_mesh_path: ""
    SE2Planner:
      # Available Planners Include: PRMstar,LazyPRMstar,RRTstar,RRTsharp,RRTXstatic,InformedRRTstar,BITstar, 
                                 #  ABITstar,AITstar,CForest,LBTRRT,SST,TRRT,SPARS,SPARStwo,FMT,AnytimePathShortening
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
      # Available Planners Include: PRMstar,LazyPRMstar,RRTstar,RRTsharp,RRTXstatic,InformedRRTstar,BITstar, 
                                 #  ABITstar,AITstar,CForest,LBTRRT,SST,TRRT,SPARS,SPARStwo,FMT,AnytimePathShortening
      plugin: "vox_nav_planning::ElevationPlanner"    # PRMstar: Reccomended
      se2_space: "REEDS"                              # "DUBINS","REEDS", "SE2" ### PS. Use DUBINS OR REEDS for Ackermann
      rho: 2.0                                        # Curve radius for reeds and dubins only
      state_space_boundries:
        minx: -100.0
        maxx:  100.0
        miny: -100.0
        maxy:  100.0
        minz: -5.0
        maxz:  10.0
    ElevationControlPlanner: 
      # Available Planners Include: RRT,RRTStarF,LQRPlanner,LQRRRTStar,SST,EST, KPIECE1
      plugin: "vox_nav_planning::ElevationControlPlanner"    # RRTStarF: Reccomended
      se2_space: "SE2"                                # "DUBINS","REEDS", "SE2" ### PS. Use DUBINS OR REEDS for Ackermann
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
      se2_space: "SE2"                                      # "DUBINS","REEDS", "SE2" ### PS. Use DUBINS OR REEDS for Ackermann
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
    PolyTunnelPlanner:
      plugin: "vox_nav_planning::PolyTunnelPlanner"
      ref_traj_se2_space: "DUBINS"                             # "DUBINS","REEDS", "SE2" ### PS. Use DUBINS OR REEDS for Ackermann
      rho: 1.0      
      transform_timeout: 0.2
      resolution: 0.01
      row_cloud_downsample_size: 0.4
      extra_interpolation: 60
```

### 1. SE2Planner Plugin

### 2. ElevationPlanner Plugin

### 3. ElevationControlPlanner Plugin

### 4. OptimalElevationPlanner Plugin

### 5. PolyTunnelPlanner Plugin

### 6. Test the planner

By sending an all zeros goal with ;
 
```bash
ros2 action send_goal /compute_path_to_pose vox_nav_msgs/action/ComputePathToPose "{}"
``` 