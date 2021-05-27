Refer to bringup/params.yaml for configuring planner server. 

```yaml
vox_nav_planner_server_rclcpp_node:
  ros__parameters:
    planner_plugin: "SE2PlannerControlSpace" # other options: "SE2Planner", "SE3Planner" ,"SE2PlannerControlSpace"
    expected_planner_frequency: 10.0
    SE2Planner:
      plugin: "vox_nav_planning::SE2Planner"
      planner_name: "RRTStar" # other options: RRTStar, RRTConnect, KPIECE1, SBL, SST
      planner_timeout: 3.0
      interpolation_parameter: 50
      octomap_topic: "octomap"
      octomap_voxel_size: 0.2
      se2_space: "DUBINS" # "DUBINS","REEDS", "SE2" ### PS. Use DUBINS OR REEDS for Ackermann
      state_space_boundries:
        minx: -50.0
        maxx: 50.0
        miny: -50.0
        maxy: 50.0
        minyaw: -3.14
        maxyaw: 3.14
      robot_body_dimens:
        x: 1.5
        y: 1.0
        z: 0.4  
    SE2PlannerControlSpace:
      plugin: "vox_nav_planning::SE2PlannerControlSpace"
      planner_name: "RRTstar"  # other options: RRT, SST, EST, KPIECE1, PRMstar, RRTstar
      planner_timeout: 2.0
      interpolation_parameter: 50
      octomap_topic: "octomap"
      octomap_voxel_size: 0.2
      se2_space: "REEDS" # "DUBINS","REEDS", "SE2" ### PS. Use DUBINS OR REEDS for Ackermann
      state_space_boundries:
        minx: -50.0
        maxx: 50.0
        miny: -50.0
        maxy: 50.0
        minyaw: -3.14
        maxyaw: 3.14
      velocity_space_boundries:
        min_v: -0.4
        max_v: 0.4 
      control_input_bounds:
        min_acc: -0.3
        max_acc: 0.3
        min_steer: -0.5
        max_steer: 0.5
      robot_body_dimens:
        x: 1.5
        y: 1.0
        z: 0.4  
    SE3Planner:
      plugin: "vox_nav_planning::SE3Planner"
      planner_name: "RRTStar"  # other options: PRMStar, RRTStar, RRTConnect, KPIECE1, , 
      planner_timeout: 1.0
      interpolation_parameter: 25
      octomap_topic: "octomap"
      octomap_voxel_size: 0.2
      state_space_boundries:
        minx: -50.0
        maxx: 50.0
        miny: -50.0
        maxy: 50.0
        minz: 0.0
        maxz: 2.5
      robot_body_dimens:
        x: 1.5
        y: 1.0
        z: 0.2   
```

Select a plugin out of 3 available plugins listed as ; SE2Planner, SE3Planner, SE2PlannerControlSpace. 

Test the planner by sending an all zeros goal with ;
 
```bash
ros2 action send_goal /compute_path_to_pose vox_nav_msgs/action/ComputePathToPose "{}"
``` 