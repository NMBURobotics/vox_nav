Refer to params/planner_server_config.yml for configurin planner server. 

```yaml
botanbot_planner_server_rclcpp_node:
  ros__parameters:
    planner_plugin: "SE3Planner" # "SE2Planner", "SE3Planner" ,"SE2PlannerControlSpace"
    expected_planner_frequency: 1.0
    SE2Planner:
      plugin: "botanbot_planning::SE2Planner"
      planner_name: "PRMStar" # other options: RRTStar, RRTConnect, KPIECE1
      planner_timeout: 1.0
      interpolation_parameter: 50
      octomap_filename: "/home/ros2-foxy/f.bt"
      octomap_voxel_size: 0.1
      state_space_boundries:
        minx: -50.0
        maxx: 50.0
        miny: -50.0
        maxy: 50.0
        minz: 0.0
        maxz: 3.0
      robot_body_dimens:
        x: 1.0
        y: 0.5
        z: 0.4
    SE3Planner:
      plugin: "botanbot_planning::SE3Planner"
      planner_name: "KPIECE1"  # other options: PRMStar  RRTConnect, KPIECE1
      planner_timeout: 1.0
      interpolation_parameter: 50
      octomap_filename: "/home/ros2-foxy/f.bt"
      octomap_voxel_size: 0.1
      state_space_boundries:
        minx: -50.0
        maxx: 50.0
        miny: -50.0
        maxy: 50.0
        minz: 0.0
        maxz: 3.0
      robot_body_dimens:
        x: 1.0
        y: 0.5
        z: 0.4    
    SE2PlannerControlSpace:
      plugin: "botanbot_planning::SE2PlannerControlSpace"
      planner_name: "RRT"  # other options: SST , EST, KPIECE1
      planner_timeout: 1.0
      interpolation_parameter: 50
      octomap_filename: "/home/ros2-foxy/f.bt"
      octomap_voxel_size: 0.1
      state_space_boundries:
        minx: -50.0
        maxx: 50.0
        miny: -50.0
        maxy: 50.0
        minz: 0.0
        maxz: 3.0
      robot_body_dimens:
        x: 1.0
        y: 0.5
        z: 0.4      
```

Select a plugin out of 3 available plugins listed as ; SE2Planner, SE3Planner, SE2PlannerControlSpace. 

Launch the planner server with; 

```bash
ros2 launch botanbot_planning planner_server.launch.py
```

Test the planner by sending an all zeros goal with ;
 
```bash
ros2 action send_goal /compute_path_to_pose botanbot_msgs/action/ComputePathToPose "{}"
``` 