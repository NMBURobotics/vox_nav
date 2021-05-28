# Utilities provoided

## Converting PCD files to Octomap.

Be sure to see the configuration file under config directory. 
Give full path to pcd file you would like to convert. Also give a vaild path for output file. 
You can define transforms, e.g if your pcd file is in camera frmaes but you would like to acquire 
Octomap in another frame, then just provide the rigid body transform to cloud_transform secion below.

Optionally you can remove some outliers from your cloud. Play with paraeters until your happy with your Octomap. 



```yaml
pcl2octomap_converter_rclcpp_node:
  ros__parameters:
    input_pcd_filename: /home/atas/pointnet2_pytorch/data/decomposed_traversability_cloud.pcd
    output_binary_octomap_filename: /home/atas/pointnet2_pytorch/data/decomposed_traversability_cloud.ot
    octomap_voxelsize: 0.2
    cloud_transform:
      translation:
        x: 0.0
        y: 0.0
        z: 0.0 #1.0
      rotation: #intrinsic rotation X-Y-Z (r-p-y)sequence
        r: 0.0 #3.14
        p: 0.0 #1.57
        y: 0.0 #1.57
    apply_filters: False    
    downsample_voxel_size: 0.1
    remove_outlier_mean_K: 50
    remove_outlier_stddev_threshold: 0.1
    remove_outlier_radius_search: 0.1
    remove_outlier_min_neighbors_in_radius: 1
```

Use Converting node with following command.

```bash
ros2 launch vox_nav_utilities pcl2octomap_converter.launch.py
 ```
Depending on the size of pointcloud map, this might take a while. Finally you should be able to see some meta information of created map on terminal output. 

## Collecting GPS waypoints

It is possible to collect GPS waypoins. This node is very simple. 
It basically subscribes to GPS and IMU topics and writes sensor data to terminal. 
You can then copy from terminal to another text file, for waypoint navigation. Be sure to chechk the launch file, you need to remap topic names correctly. 

```bash
ros2 launch vox_nav_utilities gps_collector.launch.py 
 ```

## Executing a benchmarking for various path planners

See the planner_benchmarking confif parameters. 
Basically you select a number of OMPL planners, a state-space and some other trivial parameters. 

The planning benchmark requires a Octomap to make plans in. It is supposed that an valid octomap is published. 

In the planner config you should specify the topic name and voxel_size for the Octomap. 

```yaml
planner_benchmarking_rclcpp_node:
  ros__parameters:
    planner_timeout: 1.0 # if planner cannot find a valid plan within this time, terminate the planning request
    interpolation_parameter: 120 # interpolate resulting path suvh that it has this amount of states
    octomap_topic: "octomap" # Topic to subscribe
    octomap_voxel_size: 0.2 # voxel size of Octomap
    selected_state_space: "DUBINS" # "DUBINS","REEDS", "SE2", "SE3" ### PS. Use DUBINS OR REEDS for Ackermann
    selected_planners: ["PRMstar","LazyPRMstar", "RRTstar", "RRTsharp", "RRTXstatic", 
                        "InformedRRTstar", "BITstar", "ABITstar","AITstar", "LBTRRT",
                        "SST", "SPARS", "SPARStwo","FMT", "CForest","AnytimePathShortening"] # This are all existing optimizing planners in OMPL
    min_turning_radius: 1.5 # This parame is important only for DUBINS and REEDS, it determines the radius of circle, select it accrding to your robot kinematics
    state_space_boundries: # Define hard boundries for state space
      minx: -45.0
      maxx: 45.0
      miny: -45.0
      maxy: 45.0
      minz: 1.0
      maxz: 5.5
      minyaw: -3.14
      maxyaw: 3.14
    robot_body_dimens: # A 3d box volume that surrounds robot body, it is used for collisison check
      x: 1.5
      y: 1.5
      z: 0.4
    start: # Only define elevation of start and goals, x and y are randomly generated in the node
      z: 3.5  #3.5
    goal:
      z: 3.5 #3.5
    goal_tolerance: 0.2 # max deviation form goal, in meters
    min_euclidean_dist_start_to_goal: 85.0 # random problem generation will generate start(x,y) and goal(x,y) with minimum of this distance inbetween
    batch_size: 1 # How many times to perform benchmark for a single geneated problem
    epochs: 1 # How many differnt problems to generate
    max_memory: 4096
    results_output_dir: "/home/ros2-foxy/ECMR2021/benchmark_results/" #give a valid directory so that we can dump results   
    results_file_regex: "DUBINS"  # sleect it according to your state space
    publish_a_sample_bencmark: true #it will publish a sample plan form each plan in RVIZ marker
    sample_bencmark_plans_topic: "benchmark_plan" # visualize plan samples with this topi in RVIZ
```

```bash
ros2 launch vox_nav_utilities planner_benchmarking.launch.py 
 ```

## Generating a octomap from a gazebo world

 Call the service with following to request a conversion from gazebo world to octomap
```bash
 ros2 service call /world/build_octomap vox_nav_msgs/srv/GetOctomap "{bounding_box_origin: {x: 0, y: 0, z: 10}, bounding_box_lengths: {x: 120, y: 60, z: 20}, leaf_size: 0.2, filename: /home/ros2-foxy/output_filename.bt}"
 ```
 you also need to make sure the world plugin is placed under <world> tags of gazebo.
```xml
<plugin name="gazebo_world_to_octomap" filename="libgazebo_world_to_octomap.so">
    <octomapPubTopic>world/octomap</octomapPubTopic>
    <octomapServiceName>world/build_octomap</octomapServiceName>
</plugin>
```

## Generating a pointcloud map from a gazebo world


It is possible to generate label pointcloud data for training a network to assess traversability of a pointcloud map. This plugin is parelel to 
generating an octomap from gazebo world.

Call the service with following to request a conversion from gazebo world to pointcloud map

```bash
ros2 service call /world/build_pointcloud vox_nav_msgs/srv/GetPointCloud "{bounding_box_origin: {x: 0, y: 0, z: 10}, bounding_box_lengths: {x: 120, y: 60, z: 20}, leaf_size: 0.2, filename: /home/ros2-foxy/output_filename.pcd}"
 ```
you also need to make sure the world plugin is placed under <world> tags of gazebo.

```xml
<plugin name="gazebo_world_to_pointcloud" filename="libgazebo_world_to_pointcloud.so">
    <pointcloudPubTopic>world/pointcloud</pointcloudPubTopic>
    <pointcloudServiceName>world/build_pointcloud</pointcloudServiceName>
</plugin>
```