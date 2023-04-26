## Map Server



> **_NOTE:_** Under Construction!

## Point Cloud Map;

For an outdoor robot operating in uneven outdoor terrains, a 3D point cloud map serves as an effective tool for representing the environment. The objective of this plugin is to furnish a map that incorporates semantic data, such as whether a particular region of the environment is traversable or not. The plugin offers the map and semantic data to the planner or controller plugins through ROS servers and clients.

The default map parameters are as following. 

```yaml
vox_nav_map_server_rclcpp_node:
  ros__parameters:
    octomap_filename: "/home/ros2-foxy/slope_map.bt"  # Full path to pcd map
    octomap_voxel_size: 0.2                           # An octomap will be contructed based on this point cloud,, you can determine the resolution
    octomap_publish_frequency: 50                     # Publish the pointcloud to other nodes 
    provide_utm_to_map_transform: true                # This map is working with utm->map->static_map TF tree, essentially the map is aligned W.R.T robot initial position (which is the map frame)   
    publish_octomap_as_pointcloud: true               # for visualizing
    octomap_publish_topic_name: "octomap"             # octomap_msgs::msg::Octomap type of message topic name 
    octomap_point_cloud_publish_topic: "octomap_pointcloud" # sensor_msgs::msg::PoinCloud2 that represents octomap
    map_frame_id: "map"                               
    static_map_frame_id: "static_map"
    utm_frame_id: "utm"
    yaw_offset: 1.57                                  #see navsat_transform_node from robot_localization, this offset is needed to recorrect orientation of static map
    # The GPS coordinates of this map, this is the initial pose of the robot when you started to do mapping (SLAM)
    map_datum:
      latitude: 49.89996853007036
      longitude: 8.900083512416545
      altitude: 0.6342219080870174
      quaternion:
        x: -0.0002674301579378746
        y: -7.80412511940006e-05
        z: 0.7068413617683594
        w: 0.7073720461568473
```

The point cloud map can be built with sevaral SLAM methods such as LIO-SAM or FAST-LIO. The point cloud map plugin regresses the costs to point cloud map, the resulting `traverability` map then can be used for navigation. See details of the approach in related [publication](https://arxiv.org/abs/2208.08202). 

## OSM Point Cloud Map

Under Construction