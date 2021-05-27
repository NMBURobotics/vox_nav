Refer to bringup/params.yaml for configuring map server. 

```yaml
vox_nav_map_server_rclcpp_node:
  ros__parameters:
    octomap_filename: "/home/ros2-foxy/slope_map.bt"
    octomap_voxel_size: 0.2
    octomap_publish_frequency: 50
    provide_utm_to_map_transform: true
    publish_octomap_as_pointcloud: true
    octomap_publish_topic_name: "octomap" # octomap_msgs::msg::Octomap type of message topic name 
    octomap_point_cloud_publish_topic: "octomap_pointcloud" # sensor_msgs::msg::PoinCloud2 that represents octomap
    map_frame_id: "map"
    static_map_frame_id: "static_map"
    utm_frame_id: "utm"
    yaw_offset: 1.57 #see navsat_transform_node from robot_localization, this offset is needed to recorrect orientation of static map
    map_coordinates:
      latitude: 49.89996853007036
      longitude: 8.900083512416545
      altitude: 0.6342219080870174
      quaternion:
        x: -0.0002674301579378746
        y: -7.80412511940006e-05
        z: 0.7068413617683594
        w: 0.7073720461568473
```