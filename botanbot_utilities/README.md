# Utilities provoided

## Converting pcd files to octomap

## Collectin GPS waypoints

## Executing a benchmarking for various path planners

## Generating a octomap from a gazebo world

 Calll the service with following to request a conversion from gazebo world to octomap
```bash
 ros2 service call /world/build_octomap botanbot_msgs/srv/GetOctomap "{bounding_box_origin: {x: 0, y: 0, z: 10}, bounding_box_lengths: {x: 120, y: 60, z: 20}, leaf_size: 0.2, filename: /home/ros2-foxy/output_filename.bt}"
 ```
 you also need to make sure the world plugin is placed under <world> tags of gazebo.
```xml
<plugin name="gazebo_world_to_octomap" filename="libgazebo_world_to_octomap.so">
    <octomapPubTopic>world/octomap</octomapPubTopic>
    <octomapServiceName>world/build_octomap</octomapServiceName>
</plugin>
```