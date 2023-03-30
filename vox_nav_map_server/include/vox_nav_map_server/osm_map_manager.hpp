// Copyright (c) 2023 Norwegian University of Life Sciences Fetullah Atas
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef VOX_NAV_MAP_SERVER__OSM_MAP_MANAGER_HPP_
#define VOX_NAV_MAP_SERVER__OSM_MAP_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/client.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <rviz_default_plugins/displays/pointcloud/point_cloud_helpers.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <robot_localization/srv/from_ll.hpp>
#include <vox_nav_msgs/msg/oriented_nav_sat_fix.hpp>
#include <vox_nav_msgs/srv/get_maps_and_surfels.hpp>
#include <vox_nav_utilities/pcl_helpers.hpp>
#include <vox_nav_utilities/tf_helpers.hpp>
#include <vox_nav_utilities/map_manager_helpers.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <octomap/octomap_utils.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/common/io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include <boost/progress.hpp>

#include <mutex>
#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <iostream>
#include <sstream>

/**
 * @brief namespace for vox_nav map server. The map server reads map from disk.
 *        The map needs to cd native format .pcd
 *        When you created the map the datum of the map must have been saved.
 *        Datum is GPS coordinates and orination info from absolute headed IMU.
 *        Using this origin this map is georefenced and published as octomap as well as pointcloud that represents this map
 */
namespace vox_nav_map_server
{

  struct OSM_SEMANTIC_ROAD_LABELS
  {
    std::string MAJOR_ROADS_WHITE = "255255255"; // WHITE
    std::string MISC_ORANGE = "255127000";       // ORANGE
    std::string MINOR_ROAD_MAROON = "127000000"; // MAROON
    std::string FOOTWAY_OLIVE = "127127000";     // OLIVE
    std::string TRACK_NAVY = "000000127";        // NAVY
  };

/**
 * @brief
 *
 */
  class OSMMapManager : public rclcpp::Node
  {
  public:
    /**
     * @brief Construct a new Map Manager object
     *
     */
    OSMMapManager();

    /**
     * @brief Destroy the Map Manager object
     *
     */
    ~OSMMapManager();

    /**
    * @brief periodically called function to publish map visuals
    *
    */
    void timerCallback();

    /**
     * @brief Given PCD Map's GPS coordinate and heading,
     * this method aligns PCD Map to robots initial coordinates,
     * thats basically "map" frame published by robot_localization.
     * One must think PCD Map as a static map.
     *
     */
    void transfromPCDfromGPS2Map();


    /**
     * @brief It is possible to apply some preprocessing steps to original PCD map.
     * Noise removal , downsampling , rigid body transfroms etc.
     * Look at the params.yaml for filter related paramaters
     *
     */
    void preProcessPCDMap();

    /**
     * @brief Get the Get Maps And Surfels Callback object, Service callback to
     * provide maps and surfels managed and cnfigured by this node
     *
     * @param request_header
     * @param request
     * @param response
     */
    void getGetOSMMapCallback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<vox_nav_msgs::srv::GetMapsAndSurfels::Request> request,
      std::shared_ptr<vox_nav_msgs::srv::GetMapsAndSurfels::Response> response);

    /**
     * @brief The elevation given in the MapRoads is not accurate. Use the road collider to get the accurate elevation
     *
     */
    void fixMapRoadsElevation();

    /**
     * @brief Extract semantic labels from the map roads, We have sampled points ont the mesh based on the texture, so some points belogining to same
     * road may have different labels (colors). We will use cloud transformer to transfer the labels to the points.
     *
     */
    void extractSemanticLabels();

    /**
     * @brief Publish the map roads as pointclouds for visualization
     *
     */
    void publishOSMPointClouds();

    /**
     * @brief Get the Rainbow Color object
     *
     * @param value
     * @param color
     */
    inline void getRainbowColor(float value, pcl::PointXYZRGB & color)
    {
      // this is HSV color palette with hue values going only from 0.0 to 0.833333.
      value = std::min(value, 1.0f);
      value = std::max(value, 0.0f);
      float h = value * 5.0f + 1.0f;
      int i = floor(h);
      float f = h - i;
      if (!(i & 1) ) {
        f = 1 - f;         // if i is even
      }
      float n = 1 - f;
      if (i <= 1) {
        color.r = n, color.g = 0, color.b = 1;
      } else if (i == 2) {
        color.r = 0, color.g = n, color.b = 1;
      } else if (i == 3) {
        color.r = 0, color.g = 1, color.b = n;
      } else if (i == 4) {
        color.r = n, color.g = 1, color.b = 0;
      } else if (i >= 5) {
        color.r = h, color.g = h, color.b = 0;
      }
      color.r *= 255;
      color.g *= 255;
      color.b *= 255;
    }

  protected:
    // Used to call a periodic callback function IOT publish octomap visuals
    rclcpp::TimerBase::SharedPtr timer_;
    // Provide a service to get maps managed by this node
    rclcpp::Service<vox_nav_msgs::srv::GetMapsAndSurfels>::SharedPtr get_maps_service_;
    // publishes pointclouds for visualization
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr osm_road_topologies_pointcloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr osm_road_colliders_pointcloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr osm_buildings_pointcloud_pub_;

    std::unordered_map<std::string, int> semantic_labels_map_;

    // robot_localization package provides a service to convert
    // lat,long,alt GPS cooordinates to x,y,z map points
    rclcpp::Client<robot_localization::srv::FromLL>::SharedPtr robot_localization_fromLL_client_;
    // clint node used for spinning the service callback of robot_localization_fromLL_client_
    rclcpp::Node::SharedPtr robot_localization_fromLL_client_node_;

    // GPS coordinates of the map origin
    vox_nav_msgs::msg::OrientedNavSatFix::SharedPtr map_gps_pose_;

    // pcd file names for map roads, colliders and buildings
    std::string osm_road_topologies_pcd_filename_;
    std::string osm_road_colliders_pcd_filename_;
    std::string osm_buildings_pcd_filename_;

    // PCL Pointcloud for map roads, colliders and buildings
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr osm_road_topologies_pointcloud_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr osm_road_colliders_pointcloud_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr osm_buildings_pointcloud_;

    // rclcpp parameters from yaml file: frame id for map typicall: "map"
    std::string map_frame_id_;
    std::string utm_frame_id_;
    // rclcpp parameters from yaml file: publish frequncy to publish map and transfroms
    int publish_frequency_;
    // rclcpp parameters from yaml file: if true, a cloud will be published which represents octomap
    std::once_flag align_static_map_once_;
    // tf buffer to get access to transfroms
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_transform_broadcaster_;
    // Optional rigid body transform to apply to the cloud, if cloud
    // is depth camera frames we need to pull cloud back to conventional ROS frames
    vox_nav_utilities::RigidBodyTransformation pcd_map_transform_matrix_;

    volatile bool map_configured_;
    // rclcpp parameters from yaml file: if greater than 0, a voxel grid filter will be applied to the cloud
    double downsample_leaf_size_;
  };
}  // namespace vox_nav_map_server

#endif  // VOX_NAV_MAP_SERVER__OSM_MAP_MANAGER_HPP_
