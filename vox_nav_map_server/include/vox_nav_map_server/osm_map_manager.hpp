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

#include <vector>
#include <string>
#include <memory>
#include <mutex>

/**
 * @brief namespace for vox_nav map server. The map server reads map from disk.
 *        The map needs to cd native format .pcd
 *        When you created the map the datum of the map must have been saved.
 *        Datum is GPS coordinates and orination info from absolute headed IMU.
 *        Using this origin this map is georefenced and published as octomap as well as pointcloud that represents this map
 */
namespace vox_nav_map_server
{

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
     * @brief The elevation given in the MapRoads is not accurate.
     *
     */
    void fixMapRoadsElevation();

  protected:
    // Used to call a periodic callback function IOT publish octomap visuals
    rclcpp::TimerBase::SharedPtr timer_;
    // Service to provide Octomap, elevated surfel and elevated surfel poses
    rclcpp::Service<vox_nav_msgs::srv::GetMapsAndSurfels>::SharedPtr get_maps_service_;
    // publishes octomap in form of a point cloud message
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr osm_pointloud_publisher_;

    // robot_localization package provides a service to convert
    // lat,long,al GPS cooordinates to x,y,z map points
    rclcpp::Client<robot_localization::srv::FromLL>::SharedPtr robot_localization_fromLL_client_;
    // clint node used for spinning the service callback of robot_localization_fromLL_client_
    rclcpp::Node::SharedPtr robot_localization_fromLL_client_node_;
    // reusable octomap point loud message, dont need to recreate each time we publish

    vox_nav_msgs::msg::OrientedNavSatFix::SharedPtr map_gps_pose_;

    std::string osm_map_roads_pcd_filename_;
    std::string osm_map_buildings_pcd_filename_;

    // Pointcloud map is stroed here
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr osm_map_roads_pointcloud_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr osm_map_buildings_pointcloud_;

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
    //  see the struct, it is used to keep preprocess params orginzed
    // hther map has beene configured yet

    volatile bool map_configured_;

    double downsample_leaf_size_;
  };
}  // namespace vox_nav_map_server

#endif  // VOX_NAV_MAP_SERVER__OSM_MAP_MANAGER_HPP_
