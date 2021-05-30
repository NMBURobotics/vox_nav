// Copyright (c) 2021 Norwegian University of Life Sciences Fetullah Atas
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

#ifndef VOX_NAV_MAP_SERVER__MAP_MANAGER_HPP_
#define VOX_NAV_MAP_SERVER__MAP_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/client.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <robot_localization/srv/from_ll.hpp>
#include <vox_nav_map_server/cost_regression_utils.hpp>
#include <vox_nav_msgs/msg/oriented_nav_sat_fix.hpp>
#include <vox_nav_utilities/pcl_helpers.hpp>

#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <octomap/octomap_utils.h>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>

#include <vector>
#include <string>
#include <memory>
#include <mutex>

/**
 * @brief namespace for vox_nav map server. The map server reads map from disk.
 *        The map needs to octomap native format .bt
 *        When you created the map the origin of the map must have been saved.
 *        Using this origin this map is georefenced and published as octomap as well as pointcloud that represents this map
 *
 */
namespace vox_nav_map_server
{
/**
 * @brief
 *
 */
class vox_navMapManager : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new vox_nav Map Manager object
   *
   */
  vox_navMapManager();

  /**
   * @brief Destroy the vox_nav Map Manager object
   *
   */
  ~vox_navMapManager();

  /**
  * @brief periodically called function to publish octomap and its pointcloud data
  *
  */
  void timerCallback();

  /**
   * @brief once map is georefnced, this function
   *  is called from timerCallback to publish map
   *
   */
  void publishAlignedMap();

  /**
   * @brief publishes octomap as visualization msgs
   *
   * @param tree
   */
  void fillOctomapMarkers(const octomap::ColorOcTree & tree);

  /**
   * @brief given GPS lat long alt coordinates uses
   *        robot_localization service to georefence this map
   *
   * @param request
   * @param response
   */
  void fromGPSPoseToMapPose(
    const robot_localization::srv::FromLL::Request::SharedPtr request,
    robot_localization::srv::FromLL::Response::SharedPtr response);

  /**
   * @brief
   *
   * @param static_map_to_map_transfrom
   */
  void alignStaticMapToMap(const tf2::Transform & static_map_to_map_transfrom);

protected:
  // Used to creted a periodic callback function IOT publish transfrom/octomap/cloud etc.
  rclcpp::TimerBase::SharedPtr timer_;
  // publishes octomap in its native format
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_publisher_;
  // publishes octomap in form of a point cloud message
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr octomap_pointloud_publisher_;
  // robot_localization package provides a service to convert
  // lat,long,al GPS cooordinates to x,y,z map points
  rclcpp::Client<robot_localization::srv::FromLL>::SharedPtr robot_localization_fromLL_client_;
  // clint node used for spinning the service callback of robot_localization_fromLL_client_
  rclcpp::Node::SharedPtr robot_localization_fromLL_client_node_;
  // reusable octomap point loud message, dont need to recreate each time we publish
  sensor_msgs::msg::PointCloud2::SharedPtr octomap_pointcloud_ros_msg_;
  // reusable octomp message, dont need to recreate each time we publish
  octomap_msgs::msg::Octomap::SharedPtr octomap_ros_msg_;
  // we read gps coordinates of map from yaml
  vox_nav_msgs::msg::OrientedNavSatFix::SharedPtr static_map_gps_pose_;
  // otree object to read and store binary octomap from disk
  std::shared_ptr<octomap::ColorOcTree> octomap_octree_;
  // rclcpp parameters from yaml file: full path to octomap file in disk
  std::string pcd_map_filename_;
  // Pointcloud map is stroed here
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcd_map_pointcloud_;
  // rclcpp parameters from yaml file: topic name for published octomap
  std::string octomap_publish_topic_name_;
  // rclcpp parameters from yaml file: topic name for published octomap as cloud
  std::string octomap_point_cloud_publish_topic_;
  // rclcpp parameters from yaml file: frame id for map typicall: "map"
  std::string map_frame_id_;
  std::string utm_frame_id_;
  // rclcpp parameters from yaml file: vxel size for octomap
  double octomap_voxel_size_;
  // see navsat_transform_node from robot_localization
  // this offset is needed to recorrect orientation of static map
  double yaw_offset_;
  // rclcpp parameters from yaml file: publish frequncy to publish map and transfroms
  int octomap_publish_frequency_;
  // rclcpp parameters from yaml file: if true, a cloud will be published which represents octomap
  bool publish_octomap_as_pointcloud_;
  bool publish_octomap_markers_;
  // we need to align static map to map only once, since it is static !
  std::once_flag align_static_map_once_;
  // tf buffer to get access to transfroms
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Optional rigid body transform to apply to the cloud, if cloud
  // is depth camera frames we need to pull cloud back to conventional ROS frames
  vox_nav_utilities::RigidBodyTransformation pcd_map_transform_matrix_;
  // optional point cloud transformfrom yaml file
  double pcd_map_downsample_voxel_size_;
  int remove_outlier_mean_K_;
  double remove_outlier_stddev_threshold_;
  double remove_outlier_radius_search_;
  int remove_outlier_min_neighbors_in_radius_;
  bool apply_filters_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr octomap_markers_publisher_;
  visualization_msgs::msg::MarkerArray octomap_markers_;
};
}  // namespace vox_nav_map_server

#endif  // VOX_NAV_MAP_SERVER__MAP_MANAGER_HPP_
