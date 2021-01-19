// Copyright (c) 2020 Fetullah Atas, Norwegian University of Life Sciences
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

#ifndef BOTANBOT_MAP_SERVER__BOTANBOT_MAP_MANAGER_HPP_
#define BOTANBOT_MAP_SERVER__BOTANBOT_MAP_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <botanbot_utilities/navsat_conversions.hpp>
#include <botanbot_utilities/pcl_helpers.hpp>
#include <botanbot_utilities/gps_waypoint_collector.hpp>
#include <botanbot_msgs/msg/oriented_nav_sat_fix.hpp>

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

#include <vector>
#include <string>
#include <memory>

/**
 * @brief
 *
 */
namespace botanbot_map_server
{
/**
 * @brief
 *
 */
class BotanbotMapManager : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new Botanbot Map Manager object
   *
   */
  BotanbotMapManager();

  /**
   * @brief Destroy the Botanbot Map Manager object
   *
   */
  ~BotanbotMapManager();

  /**
  * @brief periodically called function to publish octomap and its pointcloud data
  *
  */
  void timerCallback();

  /**
   * @brief fill cloud from octomap_octree
   *
   * @param octomap_octree
   * @param cloud
   */
  void fillPointCloudfromOctomap(
    const std::shared_ptr<octomap::OcTree> octomap_octree,
    sensor_msgs::msg::PointCloud2::SharedPtr cloud);

  /**
   * @brief
   *
   */
  void publishUTMMapTransfrom();

protected:
  // Used to creted a periodic callback function IOT publish transfrom/octomap/cloud etc.
  rclcpp::TimerBase::SharedPtr timer_;
  // publishes octomap in its native format
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_publisher_;
  // reusable octomp message, dont need to recreate each time we publish
  octomap_msgs::msg::Octomap::SharedPtr octomap_ros_msg_;
  // publishes octomap in form of a point cloud message
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr octomap_pointloud_publisher_;
  // reusable octomap point loud message, dont need to recreate each time we publish
  sensor_msgs::msg::PointCloud2::SharedPtr octomap_pointcloud_ros_msg_;
  // otree object to read and store binary octomap from disk
  std::shared_ptr<octomap::OcTree> octomap_octree_;
  // we read gps coordinates of map from yaml
  botanbot_msgs::msg::OrientedNavSatFix::SharedPtr oriented_navsat_fix_ros_msg_;
  // to broadcast utm -> map
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  // rclcpp parameters from yaml file: full path to octomap file in disk
  std::string octomap_filename_;
  // rclcpp parameters from yaml file: topic name for published octomap
  std::string octomap_publish_topic_name_;
  // rclcpp parameters from yaml file: vxel size for octomap
  double octomap_voxel_size_;
  // rclcpp parameters from yaml file: publish frequncy to publish map and transfroms
  int octomap_publish_frequency_;
  // rclcpp parameters from yaml file: if true, utm -> map transftom is avail for TF
  bool provide_utm_to_map_transform_;
  // rclcpp parameters from yaml file: if true, a cloud will be published which represents octomap
  bool publish_octomap_as_pointcloud_;
  // rclcpp parameters from yaml file: topic name for published octomap as cloud
  std::string octomap_point_cloud_publish_topic_;
  // rclcpp parameters from yaml file: frame id for octomap typicall: "map"
  std::string octomap_frame_id_;
  // We will get the GPS coordinates before starting the Mapping.
  std::shared_ptr<botanbot_utilities::GPSWaypointCollector> gps_waypoint_collector_node_;
  // we will recieve gp data once and that is it , weonly need this to precisely
  // define start location of map
  std::once_flag gps_data_recieved_flag_;
};
}  // namespace botanbot_map_server

#endif  // BOTANBOT_MAP_SERVER__BOTANBOT_MAP_MANAGER_HPP_
