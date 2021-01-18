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

#ifndef BOTANBOT_MAP_SERVER__BOTANBOT_MAP_SERVER_HPP_
#define BOTANBOT_MAP_SERVER__BOTANBOT_MAP_SERVER_HPP_

#include <vector>
#include <string>
#include <memory>

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
class BotanbotOctomapServer : public rclcpp::Node
{

public:
  /**
   * @brief Construct a new Botanbot Octomap Server object
   *
   */
  BotanbotOctomapServer();

  /**
   * @brief Destroy the Botanbot Octomap Server object
   *
   */
  ~BotanbotOctomapServer();

  /**
  * @brief periodically called function to publish data
  *
  */
  void publishMapWithTimer();

  /**
   * @brief
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
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_publisher_;
  octomap_msgs::msg::Octomap::SharedPtr octomap_ros_msg_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr octomap_pointloud_publisher_;
  sensor_msgs::msg::PointCloud2::SharedPtr octomap_pointcloud_ros_msg_;
  std::shared_ptr<octomap::OcTree> octomap_octree_;

  botanbot_msgs::msg::OrientedNavSatFix::SharedPtr oriented_navsat_fix_ros_msg_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  botanbot_utilities::RigidBodyTransformation pointloud_transform_matrix_;

  std::string octomap_filename_;
  std::string octomap_publish_topic_name_;
  double octomap_voxel_size_;
  int octomap_publish_frequency_;
  bool provide_utm_to_map_transform_;
  bool publish_octomap_as_pointcloud_;
  std::string octomap_point_cloud_publish_topic_;
  std::string octomap_frame_id_;

};
}  // namespace botanbot_map_server

#endif  // BOTANBOT_MAP_SERVER__BOTANBOT_MAP_SERVER_HPP_
