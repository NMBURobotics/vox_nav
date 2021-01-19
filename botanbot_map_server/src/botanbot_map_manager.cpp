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

#include <string>
#include <vector>
#include <memory>
#include "botanbot_map_server/botanbot_map_manager.hpp"

namespace botanbot_map_server
{
BotanbotMapManager::BotanbotMapManager()
: Node("botanbot_map_manager_rclcpp_node"),
  tf_broadcaster_(*this)
{
  RCLCPP_INFO(
    this->get_logger(),
    "Creating..");

  octomap_ros_msg_ = std::make_shared<octomap_msgs::msg::Octomap>();
  octomap_pointcloud_ros_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
  oriented_navsat_fix_ros_msg_ = std::make_shared<botanbot_msgs::msg::OrientedNavSatFix>();

  // Declare this node's parameters
  declare_parameter("octomap_filename", "/home/ros2-foxy/f.bt");
  declare_parameter("octomap_publish_topic_name", "octomap");
  declare_parameter("octomap_voxel_size", 0.2);
  declare_parameter("octomap_publish_frequency", 10);
  declare_parameter("provide_utm_to_map_transform", true);
  declare_parameter("publish_octomap_as_pointcloud", true);
  declare_parameter("octomap_point_cloud_publish_topic", "octomap_pointcloud");
  declare_parameter("octomap_frame_id", "map");
  declare_parameter("map_coordinates.latitude", 49.0);
  declare_parameter("map_coordinates.longitude", 3.0);
  declare_parameter("map_coordinates.altitude", 0.5);
  declare_parameter("map_coordinates.quaternion.x", 0.0);
  declare_parameter("map_coordinates.quaternion.y", 0.0);
  declare_parameter("map_coordinates.quaternion.z", 0.0);
  declare_parameter("map_coordinates.quaternion.w", 1.0);

  // get this node's parameters
  get_parameter("octomap_filename", octomap_filename_);
  get_parameter("octomap_publish_topic_name", octomap_publish_topic_name_);
  get_parameter("octomap_voxel_size", octomap_voxel_size_);
  get_parameter("octomap_publish_frequency", octomap_publish_frequency_);
  get_parameter("provide_utm_to_map_transform", provide_utm_to_map_transform_);
  get_parameter("publish_octomap_as_pointcloud", publish_octomap_as_pointcloud_);
  get_parameter("octomap_point_cloud_publish_topic", octomap_point_cloud_publish_topic_);
  get_parameter("octomap_frame_id", octomap_frame_id_);
  get_parameter("map_coordinates.latitude", oriented_navsat_fix_ros_msg_->position.latitude);
  get_parameter("map_coordinates.longitude", oriented_navsat_fix_ros_msg_->position.longitude);
  get_parameter("map_coordinates.altitude", oriented_navsat_fix_ros_msg_->position.altitude);
  get_parameter("map_coordinates.quaternion.x", oriented_navsat_fix_ros_msg_->orientation.x);
  get_parameter("map_coordinates.quaternion.y", oriented_navsat_fix_ros_msg_->orientation.y);
  get_parameter("map_coordinates.quaternion.z", oriented_navsat_fix_ros_msg_->orientation.z);
  get_parameter("map_coordinates.quaternion.w", oriented_navsat_fix_ros_msg_->orientation.w);

  octomap_octree_ = std::make_shared<octomap::OcTree>(octomap_voxel_size_);

  octomap_publisher_ = this->create_publisher<octomap_msgs::msg::Octomap>(
    octomap_publish_topic_name_, rclcpp::SystemDefaultsQoS());
  octomap_pointloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    octomap_point_cloud_publish_topic_, rclcpp::SystemDefaultsQoS());

  octomap_octree_->readBinary(octomap_filename_);
  bool is_octomap_successfully_converted(false);
  try {
    is_octomap_successfully_converted = octomap_msgs::binaryMapToMsg<octomap::OcTree>(
      *octomap_octree_, *octomap_ros_msg_);
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
    RCLCPP_ERROR(
      get_logger(),
      "Exception while converting bnary octomap  %s:", e.what());
  }

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000 / octomap_publish_frequency_)),
    std::bind(&BotanbotMapManager::timerCallback, this));
  RCLCPP_INFO(
    this->get_logger(),
    "Created an Instance of BotanbotMapManager");
}

BotanbotMapManager::~BotanbotMapManager()
{
  RCLCPP_INFO(
    this->get_logger(),
    "Destroyed an Instance of BotanbotMapManager");
}

void BotanbotMapManager::timerCallback()
{
  RCLCPP_INFO(
    this->get_logger(), "Publishing octomap with frequency of %i hertz",
    octomap_publish_frequency_);
  octomap_ros_msg_->header.stamp = this->now();
  octomap_ros_msg_->header.frame_id = octomap_frame_id_;
  octomap_publisher_->publish(*octomap_ros_msg_);
  octomap_pointloud_publisher_->publish(*octomap_pointcloud_ros_msg_);
  publishUTMMapTransfrom();
}

void BotanbotMapManager::fillPointCloudfromOctomap(
  const std::shared_ptr<octomap::OcTree> octomap_octree,
  sensor_msgs::msg::PointCloud2::SharedPtr cloud)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud =
    pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  // traverse all leafs in the tree:
  for (auto it = octomap_octree_->begin(0), end = octomap_octree_->end(); it != end; ++it) {
    if (octomap_octree_->isNodeOccupied(*it)) {
      double half_size = it.getSize() / 2.0;
      double size = it.getSize();
      double x = it.getX();
      double y = it.getY();
      double z = it.getZ();
      // insert into pointcloud:
      pcl::PointXYZ point = pcl::PointXYZ();
      point.x = x;
      point.y = y;
      point.z = z;
      pcl_cloud->points.push_back(point);
    }
  }
  pcl::toROSMsg(*pcl_cloud, *cloud);
  cloud->header.frame_id = octomap_frame_id_;
  cloud->header.stamp = this->now();
  RCLCPP_INFO(
    this->get_logger(), "Filed a pointclod from Octomap with %i points", pcl_cloud->points.size());
}

void BotanbotMapManager::publishUTMMapTransfrom()
{
  double altitude = oriented_navsat_fix_ros_msg_->position.altitude;
  double longitude = oriented_navsat_fix_ros_msg_->position.longitude;
  double latitude = oriented_navsat_fix_ros_msg_->position.latitude;
  double utmY, utmX;
  std::string utm_zone_tmp;
  botanbot_utilities::navsat_conversions::LLtoUTM(latitude, longitude, utmY, utmX, utm_zone_tmp);
  tf2::Transform map_origin;
  tf2::Quaternion map_rot;
  map_rot.setX(oriented_navsat_fix_ros_msg_->orientation.x);
  map_rot.setY(oriented_navsat_fix_ros_msg_->orientation.y);
  map_rot.setZ(oriented_navsat_fix_ros_msg_->orientation.z);
  map_rot.setW(oriented_navsat_fix_ros_msg_->orientation.w);

  geometry_msgs::msg::TransformStamped map_utm_transform_stamped;
  map_origin.setOrigin(tf2::Vector3(utmX, utmY, altitude));
  map_origin.setRotation(map_rot);

  map_utm_transform_stamped.header.stamp = this->now();
  map_utm_transform_stamped.header.frame_id = "utm";
  map_utm_transform_stamped.child_frame_id = octomap_frame_id_;
  map_utm_transform_stamped.transform = tf2::toMsg(map_origin);
  tf_broadcaster_.sendTransform(map_utm_transform_stamped);
  fillPointCloudfromOctomap(octomap_octree_, octomap_pointcloud_ros_msg_);
}
}  // namespace botanbot_map_server

/**
 * @brief
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char const * argv[])
{
  rclcpp::init(argc, argv);
  auto gps_waypoint_follower_client_node = std::make_shared
    <botanbot_map_server::BotanbotMapManager>();
  rclcpp::spin(gps_waypoint_follower_client_node);
  rclcpp::shutdown();
  return 0;
}
