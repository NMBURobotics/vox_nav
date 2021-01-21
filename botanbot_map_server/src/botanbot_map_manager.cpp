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
: Node("botanbot_map_manager_rclcpp_node")
{
  RCLCPP_INFO(
    this->get_logger(),
    "Creating..");
  octomap_ros_msg_ = std::make_shared<octomap_msgs::msg::Octomap>();
  octomap_pointcloud_ros_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
  static_map_gps_pose_ = std::make_shared<botanbot_msgs::msg::OrientedNavSatFix>();
  from_ll_to_map_client_node_ = std::make_shared<rclcpp::Node>("fromll_client_rclcpp_node");

  // Declare this node's parameters
  declare_parameter("octomap_filename", "/home/ros2-foxy/f.bt");
  declare_parameter("octomap_publish_topic_name", "octomap");
  declare_parameter("octomap_voxel_size", 0.2);
  declare_parameter("octomap_publish_frequency", 10);
  declare_parameter("publish_octomap_as_pointcloud", true);
  declare_parameter("octomap_point_cloud_publish_topic", "octomap_pointcloud");
  declare_parameter("map_frame_id", "map");
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
  get_parameter("publish_octomap_as_pointcloud", publish_octomap_as_pointcloud_);
  get_parameter("octomap_point_cloud_publish_topic", octomap_point_cloud_publish_topic_);
  get_parameter("map_frame_id", map_frame_id_);
  get_parameter("map_coordinates.latitude", static_map_gps_pose_->position.latitude);
  get_parameter("map_coordinates.longitude", static_map_gps_pose_->position.longitude);
  get_parameter("map_coordinates.altitude", static_map_gps_pose_->position.altitude);
  get_parameter("map_coordinates.quaternion.x", static_map_gps_pose_->orientation.x);
  get_parameter("map_coordinates.quaternion.y", static_map_gps_pose_->orientation.y);
  get_parameter("map_coordinates.quaternion.z", static_map_gps_pose_->orientation.z);
  get_parameter("map_coordinates.quaternion.w", static_map_gps_pose_->orientation.w);

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
      "Exception while converting binary octomap  %s:", e.what());
  }
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000 / octomap_publish_frequency_)),
    std::bind(&BotanbotMapManager::timerCallback, this));

  from_ll_to_map_client_ =
    from_ll_to_map_client_node_->create_client<robot_localization::srv::FromLL>("/fromLL");

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
  while (!from_ll_to_map_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "/fromll service not available, waiting again...");
  }

  auto request = std::make_shared<robot_localization::srv::FromLL::Request>();
  auto response = std::make_shared<robot_localization::srv::FromLL::Response>();
  request->ll_point.latitude = static_map_gps_pose_->position.latitude;
  request->ll_point.longitude = static_map_gps_pose_->position.longitude;
  request->ll_point.altitude = static_map_gps_pose_->position.altitude;

  auto result_future = from_ll_to_map_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(
      from_ll_to_map_client_node_,
      result_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "service call failed :(");
  }

  auto result = result_future.get();
  geometry_msgs::msg::PoseStamped static_map_origin_in_map_frame;
  static_map_origin_in_map_frame.header.frame_id = map_frame_id_;
  static_map_origin_in_map_frame.header.stamp = this->now();
  static_map_origin_in_map_frame.pose.position.x = result->map_point.x;
  static_map_origin_in_map_frame.pose.position.y = result->map_point.y;
  static_map_origin_in_map_frame.pose.position.z = result->map_point.z;

  tf2::Transform static_map_translation;
  static_map_translation.setOrigin(
    tf2::Vector3(
      static_map_origin_in_map_frame.pose.position.x,
      static_map_origin_in_map_frame.pose.position.y,
      static_map_origin_in_map_frame.pose.position.z));
  static_map_translation.setRotation(tf2::Quaternion::getIdentity());

  tf2::Transform static_map_rotation;
  tf2::Quaternion static_map_quaternion;
  tf2::fromMsg(static_map_gps_pose_->orientation, static_map_quaternion);
  static_map_rotation.setOrigin(tf2::Vector3(0, 0, 0));
  static_map_rotation.setRotation(static_map_quaternion);

  pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_octomap_cloud =
    pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

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
      aligned_octomap_cloud->points.push_back(point);
    }
  }

  pcl_ros::transformPointCloud(
    *aligned_octomap_cloud, *aligned_octomap_cloud,
    static_map_rotation * static_map_translation.inverse());

  pcl::toROSMsg(*aligned_octomap_cloud, *octomap_pointcloud_ros_msg_);
  octomap_pointcloud_ros_msg_->header.frame_id = "map";
  octomap_pointcloud_ros_msg_->header.stamp = this->now();
  octomap_pointloud_publisher_->publish(*octomap_pointcloud_ros_msg_);
}

void BotanbotMapManager::publishOctomap(
  rclcpp::Time stamp, std::string map_frame_id,
  geometry_msgs::msg::TransformStamped static_map_to_map_trans)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_octomap_cloud =
    pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  std::shared_ptr<octomap::OcTree> aligned_octomap =
    std::make_shared<octomap::OcTree>(octomap_voxel_size_);
  octomap::Pointcloud temp_octocloud;

  for (auto it = octomap_octree_->begin(0), end = octomap_octree_->end(); it != end; ++it) {
    if (octomap_octree_->isNodeOccupied(*it)) {
      double half_size = it.getSize() / 2.0;
      double size = it.getSize();
      double x = it.getX() - static_map_to_map_trans.transform.translation.x;
      double y = it.getY() - static_map_to_map_trans.transform.translation.y;
      double z = it.getZ() - static_map_to_map_trans.transform.translation.z;
      // insert into pointcloud:
      pcl::PointXYZ point = pcl::PointXYZ();
      point.x = x;
      point.y = y;
      point.z = z;
      aligned_octomap_cloud->points.push_back(point);
      octomap::point3d octocloud_point(x, y, z);
      temp_octocloud.push_back(octocloud_point);
    }
  }
  pcl::toROSMsg(*aligned_octomap_cloud, *octomap_pointcloud_ros_msg_);
  octomap_pointcloud_ros_msg_->header.frame_id = map_frame_id;
  octomap_pointcloud_ros_msg_->header.stamp = stamp;
  octomap::point3d sensorOrigin(0, 0, 0);
  aligned_octomap->insertPointCloud(temp_octocloud, sensorOrigin);
  try {
    octomap_msgs::binaryMapToMsg<octomap::OcTree>(*aligned_octomap, *octomap_ros_msg_);
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
    RCLCPP_ERROR(
      get_logger(),
      "Exception while converting binary octomap  %s:", e.what());
  }
  octomap_ros_msg_->header.stamp = stamp;
  octomap_ros_msg_->header.frame_id = map_frame_id;
  octomap_pointloud_publisher_->publish(*octomap_pointcloud_ros_msg_);
  octomap_publisher_->publish(*octomap_ros_msg_);
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
