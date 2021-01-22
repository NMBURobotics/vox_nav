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
  robot_localization_fromLL_client_node_ = std::make_shared<rclcpp::Node>(
    "fromll_client_rclcpp_node");

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

  robot_localization_fromLL_client_ =
    robot_localization_fromLL_client_node_->create_client<robot_localization::srv::FromLL>("/fromLL");

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
  std::call_once(
    align_static_map_once_, [this]()
    {
      auto request = std::make_shared<robot_localization::srv::FromLL::Request>();
      auto response = std::make_shared<robot_localization::srv::FromLL::Response>();
      request->ll_point.latitude = static_map_gps_pose_->position.latitude;
      request->ll_point.longitude = static_map_gps_pose_->position.longitude;
      request->ll_point.altitude = static_map_gps_pose_->position.altitude;
      fromGPSPoseToMapPose(request, response);
      // The translation from static_map origin to map is basically inverse of this transform
      tf2::Transform static_map_translation;
      static_map_translation.setOrigin(
        tf2::Vector3(
          response->map_point.x,
          response->map_point.y,
          response->map_point.z));
      // this is identity because map and utm frames are rotationally aligned
      static_map_translation.setRotation(tf2::Quaternion::getIdentity());

      tf2::Transform static_map_rotation;
      tf2::Quaternion static_map_quaternion;
      tf2::fromMsg(static_map_gps_pose_->orientation, static_map_quaternion);
      // First align the static map origin to map in translation, and then rotate the static map with its correct rotation
      static_map_rotation.setOrigin(tf2::Vector3(0, 0, 0));
      static_map_rotation.setRotation(static_map_quaternion);
      tf2::Transform static_map_to_map_transfrom = static_map_rotation *
      static_map_translation.inverse();
      alignStaticMapToMap(static_map_to_map_transfrom);
    });
  publishAlignedMap();
}

void BotanbotMapManager::publishAlignedMap()
{
  octomap_ros_msg_->header.stamp = this->now();
  octomap_ros_msg_->header.frame_id = map_frame_id_;
  octomap_publisher_->publish(*octomap_ros_msg_);

  octomap_pointcloud_ros_msg_->header.frame_id = map_frame_id_;
  octomap_pointcloud_ros_msg_->header.stamp = this->now();
  octomap_pointloud_publisher_->publish(*octomap_pointcloud_ros_msg_);
}

void BotanbotMapManager::fromGPSPoseToMapPose(
  const robot_localization::srv::FromLL::Request::SharedPtr request,
  robot_localization::srv::FromLL::Response::SharedPtr response)
{
  while (!robot_localization_fromLL_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Interrupted while waiting for the /fromLL service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "/fromLL service not available, waiting again...");
  }

  auto result_future = robot_localization_fromLL_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(
      robot_localization_fromLL_client_node_,
      result_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "/fromLL service call failed :(");
  }

  auto result = result_future.get();
  response->map_point = result->map_point;
}

void BotanbotMapManager::alignStaticMapToMap(const tf2::Transform & static_map_to_map_transfrom)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_octomap_cloud =
    pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  for (auto it = octomap_octree_->begin(0), end = octomap_octree_->end(); it != end; ++it) {
    if (octomap_octree_->isNodeOccupied(*it)) {
      // insert into pointcloud:
      pcl::PointXYZ point = pcl::PointXYZ();
      point.x = it.getX();
      point.y = it.getY();
      point.z = it.getZ();
      aligned_octomap_cloud->points.push_back(point);
    }
  }

  pcl_ros::transformPointCloud(
    *aligned_octomap_cloud, *aligned_octomap_cloud, static_map_to_map_transfrom
  );
  pcl::toROSMsg(*aligned_octomap_cloud, *octomap_pointcloud_ros_msg_);


  std::shared_ptr<octomap::OcTree> aligned_octomap =
    std::make_shared<octomap::OcTree>(octomap_voxel_size_);
  octomap::Pointcloud temp_octocloud;
  for (auto && i : aligned_octomap_cloud->points) {
    octomap::point3d octocloud_point(i.x, i.y, i.z);
    temp_octocloud.push_back(octocloud_point);
  }
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
