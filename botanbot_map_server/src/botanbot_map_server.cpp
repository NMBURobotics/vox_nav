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
#include "botanbot_map_server/botanbot_map_server.hpp"

namespace botanbot_map_server
{
BotanbotOctomapServer::BotanbotOctomapServer()
: Node("botanbot_map_server_rclcpp_node")
{
  RCLCPP_INFO(
    this->get_logger(),
    "Creating..");
  // Declare this node's parameters
  declare_parameter("octomap_filename", "/home/ros2-foxy/f.bt");
  declare_parameter("octomap_publish_topic_name", "octomap");
  declare_parameter("octomap_voxel_size", 0.2);
  declare_parameter("octomap_publish_frequency", 10);
  declare_parameter("provide_earth_to_map_transform", true);
  declare_parameter("publish_octomap_as_pointcloud", true);
  declare_parameter("octomap_point_cloud_publish_topic", "octomap_pointcloud");

  // get this node's parameters
  get_parameter("octomap_filename", octomap_filename_);
  get_parameter("octomap_publish_topic_name", octomap_publish_topic_name_);
  get_parameter("octomap_voxel_size", octomap_voxel_size_);
  get_parameter("octomap_publish_frequency", octomap_publish_frequency_);
  get_parameter("provide_earth_to_map_transform", provide_earth_to_map_transform_);
  get_parameter("publish_octomap_as_pointcloud", publish_octomap_as_pointcloud_);
  get_parameter("octomap_point_cloud_publish_topic", octomap_point_cloud_publish_topic_);

  octomap_publisher_ = this->create_publisher<octomap_msgs::msg::Octomap>(
    octomap_publish_topic_name_, rclcpp::SystemDefaultsQoS());
  octomap_ros_msg_ = std::make_shared<octomap_msgs::msg::Octomap>();
  octomap_octree_ = std::make_shared<octomap::OcTree>(octomap_voxel_size_);


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
    std::bind(&BotanbotOctomapServer::publishMapWithTimer, this));
  RCLCPP_INFO(
    this->get_logger(),
    "Created an Instance of BotanbotOctomapServer");
}

BotanbotOctomapServer::~BotanbotOctomapServer()
{
  RCLCPP_INFO(
    this->get_logger(),
    "Destroyed an Instance of BotanbotOctomapServer");
}

void BotanbotOctomapServer::publishMapWithTimer()
{
  RCLCPP_INFO(
    this->get_logger(), "Publishing octomap with frequency of %i hertz",
    octomap_publish_frequency_);
  octomap_ros_msg_->header.stamp = this->now();
  octomap_ros_msg_->header.frame_id = "map";
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
    <botanbot_map_server::BotanbotOctomapServer>();
  rclcpp::spin(gps_waypoint_follower_client_node);
  rclcpp::shutdown();
  return 0;
}
