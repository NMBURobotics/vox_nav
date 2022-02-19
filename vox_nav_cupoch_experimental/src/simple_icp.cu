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

/*
DISCLAIMER: some parts of code has been taken from; https://github.com/appinho/SARosPerceptionKitti
Credits to author: Simon Appel, https://github.com/appinho
*/

#include "vox_nav_cupoch_experimental/simple_icp.hpp"

#include <string>
#include <vector>
#include <memory>
#include <algorithm>

using namespace vox_nav_cupoch_experimental;

SimpleICP::SimpleICP()
    : Node("cloud_clustering_rclcpp_node")
{

  cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/ouster/points",
      rclcpp::SensorDataQoS(),
      std::bind(
          &SimpleICP::cloudCallback, this, std::placeholders::_1));

  map_cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/map/points",
      rclcpp::SensorDataQoS(),
      std::bind(
          &SimpleICP::mapCloudCallback, this, std::placeholders::_1));

  /*cloud_clusters_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/vox_nav/detection/clusters", rclcpp::SystemDefaultsQoS());*/

  RCLCPP_INFO(get_logger(), "Creating...");
}

SimpleICP::~SimpleICP()
{
  RCLCPP_INFO(get_logger(), "Destroying...");
}

void SimpleICP::cloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
{
  if (map_configured_)
  {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_curr(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*cloud, *pcl_curr);

    pcl_curr = vox_nav_utilities::cropBox<pcl::PointXYZRGB>(
        pcl_curr,
        Eigen::Vector4f(-20, -20, -5, 1),
        Eigen::Vector4f(20, 20, 5, 1));
  }
}

void SimpleICP::mapCloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
{
  std::call_once(
      get_map_cloud_once_, [&]()
      {
        pcl::fromROSMsg(*cloud, map_);
        map_configured_ = true; });
}

int main(int argc, char const *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimpleICP>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
