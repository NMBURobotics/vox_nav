// Copyright (c) 2023 Fetullah Atas, Norwegian University of Life Sciences
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

#include "vox_nav_misc/traversablity_estimator.hpp"

namespace vox_nav_misc
{
  TraversabilityEstimator::TraversabilityEstimator()
  : rclcpp::Node("traversability_estimator_rclcpp_node")
  {
    cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "points",
      rclcpp::SensorDataQoS(),
      std::bind(&TraversabilityEstimator::cloudCallback, this, std::placeholders::_1));

    supervoxel_clusters_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "supervoxel_clusters", rclcpp::SystemDefaultsQoS());

    supervoxel_graph_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "supervoxel_graph", rclcpp::SystemDefaultsQoS());

    this->declare_parameter("supervoxel_disable_transform", false);
    this->declare_parameter("supervoxel_resolution", 0.8);
    this->declare_parameter("supervoxel_seed_resolution", 1.0);
    this->declare_parameter("supervoxel_color_importance", 0.0);
    this->declare_parameter("supervoxel_spatial_importance", 1.0);
    this->declare_parameter("supervoxel_normal_importance", 1.0);

    this->get_parameter("supervoxel_disable_transform", supervoxel_disable_transform_);
    this->get_parameter("supervoxel_resolution", supervoxel_resolution_);
    this->get_parameter("supervoxel_seed_resolution", supervoxel_seed_resolution_);
    this->get_parameter("supervoxel_color_importance", supervoxel_color_importance_);
    this->get_parameter("supervoxel_spatial_importance", supervoxel_spatial_importance_);
    this->get_parameter("supervoxel_normal_importance", supervoxel_normal_importance_);

    RCLCPP_INFO(this->get_logger(), "Traversability Estimator Node is up and running!");
  }

  TraversabilityEstimator::~TraversabilityEstimator()
  {
    RCLCPP_INFO(this->get_logger(), "Traversability Estimator Node is shutting down!");
  }

  void TraversabilityEstimator::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Convert ROS Msg to PCL Point Cloud
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::fromROSMsg(*msg, *cloud);

    cloud =
      vox_nav_utilities::cropBox<pcl::PointXYZRGBA>(
      cloud,
      Eigen::Vector4f(-20.0, -20.0, -5.0, 1.0),
      Eigen::Vector4f(20.0, 20.0, 5.0, 1.0));

    auto super = vox_nav_utilities::supervoxelizeCloud<pcl::PointXYZRGBA>(
      cloud,
      supervoxel_disable_transform_,
      supervoxel_resolution_,
      supervoxel_seed_resolution_,
      supervoxel_color_importance_,
      supervoxel_spatial_importance_,
      supervoxel_normal_importance_);

    super.extract(supervoxel_clusters_);
    RCLCPP_INFO(get_logger(), "Found %d supervoxels", supervoxel_clusters_.size());

    pcl::PointCloud<pcl::PointXYZL>::Ptr supervoxel_cloud = super.getLabeledVoxelCloud();
    sensor_msgs::msg::PointCloud2 supervoxel_cloud_msg;
    pcl::toROSMsg(*supervoxel_cloud, supervoxel_cloud_msg);
    supervoxel_cloud_msg.header = msg->header;
    supervoxel_clusters_publisher_->publish(supervoxel_cloud_msg);

    // Get the supervoxel adjacency
    std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
    super.getSupervoxelAdjacency(supervoxel_adjacency);

    std_msgs::msg::Header header;
    header.frame_id = msg->header.frame_id;
    header.stamp = msg->header.stamp;
    visualization_msgs::msg::MarkerArray marker_array;
    // Publish empty to reset previous
    fillSuperVoxelMarkersfromAdjacency(
      supervoxel_clusters_, supervoxel_adjacency, header, marker_array);
    supervoxel_graph_publisher_->publish(marker_array);

  }

} // namespace vox_nav_misc

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vox_nav_misc::TraversabilityEstimator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
