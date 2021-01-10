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

#include "botanbot_utilities/pcl2octomap_converter.hpp"

#include <memory>

namespace botanbot_utilities
{
PCL2OctomapConverter::PCL2OctomapConverter(/* args */)
: Node("pcl2octomap_converter_rclcpp_node")
{
  this->declare_parameter("input_pcd_filename", "/home/ros2-foxy/f.pcd");
  this->declare_parameter("output_binary_octomap_filename", "/home/ros2-foxy/f.bt");
  this->declare_parameter("cloud_transform.translation.x", 0.0);
  this->declare_parameter("cloud_transform.translation.y", 0.0);
  this->declare_parameter("cloud_transform.translation.z", 0.0);
  this->declare_parameter("cloud_transform.rotation.r", 0.0);
  this->declare_parameter("cloud_transform.rotation.p", 0.0);
  this->declare_parameter("cloud_transform.rotation.y", 0.0);
  this->declare_parameter("downsample_voxel_size", 0.05);
  this->declare_parameter("remove_outlier_mean_K", 10);
  this->declare_parameter("remove_outlier_stddev_threshold", 1.0);

  input_pcd_filename_ = this->get_parameter("input_pcd_filename").as_string();
  output_binary_octomap_filename_ =
    this->get_parameter("output_binary_octomap_filename").as_string();

  pointloud_transform_matrix_.translation_.x() =
    this->get_parameter("cloud_transform.translation.x").as_double();
  pointloud_transform_matrix_.translation_.y() =
    this->get_parameter("cloud_transform.translation.y").as_double();
  pointloud_transform_matrix_.translation_.z() =
    this->get_parameter("cloud_transform.translation.z").as_double();
  pointloud_transform_matrix_.rpyIntrinsic_.x() =
    this->get_parameter("cloud_transform.rotation.r").as_double();
  pointloud_transform_matrix_.rpyIntrinsic_.y() =
    this->get_parameter("cloud_transform.rotation.p").as_double();
  pointloud_transform_matrix_.rpyIntrinsic_.z() =
    this->get_parameter("cloud_transform.rotation.y").as_double();

  downsample_voxel_size_ = this->get_parameter("downsample_voxel_size").as_double();
  remove_outlier_mean_K_ = this->get_parameter("remove_outlier_mean_K").as_int();
  remove_outlier_stddev_threshold_ =
    this->get_parameter("remove_outlier_stddev_threshold").as_double();

  pointcloud_ = botanbot_utilities::loadPointcloudFromPcd(input_pcd_filename_.c_str());
  pointcloud_ = botanbot_utilities::downsampleInputCloud(pointcloud_, downsample_voxel_size_);
  pointcloud_ = botanbot_utilities::removeOutliersFromInputCloud(
    pointcloud_,
    remove_outlier_mean_K_,
    remove_outlier_stddev_threshold_);

  pointcloud_ = botanbot_utilities::transformCloud(
    pointcloud_,
    botanbot_utilities::getRigidBodyTransform(
      pointloud_transform_matrix_.translation_,
      pointloud_transform_matrix_.rpyIntrinsic_,
      get_logger()));
}

PCL2OctomapConverter::~PCL2OctomapConverter()
{
}

void PCL2OctomapConverter::calcThresholdedNodes(
  const octomap::OcTree tree,
  unsigned int & num_thresholded,
  unsigned int & num_other)
{
  num_thresholded = 0;
  num_other = 0;

  for (octomap::OcTree::tree_iterator it = tree.begin_tree(), end = tree.end_tree(); it != end;
    ++it)
  {
    if (tree.isNodeAtThreshold(*it)) {
      num_thresholded++;
    } else {
      num_other++;
    }
  }
}

void PCL2OctomapConverter::outputStatistics(const octomap::OcTree tree)
{
  unsigned int numThresholded, numOther;
  calcThresholdedNodes(tree, numThresholded, numOther);
  size_t memUsage = tree.memoryUsage();
  int64_t memFullGrid = tree.memoryFullGrid();
  size_t numLeafNodes = tree.getNumLeafNodes();

  std::cout << "Tree size: " << tree.size() << " nodes (" << numLeafNodes << " leafs). " <<
    numThresholded << " nodes thresholded, " << numOther << " other\n";
  std::cout << "Memory: " << memUsage << " byte (" << memUsage / (1024. * 1024.) << " MB)" <<
    std::endl;
  std::cout << "Full grid: " << memFullGrid << " byte (" << memFullGrid / (1024. * 1024.) <<
    " MB)" <<
    std::endl;
  double x, y, z;
  tree.getMetricSize(x, y, z);
  std::cout << "Size: " << x << " x " << y << " x " << z << " m^3\n";
  std::cout << std::endl;
}

void PCL2OctomapConverter::processConversion()
{
  octomap::Pointcloud octocloud;
  octomap::OcTree tree(0.1);
  for (size_t i = 0; i < pointcloud_->points.size(); i++) {
    octomap::point3d endpoint(pointcloud_->points[i].x, pointcloud_->points[i].y,
      pointcloud_->points[i].z);
    octocloud.push_back(endpoint);
  }
  std::cout << "Octocloud size is:" << octocloud.size() << std::endl;
  octomap::point3d sensorOrigin(0, 0, 0);
  tree.insertPointCloud(octocloud, sensorOrigin);
  outputStatistics(tree);
  tree.writeBinary(output_binary_octomap_filename_);
}

}  // namespace botanbot_utilities

int main(int argc, char const * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<botanbot_utilities::PCL2OctomapConverter>();
  node->processConversion();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
