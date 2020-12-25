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
 * Parts of code has been taken from
 *      Edo Jelavic
 *      Institute: ETH Zurich, Robotic Systems Lab
 */

#include <memory>
#include <string>
#include "botanbot_utilities/pcl_helpers.hpp"

namespace botanbot_utilities
{

Eigen::Vector3d calculateMeanOfPointPositions(pcl::PointCloud<pcl::PointXYZ>::ConstPtr inputCloud)
{
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (const auto & point : inputCloud->points) {
    mean += Eigen::Vector3d(point.x, point.y, point.z);
  }
  mean /= inputCloud->points.size();

  return mean;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr transformCloud(
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr inputCloud,
  const Eigen::Affine3f & transformMatrix)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud(*inputCloud, *transformedCloud, transformMatrix);
  return transformedCloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr loadPointcloudFromPcd(const std::string & filename)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PCLPointCloud2 cloudBlob;
  pcl::io::loadPCDFile(filename, cloudBlob);
  pcl::fromPCLPointCloud2(cloudBlob, *cloud);
  return cloud;
}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> extractClusterCloudsFromPointcloud(
  pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud)
{
  // Create a kd tree to cluster the input point cloud
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(inputCloud);
  std::vector<pcl::PointIndices> clusterIndices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclideanClusterExtraction;
  euclideanClusterExtraction.setClusterTolerance(0.1);
  euclideanClusterExtraction.setMinClusterSize(1);
  euclideanClusterExtraction.setMaxClusterSize(5000);
  euclideanClusterExtraction.setSearchMethod(tree);
  euclideanClusterExtraction.setInputCloud(inputCloud);
  euclideanClusterExtraction.extract(clusterIndices);

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusterClouds;
  clusterClouds.reserve(clusterIndices.size());

  for (const auto & indicesSet : clusterIndices) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>());
    clusterCloud->points.reserve(indicesSet.indices.size());
    for (auto index : indicesSet.indices) {
      clusterCloud->points.push_back(inputCloud->points[index]);
    }
    clusterCloud->is_dense = true;
    clusterClouds.push_back(clusterCloud);
  }

  return clusterClouds;
}

Eigen::Matrix3f getRotationMatrix(
  double angle, XYZ axis,
  const rclcpp::Logger & node_logger)
{
  Eigen::Matrix3f rotationMatrix = Eigen::Matrix3f::Identity();
  switch (axis) {
    case XYZ::X: {
        rotationMatrix = Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitX());
        break;
      }
    case XYZ::Y: {
        rotationMatrix = Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitY());
        break;
      }
    case XYZ::Z: {
        rotationMatrix = Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitZ());
        break;
      }
    default:
      RCLCPP_ERROR(node_logger, "Unknown axis while trying to rotate the pointcloud");
  }
  return rotationMatrix;
}

Eigen::Affine3f getRigidBodyTransform(
  const Eigen::Vector3d & translation,
  const Eigen::Vector3d & intrinsicRpy,
  const rclcpp::Logger & node_logger)
{
  Eigen::Affine3f rigidBodyTransform;
  rigidBodyTransform.setIdentity();
  rigidBodyTransform.translation() << translation.x(), translation.y(), translation.z();
  Eigen::Matrix3f rotation(Eigen::Matrix3f::Identity());
  rotation *= getRotationMatrix(intrinsicRpy.x(), XYZ::X, node_logger);
  rotation *= getRotationMatrix(intrinsicRpy.y(), XYZ::Y, node_logger);
  rotation *= getRotationMatrix(intrinsicRpy.z(), XYZ::Z, node_logger);
  rigidBodyTransform.rotate(rotation);

  return rigidBodyTransform;
}
}  // namespace botanbot_utilities
