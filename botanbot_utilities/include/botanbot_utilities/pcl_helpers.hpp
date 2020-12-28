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

#ifndef BOTANBOT_UTILITIES__PCL_HELPERS_HPP_
#define BOTANBOT_UTILITIES__PCL_HELPERS_HPP_

#include <eigen3/Eigen/Core>

#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace botanbot_utilities
{
/**
* @brief
*
*/
struct RigidBodyTransformation
{
  Eigen::Vector3d translation_ {0.0, 0.0, 0.0};

  // intrinsic rotation (opposite from the ROS convention), order X-Y-Z
  Eigen::Vector3d rpyIntrinsic_ {0.0, 0.0, 0.0};
};
enum class XYZ: int {X, Y, Z};

/**
 * @brief
 *
 * @param inputCloud
 * @return Eigen::Vector3d
 */
Eigen::Vector3d calculateMeanOfPointPositions(pcl::PointCloud<pcl::PointXYZ>::ConstPtr inputCloud);

/**
 * @brief
 *
 * @param inputCloud
 * @param transformMatrix
 * @return pcl::PointCloud<pcl::PointXYZ>::Ptr
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr transformCloud(
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr inputCloud,
  const Eigen::Affine3f & transformMatrix);

/**
 * @brief
 *
 * @param filename
 * @return pcl::PointCloud<pcl::PointXYZ>::Ptr
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr loadPointcloudFromPcd(const std::string & filename);

/*!
 * Finds clusters in the input cloud and returns vector point clouds.
 * Each pointcloud in the vector is a cluster in the input cloud.
 * There can be more than one cluster.
 * @param[in] pointer to the pcl point cloud
 * @return vector of point clouds. Vector will be empty if no clusters are found.
 */
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> extractClusterCloudsFromPointcloud(
  pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud);

/**
 * @brief Get the Rigid Body Transform object
 *
 * @param translation
 * @param intrinsicRpy
 * @param node_logger
 * @return Eigen::Affine3f
 */
Eigen::Affine3f getRigidBodyTransform(
  const Eigen::Vector3d & translation,
  const Eigen::Vector3d & intrinsicRpy,
  const rclcpp::Logger & node_logger);

/**
 * @brief Get the Rotation Matrix object
 *
 * @param angle
 * @param axis
 * @param node_logger
 * @return Eigen::Matrix3f
 */
Eigen::Matrix3f getRotationMatrix(
  double angle, XYZ axis,
  const rclcpp::Logger & node_logger);

/*!
* Downsample the point cloud using voxel grid method. Implementation is
* based on the implementation from pcl. The explanation of the algorithm
* can be found here:
* http://pointclouds.org/documentation/tutorials/voxel_grid.php
* @param[in] Input point cloud
* @return Downsampled point cloud
*/
pcl::PointCloud<pcl::PointXYZ>::Ptr downsampleInputCloud(
  pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, double downsmaple_leaf_size);

/*!
* Remove outliers from the point cloud. Function is based on
* the StatisticalOutlierRemoval filter from pcl. The explanation on
* how the algorithm works can be found here:
* http://pointclouds.org/documentation/tutorials/statistical_outlier.php
* @param[in] Input point cloud
* @return Point cloud where outliers have been removed.
*/
pcl::PointCloud<pcl::PointXYZ>::Ptr removeOutliersFromInputCloud(
  pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, int mean_K, double stddev_thres);

}  // namespace botanbot_utilities

#endif  // BOTANBOT_UTILITIES__PCL_HELPERS_HPP_
