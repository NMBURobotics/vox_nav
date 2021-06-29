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

#ifndef VOX_NAV_UTILITIES__PCL_HELPERS_HPP_
#define VOX_NAV_UTILITIES__PCL_HELPERS_HPP_

#include <eigen3/Eigen/Core>

#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/uniform_sampling.h>


#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace vox_nav_utilities
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

  enum class OutlierRemovalType: int {RadiusOutlierRemoval, StatisticalOutlierRemoval};

/**
 * @brief
 *
 * @param inputCloud
 * @return Eigen::Vector3d
 */
  Eigen::Vector3d calculateMeanOfPointPositions(
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr inputCloud);

/**
 * @brief
 *
 * @param inputCloud
 * @param transformMatrix
 * @return pcl::PointCloud<pcl::PointXYZRGB>::Ptr
 */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformCloud(
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr inputCloud,
    const Eigen::Affine3f & transformMatrix);

/**
 * @brief
 *
 * @param filename
 * @return pcl::PointCloud<pcl::PointXYZRGB>::Ptr
 */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr loadPointcloudFromPcd(const std::string & filename);

/*!
 * Finds clusters in the input cloud and returns vector point clouds.
 * Each pointcloud in the vector is a cluster in the input cloud.
 * There can be more than one cluster.
 * @param[in] pointer to the pcl point cloud
 * @return vector of point clouds. Vector will be empty if no clusters are found.
 */
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> extractClusterCloudsFromPointcloud(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud);

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
* Remove outliers from the point cloud. Function is based on
* the StatisticalOutlierRemoval filter from pcl. The explanation on
* how the algorithm works can be found here:
* http://pointclouds.org/documentation/tutorials/statistical_outlier.php
* @param[in] Input point cloud
* @return Point cloud where outliers have been removed.
*/
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr removeOutliersFromInputCloud(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud, int mean_K, double stddev_thres,
    OutlierRemovalType outlier_removal_type);

/**
* @brief publish clustering objects' in one point cloud
* @param publisher
* @param header
* @param cloud_clusters
* @param trans
*/
  void publishClustersCloud(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher,
    const std_msgs::msg::Header header,
    const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters_array);

  template<typename P, typename T>
  P  getNearstPoint(
    const P & search_point,
    const T & cloud)
  {
    P nearest_point;
    pcl::KdTreeFLANN<P> kdtree;
    kdtree.setInputCloud(cloud);
    // K nearest neighbor search
    int K = 1;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    if (kdtree.nearestKSearch(search_point, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
      for (std::size_t i = 0; i < pointIdxNKNSearch.size(); ++i) {
        nearest_point = cloud->points[pointIdxNKNSearch[0]];
      }
    }
    return nearest_point;
  }

  template<typename P>
  typename pcl::PointCloud<P>::Ptr downsampleInputCloud(
    typename pcl::PointCloud<P>::Ptr inputCloud, double downsmaple_leaf_size)
  {
    pcl::VoxelGrid<P> voxelGrid;
    voxelGrid.setInputCloud(inputCloud);
    voxelGrid.setLeafSize(downsmaple_leaf_size, downsmaple_leaf_size, downsmaple_leaf_size);
    typename pcl::PointCloud<P>::Ptr downsampledCloud(new pcl::PointCloud<P>());
    voxelGrid.filter(*downsampledCloud);
    return downsampledCloud;
  }

  template<typename P>
  typename pcl::PointCloud<P>::Ptr  uniformly_sample_cloud(
    const typename pcl::PointCloud<P>::Ptr cloud,
    const double radius)
  {
    typename pcl::PointCloud<P>::Ptr uniformly_sampled_cloud(new pcl::PointCloud<P>());
    pcl::UniformSampling<P> filter;
    filter.setInputCloud(cloud);
    filter.setRadiusSearch(radius);
    filter.filter(*uniformly_sampled_cloud);
    uniformly_sampled_cloud->height = 1;
    uniformly_sampled_cloud->width = uniformly_sampled_cloud->points.size();
    return uniformly_sampled_cloud;
  }

  template<typename P>
  typename pcl::SupervoxelClustering<P>  super_voxelize_cloud(
    const typename pcl::PointCloud<P>::Ptr cloud,
    const bool disable_transform,
    const double voxel_resolution,
    const double seed_resolution,
    const double color_importance,
    const double spatial_importance,
    const double normal_importance
  )
  {
    pcl::SupervoxelClustering<P> super(voxel_resolution, seed_resolution);

    if (disable_transform) {
      super.setUseSingleCameraTransform(false);
    }
    super.setInputCloud(cloud);
    super.setColorImportance(color_importance);
    super.setSpatialImportance(spatial_importance);
    super.setNormalImportance(normal_importance);

    return super;
  }

}  // namespace vox_nav_utilities

#endif  // VOX_NAV_UTILITIES__PCL_HELPERS_HPP_
