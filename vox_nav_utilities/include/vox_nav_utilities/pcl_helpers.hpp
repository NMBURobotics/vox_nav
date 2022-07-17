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
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/model_outlier_removal.h>

#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <iostream>
#include <random>
#include <experimental/algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/point.hpp"

namespace vox_nav_utilities
{
/**
* @brief
*
*/
  struct RigidBodyTransformation
  {
    Eigen::Vector3d translation_{0.0, 0.0, 0.0};

    // intrinsic rotation (opposite from the ROS convention), order X-Y-Z
    Eigen::Vector3d rpyIntrinsic_{0.0, 0.0, 0.0};
  };
  enum class XYZ : int
  {
    X, Y, Z
  };

  enum class OutlierRemovalType : int
  {
    RadiusOutlierRemoval, StatisticalOutlierRemoval
  };

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
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud, int int_param, double double_param,
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
  P getNearstPoint(
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


  template<typename P, typename T>
  P getNearstRPoints(
    const double radius,
    const P & search_point,
    const T & cloud)
  {
    // Find NN inliers in radius and select a random one
    P nearest_point;
    pcl::KdTreeFLANN<P> kdtree;
    kdtree.setInputCloud(cloud);
    // radius nearest neighbor search
    std::vector<int> pointIdxNKNSearch;
    std::vector<float> pointNKNSquaredDistance;
    if (kdtree.radiusSearchT(
        search_point, radius, pointIdxNKNSearch,
        pointNKNSquaredDistance) > 0)
    {
      std::vector<int> out;
      size_t nelems = 1;
      /*std::sample(
        pointIdxNKNSearch.begin(),
        pointIdxNKNSearch.end(),
        std::back_inserter(out),
        nelems,
        std::mt19937{std::random_device{} ()}
      );*/
      std::experimental::fundamentals_v2::sample(
        pointIdxNKNSearch.begin(),
        pointIdxNKNSearch.end(),
        std::back_inserter(out),
        nelems,
        std::mt19937{std::random_device{} ()}
      );
      return cloud->points[out[0]];
    }
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
  typename pcl::PointCloud<P>::Ptr uniformlySampleCloud(
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
  typename pcl::SupervoxelClustering<P> supervoxelizeCloud(
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

  template<typename P>
  typename pcl::PointCloud<P>::Ptr getSubCloudWithinRadius(
    const typename pcl::PointCloud<P>::Ptr cloud,
    const P & search_point,
    const double radius)
  {
    typename pcl::PointCloud<P>::Ptr subcloud_within_radius(new pcl::PointCloud<P>());
    float resolution = 0.2;
    pcl::octree::OctreePointCloudSearch<P> octree(resolution);
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();

    // Neighbors within radius search
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    if (octree.radiusSearch(
        search_point, radius, pointIdxRadiusSearch,
        pointRadiusSquaredDistance) > 0)
    {
      for (std::size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
        subcloud_within_radius->points.push_back(cloud->points[pointIdxRadiusSearch[i]]);
      }
    }
    return subcloud_within_radius;
  }

  template<typename P>
  typename pcl::PointCloud<P>::Ptr removeNans(
    const typename pcl::PointCloud<P>::Ptr cloud)
  {
    typename pcl::PointCloud<P>::Ptr nans_removed_cloud(
      new pcl::PointCloud<P>());

    for (auto && i: cloud->points) {
      if (pcl::isFinite<P>(i)) {
        nans_removed_cloud->points.push_back(i);
      }
    }

    return nans_removed_cloud;
  }

  template<typename P>
  double PCLPointEuclideanDist(
    const P & a,
    const P & b)
  {
    double distance = std::sqrt(
      std::pow(a.x - b.x, 2) +
      std::pow(a.y - b.y, 2) +
      std::pow(a.z - b.z, 2));
    return distance;
  }

  template<typename P>
  typename pcl::PointCloud<P>::Ptr segmentSurfacePlane(
    const typename pcl::PointCloud<P>::Ptr cloud, double dist, bool set_negative)
  {
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // Create the segmentation object
    typename pcl::SACSegmentation<P> seg;
    typename pcl::ExtractIndices<P> extract;
    typename pcl::PointCloud<P>::Ptr filtered(new pcl::PointCloud<P>());

    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(dist);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(set_negative);
    extract.filter(*filtered);

    return filtered;
  }

  template<typename KeyType, typename ValueType>
  std::pair<KeyType, ValueType> getMax(const std::map<KeyType, ValueType> & x)
  {
    using pairtype = std::pair<KeyType, ValueType>;
    return *std::max_element(
      x.begin(), x.end(), [](const pairtype & p1, const pairtype & p2) {
        return p1.second < p2.second;
      });
  }

  template<typename P>
  typename pcl::PointCloud<P>::Ptr denoiseSegmentedCloud(
    const typename pcl::PointCloud<P>::Ptr cloud, double radius,
    double tolerated_divergence_rate, int min_num_neighbours)
  {
    typename pcl::PointCloud<P>::Ptr denoised_cloud(new pcl::PointCloud<P>());
    typename pcl::KdTreeFLANN<P> kdtree;

    kdtree.setInputCloud(cloud);

    for (size_t i = 0; i < cloud->points.size(); i++) {

      P searchPoint = cloud->points[i];
      std::vector<int> pointIdxRadiusSearch;
      std::vector<float> pointRadiusSquaredDistance;

      if (kdtree.nearestKSearch(
          searchPoint, min_num_neighbours, pointIdxRadiusSearch,
          pointRadiusSquaredDistance) > 0)
      {
        int serach_point_key =
          ((int) searchPoint.r) << 16 | ((int) searchPoint.g) << 8 | ((int) searchPoint.b);

        typename pcl::PointCloud<P>::Ptr neighbours(new pcl::PointCloud<P>);
        for (std::size_t j = 0; j < pointIdxRadiusSearch.size(); ++j) {
          neighbours->points.push_back(cloud->points[pointIdxRadiusSearch[j]]);
        }

        std::vector<int> neighbour_labels;
        for (std::size_t j = 0; j < neighbours->points.size(); ++j) {
          int r = neighbours->points[j].r;
          int g = neighbours->points[j].g;
          int b = neighbours->points[j].b;
          int rgb = ((int) r) << 16 | ((int) g) << 8 | ((int) b);
          neighbour_labels.push_back(rgb);
        }

        if (neighbour_labels.size() > 200) {
          continue;
        }

        std::map<int, int> M;
        for (int k = 0; k < neighbour_labels.size(); k++) {
          if (M.find(neighbour_labels[k]) == M.end()) {
            M[neighbour_labels[k]] = 1;
          } else {
            M[neighbour_labels[k]]++;
          }
        }

        auto max = getMax<int, int>(M);
        int num_search_point_neigbours_same_class;
        auto search_point_neigbours_same_class = M.find(serach_point_key);
        if (search_point_neigbours_same_class == M.end()) {
          continue;
        } else {
          num_search_point_neigbours_same_class = search_point_neigbours_same_class->second;
        }

        if (max.second > num_search_point_neigbours_same_class) {
          std::uint8_t r = (max.first >> 16) & 0x0000ff;
          std::uint8_t g = (max.first >> 8) & 0x0000ff;
          std::uint8_t b = (max.first) & 0x0000ff;
          searchPoint.r = r;
          searchPoint.g = g;
          searchPoint.b = b;
        }
      }
      searchPoint.a = 255;
      denoised_cloud->points.push_back(searchPoint);
    }

    denoised_cloud->height = 1;
    denoised_cloud->width = denoised_cloud->points.size();
    return denoised_cloud;
  }

  template<typename P>
  typename pcl::PointCloud<P>::Ptr denoiseSegmentedCloud(
    const typename pcl::PointCloud<P>::Ptr dynamic_points_pcl,
    const typename pcl::PointCloud<P>::Ptr static_points_pcl,
    double radius,
    int min_num_neighbours)
  {
    typename pcl::PointCloud<P>::Ptr denoised_cloud(new pcl::PointCloud<P>());
    typename pcl::KdTreeFLANN<P> kdtree;
    kdtree.setInputCloud(static_points_pcl);

    for (size_t i = 0; i < dynamic_points_pcl->points.size(); i++) {

      P searchPoint = dynamic_points_pcl->points[i];
      std::vector<int> pointIdxRadiusSearch;
      std::vector<float> pointRadiusSquaredDistance;

      if (kdtree.radiusSearch(
          searchPoint,
          min_num_neighbours,
          pointIdxRadiusSearch,
          pointRadiusSquaredDistance) <
        min_num_neighbours)
      {
        denoised_cloud->points.push_back(searchPoint);
      }

    }
    denoised_cloud->height = 1;
    denoised_cloud->width = denoised_cloud->points.size();
    return denoised_cloud;
  }

  template<typename P>
  typename pcl::PointCloud<P>::Ptr cropBox(
    const typename pcl::PointCloud<P>::Ptr cloud,
    Eigen::Vector4f min,
    Eigen::Vector4f max)
  {
    typename pcl::PointCloud<P>::Ptr crop_cloud(new pcl::PointCloud<P>());
    typename pcl::CropBox<P> boxFilter(true);
    boxFilter.setMin(min);
    boxFilter.setMax(max);
    boxFilter.setInputCloud(cloud);
    std::vector<int> indices;
    boxFilter.filter(indices);

    pcl::PointIndices::Ptr inliers_crop{new pcl::PointIndices};
    for (int point : indices) {
      inliers_crop->indices.push_back(point);
    }
    typename pcl::ExtractIndices<P> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers_crop);
    extract.setNegative(false);
    extract.filter(*crop_cloud);
    crop_cloud->height = 1;
    crop_cloud->width = crop_cloud->points.size();
    return crop_cloud;
  }

  template<typename P>
  typename pcl::PointCloud<P>::Ptr regionGrowingRGB(
    const typename pcl::PointCloud<P>::Ptr cloud,
    double dist_thres = 10,
    double point_color_thres = 6,
    double region_color_thres = 5,
    double filter_limit_min = 0.0,
    double filter_limit_max = 1.0,
    int min_cluster_size = 600)
  {
    typename pcl::search::Search<P>::Ptr tree(new pcl::search::KdTree<P>);
    typename pcl::PassThrough<P> pass;
    typename pcl::RegionGrowingRGB<P> reg;

    pcl::IndicesPtr indices(new std::vector<int>);

    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(filter_limit_min, filter_limit_max);
    pass.filter(*indices);

    reg.setInputCloud(cloud);
    reg.setIndices(indices);
    reg.setSearchMethod(tree);
    reg.setDistanceThreshold(dist_thres);
    reg.setPointColorThreshold(point_color_thres);
    reg.setRegionColorThreshold(region_color_thres);
    reg.setMinClusterSize(min_cluster_size);

    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);

    return reg.getColoredCloud();
  }

  template<typename P>
  typename pcl::PointCloud<P>::Ptr regionGrowingNormal(
    const typename pcl::PointCloud<P>::Ptr cloud,
    double filter_limit_min = 0.0,
    double filter_limit_max = 1.0,
    int k_search = 50,
    int min_cluster_size = 50,
    int max_cluster_size = 1000000,
    int num_neighbours = 30,
    double curvature_thres = 1.0,
    double smothness_thres = 2.0)
  {
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    typename pcl::search::Search<P>::Ptr tree(new pcl::search::KdTree<P>);
    typename pcl::NormalEstimation<P, pcl::Normal> normal_estimator;
    typename pcl::PassThrough<P> pass;
    typename pcl::RegionGrowing<P, pcl::Normal> reg;

    pcl::IndicesPtr indices(new std::vector<int>);

    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(cloud);
    normal_estimator.setKSearch(k_search);
    normal_estimator.compute(*normals);

    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(filter_limit_min, filter_limit_max);
    pass.filter(*indices);

    reg.setMinClusterSize(min_cluster_size);
    reg.setMaxClusterSize(max_cluster_size);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(num_neighbours);
    reg.setInputCloud(cloud);
    //reg.setIndices (indices);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(smothness_thres / 180.0 * M_PI);
    reg.setCurvatureThreshold(curvature_thres);

    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);

    return reg.getColoredCloud();
  }

  template<typename P>
  std::vector<typename pcl::PointCloud<P>::Ptr> euclideanClustering(
    const typename pcl::PointCloud<P>::Ptr cloud,
    int min_cluster_size = 50,
    int max_cluster_size = 1000000,
    double tolerancce = 0.2
  )
  {
    typename pcl::search::Search<P>::Ptr tree(new pcl::search::KdTree<P>);
    typename pcl::EuclideanClusterExtraction<P> ec;
    std::vector<typename pcl::PointCloud<P>::Ptr> clusters;

    pcl::IndicesPtr indices(new std::vector<int>);
    std::vector<pcl::PointIndices> cluster_indices;

    ec.setClusterTolerance(tolerancce);
    ec.setMinClusterSize(min_cluster_size);
    ec.setMaxClusterSize(max_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);


    int j = 0;
    for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
      typename pcl::PointCloud<P>::Ptr cloud_cluster(new pcl::PointCloud<P>);

      for (const auto & idx : it->indices) {
        cloud_cluster->push_back((*cloud)[idx]); //*
      }
      cloud_cluster->width = cloud_cluster->size();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;
      j++;

      clusters.push_back(cloud_cluster);

    }

    return clusters;
  }

  template<typename P>
  typename pcl::PointCloud<P>::Ptr removePointsWithinGroundPlaneofOtherCloud(
    typename pcl::PointCloud<P>::Ptr cloud,
    const typename pcl::PointCloud<P>::Ptr ground_cloud,
    double distance_thres)
  {
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    typename pcl::SACSegmentation<P> seg;
    typename pcl::ModelOutlierRemoval<P> filter;

    seg.setOptimizeCoefficients(false);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(distance_thres);
    seg.setInputCloud(ground_cloud);
    seg.segment(*inliers, *coefficients);
    filter.setModelCoefficients(*coefficients);
    filter.setThreshold(distance_thres);
    filter.setModelType(pcl::SACMODEL_PLANE);
    filter.setInputCloud(cloud);
    filter.setNegative(true);
    filter.filter(*cloud);

    return cloud;

  }

  Eigen::Vector3f getColorByIndexEig(int index);


}   // namespace vox_nav_utilities

#endif  // VOX_NAV_UTILITIES__PCL_HELPERS_HPP_
