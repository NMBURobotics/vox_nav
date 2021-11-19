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
#include <vector>
#include "vox_nav_utilities/pcl_helpers.hpp"

namespace vox_nav_utilities
{

  Eigen::Vector3d calculateMeanOfPointPositions(
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr inputCloud)
  {
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (const auto & point : inputCloud->points) {
      mean += Eigen::Vector3d(point.x, point.y, point.z);
    }
    mean /= inputCloud->points.size();

    return mean;
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformCloud(
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr inputCloud,
    const Eigen::Affine3f & transformMatrix)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::transformPointCloud(*inputCloud, *transformedCloud, transformMatrix);
    return transformedCloud;
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr loadPointcloudFromPcd(const std::string & filename)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PCLPointCloud2 cloudBlob;
    pcl::io::loadPCDFile(filename, cloudBlob);
    pcl::fromPCLPointCloud2(cloudBlob, *cloud);
    return cloud;
  }

  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> extractClusterCloudsFromPointcloud(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud)
  {
    // Create a kd tree to cluster the input point cloud
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(inputCloud);
    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> euclideanClusterExtraction;
    euclideanClusterExtraction.setClusterTolerance(0.1);
    euclideanClusterExtraction.setMinClusterSize(1);
    euclideanClusterExtraction.setMaxClusterSize(5000);
    euclideanClusterExtraction.setSearchMethod(tree);
    euclideanClusterExtraction.setInputCloud(inputCloud);
    euclideanClusterExtraction.extract(clusterIndices);

    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusterClouds;
    clusterClouds.reserve(clusterIndices.size());

    for (const auto & indicesSet : clusterIndices) {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
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

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr removeOutliersFromInputCloud(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud, int int_param, double double_param,
    OutlierRemovalType outlier_removal_type)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    if (outlier_removal_type == OutlierRemovalType::StatisticalOutlierRemoval) {
      pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
      sor.setInputCloud(inputCloud);
      sor.setMeanK(int_param);
      sor.setStddevMulThresh(double_param);
      sor.filter(*filteredCloud);
    } else {
      pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
      outrem.setInputCloud(inputCloud);
      outrem.setMinNeighborsInRadius(int_param);
      outrem.setRadiusSearch(double_param);
      outrem.setKeepOrganized(true);
      outrem.filter(*filteredCloud);
    }
    return filteredCloud;
  }

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
    const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters_array)
  {
    if (clusters_array.size() <= 0) {
      std::cout << "Publish empty clusters cloud. " << std::endl;
      sensor_msgs::msg::PointCloud2 msg_cloud;
      pcl::toROSMsg(*(new pcl::PointCloud<pcl::PointXYZRGB>), msg_cloud);
      msg_cloud.header = header;
      publisher->publish(msg_cloud);
      return;
    } else {
      std::cout << "Publishing " << clusters_array.size() << " clusters in one cloud." << std::endl;
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    // different clusters with different intensity
    for (size_t cluster_idx = 0u; cluster_idx < clusters_array.size(); ++cluster_idx) {
      if (clusters_array[cluster_idx]->points.size() <= 0) {
        //ROS_WARN_STREAM("An empty cluster #" << cluster_idx << ".");
        std::cout << "An empty cluster " << cluster_idx << "." << std::endl;
        continue;
      }
      for (size_t idx = 0u; idx < clusters_array[cluster_idx]->points.size(); ++idx) {
        pcl::PointXYZRGB point;
        point.x = clusters_array[cluster_idx]->points[idx].x;
        point.y = clusters_array[cluster_idx]->points[idx].y;
        point.z = clusters_array[cluster_idx]->points[idx].z;
        point.r = clusters_array[cluster_idx]->points[idx].r;
        point.g = clusters_array[cluster_idx]->points[idx].g;
        point.b = clusters_array[cluster_idx]->points[idx].b;
        cloud->points.push_back(point);
      }
    }
    if (cloud->size()) {
      sensor_msgs::msg::PointCloud2 msg_cloud;
      pcl::toROSMsg(*cloud, msg_cloud);
      msg_cloud.header = header;
      publisher->publish(msg_cloud);
    }
  }

  Eigen::Vector3f getColorByIndexEig(int index)
  {
    Eigen::Vector3f result;
    switch (index) {
      case -2:  // BLACK:
        result[0] = 0.0;
        result[1] = 0.0;
        result[2] = 0.0;
        break;
      case -1:  // BLACK:
        result[0] = 0.0;
        result[1] = 0.0;
        result[2] = 0.0;
        break;
      case 0:    // RED:
        result[0] = 0.8;
        result[1] = 0.1;
        result[2] = 0.1;
        break;
      case 1:    // GREEN:
        result[0] = 0.1;
        result[1] = 0.8;
        result[2] = 0.1;
        break;
      case 2:    // GREY:
        result[0] = 0.9;
        result[1] = 0.9;
        result[2] = 0.9;
        break;
      case 3:    // DARK_GREY:
        result[0] = 0.6;
        result[1] = 0.6;
        result[2] = 0.6;
        break;
      case 4:    // WHITE:
        result[0] = 1.0;
        result[1] = 1.0;
        result[2] = 1.0;
        break;
      case 5:    // ORANGE:
        result[0] = 1.0;
        result[1] = 0.5;
        result[2] = 0.0;
        break;
      case 6:    // Maroon:
        result[0] = 0.5;
        result[1] = 0.0;
        result[2] = 0.0;
        break;
      case 7:    // Olive:
        result[0] = 0.5;
        result[1] = 0.5;
        result[2] = 0.0;
        break;
      case 8:    // Navy:
        result[0] = 0.0;
        result[1] = 0.0;
        result[2] = 0.5;
        break;
      case 9:    // BLACK:
        result[0] = 0.0;
        result[1] = 0.0;
        result[2] = 0.0;
        break;
      case 10:    // YELLOW:
        result[0] = 1.0;
        result[1] = 1.0;
        result[2] = 0.0;
        break;
      case 11:    // BROWN:
        result[0] = 0.597;
        result[1] = 0.296;
        result[2] = 0.0;
        break;
      case 12:    // PINK:
        result[0] = 1.0;
        result[1] = 0.4;
        result[2] = 1;
        break;
      case 13:    // LIME_GREEN:
        result[0] = 0.6;
        result[1] = 1.0;
        result[2] = 0.2;
        break;
      case 14:    // PURPLE:
        result[0] = 0.597;
        result[1] = 0.0;
        result[2] = 0.597;
        break;
      case 15:    // CYAN:
        result[0] = 0.0;
        result[1] = 1.0;
        result[2] = 1.0;
        break;
      case 16:    // MAGENTA:
        result[0] = 1.0;
        result[1] = 0.0;
        result[2] = 1.0;
    }
    return result;
  }


}  // namespace vox_nav_utilities
