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

#ifndef BOTANBOT_UTILITIES__PCL2OCTOMAP_CONVERTER_HPP_
#define BOTANBOT_UTILITIES__PCL2OCTOMAP_CONVERTER_HPP_

#include <octomap/octomap.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <iostream>
#include <limits>
#include <exception>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "botanbot_utilities/pcl_helpers.hpp"


namespace botanbot_utilities
{

class PCL2OctomapConverter : public rclcpp::Node
{
private:
  // Optional rigid body transform to apply o the cloud, if cloud
  // is depth camera frames we need to pull cloud back to conventional ROS frames
  botanbot_utilities::RigidBodyTransformation pointloud_transform_matrix_;
  std::string input_pcd_filename_;
  std::string output_binary_octomap_filename_;
  // optional point cloud transformfrom yaml file
  double downsample_voxel_size_;
  int remove_outlier_mean_K_;
  double remove_outlier_stddev_threshold_;
  double remove_outlier_radius_search_;
  int remove_outlier_min_neighbors_in_radius_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_;

public:
/**
 * @brief Construct a new PCL2OctomapConverter object
 *
 */
  PCL2OctomapConverter();

  /**
   * @brief Destroy the PCL2OctomapConverter object
   *
   */
  ~PCL2OctomapConverter();

  /**
   * @brief
   *
   * @param tree
   * @param num_thresholded
   * @param num_other
   */
  void calcThresholdedNodes(
    const octomap::OcTree tree,
    unsigned int & num_thresholded,
    unsigned int & num_other);

  /**
   * @brief
   *
   * @param tree
   */
  void outputStatistics(const octomap::OcTree tree);

  /**
   * @brief
   *
   */
  void processConversion();
};

}  // namespace botanbot_utilities
#endif  // BOTANBOT_UTILITIES__PCL2OCTOMAP_CONVERTER_HPP_
