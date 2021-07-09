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

#ifndef VOX_NAV_UTILITIES__MAP_MANAGER_HELPERS_HPP_
#define VOX_NAV_UTILITIES__MAP_MANAGER_HELPERS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <octomap/octomap_utils.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <utility>
#include <vector>
#include <string>
#include <memory>

namespace vox_nav_utilities
{

/**
 * @brief
 *
 */
  void fillOctomapMarkers(
    visualization_msgs::msg::MarkerArray::SharedPtr & marker_array,
    const std_msgs::msg::Header::SharedPtr & header,
    const std::shared_ptr<octomap::OcTree> & octree);

/**
* @brief Given a pointcloud, denoise it with use of K- neighbour points and return a pointer to denoised cloud
*
* @param cloud
* @param radius
* @param tolerated_divergence_rate
* @param min_num_neighbours
* @return pcl::PointCloud<pcl::PointXYZRGB>::Ptr
*/
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr denoise_segmented_cloud(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
    const double radius,
    const double tolerated_divergence_rate,
    const int min_num_neighbours);

/**
 * @brief Get the traversable points object,
 * this is to seperate traversable points from rest.
 * Traverability is encoded in RGB color channels. G > 0 is traversale
 *
 * @param cloud
 * @return pcl::PointCloud<pcl::PointXYZRGB>::Ptr
 */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_traversable_points(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

/**
 * @brief Get the NON traversable points object,
 * this is to seperate NON traversable points from rest.
 * Traverability is encoded in RGB color channels. R > 0 is  NON traversale
 *
 * @param cloud
 * @return pcl::PointCloud<pcl::PointXYZRGB>::Ptr
 */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_non_traversable_points(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

/**
 * @brief This function, partitionates full pure traversable cloud into cells with given radius.
 * Center of these cells are basically uniformly sampled cloud.
 * Geometry based features ar drwan from cells to
 * assign a cost value to points of each cell.
 *
 * @param pure_traversable_pcl
 * @param uniformly_sampled_nodes
 * @param radius
 * @return std::vector<std::pair<pcl::PointXYZRGB,
 * pcl::PointCloud<pcl::PointXYZRGB>::Ptr>>
 */
  std::vector<std::pair<pcl::PointXYZRGB,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr>> surfelize_traversability_cloud(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pure_traversable_pcl,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr uniformly_sampled_nodes,
    const double radius);

/**
 * @brief This function is used o fit a plane model to each cell of traversability cloud.
 * 
 * @param coefficients 
 * @param cloud 
 * @param dist_thes 
 * @return true 
 * @return false 
 */
  bool fit_plane_to_cloud(
    pcl::ModelCoefficients::Ptr coefficients,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
    const double dist_thes);

/**
 * @brief Set the cloud color object. Paints clouds color to given colors.
 * Colors must be a vector with size of 3. Incrementally values corresponds to
 * RGB. MAX color value is 255
 *
 * @param cloud
 * @param colors
 * @return pcl::PointCloud<pcl::PointXYZRGB>::Ptr
 */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr set_cloud_color(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
    const std::vector<double> colors);

/**
 * @brief given plane model, calculate yaw pitch roll from this plane
 *
 * @param plane_model
 * @return std::vector<double>
 */
  std::vector<double> rpy_from_plane(
    const pcl::ModelCoefficients plane_model);

/**
 * @brief Average perpendicular distance of outlier points from plane model.
 * The more ditance, the more error. This function is used as an feature extractor from
 * decomposed cells.
 *
 * @param cloud
 * @param plane_model
 * @return double
 */
  double average_point_deviation_from_plane(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
    const pcl::ModelCoefficients plane_model);

/**
 * @brief Finds min and max height differnce between edge points.
 * Perfroms a simple physics based energy differnce.
 *
 * @param cloud
 * @param m
 * @param v
 * @return double
 */
  double max_energy_gap_in_cloud(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
    const double m,
    const double v);

}  // namespace vox_nav_utilities

#endif  // VOX_NAV_UTILITIES__MAP_MANAGER_HELPERS_HPP_
