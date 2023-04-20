// Copyright (c) 2021 Fetullah Atas, Norwegian University of Life Sciences
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

#ifndef VOX_NAV_MISC__NAIVE_LIDAR_CLUSTERING_TRACKING_HPP_
#define VOX_NAV_MISC__NAIVE_LIDAR_CLUSTERING_TRACKING_HPP_

#include <pcl/PCLPointCloud2.h>
#include <pcl/common/common.h>
#include <pcl/conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include <pcl/features/moment_of_inertia_estimation.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl/filters/model_outlier_removal.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "vox_nav_utilities/map_manager_helpers.hpp"
#include "vox_nav_utilities/pcl_helpers.hpp"
#include "vox_nav_utilities/tf_helpers.hpp"
#include "vox_nav_msgs/msg/object.hpp"
#include "vox_nav_msgs/msg/object_array.hpp"

#include <queue>
#include <vector>
#include <string>
#include <memory>

#include <Eigen/Core>

namespace vox_nav_misc
{
  struct NaiveLIDARClusteringParameters
  {
    float x_bound;
    float y_bound;
    float z_bound;
    float downsample_voxel_size;
    float remove_ground_plane_thres;
    float clustering_min_points;
    float clustering_max_points;
    float clustering_max_step_size;
    float sacle_up_objects;
    int N;
    float dt;
    float min_cluster_height;
  };

  /**
   * @brief NaiveLIDARClustering class
   * The class is responsible for clustering the raw point cloud data
   * and publishing the detected objects as a ObjectArray message. Typically the
   * detected objects are used for tracking purposes.
   *
   */
  class NaiveLIDARClustering : public rclcpp::Node
  {

  public:
    /**
     * @brief Construct a new NaiveLIDARClustering object
     *
     */
    NaiveLIDARClustering();

    /**
     * @brief Destroy the NaiveLIDARClustering object
     *
     */
    ~NaiveLIDARClustering();

    /**
     * @brief Processing done in this func.
     *
     * @param cloud
     * @param poses
     */
    void cloudCallback(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud);

  private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscriber_;
    rclcpp::Publisher<vox_nav_msgs::msg::ObjectArray>::SharedPtr detection_objects_pub_;

    // Always represent the detections in the map frame
    std::shared_ptr<tf2_ros::Buffer> buffer_;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_;

    // Parameters
    NaiveLIDARClusteringParameters clustering_params_;
  };

} // namespace vox_nav_misc

#endif // VOX_NAV_MISC__NAIVE_LIDAR_CLUSTERING_TRACKING_HPP_
