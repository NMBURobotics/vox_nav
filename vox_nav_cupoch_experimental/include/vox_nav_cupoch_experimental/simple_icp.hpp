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


#ifndef VOX_NAV_CUPOCH_EXPERIMENTAL__SIMPLE_ICP_HPP_
#define VOX_NAV_CUPOCH_EXPERIMENTAL__SIMPLE_ICP_HPP_

#include <pcl/PCLPointCloud2.h>
#include <pcl/common/common.h>
#include <pcl/conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl/filters/model_outlier_removal.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <cupoch/collision/collision.h>
#include <cupoch/cupoch.h>
#include <cupoch/geometry/occupancygrid.h>
#include <cupoch_conversions/cupoch_conversions.hpp>

#include "vox_nav_utilities/map_manager_helpers.hpp"
#include "vox_nav_utilities/pcl_helpers.hpp"
#include "vox_nav_utilities/tf_helpers.hpp"
#include "vox_nav_msgs/msg/object.hpp"
#include "vox_nav_msgs/msg/object_array.hpp"

#include <queue>
#include <vector>
#include <string>
#include <memory>
#include <mutex>

#include <Eigen/Core>

namespace vox_nav_cupoch_experimental
{

/**
 * @brief Given a raw point cloud,
 * clusterize it and use UKF to track clusters. Publish vis of tracks in RVIZ
 * and publish vox_nav_msgs::msg::ObjectArray
 *
 */
  class SimpleICP : public rclcpp::Node
  {

  public:
    /**
     * @brief Construct a new Raw Cloud Clustering Tracking object
     *
     */
    SimpleICP();
    /**
     * @brief Destroy the Raw Cloud Clustering Tracking object
     *
     */
    ~SimpleICP();

    /**
     * @brief Processing done in this func.
     *
     * @param cloud
     * @param poses
     */
    void cloudCallback(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud);

    /**
    * @brief Processing done in this func.
    *
    * @param cloud
    * @param poses
    */
    void mapCloudCallback(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud);

    /**
    * @brief Processing done in this func.
    *
    * @param cloud
    * @param poses
    */
    void gpsOdomCallback(
      const nav_msgs::msg::Odometry::ConstSharedPtr odom);

  private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_cloud_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gps_odom_subscriber_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr live_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_cloud_pub_;

    //Publish base to map after ICP correction
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      base_to_map_pose_pub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr
      new_robot_pose_publisher_;

    // tf buffer to get access to transfroms
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    bool map_configured_;
    std::once_flag get_map_cloud_once_;

    pcl::PointCloud<pcl::PointXYZRGB> map_;
    nav_msgs::msg::Odometry latest_gps_odom_;
    std::mutex latest_gps_odom_mutex_;


  };

}  // namespace vox_nav_cupoch_experimental

#endif  // VOX_NAV_CUPOCH_EXPERIMENTAL__SIMPLE_ICP_HPP_
