// Copyright (c) 2022 Fetullah Atas, Norwegian University of Life Sciences
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

#ifndef VOX_NAV_MISC__FAST_GICP_CLIENT_NO_GPS_HPP_
#define VOX_NAV_MISC__FAST_GICP_CLIENT_NO_GPS_HPP_

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
#include <pcl/registration/registration.h>

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

#include "vox_nav_utilities/map_manager_helpers.hpp"
#include "vox_nav_utilities/pcl_helpers.hpp"
#include "vox_nav_utilities/tf_helpers.hpp"
#include "vox_nav_msgs/msg/object.hpp"
#include "vox_nav_msgs/msg/object_array.hpp"

#include <fast_gicp/gicp/fast_gicp.hpp>
#include <fast_gicp/gicp/fast_gicp_st.hpp>
#include <fast_gicp/gicp/fast_vgicp.hpp>
#include <fast_gicp/ndt/ndt_cuda.hpp>
#include <fast_gicp/gicp/fast_vgicp_cuda.hpp>

#include <Eigen/Core>
#include <queue>
#include <vector>
#include <string>
#include <memory>
#include <mutex>
#include <algorithm>

namespace vox_nav_misc
{

struct ICPParameters
{
  float x_bound;
  float y_bound;
  float z_bound;
  float downsample_voxel_size;
  int max_icp_iter;
  float max_correspondence_distance;
  std::string method;
  int num_threads;
  bool debug;
};

/**
 * @brief Given a Point cloud map and an initial pose estimate, this class
 * performs ICP to find the best pose estimate for the robot. This class is
 * designed to be used with a live point cloud stream from a sensor, and a
 * pre-built map point cloud. The map point cloud is assumed to be in the
 * reference frame of the map, and the live point cloud is assumed to be in the
 * reference frame of the robot. The initial pose estimate is assumed to be in
 * the reference frame of the map. The output of this class is a pose estimate
 * of the robot in the reference frame of the map.
 *
 */
class FastGICPClientNoGPS : public rclcpp::Node
{
public:
  /**
   * @brief  Construct a new Fast GICP Client No GPS object
   *
   */
  FastGICPClientNoGPS();

  /**
   * @brief Destroy the Fast GICP Client No G P S object
   *
   */
  ~FastGICPClientNoGPS();

  /**
   * @brief Processing done in this func. Perform icp on live cloud and publish
   *
   * @param cloud
   * @param poses
   */
  void liveCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud);

  /**
   * @brief Get initial pose estimate from RVIZ
   *
   * @param msg
   */
  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);

  /**
   * @brief Recieves and stores map in the form of a point cloud as a member of this class
   *
   * @param cloud
   * @param poses
   */
  void mapCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud);

  /**
   * @brief Create a reg object, based on the method and number of threads specified in the params
   *
   * @param method
   * @param num_threads
   * @return pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>::Ptr
   */
  pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>::Ptr createRegistration(std::string method, int num_threads);

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr live_cloud_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_cloud_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_subscriber_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr live_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_cloud_pub_;

  // Publish base to map after ICP correction
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr base_to_map_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr new_robot_pose_publisher_;

  // tf buffer to get access to transfroms
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  bool map_configured_;
  bool initial_pose_recieved_;
  std::once_flag get_map_cloud_once_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;
  geometry_msgs::msg::PoseWithCovariance::SharedPtr initial_pose_;

  ICPParameters params_;
  Eigen::Matrix4f last_transform_estimate_;

  int sequence_;
};

}  // namespace vox_nav_misc

#endif  // VOX_NAV_MISC__FAST_GICP_CLIENT_NO_GPS_HPP_
