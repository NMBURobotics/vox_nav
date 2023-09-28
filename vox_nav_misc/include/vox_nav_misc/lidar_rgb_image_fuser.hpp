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

#ifndef VOX_NAV_MISC__LIDAR_RGB_IMAGE_FUSER_HPP_
#define VOX_NAV_MISC__LIDAR_RGB_IMAGE_FUSER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Dense>

namespace vox_nav_misc
{

class lidar_rgb_image_fuser : public rclcpp::Node
{
private:
  // tf buffer and listener
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::PointCloud2>
      LidarCamApprxTimeSyncPolicy;
  typedef message_filters::Synchronizer<LidarCamApprxTimeSyncPolicy> LidarCamApprxTimeSyncer;

  message_filters::Subscriber<sensor_msgs::msg::Image> image_subscriber_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> lidar_subscriber_;

  std::shared_ptr<LidarCamApprxTimeSyncer> time_syncher_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr fusion_image_pub_;

  // Camera model parameters are ;
  // fx 0 cx 0
  // 0 fy cy 0
  // 0 0 1 0
  // Eigen::MatrixXf K_{ { 605.36, 0, 322.282, 0 },  // NOLINT
  //                    { 0, 604.94, 248.99, 0 },   // NOLINT
  //                    { 0, 0, 1, 0 } };           // NOLINT

  // Simulated camera parameters
  Eigen::MatrixXf K_{ { 365.71429443359375, 0, 640.5, 0 },  // NOLINT
                      { 0, 365.4822082519531, 360.5, 0 },   // NOLINT
                      { 0, 0, 1, 0 } };                     // NOLINT

  // Camera distortion parameters

  // Get Eigen::Matrix4d transform from os_sensor to camera
  // This will come from tf
  Eigen::Matrix4f T_{ Eigen::Matrix4f::Identity() };

public:
  lidar_rgb_image_fuser();
  ~lidar_rgb_image_fuser();

  /**
   * @brief ousterCamCallback
   * @param image
   * @param cloud
   * This is the callback function for the synchronized lidar and camera messages
   */
  void ousterCamCallback(const sensor_msgs::msg::Image::ConstSharedPtr& image,
                         const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud);

  /**
   * @brief transform3DPointsToImage
   * @param points
   * @param TRANS_TO_IMAGE
   * @param uv
   * Use Homogenous transformation matrix to transform 3D points to image plane
   * normilized image plane coordinates are filled in uv matrix
   */
  void transform3DPointsToImage(const Eigen::MatrixXf& points,  // NOLINT
                                const Eigen::MatrixXf& TRANS_TO_IMAGE, Eigen::MatrixXf& uv);
};

}  // namespace vox_nav_misc
#endif  // VOX_NAV_MISC__LIDAR_RGB_IMAGE_FUSER_HPP_