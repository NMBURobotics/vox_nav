// Copyright (c) 2016 Intel Corporation
// Modification Copyright (c) 2020 Fetullah Atas
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef BOTANBOT_GAZEBO__PLUGINS__GAZEBO_ROS_REALSENSE_HPP_
#define BOTANBOT_GAZEBO__PLUGINS__GAZEBO_ROS_REALSENSE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>

#include <memory>
#include <string>

#include "RealSensePlugin.hpp"

namespace gazebo
{
/// \brief A plugin that simulates Real Sense camera streams.
class GazeboRosRealsense : public RealSensePlugin
{
  /// \brief Constructor.

public:
  GazeboRosRealsense();

  /// \brief Destructor.

public:
  ~GazeboRosRealsense();

  // Documentation Inherited.

public:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  /// \brief Callback that publishes a received Depth Camera Frame as an
  /// ImageStamped message.

public:
  virtual void OnNewDepthFrame();

  /// \brief Helper function to fill the pointcloud information
  bool FillPointCloudHelper(
    sensor_msgs::msg::PointCloud2 & point_cloud_msg, uint32_t rows_arg,
    uint32_t cols_arg, uint32_t step_arg, void * data_arg);

  /// \brief Callback that publishes a received Camera Frame as an
  /// ImageStamped message.

public:
  virtual void OnNewFrame(
    const rendering::CameraPtr cam,
    const transport::PublisherPtr pub);

protected:
  boost::shared_ptr<camera_info_manager::CameraInfoManager>
  camera_info_manager_;

  /// \brief A pointer to the ROS node.
  ///  A node will be instantiated if it does not exist.

protected:
  rclcpp::Node::SharedPtr node_;

private:
  image_transport::ImageTransport * itnode_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;

protected:
  image_transport::CameraPublisher color_pub_, ir1_pub_, ir2_pub_, depth_pub_;

  /// \brief ROS image messages

protected:
  sensor_msgs::msg::Image image_msg_, depth_msg_;
  sensor_msgs::msg::PointCloud2 pointcloud_msg_;
};
}
#endif  // BOTANBOT_GAZEBO__PLUGINS__GAZEBO_ROS_REALSENSE_HPP_
