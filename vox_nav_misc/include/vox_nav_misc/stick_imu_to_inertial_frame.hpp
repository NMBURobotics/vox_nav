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

#ifndef VOX_NAV_MISC__STICK_IMU_TO_INERTIAL_FRAME_HPP_
#define VOX_NAV_MISC__STICK_IMU_TO_INERTIAL_FRAME_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace vox_nav_misc
{

class stick_imu_to_inertial_frame : public rclcpp::Node
{
private:
  // tf buffer and listener
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr corrected_imu_pub_;

public:
  stick_imu_to_inertial_frame();
  ~stick_imu_to_inertial_frame();

  /**
   *
   * @brief imuCallback
   * @param msg
   * Callback function for imu messages
   */
  void imuCallback(sensor_msgs::msg::Imu::ConstSharedPtr msg);
};

}  // namespace vox_nav_misc
#endif  // VOX_NAV_MISC__STICK_IMU_TO_INERTIAL_FRAME_HPP_