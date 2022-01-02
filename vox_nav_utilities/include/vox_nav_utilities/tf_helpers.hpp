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

#ifndef VOX_NAV_UTILITIES__TF_HELPERS_HPP_
#define VOX_NAV_UTILITIES__TF_HELPERS_HPP_

#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace vox_nav_utilities
{
/**
 * @brief Transform a PoseStamped from one frame to another while catching exceptions
 *
 * Also returns immediately if the frames are equal.
 * @param tf Smart pointer to TFListener
 * @param frame Frame to transform the pose into
 * @param in_pose Pose to transform
 * @param out_pose Place to store the resulting transformed pose
 * @return True if successful transform
 */
  bool transformPose(
    const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::string frame,
    const geometry_msgs::msg::PoseStamped & in_pose,
    geometry_msgs::msg::PoseStamped & out_pose,
    rclcpp::Duration & transform_tolerance
  );

/**
 * @brief Get the Current Pose object
 *
 * @param global_pose
 * @param tf_buffer
 * @param global_frame
 * @param robot_frame
 * @param transform_timeout
 * @return true
 * @return false
 */
  bool getCurrentPose(
    geometry_msgs::msg::PoseStamped & global_pose,
    tf2_ros::Buffer & tf_buffer, const std::string global_frame = "map",
    const std::string robot_frame = "base_link", const double transform_timeout = 0.1);

/**
 * @brief Get the Euclidian Dist Between Poses object
 *
 * @param a
 * @param b
 * @return double
 */
  double getEuclidianDistBetweenPoses(
    const geometry_msgs::msg::PoseStamped a,
    const geometry_msgs::msg::PoseStamped b);

  /**
 * @brief Get the Euclidian Dist Between Poses object
 *
 * @param a
 * @param b
 * @return double
 */
  double getEuclidianDistBetweenPoses(
    const geometry_msgs::msg::Pose a,
    const geometry_msgs::msg::Pose b);

  /**
* @brief Get the Euclidian Dist Between Poses object
*
* @param a
* @param b
* @return double
*/
  double getEuclidianDistBetweenPoints(
    const geometry_msgs::msg::Point a,
    const geometry_msgs::msg::Point b);

/**
 * @brief Provide tf2::Quaternion and get roll pitch yaw
 *
 * @param q
 * @param roll
 * @param pitch
 * @param yaw
 */
  void getRPYfromTFQuaternion(const tf2::Quaternion q, double & roll, double & pitch, double & yaw);

/**
 * @brief Get the Quaternionfrom R P Y object
 *
 * @param roll
 * @param pitch
 * @param yaw
 * @return tf2::Quaternion
 */
  tf2::Quaternion getTFQuaternionfromRPY(const double roll, const double pitch, const double yaw);

/**
 * @brief Provide geometry_msgs::msg::Quaternion and get roll pitch yaw
 *
 * @param q_msg
 * @param roll
 * @param pitch
 * @param yaw
 */
  void getRPYfromMsgQuaternion(
    const geometry_msgs::msg::Quaternion q_msg, double & roll, double & pitch,
    double & yaw);

/**
 * @brief Get the Msg Quaternionfrom R P Y object
 *
 * @param roll
 * @param pitch
 * @param yaw
 * @return geometry_msgs::msg::Quaternion
 */
  geometry_msgs::msg::Quaternion getMsgQuaternionfromRPY(
    const double roll, const double pitch,
    const double yaw);

}  // namespace vox_nav_utilities

#endif  // VOX_NAV_UTILITIES__TF_HELPERS_HPP_
