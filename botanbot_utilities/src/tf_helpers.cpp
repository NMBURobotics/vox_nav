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

#include <memory>
#include <string>
#include "botanbot_utilities/tf_helpers.hpp"

namespace botanbot_utilities
{

bool transformPose(
  const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::string frame,
  const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose,
  rclcpp::Duration & transform_tolerance
)
{
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    tf->transform(in_pose, out_pose, frame);
    return true;
  } catch (tf2::ExtrapolationException & ex) {
    auto transform = tf->lookupTransform(
      frame,
      in_pose.header.frame_id,
      tf2::TimePointZero
    );
    if (
      (rclcpp::Time(in_pose.header.stamp) - rclcpp::Time(transform.header.stamp)) >
      transform_tolerance)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("tf_help"),
        "Transform data too old when converting from %s to %s",
        in_pose.header.frame_id.c_str(),
        frame.c_str()
      );
      RCLCPP_ERROR(
        rclcpp::get_logger("tf_help"),
        "Data time: %ds %uns, Transform time: %ds %uns",
        in_pose.header.stamp.sec,
        in_pose.header.stamp.nanosec,
        transform.header.stamp.sec,
        transform.header.stamp.nanosec
      );
      return false;
    } else {
      tf2::doTransform(in_pose, out_pose, transform);
      return true;
    }
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(
      rclcpp::get_logger("tf_help"),
      "Exception in transformPose: %s",
      ex.what()
    );
    return false;
  }
  return false;
}

bool getCurrentPose(
  geometry_msgs::msg::PoseStamped & global_pose,
  tf2_ros::Buffer & tf_buffer, const std::string global_frame,
  const std::string robot_frame, const double transform_timeout)
{
  static rclcpp::Logger logger = rclcpp::get_logger("getCurrentPose");
  geometry_msgs::msg::PoseStamped robot_pose;

  tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
  tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
  robot_pose.header.frame_id = robot_frame;
  robot_pose.header.stamp = rclcpp::Time();

  try {
    global_pose = tf_buffer.transform(
      robot_pose, global_frame,
      tf2::durationFromSec(transform_timeout));
    return true;
  } catch (tf2::LookupException & ex) {
    RCLCPP_ERROR(
      logger,
      "No Transform available Error looking up robot pose: %s\n", ex.what());
  } catch (tf2::ConnectivityException & ex) {
    RCLCPP_ERROR(
      logger,
      "Connectivity Error looking up robot pose: %s\n", ex.what());
  } catch (tf2::ExtrapolationException & ex) {
    RCLCPP_ERROR(
      logger,
      "Extrapolation Error looking up robot pose: %s\n", ex.what());
  } catch (tf2::TimeoutException & ex) {
    RCLCPP_ERROR(
      logger,
      "Transform timeout with tolerance: %.4f", transform_timeout);
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(
      logger, "Failed to transform from %s to %s",
      global_frame.c_str(), robot_frame.c_str());
  }

  return false;
}
}  // namespace botanbot_utilities
