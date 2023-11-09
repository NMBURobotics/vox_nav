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
#include "vox_nav_utilities/tf_helpers.hpp"

namespace vox_nav_utilities
{

bool transformPose(const std::shared_ptr<tf2_ros::Buffer> tf, const std::string frame,
                   const geometry_msgs::msg::PoseStamped& in_pose, geometry_msgs::msg::PoseStamped& out_pose,
                   rclcpp::Duration& transform_tolerance)
{
  // check to make sure a proper TF is present
  while (!tf->canTransform("base_link", "map", rclcpp::Time(0)))
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    RCLCPP_INFO(rclcpp::get_logger("tf_help"), "Waiting for Transforms to be available!");
  }
  if (in_pose.header.frame_id == frame)
  {
    out_pose = in_pose;
    return true;
  }

  try
  {
    tf->transform(in_pose, out_pose, frame);
    return true;
  }
  catch (tf2::ExtrapolationException& ex)
  {
    auto transform = tf->lookupTransform(frame, in_pose.header.frame_id, tf2::TimePointZero);
    if ((rclcpp::Time(in_pose.header.stamp) - rclcpp::Time(transform.header.stamp)) > transform_tolerance)
    {
      RCLCPP_ERROR(rclcpp::get_logger("tf_help"), "Transform data too old when converting from %s to %s",
                   in_pose.header.frame_id.c_str(), frame.c_str());
      RCLCPP_ERROR(rclcpp::get_logger("tf_help"), "Data time: %ds %uns, Transform time: %ds %uns",
                   in_pose.header.stamp.sec, in_pose.header.stamp.nanosec, transform.header.stamp.sec,
                   transform.header.stamp.nanosec);
      return false;
    }
    else
    {
      tf2::doTransform(in_pose, out_pose, transform);
      return true;
    }
  }
  catch (tf2::TransformException& ex)
  {
    RCLCPP_ERROR(rclcpp::get_logger("tf_help"), "Exception in transformPose: %s", ex.what());
    return false;
  }
  return false;
}

bool getCurrentPose(geometry_msgs::msg::PoseStamped& global_pose, tf2_ros::Buffer& tf_buffer,
                    const std::string global_frame, const std::string robot_frame, const double transform_timeout)
{
  static rclcpp::Logger logger = rclcpp::get_logger("getCurrentPose");
  geometry_msgs::msg::PoseStamped robot_pose;

  tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
  tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
  robot_pose.header.frame_id = robot_frame;
  robot_pose.header.stamp = rclcpp::Time();

  try
  {
    global_pose = tf_buffer.transform(robot_pose, global_frame, tf2::durationFromSec(transform_timeout));
    global_pose.pose.position.z = 0.5;
    return true;
  }
  catch (tf2::LookupException& ex)
  {
    RCLCPP_ERROR(logger, "No Transform available Error looking up robot pose: %s\n", ex.what());
  }
  catch (tf2::ConnectivityException& ex)
  {
    RCLCPP_ERROR(logger, "Connectivity Error looking up robot pose: %s\n", ex.what());
  }
  catch (tf2::ExtrapolationException& ex)
  {
    RCLCPP_ERROR(logger, "Extrapolation Error looking up robot pose: %s\n", ex.what());
  }
  catch (tf2::TimeoutException& ex)
  {
    RCLCPP_ERROR(logger, "Transform timeout with tolerance: %.4f", transform_timeout);
  }
  catch (tf2::TransformException& ex)
  {
    RCLCPP_ERROR(logger, "Failed to transform from %s to %s", global_frame.c_str(), robot_frame.c_str());
  }
  return false;
}

double getEuclidianDistBetweenPoses(const geometry_msgs::msg::PoseStamped a, const geometry_msgs::msg::PoseStamped b)
{
  double distance = std::sqrt(std::pow(a.pose.position.x - b.pose.position.x, 2) +
                              std::pow(a.pose.position.y - b.pose.position.y, 2) +
                              std::pow(a.pose.position.z - b.pose.position.z, 2));
  return distance;
}

double getEuclidianDistBetweenPoses(const geometry_msgs::msg::Pose a, const geometry_msgs::msg::Pose b)
{
  double distance = std::sqrt(std::pow(a.position.x - b.position.x, 2) + std::pow(a.position.y - b.position.y, 2) +
                              std::pow(a.position.z - b.position.z, 2));
  return distance;
}

double getEuclidianDistBetweenPoints(const geometry_msgs::msg::Point a, const geometry_msgs::msg::Point b)
{
  double distance = std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) + std::pow(a.z - b.z, 2));
  return distance;
}

void getRPYfromTFQuaternion(const tf2::Quaternion q, double& roll, double& pitch, double& yaw)
{
  tf2::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
}

tf2::Quaternion getTFQuaternionfromRPY(const double roll, const double pitch, const double yaw)
{
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  return q;
}

void getRPYfromMsgQuaternion(const geometry_msgs::msg::Quaternion q_msg, double& roll, double& pitch, double& yaw)
{
  tf2::Quaternion q;
  tf2::fromMsg(q_msg, q);
  tf2::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
}

geometry_msgs::msg::Quaternion getMsgQuaternionfromRPY(const double roll, const double pitch, const double yaw)
{
  tf2::Quaternion q;
  geometry_msgs::msg::Quaternion q_msg;
  q.setRPY(roll, pitch, yaw);
  q_msg = tf2::toMsg(q);
  return q_msg;
}

std::tuple<int, int, int> convert_to_rgb(double minval, double maxval, double val,
                                         const std::vector<std::tuple<int, int, int>>& colors)
{
  // Determine where the given value falls proportionality within
  // the range from minval->maxval and scale that fractional value
  // by the total number in the `colors` palette.
  double i_f = (val - minval) / (maxval - minval) * (colors.size() - 1);

  // Determine the lower index of the pair of color indices this
  // value corresponds and its fractional distance between the lower
  // and the upper colors.
  int i = static_cast<int>(i_f);
  double f = i_f - i;

  // Does it fall exactly on one of the color points?
  if (f < EPSILON)
  {
    return colors[i];
  }
  else
  {
    // Return a color linearly interpolated between the range of it and the following one.
    int r1, g1, b1;
    std::tie(r1, g1, b1) = colors[i];

    int r2, g2, b2;
    std::tie(r2, g2, b2) = colors[i + 1];

    return std::make_tuple(static_cast<int>(r1 + f * (r2 - r1)), static_cast<int>(g1 + f * (g2 - g1)),
                           static_cast<int>(b1 + f * (b2 - b1)));
  }
}

double euclidean_distance(const std::tuple<int, int, int>& c1, const std::tuple<int, int, int>& c2)
{
  int r1, g1, b1, r2, g2, b2;
  std::tie(r1, g1, b1) = c1;
  std::tie(r2, g2, b2) = c2;
  return std::sqrt(std::pow(r1 - r2, 2) + std::pow(g1 - g2, 2) + std::pow(b1 - b2, 2));
}

double convert_to_value(const std::tuple<int, int, int>& rgb, double minval, double maxval,
                        const std::vector<std::tuple<int, int, int>>& colors)
{
  int r, g, b;
  std::tie(r, g, b) = rgb;

  for (size_t idx = 0; idx < colors.size() - 1; ++idx)
  {
    if (euclidean_distance(rgb, colors[idx]) + euclidean_distance(rgb, colors[idx + 1]) <=
        euclidean_distance(colors[idx], colors[idx + 1]))
    {
      double f = euclidean_distance(rgb, colors[idx]) / euclidean_distance(colors[idx], colors[idx + 1]);
      return minval + (maxval - minval) * (idx + f) / static_cast<double>(colors.size() - 1);
    }
  }

  return minval - 1.0;
}

}  // namespace vox_nav_utilities
