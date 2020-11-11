/*
 * Copyright (c) 2020, Fetullah Atas
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   *
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// Navigation2 headers
#include <nav2_behavior_tree/plugins/action/navigate_to_pose_action.hpp>
#include <nav2_lifecycle_manager/lifecycle_manager_client.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <nav2_util/geometry_utils.hpp>
#include <nav2_util/robot_utils.hpp>
// ROS2 headers
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.h>
// TF2 headers
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/create_timer_interface.h>
// STL headers
#include <vector>

namespace botanbot_gui
{
enum class ActionStatus
{
  UNKNOWN = 0,
  PROCESSING = 1,
  FAILED = 2,
  SUCCEEDED = 3
};

/**
 * @brief Controls robot actions through navigation stack
 *
 */
class RobotController
{
public:
  // type shorteneings
  using ClientT = nav2_msgs::action::NavigateToPose;
  using ActionClient = rclcpp_action::Client<ClientT>;
  using NavigationGoalHandle = rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;

  /**
   * @brief Construct a new Robot Controller object
   *
   */
  RobotController();

  /**
   * @brief Destroy the Robot Controller object
   *
   */
  ~RobotController();

  /**
   * @brief send robot to passed pose
   *
   * @param pose goal pose
   */
  void toTargetPose(geometry_msgs::msg::PoseStamped pose);

  /**
   * @brief cancel any active goals
   *
   */
  void cancelGoals();

  /**
   * @brief Get the Robot States object
   *
   * @return geometry_msgs::msg::TransformStamped
   */
  geometry_msgs::msg::TransformStamped getRobotStates();

private:
  // client to navigate to pose action
  ActionClient::SharedPtr nav_to_pose_client_;
  // rclcpp node
  rclcpp::Node::SharedPtr node_;
  // tf buffer to get transfroms
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  // tf listner for tf transforms
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  // A client to cancel ongoing waypoint following
  rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SharedPtr
    waypoint_follower_action_client_;
};
}  // namespace botanbot_gui
