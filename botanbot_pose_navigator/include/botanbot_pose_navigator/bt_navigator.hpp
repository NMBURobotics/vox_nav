// Copyright (c) 2018 Intel Corporation
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

#ifndef botanbot_pose_navigator__BT_NAVIGATOR_HPP_
#define botanbot_pose_navigator__BT_NAVIGATOR_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "botanbot_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_action/server.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "botanbot_pose_navigator/behavior_tree.hpp"

namespace botanbot_pose_navigator
{
/**
 * @class botanbot_pose_navigator::BtNavigator
 * @brief An action server that uses behavior tree for navigating a robot to its
 * goal position.
 */
class BtNavigator : public rclcpp::Node
{
public:
  BtNavigator();

  ~BtNavigator();

protected:
  using ActionServer = botanbot_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ActionServer>;

  // The action server presented by this node
  rclcpp_action::Server<ActionServer>::SharedPtr action_server_;

  // The three action server callbacks
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ActionServer::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandle> goal_handle);

  void handle_accepted(
    const std::shared_ptr<GoalHandle> goal_handle);

  // The routine to run on the separate thread
  void navigateToPose(const std::shared_ptr<GoalHandle> goal_handle);

  // A regular, non-spinning ROS node that we can use for calls to the action client
  rclcpp::Node::SharedPtr client_node_;

  // Spinning transform that can be used by the BT nodes
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Metrics for feedback
  rclcpp::Time start_time_;
  std::string robot_frame_;
  std::string global_frame_;
  double transform_tolerance_;

  // The XML string that defines the Behavior Tree used to implement the print_message action
  static const char bt_xml_[];
};

}  // namespace botanbot_pose_navigator

#endif  // botanbot_pose_navigator__BT_NAVIGATOR_HPP_
