// Copyright (c) 2020 Fetullah Atas
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

#ifndef BOTANBOT_TO_POSE_NAVIGATOR__BOTANBOT_TO_POSE_NAVIGATOR_HPP_
#define BOTANBOT_TO_POSE_NAVIGATOR__BOTANBOT_TO_POSE_NAVIGATOR_HPP_

#include <vector>
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "botanbot_msgs/action/compute_path_to_pose.hpp"
#include "botanbot_msgs/action/follow_path.hpp"
#include "botanbot_msgs/action/navigate_to_pose.hpp"

/**
 * @brief namespace for way point following, points are from a yaml file
 *
 */
namespace botanbot_to_pose_navigator
{

/**
 * @brief A ros node that drives robot through to requested poses
 *
 */
class NavigateToPoseServer : public rclcpp::Node
{
public:
// we need to be a client to ComputePathToPose to make a plan to requested pose
  using ComputePathToPose = botanbot_msgs::action::ComputePathToPose;
  using ComputePathToPoseGoalHandle =
    rclcpp_action::ClientGoalHandle<ComputePathToPose>;

  //  we need to be a client to FollowPath to control robot through planned path by
  // ComputePathToPose
  using FollowPath = botanbot_msgs::action::FollowPath;
  using FollowPathGoalHandle =
    rclcpp_action::ClientGoalHandle<FollowPath>;

  // using ComputePathToPose and FollowPath, we create an ation server
  // interafce named NavigateToPose
  using NavigateToPose = botanbot_msgs::action::NavigateToPose;
  using NavigateToPoseGoalHandle = rclcpp_action::ServerGoalHandle<NavigateToPose>;

  /**
   * @brief Construct a new Navigate To Pose object
   *
   */
  NavigateToPoseServer();

  /**
   * @brief Destroy the Navigate To Pose object
   *
   */
  ~NavigateToPoseServer();

  // ACTION CLIENT FUNCTIONs FOR ComputePathToPose
  void compute_path_to_pose_send_goal();
  void compute_path_to_pose_feedback_callback(
    ComputePathToPoseGoalHandle::SharedPtr,
    const std::shared_ptr<const ComputePathToPose::Feedback> feedback);
  void compute_path_to_pose_response_callback(
    std::shared_future<ComputePathToPoseGoalHandle::SharedPtr> future);
  void compute_path_to_pose_result_callback(
    const ComputePathToPoseGoalHandle::WrappedResult & result);


  // ACTION CLIENT FUNCTIONs FOR FollowPath
  void follow_path_send_goal();
  void follow_path_feedback_callback(
    FollowPathGoalHandle::SharedPtr,
    const std::shared_ptr<const FollowPath::Feedback> feedback);
  void follow_path_response_callback(
    std::shared_future<FollowPathGoalHandle::SharedPtr> future);
  void follow_path_result_callback(const FollowPathGoalHandle::WrappedResult & result);


  rclcpp_action::GoalResponse  navigate_to_pose_handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const NavigateToPose::Goal> goal);

  rclcpp_action::CancelResponse navigate_to_pose_handle_cancel(
    const std::shared_ptr<NavigateToPoseGoalHandle> goal_handle);

  void navigate_to_pose_handle_accepted(
    const std::shared_ptr<NavigateToPoseGoalHandle> goal_handle);

  void execute_navigate_to_pose(const std::shared_ptr<NavigateToPoseGoalHandle> goal_handle);

protected:
  // client to connect ComputePathToPose service
  rclcpp_action::Client<ComputePathToPose>::SharedPtr
    compute_path_to_pose_action_client_;
  // client to connect FollowPath service
  rclcpp_action::Client<FollowPath>::SharedPtr
    follow_path_action_client_;
  // Create a action server for NavigateToPose service
  rclcpp_action::Server<NavigateToPose>::SharedPtr
    navigate_to_pose_action_server_;
  // Node for spinning while waiting for an result from  ComputePathToPose
  rclcpp::Node::SharedPtr compute_path_to_pose_node_;
  // Node for spinning while waiting for an result from  FollowPath
  rclcpp::Node::SharedPtr follow_path_node_;
};
}  // namespace botanbot_to_pose_navigator

#endif  // BOTANBOT_TO_POSE_NAVIGATOR__BOTANBOT_TO_POSE_NAVIGATOR_HPP_
