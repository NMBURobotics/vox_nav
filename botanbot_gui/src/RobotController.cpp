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

#include <botanbot_gui/RobotController.h>

namespace botanbot_gui
{
using namespace std::chrono_literals;
RobotController::RobotController(/* args */)
{
  node_ = rclcpp::Node::make_shared("RobotController");
  nav_to_pose_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
    node_, "navigate"
    "_to_"
    "pose");
  waypoint_follower_action_client_ =
    rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(
    node_, "FollowW"
    "a"
    "ypoint"
    "s");

  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  node_->set_parameter(rclcpp::Parameter("use_sim_time", true));

  RCLCPP_INFO(node_->get_logger(), "Creating Robot Controller Instance");
}

RobotController::~RobotController()
{
  RCLCPP_INFO(node_->get_logger(), "Destroying A Robot Controller Instance");
}

void RobotController::toTargetPose(geometry_msgs::msg::PoseStamped pose)
{
  auto is_action_server_ready =
    nav_to_pose_client_->wait_for_action_server(std::chrono::seconds(5));
  if (!is_action_server_ready) {
    RCLCPP_ERROR(
      node_->get_logger(), "FollowWaypoints action server is not available."
      " Is the initial pose set?");
    return;
  }

  nav2_msgs::action::NavigateToPose::Goal navigation_goal;
  navigation_goal.pose = pose;

  // Enable result awareness by providing an empty lambda function
  auto send_goal_options =
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  send_goal_options.result_callback = [](auto) {};

  auto future_goal_handle =
    nav_to_pose_client_->async_send_goal(navigation_goal, send_goal_options);

  if (rclcpp::spin_until_future_complete(
      node_,
      future_goal_handle) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "Send goal call failed");
    return;
  }

  // Get the goal handle and save so that we can check on completion in the timer callback
  NavigationGoalHandle::SharedPtr navigation_goal_handle = future_goal_handle.get();

  auto result_future = nav_to_pose_client_->async_get_result(navigation_goal_handle);

  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult wrapped_result =
    result_future.get();

  if (!navigation_goal_handle) {
    RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
    return;
  }
}

geometry_msgs::msg::TransformStamped RobotController::getRobotStates()
{
  geometry_msgs::msg::TransformStamped current_pose;

  tf_buffer_->setUsingDedicatedThread(true);

  /*try {
    current_pose = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
  } catch (tf2::TransformException ex) {
    RCLCPP_ERROR(node_->get_logger(), " %s", ex.what());

    //rclcpp::sleep_for(20ns);
  }*/

  return current_pose;
}

void RobotController::cancelGoals()
{
  auto future_cancel = waypoint_follower_action_client_->async_cancel_all_goals();

  if (rclcpp::spin_until_future_complete(
      node_,
      future_cancel) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to cancel waypoint follower");
    return;
  }

  future_cancel = nav_to_pose_client_->async_cancel_all_goals();

  if (rclcpp::spin_until_future_complete(
      node_,
      future_cancel) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to cancel goal");
    return;
  }
}
}  // namespace botanbot_gui
