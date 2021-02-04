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

#include <string>
#include <vector>
#include <memory>
#include "botanbot_to_pose_navigator/botanbot_to_pose_navigator.hpp"

namespace botanbot_to_pose_navigator
{
NavigateToPoseServer::NavigateToPoseServer()
: Node("navigate_to_pose_action_server_rclcpp_node")
{
  compute_path_to_pose_action_client_ =
    rclcpp_action::create_client<ComputePathToPose>(
    this->get_node_base_interface(),
    this->get_node_graph_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "compute_path_to_pose");

  follow_path_action_client_ =
    rclcpp_action::create_client<FollowPath>(
    this->get_node_base_interface(),
    this->get_node_graph_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "follow_path");

  using namespace std::placeholders;
  navigate_to_pose_action_server_ = rclcpp_action::create_server<NavigateToPose>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "navigate_to_pose",
    std::bind(&NavigateToPoseServer::navigate_to_pose_handle_goal, this, _1, _2),
    std::bind(&NavigateToPoseServer::navigate_to_pose_handle_cancel, this, _1),
    std::bind(&NavigateToPoseServer::navigate_to_pose_handle_accepted, this, _1));

  compute_path_to_pose_node_ = std::make_shared<rclcpp::Node>(
    "compute_path_to_pose_action_client_rlccpp_node");
  follow_path_node_ = std::make_shared<rclcpp::Node>(
    "follow_action_client_rlccpp_node");

  RCLCPP_INFO(
    this->get_logger(),
    "Created an Instance of NavigateToPoseServer");
}

NavigateToPoseServer::~NavigateToPoseServer()
{
  RCLCPP_INFO(
    this->get_logger(),
    "Destroyed an Instance of NavigateToPoseServer");
}

void NavigateToPoseServer::compute_path_to_pose_feedback_callback(
  ComputePathToPoseGoalHandle::SharedPtr,
  const std::shared_ptr<const ComputePathToPose::Feedback> feedback)
{
  RCLCPP_INFO(
    this->get_logger(), "Elapsed time in seconds %.3f, while processng ComputePathToPose goal",
    feedback->elapsed_time.sec);
}

void NavigateToPoseServer::compute_path_to_pose_response_callback(
  std::shared_future<ComputePathToPoseGoalHandle::SharedPtr> future)
{
  auto goal_handle = future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by ComputePathToPose server");
  } else {
    RCLCPP_INFO(
      this->get_logger(),
      "Goal accepted by ComputePathToPose server, waiting for result");
  }
}

void NavigateToPoseServer::compute_path_to_pose_result_callback(
  const ComputePathToPoseGoalHandle::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
  }
}

void NavigateToPoseServer::follow_path_feedback_callback(
  FollowPathGoalHandle::SharedPtr,
  const std::shared_ptr<const FollowPath::Feedback> feedback)
{
  RCLCPP_INFO(
    this->get_logger(), "Elapsed time in seconds %.3f, while processng FollowPath goal",
    feedback->elapsed_time.sec);
  RCLCPP_INFO(
    this->get_logger(), "Distance to goal %.3f, while processng FollowPath goal",
    feedback->distance_to_goal);
  RCLCPP_INFO(
    this->get_logger(), "Robot speed %.3f, while processng FollowPath goal",
    feedback->speed);
}

void NavigateToPoseServer::follow_path_response_callback(
  std::shared_future<FollowPathGoalHandle::SharedPtr> future)
{
  auto goal_handle = future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by FollowPath server");
  } else {
    RCLCPP_INFO(
      this->get_logger(),
      "Goal accepted by FollowPath server, waiting for result");
  }
}

void NavigateToPoseServer::follow_path_result_callback(
  const FollowPathGoalHandle::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
  }
}

rclcpp_action::GoalResponse NavigateToPoseServer::navigate_to_pose_handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const NavigateToPose::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request for NavigateToPose");
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse NavigateToPoseServer::navigate_to_pose_handle_cancel(
  const std::shared_ptr<NavigateToPoseGoalHandle> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal of NavigateToPose");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void NavigateToPoseServer::navigate_to_pose_handle_accepted(
  const std::shared_ptr<NavigateToPoseGoalHandle> goal_handle)
{
  using namespace std::placeholders;
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&NavigateToPoseServer::execute_navigate_to_pose, this, _1),
    goal_handle}.detach();
}

void NavigateToPoseServer::execute_navigate_to_pose(
  const std::shared_ptr<NavigateToPoseGoalHandle> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing NavigateToPose goal");
  rclcpp::Rate loop_rate(1);
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<NavigateToPose::Feedback>();
  auto result = std::make_shared<NavigateToPose::Result>();

  if (!this->compute_path_to_pose_action_client_->
    wait_for_action_server(std::chrono::seconds(1)))
  {
    RCLCPP_ERROR(this->get_logger(), "ComputePathToPose Action server not available after waiting");
    return;
  }
  auto compute_path_to_pose_goal = ComputePathToPose::Goal();
  compute_path_to_pose_goal.pose = goal->pose;
  compute_path_to_pose_goal.planner_id = "default";
  using namespace std::placeholders;
  auto compute_path_to_pose_goal_options =
    rclcpp_action::Client<ComputePathToPose>::SendGoalOptions();
  compute_path_to_pose_goal_options.goal_response_callback =
    std::bind(&NavigateToPoseServer::compute_path_to_pose_response_callback, this, _1);
  compute_path_to_pose_goal_options.feedback_callback =
    std::bind(&NavigateToPoseServer::compute_path_to_pose_feedback_callback, this, _1, _2);
  compute_path_to_pose_goal_options.result_callback =
    std::bind(&NavigateToPoseServer::compute_path_to_pose_result_callback, this, _1);
  auto compute_path_to_pose_future_goal_handle =
    compute_path_to_pose_action_client_->async_send_goal(
    compute_path_to_pose_goal,
    compute_path_to_pose_goal_options);
  std::chrono::milliseconds server_timeout = std::chrono::milliseconds(500);
  if (rclcpp::spin_until_future_complete(
      compute_path_to_pose_node_,
      compute_path_to_pose_future_goal_handle,
      server_timeout) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    throw std::runtime_error("compute_path_to_pose send goal failed");
    return;
  }
  auto compute_path_to_pose_goal_handle = compute_path_to_pose_future_goal_handle.get();
  if (!compute_path_to_pose_goal_handle) {
    throw std::runtime_error("Goal was rejected by the compute_path_to_pose action server");
    return;
  }

  auto compute_path_to_pose_result = compute_path_to_pose_action_client_->async_get_result(
    compute_path_to_pose_goal_handle);


  if (!this->follow_path_action_client_->
    wait_for_action_server(std::chrono::seconds(1)))
  {
    RCLCPP_ERROR(this->get_logger(), "FollowPath Action server not available after waiting 1s");
    return;
  }
  auto folow_path_goal = FollowPath::Goal();
  folow_path_goal.path = compute_path_to_pose_result.get().result->path;
  folow_path_goal.controller_id = "default";
  using namespace std::placeholders;
  auto compute_path_to_pose_goal_options =
    rclcpp_action::Client<FollowPath>::SendGoalOptions();
  compute_path_to_pose_goal_options.goal_response_callback =
    std::bind(&NavigateToPoseServer::follow_path_response_callback, this, _1);
  compute_path_to_pose_goal_options.feedback_callback =
    std::bind(&NavigateToPoseServer::follow_path_feedback_callback, this, _1, _2);
  compute_path_to_pose_goal_options.result_callback =
    std::bind(&NavigateToPoseServer::follow_path_result_callback, this, _1);
  auto compute_path_to_pose_future_goal_handle =
    compute_path_to_pose_action_client_->async_send_goal(
    compute_path_to_pose_goal,
    compute_path_to_pose_goal_options);
  std::chrono::milliseconds server_timeout = std::chrono::milliseconds(500);
  if (rclcpp::spin_until_future_complete(
      compute_path_to_pose_node_,
      compute_path_to_pose_future_goal_handle,
      server_timeout) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    throw std::runtime_error("compute_path_to_pose send goal failed");
    return;
  }
  auto compute_path_to_pose_goal_handle = compute_path_to_pose_future_goal_handle.get();
  if (!compute_path_to_pose_goal_handle) {
    throw std::runtime_error("Goal was rejected by the compute_path_to_pose action server");
    return;
  }

  auto result = compute_path_to_pose_action_client_->async_get_result(
    compute_path_to_pose_goal_handle);
  // At this point our planner has created a apln and now we will call folloow path action to follow path
  // returned by planner


  // Check if goal is done
  if (rclcpp::ok()) {
    result->sequence = sequence;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }
}

}  // namespace botanbot_to_pose_navigator
