// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2019 Samsung Research America
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

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "builtin_interfaces/msg/duration.hpp"
#include "vox_nav_planning/planner_server.hpp"

using namespace std::chrono_literals;

namespace vox_nav_planning
{
  PlannerServer::PlannerServer()
  : Node("vox_nav_planning_server_rclcpp_node"),
    pc_loader_("vox_nav_planning", "vox_nav_planning::PlannerCore"),
    planner_id_("SE2Planner"),
    planner_type_("vox_nav_planning::SE2Planner")
  {
    RCLCPP_INFO(get_logger(), "Creating");

    // Declare this node's parameters
    declare_parameter("expected_planner_frequency", 1.0);
    declare_parameter("planner_plugin", "SE2Planner");
    declare_parameter("enabled", true);
    declare_parameter("planner_name", "PRMStar");
    declare_parameter("planner_timeout", 5.0);
    declare_parameter("interpolation_parameter", 50);
    declare_parameter("octomap_voxel_size", 0.2);
    declare_parameter("robot_body_dimens.x", 1.0);
    declare_parameter("robot_body_dimens.y", 0.8);
    declare_parameter("robot_body_dimens.z", 0.6);
    declare_parameter("robot_mesh_path", "");

    get_parameter("expected_planner_frequency", expected_planner_frequency_);
    get_parameter("planner_plugin", planner_id_);
    get_parameter("robot_mesh_path", robot_mesh_path_);


    declare_parameter(planner_id_ + ".plugin", planner_type_);
    get_parameter(planner_id_ + ".plugin", planner_type_);

    try {
      vox_nav_planning::PlannerCore::Ptr planner =
        pc_loader_.createSharedInstance(planner_type_);
      planner->initialize(this, planner_id_);
      RCLCPP_INFO(
        get_logger(), "Created planner plugin %s of type %s",
        planner_id_.c_str(), planner_type_.c_str());
      planners_.insert({planner_id_, planner});
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(
        get_logger(), "Failed to create planner. Exception: %s",
        ex.what());
    }

    planner_ids_concat_ += planner_id_ + std::string(" ");

    RCLCPP_INFO(
      get_logger(),
      "Planner Server has %s planners available.", planner_ids_concat_.c_str());
    if (expected_planner_frequency_ > 0) {
      max_planner_duration_ = 1 / expected_planner_frequency_;
    } else {
      RCLCPP_WARN(
        get_logger(),
        "The expected planner frequency parameter is %.4f Hz. The value should to be greater"
        " than 0.0 to turn on duration overrrun warning messages", expected_planner_frequency_);
      max_planner_duration_ = 0.0;
    }

    // Initialize pubs & subs
    plan_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("plan", 1);

    this->action_server_ = rclcpp_action::create_server<ComputePathToPose>(
      this->get_node_base_interface(),
      this->get_node_clock_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "compute_path_to_pose",
      std::bind(&PlannerServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&PlannerServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&PlannerServer::handle_accepted, this, std::placeholders::_1));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  PlannerServer::~PlannerServer()
  {
    RCLCPP_INFO(get_logger(), "Destroying");
    planners_.clear();
    action_server_.reset();
    plan_publisher_.reset();
    RCLCPP_INFO(get_logger(), "Shutting down");
  }

  rclcpp_action::GoalResponse PlannerServer::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ComputePathToPose::Goal> goal)
  {
    RCLCPP_INFO(
      this->get_logger(), "Received goal request in order to compute a path to pose");
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse PlannerServer::handle_cancel(
    const std::shared_ptr<GoalHandleComputePathToPose> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void PlannerServer::handle_accepted(
    const std::shared_ptr<GoalHandleComputePathToPose> goal_handle)
  {
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&PlannerServer::computePlan, this, std::placeholders::_1),
      goal_handle}.detach();
  }

  void
  PlannerServer::computePlan(const std::shared_ptr<GoalHandleComputePathToPose> goal_handle)
  {
    auto start_time = steady_clock_.now();
    rclcpp::Rate loop_rate(expected_planner_frequency_);

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<ComputePathToPose::Feedback>();
    auto result = std::make_shared<ComputePathToPose::Result>();

    RCLCPP_INFO(
      this->get_logger(), "Received a planning request to (%.3f, %.3f)",
      goal->pose.pose.position.x, goal->pose.pose.position.y);

    geometry_msgs::msg::PoseStamped start_pose, goal_pose;
    vox_nav_utilities::getCurrentPose(start_pose, *tf_buffer_, "map", "base_link", 0.1);
    goal_pose = goal->pose;

    result->path.poses = getPlan(start_pose, goal_pose, planner_id_);

    if (result->path.poses.size() == 0) {
      RCLCPP_WARN(
        get_logger(), "Planning algorithm %s failed to generate a valid",
        goal->planner_id.c_str());
      return;
    }

    // Check if there is a cancel request
    if (goal_handle->is_canceling()) {
      result->path.poses = std::vector<geometry_msgs::msg::PoseStamped>();
      goal_handle->canceled(result);
      RCLCPP_INFO(get_logger(), "Goal was canceled. Canceling planning action.");
      return;
    }
    // Update sequence
    auto elapsed_time = steady_clock_.now() - start_time;
    feedback->elapsed_time = elapsed_time;
    goal_handle->publish_feedback(feedback);

    RCLCPP_DEBUG(
      get_logger(),
      "Found valid path of size %u to (%.2f, %.2f)",
      result->path.poses.size(), goal_pose.pose.position.x,
      goal_pose.pose.position.y);

    auto cycle_duration = steady_clock_.now() - start_time;

    // Check if goal is done
    if (rclcpp::ok()) {
      cycle_duration = steady_clock_.now() - start_time;
      result->planning_time = cycle_duration;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
      // Publish the plan for visualization purposes
      if (planners_.find(planner_id_) != planners_.end()) {
        auto overlayed_start_goal = planners_[planner_id_]->getOverlayedStartandGoal();
        if (overlayed_start_goal.size() == 2) {
          start_pose = overlayed_start_goal.front();
          goal_pose = overlayed_start_goal.back();
        }
      }
      publishPlan(result->path.poses, start_pose, goal_pose);
    }
    cycle_duration = steady_clock_.now() - start_time;
    if (max_planner_duration_ && cycle_duration.seconds() > max_planner_duration_) {
      RCLCPP_WARN(
        get_logger(),
        "Planner loop missed its desired rate of %.4f Hz. Current loop rate is %.4f Hz",
        1 / max_planner_duration_, 1 / cycle_duration.seconds());
    }
    loop_rate.sleep();
  }

  std::vector<geometry_msgs::msg::PoseStamped>
  PlannerServer::getPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const std::string & planner_id)
  {
    if (planners_.find(planner_id) != planners_.end()) {
      std::vector<geometry_msgs::msg::PoseStamped> plan =
        planners_[planner_id]->createPlan(start, goal);
      return plan;
    } else {
      RCLCPP_ERROR(
        get_logger(), "planner %s is not a valid planner. "
        "Planner names are: %s", planner_id.c_str(),
        planner_ids_concat_.c_str());
    }
    RCLCPP_ERROR(get_logger(), "planner could not find a valid plan");
    // if we are here, get plan was not succeeded , return an ampty path
    return std::vector<geometry_msgs::msg::PoseStamped>();
  }

  void PlannerServer::publishPlan(
    const std::vector<geometry_msgs::msg::PoseStamped> & path,
    const geometry_msgs::msg::PoseStamped & start_pose,
    const geometry_msgs::msg::PoseStamped & goal_pose)
  {
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker start_marker, goal_marker;

    auto path_idx = 0;
    for (auto && i : path) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = rclcpp::Clock().now();
      marker.ns = "path";
      marker.id = path_idx;
      if (!robot_mesh_path_.empty()) {
        marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
        marker.mesh_resource = robot_mesh_path_;
      } else {
        marker.type = visualization_msgs::msg::Marker::CUBE;
      }
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.lifetime = rclcpp::Duration::from_seconds(0);
      marker.pose = i.pose;
      marker.scale.x = get_parameter("robot_body_dimens.x").as_double();
      marker.scale.y = get_parameter("robot_body_dimens.y").as_double();
      marker.scale.z = get_parameter("robot_body_dimens.z").as_double();
      marker.color.a = 0.4;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 1.0;
      marker_array.markers.push_back(marker);
      path_idx++;
      start_marker = marker;
      goal_marker = marker;

    }
    // Publish goal and start states for debuging
    start_marker.pose = start_pose.pose;
    start_marker.color.b = 0;
    goal_marker.pose = goal_pose.pose;
    goal_marker.color.b = 0;
    marker_array.markers.push_back(start_marker);
    marker_array.markers.push_back(goal_marker);
    plan_publisher_->publish(marker_array);
  }
}  // namespace vox_nav_planning

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vox_nav_planning::PlannerServer>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
