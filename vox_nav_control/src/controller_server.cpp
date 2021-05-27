// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2019 Samsung Research America
// Copyright (c) 2021 Norwegian University of Life Sciences, Fetullah Atas
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

#include "builtin_interfaces/msg/duration.hpp"
#include "vox_nav_control/controller_server.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <utility>

namespace vox_nav_control
{
ControllerServer::ControllerServer()
: Node("vox_nav_controller_server_rclcpp_node"),
  pc_loader_("vox_nav_control", "vox_nav_control::ControllerCore"),
  controller_id_("MPCControllerROS"),
  controller_type_("mpc_controller::MPCControllerROS")
{
  RCLCPP_INFO(get_logger(), "Creating");

  // Declare this node's parameters
  declare_parameter("controller_frequency", 5.0);
  get_parameter("controller_frequency", controller_frequency_);

  declare_parameter("controller_plugin", controller_id_);
  get_parameter("controller_plugin", controller_id_);

  declare_parameter(controller_id_ + ".plugin", controller_type_);
  get_parameter(controller_id_ + ".plugin", controller_type_);

  try {
    controller_ =
      pc_loader_.createSharedInstance(controller_type_);
    controller_->initialize(this, controller_id_);
    RCLCPP_INFO(
      get_logger(), "Created controller plugin %s of type %s",
      controller_id_.c_str(), controller_type_.c_str());
  } catch (const pluginlib::PluginlibException & ex) {
    RCLCPP_FATAL(
      get_logger(), "Failed to create controller. Exception: %s",
      ex.what());
  }

  RCLCPP_INFO(
    get_logger(),
    "Selected controller id: %s", controller_id_.c_str());
  if (controller_frequency_ > 0) {
    controller_duration_ = 1.0 / controller_frequency_;
  } else {
    RCLCPP_WARN(
      get_logger(),
      "The expected controller frequency parameter is %.4f Hz. The value should to be greater"
      " than 0.0 to turn on duration overrrun warning messages", controller_frequency_);
    controller_duration_ = 0.0;
  }

  this->action_server_ = rclcpp_action::create_server<FollowPath>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "follow_path",
    std::bind(&ControllerServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&ControllerServer::handle_cancel, this, std::placeholders::_1),
    std::bind(&ControllerServer::handle_accepted, this, std::placeholders::_1));

  // setup TF buffer and listerner to read transforms
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  cmd_vel_publisher_ =
    this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
}

ControllerServer::~ControllerServer()
{
  RCLCPP_INFO(get_logger(), "Destroying");
  controller_.reset();
  action_server_.reset();
}

rclcpp_action::GoalResponse ControllerServer::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const FollowPath::Goal> goal)
{
  RCLCPP_INFO(
    this->get_logger(), "Received goal request in order to follow a path");
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ControllerServer::handle_cancel(
  const std::shared_ptr<GoalHandleFollowPath> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ControllerServer::handle_accepted(
  const std::shared_ptr<GoalHandleFollowPath> goal_handle)
{
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&ControllerServer::followPath, this, std::placeholders::_1),
    goal_handle}.detach();
}

void
ControllerServer::followPath(const std::shared_ptr<GoalHandleFollowPath> goal_handle)
{
  auto start_time = steady_clock_.now();
  rclcpp::Rate loop_rate(controller_frequency_);

  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<FollowPath::Feedback>();
  auto result = std::make_shared<FollowPath::Result>();

  if (!goal->path.poses.size() > 0) {
    RCLCPP_WARN(
      get_logger(), "Recieved an empty path, this was probaly unintended, ignoring the request.");
    if (rclcpp::ok()) {
      auto cycle_duration = steady_clock_.now() - start_time;
      result->total_time = cycle_duration;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Follow Path Succeeded!");
      return;
    }
  }

  geometry_msgs::msg::Twist computed_velocity_commands;
  // set Plan
  controller_->setPlan(goal->path);

  rclcpp::WallRate rate(controller_frequency_);
  double goal_tolerance_distance = 0.7;
  volatile bool is_goal_tolerance_satisfied = false;

  while (rclcpp::ok() && !is_goal_tolerance_satisfied) {
    auto loop_start_time = steady_clock_.now();
    geometry_msgs::msg::PoseStamped curr_robot_pose;
    vox_nav_utilities::getCurrentPose(
      curr_robot_pose, *tf_buffer_, "map", "base_link", 0.1);
    // Check if there is a cancel request
    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
      RCLCPP_INFO(get_logger(), "Goal was canceled. Canceling planning action.");
      cmd_vel_publisher_->publish(geometry_msgs::msg::Twist());
      return;
    }

    // check if we have arrived to goal, note the goal is last pose of path
    if (vox_nav_utilities::getEuclidianDistBetweenPoses(
        curr_robot_pose,
        goal->path.poses.back()) < goal_tolerance_distance)
    {
      // goal has been reached
      is_goal_tolerance_satisfied = true;
      // reset the velocity
      cmd_vel_publisher_->publish(geometry_msgs::msg::Twist());
      RCLCPP_INFO(this->get_logger(), "Goal has been reached");
      break;
    }

    computed_velocity_commands = controller_->computeVelocityCommands(curr_robot_pose);
    // Update sequence
    auto elapsed_time = steady_clock_.now() - start_time;
    feedback->elapsed_time = elapsed_time;
    feedback->speed = computed_velocity_commands.linear.x;
    goal_handle->publish_feedback(feedback);
    cmd_vel_publisher_->publish(computed_velocity_commands);
    // If loop is slower than expected , notify
    auto cycle_duration = steady_clock_.now() - loop_start_time;
    if (controller_duration_ && cycle_duration.seconds() > controller_duration_) {
      RCLCPP_WARN(
        get_logger(),
        "Contol loop missed its desired rate of %.4f Hz. Current loop rate is %.4f Hz",
        1 / controller_duration_, 1 / cycle_duration.seconds());
    }
    rate.sleep();
  }
  auto cycle_duration = steady_clock_.now() - start_time;
  // Check if goal is done
  if (rclcpp::ok()) {
    cycle_duration = steady_clock_.now() - start_time;
    result->total_time = cycle_duration;
    goal_handle->succeed(result);
    cmd_vel_publisher_->publish(geometry_msgs::msg::Twist());
    RCLCPP_INFO(this->get_logger(), "Follow Path Succeeded!");
  }
}
}  // namespace vox_nav_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vox_nav_control::ControllerServer>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
