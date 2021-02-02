// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2019 Samsung Research America
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
#include "botanbot_control/controller_server.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <utility>

namespace botanbot_control
{
ControllerServer::ControllerServer()
: Node("botanbot_controller_server_rclcpp_node"),
  pc_loader_("botanbot_control", "botanbot_control::ControlerCore"),
  controller_id_("MPCControllerROS"),
  controller_type_("mpc_controller::MPCControllerROS")
{
  RCLCPP_INFO(get_logger(), "Creating");

  // Declare this node's parameters
  declare_parameter("controller_frequency", controller_frequency_);
  get_parameter("controller_frequency", controller_frequency_);

  declare_parameter("controller_plugin", controller_id_);
  get_parameter("controller_plugin", controller_id_);

  declare_parameter(controller_id_ + ".plugin", controller_type_);
  get_parameter(controller_id_ + ".plugin", controller_type_);

  try {
    botanbot_control::ControllerCore::Ptr controller =
      pc_loader_.createSharedInstance(controller_type_);
    controller->initialize(this, controller_id_);
    RCLCPP_INFO(
      get_logger(), "Created controller plugin %s of type %s",
      controller_id_.c_str(), controller_type_.c_str());
    controllers_.insert({controller_id_, controller});
  } catch (const pluginlib::PluginlibException & ex) {
    RCLCPP_FATAL(
      get_logger(), "Failed to create planner. Exception: %s",
      ex.what());
  }

  controller_ids_concat_ += controller_id_ + std::string(" ");

  RCLCPP_INFO(
    get_logger(),
    "Controller Server has %s planners available.", controller_ids_concat_.c_str());
  if (controller_frequency_ > 0) {
    controller_duration_ = 1.0 / controller_frequency_;
  } else {
    RCLCPP_WARN(
      get_logger(),
      "The expected planner frequency parameter is %.4f Hz. The value should to be greater"
      " than 0.0 to turn on duration overrrun warning messages", controller_frequency_);
    controller_duration_ = 0.0;
  }

  // Initialize pubs & subs
  actual_control_states_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "control", 1);

  this->action_server_ = rclcpp_action::create_server<FollowPath>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "follow_path",
    std::bind(&ControllerServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&ControllerServer::handle_cancel, this, std::placeholders::_1),
    std::bind(&ControllerServer::handle_accepted, this, std::placeholders::_1));
}

ControllerServer::~ControllerServer()
{
  RCLCPP_INFO(get_logger(), "Destroying");
  controllers_.clear();
  action_server_.reset();
  actual_control_states_publisher_.reset();
  RCLCPP_INFO(get_logger(), "Shutting down");
}

rclcpp_action::GoalResponse ControllerServer::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const FollowPath::Goal> goal)
{
  RCLCPP_INFO(
    this->get_logger(), "Received goal request in order to compute a path to pose");
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

  geometry_msgs::msg::Twist computed_velcity_commands;
  // set Plan
  if (controllers_.find(controller_id_) != controllers_.end()) {
    controllers_[controller_id_]->setPlan(goal->path);
    computed_velcity_commands = controllers_[controller_id_]->computeVelocityCommands();
  } else {
    if (controllers_.size() == 1 && controller_id_.empty()) {
      RCLCPP_WARN_ONCE(
        get_logger(), "No Controllers specified in action call. "
        "Server will use only plugin %s in server."
        " This warning will appear once.", controller_ids_concat_.c_str());
      controllers_[controllers_.begin()->first]->setPlan(goal->path);
      controllers_[controller_id_]->computeVelocityCommands();
    } else {
      RCLCPP_ERROR(
        get_logger(), "controller %s is not a valid controller. "
        "Controller names are: %s", controller_id_.c_str(),
        controller_ids_concat_.c_str());
    }
  }

  // Check if there is a cancel request
  if (goal_handle->is_canceling()) {
    goal_handle->canceled(result);
    RCLCPP_INFO(get_logger(), "Goal was canceled. Canceling planning action.");
    return;
  }
  // Update sequence
  auto elapsed_time = steady_clock_.now() - start_time;
  feedback->elapsed_time = elapsed_time;
  feedback->speed = computed_velcity_commands.linear.x;
  goal_handle->publish_feedback(feedback);

  RCLCPP_DEBUG(
    get_logger(),
    "Giving velocity commands to robot linear %.2f, angular %.2f)",
    computed_velcity_commands.linear.x,
    computed_velcity_commands.angular.z);

  auto cycle_duration = steady_clock_.now() - start_time;

  // Check if goal is done
  if (rclcpp::ok()) {
    cycle_duration = steady_clock_.now() - start_time;
    result->total_time = cycle_duration;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Follow Path Succeeded");
    // Publish the plan for visualization purposes
    //publishPlan(result->path);
  }
  cycle_duration = steady_clock_.now() - start_time;
  if (controller_duration_ && cycle_duration.seconds() > controller_duration_) {
    RCLCPP_WARN(
      get_logger(),
      "Controller loop missed its desired rate of %.4f Hz. Current loop rate is %.4f Hz",
      1 / controller_duration_, 1 / cycle_duration.seconds());
  }
  loop_rate.sleep();
}

}  // namespace botanbot_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<botanbot_control::ControllerServer>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
