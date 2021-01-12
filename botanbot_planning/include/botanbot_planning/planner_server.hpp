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

#ifndef BOTANBOT_PLANNING__PLANNER_SERVER_HPP_
#define BOTANBOT_PLANNING__PLANNER_SERVER_HPP_


#include <chrono>
#include <string>
#include <memory>
#include <vector>
#include <unordered_map>

#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "botanbot_msgs/action/compute_path_to_pose.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "botanbot_planning/planner_core.hpp"

namespace botanbot_planning
{
class PlannerServer : public rclcpp::Node
{
public:
  using ComputePathToPose = botanbot_msgs::action::ComputePathToPose;
  using GoalHandleComputePathToPose = rclcpp_action::ServerGoalHandle<ComputePathToPose>;
  /**
   * @brief Construct a new Planner Server object
   *
   */
  PlannerServer();

  /**
   * @brief Destroy the Planner Server object
   *
   */
  ~PlannerServer();

  using PlannerMap = std::unordered_map<std::string, botanbot_planning::PlannerCore::Ptr>;

  /**
   * @brief Method to get plan from the desired plugin
   * @param start starting pose
   * @param goal goal request
   * @return Path
   */
  std::vector<geometry_msgs::msg::PoseStamped> getPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const std::string & planner_id);

  /**
   * @brief
   *
   * @param uuid
   * @param goal
   * @return rclcpp_action::GoalResponse
   */
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ComputePathToPose::Goal> goal);

  /**
   * @brief
   *
   * @param goal_handle
   * @return rclcpp_action::CancelResponse
   */
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleComputePathToPose> goal_handle);

  /**
   * @brief
   *
   * @param goal_handle
   */
  void handle_accepted(const std::shared_ptr<GoalHandleComputePathToPose> goal_handle);

protected:
  // Our action server implements the ComputePathToPose action
  rclcpp_action::Server<ComputePathToPose>::SharedPtr action_server_;

  /**
   * @brief The action server callback which calls planner to get the path
   */
  void computePlan(const std::shared_ptr<GoalHandleComputePathToPose> goal_handle);

  /**
   * @brief Publish a path for visualization purposes
   * @param path Reference to Global Path
   */
  void publishPlan(const std::vector<geometry_msgs::msg::PoseStamped> & path);

  // Planner
  PlannerMap planners_;
  pluginlib::ClassLoader<botanbot_planning::PlannerCore> pc_loader_;
  std::vector<std::string> default_ids_;
  std::vector<std::string> default_types_;
  std::vector<std::string> planner_ids_;
  std::vector<std::string> planner_types_;
  double max_planner_duration_;
  std::string planner_ids_concat_;

  // Clock
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  // TF buffer
  std::shared_ptr<tf2_ros::Buffer> tf_;

  // Publishers for the path
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr plan_publisher_;
};

}  // namespace botanbot_planning

#endif  // BOTANBOT_PLANNING__PLANNER_SERVER_HPP_
