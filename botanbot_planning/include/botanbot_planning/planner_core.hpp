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


#ifndef BOTANBOT_PLANNING__PLANNER_CORE_HPP_
#define BOTANBOT_PLANNING__PLANNER_CORE_HPP_
#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace botanbot_planning
{
/**
 * @brief Base class for creating a plugin in order to perform a specific task at waypoint arrivals.
 *
 */
class PlannerCore
{
public:
  using Ptr = std::shared_ptr<PlannerCore>;
  /**
   * @brief Construct a new Simple Task Execution At Waypoint Base object
   *
   */
  PlannerCore() {}

  /**
   * @brief Destroy the Simple Task Execution At Waypoint Base object
   *
   */
  virtual ~PlannerCore() {}

  /**
   * @brief Override this to setup your pub, sub or any ros services that you will use in the plugin.
   *
   * @param parent parent node that plugin will be created within(for an example see nav_waypoint_follower)
   * @param plugin_name plugin name comes from parameters in yaml file
   */
  virtual void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name) = 0;

  /**
   * @brief Method create the plan from a starting and ending goal.
   *
   * @param start The starting pose of the robot
   * @param goal  The goal pose of the robot
   * @return std::vector<geometry_msgs::msg::PoseStamped>   The sequence of poses to get from start to goal, if any
   */
  virtual std::vector<geometry_msgs::msg::PoseStamped> createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) = 0;
};
}  // namespace botanbot_planning
#endif  // BOTANBOT_PLANNING__PLANNER_CORE_HPP_
