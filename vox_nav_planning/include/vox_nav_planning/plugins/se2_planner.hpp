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

#ifndef VOX_NAV_PLANNING__PLUGINS__SE2_PLANNER_HPP_
#define VOX_NAV_PLANNING__PLUGINS__SE2_PLANNER_HPP_

#include <vector>
#include <string>
#include <memory>

#include "vox_nav_planning/planner_core.hpp"
/**
 * @brief
 *
 */
namespace vox_nav_planning
{

/**
 * @brief
 *
 */
class SE2Planner : public vox_nav_planning::PlannerCore
{
public:
/**
 * @brief Construct a new vox_nav O M P L Experimental object
 *
 */
  SE2Planner();

  /**
  * @brief Destroy the vox_nav O M P L Experimental object
  *
  */
  ~SE2Planner();

  /**
   * @brief
   *
   */
  void initialize(
    rclcpp::Node * parent,
    const std::string & plugin_name) override;

  /**
   * @brief Method create the plan from a starting and ending goal.
   *
   * @param start The starting pose of the robot
   * @param goal  The goal pose of the robot
   * @return std::vector<geometry_msgs::msg::PoseStamped>   The sequence of poses to get from start to goal, if any
   */
  std::vector<geometry_msgs::msg::PoseStamped> createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

  /**
  * @brief
  *
  * @param state
  * @return true
  * @return false
  */
  bool isStateValid(const ompl::base::State * state) override;

  /**
  * @brief Get the Overlayed Start and Goal poses, only x and y are provided for goal ,
  * but internally planner finds closest valid node on octomap and reassigns goal to this pose
  *
  * @return std::vector<geometry_msgs::msg::PoseStamped>
  */
  std::vector<geometry_msgs::msg::PoseStamped> getOverlayedStartandGoal() override;

  /**
   * @brief
   *
   */
  void setupMap() override;

protected:
  rclcpp::Logger logger_{rclcpp::get_logger("se2_planner")};
  // Which state space is slected ? REEDS,DUBINS, SE2
  std::string selected_se2_space_name_;
};
}  // namespace vox_nav_planning

#endif  // VOX_NAV_PLANNING__PLUGINS__SE2_PLANNER_HPP_
