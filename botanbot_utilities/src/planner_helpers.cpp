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

#include <memory>
#include <string>
#include "botanbot_utilities/planner_helpers.hpp"

namespace botanbot_utilities
{

geometry_msgs::msg::PoseStamped getNearstNode(
  const geometry_msgs::msg::PoseStamped & state,
  const std::shared_ptr<octomap::ColorOcTree> & color_octomap_octree)
{
  auto nearest_node_pose = state;
  double dist = INFINITY;
  for (auto it = color_octomap_octree->begin(),
    end = color_octomap_octree->end(); it != end; ++it)
  {
    if (color_octomap_octree->isNodeOccupied(*it)) {
      auto dist_to_crr_node = std::sqrt(
        std::pow(it.getCoordinate().x() - state.pose.position.x, 2) +
        std::pow(it.getCoordinate().y() - state.pose.position.y, 2) +
        std::pow(it.getCoordinate().z() - state.pose.position.z, 2));
      if (dist_to_crr_node < dist) {
        dist = dist_to_crr_node;
        nearest_node_pose.pose.position.x = it.getCoordinate().x();
        nearest_node_pose.pose.position.y = it.getCoordinate().y();
        nearest_node_pose.pose.position.z = it.getCoordinate().z();
      }
    }
  }
  return nearest_node_pose;
}

void initializeSelectedPlanner(
  ompl::base::PlannerPtr & planner,
  const std::string & selected_planner_name,
  const ompl::base::SpaceInformationPtr & si,
  const rclcpp::Logger logger)
{
  if (selected_planner_name == std::string("PRMStar")) {
    planner = ompl::base::PlannerPtr(new ompl::geometric::PRMstar(si));
  } else if (selected_planner_name == std::string("RRTstar")) {
    planner = ompl::base::PlannerPtr(new ompl::geometric::RRTstar(si));
  } else if (selected_planner_name == std::string("RRTConnect")) {
    planner = ompl::base::PlannerPtr(new ompl::geometric::RRTConnect(si));
  } else if (selected_planner_name == std::string("KPIECE1")) {
    planner = ompl::base::PlannerPtr(new ompl::geometric::KPIECE1(si));
  } else if (selected_planner_name == std::string("SBL")) {
    planner = ompl::base::PlannerPtr(new ompl::geometric::SBL(si));
  } else if (selected_planner_name == std::string("SST")) {
    planner = ompl::base::PlannerPtr(new ompl::geometric::SST(si));
  } else {
    RCLCPP_WARN(
      logger,
      "Selected planner is not Found in available planners, using the default planner: KPIECE1");
    planner = ompl::base::PlannerPtr(new ompl::geometric::KPIECE1(si));
  }
}

}  // namespace botanbot_utilities
