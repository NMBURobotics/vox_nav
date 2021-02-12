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

// ROS
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <botanbot_utilities/tf_helpers.hpp>

// PCL
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
// OMPL
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/sst/SST.h>
#include <ompl/control/planners/est/EST.h>
#include <ompl/control/planners/syclop/SyclopRRT.h>
#include <ompl/control/planners/syclop/SyclopEST.h>
#include <ompl/control/planners/pdst/PDST.h>
#include <ompl/control/planners/syclop/GridDecomposition.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/samplers/MaximizeClearanceValidStateSampler.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/cforest/CForest.h>
// OCTOMAP
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <octomap/octomap_utils.h>
// FCL
#include <fcl/config.h>
#include <fcl/octree.h>
#include <fcl/traversal/traversal_node_octree.h>
#include <fcl/collision.h>
#include <fcl/broadphase/broadphase.h>
#include <fcl/math/transform.h>

// STL
#include <string>
#include <iostream>
#include <memory>
#include <vector>

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
   * @brief
   *
   * @param parent
   * @param plugin_name
   */
  virtual void initialize(
    rclcpp::Node * parent,
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

  /**
  * @brief
  *
  * @param state
  * @return true
  * @return false
  */
  virtual bool isStateValid(const ompl::base::State * state) = 0;

/**
 * @brief Get the Selected Planner object
 *
 * @param planner_name
 * @param state_space_information
 * @param planner
 * @return true
 * @return false
 */
  virtual bool getSelectedPlanner(
    const std::string & planner_name,
    const ompl::base::SpaceInformationPtr & state_space_information,
    ompl::base::PlannerPtr planner) = 0;

  /**
   * @brief Callback to subscribe ang get octomap
   *
   * @param octomap
   */
  virtual void octomapCallback(const octomap_msgs::msg::Octomap::ConstSharedPtr msg) = 0;
};
}  // namespace botanbot_planning
#endif  // BOTANBOT_PLANNING__PLANNER_CORE_HPP_
