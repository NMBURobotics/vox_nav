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

#ifndef VOX_NAV_PLANNING__PLUGINS__SE3_PLANNER_UTILS_HPP_
#define VOX_NAV_PLANNING__PLUGINS__SE3_PLANNER_UTILS_HPP_

#include "vox_nav_planning/planner_core.hpp"
#include <pcl/octree/octree_search.h>
#include <pcl/filters/random_sample.h>
#include <vox_nav_utilities/tf_helpers.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

/**
 * @brief
 *
 */
namespace vox_nav_planning
{

class OctoCostOptimizationObjective : public ompl::base::StateCostIntegralObjective
{
public:
  /**
   * @brief Construct a new Octo Cost Optimization Objective object
   *
   * @param si
   * @param tree
  */
  OctoCostOptimizationObjective(
    const ompl::base::SpaceInformationPtr & si,
    const std::shared_ptr<octomap::OcTree> & elevated_surfels_octree);

  /**
   * @brief Destroy the Octo Cost Optimization Objective object
   *
   */
  ~OctoCostOptimizationObjective();

  /**
   * @brief get amount of the cost assigned to this state
   *
   * @param s
   * @return ompl::base::Cost
   */
  ompl::base::Cost stateCost(const ompl::base::State * s) const override;

protected:
  std::shared_ptr<octomap::OcTree> elevated_surfels_octree_;
  rclcpp::Logger logger_{rclcpp::get_logger("se3_planner_utils")};
};


class OctoCellValidStateSampler : public ompl::base::ValidStateSampler
{
public:
  /**
   * @brief Construct a new Octo Cell State Sampler object
   *
   * @param si
   * @param tree
   */
  OctoCellValidStateSampler(
    const ompl::base::SpaceInformationPtr & si,
    const geometry_msgs::msg::PoseStamped start,
    const geometry_msgs::msg::PoseStamped goal,
    const std::shared_ptr<fcl::CollisionObject> & robot_collision_object,
    const std::shared_ptr<fcl::CollisionObject> & original_octomap_collision_object,
    const geometry_msgs::msg::PoseArray::SharedPtr & elevated_surfels_poses);

  /**
   * @brief
   *
   * @param state
   * @return true
   * @return false
   */
  bool sample(ompl::base::State * state) override;

  /**
   * @brief
   *
   * @param state
   * @param near
   * @param distance
   * @return true
   * @return false
   */
  bool sampleNear(
    ompl::base::State * state,
    const ompl::base::State * near,
    const double distance) override;

  /**
   * @brief
   *
   * @param start
   * @param goal
   */
  void updateSearchArea(
    const geometry_msgs::msg::PoseStamped start,
    const geometry_msgs::msg::PoseStamped goal);

  bool  isStateValid(const ompl::base::State * state);

protected:
  rclcpp::Logger logger_{rclcpp::get_logger("se3_planner_utils")};
  geometry_msgs::msg::PoseArray elevated_surfels_poses_;
  pcl::PointCloud<pcl::PointSurfel>::Ptr workspace_surfels_;
  pcl::PointCloud<pcl::PointSurfel>::Ptr search_area_surfels_;
  std::shared_ptr<fcl::CollisionObject> robot_collision_object_;
  std::shared_ptr<fcl::CollisionObject> original_octomap_collision_object_;
};
}  // namespace vox_nav_planning

#endif  // VOX_NAV_PLANNING__PLUGINS__SE3_PLANNER_UTILS_HPP_
