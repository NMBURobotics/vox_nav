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

#ifndef VOX_NAV_PLANNING__PLUGINS__SE3_PLANNER_HPP_
#define VOX_NAV_PLANNING__PLUGINS__SE3_PLANNER_HPP_

#include <vector>
#include <string>
#include <memory>


#include "vox_nav_planning/planner_core.hpp"
#include "vox_nav_planning/plugins/se3_planner_utils.hpp"


namespace vox_nav_planning
{

class SE3Planner : public vox_nav_planning::PlannerCore
{

public:
/**
 * @brief Construct a new SE3Planner object
 *
 */
  SE3Planner();

/**
 * @brief Destroy the SE3Planner object
 *
 */
  ~SE3Planner();

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
  * @brief Callback to subscribe ang get octomap
  *
  * @param octomap
  */
  virtual void octomapCallback(const octomap_msgs::msg::Octomap::ConstSharedPtr msg) override;

  /**
   * @brief
   *
   * @param si
   * @return ompl::base::ValidStateSamplerPtr
   */
  ompl::base::ValidStateSamplerPtr allocValidStateSampler(const ompl::base::SpaceInformation * si);

  /**
   * @brief Get the Opt Objective object
   *
   * @return ompl::base::OptimizationObjectivePtr
   */
  ompl::base::OptimizationObjectivePtr getOptimizationObjective();

protected:
  rclcpp::Logger logger_{rclcpp::get_logger("se3_planner")};
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_subscriber_;
  octomap_msgs::msg::Octomap::ConstSharedPtr octomap_msg_;

  std::shared_ptr<fcl::CollisionObject> robot_collision_object_;
  std::shared_ptr<fcl::OcTree> fcl_octree_;
  std::shared_ptr<fcl::CollisionObject> fcl_octree_collision_object_;
  std::shared_ptr<octomap::ColorOcTree> color_octomap_octree_;
  std::shared_ptr<octomap::OcTree> octomap_octree_;
  std::shared_ptr<ompl::base::RealVectorBounds> state_space_bounds_;
  std::shared_ptr<OctoCellValidStateSampler> octocell_state_sampler_;

  ompl::base::ScopedState<ompl::base::SE3StateSpace> * start_;
  ompl::base::ScopedState<ompl::base::SE3StateSpace> * goal_;
  ompl::base::StateSpacePtr state_space_;
  ompl::base::OptimizationObjectivePtr octocost_optimization_;
  ompl::geometric::SimpleSetupPtr simple_setup_;

  // to ensure safety when accessing global var curr_frame_
  std::mutex global_mutex_;
  // the topic to subscribe in order capture a frame
  std::string planner_name_;
  // The topic of octomap to subscribe, this octomap is published by map_server
  std::string octomap_topic_;
  // Better t keep this parameter consistent with map_server, 0.2 is a OK default fo this
  double octomap_voxel_size_;
  // whether plugin is enabled
  bool is_enabled_;
  // related to density of created path
  int interpolation_parameter_;
  // max time the planner can spend before coming up with a solution
  double planner_timeout_;
  // whether otomap is recieved
  volatile bool is_octomap_ready_;
  // global mutex to guard octomap
  std::mutex octomap_mutex_;
};
}  // namespace vox_nav_planning

#endif  // VOX_NAV_PLANNING__PLUGINS__SE3_PLANNER_HPP_
