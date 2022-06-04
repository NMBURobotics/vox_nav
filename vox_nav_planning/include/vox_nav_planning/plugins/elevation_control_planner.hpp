// Copyright (c) 2022 Fetullah Atas, Norwegian University of Life Sciences
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

#ifndef VOX_NAV_PLANNING__PLUGINS__ELEVATION_CONTROL_PLANNER_HPP_
#define VOX_NAV_PLANNING__PLUGINS__ELEVATION_CONTROL_PLANNER_HPP_

#include <vector>
#include <string>
#include <memory>

#include "vox_nav_planning/planner_core.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "vox_nav_utilities/elevation_state_space.hpp"
#include "ompl/control/spaces/RealVectorControlSpace.h"
#include "ompl/control/SimpleSetup.h"
#include "ompl/control/planners/sst/SST.h"
#include "ompl/control/planners/rrt/RRT.h"

namespace vox_nav_planning
{

  class ElevationControlPlanner : public vox_nav_planning::PlannerCore
  {

  public:
/**
 * @brief Construct a new ElevationControlPlanner object
 *
 */
    ElevationControlPlanner();

/**
 * @brief Destroy the ElevationControlPlanner object
 *
 */
    ~ElevationControlPlanner();

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
     * @brief propogates the states to next phase, given the control input
     *
     * @param start
     * @param control
     * @param duration
     * @param result
     */
    void propagate(
      const ompl::control::SpaceInformation * si,
      const ompl::base::State * start,
      const ompl::control::Control * control,
      const double duration,
      ompl::base::State * result);

    /**
    * @brief
    *
    * @param state
    * @return true
    * @return false
    */
    bool isStateValid(const ompl::base::State * state) override;

    /**
     * @brief
     *
     * @param msg
     */
    void nodePosesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);

    /**
     * @brief Get the Opt Objective object
     *
     * @return ompl::base::OptimizationObjectivePtr
     */
    ompl::base::OptimizationObjectivePtr getOptimizationObjective();

    /**
     * @brief
     *
     * @param si
     * @return ompl::base::ValidStateSamplerPtr
     */
    ompl::base::ValidStateSamplerPtr allocValidStateSampler(
      const ompl::base::SpaceInformation * si);

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
    rclcpp::Logger logger_{rclcpp::get_logger("elevation_control_planner")};
    // Surfels centers are elevated by node_elevation_distance_, and are stored in this
    // octomap, this maps is used by planner to sample states that are
    // strictly laying on ground but not touching. So it constrains the path to be on ground
    // while it can elevate thorogh ramps or slopes
    std::shared_ptr<octomap::OcTree> elevated_surfel_octomap_octree_;
    // it is also required to have orientation information of surfels, they are kept in
    // elevated_surfel_poses_msg_
    geometry_msgs::msg::PoseArray::SharedPtr elevated_surfel_poses_msg_;
    pcl::PointCloud<pcl::PointSurfel>::Ptr elevated_surfel_cloud_;
    geometry_msgs::msg::PoseStamped nearest_elevated_surfel_to_start_;
    geometry_msgs::msg::PoseStamped nearest_elevated_surfel_to_goal_;
    std::shared_ptr<fcl::CollisionObject> elevated_surfels_collision_object_;
    ompl::base::OptimizationObjectivePtr octocost_optimization_;
    ompl::base::StateSpacePtr state_space_;
    ompl::control::ControlSpacePtr control_state_space_;
    ompl::control::SimpleSetupPtr control_simple_setup_;

    std::shared_ptr<ompl::base::RealVectorBounds> z_bounds_;
    std::shared_ptr<ompl::base::RealVectorBounds> se2_bounds_;

    std::string selected_se2_space_name_;
    ompl::base::ElevationStateSpace::SE2StateType se2_space_type_;
    // curve radius for reeds and dubins only
    double rho_;
  };
}  // namespace vox_nav_planning

#endif  // VOX_NAV_PLANNING__PLUGINS__ELEVATION_CONTROL_PLANNER_HPP_
