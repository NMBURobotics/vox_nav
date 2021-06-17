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

#ifndef VOX_NAV_PLANNING__PLUGINS__DUBINS_3D_PLANNER_HPP_
#define VOX_NAV_PLANNING__PLUGINS__DUBINS_3D_PLANNER_HPP_

#include <vector>
#include <string>
#include <memory>

#include "vox_nav_planning/planner_core.hpp"
#include "vox_nav_planning/plugins/se3_planner_utils.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "vox_nav_planning/plugins/dubins_3d_state_space.hpp"


namespace vox_nav_planning
{

  class Dubins3DPlanner : public vox_nav_planning::PlannerCore
  {

  public:
/**
 * @brief Construct a new Dubins3DPlanner object
 *
 */
    Dubins3DPlanner();

/**
 * @brief Destroy the Dubins3DPlanner object
 *
 */
    ~Dubins3DPlanner();

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
     * @brief
     *
     * @param msg
     */
    void nodePosesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);

    /**
     * @brief
     *
     * @param si
     * @return ompl::base::ValidStateSamplerPtr
     */
    ompl::base::ValidStateSamplerPtr allocValidStateSampler(
      const ompl::base::SpaceInformation * si);

    /**
     * @brief
     *
     * @param space
     * @return ompl::base::StateSamplerPtr
     */
    /*ompl::base::StateSamplerPtr allocStateSampler(const ompl::base::StateSpace * space);*/

    /**
     * @brief Get the Opt Objective object
     *
     * @return ompl::base::OptimizationObjectivePtr
     */
    ompl::base::OptimizationObjectivePtr getOptimizationObjective();

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
    rclcpp::Logger logger_{rclcpp::get_logger("dubins_3d_planner")};

    std::shared_ptr<ompl::base::RealVectorBounds> horizontal_dubins_bounds_;
    std::shared_ptr<ompl::base::RealVectorBounds> vertical_dubins_bounds_;

    std::shared_ptr<octomap::OcTree> elevated_surfel_octomap_octree_;
    std::shared_ptr<fcl::CollisionObject> elevated_surfels_collision_object_;


    geometry_msgs::msg::PoseArray::SharedPtr elevated_surfel_poses_msg_;
    pcl::PointCloud<pcl::PointSurfel>::Ptr elevated_surfel_cloud_;
    geometry_msgs::msg::PoseStamped nearest_elevated_surfel_to_start_;
    geometry_msgs::msg::PoseStamped nearest_elevated_surfel_to_goal_;

  };
}  // namespace vox_nav_planning

#endif  // VOX_NAV_PLANNING__PLUGINS__DUBINS_3D_PLANNER_HPP_
