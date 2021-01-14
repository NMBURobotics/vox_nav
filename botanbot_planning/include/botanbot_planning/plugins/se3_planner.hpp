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

#ifndef BOTANBOT_PLANNING__PLUGINS__SE3_PLANNER_HPP_
#define BOTANBOT_PLANNING__PLUGINS__SE3_PLANNER_HPP_

#include <vector>
#include <string>
#include <memory>

#include "botanbot_planning/planner_core.hpp"

/**
 * @brief
 *
 */
namespace botanbot_planning
{

/**
 * @brief
 *
 */
class SE3Planner : public botanbot_planning::PlannerCore
{
public:
/**
 * @brief Construct a new Botanbot O M P L Experimental object
 *
 */
  SE3Planner();

/**
 * @brief Destroy the Botanbot O M P L Experimental object
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
   * @brief Get the Selected Planner object
   *
   * @param planner_name
   * @param state_space_information
   * @param planner
   * @return true
   * @return false
   */
  bool getSelectedPlanner(
    const std::string & planner_name,
    const ompl::base::SpaceInformationPtr & state_space_information,
    ompl::base::PlannerPtr planner) override;

protected:
  /**
   * @brief
   *
   * @param si
   * @return ompl::base::OptimizationObjectivePtr
   */
  ompl::base::OptimizationObjectivePtr getOptObjective(const ompl::base::SpaceInformationPtr & si);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("se3_planner")};
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vis_pub_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr octomap_pointcloud_pub_;

  std::shared_ptr<fcl::CollisionObject> robot_collision_object_;
  std::shared_ptr<octomap::OcTree> octomap_octree_;
  std::shared_ptr<fcl::OcTree> fcl_octree_;
  std::shared_ptr<fcl::CollisionObject> fcl_octree_collision_object_;

  std::shared_ptr<ompl::base::RealVectorBounds> state_space_bounds_;
  ompl::base::StateSpacePtr state_space_;
  ompl::base::SpaceInformationPtr state_space_information_;

  // to ensure safety when accessing global var curr_frame_
  std::mutex global_mutex_;
  // the topic to subscribe in order capture a frame
  std::string planner_name_;

  std::string octomap_filename_;
  double octomap_voxel_size_;
  // whether plugin is enabled
  bool is_enabled_;
  // related to density of created path
  int interpolation_parameter_;
  //
  double planner_timeout_;
};
}  // namespace botanbot_planning

#endif  // BOTANBOT_PLANNING__PLUGINS__SE3_PLANNER_HPP_
