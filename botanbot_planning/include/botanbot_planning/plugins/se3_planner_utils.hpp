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

#ifndef BOTANBOT_PLANNING__PLUGINS__SE3_PLANNER_UTILS_HPP_
#define BOTANBOT_PLANNING__PLUGINS__SE3_PLANNER_UTILS_HPP_

#include "botanbot_planning/planner_core.hpp"

/**
 * @brief
 *
 */
namespace botanbot_planning
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
    std::shared_ptr<octomap::ColorOcTree> tree);

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

private:
  std::shared_ptr<octomap::ColorOcTree> color_octomap_octree_;
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
    const std::shared_ptr<octomap::ColorOcTree> & tree);

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
    ompl::base::State * state, const ompl::base::State * near,
    const double distance) override;

protected:
  octomap::unordered_ns::unordered_multimap<
    octomap::OcTreeKey,
    octomap::point3d,
    octomap::OcTreeKey::KeyHash> color_octomap_node_colors_;
  std::shared_ptr<octomap::ColorOcTree> color_octomap_octree_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr nodes_as_pcl_;
};

}  // namespace botanbot_planning

#endif  // BOTANBOT_PLANNING__PLUGINS__SE3_PLANNER_UTILS_HPP_
