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

#include "botanbot_planning/plugins/se3_planner_utils.hpp"

namespace botanbot_planning
{

OctoCostOptimizationObjective::OctoCostOptimizationObjective(
  const ompl::base::SpaceInformationPtr & si,
  std::shared_ptr<octomap::ColorOcTree> tree)
: ompl::base::StateCostIntegralObjective(si, true)
{
  description_ = "OctoCost Objective";
  color_octomap_octree_ = tree;
  std::cout << "OctoCost Optimization objective bases on an Octomap with " <<
    color_octomap_octree_->size() << " nodes" << std::endl;
}

OctoCostOptimizationObjective::~OctoCostOptimizationObjective()
{
}

ompl::base::Cost OctoCostOptimizationObjective::stateCost(const ompl::base::State * s) const
{
  const ompl::base::SE3StateSpace::StateType * se3_state =
    s->as<ompl::base::SE3StateSpace::StateType>();

  double x = se3_state->getX();
  double y = se3_state->getY();
  double z = se3_state->getZ();

  float cost(0.0);
  auto node_at_samppled_state = color_octomap_octree_->search(x, y, z, 0);

  if (node_at_samppled_state) {
    if (!node_at_samppled_state->getColor().r) {
      cost = 1.0 * static_cast<double>(node_at_samppled_state->getColor().b / 255.0);
    }
  } else {
    cost = 5.0;
  }
  return ompl::base::Cost(cost);
}

///////////////////////////////////////////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
///////////////////////////////////////////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
///////////////////////////////////////////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

OctoCellValidStateSampler::OctoCellValidStateSampler(
  const ompl::base::SpaceInformation * si,
  const std::shared_ptr<octomap::ColorOcTree> & tree)
: ValidStateSampler(si)
{
  name_ = "OctoCellValidStateSampler";

  color_octomap_octree_ = tree;

  for (auto it = tree->begin(),
    end = tree->end(); it != end; ++it)
  {
    if (it->getColor().r == 255 && it->getColor().g == 255) {
      auto pair =
        std::pair<octomap::OcTreeKey, octomap::point3d>(it.getKey(), it.getCoordinate());
      color_octomap_node_colors_.insert(pair);
    }
  }
  std::cout << "OctoCellValidStateSampler bases on an Octomap with " <<
    color_octomap_node_colors_.size() << " nodes" << std::endl;
}

bool OctoCellValidStateSampler::sample(ompl::base::State * state)
{
  auto se3_state = static_cast<ompl::base::SE3StateSpace::StateType *>(state);
  unsigned int attempts = 0;
  bool valid = false;
  do {

    octomap::unordered_ns::unordered_multimap<
      octomap::OcTreeKey,
      octomap::point3d,
      octomap::OcTreeKey::KeyHash> random_octomap_node;

    std::sample(
      color_octomap_node_colors_.begin(),
      color_octomap_node_colors_.end(),
      std::inserter(random_octomap_node, random_octomap_node.end()),
      1,
      std::mt19937{std::random_device{} ()}
    );

    se3_state->setXYZ(
      random_octomap_node.begin()->second.x(),
      random_octomap_node.begin()->second.y(),
      random_octomap_node.begin()->second.z());

    valid = si_->isValid(state);
    ++attempts;
  } while (!valid && attempts < attempts_ && color_octomap_node_colors_.size());
  return valid;
}

bool OctoCellValidStateSampler::sampleNear(
  ompl::base::State * state, const ompl::base::State * near,
  const double distance)
{
  auto se3_state = static_cast<ompl::base::SE3StateSpace::StateType *>(state);
  unsigned int attempts = 0;
  bool valid = false;
  do {

    octomap::unordered_ns::unordered_multimap<
      octomap::OcTreeKey,
      octomap::point3d,
      octomap::OcTreeKey::KeyHash> random_octomap_node;

    std::sample(
      color_octomap_node_colors_.begin(),
      color_octomap_node_colors_.end(),
      std::inserter(random_octomap_node, random_octomap_node.end()),
      1,
      std::mt19937{std::random_device{} ()}
    );

    se3_state->setXYZ(
      random_octomap_node.begin()->second.x(),
      random_octomap_node.begin()->second.y(),
      random_octomap_node.begin()->second.z());

    valid = si_->isValid(state);
    ++attempts;

  } while (!valid && attempts < attempts_ && color_octomap_node_colors_.size());
  return valid;
}
}  // namespace botanbot_planning
