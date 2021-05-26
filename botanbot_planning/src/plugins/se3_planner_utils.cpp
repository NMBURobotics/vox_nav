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
  const ompl::base::SpaceInformationPtr & si,
  const ompl::base::ScopedState<ompl::base::SE3StateSpace> * start,
  const ompl::base::ScopedState<ompl::base::SE3StateSpace> * goal,
  const std::shared_ptr<octomap::ColorOcTree> & tree)
: ValidStateSampler(si.get())
{
  name_ = "OctoCellValidStateSampler";

  workspace_pcl_ =
    pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

  for (auto it = tree->begin(),
    end = tree->end();
    it != end; ++it)
  {
    if (it->getValue() > 2.0) {
      pcl::PointXYZ node_as_point;
      node_as_point.x = it.getCoordinate().x();
      node_as_point.y = it.getCoordinate().y();
      node_as_point.z = it.getCoordinate().z();
      workspace_pcl_->points.push_back(node_as_point);
    }
  }
  workspace_pcl_->width = 1;
  workspace_pcl_->height = workspace_pcl_->points.size();

  std::cout << "OctoCellValidStateSampler bases on an Octomap with " <<
    workspace_pcl_->points.size() << " nodes" << std::endl;

  updateSearchArea(start, goal);
}

bool OctoCellValidStateSampler::sample(ompl::base::State * state)
{
  auto se3_state = static_cast<ompl::base::SE3StateSpace::StateType *>(state);
  unsigned int attempts = 0;
  bool valid = false;
  do {

    pcl::PointCloud<pcl::PointXYZ>::Ptr out_sample(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::RandomSample<pcl::PointXYZ> random_sample(true);
    random_sample.setInputCloud(search_area_pcl_);
    random_sample.setSample(1);
    pcl::Indices indices;
    random_sample.filter(indices);
    random_sample.filter(*out_sample);

    se3_state->setXYZ(
      out_sample->points.front().x,
      out_sample->points.front().y,
      out_sample->points.front().z);

    valid = si_->isValid(state);
    ++attempts;

  } while (!valid && attempts < attempts_ && search_area_pcl_->points.size());
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
    se3_state->setXYZ(0, 0, 0);
    valid = si_->isValid(state);
    ++attempts;
  } while (!valid && attempts < attempts_ && workspace_pcl_->points.size());
  return valid;
}

void OctoCellValidStateSampler::updateSearchArea(
  const ompl::base::ScopedState<ompl::base::SE3StateSpace> * start,
  const ompl::base::ScopedState<ompl::base::SE3StateSpace> * goal)
{
  std::cout << "Updating search area " << std::endl;

  search_area_pcl_ =
    pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

  float resolution = 0.2;

  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);
  octree.setInputCloud(workspace_pcl_);
  octree.addPointsFromInputCloud();

  pcl::PointXYZ searchPoint;
  searchPoint.x = (goal->get()->getX() + start->get()->getX()) / 2.0;
  searchPoint.y = (goal->get()->getY() + start->get()->getY()) / 2.0;
  searchPoint.z = (goal->get()->getZ() + start->get()->getZ()) / 2.0;

  // Neighbors within radius search

  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

  float radius = std::sqrt(
    std::pow( (goal->get()->getX() - start->get()->getX()), 2) +
    std::pow( (goal->get()->getY() - start->get()->getY()), 2) +
    std::pow( (goal->get()->getZ() - start->get()->getZ()), 2)
  );

  std::cout << "Adjusting a search area with radius of: " << radius << std::endl;

  if (octree.radiusSearch(
      searchPoint, radius, pointIdxRadiusSearch,
      pointRadiusSquaredDistance) > 0)
  {
    for (std::size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
      search_area_pcl_->points.push_back(workspace_pcl_->points[pointIdxRadiusSearch[i]]);
    }
  }
  std::cout << "Updated search area nodes." << std::endl;
  std::cout << "Search area has nodes: " << search_area_pcl_->points.size() << std::endl;
}

}  // namespace botanbot_planning
