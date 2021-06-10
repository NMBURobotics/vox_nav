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


#include "vox_nav_planning/plugins/elevation_state_space.hpp"
#include "ompl/tools/config/MagicConstants.h"
using namespace ompl::base;


ElevationStateSampler::ElevationStateSampler(
  const StateSpace * space,
  const pcl::PointCloud<pcl::PointSurfel>::Ptr workspace_surfels,
  const pcl::PointCloud<pcl::PointSurfel>::Ptr search_area_surfels)
: StateSampler(space),
  workspace_surfels_(workspace_surfels),
  search_area_surfels_(search_area_surfels)
{
}

void ElevationStateSampler::sampleUniform(State * state)
{
  auto * cstate = state->as<ompl::base::ElevationStateSpace::StateType>();
  pcl::PointCloud<pcl::PointSurfel>::Ptr out_sample(new pcl::PointCloud<pcl::PointSurfel>);
  pcl::RandomSample<pcl::PointSurfel> random_sample(true);
  random_sample.setInputCloud(search_area_surfels_);
  random_sample.setSample(1);
  random_sample.setSeed(rand());
  random_sample.filter(*out_sample);
  cstate->setSE2(
    out_sample->points.front().x,
    out_sample->points.front().y, 0);
  cstate->setZ(out_sample->points.front().z);
}

void ElevationStateSampler::sampleUniformNear(State * state, const State * near, double distance)
{
  auto * cstate = state->as<ompl::base::ElevationStateSpace::StateType>();
  pcl::PointCloud<pcl::PointSurfel>::Ptr out_sample(new pcl::PointCloud<pcl::PointSurfel>);
  pcl::RandomSample<pcl::PointSurfel> random_sample(true);
  random_sample.setInputCloud(search_area_surfels_);
  random_sample.setSample(1);
  random_sample.setSeed(rand());
  random_sample.filter(*out_sample);
  cstate->setSE2(
    out_sample->points.front().x,
    out_sample->points.front().y, 0);
  cstate->setZ(out_sample->points.front().z);
  space_->enforceBounds(state);
}

void ElevationStateSampler::sampleGaussian(State * state, const State * mean, double stdDev)
{
  auto * cstate = state->as<ompl::base::ElevationStateSpace::StateType>();
  pcl::PointCloud<pcl::PointSurfel>::Ptr out_sample(new pcl::PointCloud<pcl::PointSurfel>);
  pcl::RandomSample<pcl::PointSurfel> random_sample(true);
  random_sample.setInputCloud(search_area_surfels_);
  random_sample.setSample(1);
  random_sample.setSeed(rand());
  random_sample.filter(*out_sample);
  cstate->setSE2(
    out_sample->points.front().x,
    out_sample->points.front().y, 0);
  cstate->setZ(out_sample->points.front().z);
  space_->enforceBounds(state);
}

ElevationStateSpace::ElevationStateSpace(
  const geometry_msgs::msg::PoseStamped start,
  const geometry_msgs::msg::PoseStamped goal,
  const geometry_msgs::msg::PoseArray::SharedPtr & elevated_surfels_poses)
: elevated_surfels_poses_(*elevated_surfels_poses)
{
  setName("Elevation" + getName());
  type_ = 31;       // why ?
  addSubspace(std::make_shared<DubinsStateSpace>(), 1.0);
  addSubspace(std::make_shared<RealVectorStateSpace>(1), 1.0);
  lock();

  workspace_surfels_ = pcl::PointCloud<pcl::PointSurfel>::Ptr(
    new pcl::PointCloud<pcl::PointSurfel>);
  search_area_surfels_ = pcl::PointCloud<pcl::PointSurfel>::Ptr(
    new pcl::PointCloud<pcl::PointSurfel>);

  for (auto && i : elevated_surfels_poses_.poses) {
    pcl::PointSurfel surfel;
    surfel.x = i.position.x;
    surfel.y = i.position.y;
    surfel.z = i.position.z;
    double r, p, y;
    vox_nav_utilities::getRPYfromMsgQuaternion(i.orientation, r, p, y);
    surfel.normal_x = r;
    surfel.normal_y = p;
    surfel.normal_z = y;
    workspace_surfels_->points.push_back(surfel);
  }

  RCLCPP_INFO(
    logger_,
    "OctoCellStateSampler bases on an Octomap with %d surfels",
    elevated_surfels_poses_.poses.size());

  updateSearchArea(start, goal);
}

void ElevationStateSpace::setBounds(
  const RealVectorBounds & se2_bounds,
  const RealVectorBounds & z_bounds)
{
  as<DubinsStateSpace>(0)->setBounds(se2_bounds);
  as<RealVectorStateSpace>(1)->setBounds(z_bounds);
}

const RealVectorBounds ElevationStateSpace::getBounds() const
{
  auto merged_bounds = std::make_shared<ompl::base::RealVectorBounds>(3);
  merged_bounds->setLow(0, as<DubinsStateSpace>(0)->getBounds().low[0]);
  merged_bounds->setHigh(0, as<DubinsStateSpace>(0)->getBounds().high[0]);
  merged_bounds->setLow(1, as<DubinsStateSpace>(0)->getBounds().low[1]);
  merged_bounds->setHigh(1, as<DubinsStateSpace>(0)->getBounds().high[1]);
  merged_bounds->setLow(2, as<RealVectorStateSpace>(1)->getBounds().low[0]);
  merged_bounds->setHigh(2, as<RealVectorStateSpace>(1)->getBounds().high[0]);
  return *merged_bounds;
}

State * ElevationStateSpace::allocState() const
{
  auto * state = new StateType();
  allocStateComponents(state);
  return state;
}

void ElevationStateSpace::freeState(State * state) const
{
  CompoundStateSpace::freeState(state);
}

StateSamplerPtr ElevationStateSpace::allocDefaultStateSampler() const
{
  return std::make_shared<ElevationStateSampler>(this, workspace_surfels_, search_area_surfels_);
}

double ompl::base::ElevationStateSpace::distance(const State * state1, const State * state2) const
{
  return CompoundStateSpace::distance(state1, state2);
}

void ompl::base::ElevationStateSpace::interpolate(
  const State * from, const State * to, double t,
  State * state) const
{
  CompoundStateSpace::interpolate(from, to, t, state);
}

void ompl::base::ElevationStateSpace::registerProjections()
{
  class ElevationDefaultProjection : public ProjectionEvaluator
  {
  public:
    ElevationDefaultProjection(const StateSpace * space)
    : ProjectionEvaluator(space)
    {
    }

    unsigned int getDimension() const override
    {
      return 3;
    }

    void defaultCellSizes() override
    {
      cellSizes_.resize(3);
      bounds_ = space_->as<ElevationStateSpace>()->getBounds();
      std::cout << "kokkokoko " << bounds_.high[0] << bounds_.low[0];
      cellSizes_[0] = (bounds_.high[0] - bounds_.low[0]) / magic::PROJECTION_DIMENSION_SPLITS;
      cellSizes_[1] = (bounds_.high[1] - bounds_.low[1]) / magic::PROJECTION_DIMENSION_SPLITS;
      cellSizes_[2] = (bounds_.high[2] - bounds_.low[2]) / magic::PROJECTION_DIMENSION_SPLITS;
    }

    void project(const State * state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
      const auto * start = state->as<ElevationStateSpace::StateType>();
      const auto * state_se2 = start->as<DubinsStateSpace::StateType>(0);
      const auto * state_z = start->as<RealVectorStateSpace::StateType>(1);
      double * data = new double[3];
      data[0] = state_se2->getX();
      data[1] = state_se2->getY();
      data[2] = state_z->values[0];
      projection = Eigen::Map<const Eigen::VectorXd>(data, 3);
    }
  };

  registerDefaultProjection(std::make_shared<ElevationDefaultProjection>(this));
}

void ElevationStateSpace::updateSearchArea(
  const geometry_msgs::msg::PoseStamped start,
  const geometry_msgs::msg::PoseStamped goal)
{
  RCLCPP_INFO(logger_, "Updating search area");
  search_area_surfels_ =
    pcl::PointCloud<pcl::PointSurfel>::Ptr(
    new pcl::PointCloud<pcl::PointSurfel>);
  float resolution = 0.2;
  pcl::octree::OctreePointCloudSearch<pcl::PointSurfel> octree(resolution);
  octree.setInputCloud(workspace_surfels_);
  octree.addPointsFromInputCloud();
  pcl::PointSurfel searchPoint;
  searchPoint.x = (goal.pose.position.x + start.pose.position.x) / 2.0;
  searchPoint.y = (goal.pose.position.y + start.pose.position.y) / 2.0;
  searchPoint.z = (goal.pose.position.z + start.pose.position.z) / 2.0;
  // Neighbors within radius search
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;
  float radius = std::sqrt(
    std::pow( (goal.pose.position.x - start.pose.position.x), 2) +
    std::pow( (goal.pose.position.y - start.pose.position.y), 2) +
    std::pow( (goal.pose.position.z - start.pose.position.z), 2)
  );
  RCLCPP_INFO(logger_, "Adjusting a search area with radius of: %.3f", radius);
  if (octree.radiusSearch(
      searchPoint, radius, pointIdxRadiusSearch,
      pointRadiusSquaredDistance) > 0)
  {
    for (std::size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
      search_area_surfels_->points.push_back(workspace_surfels_->points[pointIdxRadiusSearch[i]]);
    }
  }

  RCLCPP_INFO(logger_, "Updated search area surfels, %d", search_area_surfels_->points.size());
}
