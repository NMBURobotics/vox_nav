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

OctoCostOptimizationObjective::OctoCostOptimizationObjective(
  const ompl::base::SpaceInformationPtr & si,
  const std::shared_ptr<octomap::OcTree> & elevated_surfels_octree)
: ompl::base::StateCostIntegralObjective(si, true),
  elevated_surfels_octree_(elevated_surfels_octree)
{
  description_ = "OctoCost Objective";
  /*RCLCPP_INFO(
    logger_,
    "OctoCost Optimization objective bases on an Octomap with %d nodes",
    elevated_surfels_octree_->size());*/
}

OctoCostOptimizationObjective::~OctoCostOptimizationObjective()
{
}

ompl::base::Cost OctoCostOptimizationObjective::stateCost(const ompl::base::State * s) const
{
  float cost = 0.0;
  /*const ompl::base::SE3StateSpace::StateType * se3_state =
    s->as<ompl::base::SE3StateSpace::StateType>();

  auto node_at_samppled_state = elevated_surfels_octree_->search(
    se3_state->getX(),
    se3_state->getY(),
    se3_state->getZ(), 0);
  if (node_at_samppled_state) {
    if (elevated_surfels_octree_->isNodeOccupied(node_at_samppled_state)) {
      cost = 1.0 * static_cast<double>(node_at_samppled_state->getValue());
    }
  } else {
    cost = 5.0;
  }*/
  return ompl::base::Cost(cost);
}

ElevationStateSpace::ElevationStateSpace(
  const SE2StateType state_type,
  const geometry_msgs::msg::PoseArray::SharedPtr & elevated_surfels_poses,
  double turningRadius, bool isSymmetric)
:  rho_(turningRadius),
  isSymmetric_(isSymmetric),
  elevated_surfels_poses_(*elevated_surfels_poses),
  se2_state_type_(state_type)
{
  setName("ElevationStateSpace" + getName());
  type_ = 32;
  addSubspace(std::make_shared<DubinsStateSpace>(), 1.0);
  addSubspace(std::make_shared<RealVectorStateSpace>(1), 1.0);
  lock();

  se2_ = std::make_shared<ompl::base::SE2StateSpace>();
  dubins_ = std::make_shared<ompl::base::DubinsStateSpace>(rho_, isSymmetric_);
  reeds_sheep_ = std::make_shared<ompl::base::ReedsSheppStateSpace>(rho_);

  workspace_surfels_ = pcl::PointCloud<pcl::PointSurfel>::Ptr(
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
}

void ElevationStateSpace::setBounds(
  const RealVectorBounds & se2_bounds,
  const RealVectorBounds & z_bounds)
{
  as<SE2StateSpace>(0)->setBounds(se2_bounds);
  as<RealVectorStateSpace>(1)->setBounds(z_bounds);

  se2_->setBounds(se2_bounds);
  dubins_->setBounds(se2_bounds);
  reeds_sheep_->setBounds(se2_bounds);
}

const RealVectorBounds ElevationStateSpace::getBounds() const
{
  auto merged_bounds = std::make_shared<ompl::base::RealVectorBounds>(3);
  merged_bounds->setLow(0, as<SE2StateSpace>(0)->getBounds().low[0]);
  merged_bounds->setHigh(0, as<SE2StateSpace>(0)->getBounds().high[0]);
  merged_bounds->setLow(1, as<SE2StateSpace>(0)->getBounds().low[1]);
  merged_bounds->setHigh(1, as<SE2StateSpace>(0)->getBounds().high[1]);
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

double ompl::base::ElevationStateSpace::distance(const State * state1, const State * state2) const
{
  const auto * state1_se2 =
    state1->as<StateType>()->as<SE2StateSpace::StateType>(0);
  const auto * state1_z =
    state1->as<StateType>()->as<RealVectorStateSpace::StateType>(1);
  const auto * state2_se2 =
    state2->as<StateType>()->as<SE2StateSpace::StateType>(0);
  const auto * state2_z =
    state2->as<StateType>()->as<RealVectorStateSpace::StateType>(1);

  if (se2_state_type_ == SE2StateType::SE2) {
    return se2_->distance(state1_se2, state2_se2);
  } else if (se2_state_type_ == SE2StateType::DUBINS) {
    if (isSymmetric_) {
      return rho_ * std::min(
        dubins_->dubins(state1_se2, state2_se2).length(),
        dubins_->dubins(state2_se2, state1_se2).length());
    }
    return rho_ * dubins_->dubins(state1_se2, state2_se2).length();
  } else {
    return rho_ * reeds_sheep_->reedsShepp(state1_se2, state2_se2).length();
  }
  return 0;
}

void ompl::base::ElevationStateSpace::interpolate(
  const State * from, const State * to, double t,
  State * state) const
{
  const auto * from_dubins = from->as<StateType>()->as<SE2StateSpace::StateType>(0);
  const auto * from_z = from->as<StateType>()->as<RealVectorStateSpace::StateType>(1);

  const auto * to_dubins = to->as<StateType>()->as<SE2StateSpace::StateType>(0);
  const auto * to_z = to->as<StateType>()->as<RealVectorStateSpace::StateType>(1);

  auto * state_dubins = state->as<StateType>()->as<SE2StateSpace::StateType>(0);
  auto * state_z = state->as<StateType>()->as<RealVectorStateSpace::StateType>(1);

  if (se2_state_type_ == SE2StateType::SE2) {
    se2_->interpolate(from_dubins, to_dubins, t, state_dubins);
  } else if (se2_state_type_ == SE2StateType::DUBINS) {
    dubins_->interpolate(from_dubins, to_dubins, t, state_dubins);
  } else {
    reeds_sheep_->interpolate(from_dubins, to_dubins, t, state_dubins);
  }

  pcl::PointSurfel dubins_surfel, nearest_intermediate_surfel;
  dubins_surfel.x = state_dubins->getX();
  dubins_surfel.y = state_dubins->getY();
  dubins_surfel.z = from_z->values[0];

  nearest_intermediate_surfel = vox_nav_utilities::getNearstPoint<
    pcl::PointSurfel,
    pcl::PointCloud<pcl::PointSurfel>::Ptr>(
    dubins_surfel,
    workspace_surfels_);

  state_z->values[0] = nearest_intermediate_surfel.z;
}

OctoCellValidStateSampler::OctoCellValidStateSampler(
  const ompl::base::SpaceInformationPtr & si,
  const geometry_msgs::msg::PoseStamped start,
  const geometry_msgs::msg::PoseStamped goal,
  const std::shared_ptr<fcl::CollisionObject> & robot_collision_object,
  const std::shared_ptr<fcl::CollisionObject> & original_octomap_collision_object,
  const geometry_msgs::msg::PoseArray::SharedPtr & elevated_surfels_poses)
: ValidStateSampler(si.get()),
  robot_collision_object_(robot_collision_object),
  original_octomap_collision_object_(original_octomap_collision_object),
  elevated_surfels_poses_(*elevated_surfels_poses)
{
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

  name_ = "OctoCellValidStateSampler";
  RCLCPP_INFO(
    logger_,
    "OctoCellValidStateSampler bases on an Octomap with %d surfels",
    elevated_surfels_poses_.poses.size());

  updateSearchArea(start, goal);
}

bool OctoCellValidStateSampler::sample(ompl::base::State * state)
{
  auto * cstate = state->as<ompl::base::ElevationStateSpace::StateType>();
  std::random_device rd;
  std::mt19937 rng(rd());
  int val = distrubutions_(rng);
  auto out_sample = search_area_surfels_->points.at(val);
  cstate->setSE2(
    out_sample.x,
    out_sample.y, 0);
  cstate->setZ(out_sample.z);
  return true;
}

bool OctoCellValidStateSampler::sampleNear(
  ompl::base::State * state, const ompl::base::State * near,
  const double distance)
{
  throw ompl::Exception("OctoCellValidStateSampler::sampleNear", "not implemented");
  RCLCPP_ERROR(logger_, "Non implementd function call OctoCellValidStateSampler::sampleNear");
  return false;
}

void OctoCellValidStateSampler::updateSearchArea(
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
  search_area_surfels_ = vox_nav_utilities::uniformly_sample_cloud<pcl::PointSurfel>(
    search_area_surfels_, radius / 10);

  RCLCPP_INFO(
    logger_, "Uniformly sampled %d search area surfels,", search_area_surfels_->points.size());


  std::vector<int> weights;
  for (auto && i : search_area_surfels_->points) {
    //auto max_tilt_angle = std::max(std::abs(i.normal_x), std::abs(i.normal_y)) * 180.0 / M_PI;
    //weights.push_back(40 / max_tilt_angle);
    weights.push_back(1);
  }
  std::discrete_distribution<> distrubutions(weights.begin(), weights.end());
  distrubutions_ = distrubutions;
}
