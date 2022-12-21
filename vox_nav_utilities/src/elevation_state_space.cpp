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


#include "vox_nav_utilities/elevation_state_space.hpp"
#include "ompl/tools/config/MagicConstants.h"
using namespace ompl::base;

OctoCostOptimizationObjective::OctoCostOptimizationObjective(
  const ompl::base::SpaceInformationPtr & si,
  const std::shared_ptr<octomap::OcTree> & elevated_surfels_octree)
: ompl::base::StateCostIntegralObjective(si, true),
  elevated_surfels_octree_(elevated_surfels_octree)
{
  description_ = "OctoCost Objective";
  RCLCPP_INFO(
    logger_,
    "OctoCost Optimization objective bases on an Octomap with %d nodes",
    elevated_surfels_octree_->size());
}

OctoCostOptimizationObjective::~OctoCostOptimizationObjective()
{
}

ompl::base::Cost OctoCostOptimizationObjective::stateCost(const ompl::base::State * s) const
{
  float cost = 5.0;
  const auto * s_se2 =
    s->as<ElevationStateSpace::StateType>()->as<SE2StateSpace::StateType>(0);
  const auto * s_z =
    s->as<ElevationStateSpace::StateType>()->as<RealVectorStateSpace::StateType>(1);
  auto node_at_samppled_state = elevated_surfels_octree_->search(
    s_se2->getX(),
    s_se2->getY(),
    s_z->values[0], 0);
  if (node_at_samppled_state) {
    if (elevated_surfels_octree_->isNodeOccupied(node_at_samppled_state)) {
      cost = static_cast<double>(node_at_samppled_state->getValue());
    }
  }
  return ompl::base::Cost(cost);
}

ompl::base::Cost OctoCostOptimizationObjective::motionCost(const State * s1, const State * s2) const
{
  return Cost(si_->distance(s1, s2));
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
  registerDefaultProjection(std::make_shared<ElevationStateSpaceProjection>(this));
  registerProjection(
    "ElevationStateSpaceProjection", std::make_shared<ElevationStateSpaceProjection>(this));

  type_ = 32;
  addSubspace(std::make_shared<SE2StateSpace>(), 1.0);
  addSubspace(std::make_shared<RealVectorStateSpace>(2), 1.0);
  lock();

  real_vector_ = std::make_shared<ompl::base::RealVectorStateSpace>(2);
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
  const RealVectorBounds & z_bounds,
  const RealVectorBounds & v_bounds)
{
  as<SE2StateSpace>(0)->setBounds(se2_bounds);

  auto z_and_v_bounds = std::make_shared<ompl::base::RealVectorBounds>(2);
  z_and_v_bounds->setLow(0, z_bounds.low[0]);
  z_and_v_bounds->setHigh(0, z_bounds.high[0]);
  z_and_v_bounds->setLow(1, v_bounds.low[0]);
  z_and_v_bounds->setHigh(1, v_bounds.high[0]);
  as<RealVectorStateSpace>(1)->setBounds(*z_and_v_bounds);

  se2_->setBounds(se2_bounds);
  dubins_->setBounds(se2_bounds);
  reeds_sheep_->setBounds(se2_bounds);
  real_vector_->setBounds(*z_and_v_bounds);
}

void ElevationStateSpace::enforceBounds(State * state) const
{
  auto * se2 = state->as<StateType>()->as<SE2StateSpace::StateType>(0);
  auto * z_and_v = state->as<StateType>()->as<RealVectorStateSpace::StateType>(1);

  se2_->enforceBounds(se2);
  real_vector_->enforceBounds(z_and_v);
}


const RealVectorBounds ElevationStateSpace::getBounds() const
{
  auto merged_bounds = std::make_shared<ompl::base::RealVectorBounds>(4);
  merged_bounds->setLow(0, as<SE2StateSpace>(0)->getBounds().low[0]);
  merged_bounds->setHigh(0, as<SE2StateSpace>(0)->getBounds().high[0]);
  merged_bounds->setLow(1, as<SE2StateSpace>(0)->getBounds().low[1]);
  merged_bounds->setHigh(1, as<SE2StateSpace>(0)->getBounds().high[1]);
  merged_bounds->setLow(2, as<RealVectorStateSpace>(1)->getBounds().low[0]);
  merged_bounds->setHigh(2, as<RealVectorStateSpace>(1)->getBounds().high[0]);
  merged_bounds->setLow(3, as<RealVectorStateSpace>(1)->getBounds().low[1]);
  merged_bounds->setHigh(3, as<RealVectorStateSpace>(1)->getBounds().high[1]);
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
    return std::sqrt(
      std::pow(state1_se2->getX() - state2_se2->getX(), 2) +
      std::pow(state1_se2->getY() - state2_se2->getY(), 2) +
      std::pow(state1_z->values[0] - state2_z->values[0], 2));

    //return se2_->distance(state1_se2, state2_se2);
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

  // WARN , This is too expensive
  /*nearest_intermediate_surfel = vox_nav_utilities::getNearstPoint<
    pcl::PointSurfel,
    pcl::PointCloud<pcl::PointSurfel>::Ptr>(
    dubins_surfel,
    workspace_surfels_);
  state_z->values[0] = nearest_intermediate_surfel.z;*/

  state_z->values[0] = (from_z->values[0] + to_z->values[0]) / 2.0;
}

void ompl::base::ElevationStateSpace::printState(const State * state, std::ostream & out) const
{
  auto * se2 = state->as<StateType>()->as<SE2StateSpace::StateType>(0);
  auto * z_and_v = state->as<StateType>()->as<RealVectorStateSpace::StateType>(1);
  se2_->printState(se2, out);
  real_vector_->printState(z_and_v, out);
}


OctoCellValidStateSampler::OctoCellValidStateSampler(
  const ompl::base::SpaceInformationPtr & si,
  const geometry_msgs::msg::PoseStamped start,
  const geometry_msgs::msg::PoseStamped goal,
  const geometry_msgs::msg::PoseArray::SharedPtr & elevated_surfels_poses)
: ValidStateSampler(si.get()),
  elevated_surfels_poses_(*elevated_surfels_poses),
  rng_(rd_())
{
  workspace_surfels_ = pcl::PointCloud<pcl::PointSurfel>::Ptr(
    new pcl::PointCloud<pcl::PointSurfel>);

  vox_nav_utilities::fillSurfelsfromMsgPoses(elevated_surfels_poses_, workspace_surfels_);

  name_ = "OctoCellValidStateSampler";
  RCLCPP_INFO(
    logger_, "OctoCellValidStateSampler bases on an Octomap with %d surfels",
    workspace_surfels_->points.size());

  updateSearchArea(start, goal);
}

bool OctoCellValidStateSampler::sample(ompl::base::State * state)
{
  auto * cstate = state->as<ompl::base::ElevationStateSpace::StateType>();
  auto val = (*int_distr_)(rng_);
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
  auto * cstate = state->as<ompl::base::ElevationStateSpace::StateType>();
  auto * near_cstate = near->as<ompl::base::ElevationStateSpace::StateType>();
  const auto * se2 = near_cstate->as<ompl::base::SE2StateSpace::StateType>(0);
  const auto * z = near_cstate->as<ompl::base::RealVectorStateSpace::StateType>(1);

  pcl::PointSurfel surfel;
  surfel.x = se2->getX();
  surfel.y = se2->getY();
  surfel.z = z->values[0];

  auto out_sample = vox_nav_utilities::getNearstRPoints<
    pcl::PointSurfel,
    pcl::PointCloud<pcl::PointSurfel>::Ptr>(
    distance,
    surfel,
    search_area_surfels_);

  cstate->setSE2(
    out_sample.x,
    out_sample.y, 0);
  cstate->setZ(out_sample.z);

  return true;
}

void OctoCellValidStateSampler::updateSearchArea(
  const geometry_msgs::msg::PoseStamped start,
  const geometry_msgs::msg::PoseStamped goal)
{
  RCLCPP_INFO(logger_, "Updating search area");

  double radius = vox_nav_utilities::getEuclidianDistBetweenPoses(goal, start) / 1.0;
  auto search_point_pose = vox_nav_utilities::getLinearInterpolatedPose(goal, start);
  auto search_point_surfel = vox_nav_utilities::poseMsg2PCLSurfel(search_point_pose);

  search_area_surfels_ =
    vox_nav_utilities::getSubCloudWithinRadius<pcl::PointSurfel>(
    workspace_surfels_, search_point_surfel,
    radius);

  RCLCPP_INFO(logger_, "Updated search area surfels, %d", search_area_surfels_->points.size());

  /*search_area_surfels_ = vox_nav_utilities::uniformlySampleCloud<pcl::PointSurfel>(
    search_area_surfels_, 1.2);*/

  RCLCPP_INFO(
    logger_, "Uniformly sampled %d search area surfels,", search_area_surfels_->points.size());

  std::vector<int> weights;
  for (auto && i : search_area_surfels_->points) {
    //auto tilt_angle = std::max(std::abs(i.normal_x), std::abs(i.normal_y)) * 180.0 / M_PI;
    //weights.push_back(200 / tilt_angle);
    weights.push_back(1);
  }
  std::discrete_distribution<> distrubutions(weights.begin(), weights.end());
  distrubutions_ = distrubutions;

  int_distr_ = std::make_shared<std::uniform_int_distribution<int>>(
    0,
    search_area_surfels_->points.size() - 1);

}
