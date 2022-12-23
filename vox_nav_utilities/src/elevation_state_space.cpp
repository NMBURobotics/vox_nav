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
  const auto * s_so2 = s->as<ElevationStateSpace::StateType>()->as<SO2StateSpace::StateType>(0);
  const auto * s_xyzv =
    s->as<ElevationStateSpace::StateType>()->as<RealVectorStateSpace::StateType>(1);
  auto node_at_samppled_state = elevated_surfels_octree_->search(
    s_xyzv->values[0],
    s_xyzv->values[1],
    s_xyzv->values[2], 0);
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

  type_ = STATE_SPACE_SE3; // Well, not exactly, but this is the closest, Infromed sampling requirement
  addSubspace(std::make_shared<SO2StateSpace>(), 1.0);
  addSubspace(std::make_shared<RealVectorStateSpace>(4), 1.0); // x, y, z, v(linear speed)
  lock();

  real_vector_ = std::make_shared<ompl::base::RealVectorStateSpace>(4); // x,y,z, v(linear speed)
  se2_ = std::make_shared<ompl::base::SE2StateSpace>();
  dubins_ = std::make_shared<ompl::base::DubinsStateSpace>(rho_, isSymmetric_);
  reeds_sheep_ = std::make_shared<ompl::base::ReedsSheppStateSpace>(rho_);
  so2_ = std::make_shared<ompl::base::SO2StateSpace>();

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

  auto xyzv_bounds = std::make_shared<ompl::base::RealVectorBounds>(4);
  xyzv_bounds->setLow(0, se2_bounds.low[0]);    // x-
  xyzv_bounds->setHigh(0, se2_bounds.high[0]);  // x+
  xyzv_bounds->setLow(1, se2_bounds.low[0]);    // y-
  xyzv_bounds->setHigh(1, se2_bounds.high[0]);  // y+
  xyzv_bounds->setLow(2, z_bounds.low[0]);      // z-
  xyzv_bounds->setHigh(2, z_bounds.high[0]);    // z+
  xyzv_bounds->setLow(3, v_bounds.low[0]);      // v-
  xyzv_bounds->setHigh(3, v_bounds.high[0]);    // v+

  as<RealVectorStateSpace>(1)->setBounds(*xyzv_bounds);

  se2_->setBounds(se2_bounds);
  dubins_->setBounds(se2_bounds);
  reeds_sheep_->setBounds(se2_bounds);
  real_vector_->setBounds(*xyzv_bounds);
}

void ElevationStateSpace::enforceBounds(State * state) const
{
  auto * xyzv = state->as<StateType>()->as<RealVectorStateSpace::StateType>(1);

  real_vector_->enforceBounds(xyzv);
}


const RealVectorBounds ElevationStateSpace::getBounds() const
{
  auto merged_bounds = std::make_shared<ompl::base::RealVectorBounds>(5);
  merged_bounds->setLow(0, -M_PI);
  merged_bounds->setHigh(0, M_PI);
  merged_bounds->setLow(1, as<RealVectorStateSpace>(1)->getBounds().low[0]);
  merged_bounds->setHigh(1, as<RealVectorStateSpace>(1)->getBounds().high[0]);
  merged_bounds->setLow(2, as<RealVectorStateSpace>(1)->getBounds().low[1]);
  merged_bounds->setHigh(2, as<RealVectorStateSpace>(1)->getBounds().high[1]);
  merged_bounds->setLow(3, as<RealVectorStateSpace>(1)->getBounds().low[0]);
  merged_bounds->setHigh(3, as<RealVectorStateSpace>(1)->getBounds().high[0]);
  merged_bounds->setLow(4, as<RealVectorStateSpace>(1)->getBounds().low[1]);
  merged_bounds->setHigh(4, as<RealVectorStateSpace>(1)->getBounds().high[1]);
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

  const auto * state1_so2 = state1->as<StateType>()->as<SO2StateSpace::StateType>(0);
  const auto * state1_xyzv = state1->as<StateType>()->as<RealVectorStateSpace::StateType>(1);

  const auto * state2_so2 = state2->as<StateType>()->as<SO2StateSpace::StateType>(0);
  const auto * state2_xyzv = state2->as<StateType>()->as<RealVectorStateSpace::StateType>(1);

  auto state1_se2 = se2_->allocState(); auto state2_se2 = se2_->allocState();
  state1_se2->as<SE2StateSpace::StateType>()->setXY(state1_xyzv->values[0], state1_xyzv->values[1]);
  state1_se2->as<SE2StateSpace::StateType>()->setYaw(state1_so2->value);
  state2_se2->as<SE2StateSpace::StateType>()->setXY(state2_xyzv->values[0], state2_xyzv->values[1]);
  state2_se2->as<SE2StateSpace::StateType>()->setYaw(state2_so2->value);

  if (se2_state_type_ == SE2StateType::SE2) {
    return std::sqrt(
      std::pow(state1_xyzv->values[0] - state2_xyzv->values[0], 2) +
      std::pow(state1_xyzv->values[1] - state2_xyzv->values[1], 2) +
      std::pow(state1_xyzv->values[2] - state2_xyzv->values[2], 2));

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
  const auto * from_so2 = from->as<StateType>()->as<SO2StateSpace::StateType>(0);
  const auto * from_xyzv = from->as<StateType>()->as<RealVectorStateSpace::StateType>(1);

  const auto * to_so2 = to->as<StateType>()->as<SO2StateSpace::StateType>(0);
  const auto * to_xyzv = to->as<StateType>()->as<RealVectorStateSpace::StateType>(1);

  auto * interpolated_so2 = state->as<StateType>()->as<SO2StateSpace::StateType>(0);
  auto * interpolated_xyzv = state->as<StateType>()->as<RealVectorStateSpace::StateType>(1);

  auto from_se2 = se2_->allocState(); auto to_se2 = se2_->allocState();
  from_se2->as<SE2StateSpace::StateType>()->setXY(from_xyzv->values[0], from_xyzv->values[1]);
  from_se2->as<SE2StateSpace::StateType>()->setYaw(from_so2->value);
  to_se2->as<SE2StateSpace::StateType>()->setXY(to_xyzv->values[0], to_xyzv->values[1]);
  to_se2->as<SE2StateSpace::StateType>()->setYaw(to_so2->value);

  auto interpolated_se2 = se2_->allocState();  // This has X, Y , SO2

  if (se2_state_type_ == SE2StateType::SE2) {
    se2_->interpolate(from_se2, to_se2, t, interpolated_se2);
  } else if (se2_state_type_ == SE2StateType::DUBINS) {
    dubins_->interpolate(from_se2, to_se2, t, interpolated_se2);
  } else {
    reeds_sheep_->interpolate(from_se2, to_se2, t, interpolated_se2);
  }

  interpolated_so2->value = interpolated_se2->as<SE2StateSpace::StateType>()->getYaw();     // so2
  interpolated_xyzv->values[0] = interpolated_se2->as<SE2StateSpace::StateType>()->getX();  // x
  interpolated_xyzv->values[1] = interpolated_se2->as<SE2StateSpace::StateType>()->getY();  // y
  interpolated_xyzv->values[2] = (from_xyzv->values[2] + to_xyzv->values[2]) / 2.0;         // z
  interpolated_xyzv->values[3] = (from_xyzv->values[3] + to_xyzv->values[3]) / 2.0;         // v

  /*pcl::PointSurfel dubins_surfel, nearest_intermediate_surfel;
  dubins_surfel.x = state_dubins->getX();
  dubins_surfel.y = state_dubins->getY();
  dubins_surfel.z = from_z->values[0];

  // WARN , This is too expensive
  nearest_intermediate_surfel = vox_nav_utilities::getNearstPoint<
    pcl::PointSurfel,
    pcl::PointCloud<pcl::PointSurfel>::Ptr>(
    dubins_surfel,
    workspace_surfels_);
  state_z->values[0] = nearest_intermediate_surfel.z;*/

}

void ompl::base::ElevationStateSpace::printState(const State * state, std::ostream & out) const
{
  auto * so2 = state->as<StateType>()->as<SO2StateSpace::StateType>(0);
  auto * xyzv = state->as<StateType>()->as<RealVectorStateSpace::StateType>(1);
  so2_->printState(so2, out);
  real_vector_->printState(xyzv, out);
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
  cstate->setXYZV(out_sample.x, out_sample.y, out_sample.z, 0);
  cstate->setSO2(0);
  return true;
}

bool OctoCellValidStateSampler::sampleNear(
  ompl::base::State * state, const ompl::base::State * near,
  const double distance)
{
  auto * cstate = state->as<ompl::base::ElevationStateSpace::StateType>();
  auto * near_cstate = near->as<ompl::base::ElevationStateSpace::StateType>();
  const auto * so2 = near_cstate->as<ompl::base::SO2StateSpace::StateType>(0);
  const auto * xyzv = near_cstate->as<ompl::base::RealVectorStateSpace::StateType>(1);

  pcl::PointSurfel surfel;
  surfel.x = xyzv->values[0];
  surfel.y = xyzv->values[1];
  surfel.z = xyzv->values[2];

  auto out_sample = vox_nav_utilities::getNearstRPoints<
    pcl::PointSurfel,
    pcl::PointCloud<pcl::PointSurfel>::Ptr>(
    distance,
    surfel,
    search_area_surfels_);

  cstate->setXYZV(out_sample.x, out_sample.y, out_sample.z, 0);
  cstate->setSO2(0);

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
