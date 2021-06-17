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


#include "vox_nav_planning/plugins/dubins_3d_state_space.hpp"
#include "ompl/tools/config/MagicConstants.h"

using namespace ompl::base;

Dubins3DStateSpace::Dubins3DStateSpace(
  double turningRadius,
  bool isSymmetric)
: rho_(turningRadius),
  isSymmetric_(isSymmetric)
{
  setName("Dubins3D" + getName());
  type_ = 32;
  addSubspace(std::make_shared<DubinsStateSpace>(turningRadius), 1.0);
  addSubspace(std::make_shared<DubinsStateSpace>(turningRadius), 1.0);
  lock();

  horizontal_dubins_ = std::make_shared<ompl::base::DubinsStateSpace>(turningRadius, isSymmetric_);
  vertical_dubins_ = std::make_shared<ompl::base::DubinsStateSpace>(turningRadius, isSymmetric_);
}

void Dubins3DStateSpace::setBounds(
  const RealVectorBounds & horizontal_dubins_bounds,
  const RealVectorBounds & vertical_dubins_bounds)
{
  as<DubinsStateSpace>(0)->setBounds(horizontal_dubins_bounds);
  as<DubinsStateSpace>(1)->setBounds(vertical_dubins_bounds);

  horizontal_dubins_->setBounds(horizontal_dubins_bounds);
  vertical_dubins_->setBounds(vertical_dubins_bounds);
}

const RealVectorBounds Dubins3DStateSpace::getMergedBounds() const
{
  auto merged_bounds = std::make_shared<ompl::base::RealVectorBounds>(4);

  merged_bounds->setLow(0, as<DubinsStateSpace>(0)->getBounds().low[0]);
  merged_bounds->setHigh(0, as<DubinsStateSpace>(0)->getBounds().high[0]);
  merged_bounds->setLow(1, as<DubinsStateSpace>(0)->getBounds().low[1]);
  merged_bounds->setHigh(1, as<DubinsStateSpace>(0)->getBounds().high[1]);

  merged_bounds->setLow(2, as<DubinsStateSpace>(1)->getBounds().low[0]);
  merged_bounds->setHigh(2, as<DubinsStateSpace>(1)->getBounds().high[0]);
  merged_bounds->setLow(3, as<DubinsStateSpace>(1)->getBounds().low[1]);
  merged_bounds->setHigh(3, as<DubinsStateSpace>(1)->getBounds().high[1]);

  return *merged_bounds;
}

State * Dubins3DStateSpace::allocState() const
{
  auto * state = new StateType();
  allocStateComponents(state);
  return state;
}

void Dubins3DStateSpace::freeState(State * state) const
{
  CompoundStateSpace::freeState(state);
}

double ompl::base::Dubins3DStateSpace::distance(
  const State * state1,
  const State * state2) const
{
  //  cast to our type
  const auto * start = state1->as<StateType>();
  const auto * goal = state2->as<StateType>();

  const auto * start_horizontal = start->as<DubinsStateSpace::StateType>(0);
  const auto * start_vertical = start->as<DubinsStateSpace::StateType>(1);

  const auto * goal_horizontal = goal->as<DubinsStateSpace::StateType>(0);
  const auto * goal_vertical = goal->as<DubinsStateSpace::StateType>(1);

  auto horizontal_dist = horizontal_dubins_->dubins(start_horizontal, goal_horizontal).length();
  auto vertical_dist = vertical_dubins_->dubins(start_vertical, goal_vertical).length();

  return horizontal_dist;
}

void ompl::base::Dubins3DStateSpace::interpolate(
  const State * from,
  const State * to,
  double t,
  State * state) const
{
  //  cast to our type
  const auto * start = from->as<StateType>();
  const auto * goal = to->as<StateType>();
  auto * state_dubins_3d = state->as<StateType>();

  const auto * start_horizontal = start->as<DubinsStateSpace::StateType>(0);
  const auto * start_vertical = start->as<DubinsStateSpace::StateType>(1);
  const auto * goal_horizontal = goal->as<DubinsStateSpace::StateType>(0);
  const auto * goal_vertical = goal->as<DubinsStateSpace::StateType>(1);
  auto * state_horizontal = state_dubins_3d->as<DubinsStateSpace::StateType>(0);
  auto * state_vertical = state_dubins_3d->as<DubinsStateSpace::StateType>(1);

  horizontal_dubins_->interpolate(start_horizontal, goal_horizontal, t, state_horizontal);
  vertical_dubins_->interpolate(start_vertical, goal_vertical, t, state_vertical);
}
