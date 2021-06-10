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


ompl::base::State * ompl::base::ElevationStateSpace::allocState() const
{
  auto * state = new StateType();
  allocStateComponents(state);
  return state;
}

void ompl::base::ElevationStateSpace::freeState(State * state) const
{
  CompoundStateSpace::freeState(state);
}

double ompl::base::ElevationStateSpace::distance(const State * state1, const State * state2) const
{
  const auto * start = state1->as<ElevationStateSpace::StateType>();
  const auto * goal = state2->as<ElevationStateSpace::StateType>();

  const auto * start_se2 = start->as<DubinsStateSpace::StateType>(0);
  const auto * start_z = start->as<RealVectorStateSpace::StateType>(1);

  const auto * goal_se2 = goal->as<DubinsStateSpace::StateType>(0);
  const auto * goal_z = goal->as<RealVectorStateSpace::StateType>(1);

  auto se2_state_space = std::make_shared<ompl::base::DubinsStateSpace>();


  return se2_state_space->distance(start_se2, goal_se2);
}

void ompl::base::ElevationStateSpace::interpolate(
  const State * from, const State * to, double t,
  State * state) const
{
  const auto * start = from->as<ElevationStateSpace::StateType>();
  const auto * goal = to->as<ElevationStateSpace::StateType>();
  auto * _state = state->as<ElevationStateSpace::StateType>();

  const auto * start_se2 = start->as<DubinsStateSpace::StateType>(0);
  const auto * start_z = start->as<RealVectorStateSpace::StateType>(1);

  const auto * goal_se2 = goal->as<DubinsStateSpace::StateType>(0);
  const auto * goal_z = goal->as<RealVectorStateSpace::StateType>(1);

  auto * _state_se2 = _state->as<DubinsStateSpace::StateType>(0);

  auto se2_state_space = std::make_shared<ompl::base::DubinsStateSpace>();

  se2_state_space->interpolate(start_se2, goal_se2, t, _state_se2);

  
}
