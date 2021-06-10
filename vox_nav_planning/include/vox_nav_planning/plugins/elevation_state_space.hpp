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

#ifndef VOX_NAV_PLANNING__PLUGINS__ELEVATION_STATE_SPACE_HPP_
#define VOX_NAV_PLANNING__PLUGINS__ELEVATION_STATE_SPACE_HPP_

#include "ompl/base/spaces/DubinsStateSpace.h"
#include "ompl/base/spaces/SE2StateSpace.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"

namespace ompl
{
namespace base
{
class ElevationStateSpace : public CompoundStateSpace
{
public:
  class StateType : public CompoundStateSpace::StateType
  {
public:
    StateType() = default;


    DubinsStateSpace::StateType * getSE2()
    {
      return as<DubinsStateSpace::StateType>(0);
    }

    RealVectorStateSpace::StateType * getZ()
    {
      return as<RealVectorStateSpace::StateType>(1);
    }

    void setSE2(double x, double y, double yaw)
    {
      as<DubinsStateSpace::StateType>(0)->setXY(x, y);
      as<DubinsStateSpace::StateType>(0)->setYaw(yaw);
    }

    void setZ(double z)
    {
      as<RealVectorStateSpace::StateType>(1)->values[0] = z;
    }
  };

  ElevationStateSpace()
  {
    setName("Elevation" + getName());
    type_ = 31; // why ?
    addSubspace(std::make_shared<DubinsStateSpace>(), 1.0);
    addSubspace(std::make_shared<RealVectorStateSpace>(1), 1.0);
    lock();
  }

  ~ElevationStateSpace() override = default;

  void setBounds(const RealVectorBounds & se2_bounds, const RealVectorBounds & z_bounds)
  {
    as<DubinsStateSpace>(0)->setBounds(se2_bounds);
    as<RealVectorStateSpace>(1)->setBounds(z_bounds);
  }

  State * allocState() const override;

  void freeState(State * state) const override;

  double distance(const State * state1, const State * state2) const override;

  void interpolate(const State * from, const State * to, double t, State * state) const override;
 
};
}  // namespace base
}  // namespace ompl

#endif  // VOX_NAV_PLANNING__PLUGINS__ELEVATION_STATE_SPACE_HPP_
