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

#ifndef VOX_NAV_PLANNING__PLUGINS__DUBINS_3D_STATE_SPACE_HPP_
#define VOX_NAV_PLANNING__PLUGINS__DUBINS_3D_STATE_SPACE_HPP_

#include "ompl/base/spaces/DubinsStateSpace.h"
#include "ompl/base/spaces/SE3StateSpace.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "vox_nav_planning/planner_core.hpp"
#include <pcl/octree/octree_search.h>
#include <pcl/filters/random_sample.h>


namespace ompl
{
  namespace base
  {
    class Dubins3DStateSpace : public CompoundStateSpace
    {
    public:
      enum HorizotalPathSegmentType
      {
        DUBINS_LEFT = 0,
        DUBINS_STRAIGHT = 1,
        DUBINS_RIGHT = 2
      };
      enum VerticalPathSegmentType
      {
        DUBINS_DOWN = 0,
        DUBINS_UP = 2
      };

      static const HorizotalPathSegmentType dubinsHorizontalPathType[6][3];
      static const VerticalPathSegmentType dubinsVerticalPathType[6][3];

      class Dubins3DPath
      {
      public:
        Dubins3DPath(
          const HorizotalPathSegmentType * horizontal_type = dubinsHorizontalPathType[0],
          const VerticalPathSegmentType * vertical_type = dubinsVerticalPathType[0],
          double t = 0.0,
          double p = std::numeric_limits<double>::max(),
          double q = 0.0)
        : horizontal_type_(horizontal_type),
          vertical_type_(vertical_type)
        {
          horizonal_length_[0] = t;
          horizonal_length_[1] = p;
          horizonal_length_[2] = q;

          vertical_length_[0] = t;
          vertical_length_[1] = p;
          vertical_length_[2] = q;

          assert(t >= 0.);
          assert(p >= 0.);
          assert(q >= 0.);
        }
        double horizontal_length() const
        {
          return horizonal_length_[0] + horizonal_length_[1] + horizonal_length_[2];
        }
        double vertical_length() const
        {
          return vertical_length_[0] + vertical_length_[1] + vertical_length_[2];
        }

        const HorizotalPathSegmentType * horizontal_type_;
        const VerticalPathSegmentType * vertical_type_;

        double horizonal_length_[3];
        double vertical_length_[3];

        bool reverse_{false};
      };

      class StateType : public CompoundStateSpace::StateType
      {
      public:
        StateType() = default;


        DubinsStateSpace::StateType * getHorizontalDubins()
        {
          return as<DubinsStateSpace::StateType>(0);
        }

        DubinsStateSpace::StateType * getVerticalDubins()
        {
          return as<DubinsStateSpace::StateType>(1);
        }

        void setHorizontalDubins(double x, double y, double yaw)
        {
          as<DubinsStateSpace::StateType>(0)->setXY(x, y);
          as<DubinsStateSpace::StateType>(0)->setYaw(yaw);
        }

        void setVerticalDubins(double x, double y, double yaw)
        {
          as<DubinsStateSpace::StateType>(1)->setXY(x, y);
          as<DubinsStateSpace::StateType>(1)->setYaw(yaw);
        }
      };

      Dubins3DStateSpace(double turningRadius = 1.0, bool isSymmetric = false);

      ~Dubins3DStateSpace() override = default;

      void setBounds(
        const RealVectorBounds & horizontal_dubins_bounds,
        const RealVectorBounds & vertical_dubins_bounds);

      const RealVectorBounds getMergedBounds() const;

      State * allocState() const override;

      void freeState(State * state) const override;

      double distance(
        const State * state1,
        const State * state2) const override;

      void  interpolate(
        const State * from, const State * to, double t,
        State * state) const override;

    protected:
      std::shared_ptr<DubinsStateSpace> horizontal_dubins_;
      std::shared_ptr<DubinsStateSpace> vertical_dubins_;

      double rho_;
      bool isSymmetric_;
    };
  }   // namespace base
}   // namespace ompl

#endif  // VOX_NAV_PLANNING__PLUGINS__DUBINS_3D_STATE_SPACE_HPP_
