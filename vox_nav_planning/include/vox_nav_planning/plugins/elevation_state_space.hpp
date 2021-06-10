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
#include "vox_nav_planning/planner_core.hpp"
#include <pcl/octree/octree_search.h>
#include <pcl/filters/random_sample.h>

namespace ompl
{
  namespace base
  {

    class ElevationStateSampler : public StateSampler
    {
    public:
      ElevationStateSampler(
        const StateSpace * space,
        const pcl::PointCloud<pcl::PointSurfel>::Ptr workspace_surfels,
        const pcl::PointCloud<pcl::PointSurfel>::Ptr search_area_surfels);

      void sampleUniform(State * state) override;

      void sampleUniformNear(State * state, const State * near, double distance) override;

      void sampleGaussian(State * state, const State * mean, double stdDev) override;

    protected:
      pcl::PointCloud<pcl::PointSurfel>::Ptr workspace_surfels_;
      pcl::PointCloud<pcl::PointSurfel>::Ptr search_area_surfels_;
    };


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

      ElevationStateSpace(
        const geometry_msgs::msg::PoseStamped start,
        const geometry_msgs::msg::PoseStamped goal,
        const geometry_msgs::msg::PoseArray::SharedPtr & elevated_surfels_poses);

      ~ElevationStateSpace() override = default;

      void setBounds(const RealVectorBounds & se2_bounds, const RealVectorBounds & z_bounds);

      const RealVectorBounds getBounds() const;

      State * allocState() const override;

      void freeState(State * state) const override;

      StateSamplerPtr allocDefaultStateSampler() const override;

      double distance(
        const State * state1,
        const State * state2) const override;

      void  interpolate(
        const State * from, const State * to, double t,
        State * state) const override;

      void registerProjections() override;

      void updateSearchArea(
        const geometry_msgs::msg::PoseStamped start,
        const geometry_msgs::msg::PoseStamped goal);

    protected:
      rclcpp::Logger logger_{rclcpp::get_logger("elevation_planner_utils")};
      geometry_msgs::msg::PoseArray elevated_surfels_poses_;
      pcl::PointCloud<pcl::PointSurfel>::Ptr workspace_surfels_;
      pcl::PointCloud<pcl::PointSurfel>::Ptr search_area_surfels_;
      std::shared_ptr<fcl::CollisionObject> robot_collision_object_;
      std::shared_ptr<fcl::CollisionObject> original_octomap_collision_object_;
    };
  } // namespace base
}  // namespace ompl

#endif  // VOX_NAV_PLANNING__PLUGINS__ELEVATION_STATE_SPACE_HPP_
