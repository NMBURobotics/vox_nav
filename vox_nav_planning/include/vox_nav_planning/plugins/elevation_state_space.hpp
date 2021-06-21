// Copyright (c) 2021 Fetullah Atas, Norwegian University of Life Sciences
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
#include "ompl/base/spaces/ReedsSheppStateSpace.h"
#include "ompl/base/spaces/SE2StateSpace.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "vox_nav_planning/planner_core.hpp"
#include <pcl/octree/octree_search.h>
#include <pcl/filters/random_sample.h>

namespace ompl
{
  namespace base
  {
    /**
     * @brief Class desgined to define a optimization objective BASED ON
     * OCTOMAP'S cell values.
     */
    class OctoCostOptimizationObjective : public StateCostIntegralObjective
    {
    public:
      /**
       * @brief Construct a new OctoCostOptimization Objective object
       *
       * @param si space information
       * @param tree shared ptr to octomap octree
      */
      OctoCostOptimizationObjective(
        const SpaceInformationPtr & si,
        const std::shared_ptr<octomap::OcTree> & elevated_surfels_octree);

      /**
       * @brief Destroy the OctoCostOptimization Objective object
       *
       */
      ~OctoCostOptimizationObjective();

      /**
       * @brief get amount of the cost assigned to this state
       *
       * @param s state pointer
       * @return Cost
       */
      Cost stateCost(const State * s) const override;

    protected:
      // Octree where the elevated surfesl are stored in
      std::shared_ptr<octomap::OcTree> elevated_surfels_octree_;
    };

    class ElevationStateSpace : public CompoundStateSpace
    {
    public:
      class StateType : public CompoundStateSpace::StateType
      {
      public:
        StateType() = default;

        SE2StateSpace::StateType * getSE2()
        {
          return as<SE2StateSpace::StateType>(0);
        }

        RealVectorStateSpace::StateType * getZ()
        {
          return as<RealVectorStateSpace::StateType>(1);
        }

        void setSE2(double x, double y, double yaw)
        {
          as<SE2StateSpace::StateType>(0)->setXY(x, y);
          as<SE2StateSpace::StateType>(0)->setYaw(yaw);
        }

        void setZ(double z)
        {
          as<RealVectorStateSpace::StateType>(1)->values[0] = z;
        }
      };

      enum SE2StateType
      {
        SE2,
        DUBINS,
        REDDSSHEEP
      };

      ElevationStateSpace(
        const geometry_msgs::msg::PoseStamped start,
        const geometry_msgs::msg::PoseStamped goal,
        const SE2StateType state_type,
        const geometry_msgs::msg::PoseArray::SharedPtr & elevated_surfels_poses,
        double turningRadius = 1.0,
        bool isSymmetric = false);

      ~ElevationStateSpace() override = default;

      void setBounds(
        const RealVectorBounds & se2_bounds,
        const RealVectorBounds & z_bounds);

      const RealVectorBounds getBounds() const;

      State * allocState() const override;

      void freeState(State * state) const override;

      double distance(
        const State * state1,
        const State * state2) const override;

      void  interpolate(
        const State * from, const State * to, double t,
        State * state) const override;

    protected:
      rclcpp::Logger logger_{rclcpp::get_logger("elevation_planner_utils")};
      geometry_msgs::msg::PoseArray elevated_surfels_poses_;
      pcl::PointCloud<pcl::PointSurfel>::Ptr workspace_surfels_;
      SE2StateType se2_state_type_;

      std::shared_ptr<DubinsStateSpace> dubins_;
      std::shared_ptr<ReedsSheppStateSpace> reeds_sheep_;
      std::shared_ptr<SE2StateSpace> se2_;

      double rho_;
      bool isSymmetric_;
    };

    class OctoCellValidStateSampler : public ValidStateSampler
    {
    public:
      /**
       * @brief Construct a new Octo Cell State Sampler object
       *
       * @param si
       * @param tree
       */
      OctoCellValidStateSampler(
        const ompl::base::SpaceInformationPtr & si,
        const geometry_msgs::msg::PoseStamped start,
        const geometry_msgs::msg::PoseStamped goal,
        const std::shared_ptr<fcl::CollisionObject> & robot_collision_object,
        const std::shared_ptr<fcl::CollisionObject> & original_octomap_collision_object,
        const geometry_msgs::msg::PoseArray::SharedPtr & elevated_surfels_poses);

      /**
       * @brief
       *
       * @param state
       * @return true
       * @return false
       */
      bool sample(ompl::base::State * state) override;

      /**
       * @brief
       *
       * @param state
       * @param near
       * @param distance
       * @return true
       * @return false
       */
      bool sampleNear(
        ompl::base::State * state,
        const ompl::base::State * near,
        const double distance) override;

      /**
       * @brief
       *
       * @param start
       * @param goal
       */
      void updateSearchArea(
        const geometry_msgs::msg::PoseStamped start,
        const geometry_msgs::msg::PoseStamped goal);

      bool  isStateValid(const ompl::base::State * state);

    protected:
      rclcpp::Logger logger_{rclcpp::get_logger("OctoCellValidStateSampler")};
      geometry_msgs::msg::PoseArray elevated_surfels_poses_;
      pcl::PointCloud<pcl::PointSurfel>::Ptr workspace_surfels_;
      pcl::PointCloud<pcl::PointSurfel>::Ptr search_area_surfels_;
      std::shared_ptr<fcl::CollisionObject> robot_collision_object_;
      std::shared_ptr<fcl::CollisionObject> original_octomap_collision_object_;
      std::discrete_distribution<> distrubutions_;
    };

  } // namespace base
}  // namespace ompl

#endif  // VOX_NAV_PLANNING__PLUGINS__ELEVATION_STATE_SPACE_HPP_
