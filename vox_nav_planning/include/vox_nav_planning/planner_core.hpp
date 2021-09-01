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

#ifndef VOX_NAV_PLANNING__PLANNER_CORE_HPP_
#define VOX_NAV_PLANNING__PLANNER_CORE_HPP_
#pragma once

// ROS
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/client.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vox_nav_utilities/tf_helpers.hpp>
#include <vox_nav_utilities/pcl_helpers.hpp>
#include <vox_nav_utilities/planner_helpers.hpp>
#include <vox_nav_msgs/srv/get_maps_and_surfels.hpp>
// PCL
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
// OMPL GEOMETRIC
#include <ompl/geometric/planners/fmt/BFMT.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/sst/SST.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/prm/SPARS.h>
#include <ompl/geometric/planners/prm/SPARStwo.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/AnytimePathShortening.h>
#include <ompl/geometric/planners/cforest/CForest.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/informedtrees/ABITstar.h>
#include <ompl/geometric/planners/informedtrees/AITstar.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/OptimizationObjective.h>
// OMPL BASE
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/samplers/MaximizeClearanceValidStateSampler.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/base/StateSampler.h>
// OCTOMAP
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <octomap/octomap_utils.h>
// FCL
#include <fcl/config.h>
#include <fcl/octree.h>
#include <fcl/traversal/traversal_node_octree.h>
#include <fcl/collision.h>
#include <fcl/broadphase/broadphase.h>
#include <fcl/math/transform.h>
// STL
#include <string>
#include <iostream>
#include <memory>
#include <vector>

namespace vox_nav_planning
{

/**
 * @brief Base class for creating a planner plugins
 *
 */
  class PlannerCore
  {
  public:
    using Ptr = std::shared_ptr<PlannerCore>;
    /**
     * @brief Construct a new Planner Core object
     *
     */
    PlannerCore() {}

    /**
     * @brief Destroy the Planner Core object
     *
     */
    virtual ~PlannerCore() {}

    /**
     * @brief
     *
     * @param parent
     * @param plugin_name
     */
    virtual void initialize(
      rclcpp::Node * parent,
      const std::string & plugin_name) = 0;

    /**
     * @brief Method create the plan from a starting and ending goal.
     *
     * @param start The starting pose of the robot
     * @param goal  The goal pose of the robot
     * @return std::vector<geometry_msgs::msg::PoseStamped>   The sequence of poses to get from start to goal, if any
     */
    virtual std::vector<geometry_msgs::msg::PoseStamped> createPlan(
      const geometry_msgs::msg::PoseStamped & start,
      const geometry_msgs::msg::PoseStamped & goal) = 0;

    /**
    * @brief
    *
    * @param state
    * @return true
    * @return false
    */
    virtual bool isStateValid(const ompl::base::State * state) = 0;

/**
 * @brief Get the Overlayed Start and Goal poses, only x and y are provided for goal ,
 * but internally planner finds closest valid node on octomap and reassigns goal to this pose
 *
 * @return std::vector<geometry_msgs::msg::PoseStamped>
 */
    virtual std::vector<geometry_msgs::msg::PoseStamped> getOverlayedStartandGoal() = 0;

    /**
     * @brief
     *
     */
    virtual void setupMap() = 0;

  protected:
    rclcpp::Client<vox_nav_msgs::srv::GetMapsAndSurfels>::SharedPtr get_maps_and_surfels_client_;
    rclcpp::Node::SharedPtr get_maps_and_surfels_client_node_;
    // octomap acquired from original PCD map
    std::shared_ptr<octomap::OcTree> original_octomap_octree_;
    std::shared_ptr<fcl::CollisionObject> original_octomap_collision_object_;
    std::shared_ptr<fcl::CollisionObject> robot_collision_object_;
    std::shared_ptr<fcl::CollisionObject> robot_collision_object_minimal_;
    ompl::geometric::SimpleSetupPtr simple_setup_;
    // to ensure safety when accessing global var curr_frame_
    std::mutex global_mutex_;
    // the topic to subscribe in order capture a frame
    std::string planner_name_;
    // Better t keep this parameter consistent with map_server, 0.2 is a OK default fo this
    double octomap_voxel_size_;
    // related to density of created path
    int interpolation_parameter_;
    // max time the planner can spend before coming up with a solution
    double planner_timeout_;
    // global mutex to guard octomap
    std::mutex octomap_mutex_;
    volatile bool is_map_ready_;
  };
}  // namespace vox_nav_planning
#endif  // VOX_NAV_PLANNING__PLANNER_CORE_HPP_
