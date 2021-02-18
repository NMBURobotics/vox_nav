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

#ifndef BOTANBOT_UTILITIES__PLANNER_BENCHMARKING_HPP_
#define BOTANBOT_UTILITIES__PLANNER_BENCHMARKING_HPP_
#pragma once

// ROS
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <botanbot_utilities/tf_helpers.hpp>
// PCL
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
// OMPL GEOMETRIC
#include <ompl/geometric/planners/sst/SST.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/BiTRRT.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/rrt/SORRTstar.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/SPARStwo.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/cforest/CForest.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/OptimizationObjective.h>
// OMPL CONTROL
#include <ompl/control/ControlSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/sst/SST.h>
#include <ompl/control/planners/est/EST.h>
#include <ompl/control/planners/syclop/SyclopRRT.h>
#include <ompl/control/planners/syclop/SyclopEST.h>
#include <ompl/control/planners/pdst/PDST.h>
#include <ompl/control/planners/syclop/GridDecomposition.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>
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

namespace botanbot_utilities
{

struct GroundRobotPose
{
  double x;
  double y;
  double z;
  double yaw;
  GroundRobotPose()
  : x(0.0),
    y(0.0),
    z(0.0),
    yaw(0.0)
  {}
};

struct SEBounds
{
  double minx;
  double maxx;
  double miny;
  double maxy;
  double minz;
  double maxz;
  double minyaw;
  double maxyaw;
  SEBounds()
  : minx(0.0),
    maxx(0.0),
    miny(0.0),
    maxy(0.0),
    minz(0.0),
    maxz(0.0),
    minyaw(0.0),
    maxyaw(0.0)
  {}
};

class PlannerBenchMarking : public rclcpp::Node
{
private:
  std::string selected_state_space_; // se2 ? se3
  SEBounds se_bounds_; // struct for keeping things clean
  std::shared_ptr<ompl::base::RealVectorBounds> ompl_se_bounds_;
  ompl::base::StateSpacePtr state_space_;

  std::vector<std::string> selected_planners_;
  std::string octomap_topic_;
  std::string results_output_file_;
  double octomap_voxel_size_;
  double planner_timeout_;
  // Only used for REEDS or DUBINS
  double min_turning_radius_;
  double goal_tolerance_;
  int interpolation_parameter_;
  int num_benchmark_runs_;
  int max_memory_;

  // We only need to creae a FLC cotomap collision from
  // octomap once, because this is static map
  std::once_flag fcl_tree_from_octomap_once_;
  // global mutex to guard octomap
  std::mutex octomap_mutex_;

  GroundRobotPose start_;
  GroundRobotPose goal_;
  geometry_msgs::msg::Vector3 robot_body_dimensions_;

  std::shared_ptr<fcl::CollisionObject> robot_collision_object_;
  std::shared_ptr<fcl::OcTree> fcl_octree_;
  std::shared_ptr<fcl::CollisionObject> fcl_octree_collision_object_;

  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_subscriber_;
  octomap_msgs::msg::Octomap::ConstSharedPtr octomap_msg_;

public:
  double is_octomap_ready_;
  /**
  * @brief Construct a new Planner Bench Marking object
  *
  */
  PlannerBenchMarking();

  /**
   * @brief Destroy the Planner Bench Marking object
   *
   */
  ~PlannerBenchMarking();

  /**
   * @brief perfrom actual benchmark
   *
   */
  void doBenchMarking();

  /**
  * @brief Callback to subscribe ang get octomap
  *
  * @param octomap
  */
  void octomapCallback(const octomap_msgs::msg::Octomap::ConstSharedPtr msg);

  /**
  * @brief
  *
  * @param state
  * @return true
  * @return false
  */
  bool isStateValidSE2(const ompl::base::State * state);

  /**
    * @brief
    *
    * @param state
    * @return true
    * @return false
    */
  bool isStateValidSE3(const ompl::base::State * state);
};
}  // namespace botanbot_utilities

#endif  // BOTANBOT_UTILITIES__PLANNER_BENCHMARKING_HPP_
