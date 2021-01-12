// Copyright (c) 2020 Fetullah Atas
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

#ifndef botanbot_planning__GPS_WAYPOINT_FOLLOWER_CLIENT_HPP_
#define botanbot_planning__GPS_WAYPOINT_FOLLOWER_CLIENT_HPP_
// ROS
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "visualization_msgs/msg/marker_array.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
// OCTOMAP
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <octomap/octomap_utils.h>
#include <message_filters/subscriber.h>
// OMPL
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/samplers/MaximizeClearanceValidStateSampler.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include "ompl/geometric/planners/cforest/CForest.h"
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/config.h>
#include <iostream>
//FCL
#include "fcl/config.h"
#include "fcl/octree.h"
#include "fcl/traversal/traversal_node_octree.h"
#include "fcl/collision.h"
#include "fcl/broadphase/broadphase.h"
#include "fcl/math/transform.h"
// PCL
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/**
 * @brief
 *
 */
namespace botanbot_planning
{

/**
 * @brief
 *
 */
class BotanbotSE3Planning : public rclcpp::Node
{
public:
/**
 * @brief Construct a new Botanbot O M P L Experimental object
 *
 */
  BotanbotSE3Planning();

/**
 * @brief Destroy the Botanbot O M P L Experimental object
 *
 */
  ~BotanbotSE3Planning();

  /**
   * @brief
   *
   * @param si
   * @return ompl::base::OptimizationObjectivePtr
   */
  ompl::base::OptimizationObjectivePtr get2(const ompl::base::SpaceInformationPtr & si);

  /**
   * @brief
   *
   * @param state
   * @return true
   * @return false
   */
  bool isStateValid(const ompl::base::State * state);

  /**
   * @brief
   *
   */
  void plan();

protected:
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vis_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr traj_pub_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr octomap_pointcloud_pub_;

  std::shared_ptr<fcl::CollisionGeometry> robot_collision_geometry_;
  std::shared_ptr<fcl::CollisionObject> robot_collision_object_;
  std::shared_ptr<octomap::OcTree> octomap_octree_;
  std::shared_ptr<fcl::OcTree> fcl_octree_;
  std::shared_ptr<fcl::CollisionObject> fcl_octree_collision_object_;
};
}  // namespace botanbot_planning

#endif  // botanbot_planning__GPS_WAYPOINT_FOLLOWER_CLIENT_HPP_
