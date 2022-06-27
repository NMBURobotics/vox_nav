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

#ifndef VOX_NAV_UTILITIES__PLANNER_HELPERS_HPP_
#define VOX_NAV_UTILITIES__PLANNER_HELPERS_HPP_

#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "vox_nav_utilities/tf_helpers.hpp"
#include "vox_nav_utilities/pcl_helpers.hpp"
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


// OCTOMAP
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <octomap/octomap_utils.h>
// PCL
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>

namespace vox_nav_utilities
{

/**
 * @brief Get the Nearst Node to given state object
 *
 * @param state
 * @param color_octomap_octree
 * @return geometry_msgs::msg::PoseStamped
 */
  geometry_msgs::msg::PoseStamped getNearstNode(
    const geometry_msgs::msg::PoseStamped & state,
    const std::shared_ptr<octomap::OcTree> & nodes_octree);

/**
 * @brief
 *
 * @param planner
 * @param selected_planner_name
 * @param si
 * @param logger
 */
  void initializeSelectedPlanner(
    ompl::base::PlannerPtr & planner,
    const std::string & selected_planner_name,
    const ompl::base::SpaceInformationPtr & si,
    const rclcpp::Logger logger);

  /**
   * @brief populate pcl surfel from geometry msgs Pose
   *
   * @param pose
   * @return pcl::PointSurfel
   */
  pcl::PointSurfel poseMsg2PCLSurfel(const geometry_msgs::msg::PoseStamped & pose_stamped);

  /**
   * @brief
   *
   * @param surfel
   * @return geometry_msgs::msg::PoseStamped
   */
  geometry_msgs::msg::PoseStamped PCLSurfel2PoseMsg(const pcl::PointSurfel & surfel);

/**
 * @brief
 *
 * @param nearest_valid_start
 * @param nearest_valid_goal
 * @param actual_start
 * @param actual_goal
 * @param elevated_surfel_cloud
 */
  void determineValidNearestGoalStart(
    geometry_msgs::msg::PoseStamped & nearest_valid_start,
    geometry_msgs::msg::PoseStamped & nearest_valid_goal,
    const geometry_msgs::msg::PoseStamped & actual_start,
    const geometry_msgs::msg::PoseStamped & actual_goal,
    const pcl::PointCloud<pcl::PointSurfel>::Ptr & elevated_surfel_cloud
  );

  /**
   * @brief
   *
   * @param poses
   * @param surfels
   */
  void fillSurfelsfromMsgPoses(
    const geometry_msgs::msg::PoseArray & poses,
    pcl::PointCloud<pcl::PointSurfel>::Ptr & surfels);

  /**
  * @brief
  *
  * @param poses
  * @param surfels
  */
  void fillMsgPosesfromSurfels(
    geometry_msgs::msg::PoseArray & poses,
    const pcl::PointCloud<pcl::PointSurfel>::Ptr & surfels);

  /**
   * @brief
   *
   * @param supervoxel_adjacency
   * @param header
   * @param marker_array
   */
  void fillSuperVoxelMarkersfromAdjacency(
    const std::map<std::uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr> & supervoxel_clusters,
    const std::multimap<std::uint32_t, std::uint32_t> & supervoxel_adjacency,
    const std_msgs::msg::Header & header,
    visualization_msgs::msg::MarkerArray & marker_array
  );

  /**
   * @brief
   *
   * @param a
   * @param b
   * @return geometry_msgs::msg::PoseStamped
   */
  geometry_msgs::msg::PoseStamped getLinearInterpolatedPose(
    const geometry_msgs::msg::PoseStamped a,
    const geometry_msgs::msg::PoseStamped b
  );

}  // namespace vox_nav_utilities

#endif  // VOX_NAV_UTILITIES__PLANNER_HELPERS_HPP_
