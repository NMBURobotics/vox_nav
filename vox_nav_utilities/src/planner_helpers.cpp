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

#include <memory>
#include <string>
#include "vox_nav_utilities/planner_helpers.hpp"

namespace vox_nav_utilities
{

  geometry_msgs::msg::PoseStamped getNearstNode(
    const geometry_msgs::msg::PoseStamped & state,
    const std::shared_ptr<octomap::OcTree> & nodes_octree)
  {
    auto nearest_node_pose = state;
    double dist = INFINITY;
    for (auto it = nodes_octree->begin(),
      end = nodes_octree->end(); it != end; ++it)
    {
      if (nodes_octree->isNodeOccupied(*it)) {
        auto dist_to_crr_node = std::sqrt(
          std::pow(it.getCoordinate().x() - state.pose.position.x, 2) +
          std::pow(it.getCoordinate().y() - state.pose.position.y, 2) +
          std::pow(it.getCoordinate().z() - state.pose.position.z, 2));
        if (dist_to_crr_node < dist) {
          dist = dist_to_crr_node;
          nearest_node_pose.pose.position.x = it.getCoordinate().x();
          nearest_node_pose.pose.position.y = it.getCoordinate().y();
          nearest_node_pose.pose.position.z = it.getCoordinate().z();
        }
      }
    }
    return nearest_node_pose;
  }

  void initializeSelectedPlanner(
    ompl::base::PlannerPtr & planner,
    const std::string & selected_planner_name,
    const ompl::base::SpaceInformationPtr & si,
    const rclcpp::Logger logger)
  {
    if (selected_planner_name == std::string("PRMstar")) {
      planner = ompl::base::PlannerPtr(new ompl::geometric::PRMstar(si));
    } else if (selected_planner_name == std::string("LazyPRMstar")) {
      planner = ompl::base::PlannerPtr(new ompl::geometric::LazyPRMstar(si));
    } else if (selected_planner_name == std::string("RRTstar")) {
      planner = ompl::base::PlannerPtr(new ompl::geometric::RRTstar(si));
    } else if (selected_planner_name == std::string("RRTsharp")) {
      planner = ompl::base::PlannerPtr(new ompl::geometric::RRTsharp(si));
    } else if (selected_planner_name == std::string("RRTXstatic")) {
      planner = ompl::base::PlannerPtr(new ompl::geometric::RRTXstatic(si));
    } else if (selected_planner_name == std::string("InformedRRTstar")) {
      planner = ompl::base::PlannerPtr(new ompl::geometric::InformedRRTstar(si));
    } else if (selected_planner_name == std::string("BITstar")) {
      planner = ompl::base::PlannerPtr(new ompl::geometric::BITstar(si));
    } else if (selected_planner_name == std::string("ABITstar")) {
      planner = ompl::base::PlannerPtr(new ompl::geometric::ABITstar(si));
    } else if (selected_planner_name == std::string("AITstar")) {
      planner = ompl::base::PlannerPtr(new ompl::geometric::AITstar(si));
    } else if (selected_planner_name == std::string("CForest")) {
      planner = ompl::base::PlannerPtr(new ompl::geometric::CForest(si));
    } else if (selected_planner_name == std::string("LBTRRT")) {
      planner = ompl::base::PlannerPtr(new ompl::geometric::LBTRRT(si));
    } else if (selected_planner_name == std::string("SST")) {
      planner = ompl::base::PlannerPtr(new ompl::geometric::SST(si));
    } else if (selected_planner_name == std::string("TRRT")) {
      planner = ompl::base::PlannerPtr(new ompl::geometric::TRRT(si));
    } else if (selected_planner_name == std::string("SPARS")) {
      planner = ompl::base::PlannerPtr(new ompl::geometric::SPARS(si));
    } else if (selected_planner_name == std::string("SPARStwo")) {
      planner = ompl::base::PlannerPtr(new ompl::geometric::SPARStwo(si));
    } else if (selected_planner_name == std::string("FMT")) {
      planner = ompl::base::PlannerPtr(new ompl::geometric::FMT(si));
    } else if (selected_planner_name == std::string("AnytimePathShortening")) {
      planner = ompl::base::PlannerPtr(new ompl::geometric::AnytimePathShortening(si));
    } else {
      RCLCPP_WARN(
        logger,
        "Selected planner is not Found in available planners, using the default planner: RRTstar");
      planner = ompl::base::PlannerPtr(new ompl::geometric::RRTstar(si));
    }
  }


  pcl::PointSurfel poseMsg2PCLSurfel(const geometry_msgs::msg::PoseStamped & pose_stamped)
  {
    pcl::PointSurfel surfel;
    surfel.x = pose_stamped.pose.position.x;
    surfel.y = pose_stamped.pose.position.y;
    surfel.z = pose_stamped.pose.position.z;

    double normal_x, normal_y, normal_z;
    vox_nav_utilities::getRPYfromMsgQuaternion(
      pose_stamped.pose.orientation,
      normal_x,
      normal_y,
      normal_z);

    surfel.normal_x = normal_x;
    surfel.normal_y = normal_y;
    surfel.normal_z = normal_z;
    return surfel;
  }

  geometry_msgs::msg::PoseStamped PCLSurfel2PoseMsg(const pcl::PointSurfel & surfel)
  {
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.pose.position.x = surfel.x;
    pose_stamped.pose.position.y = surfel.y;
    pose_stamped.pose.position.z = surfel.z;
    pose_stamped.pose.orientation =
      vox_nav_utilities::getMsgQuaternionfromRPY(
      surfel.normal_x,
      surfel.normal_y,
      surfel.normal_z
      );

    return pose_stamped;
  }

  void determineValidNearestGoalStart(
    geometry_msgs::msg::PoseStamped & nearest_valid_start,
    geometry_msgs::msg::PoseStamped & nearest_valid_goal,
    const geometry_msgs::msg::PoseStamped & actual_start,
    const geometry_msgs::msg::PoseStamped & actual_goal,
    const pcl::PointCloud<pcl::PointSurfel>::Ptr & elevated_surfel_cloud
  )
  {
    pcl::PointSurfel start_nearest_surfel, goal_nearest_surfel;
    start_nearest_surfel = vox_nav_utilities::poseMsg2PCLSurfel(actual_start);
    goal_nearest_surfel = vox_nav_utilities::poseMsg2PCLSurfel(actual_goal);
    start_nearest_surfel = vox_nav_utilities::getNearstPoint<
      pcl::PointSurfel,
      pcl::PointCloud<pcl::PointSurfel>::Ptr>(
      start_nearest_surfel,
      elevated_surfel_cloud);

    goal_nearest_surfel = vox_nav_utilities::getNearstPoint<
      pcl::PointSurfel,
      pcl::PointCloud<pcl::PointSurfel>::Ptr>(
      goal_nearest_surfel,
      elevated_surfel_cloud);

    nearest_valid_start = vox_nav_utilities::PCLSurfel2PoseMsg(start_nearest_surfel);
    nearest_valid_goal = vox_nav_utilities::PCLSurfel2PoseMsg(goal_nearest_surfel);
  }
}  // namespace vox_nav_utilities
