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

  void fillSurfelsfromMsgPoses(
    const geometry_msgs::msg::PoseArray & poses,
    pcl::PointCloud<pcl::PointSurfel>::Ptr & surfels)
  {
    for (auto && i : poses.poses) {
      pcl::PointSurfel surfel;
      surfel.x = i.position.x;
      surfel.y = i.position.y;
      surfel.z = i.position.z;
      double r, p, y;
      vox_nav_utilities::getRPYfromMsgQuaternion(i.orientation, r, p, y);
      surfel.normal_x = r;
      surfel.normal_y = p;
      surfel.normal_z = y;
      surfels->points.push_back(surfel);
    }
  }

  void fillMsgPosesfromSurfels(
    geometry_msgs::msg::PoseArray & poses,
    const pcl::PointCloud<pcl::PointSurfel>::Ptr & surfels)
  {
    for (auto && i : surfels->points) {
      geometry_msgs::msg::Pose pose;
      pose.position.x = i.x;
      pose.position.y = i.y;
      pose.position.z = i.z;
      pose.orientation =
        vox_nav_utilities::getMsgQuaternionfromRPY(i.normal_x, i.normal_y, i.normal_z);
      poses.poses.push_back(pose);
    }
  }

  void fillSuperVoxelMarkersfromAdjacency(
    const std::map<std::uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr> & supervoxel_clusters,
    const std::multimap<std::uint32_t, std::uint32_t> & supervoxel_adjacency,
    const std_msgs::msg::Header & header,
    visualization_msgs::msg::MarkerArray & marker_array)
  {
    int index = 0;
    // To make a graph of the supervoxel adjacency,
    // we need to iterate through the supervoxel adjacency multimap
    for (auto label_itr = supervoxel_adjacency.cbegin();
      label_itr != supervoxel_adjacency.cend(); )
    {
      // First get the label
      std::uint32_t supervoxel_label = label_itr->first;
      // Now get the supervoxel corresponding to the label
      auto supervoxel = supervoxel_clusters.at(supervoxel_label);

      visualization_msgs::msg::Marker line_strip;
      line_strip.header = header;
      line_strip.ns = "supervoxel_markers_ns";
      line_strip.id = index;
      line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
      line_strip.action = visualization_msgs::msg::Marker::ADD;
      line_strip.scale.x = 0.1;
      geometry_msgs::msg::Point point;
      point.x = supervoxel->centroid_.x;
      point.y = supervoxel->centroid_.y;
      point.z = supervoxel->centroid_.z;
      std_msgs::msg::ColorRGBA yellow_color;
      yellow_color.r = 1.0;
      yellow_color.g = 1.0;
      yellow_color.a = 0.4;
      line_strip.points.push_back(point);
      line_strip.colors.push_back(yellow_color);

      visualization_msgs::msg::Marker sphere;
      sphere.header = header;
      sphere.ns = "supervoxel_markers_ns";
      sphere.id = index + 10000;
      sphere.type = visualization_msgs::msg::Marker::SPHERE;
      sphere.action = visualization_msgs::msg::Marker::ADD;
      sphere.pose.position = point;
      sphere.scale.x = 0.3;
      sphere.scale.y = 0.3;
      sphere.scale.z = 0.3;
      sphere.color.a = 1.0;
      sphere.color.g = 1.0;
      sphere.color.b = 1.0;

      for (auto adjacent_itr = supervoxel_adjacency.equal_range(supervoxel_label).first;
        adjacent_itr != supervoxel_adjacency.equal_range(supervoxel_label).second; ++adjacent_itr)
      {
        auto neighbor_supervoxel =
          supervoxel_clusters.at(adjacent_itr->second);

        geometry_msgs::msg::Point n_point;
        n_point.x = neighbor_supervoxel->centroid_.x;
        n_point.y = neighbor_supervoxel->centroid_.y;
        n_point.z = neighbor_supervoxel->centroid_.z;
        line_strip.points.push_back(n_point);
        line_strip.colors.push_back(yellow_color);
      }
      // Move iterator forward to next label
      label_itr = supervoxel_adjacency.upper_bound(supervoxel_label);
      index++;

      marker_array.markers.push_back(sphere);
      marker_array.markers.push_back(line_strip);
    }
  }

  geometry_msgs::msg::PoseStamped getLinearInterpolatedPose(
    const geometry_msgs::msg::PoseStamped a,
    const geometry_msgs::msg::PoseStamped b
  )
  {
    geometry_msgs::msg::PoseStamped linear_interpolated_pose;
    linear_interpolated_pose.pose.position.x = (a.pose.position.x + b.pose.position.x) / 2.0;
    linear_interpolated_pose.pose.position.y = (a.pose.position.y + b.pose.position.y) / 2.0;
    linear_interpolated_pose.pose.position.z = (a.pose.position.z + b.pose.position.z) / 2.0;
    return linear_interpolated_pose;
  }
}  // namespace vox_nav_utilities
