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

#ifndef VOX_NAV_PLANNING__PLUGINS__RRT_UTILS_HPP_
#define VOX_NAV_PLANNING__PLUGINS__RRT_UTILS_HPP_

#include <vector>
#include <string>
#include <memory>

#define PCL_NO_PRECOMPILE
// PCL
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/octree/octree_search.h>
#include <pcl/filters/random_sample.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
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


struct Node
{
  PCL_ADD_POINT4D;                       // preferred way of adding a XYZ+padding
  double psi;                            // heading angle
  double v;
  PCL_MAKE_ALIGNED_OPERATOR_NEW          // make sure our new allocators are aligned
} EIGEN_ALIGN16;                         // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT(
  Node,                                            // here we assume a XYZ + "test" (as fields)
  (float, x, x)(float, y, y)(float, z, z)(float, psi, psi)(float, v, v)
)

class OctoCellValidStateSampler
{
public:
  OctoCellValidStateSampler(
    const geometry_msgs::msg::PoseStamped start,
    const geometry_msgs::msg::PoseStamped goal,
    const geometry_msgs::msg::PoseArray::SharedPtr & elevated_surfels_poses)
  : elevated_surfels_poses_(*elevated_surfels_poses)
  {
    workspace_surfels_ = pcl::PointCloud<pcl::PointSurfel>::Ptr(
      new pcl::PointCloud<pcl::PointSurfel>);
    vox_nav_utilities::fillSurfelsfromMsgPoses(elevated_surfels_poses_, workspace_surfels_);
    RCLCPP_INFO(
      logger_, "OctoCellValidStateSampler bases on an Octomap with %d surfels",
      workspace_surfels_->points.size());
    updateSearchArea(start, goal);
  }

  bool sample(Node * state)
  {
    std::random_device rd;
    std::mt19937 rng(rd());
    int val = distrubutions_(rng);
    auto out_sample = search_area_surfels_->points.at(val);
    state->x = out_sample.x;
    state->y = out_sample.y;
    state->z = out_sample.z;
    return true;
  }

  void updateSearchArea(
    const geometry_msgs::msg::PoseStamped start,
    const geometry_msgs::msg::PoseStamped goal)
  {
    RCLCPP_INFO(logger_, "Updating search area");

    double radius = vox_nav_utilities::getEuclidianDistBetweenPoses(goal, start) / 2.0;
    auto search_point_pose = vox_nav_utilities::getLinearInterpolatedPose(goal, start);
    auto search_point_surfel = vox_nav_utilities::poseMsg2PCLSurfel(search_point_pose);

    search_area_surfels_ =
      vox_nav_utilities::getSubCloudWithinRadius<pcl::PointSurfel>(
      workspace_surfels_, search_point_surfel,
      radius);

    RCLCPP_INFO(logger_, "Updated search area surfels, %d", search_area_surfels_->points.size());

    search_area_surfels_ = vox_nav_utilities::uniformlySampleCloud<pcl::PointSurfel>(
      search_area_surfels_, 1.2);

    RCLCPP_INFO(
      logger_, "Uniformly sampled %d search area surfels,", search_area_surfels_->points.size());

    std::vector<int> weights;
    for (auto && i : search_area_surfels_->points) {
      //auto tilt_angle = std::max(std::abs(i.normal_x), std::abs(i.normal_y)) * 180.0 / M_PI;
      //weights.push_back(200 / tilt_angle);
      weights.push_back(1);
    }
    std::discrete_distribution<> distrubutions(weights.begin(), weights.end());
    distrubutions_ = distrubutions;
  }

protected:
  rclcpp::Logger logger_{rclcpp::get_logger("octo_cell_valid_state_sampler")};
  geometry_msgs::msg::PoseArray elevated_surfels_poses_;
  pcl::PointCloud<pcl::PointSurfel>::Ptr workspace_surfels_;
  pcl::PointCloud<pcl::PointSurfel>::Ptr search_area_surfels_;
  std::discrete_distribution<> distrubutions_;
};

struct Bounds
{
  double x_;       // X position
  double y_;       // Y position
  double z_;       // Z position
  double psi_;     // heading angle
  double v_;       // linear velocity
  Bounds()
  : x_(0.0),
    y_(0.0),
    z_(0.0),
    psi_(0.0),
    v_(0.0) {}
  Bounds(double x, double y, double z, double psi, double v)
  : x_(x),
    y_(y),
    z_(z),
    psi_(psi),
    v_(v) {}
};


#endif  // VOX_NAV_PLANNING__PLUGINS__RRT_UTILS_HPP_
