// Copyright (c) 2022 Fetullah Atas, Norwegian University of Life Sciences
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

#include "vox_nav_planning/plugins/polytunnel_planner.hpp"
#include <pluginlib/class_list_macros.hpp>

#include <string>
#include <memory>
#include <vector>
#include <random>

namespace vox_nav_planning
{

  PolyTunnelPlanner::PolyTunnelPlanner()
  {
  }

  PolyTunnelPlanner::~PolyTunnelPlanner()
  {
  }

  void PolyTunnelPlanner::initialize(
    rclcpp::Node * parent,
    const std::string & plugin_name)
  {
    polytunnel_cloud_sub_ = parent->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/rviz_selected_points",
      rclcpp::SystemDefaultsQoS(),
      std::bind(&PolyTunnelPlanner::polytunnelCloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(logger_, "Selected planner is: %s", planner_name_.c_str());
  }

  void PolyTunnelPlanner::polytunnelCloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    polytunnel_cloud_ = *msg;
    RCLCPP_INFO(logger_, "Selected points received");
  }


  std::vector<geometry_msgs::msg::PoseStamped> PolyTunnelPlanner::createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal)
  {

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_curr(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(polytunnel_cloud_, *pcl_curr);

    if (pcl_curr->points.size() == 0) {
      RCLCPP_ERROR(logger_, "No points selected from RVIZ");
      return std::vector<geometry_msgs::msg::PoseStamped>();
    }
    RCLCPP_INFO(logger_, "Creating a plan with %d points", pcl_curr->points.size());

    std::vector<geometry_msgs::msg::PoseStamped> plan;

    return plan;
  }


}  // namespace vox_nav_planning

PLUGINLIB_EXPORT_CLASS(vox_nav_planning::PolyTunnelPlanner, vox_nav_planning::PlannerCore)
