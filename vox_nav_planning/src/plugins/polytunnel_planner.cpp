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

    polytunnel_cloud_pub_ = parent->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/detected_tunnel_rows",
      rclcpp::SystemDefaultsQoS());

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(parent->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    parent->declare_parameter(plugin_name + ".ref_traj_se2_space", "DUBINS");
    parent->declare_parameter(plugin_name + ".rho", 1.0);
    parent->declare_parameter(plugin_name + ".transform_timeout", 0.1);
    parent->declare_parameter(plugin_name + ".resolution", 0.01);
    parent->declare_parameter(plugin_name + ".row_cloud_downsample_size", 0.4);
    parent->declare_parameter(plugin_name + ".extra_interpolation", 60);
    parent->declare_parameter(plugin_name + ".y_offset", -0.2);
    parent->declare_parameter(plugin_name + ".row_extension_dist", 2.4);

    parent->get_parameter(plugin_name + ".ref_traj_se2_space", selected_se2_space_name_);
    parent->get_parameter(plugin_name + ".rho", rho_);
    parent->get_parameter(plugin_name + ".transform_timeout", transform_timeout_);
    parent->get_parameter(plugin_name + ".resolution", resolution_);
    parent->get_parameter(plugin_name + ".row_cloud_downsample_size", row_cloud_downsample_size_);
    parent->get_parameter(plugin_name + ".extra_interpolation", extra_interpolation_);
    parent->get_parameter(plugin_name + ".y_offset", y_offset_);
    parent->get_parameter(plugin_name + ".row_extension_dist", row_extension_dist_);


    // This is used to interpolate local refernce states
    std::shared_ptr<ompl::base::RealVectorBounds> state_space_bounds =
      std::make_shared<ompl::base::RealVectorBounds>(2);
    if (selected_se2_space_name_ == "SE2") {
      state_space_ = std::make_shared<ompl::base::SE2StateSpace>();
      state_space_->as<ompl::base::SE2StateSpace>()->setBounds(*state_space_bounds);
    } else if (selected_se2_space_name_ == "DUBINS") {
      state_space_ = std::make_shared<ompl::base::DubinsStateSpace>(rho_, false);
      state_space_->as<ompl::base::DubinsStateSpace>()->setBounds(*state_space_bounds);
    } else {
      state_space_ = std::make_shared<ompl::base::ReedsSheppStateSpace>(rho_);
      state_space_->as<ompl::base::ReedsSheppStateSpace>()->setBounds(*state_space_bounds);
    }
    state_space_information_ = std::make_shared<ompl::base::SpaceInformation>(state_space_);

    RCLCPP_INFO(logger_, "Creating...");

  }

  void PolyTunnelPlanner::polytunnelCloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    polytunnel_cloud_ = *msg;
    RCLCPP_INFO(logger_, "Selected points received.");
  }


  std::vector<geometry_msgs::msg::PoseStamped> PolyTunnelPlanner::createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal)
  {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(polytunnel_cloud_, *cloud);

    if (cloud->points.size() == 0) {
      RCLCPP_ERROR(logger_, "No points selected from RVIZ");
      return std::vector<geometry_msgs::msg::PoseStamped>();
    }
    RCLCPP_INFO(logger_, "Creating a plan with %d points", cloud->points.size());

    //Get the side view image of the point cloud
    cv::Mat image = getCloudSideViewImage(cloud, resolution_);

    //Select the rows with bounding boxes
    const cv::String windowName = "Select";
    std::vector<cv::Rect> boundingBoxes;
    bool showCrosshair = true;
    bool fromCenter = false;
    cv::selectROIs(windowName, image, boundingBoxes, showCrosshair, fromCenter);
    cv::destroyWindow("Select");

    pcl::PointXYZRGB average_point;

    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters = getClustersWithinBBXs(
      cloud,
      boundingBoxes,
      resolution_,
      row_cloud_downsample_size_,
      average_point);

    geometry_msgs::msg::PoseStamped curr_robot_pose;
    vox_nav_utilities::getCurrentPose(
      curr_robot_pose, *tf_buffer_, "map", "base_link", transform_timeout_);
    pcl::PointXYZ robot_position(
      curr_robot_pose.pose.position.x,
      curr_robot_pose.pose.position.y,
      curr_robot_pose.pose.position.z);

    std::map<int, double> cluster_distance_map =
      getClusterDistancetoRobot(clusters, robot_position);

    // sort the  map by value,
    std::set<std::pair<int, double>, Comparator> set(
      cluster_distance_map.begin(), cluster_distance_map.end(), compFunctor);

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters_organized =
      orginizeAndSortClusterPoints(clusters, set, robot_position);

    std::vector<geometry_msgs::msg::PoseStamped> plan_poses = rowClusters2InterpolatedPath(
      clusters_organized, extra_interpolation_, average_point, curr_robot_pose);

    vox_nav_utilities::publishClustersCloud<pcl::PointCloud<pcl::PointXYZ>::Ptr>(
      polytunnel_cloud_pub_, polytunnel_cloud_.header,
      clusters_organized);

    RCLCPP_INFO(logger_, "Created a plan with %d pose", plan_poses.size());

    return plan_poses;
  }

  std::vector<geometry_msgs::msg::PoseStamped> PolyTunnelPlanner::getOverlayedStartandGoal()
  {
    return std::vector<geometry_msgs::msg::PoseStamped>();
  }

}   // namespace vox_nav_planning

PLUGINLIB_EXPORT_CLASS(vox_nav_planning::PolyTunnelPlanner, vox_nav_planning::PlannerCore)
