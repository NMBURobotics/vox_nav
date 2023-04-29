// Copyright (c) 2023 Norwegian University of Life Sciences, Fetullah Atas
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

#include "vox_nav_control/plan_refiner_plugins/traversability_based_plan_refiner.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace vox_nav_control
{

  TraversabilityBasedPlanRefiner::TraversabilityBasedPlanRefiner()
  {


  }

  TraversabilityBasedPlanRefiner::~TraversabilityBasedPlanRefiner()
  {

  }

  void TraversabilityBasedPlanRefiner::initialize(
    rclcpp::Node * parent,
    const std::string & plugin_name)
  {
    node_ = parent->shared_from_this();

    traversability_map_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
    traversability_marker_ = std::make_shared<visualization_msgs::msg::MarkerArray>();

    // init tf2 buffer and listener
    tf_buffer_ptr_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ptr_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_ptr_);

    // Bind to traversability map and marker topics
    traversability_map_subscriber_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      "traversability_map", rclcpp::SystemDefaultsQoS(),
      std::bind(
        &TraversabilityBasedPlanRefiner::traversabilityMapCallback, this,
        std::placeholders::_1));

    traversability_marker_subscriber_ =
      node_->create_subscription<visualization_msgs::msg::MarkerArray>(
      "traversability_marker", rclcpp::SystemDefaultsQoS(),
      std::bind(
        &TraversabilityBasedPlanRefiner::traversabilityMarkerCallback, this,
        std::placeholders::_1));

    node_->declare_parameter(plugin_name + ".traversability_layer_name", "traversability");
    node_->declare_parameter(plugin_name + ".traversability_threshold", 0.5);
    node_->get_parameter(plugin_name + ".traversability_layer_name", traversability_layer_name_);
    node_->get_parameter(plugin_name + ".traversability_threshold", traversability_threshold_);

  }

  bool TraversabilityBasedPlanRefiner::refinePlan(
    const geometry_msgs::msg::PoseStamped & curr_pose,
    nav_msgs::msg::Path & plan_to_refine)
  {

    // Get the closest point on the path to edge of the traversability map
    // and check if it is traversable

    // Convert most recent traversability map to pcl pointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr traversability_map_pcl(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*traversability_map_, *traversability_map_pcl);

    // Fit a box around the traversability map
    vox_nav_msgs::msg::Object traversability_map_box;
    vox_nav_utilities::fitBoxtoPointCloud(traversability_map_pcl, traversability_map_box);

    return true;

  }

  void TraversabilityBasedPlanRefiner::traversabilityMapCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    traversability_map_ = msg;
  }

  void TraversabilityBasedPlanRefiner::traversabilityMarkerCallback(
    const visualization_msgs::msg::MarkerArray::SharedPtr msg)
  {
    traversability_marker_ = msg;
  }

}      // namespace vox_nav_control

PLUGINLIB_EXPORT_CLASS(
  vox_nav_control::TraversabilityBasedPlanRefiner,
  vox_nav_control::PlanRefinerCore)
