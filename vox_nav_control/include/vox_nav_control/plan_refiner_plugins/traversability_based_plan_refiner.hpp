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


#ifndef VOX_NAV_CONTROL__PLAN_REFINER_PLUGINS__TRAVERSABILITY_BASED_PLAN_REFINER_HPP_
#define VOX_NAV_CONTROL__PLAN_REFINER_PLUGINS__TRAVERSABILITY_BASED_PLAN_REFINER_HPP_
#pragma once

#include <string>
#include <memory>
#include <vector>

#include "vox_nav_control/plan_refiner_core.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/path.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "vox_nav_utilities/pcl_helpers.hpp"

namespace vox_nav_control
{

  class TraversabilityBasedPlanRefiner : public vox_nav_control::PlanRefinerCore
  {
  public:
    /**
     * @brief Construct a new Cam Based Plan Refiner object
     *
     */
    TraversabilityBasedPlanRefiner();

    /**
     * @brief Destroy the Cam Based Plan Refiner object
     *
     */
    ~TraversabilityBasedPlanRefiner();

    /**
     * @brief
     *
     * @param parent rclcpp node
     * @param plugin_name refiner plugin name
     */
    void initialize(
      rclcpp::Node * parent,
      const std::string & plugin_name) override;

    /**
     * @brief Refine the plan locally, only in the vicinity of the robot
     *
     * @param curr_pose
     * @param plan
     * @return std::vector<geometry_msgs::msg::PoseStamped>
     */
    bool refinePlan(
      const geometry_msgs::msg::PoseStamped & curr_pose,
      nav_msgs::msg::Path & plan_to_refine) override;

    void traversabilityMapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    void traversabilityMarkerCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);

  private:
    std::shared_ptr<rclcpp::Node> node_;
    std::string plugin_name_;

    std::string traversability_layer_name_;
    double traversability_threshold_;

    // Subscribe to image topic
    // Subscribe to camera info topic
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr traversability_map_subscriber_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr
      traversability_marker_subscriber_;

    sensor_msgs::msg::PointCloud2::SharedPtr traversability_map_;
    visualization_msgs::msg::MarkerArray::SharedPtr traversability_marker_;

    // Project the plan to the image plane with tf and camera info
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_ptr_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_ptr_;
  };

}      // namespace vox_nav_control


#endif  // VOX_NAV_CONTROL__PLAN_REFINER_PLUGINS__TRAVERSABILITY_BASED_PLAN_REFINER_HPP_
