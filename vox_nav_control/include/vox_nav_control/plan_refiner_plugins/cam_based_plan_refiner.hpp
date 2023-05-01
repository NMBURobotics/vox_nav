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


#ifndef VOX_NAV_CONTROL__PLAN_REFINER_PLUGINS__CAM_BASED_PLAN_REFINER_HPP_
#define VOX_NAV_CONTROL__PLAN_REFINER_PLUGINS__CAM_BASED_PLAN_REFINER_HPP_
#pragma once

#include <string>
#include <memory>
#include <vector>

#include "vox_nav_control/plan_refiner_core.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"

namespace vox_nav_control
{

  class CamBasedPlanRefiner : public vox_nav_control::PlanRefinerCore
  {
  public:
    /**
     * @brief Construct a new Cam Based Plan Refiner object
     *
     */
    CamBasedPlanRefiner();

    /**
     * @brief Destroy the Cam Based Plan Refiner object
     *
     */
    ~CamBasedPlanRefiner();

    /**
     * @brief
     *
     * @param parent rclcpp parent node
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

  private:
    // Pointer to the node that owns this plugin
    rclcpp::Node * node_;
    // Name for this path refiner plugin
    std::string plugin_name_;

    // Subscribe to image topic
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    // Subscribe to camera info topic
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;

    // Just in case we need to transform the image to the map frame
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
  };

}      // namespace vox_nav_control


#endif  // VOX_NAV_CONTROL__PLAN_REFINER_PLUGINS__CAM_BASED_PLAN_REFINER_HPP_
