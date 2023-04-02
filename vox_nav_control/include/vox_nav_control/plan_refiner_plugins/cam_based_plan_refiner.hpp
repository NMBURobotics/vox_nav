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

#include "vox_nav_control/plan_refiner.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"

namespace vox_nav_control
{

  class CamBasedPlanRefiner : public vox_nav_control::PlanRefiner
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
     * @param parent rclcpp node
     * @param plugin_name refiner plugin name
     */
    void initialize(
      const rclcpp::Node * parent,
      const std::string & plugin_name) override;

    /**
     * @brief Refine the plan locally, only in the vicinity of the robot
     *
     * @param curr_pose
     * @param plan
     * @return std::vector<geometry_msgs::msg::PoseStamped>
     */
    bool refinePlan(
      const geometry_msgs::msg::PoseStamped & curr_pose) override;

  private:
    std::shared_ptr<rclcpp::Node> node_;
    std::string plugin_name_;

    // Subscribe to image topic
    // Subscribe to camera info topic

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;

    // Project the plan to the image plane with tf and camera info

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
  };

}      // namespace vox_nav_control


#endif  // VOX_NAV_CONTROL__PLAN_REFINER_PLUGINS__CAM_BASED_PLAN_REFINER_HPP_
