// Copyright (c) 2019 Samsung Research America
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

#ifndef VOX_NAV_PLANNING__PLANNER_SERVER_HPP_
#define VOX_NAV_PLANNING__PLANNER_SERVER_HPP_
#pragma once

#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pluginlib/class_loader.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/create_timer_interface.h>
#include <builtin_interfaces/msg/duration.hpp>

#include "vox_nav_planning/planner_core.hpp"
#include "vox_nav_utilities/tf_helpers.hpp"
#include "vox_nav_msgs/action/compute_path_to_pose.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <utility>

namespace vox_nav_planning
{
  class PlannerServer : public rclcpp::Node
  {
  public:
    using ComputePathToPose = vox_nav_msgs::action::ComputePathToPose;
    using GoalHandleComputePathToPose = rclcpp_action::ServerGoalHandle<ComputePathToPose>;
    /**
     * @brief Construct a new Planner Server object
     *
     */
    PlannerServer();

    /**
     * @brief Destroy the Planner Server object
     *
     */
    ~PlannerServer();

    using PlannerMap = std::unordered_map<std::string, vox_nav_planning::PlannerCore::Ptr>;

    /**
     * @brief Method to get plan from the desired plugin
     * @param start starting pose
     * @param goal goal request
     * @return Path
     */
    std::vector<geometry_msgs::msg::PoseStamped> getPlan(
      const geometry_msgs::msg::PoseStamped & start,
      const geometry_msgs::msg::PoseStamped & goal,
      const std::string & planner_id);

    /**
     * @brief
     *
     * @param uuid
     * @param goal
     * @return rclcpp_action::GoalResponse
     */
    rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const ComputePathToPose::Goal> goal);

    /**
     * @brief
     *
     * @param goal_handle
     * @return rclcpp_action::CancelResponse
     */
    rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleComputePathToPose> goal_handle);

    /**
     * @brief
     *
     * @param goal_handle
     */
    void handle_accepted(const std::shared_ptr<GoalHandleComputePathToPose> goal_handle);

  protected:
    // Our action server implements the ComputePathToPose action
    rclcpp_action::Server<ComputePathToPose>::SharedPtr action_server_;

    /**
     * @brief The action server callback which calls planner to get the path
     */
    void computePlan(const std::shared_ptr<GoalHandleComputePathToPose> goal_handle);

    // Planner
    PlannerMap planners_;
    pluginlib::ClassLoader<vox_nav_planning::PlannerCore> pc_loader_;
    std::string planner_id_;
    std::string planner_type_;
    double max_planner_duration_;
    std::string planner_ids_concat_;
    double expected_planner_frequency_;
    // Clock
    rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
    // tf buffer to get transfroms
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    // tf listner for tf transforms
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    // Publishers for the path
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr plan_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr nav_msgs_path_pub_;
    // obot mesh path, if there is one
    std::string robot_mesh_path_;
    bool publish_segment_ids_;
  };

}  // namespace vox_nav_planning

#endif  // VOX_NAV_PLANNING__PLANNER_SERVER_HPP_
