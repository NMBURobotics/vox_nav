// Copyright (c) 2021 Norwegian University of Life Sciences, Fetullah Atas
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

#ifndef VOX_NAV_CONTROL__CONTROLLER_SERVER_HPP_
#define VOX_NAV_CONTROL__CONTROLLER_SERVER_HPP_

#include <chrono>
#include <string>
#include <memory>
#include <vector>
#include <unordered_map>

#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "vox_nav_msgs/action/follow_path.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "vox_nav_control/controller_core.hpp"
#include "vox_nav_control/common.hpp"
#include "vox_nav_utilities/tf_helpers.hpp"

#include <mosquittopp.h>

namespace vox_nav_control
{
  class ControllerServer : public rclcpp::Node
  {
  public:
    using FollowPath = vox_nav_msgs::action::FollowPath;
    using GoalHandleFollowPath = rclcpp_action::ServerGoalHandle<FollowPath>;

    /**
     * @brief Construct a new Controller Server object
     *
     */
    ControllerServer();

    /**
     * @brief Destroy the Controller Server object
     *
     */
    ~ControllerServer();

    /**
     * @brief
     *
     * @param uuid
     * @param goal
     * @return rclcpp_action::GoalResponse
     */
    rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const FollowPath::Goal> goal);

    /**
     * @brief
     *
     * @param goal_handle
     * @return rclcpp_action::CancelResponse
     */
    rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleFollowPath> goal_handle);

    /**
     * @brief
     *
     * @param goal_handle
     */
    void handle_accepted(const std::shared_ptr<GoalHandleFollowPath> goal_handle);

    /**
     * @brief The action server callback which computes control effort
     */
    void followPath(const std::shared_ptr<GoalHandleFollowPath> goal_handle);

  protected:
    // FollowPath action server
    rclcpp_action::Server<FollowPath>::SharedPtr action_server_;

    // Controller, pluginized server
    vox_nav_control::ControllerCore::Ptr controller_;
    pluginlib::ClassLoader<vox_nav_control::ControllerCore> pc_loader_;
    std::string controller_id_;
    std::string controller_type_;
    double controller_frequency_;
    double controller_duration_;
    double goal_tolerance_distance_;
    double goal_tolerance_orientation_;
    double transform_timeout_;

    // Clock
    rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

    // tf buffer to get access to transfroms
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // ROS Publisher to publish velocity commands
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  };

}  // namespace vox_nav_control

#endif  // VOX_NAV_CONTROL__CONTROLLER_SERVER_HPP_
