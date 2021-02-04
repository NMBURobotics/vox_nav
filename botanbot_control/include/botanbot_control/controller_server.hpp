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

#ifndef BOTANBOT_CONTROL__CONTROLLER_SERVER_HPP_
#define BOTANBOT_CONTROL__CONTROLLER_SERVER_HPP_

#include <chrono>
#include <string>
#include <memory>
#include <vector>
#include <unordered_map>

#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "botanbot_msgs/action/follow_path.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <botanbot_control/controller_core.hpp>
#include <botanbot_utilities/tf_helpers.hpp>

namespace botanbot_control
{

class ControllerServer : public rclcpp::Node
{
public:
  using FollowPath = botanbot_msgs::action::FollowPath;
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

  using ControllerMap = std::unordered_map<std::string, botanbot_control::ControllerCore::Ptr>;

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

protected:
  // Our action server implements the FollowPath action
  rclcpp_action::Server<FollowPath>::SharedPtr action_server_;

  /**
   * @brief The action server callback which computes control effort
   */
  void followPath(const std::shared_ptr<GoalHandleFollowPath> goal_handle);


  void publishControl(const std::vector<geometry_msgs::msg::PoseStamped> & path);

  // Planner
  ControllerMap controllers_;
  pluginlib::ClassLoader<botanbot_control::ControllerCore> pc_loader_;
  std::string controller_id_;
  std::string controller_type_;
  std::string controller_ids_concat_;
  double controller_frequency_;
  double controller_duration_;

  // Clock
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  // TF buffer
  std::shared_ptr<tf2_ros::Buffer> tf_;

  // Publishers for the controller
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    actual_control_states_publisher_;

  // tf buffer to get access to transfroms
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace botanbot_control

#endif  // BOTANBOT_CONTROL__CONTROLLER_SERVER_HPP_
