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

#ifndef BOTANBOT_CONTROL__CONTROLLER_CORE_HPP_
#define BOTANBOT_CONTROL__CONTROLLER_CORE_HPP_

#include <memory>
#include <string>

#include "tf2_ros/transform_listener.h"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace botanbot_control
{
/**
 * @class ControllerCore
 * @brief controllerCore interface that acts as a virtual base class for all controllerCore plugins
 */
class ControllerCore
{
public:
  using Ptr = std::shared_ptr<botanbot_control::ControllerCore>;

  /**
   * @brief Virtual destructor
   */
  virtual ~ControllerCore() {}

  /**
   * @brief
   *
   * @param parent
   * @param plugin_name
   */
  virtual void initialize(
    rclcpp::Node * parent,
    const std::string & plugin_name) = 0;

  /**
   * @brief local setPlan - Sets the global plan
   * @param path The global plan
   */
  virtual void setPlan(const nav_msgs::msg::Path & path) = 0;

  /**
   * @brief
   *
   * @return geometry_msgs::msg::Twist
   */
  virtual geometry_msgs::msg::Twist computeVelocityCommands() = 0;
};

}  // namespace botanbot_control

#endif  // BOTANBOT_CONTROL__CONTROLLER_CORE_HPP_
