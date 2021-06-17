// Copyright (c) 2018 Intel Corporation
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

#ifndef VOX_NAV_POSE_NAVIGATOR__PLUGINS__ACTIONS__NAVIGATE_TO_POSE_HPP_
#define VOX_NAV_POSE_NAVIGATOR__PLUGINS__ACTIONS__NAVIGATE_TO_POSE_HPP_

#include <string>

#include "vox_nav_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/path.h"
#include "vox_nav_pose_navigator/plugins/actions/base_action_client_node.hpp"

namespace vox_nav_pose_navigator
{
  using NavigateToPose = vox_nav_msgs::action::NavigateToPose;
  class NavigateToPoseNode : public BaseActionClientNode<NavigateToPose>
  {
  public:
    NavigateToPoseNode(
      const std::string & xml_tag_name,
      const std::string & action_name,
      const BT::NodeConfiguration & conf)
    : BaseActionClientNode<NavigateToPose>(xml_tag_name, action_name, conf)
    {
    }

    static BT::PortsList providedPorts()
    {
      return providedBasicPorts(
      {
        BT::InputPort<geometry_msgs::msg::PoseStamped>("pose", "pose"),
      });
    }

    void on_tick()
    {
      // Use the position and orientation fields from the XML attributes to initialize the goal
      geometry_msgs::msg::PoseStamped pose;
      if (!getInput("pose", pose)) {
        RCLCPP_ERROR(
          node_->get_logger(),
          "NavigateToPoseNode: pose not provided");
        return;
      }
      goal_.pose = pose;
    }
  };

}  // namespace vox_nav_pose_navigator

#endif  // VOX_NAV_POSE_NAVIGATOR__PLUGINS__ACTIONS__NAVIGATE_TO_POSE_HPP_
