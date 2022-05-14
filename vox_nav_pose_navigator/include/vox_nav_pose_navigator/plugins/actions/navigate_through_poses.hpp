// Copyright (c) 2021 Fetullah Atas, Norwegian University of Life Sciences
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

#ifndef VOX_NAV_POSE_NAVIGATOR__PLUGINS__ACTIONS__NAVIGATE_THROUGH_POSES_HPP_
#define VOX_NAV_POSE_NAVIGATOR__PLUGINS__ACTIONS__NAVIGATE_THROUGH_POSES_HPP_

#include <string>

#include "vox_nav_msgs/action/navigate_through_poses.hpp"
#include "nav_msgs/msg/path.h"
#include "vox_nav_pose_navigator/plugins/actions/base_action_client_node.hpp"

namespace vox_nav_pose_navigator
{
  using NavigateThroughPoses = vox_nav_msgs::action::NavigateThroughPoses;
  class NavigateThroughPosesNode : public BaseActionClientNode<NavigateThroughPoses>
  {
  public:
    NavigateThroughPosesNode(
      const std::string & xml_tag_name,
      const std::string & action_name,
      const BT::NodeConfiguration & conf)
    : BaseActionClientNode<NavigateThroughPoses>(xml_tag_name, action_name, conf)
    {
    }

    static BT::PortsList providedPorts()
    {
      return providedBasicPorts(
      {
        BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("poses", "poses"),
      });
    }

    void on_tick()
    {
      // Use the position and orientation fields from the XML attributes to initialize the goal
      std::vector<geometry_msgs::msg::PoseStamped> poses;
      if (!getInput("poses", poses)) {
        RCLCPP_ERROR(
          node_->get_logger(),
          "NavigateThroughPosesNode: pose not provided");
        return;
      }
      goal_.poses = poses;
    }
  };

}  // namespace vox_nav_pose_navigator

#endif  // VOX_NAV_POSE_NAVIGATOR__PLUGINS__ACTIONS__NAVIGATE_THROUGH_POSES_HPP_
