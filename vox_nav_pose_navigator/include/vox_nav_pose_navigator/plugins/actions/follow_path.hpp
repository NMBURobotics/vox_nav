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

#ifndef VOX_NAV_POSE_NAVIGATOR__PLUGINS__ACTIONS__FOLLOW_PATH_HPP_
#define VOX_NAV_POSE_NAVIGATOR__PLUGINS__ACTIONS__FOLLOW_PATH_HPP_

#include <string>

#include "vox_nav_msgs/action/follow_path.hpp"
#include "nav_msgs/msg/path.h"
#include "vox_nav_pose_navigator/plugins/actions/base_action_client_node.hpp"

namespace vox_nav_pose_navigator
{
using FollowPath = vox_nav_msgs::action::FollowPath;

class FollowPathNode : public BaseActionClientNode<FollowPath>
{
public:
  FollowPathNode(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : BaseActionClientNode<FollowPath>(xml_tag_name, action_name, conf)
  {
    config().blackboard->set("path_updated", false);
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<nav_msgs::msg::Path>("path", "Path to follow"),
        BT::InputPort<std::string>("controller_id", ""),
      });
  }

  void on_tick()
  {
    getInput("path", goal_.path);
    getInput("controller_id", goal_.controller_id);
  }

  void on_wait_for_result()
  {
    // Check if the goal has been updated
    if (config().blackboard->get<bool>("path_updated")) {
      // Reset the flag in the blackboard
      config().blackboard->set("path_updated", false);

      // Grab the new goal and set the flag so that we send the new goal to
      // the action server on the next loop iteration
      getInput("path", goal_.path);
      goal_updated_ = true;
    }
  }
};

}  // namespace vox_nav_pose_navigator

#endif  // VOX_NAV_POSE_NAVIGATOR__PLUGINS__ACTIONS__FOLLOW_PATH_HPP_
