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

#ifndef botanbot_pose_navigator__PLUGINS__ACTION__NAVIGATE_TO_POSE_ACTION_HPP_
#define botanbot_pose_navigator__PLUGINS__ACTION__NAVIGATE_TO_POSE_ACTION_HPP_

#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "botanbot_msgs/action/navigate_to_pose.hpp"
#include "botanbot_pose_navigator/bt_action_node.hpp"

namespace botanbot_pose_navigator
{

class NavigateToPoseAction : public BtActionNode<botanbot_msgs::action::NavigateToPose>
{
public:
  NavigateToPoseAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;

  // Any BT node that accepts parameters must provide a requiredNodeParameters method
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<geometry_msgs::msg::Point>("position", "Position"),
        BT::InputPort<geometry_msgs::msg::Quaternion>("orientation", "Orientation")
      });
  }
};

}  // namespace botanbot_pose_navigator

#endif  // botanbot_pose_navigator__PLUGINS__ACTION__NAVIGATE_TO_POSE_ACTION_HPP_
