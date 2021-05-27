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

#ifndef vox_nav_POSE_NAVIGATOR__PLUGINS__ACTIONS__COMPUTE_PATH_TO_POSE_HPP_
#define vox_nav_POSE_NAVIGATOR__PLUGINS__ACTIONS__COMPUTE_PATH_TO_POSE_HPP_

#include <string>

#include "nav_msgs/msg/path.h"
#include "vox_nav_msgs/action/compute_path_to_pose.hpp"
#include "vox_nav_pose_navigator/plugins/actions/base_action_client_node.hpp"


namespace vox_nav_pose_navigator
{

using ComputePathToPose = vox_nav_msgs::action::ComputePathToPose;

class ComputePathToPoseNode : public BaseActionClientNode<ComputePathToPose>
{
public:
  explicit ComputePathToPoseNode(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : BaseActionClientNode<ComputePathToPose>(xml_tag_name, action_name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::OutputPort<nav_msgs::msg::Path>("path", "Path created by ComputePathToPose node"),
        BT::InputPort<geometry_msgs::msg::PoseStamped>("pose", "Destination to plan to"),
        BT::InputPort<std::string>("planner_id", ""),
      });
  }

  void  on_tick()
  {
    getInput("pose", goal_.pose);
    getInput("planner_id", goal_.planner_id);
  }

  BT::NodeStatus on_success()
  {
    setOutput("path", result_.result->path);

    if (first_time_) {
      first_time_ = false;
    } else {
      config().blackboard->set("path_updated", true);
    }
    return BT::NodeStatus::SUCCESS;
  }

private:
  bool first_time_{true};
};
}  // namespace vox_nav_pose_navigator


#endif  // vox_nav_POSE_NAVIGATOR__PLUGINS__ACTIONS__COMPUTE_PATH_TO_POSE_HPP_
