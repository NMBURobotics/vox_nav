// Copyright (c) 2019 Intel Corporation
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

#ifndef VOX_NAV_POSE_NAVIGATOR__BEHAVIOR_TREE_HPP_
#define VOX_NAV_POSE_NAVIGATOR__BEHAVIOR_TREE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/xml_parsing.h"

namespace vox_nav_pose_navigator
{

// The possible return values from the execution of a Behavior Tree
  enum class BtStatus { SUCCEEDED, FAILED, HALTED };

  class BehaviorTree
  {
  public:
    explicit BehaviorTree(
      const std::string & bt_xml,
      const std::vector<std::string> & plugin_library_names = {
      "compute_path_to_pose_action_bt_node",
      "follow_path_action_bt_node"}
    );
    BehaviorTree() = delete;
    virtual ~BehaviorTree() {}

    BtStatus execute(
      std::function<bool()> should_halt = []() {return false;},
      std::function<void()> on_loop_iteration = []() {},
      std::chrono::milliseconds tick_period = std::chrono::milliseconds(10));

    BT::Blackboard::Ptr blackboard() {return blackboard_;}
    BT::BehaviorTreeFactory & factory() {return factory_;}

  protected:
    // The factory to use when dynamically constructing the Behavior Tree
    BT::BehaviorTreeFactory factory_;

    // XML parser to parse the supplied BT XML input
    BT::XMLParser xml_parser_;

    // The blackboard to be shared by all of the Behavior Tree's nodes
    BT::Blackboard::Ptr blackboard_;
  };

}  // namespace vox_nav_pose_navigator

#endif  // VOX_NAV_POSE_NAVIGATOR__BEHAVIOR_TREE_HPP_
