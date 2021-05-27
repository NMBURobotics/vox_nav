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

#include "vox_nav_pose_navigator/behavior_tree.hpp"

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/xml_parsing.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "rclcpp/rclcpp.hpp"

namespace vox_nav_pose_navigator
{

BehaviorTree::BehaviorTree(
  const std::string & bt_xml,
  const std::vector<std::string> & plugin_library_names)
: xml_parser_(factory_)
{
  // Load any specified BT plugins
  for (const auto & library_name : plugin_library_names) {
    factory_.registerFromPlugin(std::string{"lib" + library_name + ".so"});
  }

  // Parse the input XML
  xml_parser_.loadFromText(bt_xml);

  // Create a blackboard for this Behavior Tree
  blackboard_ = BT::Blackboard::create();
}

BtStatus
BehaviorTree::execute(
  std::function<bool()> should_halt,
  std::function<void()> on_loop_iteration,
  std::chrono::milliseconds tick_period)
{
  // Create the corresponding Behavior Tree
  BT::Tree tree = xml_parser_.instantiateTree(blackboard_);

  BT::StdCoutLogger logger(tree);

  // Set up a loop rate controller based on the desired tick period
  rclcpp::WallRate loop_rate(tick_period);

  // Loop until something happens with ROS or the node completes
  BT::NodeStatus result = BT::NodeStatus::RUNNING;
  while (rclcpp::ok() && result == BT::NodeStatus::RUNNING) {
    if (should_halt()) {
      tree.rootNode()->halt();
      return BtStatus::HALTED;
    }

    // Execute one tick of the tree
    result = tree.rootNode()->executeTick();

    // Give the caller a chance to do something on each loop iteration
    on_loop_iteration();

    // Throttle the BT loop rate, based on the provided tick period value
    loop_rate.sleep();
  }

  return (result == BT::NodeStatus::SUCCESS) ? BtStatus::SUCCEEDED : BtStatus::FAILED;
}

}  // namespace vox_nav_pose_navigator
