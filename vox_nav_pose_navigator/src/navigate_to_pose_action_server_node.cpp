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


#include <memory>
#include <string>
#include <set>

#include "vox_nav_pose_navigator/navigate_to_pose_action_server_node.hpp"


namespace vox_nav_pose_navigator
{

// The Behavior Tree to execute
const char NavigateToPoseActionServer::bt_xml_[] =
  R"(
  <root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
      <Sequence name="NavigateWithReplanning">
        <ComputePathToPose pose="{pose}" path="{path}" planner_id="GridBased"/>
        <FollowPath path="{path}"  controller_id="FollowPath"/>
      </Sequence>
    </BehaviorTree>
  </root>
)";

NavigateToPoseActionServer::NavigateToPoseActionServer()
: Node("navigate_to_pose_server_node")
{
  RCLCPP_INFO(get_logger(), "Creating");
  // Create an action server that we implement with our print_message method
  action_server_ = rclcpp_action::create_server<ActionServer>(
    get_node_base_interface(),
    get_node_clock_interface(),
    get_node_logging_interface(),
    get_node_waitables_interface(),
    "navigate_to_pose",
    std::bind(
      &NavigateToPoseActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&NavigateToPoseActionServer::handle_cancel, this, std::placeholders::_1),
    std::bind(&NavigateToPoseActionServer::handle_accepted, this, std::placeholders::_1)
  );
}

NavigateToPoseActionServer::~NavigateToPoseActionServer()
{
  RCLCPP_INFO(get_logger(), "Destroying");
}

rclcpp_action::GoalResponse
NavigateToPoseActionServer::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const ActionServer::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
NavigateToPoseActionServer::handle_cancel(const std::shared_ptr<GoalHandle> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void
NavigateToPoseActionServer::handle_accepted(
  const std::shared_ptr<GoalHandle> goal_handle)
{
  std::thread{
    std::bind(&NavigateToPoseActionServer::navigate_to_pose, this, std::placeholders::_1),
    goal_handle
  }.detach();
}

void
NavigateToPoseActionServer::navigate_to_pose(const std::shared_ptr<GoalHandle> goal_handle)
{
  BehaviorTree bt(bt_xml_);
  auto result = std::make_shared<ActionServer::Result>();

  // Get the incoming goal from the goal handle
  auto goal = goal_handle->get_goal();
  auto blackboard = bt.blackboard();

  blackboard->set<std::chrono::seconds>("server_timeout", std::chrono::seconds(1));     // NOLINT
  blackboard->set<int>("number_recoveries", 0);    // NOLINT
  blackboard->set<geometry_msgs::msg::PoseStamped>("pose", goal->pose);

  auto should_cancel = [goal_handle]() {return goal_handle->is_canceling();};

  switch (bt.execute(should_cancel)) {
    case vox_nav_pose_navigator::BtStatus::SUCCEEDED:
      RCLCPP_INFO(get_logger(), "Behavior Tree execution succeeded");
      goal_handle->succeed(result);
      break;
    case vox_nav_pose_navigator::BtStatus::FAILED:
      RCLCPP_ERROR(get_logger(), "Behavior Tree execution failed!");
      goal_handle->abort(result);
      break;
    case vox_nav_pose_navigator::BtStatus::HALTED:
      RCLCPP_INFO(get_logger(), "Behavior Tree halted");
      goal_handle->canceled(result);
      break;
    default:
      throw std::logic_error("Invalid status return from BT");
  }

}
}  // namespace vox_nav_pose_navigator

/**
 * @brief
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vox_nav_pose_navigator::NavigateToPoseActionServer>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
