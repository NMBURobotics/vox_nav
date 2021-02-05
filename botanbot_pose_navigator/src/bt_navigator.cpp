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

#include <fstream>
#include <memory>
#include <streambuf>
#include <string>
#include <utility>
#include <vector>
#include <set>

#include "botanbot_utilities/tf_helpers.hpp"
#include "botanbot_pose_navigator/bt_conversions.hpp"
#include "botanbot_pose_navigator/bt_navigator.hpp"

namespace botanbot_pose_navigator
{

const char BtNavigator::bt_xml_[] =
  R"(
  <root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
      <PipelineSequence name="NavigateWithReplanning">
        <RateController hz="1.0">
          <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
        </RateController>
        <FollowPath path="{path}" controller_id="FollowPath"/>
      </PipelineSequence>
    </BehaviorTree>
  </root>
  )";

BtNavigator::BtNavigator()
: rclcpp::Node("bt_navigator"),
  start_time_(0)
{
  RCLCPP_INFO(get_logger(), "Creating");

  const std::vector<std::string> plugin_libs = {
    "nav2_compute_path_to_pose_action_bt_node",
    "nav2_follow_path_action_bt_node"
  };

  // Support for handling the topic-based goal pose from rviz
  client_node_ = std::make_shared<rclcpp::Node>("_");


  using namespace std::placeholders;

  action_server_ = rclcpp_action::create_server<ActionServer>(
    get_node_base_interface(),
    get_node_clock_interface(),
    get_node_logging_interface(),
    get_node_waitables_interface(),
    "navigate_to_pose",
    std::bind(&BtNavigator::handle_goal, this, _1, _2),
    std::bind(&BtNavigator::handle_cancel, this, _1),
    std::bind(&BtNavigator::handle_accepted, this, _1)
  );
}

BtNavigator::~BtNavigator()
{
  RCLCPP_INFO(get_logger(), "Destroying");
}

rclcpp_action::GoalResponse
BtNavigator::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const ActionServer::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
BtNavigator::handle_cancel(const std::shared_ptr<GoalHandle> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void
BtNavigator::handle_accepted(
  const std::shared_ptr<GoalHandle> goal_handle)
{
  using namespace std::placeholders;
  std::thread{
    std::bind(&BtNavigator::navigateToPose, this, _1), goal_handle
  }.detach();
}

void
BtNavigator::navigateToPose(const std::shared_ptr<GoalHandle> goal_handle)
{
  botanbot_pose_navigator::BehaviorTree bt(bt_xml_);
  auto result = std::make_shared<ActionServer::Result>();

  // Get the incoming goal from the goal handle
  auto goal = goal_handle->get_goal();

  // Pass the values from the goal to the Behavior Tree via the blackboard
  bt.blackboard()->set<geometry_msgs::msg::PoseStamped>("pose", goal->pose);  // NOLINT

  auto should_cancel = [goal_handle]() {return goal_handle->is_canceling();};

  switch (bt.execute(should_cancel)) {
    case botanbot_pose_navigator::BtStatus::SUCCEEDED:
      RCLCPP_INFO(get_logger(), "Behavior Tree execution succeeded");
      goal_handle->succeed(result);
      break;

    case botanbot_pose_navigator::BtStatus::FAILED:
      RCLCPP_ERROR(get_logger(), "Behavior Tree execution failed!");
      goal_handle->abort(result);
      break;

    case botanbot_pose_navigator::BtStatus::HALTED:
      RCLCPP_INFO(get_logger(), "Behavior Tree halted");
      goal_handle->canceled(result);
      break;

    default:
      throw std::logic_error("Invalid status return from BT");
  }
}
}  // namespace botanbot_pose_navigator
