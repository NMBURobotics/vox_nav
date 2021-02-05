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

#ifndef botanbot_pose_navigator__ROS2_ACTION_CLIENT_NODE_HPP_
#define botanbot_pose_navigator__ROS2_ACTION_CLIENT_NODE_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "botanbot_pose_navigator/bt_conversions.hpp"

namespace botanbot_pose_navigator
{

template<class ActionT>
class ActionClientNode : public BT::CoroActionNode
{
public:
  ActionClientNode(const std::string & name, const BT::NodeConfiguration & config)
  : BT::CoroActionNode(name, config)
  {
    // Initialize the input and output messages
    goal_ = typename ActionT::Goal();
    result_ = typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult();
  }

  ActionClientNode() = delete;

  // Define the ports required by the ROS2ActionClient node
  static BT::PortsList augment_basic_ports(BT::PortsList additional_ports)
  {
    BT::PortsList basic_ports = {
      BT::InputPort<std::string>("action_name", "The name of the action to call"),
      BT::InputPort<std::chrono::milliseconds>(
        "server_timeout",
        "The timeout value, in milliseconds, to use when waiting for the action server responses"),
      BT::InputPort<std::shared_ptr<rclcpp::Node>>(
        "ros2_node",
        "The ROS2 node to use when creating the service")
    };

    basic_ports.insert(additional_ports.begin(), additional_ports.end());
    return basic_ports;
  }

  // Any subclass of ActionClientNode that defines additional ports must then define
  // its own providedPorts method and call augment_basic_ports to add the subclass's ports
  // to the required basic ports
  static BT::PortsList providedPorts()
  {
    return augment_basic_ports({});
  }

  // A derived class the defines input and/or output ports can override these methods
  // to get/set the ports
  virtual void read_input_ports(typename ActionT::Goal & goal) {}
  virtual bool read_new_goal(typename ActionT::Goal & goal) {return false;}
  virtual void write_feedback_ports(const std::shared_ptr<const typename ActionT::Feedback>) {}
  virtual void write_output_ports(
    typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult & result) {}

  void feedback_callback(
    typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr goal_handle,
    const std::shared_ptr<const typename ActionT::Feedback> feedback)
  {
    write_feedback_ports(feedback);
  }

  // The main override required by a BT action
  BT::NodeStatus tick() override
  {
    if (!getInput("action_name", action_name_)) {
      throw BT::RuntimeError("Missing parameter [action_name] in ActionClientNode");
    }

    if (!getInput<std::chrono::milliseconds>("server_timeout", server_timeout_)) {
      throw BT::RuntimeError("Missing parameter [server_timeout] in ActionClientNode");
    }

    if (!getInput<std::shared_ptr<rclcpp::Node>>("ros2_node", ros2_node_)) {
      throw BT::RuntimeError("Missing parameter [ros2_node] in ActionClientNode");
    }

    read_input_ports(goal_);

    if (action_client_ == nullptr) {
      action_client_ = rclcpp_action::create_client<ActionT>(ros2_node_, action_name_);
    }

    // Make sure the action server is available there before continuing
    if (!action_client_->wait_for_action_server(std::chrono::milliseconds(server_timeout_))) {
      RCLCPP_ERROR(
        ros2_node_->get_logger(),
        "Timed out waiting for action server \"%s\" to become available", action_name_.c_str());
      return BT::NodeStatus::FAILURE;
    }

    // Enable result awareness by providing an empty lambda function
    auto send_goal_options = typename rclcpp_action::Client<ActionT>::SendGoalOptions();
    send_goal_options.result_callback = [](auto) {};
    send_goal_options.feedback_callback = std::bind(
      &ActionClientNode<ActionT>::feedback_callback, this,
      std::placeholders::_1, std::placeholders::_2);

new_goal_received:
    auto future_goal_handle = action_client_->async_send_goal(goal_, send_goal_options);

    if (future_goal_handle.wait_for(server_timeout_) == std::future_status::timeout) {
      throw std::runtime_error("send_goal failed");
    }

    goal_handle_ = future_goal_handle.get();
    if (!goal_handle_) {
      throw std::runtime_error("Goal was rejected by the action server");
    }

    auto future_result = action_client_->async_get_result(goal_handle_);
    std::future_status rc;
    do {
      rc = future_result.wait_for(server_timeout_);
      if (rc == std::future_status::timeout) {
        if (read_new_goal(goal_)) {
          // If we're received a new goal on the input port, cancel the current goal
          // and start a new one
          auto future = action_client_->async_cancel_goal(goal_handle_);
          if (future.wait_for(server_timeout_) == std::future_status::timeout) {
            RCLCPP_WARN(ros2_node_->get_logger(), "failed to cancel goal");
          } else {
            goto new_goal_received;
          }
        }

        // Yield to any other CoroActionNodes (coroutines)
        setStatusRunningAndYield();
      }
    } while (rc != std::future_status::ready);

    result_ = future_result.get();
    switch (result_.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        write_output_ports(result_);
        return BT::NodeStatus::SUCCESS;

      case rclcpp_action::ResultCode::ABORTED:
        return BT::NodeStatus::FAILURE;

      case rclcpp_action::ResultCode::CANCELED:
        return BT::NodeStatus::SUCCESS;

      default:
        throw std::logic_error("ActionClientNode::tick: invalid status value");
    }
  }

  // The other (optional) override required by a BT action. In this case, we
  // make sure to cancel the ROS2 action if it is still running.
  void halt() override
  {
    if (should_cancel_goal()) {
      auto future_cancel = action_client_->async_cancel_goal(goal_handle_);
      if (future_cancel.wait_for(server_timeout_) != std::future_status::ready) {
        RCLCPP_ERROR(
          ros2_node_->get_logger(),
          "Failed to cancel action server for %s", action_name_.c_str());
      }
    }

    CoroActionNode::halt();
  }

protected:
  bool should_cancel_goal()
  {
    // Shut the node down if it is currently running
    if (status() != BT::NodeStatus::RUNNING) {
      return false;
    }

    // Check if the goal is still in progress
    auto status = goal_handle_->get_status();
    return status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
           status == action_msgs::msg::GoalStatus::STATUS_EXECUTING;
  }

  typename std::shared_ptr<rclcpp_action::Client<ActionT>> action_client_;
  typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr goal_handle_;

  // The ROS node to use when calling the service
  rclcpp::Node::SharedPtr ros2_node_;

  std::string action_name_;

  std::chrono::milliseconds server_timeout_;

  typename ActionT::Goal goal_;
  typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult result_;
};

}  // namespace botanbot_pose_navigator

#endif  // botanbot_pose_navigator__ROS2_ACTION_CLIENT_NODE_HPP_
