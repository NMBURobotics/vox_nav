// Copyright (c) 2022 Fetullah Atas, Norwegian University of Life Sciences
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

#include "vox_nav_waypoint_nav_clients/navigate_through_gps_poses_client.hpp"

#include <string>
#include <vector>
#include <memory>
#include <algorithm>

namespace vox_nav_waypoint_nav_clients
{
  NavigateThroughGPSPosesClient::NavigateThroughGPSPosesClient()
  : Node("navigate_through_poses_client_node"), goal_done_(false)
  {
    navigate_through_gps_poses_action_client_ =
      rclcpp_action::create_client<ClientT>(
      this->get_node_base_interface(),
      this->get_node_graph_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "navigate_through_gps_poses");
    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&NavigateThroughGPSPosesClient::startGPSWaypointFollowing, this));
    // number of poses that robot will go throug, specified in yaml file
    this->declare_parameter("gps_poses", std::vector<std::string>({"0"}));
    gps_poses_from_yaml_ = loadGPSWaypointsFromYAML();
    RCLCPP_INFO(
      this->get_logger(),
      "Loaded %i gps waypoints from YAML, gonna pass them to navigate_through_gps_poses...",
      static_cast<int>(gps_poses_from_yaml_.size()));
    RCLCPP_INFO(
      this->get_logger(),
      "Created an Instance of NavigateThroughGPSPosesClient");
  }

  NavigateThroughGPSPosesClient::~NavigateThroughGPSPosesClient()
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Destroyed an Instance of NavigateThroughGPSPosesClient");
  }

  void NavigateThroughGPSPosesClient::startGPSWaypointFollowing()
  {
    using namespace std::placeholders;
    this->timer_->cancel();
    this->goal_done_ = false;

    if (!this->navigate_through_gps_poses_action_client_) {
      RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
    }

    auto is_action_server_ready =
      navigate_through_gps_poses_action_client_->wait_for_action_server(
      std::chrono::seconds(5));
    if (!is_action_server_ready) {
      RCLCPP_ERROR(
        this->get_logger(), "navigate_through_gps_poses action server is not available."
        " Make sure an instance of navigate_through_gps_poses is up and running");
      this->goal_done_ = true;
      return;
    }
    navigate_through_gps_poses_goal_ = ClientT::Goal();
    // Send the goal poses
    navigate_through_gps_poses_goal_.gps_poses = gps_poses_from_yaml_;

    RCLCPP_INFO(
      this->get_logger(),
      "Sending a path of %zu gps_poses:", navigate_through_gps_poses_goal_.gps_poses.size());
    for (auto pose : navigate_through_gps_poses_goal_.gps_poses) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "\t(%lf, %lf)", pose.latitude, pose.longitude);
    }

    auto goal_options =
      rclcpp_action::Client<ClientT>::SendGoalOptions();

    goal_options.goal_response_callback = std::bind(
      &NavigateThroughGPSPosesClient::goalResponseCallback, this, _1);

    goal_options.feedback_callback =
      std::bind(&NavigateThroughGPSPosesClient::feedbackCallback, this, _1, _2);

    goal_options.result_callback = std::bind(
      &NavigateThroughGPSPosesClient::resultCallback, this, _1);

    auto future_goal_handle = navigate_through_gps_poses_action_client_->async_send_goal(
      navigate_through_gps_poses_goal_, goal_options);

  }

  std::vector<sensor_msgs::msg::NavSatFix> NavigateThroughGPSPosesClient::loadGPSWaypointsFromYAML()
  {
    std::vector<std::string> poses_vector = this->get_parameter("gps_poses").as_string_array();
    std::vector<sensor_msgs::msg::NavSatFix> poses_msg_vector;

    for (auto && curr_pose : poses_vector) {
      try {
        this->declare_parameter(curr_pose, std::vector<double>({0}));
        std::vector<double> pose_vector = this->get_parameter(curr_pose).as_double_array();
        // throw exception if incorrect format was detected from yaml file reading
        if (pose_vector.size() != 2) {
          RCLCPP_FATAL(
            this->get_logger(),
            "Pose that was loaded from YAML file seems to have incorrect"
            " form, the right format is; wpN: [lat, long] with double types");
          throw rclcpp::exceptions::InvalidParametersException(
                  "[ERROR] See above error"
                  " the right format is; wpN: [lat, long] with double types"
                  "E.g pose0; [59.66429317, 10.76302156], please check YAML file");
        }
        // construct the gps waypoint and push them to their vector
        // lat, long , alt
        sensor_msgs::msg::NavSatFix gps_pose_to_navigate;
        gps_pose_to_navigate.latitude = pose_vector.at(0);
        gps_pose_to_navigate.longitude = pose_vector.at(1);
        poses_msg_vector.push_back(gps_pose_to_navigate);
      } catch (const std::exception & e) {
        std::cerr << e.what() << '\n';
      }
    }
    // return the read pair of this gps waypoint to it's caller
    return poses_msg_vector;
  }

  void NavigateThroughGPSPosesClient::goalResponseCallback(
    std::shared_future<NavigateThroughGPSPosesGoalHandle::SharedPtr> future)
  {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void NavigateThroughGPSPosesClient::feedbackCallback(
    NavigateThroughGPSPosesGoalHandle::SharedPtr,
    const std::shared_ptr<const ClientT::Feedback> feedback)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Current gps waypoint: %i", feedback->distance_remaining);
  }

  void NavigateThroughGPSPosesClient::resultCallback(
    const NavigateThroughGPSPosesGoalHandle::WrappedResult & result)
  {
    this->goal_done_ = true;
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Result received");
    //for (auto number : result.result->result) {
    //  RCLCPP_INFO(this->get_logger(), "Missed Waypoint %i", number);
    //}
  }

  bool NavigateThroughGPSPosesClient::is_goal_done() const
  {
    return this->goal_done_;
  }
}  // namespace vox_nav_waypoint_nav_clients

/**
 * @brief
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char const * argv[])
{
  rclcpp::init(argc, argv);
  auto client_node = std::make_shared
    <vox_nav_waypoint_nav_clients::NavigateThroughGPSPosesClient>();

  while (!client_node->is_goal_done()) {
    rclcpp::spin_some(client_node);
  }
  rclcpp::shutdown();
  return 0;
}
