// Copyright (c) 2020 Fetullah Atas
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


#include "botanbot_gps_waypoint_follower/waypoint_follower_client.hpp"

namespace botanbot_gps_waypoint_follower
{
GPSWayPointFollower::GPSWayPointFollower()
: Node("gps_waypoint_follower")
{
  // Connect to waypoint following service provided by navigation2 ,
  // https://github.com/ros-planning/navigation2/tree/main/nav2_waypoint_follower
  waypoint_follower_action_client_ =
    rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(
    this, "FollowWaypoints");
  // initialize the Goal action variable
  waypoint_follower_goal_ = nav2_msgs::action::FollowWaypoints::Goal();

  // create client to convert LL to map client from robot_localization
  from_ll_to_map_client_ =
    this->create_client<robot_localization::srv::FromLL>("/fromLL");
  // number of poses that robot will go throug, specified in yaml file
  this->declare_parameter("number_of_gps_waypoints");
  rclcpp::Parameter number_of_gps_waypoints =
    this->get_parameter("number_of_gps_waypoints");

  // get all the poses in yaml file using provided utility function
  for (int i = 0; i < number_of_gps_waypoints.as_int(); i++) {
    // prefix for each fake task, the last letter is basically index
    std::string this_param_name = "gps_waypoint" + std::to_string(i);
    // we might have an exeption thrown in
    // utility  function loadVectorofDoubleAsPoseFromYAML,
    // hence this try catch block
    try {
      std::pair<double, double>
      this_waypoint_as_pair_of_doubles = loadVectorofDoubleAsPairFromYAML(
        this_param_name);
      acummulated_gps_waypoints_.push_back(this_waypoint_as_pair_of_doubles);
    } catch (const std::exception & e) {
      std::cerr << e.what() << '\n';
    }
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Loaded %i GPS waypoints from YAML, gonna convert them to Map frame...",
    acummulated_gps_waypoints_.size());
  RCLCPP_INFO(
    this->get_logger(),
    "Created an Instance of waypoint_follower_client");
  RCLCPP_INFO(this->get_logger(), "GOING OUT OF SCOPE OF CONSTRUCTUR");
}

GPSWayPointFollower::~GPSWayPointFollower()
{
}

void GPSWayPointFollower::startWaypointFollowing()
{
  std::vector<geometry_msgs::msg::PoseStamped> poses = acummulated_poses_;
  // action server needs to be active, wait a it and
  // make sure it is up and running
  auto is_action_server_ready =
    waypoint_follower_action_client_->wait_for_action_server(
    std::chrono::seconds(
      5));
  if (!is_action_server_ready) {
    RCLCPP_ERROR(
      this->get_logger(), "FollowWaypoints action server is not available."
      " Is the initial pose set?");
    return;
  }

  // Send the goal poses
  waypoint_follower_goal_.poses = poses;

  RCLCPP_INFO(
    this->get_logger(),
    "Sending a path of %zu waypoints:", waypoint_follower_goal_.poses.size());
  for (auto waypoint : waypoint_follower_goal_.poses) {
    RCLCPP_DEBUG(
      this->get_logger(),
      "\t(%lf, %lf)", waypoint.pose.position.x, waypoint.pose.position.y);
  }

  // Enable result awareness by providing an empty lambda function
  // auto send_goal_options =
  // rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SendGoalOptions();
  // send_goal_options.result_callback = [](auto) {};
  auto goal_options =
    rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SendGoalOptions();
  goal_options.result_callback = std::bind(
    &GPSWayPointFollower::resultCallback, this,
    std::placeholders::_1);
  goal_options.goal_response_callback = std::bind(
    &GPSWayPointFollower::goalResponseCallback, this,
    std::placeholders::_1);

  auto future_goal_handle = waypoint_follower_action_client_->async_send_goal(
    waypoint_follower_goal_, goal_options);
  if (rclcpp::spin_until_future_complete(
      this->get_node_base_interface(),
      future_goal_handle) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "Send goal call failed");
    return;
  }

  // Get the goal handle and save so that we can check
  // on completion in the timer callback
  waypoint_follower_goal_handle_ = future_goal_handle.get();
  if (!waypoint_follower_goal_handle_) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    return;
  }
}

void GPSWayPointFollower::convertGPSWaypointstoPosesinMap()
{
  for (auto && curr_gps_waypoint : acummulated_gps_waypoints_) {
    auto fromLLRequest = std::make_shared<robot_localization::srv::FromLL::Request>();
    fromLLRequest->ll_point.longitude = curr_gps_waypoint.first;
    fromLLRequest->ll_point.latitude = curr_gps_waypoint.second;

    if (!from_ll_to_map_client_->wait_for_service((std::chrono::seconds(5)))) {
      RCLCPP_ERROR(
        this->get_logger(),
        "fromLL service from robot_localization is not available"
        "cannot convert GPS wapoint to Map frame poses");
      return;
    }
    auto result = from_ll_to_map_client_->async_send_request(fromLLRequest);
    auto ret = rclcpp::spin_until_future_complete(
      this->shared_from_this(), result, std::chrono::seconds(5));

    // Timing and spinning is updated as per ros2
    if (ret == rclcpp::FutureReturnCode::SUCCESS) {
      rclcpp::Rate(2).sleep();
      rclcpp::spin_some(this->shared_from_this());

      // this poses are assumed to be on global frame (map)
      geometry_msgs::msg::PoseStamped this_pose_msg;
      this_pose_msg.header.frame_id = "map";
      this_pose_msg.header.stamp = rclcpp::Clock().now();

      this_pose_msg.pose.position.x = result.get()->map_point.x;
      this_pose_msg.pose.position.y = result.get()->map_point.y;
      this_pose_msg.pose.position.z = 0;

      // convert this RPY radian angles to geometry::msg::Quaternion type
      tf2::Quaternion quat_tf;
      quat_tf.setRPY(0, 0, 0);
      geometry_msgs::msg::Quaternion quat_msg;
      tf2::convert(quat_msg, quat_tf);
      this_pose_msg.pose.orientation = quat_msg;

      acummulated_poses_.push_back(this_pose_msg);

      RCLCPP_INFO(
        this->get_logger(),
        "GPS Long, Lat: %.8f , %.8f converted to "
        "Map Point X,Y: %.8f , %.8f", fromLLRequest->ll_point.longitude,
        fromLLRequest->ll_point.latitude, this_pose_msg.pose.position.x,
        this_pose_msg.pose.position.y);
    }
  }
  RCLCPP_INFO(
    this->get_logger(),
    "Converted all %i GPS waypoint o Map frame", acummulated_poses_.size());
}

std::pair<double, double> GPSWayPointFollower::loadVectorofDoubleAsPairFromYAML(
  std::string param_name)
{
  // The function loads an array of doubles where the items of this array
  // for 2D pose [x, y] Supposed to be RAW GPS coordnates

  this->declare_parameter(param_name);
  rclcpp::Parameter this_pair_rclcpp = this->get_parameter(param_name);
  std::vector<double> this_pair_as_vector = this_pair_rclcpp.as_double_array();

  // throw exeption if incorrect format was detected from yaml file reading
  if (this_pair_as_vector.size() < 2) {
    RCLCPP_FATAL(
      this->get_logger(),
      "GPS waypoint that was loaded from YAML file seems to have incorrect"
      "form, the right format is; x,y with doble types");
    throw rclcpp::exceptions::InvalidParametersException(
            "[ERROR] See above error, correct "
            "format is; x, y as double types, please chechk YAML file");
  }
  // construct the gps waypoint and push them to pair
  std::pair<double, double> this_pair(0.0000, 0.0000);
  this_pair.first = this_pair_as_vector.at(0);
  this_pair.second = this_pair_as_vector.at(1);

  // return the read pair of this gps waypoint to it's caller
  return this_pair;
}

void GPSWayPointFollower::resultCallback(
  const rclcpp_action::ClientGoalHandle
  <nav2_msgs::action::FollowWaypoints>::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      current_goal_status_ = ActionStatus::SUCCEEDED;
      return;
    case rclcpp_action::ResultCode::ABORTED:
      current_goal_status_ = ActionStatus::FAILED;
      return;
    case rclcpp_action::ResultCode::CANCELED:
      current_goal_status_ = ActionStatus::FAILED;
      return;
    default:
      current_goal_status_ = ActionStatus::UNKNOWN;
      return;
  }

  RCLCPP_INFO(this->get_logger(), "Result received");
  for (auto number : result.result->missed_waypoints) {
    RCLCPP_INFO(
      this->get_logger(),
      "Missed"
      "%d points from given Yaml waypoints", number);
  }
}

void GPSWayPointFollower::goalResponseCallback(
  std::shared_future<rclcpp_action::ClientGoalHandle
  <nav2_msgs::action::FollowWaypoints>::SharedPtr> future)
{
  auto goal_handle = future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(
      get_logger(),
      "navigate_to_pose action client failed to send goal to server");
    current_goal_status_ = ActionStatus::FAILED;
  }
}

}  // namespace botanbot_gps_waypoint_follower

/**
 * @brief Entry point for Way Point following demo Node
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char const * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared
    <botanbot_gps_waypoint_follower::GPSWayPointFollower>();

  node->convertGPSWaypointstoPosesinMap();
  // Now follow the provided way points in yaml file
  try {
    node->startWaypointFollowing();
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
  }
  while (node->current_goal_status_ !=
    botanbot_gps_waypoint_follower::ActionStatus::SUCCEEDED)
  {
    rclcpp::spin(node);
    return 0;
  }
}
