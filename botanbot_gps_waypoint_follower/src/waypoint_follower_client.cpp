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
WayPointFollowerDemo::WayPointFollowerDemo()
: Node("waypoint_follower_client")
{
  // number of poses that robot will go throug, specified in yaml file
  this->declare_parameter("number_of_fake_move_tasks");
  rclcpp::Parameter number_of_fake_move_tasks =
    this->get_parameter("number_of_fake_move_tasks");

  // get all the poses in yaml file using provided utility function
  for (int i = 0; i < number_of_fake_move_tasks.as_int(); i++) {
    // prefix for each fake task, the last letter is basically index
    std::string this_param_name = "fake_task_pose_" + std::to_string(i);
    // we might have an exeption thrown in
    // utility  function loadVectorofDoubleAsPoseFromYAML,
    // hence this try catch block
    try {
      geometry_msgs::msg::PoseStamped
        this_pose_as_msg = loadVectorofDoubleAsPoseFromYAML(
        this_param_name);
      acummulated_poses_.push_back(this_pose_as_msg);
    } catch (const std::exception & e) {
      std::cerr << e.what() << '\n';
    }
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Loaded %i poses from Yaml, gonna run robot through them ...",
    acummulated_poses_.size());

  // Connect to waypoint following service provided by navigation2 ,
  // https://github.com/ros-planning/navigation2/tree/main/nav2_waypoint_follower
  waypoint_follower_action_client_ =
    rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(
    this, "FollowWa"
    "ypoint"
    "s");
  // initialize the Goal action variable
  waypoint_follower_goal_ = nav2_msgs::action::FollowWaypoints::Goal();

  RCLCPP_INFO(
    this->get_logger(),
    "Created an Instance of waypoint_follower_client");

  // Now follow the provided way points in yaml file
  try {
    startWaypointFollowing(acummulated_poses_);
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
  }
  RCLCPP_INFO(this->get_logger(), "GOING OUT OF SCOPE OF CONSTRUCTUR");
}

WayPointFollowerDemo::~WayPointFollowerDemo()
{
}

void WayPointFollowerDemo::startWaypointFollowing(
  std::vector<geometry_msgs::msg::PoseStamped> poses)
{
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
    &WayPointFollowerDemo::resultCallback, this,
    std::placeholders::_1);
  goal_options.goal_response_callback = std::bind(
    &WayPointFollowerDemo::goalResponseCallback, this,
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

geometry_msgs::msg::PoseStamped WayPointFollowerDemo::loadVectorofDoubleAsPoseFromYAML(
  std::string param_name)
{
  // The function loads an array of doubles where the items of this array
  // for 6D pose [x, y, z , rx , ry , rz]
  // node that postion is in terms of meters, and orientation is
  // in terms of radians
  this->declare_parameter(param_name);
  rclcpp::Parameter this_pose = this->get_parameter(param_name);
  std::vector<double> this_pose_as_vector = this_pose.as_double_array();

  // throw exeption if incorrect format was detected from yaml file reading
  if (this_pose_as_vector.size() < 6) {
    RCLCPP_FATAL(
      this->get_logger(),
      "Pose that was loaded from YAML file seems to have incorrect "
      "form, the right format is; x, y, z , rx, ry, rz");

    throw rclcpp::exceptions::InvalidParametersException(
            "[ERROR] See above error, correct "
            "format is; x, y, z , "
            "rx, ry, rz");
  }

  // this poses are assumed to be on global frame (map)
  geometry_msgs::msg::PoseStamped this_pose_msg;
  this_pose_msg.header.frame_id = "map";
  this_pose_msg.header.stamp = rclcpp::Clock().now();

  this_pose_msg.pose.position.x = this_pose_as_vector[0];
  this_pose_msg.pose.position.y = this_pose_as_vector[1];
  this_pose_msg.pose.position.z = this_pose_as_vector[2];

  // covert this RPY radian angles to geometry::msg::Quaternion type
  tf2::Quaternion quat_tf;
  quat_tf.setRPY(
    this_pose_as_vector[3],
    this_pose_as_vector[4], this_pose_as_vector[5]);
  geometry_msgs::msg::Quaternion quat_msg;
  tf2::convert(quat_msg, quat_tf);
  this_pose_msg.pose.orientation = quat_msg;

  // return the read pose to it's caller
  return this_pose_msg;
}

void WayPointFollowerDemo::resultCallback(
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
      "Missed %d points from given Yaml waypoints", number);
  }
}

void WayPointFollowerDemo::goalResponseCallback(
  std::shared_future<rclcpp_action::ClientGoalHandle
  <nav2_msgs::action::FollowWaypoints>::SharedPtr> future)
{
  auto goal_handle = future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(
      get_logger(),
      "navigate_to_pose action client failed to send goal to server.");
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
    <botanbot_gps_waypoint_follower::WayPointFollowerDemo>();

  while (node->current_goal_status_ !=
    botanbot_gps_waypoint_follower::ActionStatus::SUCCEEDED)
  {
    // rclcpp::spin(node);}
    return 0;
  }
}
