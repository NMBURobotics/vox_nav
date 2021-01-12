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

#include "botanbot_planning/plugins/se2_planner.hpp"
#include <pluginlib/class_list_macros.hpp>

#include <string>
#include <memory>

namespace botanbot_planning
{

SE2Planner::SE2Planner()
{
}

SE2Planner::~SE2Planner()
{

}

void SE2Planner::initialize(
  const rclcpp::Node::SharedPtr & parent,
  const std::string & plugin_name)
{
  node_ = parent;

  state_space_bounds_ = std::make_shared<ompl::base::RealVectorBounds>(2);

  node_->declare_parameter(plugin_name + ".enabled", true);
  node_->declare_parameter(plugin_name + ".planner_name", "PRMStar");
  node_->declare_parameter(plugin_name + ".planner_timeout", 5.0);
  node_->declare_parameter(plugin_name + ".interpolation_parameter", 50);
  node_->declare_parameter(plugin_name + ".octomap_filename", "/home/ros2-foxy/f.bt");
  node_->declare_parameter(plugin_name + ".octomap_voxel_size", 0.1);
  node_->declare_parameter(plugin_name + ".state_space_boundries.minx", -50.0);
  node_->declare_parameter(plugin_name + ".state_space_boundries.maxx", 50.0);
  node_->declare_parameter(plugin_name + ".state_space_boundries.miny", -10.0);
  node_->declare_parameter(plugin_name + ".state_space_boundries.maxy", 10.0);
  node_->declare_parameter(plugin_name + ".state_space_boundries.minz", -10.0);
  node_->declare_parameter(plugin_name + ".state_space_boundries.maxz", 10.0);
  node_->declare_parameter(plugin_name + ".robot_body_dimens.x", 1.0);
  node_->declare_parameter(plugin_name + ".robot_body_dimens.y", 0.8);
  node_->declare_parameter(plugin_name + ".robot_body_dimens.z", 0.6);

  node_->get_parameter(plugin_name + ".enabled", is_enabled_);
  node_->get_parameter(plugin_name + ".planner_name", planner_name_);
  node_->get_parameter(plugin_name + ".planner_timeout", planner_timeout_);
  node_->get_parameter(plugin_name + ".interpolation_parameter", interpolation_parameter_);
  node_->get_parameter(plugin_name + ".octomap_filename", octomap_filename_);
  node_->get_parameter(plugin_name + ".octomap_voxel_size", octomap_voxel_size_);

  state_space_bounds_->setLow(
    node_->get_parameter(plugin_name + ".state_space_boundries.minx").as_double());
  state_space_bounds_->setHigh(
    node_->get_parameter(plugin_name + ".state_space_boundries.maxx").as_double());

  std::shared_ptr<fcl::CollisionGeometry> robot_collision_geometry(new fcl::Box(
      node_->get_parameter(plugin_name + ".robot_body_dimens.x").as_double(),
      node_->get_parameter(plugin_name + ".robot_body_dimens.y").as_double(),
      node_->get_parameter(plugin_name + ".robot_body_dimens.z").as_double()));

  if (!is_enabled_) {
    RCLCPP_INFO(
      node_->get_logger(), "SE2Planner plugin is disabled.");
  } else {
    RCLCPP_INFO(
      node_->get_logger(), "Initializing SE2Planner plugin, selected planner is; %s",
      planner_name_.c_str());
  }

  robot_collision_object_ = std::make_shared<fcl::CollisionObject>(robot_collision_geometry);
  octomap_octree_ = std::make_shared<octomap::OcTree>(octomap_voxel_size_);
  octomap_octree_->readBinary(octomap_filename_);

  state_space_ = std::make_shared<ompl::base::ReedsSheppStateSpace>();
  state_space_->as<ompl::base::ReedsSheppStateSpace>()->setBounds(*state_space_bounds_);
  state_space_information_ = std::make_shared<ompl::base::SpaceInformation>(state_space_);
  state_space_information_->setStateValidityChecker(
    std::bind(&SE2Planner::isStateValid, this, std::placeholders::_1));
}

std::vector<geometry_msgs::msg::PoseStamped> SE2Planner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  if (!is_enabled_) {
    RCLCPP_WARN(
      node_->get_logger(),
      "SE2Planner plugin is disabled. Not performing anything returning an empty path"
    );
    return std::vector<geometry_msgs::msg::PoseStamped>();
  }

  ompl::base::ScopedState<ompl::base::SE2StateSpace> se2_start(state_space_),
  se2_goal(state_space_);
  // set the start and goal states
  tf2::Quaternion start_quat, goal_quat;
  tf2::fromMsg(start.pose.orientation, start_quat);
  tf2::fromMsg(goal.pose.orientation, goal_quat);

  se2_start[0] = start.pose.position.x;
  se2_start[1] = start.pose.position.y;
  se2_start[2] = start_quat.getAngle();
  se2_goal[0] = goal.pose.position.x;
  se2_goal[1] = goal.pose.position.y;
  se2_goal[2] = goal_quat.getAngle();

  // create a problem instance
  // define a simple setup class
  ompl::geometric::SimpleSetup simple_setup(state_space_);

  simple_setup.setStartAndGoalStates(se2_start, se2_goal);

  simple_setup.setup();

  // create a planner for the defined space
  ompl::base::PlannerPtr planner;
  if (!getSelectedPlanner(planner_name_, state_space_information_, planner)) {
    RCLCPP_WARN(
      node_->get_logger(), "Selected planner name: %s is not valid planner "
      "make sure you to set a valid planner name, Selecting a PRMStar as default planner",
      planner_name_.c_str());
    planner = ompl::base::PlannerPtr(new ompl::geometric::PRMstar(state_space_information_));
  }

  // print the settings for this space
  state_space_information_->printSettings(std::cout);

  // attempt to solve the problem within one second of planning time
  ompl::base::PlannerStatus solved = planner->solve(planner_timeout_);
  std::vector<geometry_msgs::msg::PoseStamped> plan_poses;

  if (solved) {
    simple_setup.simplifySolution();
    ompl::geometric::PathGeometric path = simple_setup.getSolutionPath();
    path.interpolate(interpolation_parameter_);

    for (std::size_t path_idx = 0; path_idx < path.getStateCount(); path_idx++) {
      // cast the abstract state type to the type we expect
      const ompl::base::ReedsSheppStateSpace::StateType * se2state =
        path.getState(path_idx)->as<ompl::base::ReedsSheppStateSpace::StateType>();

      tf2::Quaternion this_pose_quat;
      this_pose_quat.setRPY(0, 0, se2state->getYaw());

      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = start.header.frame_id;
      pose.header.stamp = rclcpp::Clock().now();
      pose.pose.position.x = se2state->getX();
      pose.pose.position.y = se2state->getY();
      pose.pose.position.z = 0.5;
      pose.pose.orientation.x = this_pose_quat.getX();
      pose.pose.orientation.y = this_pose_quat.getY();
      pose.pose.orientation.z = this_pose_quat.getZ();
      pose.pose.orientation.w = this_pose_quat.getW();
      plan_poses.push_back(pose);
    }
    RCLCPP_INFO(
      node_->get_logger(), "Found A plan with %i poses", plan_poses.size());
  } else {
    RCLCPP_WARN(
      node_->get_logger(), "No solution for requested path planning !");
  }
  return plan_poses;
}

bool SE2Planner::isStateValid(const ompl::base::State * state)
{
  // cast the abstract state type to the type we expect
  const ompl::base::ReedsSheppStateSpace::StateType * red_state =
    state->as<ompl::base::ReedsSheppStateSpace::StateType>();
  // check validity of state Fdefined by pos & rot
  fcl::Vec3f translation(red_state->getX(), red_state->getY(), 0.5);
  tf2::Quaternion myQuaternion;
  myQuaternion.setRPY(0, 0, red_state->getYaw());
  fcl::Quaternion3f rotation(myQuaternion.getX(), myQuaternion.getY(),
    myQuaternion.getZ(), myQuaternion.getW());
  robot_collision_object_->setTransform(rotation, translation);
  fcl::CollisionRequest requestType(1, false, 1, false);
  fcl::CollisionResult collisionResult;
  fcl::collide(
    robot_collision_object_.get(),
    fcl_octree_collision_object_.get(), requestType, collisionResult);
  return !collisionResult.isCollision();
}

bool SE2Planner::getSelectedPlanner(
  const std::string & planner_name,
  const ompl::base::SpaceInformationPtr & state_space_information,
  ompl::base::PlannerPtr planner)
{
  bool found_a_valid_planner = false;
  if (planner_name.c_str() == "PRMStar") {
    planner = ompl::base::PlannerPtr(new ompl::geometric::PRMstar(state_space_information));
    found_a_valid_planner = true;
  } else if (planner_name.c_str() == "RRTStar") {
    planner = ompl::base::PlannerPtr(new ompl::geometric::RRTstar(state_space_information));
    found_a_valid_planner = true;
  } else if (planner_name.c_str() == "RRTConnect") {
    planner = ompl::base::PlannerPtr(new ompl::geometric::RRTConnect(state_space_information));
    found_a_valid_planner = true;
  } else {
    found_a_valid_planner = false;
  }
  return found_a_valid_planner;
}

}  // namespace botanbot_planning

PLUGINLIB_EXPORT_CLASS(botanbot_planning::SE2Planner, botanbot_planning::PlannerCore)
