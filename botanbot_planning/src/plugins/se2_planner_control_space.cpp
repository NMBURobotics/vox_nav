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

#include "botanbot_planning/plugins/se2_planner_control_space.hpp"
#include <pluginlib/class_list_macros.hpp>

#include <string>
#include <memory>

namespace botanbot_planning
{

SE2PlannerControlSpace::SE2PlannerControlSpace()
{
}

SE2PlannerControlSpace::~SE2PlannerControlSpace()
{
}

void SE2PlannerControlSpace::initialize(
  rclcpp::Node * parent,
  const std::string & plugin_name)
{
  state_space_bounds_ = std::make_shared<ompl::base::RealVectorBounds>(2);

  parent->declare_parameter(plugin_name + ".enabled", true);
  parent->declare_parameter(plugin_name + ".planner_name", "PRMStar");
  parent->declare_parameter(plugin_name + ".planner_timeout", 5.0);
  parent->declare_parameter(plugin_name + ".interpolation_parameter", 50);
  parent->declare_parameter(plugin_name + ".octomap_filename", "/home/ros2-foxy/f.bt");
  parent->declare_parameter(plugin_name + ".octomap_voxel_size", 0.1);
  parent->declare_parameter(plugin_name + ".state_space_boundries.minx", -50.0);
  parent->declare_parameter(plugin_name + ".state_space_boundries.maxx", 50.0);
  parent->declare_parameter(plugin_name + ".state_space_boundries.miny", -10.0);
  parent->declare_parameter(plugin_name + ".state_space_boundries.maxy", 10.0);
  parent->declare_parameter(plugin_name + ".state_space_boundries.minz", -10.0);
  parent->declare_parameter(plugin_name + ".state_space_boundries.maxz", 10.0);
  parent->declare_parameter(plugin_name + ".robot_body_dimens.x", 1.0);
  parent->declare_parameter(plugin_name + ".robot_body_dimens.y", 0.8);
  parent->declare_parameter(plugin_name + ".robot_body_dimens.z", 0.6);

  parent->get_parameter(plugin_name + ".enabled", is_enabled_);
  parent->get_parameter(plugin_name + ".planner_name", planner_name_);
  parent->get_parameter(plugin_name + ".planner_timeout", planner_timeout_);
  parent->get_parameter(plugin_name + ".interpolation_parameter", interpolation_parameter_);
  parent->get_parameter(plugin_name + ".octomap_filename", octomap_filename_);
  parent->get_parameter(plugin_name + ".octomap_voxel_size", octomap_voxel_size_);

  state_space_bounds_->setLow(
    parent->get_parameter(plugin_name + ".state_space_boundries.minx").as_double());
  state_space_bounds_->setHigh(
    parent->get_parameter(plugin_name + ".state_space_boundries.maxx").as_double());

  std::shared_ptr<fcl::CollisionGeometry> robot_collision_geometry(new fcl::Box(
      parent->get_parameter(plugin_name + ".robot_body_dimens.x").as_double(),
      parent->get_parameter(plugin_name + ".robot_body_dimens.y").as_double(),
      parent->get_parameter(plugin_name + ".robot_body_dimens.z").as_double()));

  if (!is_enabled_) {
    RCLCPP_INFO(
      logger_, "SE2PlannerControlSpace plugin is disabled.");
  } else {
    RCLCPP_INFO(
      logger_, "Initializing SE2PlannerControlSpace plugin, selected planner is; %s",
      planner_name_.c_str());
  }

  robot_collision_object_ = std::make_shared<fcl::CollisionObject>(robot_collision_geometry);
  octomap_octree_ = std::make_shared<octomap::OcTree>(octomap_voxel_size_);
  octomap_octree_->readBinary(octomap_filename_);

  state_space_ = std::make_shared<ompl::base::ReedsSheppStateSpace>();
  state_space_->as<ompl::base::ReedsSheppStateSpace>()->setBounds(*state_space_bounds_);
  state_space_information_ = std::make_shared<ompl::base::SpaceInformation>(state_space_);
  state_space_information_->setStateValidityChecker(
    std::bind(&SE2PlannerControlSpace::isStateValid, this, std::placeholders::_1));
}

std::vector<geometry_msgs::msg::PoseStamped> SE2PlannerControlSpace::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  if (!is_enabled_) {
    RCLCPP_WARN(
      logger_,
      "SE2PlannerControlSpace plugin is disabled. Not performing anything returning an empty path"
    );
    return std::vector<geometry_msgs::msg::PoseStamped>();
  }

  // create a control space
  auto control_space(std::make_shared<ompl::control::RealVectorControlSpace>(state_space_, 2));
  // set the bounds for the control space
  ompl::base::RealVectorBounds control_space_bounds(2);
  control_space_bounds.setLow(-0.1);
  control_space_bounds.setHigh(0.1);
  control_space->setBounds(control_space_bounds);


  // construct an instance of  space information from this control space
  auto control_space_information(std::make_shared<ompl::control::SpaceInformation>(
      state_space_,
      control_space));

  // set state validity checking for this space
  control_space_information->setStateValidityChecker(
    [this](const ompl::base::State * state) {return isStateValid(state);});
  // set the state propagation routine
  control_space_information->setStatePropagator(propagate);

  // set the start and goal states
  tf2::Quaternion start_quat, goal_quat;
  tf2::fromMsg(start.pose.orientation, start_quat);
  tf2::fromMsg(goal.pose.orientation, goal_quat);


  ompl::base::ScopedState<ompl::base::SE2StateSpace> se2_start(state_space_),
  se2_goal(state_space_);

  se2_start->setX(start.pose.position.x);
  se2_start->setY(start.pose.position.y);
  se2_start->setYaw(start_quat.getAngle());

  se2_goal->setX(goal.pose.position.x);
  se2_goal->setY(goal.pose.position.y);
  se2_goal->setYaw(goal_quat.getAngle());

  // create a problem instance
  auto pdef(std::make_shared<ompl::base::ProblemDefinition>(control_space_information));

  // set the start and goal states
  pdef->setStartAndGoalStates(se2_start, se2_goal, 0.2);

  // create a planner for the defined space
  ompl::base::PlannerPtr planner;
  if (planner_name_ == std::string("RRT")) {
    planner = ompl::base::PlannerPtr(
      new ompl::control::RRT(control_space_information));
  } else if (planner_name_ == std::string("SST")) {
    planner = ompl::base::PlannerPtr(
      new ompl::control::SST(control_space_information) );
  } else if (planner_name_ == std::string("EST")) {
    planner = ompl::base::PlannerPtr(
      new ompl::control::EST(control_space_information) );
  } else if (planner_name_ == std::string("KPIECE1")) {
    planner = ompl::base::PlannerPtr(
      new ompl::control::KPIECE1(control_space_information) );
  } else {
    RCLCPP_WARN(
      logger_,
      "Selected planner is not Found in available planners, using the default planner: %s",
      planner_name_.c_str());
  }
  RCLCPP_INFO(logger_, "Selected planner is: %s", planner_name_.c_str());

  // set the problem we are trying to solve for the planner
  planner->setProblemDefinition(pdef);

  // perform setup steps for the planner
  planner->setup();

  // print the problem settings
  pdef->print(std::cout);

  ompl::base::PlannerStatus solved = planner->ompl::base::Planner::solve(planner_timeout_);
  std::vector<geometry_msgs::msg::PoseStamped> plan_poses;

  if (solved) {

    ompl::geometric::PathGeometric * path =
      pdef->getSolutionPath()->as<ompl::geometric::PathGeometric>();
    path->interpolate(interpolation_parameter_);

    for (std::size_t path_idx = 0; path_idx < path->getStateCount(); path_idx++) {

      // cast the abstract state type to the type we expect
      const ompl::base::SE2StateSpace::StateType * se2state =
        path->getState(path_idx)->as<ompl::base::SE2StateSpace::StateType>();

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
      logger_, "Found A plan with %i poses", plan_poses.size());
  } else {
    RCLCPP_WARN(
      logger_, "No solution for requested path planning !");
  }
  return plan_poses;
}

bool SE2PlannerControlSpace::isStateValid(const ompl::base::State * state)
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

bool SE2PlannerControlSpace::getSelectedPlanner(
  const std::string & planner_name,
  const ompl::base::SpaceInformationPtr & state_space_information,
  ompl::base::PlannerPtr planner)
{
  bool found_a_valid_planner = false;
  if (planner_name == std::string("PRMStar")) {
    planner = ompl::base::PlannerPtr(new ompl::geometric::PRMstar(state_space_information));
    found_a_valid_planner = true;
    return found_a_valid_planner;
  } else if (planner_name == std::string("RRTStar")) {
    planner = ompl::base::PlannerPtr(new ompl::geometric::RRTstar(state_space_information));
    found_a_valid_planner = true;
    return found_a_valid_planner;
  } else if (planner_name == std::string("RRTConnect")) {
    planner = ompl::base::PlannerPtr(new ompl::geometric::RRTConnect(state_space_information));
    found_a_valid_planner = true;
    return found_a_valid_planner;
  } else {
    found_a_valid_planner = false;
    return found_a_valid_planner;
  }
  return found_a_valid_planner;
}

void SE2PlannerControlSpace::propagate(
  const ompl::base::State * start, const ompl::control::Control * control, const double duration,
  ompl::base::State * result)
{
  const auto * se2state = start->as<ompl::base::SE2StateSpace::StateType>();
  const double * pos = se2state->as<ompl::base::RealVectorStateSpace::StateType>(0)->values;
  const double rot = se2state->as<ompl::base::SO2StateSpace::StateType>(1)->value;
  const double * ctrl = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
  result->as<ompl::base::SE2StateSpace::StateType>()->setXY(
    pos[0] + ctrl[0] * duration * cos(rot),
    pos[1] + ctrl[0] * duration * sin(rot));
  result->as<ompl::base::SE2StateSpace::StateType>()->setYaw(
    rot + ctrl[1] * duration);
}


}  // namespace botanbot_planning

PLUGINLIB_EXPORT_CLASS(botanbot_planning::SE2PlannerControlSpace, botanbot_planning::PlannerCore)
