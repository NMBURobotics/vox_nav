// Copyright (c) 2020 Fetullah Atas, Norwegian University of Life Sciences
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
#include <vector>

namespace botanbot_planning
{

SE2Planner::SE2Planner()
{
}

SE2Planner::~SE2Planner()
{
}

void SE2Planner::initialize(
  rclcpp::Node * parent,
  const std::string & plugin_name)
{
  se2_space_bounds_ = std::make_shared<ompl::base::RealVectorBounds>(2);
  octomap_msg_ = std::make_shared<octomap_msgs::msg::Octomap>();
  is_octomap_ready_ = false;

  parent->declare_parameter(plugin_name + ".enabled", true);
  parent->declare_parameter(plugin_name + ".planner_name", "PRMStar");
  parent->declare_parameter(plugin_name + ".planner_timeout", 5.0);
  parent->declare_parameter(plugin_name + ".interpolation_parameter", 50);
  parent->declare_parameter(plugin_name + ".octomap_topic", "octomap");
  parent->declare_parameter(plugin_name + ".octomap_voxel_size", 0.2);
  parent->declare_parameter(plugin_name + ".se2_space", "REEDS");
  parent->declare_parameter(plugin_name + ".state_space_boundries.minx", -50.0);
  parent->declare_parameter(plugin_name + ".state_space_boundries.maxx", 50.0);
  parent->declare_parameter(plugin_name + ".state_space_boundries.miny", -10.0);
  parent->declare_parameter(plugin_name + ".state_space_boundries.maxy", 10.0);
  parent->declare_parameter(plugin_name + ".state_space_boundries.minyaw", -3.14);
  parent->declare_parameter(plugin_name + ".state_space_boundries.maxyaw", 3.14);
  parent->declare_parameter(plugin_name + ".robot_body_dimens.x", 1.5);
  parent->declare_parameter(plugin_name + ".robot_body_dimens.y", 1.5);
  parent->declare_parameter(plugin_name + ".robot_body_dimens.z", 0.4);

  parent->get_parameter(plugin_name + ".enabled", is_enabled_);
  parent->get_parameter(plugin_name + ".planner_name", planner_name_);
  parent->get_parameter(plugin_name + ".planner_timeout", planner_timeout_);
  parent->get_parameter(plugin_name + ".interpolation_parameter", interpolation_parameter_);
  parent->get_parameter(plugin_name + ".octomap_topic", octomap_topic_);
  parent->get_parameter(plugin_name + ".octomap_voxel_size", octomap_voxel_size_);
  parent->get_parameter(plugin_name + ".se2_space", selected_se2_space_name_);

  se2_space_bounds_->setLow(
    0, parent->get_parameter(plugin_name + ".state_space_boundries.minx").as_double());
  se2_space_bounds_->setHigh(
    0, parent->get_parameter(plugin_name + ".state_space_boundries.maxx").as_double());
  se2_space_bounds_->setLow(
    1, parent->get_parameter(plugin_name + ".state_space_boundries.miny").as_double());
  se2_space_bounds_->setHigh(
    1, parent->get_parameter(plugin_name + ".state_space_boundries.maxy").as_double());
  se2_space_bounds_->setLow(
    2, parent->get_parameter(plugin_name + ".state_space_boundries.minyaw").as_double());
  se2_space_bounds_->setHigh(
    2, parent->get_parameter(plugin_name + ".state_space_boundries.maxyaw").as_double());

  se2_space_ = std::make_shared<ompl::base::ReedsSheppStateSpace>();
  se2_space_->as<ompl::base::ReedsSheppStateSpace>()->setBounds(*se2_space_bounds_);
  if (selected_se2_space_name_ == "DUBINS") {
    se2_space_ = std::make_shared<ompl::base::DubinsStateSpace>();
    se2_space_->as<ompl::base::DubinsStateSpace>()->setBounds(*se2_space_bounds_);

  } else if (selected_se2_space_name_ == "SE2") {
    se2_space_ = std::make_shared<ompl::base::SE2StateSpace>();
    se2_space_->as<ompl::base::SE2StateSpace>()->setBounds(*se2_space_bounds_);
  }

  typedef std::shared_ptr<fcl::CollisionGeometry> CollisionGeometryPtr_t;
  CollisionGeometryPtr_t robot_body_box(new fcl::Box(
      parent->get_parameter(plugin_name + ".robot_body_dimens.x").as_double(),
      parent->get_parameter(plugin_name + ".robot_body_dimens.y").as_double(),
      parent->get_parameter(plugin_name + ".robot_body_dimens.z").as_double()));
  fcl::Transform3f tf2;
  fcl::CollisionObject robot_body_box_object(robot_body_box, tf2);
  robot_collision_object_ = std::make_shared<fcl::CollisionObject>(robot_body_box_object);
  octomap_subscriber_ = parent->create_subscription<octomap_msgs::msg::Octomap>(
    octomap_topic_, rclcpp::SystemDefaultsQoS(),
    std::bind(&SE2Planner::octomapCallback, this, std::placeholders::_1));

  se2_state_space_information_ = std::make_shared<ompl::base::SpaceInformation>(se2_space_);
  se2_state_space_information_->setStateValidityChecker(
    std::bind(&SE2Planner::isStateValid, this, std::placeholders::_1));

  if (!is_enabled_) {
    RCLCPP_WARN(
      logger_, "SE2PlannerControlSpace plugin is disabled.");
  }
  RCLCPP_INFO(logger_, "Selected planner is: %s", planner_name_.c_str());
}

std::vector<geometry_msgs::msg::PoseStamped> SE2Planner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  if (!is_enabled_) {
    RCLCPP_WARN(
      logger_,
      "SE2Planner plugin is disabled. Not performing anything returning an empty path"
    );
    return std::vector<geometry_msgs::msg::PoseStamped>();
  }

  ompl::base::ScopedState<ompl::base::DubinsStateSpace>
  se2_start(se2_space_),
  se2_goal(se2_space_);
  // set the start and goal states
  double start_yaw, goal_yaw, nan;
  botanbot_utilities::getRPYfromMsgQuaternion(start.pose.orientation, nan, nan, start_yaw);
  botanbot_utilities::getRPYfromMsgQuaternion(goal.pose.orientation, nan, nan, goal_yaw);

  se2_start[0] = start.pose.position.x;
  se2_start[1] = start.pose.position.y;
  se2_start[2] = start_yaw;

  se2_goal[0] = goal.pose.position.x;
  se2_goal[1] = goal.pose.position.y;
  se2_goal[2] = goal_yaw;

  // create a problem instance
  // define a simple setup class
  ompl::geometric::SimpleSetup simple_setup(se2_space_);
  simple_setup.setStateValidityChecker(
    [this](const ompl::base::State * state)
    {
      return isStateValid(state);
    });
  simple_setup.setStartAndGoalStates(se2_start, se2_goal);

  // objective is to minimize the planned path
  ompl::base::OptimizationObjectivePtr objective(
    new ompl::base::PathLengthOptimizationObjective(simple_setup.getSpaceInformation()));
  simple_setup.setOptimizationObjective(objective);

  simple_setup.setup();

  // create a planner for the defined space
  ompl::base::PlannerPtr planner;
  initializeSelectedPlanner(planner, planner_name_, simple_setup.getSpaceInformation());

  simple_setup.setPlanner(planner);
  // print the settings for this space
  se2_state_space_information_->printSettings(std::cout);

  // attempt to solve the problem within one second of planning time
  ompl::base::PlannerStatus solved = simple_setup.solve(planner_timeout_);
  std::vector<geometry_msgs::msg::PoseStamped> plan_poses;

  if (solved) {
    ompl::geometric::PathGeometric path = simple_setup.getSolutionPath();
    // Path smoothing using bspline
    ompl::geometric::PathSimplifier * path_simlifier =
      new ompl::geometric::PathSimplifier(simple_setup.getSpaceInformation());
    path_simlifier->smoothBSpline(path, 3);
    path_simlifier->collapseCloseVertices(path, 3);
    path.checkAndRepair(2);
    path.interpolate(interpolation_parameter_);

    for (std::size_t path_idx = 0; path_idx < path.getStateCount(); path_idx++) {
      // cast the abstract state type to the type we expect
      const ompl::base::SE2StateSpace::StateType * se2_state =
        path.getState(path_idx)->as<ompl::base::SE2StateSpace::StateType>();

      tf2::Quaternion this_pose_quat;
      this_pose_quat.setRPY(0, 0, se2_state->getYaw());

      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = start.header.frame_id;
      pose.header.stamp = rclcpp::Clock().now();
      pose.pose.position.x = se2_state->getX();
      pose.pose.position.y = se2_state->getY();
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

bool SE2Planner::isStateValid(const ompl::base::State * state)
{
  if (is_octomap_ready_) {
    std::call_once(
      fcl_tree_from_octomap_once_, [this]() {
        std::shared_ptr<octomap::OcTree> octomap_octree =
        std::make_shared<octomap::OcTree>(0.2);
        octomap_msgs::readTree<octomap::OcTree>(octomap_octree.get(), *octomap_msg_);
        fcl_octree_ = std::make_shared<fcl::OcTree>(octomap_octree);
        fcl_octree_collision_object_ = std::make_shared<fcl::CollisionObject>(
          std::shared_ptr<fcl::CollisionGeometry>(fcl_octree_));
        RCLCPP_INFO(
          logger_,
          "Recieved a valid Octomap, A FCL collision tree will be created from this "
          "octomap for state validity(aka collision check)");
      });
  } else {
    RCLCPP_ERROR(
      logger_,
      "The Octomap has not been recieved correctly, Collision check "
      "cannot be processed without a valid Octomap!");
    return false;
  }
  // cast the abstract state type to the type we expect
  const ompl::base::SE2StateSpace::StateType * se2_state =
    state->as<ompl::base::SE2StateSpace::StateType>();
  // check validity of state Fdefined by pos & rot
  fcl::Vec3f translation(se2_state->getX(), se2_state->getY(), 0.5);
  tf2::Quaternion myQuaternion;
  myQuaternion.setRPY(0, 0, se2_state->getYaw());
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

void SE2Planner::octomapCallback(
  const octomap_msgs::msg::Octomap::ConstSharedPtr msg)
{
  const std::lock_guard<std::mutex> lock(octomap_mutex_);
  if (!is_octomap_ready_) {
    is_octomap_ready_ = true;
    octomap_msg_ = msg;
  }
}

void SE2Planner::initializeSelectedPlanner(
  ompl::base::PlannerPtr & planner,
  const std::string & selected_planner_name,
  const ompl::base::SpaceInformationPtr & si)
{
  if (selected_planner_name == std::string("PRMStar")) {
    planner = ompl::base::PlannerPtr(new ompl::geometric::PRMstar(si));
  } else if (selected_planner_name == std::string("RRTstar")) {
    planner = ompl::base::PlannerPtr(new ompl::geometric::RRTstar(si));
  } else if (selected_planner_name == std::string("RRTConnect")) {
    planner = ompl::base::PlannerPtr(new ompl::geometric::RRTConnect(si));
  } else if (selected_planner_name == std::string("KPIECE1")) {
    planner = ompl::base::PlannerPtr(new ompl::geometric::KPIECE1(si));
  } else if (selected_planner_name == std::string("SBL")) {
    planner = ompl::base::PlannerPtr(new ompl::geometric::SBL(si));
  } else if (selected_planner_name == std::string("SST")) {
    planner = ompl::base::PlannerPtr(new ompl::geometric::SST(si));
  } else {
    RCLCPP_WARN(
      logger_,
      "Selected planner is not Found in available planners, using the default planner: KPIECE1");
    planner = ompl::base::PlannerPtr(new ompl::geometric::KPIECE1(si));
  }
}

}  // namespace botanbot_planning

PLUGINLIB_EXPORT_CLASS(botanbot_planning::SE2Planner, botanbot_planning::PlannerCore)
