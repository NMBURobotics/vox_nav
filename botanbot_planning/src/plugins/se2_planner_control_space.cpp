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

#include "botanbot_planning/plugins/se2_planner_control_space.hpp"
#include <pluginlib/class_list_macros.hpp>

#include <string>
#include <memory>
#include <vector>
#include <mutex>

namespace botanbot_planning
{

SE2PlannerControlSpace::SE2PlannerControlSpace()
: se2_space_(new ompl::base::ReedsSheppStateSpace()),
  velocity_space_(new ompl::base::RealVectorStateSpace(1)),
  space_(se2_space_ + velocity_space_),
  is_octomap_ready_(false)
{
}

SE2PlannerControlSpace::~SE2PlannerControlSpace()
{
}

void SE2PlannerControlSpace::initialize(
  rclcpp::Node * parent,
  const std::string & plugin_name)
{
  octomap_msg_ = std::make_shared<octomap_msgs::msg::Octomap>();
  se2_space_bounds_ = std::make_shared<ompl::base::RealVectorBounds>(2);
  velocity_space_bounds_ = std::make_shared<ompl::base::RealVectorBounds>(1);

  parent->declare_parameter(plugin_name + ".enabled", true);
  parent->declare_parameter(plugin_name + ".planner_name", "PRMStar");
  parent->declare_parameter(plugin_name + ".planner_timeout", 5.0);
  parent->declare_parameter(plugin_name + ".interpolation_parameter", 50);
  parent->declare_parameter(plugin_name + ".octomap_topic", "octomap");
  parent->declare_parameter(plugin_name + ".octomap_voxel_size", 0.2);
  parent->declare_parameter(plugin_name + ".state_space_boundries.minx", -50.0);
  parent->declare_parameter(plugin_name + ".state_space_boundries.maxx", 50.0);
  parent->declare_parameter(plugin_name + ".state_space_boundries.miny", -10.0);
  parent->declare_parameter(plugin_name + ".state_space_boundries.maxy", 10.0);
  parent->declare_parameter(plugin_name + ".state_space_boundries.minz", -10.0);
  parent->declare_parameter(plugin_name + ".state_space_boundries.maxz", 10.0);
  parent->declare_parameter(plugin_name + ".velocity_space_boundries.min_v", -0.3);
  parent->declare_parameter(plugin_name + ".velocity_space_boundries.max_v", 0.3);
  parent->declare_parameter(plugin_name + ".robot_body_dimens.x", 1.0);
  parent->declare_parameter(plugin_name + ".robot_body_dimens.y", 0.8);
  parent->declare_parameter(plugin_name + ".robot_body_dimens.z", 0.6);

  parent->get_parameter(plugin_name + ".enabled", is_enabled_);
  parent->get_parameter(plugin_name + ".planner_name", planner_name_);
  parent->get_parameter(plugin_name + ".planner_timeout", planner_timeout_);
  parent->get_parameter(plugin_name + ".interpolation_parameter", interpolation_parameter_);
  parent->get_parameter(plugin_name + ".octomap_topic", octomap_topic_);
  parent->get_parameter(plugin_name + ".octomap_voxel_size", octomap_voxel_size_);

  se2_space_bounds_->setLow(
    0, parent->get_parameter(plugin_name + ".state_space_boundries.minx").as_double());
  se2_space_bounds_->setHigh(
    0, parent->get_parameter(plugin_name + ".state_space_boundries.maxx").as_double());
  se2_space_bounds_->setLow(
    1, parent->get_parameter(plugin_name + ".state_space_boundries.miny").as_double());
  se2_space_bounds_->setHigh(
    1, parent->get_parameter(plugin_name + ".state_space_boundries.maxy").as_double());
  se2_space_bounds_->setLow(
    2, -0.3);
  se2_space_bounds_->setHigh(
    2, 0.3);


  velocity_space_bounds_->setHigh(
    parent->get_parameter(plugin_name + ".velocity_space_boundries.max_v").as_double());
  velocity_space_bounds_->setLow(
    parent->get_parameter(plugin_name + ".velocity_space_boundries.min_v").as_double());

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
    std::bind(&SE2PlannerControlSpace::octomapCallback, this, std::placeholders::_1));

  se2_space_->as<ompl::base::ReedsSheppStateSpace>()->setBounds(*se2_space_bounds_);
  velocity_space_->as<ompl::base::RealVectorStateSpace>()->setBounds(*velocity_space_bounds_);
  space_ = se2_space_ + velocity_space_;

  if (!is_enabled_) {
    RCLCPP_WARN(
      logger_, "SE2PlannerControlSpace plugin is disabled.");
  } else {
    RCLCPP_INFO(
      logger_, "Initializing SE2PlannerControlSpace plugin, selected planner is; %s",
      planner_name_.c_str());
  }
  RCLCPP_INFO(logger_, "Selected planner is: %s", planner_name_.c_str());
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
  ompl::control::ControlSpacePtr control_space(
    new ompl::control::RealVectorControlSpace(space_, 2));
  ompl::base::RealVectorBounds control_bounds(2);
  control_bounds.setLow(0, -0.0);
  control_bounds.setHigh(0, 0.3);
  control_bounds.setLow(1, -0.1);
  control_bounds.setHigh(1, 0.1);
  control_space->as<ompl::control::RealVectorControlSpace>()->setBounds(control_bounds);
  ompl::control::SpaceInformationPtr si(new ompl::control::SpaceInformation(space_, control_space));

  si->setStatePropagator(propagate);
  si->setStateValidityChecker(
    [this](const ompl::base::State * state)
    {return isStateValid(state);});

  // set the start and goal states
  double start_yaw, goal_yaw, nan;
  botanbot_utilities::getRPYfromMsgQuaternion(start.pose.orientation, nan, nan, start_yaw);
  botanbot_utilities::getRPYfromMsgQuaternion(goal.pose.orientation, nan, nan, goal_yaw);
  ompl::base::ScopedState<> start_ompl(space_), goal_ompl(space_);

  start_ompl[0] = start.pose.position.x;
  start_ompl[1] = start.pose.position.y;
  start_ompl[2] = start_yaw;
  start_ompl[3] = 0.0;

  goal_ompl[0] = goal.pose.position.x;
  goal_ompl[1] = goal.pose.position.y;
  goal_ompl[2] = goal_yaw;
  goal_ompl[3] = 0.0;

  ompl::base::ProblemDefinitionPtr pdef(new ompl::base::ProblemDefinition(si));
  pdef->setStartAndGoalStates(start_ompl, goal_ompl, 0.05);
  ompl::base::OptimizationObjectivePtr length_objective(
    new ompl::base::PathLengthOptimizationObjective(si));
  pdef->setOptimizationObjective(length_objective);

  // create a planner for the defined space
  ompl::base::PlannerPtr planner;
  if (planner_name_ == std::string("RRT")) {
    planner = ompl::base::PlannerPtr(new ompl::control::RRT(si));
  } else if (planner_name_ == std::string("SST")) {
    planner = ompl::base::PlannerPtr(new ompl::control::SST(si));
  } else if (planner_name_ == std::string("EST")) {
    planner = ompl::base::PlannerPtr(new ompl::control::EST(si));
  } else if (planner_name_ == std::string("KPIECE1")) {
    planner = ompl::base::PlannerPtr(new ompl::control::KPIECE1(si));
  } else if (planner_name_ == std::string("PRMstar")) {
    planner = ompl::base::PlannerPtr(new ompl::geometric::PRMstar(si));
  } else {
    RCLCPP_WARN(
      logger_,
      "Selected planner is not Found in available planners, using the default planner: KPIECE1");
    planner = ompl::base::PlannerPtr(new ompl::control::KPIECE1(si));
  }
  planner->setProblemDefinition(pdef);
  planner->setup();

  ompl::base::PlannerStatus solved = planner->solve(planner_timeout_);
  std::vector<geometry_msgs::msg::PoseStamped> plan_poses;

  if (solved) {
    auto path_temp = pdef->getSolutionPath();
    ompl::geometric::PathGeometric & path =
      static_cast<ompl::geometric::PathGeometric &>(*path_temp);
    // Path smoothing using bspline
    ompl::geometric::PathSimplifier * pathBSpline = new ompl::geometric::PathSimplifier(si);
    pathBSpline->smoothBSpline(path, 3);
    path.interpolate(interpolation_parameter_);

    for (std::size_t path_idx = 0; path_idx < path.getStateCount(); path_idx++) {
      const ompl::base::CompoundStateSpace::StateType * compound_state =
        path.getState(path_idx)->as<ompl::base::ReedsSheppStateSpace::StateType>();
      const ompl::base::ReedsSheppStateSpace::StateType * se2state =
        compound_state->as<ompl::base::ReedsSheppStateSpace::StateType>(0);
      fcl::Vec3f translation(se2state->getX(), se2state->getY(), 0.5);
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

  const ompl::base::CompoundStateSpace::StateType * compound_state =
    state->as<ompl::base::CompoundStateSpace::StateType>();

  const ompl::base::ReedsSheppStateSpace::StateType * se2state =
    compound_state->as<ompl::base::ReedsSheppStateSpace::StateType>(0);

  const ompl::base::RealVectorStateSpace::StateType * v_state =
    compound_state->as<ompl::base::RealVectorStateSpace::StateType>(1);

  fcl::Vec3f translation(se2state->getX(), se2state->getY(), 0.5);
  tf2::Quaternion myQuaternion;
  myQuaternion.setRPY(0, 0, se2state->getYaw());

  fcl::Quaternion3f rotation(
    myQuaternion.getX(),
    myQuaternion.getY(),
    myQuaternion.getZ(),
    myQuaternion.getW());

  robot_collision_object_->setTransform(rotation, translation);
  fcl::CollisionRequest requestType(1, false, 1, false);

  fcl::CollisionResult collisionResult;
  fcl::collide(
    robot_collision_object_.get(),
    fcl_octree_collision_object_.get(), requestType, collisionResult);
  return !collisionResult.isCollision();
}

void SE2PlannerControlSpace::propagate(
  const ompl::base::State * start, const ompl::control::Control * control, const double duration,
  ompl::base::State * result)
{
  const ompl::base::CompoundStateSpace::StateType * compound_state =
    start->as<ompl::base::CompoundStateSpace::StateType>();
  const ompl::base::ReedsSheppStateSpace::StateType * se2state =
    compound_state->as<ompl::base::ReedsSheppStateSpace::StateType>(0);
  const ompl::base::RealVectorStateSpace::StateType * v_state =
    compound_state->as<ompl::base::RealVectorStateSpace::StateType>(1);

  const double * ctrl = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;

  double acc = ctrl[0];
  double steer_angle = ctrl[1];
  double velocity = (*v_state)[0];

  double x_n, y_n, theta_n, velocity_n;
  x_n = se2state->getX() + velocity * duration * cos(se2state->getYaw());
  y_n = se2state->getY() + velocity * duration * sin(se2state->getYaw());
  velocity_n = velocity + acc * duration;

  double vehicle_length = 1.32;
  double lengthInv = 1.0 / vehicle_length;
  double omega = velocity * lengthInv * std::tan(steer_angle);
  theta_n = se2state->getYaw() + omega * duration;

  result->as<ompl::base::ReedsSheppStateSpace::StateType>()->setXY(x_n, y_n);
  result->as<ompl::base::ReedsSheppStateSpace::StateType>()->setYaw(theta_n);
  v_state->values[0] = velocity_n;
}

void SE2PlannerControlSpace::octomapCallback(
  const octomap_msgs::msg::Octomap::ConstSharedPtr msg)
{
  const std::lock_guard<std::mutex> lock(octomap_mutex_);
  if (!is_octomap_ready_) {
    is_octomap_ready_ = true;
    octomap_msg_ = msg;
  }
}

}  // namespace botanbot_planning

PLUGINLIB_EXPORT_CLASS(botanbot_planning::SE2PlannerControlSpace, botanbot_planning::PlannerCore)
