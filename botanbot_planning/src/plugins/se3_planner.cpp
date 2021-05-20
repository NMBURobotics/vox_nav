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

#include "botanbot_planning/plugins/se3_planner.hpp"
#include <pluginlib/class_list_macros.hpp>

#include <string>
#include <memory>
#include <vector>

namespace botanbot_planning
{
class ChildOptimizationObjective : public ompl::base::OptimizationObjective
{
public:
  ChildOptimizationObjective(const ompl::base::SpaceInformationPtr & si)
  : ompl::base::OptimizationObjective(si)
  {
  }
  ompl::base::Cost motionCost(
    const ompl::base::State * s1,
    const ompl::base::State * s2) const override
  {
    ompl::base::Cost C1(stateCost(s1));
    ompl::base::Cost C2(stateCost(s2));
    return ompl::base::Cost(C1.value() + C2.value());
  }

  ompl::base::Cost stateCost(const ompl::base::State * s) const
  {
    const ompl::base::SE3StateSpace::StateType * Cstate3D =
      s->as<ompl::base::SE3StateSpace::StateType>();
    const ompl::base::RealVectorStateSpace::StateType * state3D =
      Cstate3D->as<ompl::base::RealVectorStateSpace::StateType>(0);
    double z = state3D->values[2];
    double y = state3D->values[1];
    return ompl::base::Cost(1);
  }
};

SE3Planner::SE3Planner()
{
}

SE3Planner::~SE3Planner()
{
}

void SE3Planner::initialize(
  rclcpp::Node * parent,
  const std::string & plugin_name)
{
  state_space_bounds_ = std::make_shared<ompl::base::RealVectorBounds>(3);
  octomap_msg_ = std::make_shared<octomap_msgs::msg::Octomap>();
  is_octomap_ready_ = false;

  parent->declare_parameter(plugin_name + ".enabled", true);
  parent->declare_parameter(plugin_name + ".planner_name", "PRMStar");
  parent->declare_parameter(plugin_name + ".planner_timeout", 5.0);
  parent->declare_parameter(plugin_name + ".interpolation_parameter", 50);
  parent->declare_parameter(plugin_name + ".octomap_topic", "octomap");
  parent->declare_parameter(plugin_name + ".octomap_voxel_size", 0.2);
  parent->declare_parameter(plugin_name + ".state_space_boundries.minx", -10.0);
  parent->declare_parameter(plugin_name + ".state_space_boundries.maxx", 10.0);
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
  parent->get_parameter(plugin_name + ".octomap_topic", octomap_topic_);
  parent->get_parameter(plugin_name + ".octomap_voxel_size", octomap_voxel_size_);

  state_space_bounds_->setLow(
    0, parent->get_parameter(plugin_name + ".state_space_boundries.minx").as_double());
  state_space_bounds_->setHigh(
    0, parent->get_parameter(plugin_name + ".state_space_boundries.maxx").as_double());
  state_space_bounds_->setLow(
    1, parent->get_parameter(plugin_name + ".state_space_boundries.miny").as_double());
  state_space_bounds_->setHigh(
    1, parent->get_parameter(plugin_name + ".state_space_boundries.maxy").as_double());
  state_space_bounds_->setLow(
    2, parent->get_parameter(plugin_name + ".state_space_boundries.minz").as_double());
  state_space_bounds_->setHigh(
    2, parent->get_parameter(plugin_name + ".state_space_boundries.maxz").as_double());

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
    std::bind(&SE3Planner::octomapCallback, this, std::placeholders::_1));

  state_space_ = std::make_shared<ompl::base::SE3StateSpace>();
  state_space_->as<ompl::base::SE3StateSpace>()->setBounds(*state_space_bounds_);
  state_space_information_ = std::make_shared<ompl::base::SpaceInformation>(state_space_);
  state_space_information_->setStateValidityChecker(
    std::bind(&SE3Planner::isStateValid, this, std::placeholders::_1));

  if (!is_enabled_) {
    RCLCPP_WARN(
      logger_, "SE2PlannerControlSpace plugin is disabled.");
  }

  RCLCPP_INFO(logger_, "Selected planner is: %s", planner_name_.c_str());
}

std::vector<geometry_msgs::msg::PoseStamped> SE3Planner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  if (!is_enabled_) {
    RCLCPP_WARN(
      logger_,
      "SE3Planner plugin is disabled. Not performing anything returning an empty path"
    );
    return std::vector<geometry_msgs::msg::PoseStamped>();
  }

  // set the start and goal states
  double start_yaw, goal_yaw, nan;
  botanbot_utilities::getRPYfromMsgQuaternion(start.pose.orientation, nan, nan, start_yaw);
  botanbot_utilities::getRPYfromMsgQuaternion(goal.pose.orientation, nan, nan, goal_yaw);

  ompl::base::ScopedState<ompl::base::SE3StateSpace> se3_start(state_space_),
  se3_goal(state_space_);
  se3_start->setXYZ(start.pose.position.x, start.pose.position.y, start.pose.position.z + 3.2);
  se3_start->as<ompl::base::SO3StateSpace::StateType>(1)->setAxisAngle(
    0,
    0,
    1,
    start_yaw);

  se3_goal->setXYZ(goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
  se3_goal->as<ompl::base::SO3StateSpace::StateType>(1)->setAxisAngle(
    0,
    0,
    1,
    goal_yaw);

  // create a problem instance
  ompl::base::ProblemDefinitionPtr
    pdef(new ompl::base::ProblemDefinition(state_space_information_));

  // set the start and goal states
  pdef->setStartAndGoalStates(se3_start, se3_goal);
  //pdef->setOptimizationObjective(getOptObjective(state_space_information_));
  ompl::base::OptimizationObjectivePtr length_objective(
    new ompl::base::PathLengthOptimizationObjective(state_space_information_));
  pdef->setOptimizationObjective(length_objective);

  // create a planner for the defined space
  ompl::base::PlannerPtr planner;
  initializeSelectedPlanner(planner, planner_name_, state_space_information_);

  // set the problem we are trying to solve for the planner
  planner->setProblemDefinition(pdef);

  // perform setup steps for the planner
  planner->setup();

  // print the settings for this space
  state_space_information_->printSettings(std::cout);

  // print the problem settings
  pdef->print(std::cout);

  // attempt to solve the problem within one second of planning time
  ompl::base::PlannerStatus solved = planner->solve(planner_timeout_);
  std::vector<geometry_msgs::msg::PoseStamped> plan_poses;

  if (solved) {
    ompl::base::PathPtr path = pdef->getSolutionPath();

    ompl::geometric::PathGeometric * pth =
      pdef->getSolutionPath()->as<ompl::geometric::PathGeometric>();
    pth->interpolate(interpolation_parameter_);

    // Path smoothing using bspline
    ompl::geometric::PathSimplifier * pathBSpline = new ompl::geometric::PathSimplifier(
      state_space_information_);
    ompl::geometric::PathGeometric path_smooth(
      dynamic_cast<const ompl::geometric::PathGeometric &>(*pdef->getSolutionPath()));
    pathBSpline->smoothBSpline(path_smooth, 3);

    for (std::size_t path_idx = 0; path_idx < path_smooth.getStateCount(); path_idx++) {
      const ompl::base::SE3StateSpace::StateType * se3state =
        path_smooth.getState(path_idx)->as<ompl::base::SE3StateSpace::StateType>();

      // extract the first component of the state and cast it to what we expect
      const ompl::base::RealVectorStateSpace::StateType * pos =
        se3state->as<ompl::base::RealVectorStateSpace::StateType>(0);

      // extract the second component of the state and cast it to what we expect
      const ompl::base::SO3StateSpace::StateType * rot =
        se3state->as<ompl::base::SO3StateSpace::StateType>(1);

      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = start.header.frame_id;
      pose.header.stamp = rclcpp::Clock().now();
      pose.pose.position.x = pos->values[0];
      pose.pose.position.y = pos->values[1];
      pose.pose.position.z = pos->values[2];
      pose.pose.orientation.x = rot->x;
      pose.pose.orientation.y = rot->y;
      pose.pose.orientation.z = rot->z;
      pose.pose.orientation.w = rot->w;
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

bool SE3Planner::isStateValid(const ompl::base::State * state)
{
  if (is_octomap_ready_) {
    // cast the abstract state type to the type we expect
    const ompl::base::SE3StateSpace::StateType * se3state =
      state->as<ompl::base::SE3StateSpace::StateType>();
    // extract the first component of the state and cast it to what we expect
    const ompl::base::RealVectorStateSpace::StateType * pos =
      se3state->as<ompl::base::RealVectorStateSpace::StateType>(0);
    // extract the second component of the state and cast it to what we expect
    const ompl::base::SO3StateSpace::StateType * rot =
      se3state->as<ompl::base::SO3StateSpace::StateType>(1);
    // check validity of state Fdefined by pos & rot
    fcl::Vec3f translation(pos->values[0], pos->values[1], pos->values[2]);
    fcl::Quaternion3f rotation(rot->w, rot->x, rot->y, rot->z);
    robot_collision_object_->setTransform(rotation, translation);
    fcl::CollisionRequest requestType(1, false, 1, false);
    fcl::CollisionResult collisionResult;
    fcl::collide(
      robot_collision_object_.get(),
      fcl_octree_collision_object_.get(), requestType, collisionResult);
    return !collisionResult.isCollision();
  } else {
    RCLCPP_ERROR(
      logger_,
      "The Octomap has not been recieved correctly, Collision check "
      "cannot be processed without a valid Octomap!");
    return false;
  }
}

ompl::base::OptimizationObjectivePtr SE3Planner::getOptObjective(
  const ompl::base::SpaceInformationPtr & si)
{
  ompl::base::OptimizationObjectivePtr obj = std::make_shared<ChildOptimizationObjective>(si);
  obj->setCostToGoHeuristic(&ompl::base::goalRegionCostToGo);
  return obj;
}

void SE3Planner::octomapCallback(
  const octomap_msgs::msg::Octomap::ConstSharedPtr msg)
{
  const std::lock_guard<std::mutex> lock(octomap_mutex_);
  if (!is_octomap_ready_) {
    octomap_msg_ = msg;
    RCLCPP_INFO(logger_, "Octomap has been recieved!");

    std::shared_ptr<octomap::ColorOcTree> color_octomap_octree =
      std::make_shared<octomap::ColorOcTree>(0.2);
    octomap_msgs::readTree<octomap::ColorOcTree>(color_octomap_octree.get(), *octomap_msg_);
    std::shared_ptr<octomap::OcTree> octomap_octree =
      std::make_shared<octomap::OcTree>(0.2);

    auto m_treeDepth = color_octomap_octree->getTreeDepth();
    // now, traverse all leafs in the tree:
    for (auto it = color_octomap_octree->begin(m_treeDepth),
      end = color_octomap_octree->end(); it != end; ++it)
    {
      if (color_octomap_octree->isNodeOccupied(*it)) {
        double x = it.getX();
        double y = it.getY();
        double z = it.getZ();
        octomap::point3d crr_point(x, y, z);
        auto crr_point_node_key = color_octomap_octree->coordToKey(crr_point);
        octomap_octree->setNodeValue(crr_point_node_key, 1.0, false);
      }
    }

    fcl_octree_ = std::make_shared<fcl::OcTree>(octomap_octree);
    fcl_octree_collision_object_ = std::make_shared<fcl::CollisionObject>(
      std::shared_ptr<fcl::CollisionGeometry>(fcl_octree_));

    RCLCPP_INFO(
      logger_,
      "Recieved a valid Octomap with %d nodes, A FCL collision tree will be created from this "
      "octomap for state validity (aka collision check)", octomap_octree->size());

    is_octomap_ready_ = true;
  }
}

void SE3Planner::initializeSelectedPlanner(
  ompl::base::PlannerPtr & planner,
  const std::string & selected_planner_name,
  const ompl::base::SpaceInformationPtr & si)
{
  if (selected_planner_name == std::string("PRMStar")) {
    planner = ompl::base::PlannerPtr(new ompl::geometric::PRMstar(si));
  } else if (selected_planner_name == std::string("RRTstar")) {
    planner = ompl::base::PlannerPtr(new ompl::geometric::RRTstar(si) );
  } else if (selected_planner_name == std::string("RRTConnect")) {
    planner = ompl::base::PlannerPtr(new ompl::geometric::RRTConnect(si) );
  } else if (selected_planner_name == std::string("KPIECE1")) {
    planner = ompl::base::PlannerPtr(new ompl::geometric::KPIECE1(si) );
  } else {
    RCLCPP_WARN(
      logger_,
      "Selected planner is not Found in available planners, using the default planner: KPIECE1");
    planner = ompl::base::PlannerPtr(new ompl::geometric::KPIECE1(si));
  }
}

}  // namespace botanbot_planning

PLUGINLIB_EXPORT_CLASS(botanbot_planning::SE3Planner, botanbot_planning::PlannerCore)
