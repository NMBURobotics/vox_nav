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

#include "vox_nav_planning/plugins/se3_planner.hpp"
#include <pluginlib/class_list_macros.hpp>

#include <string>
#include <memory>
#include <vector>

namespace vox_nav_planning
{

SE3Planner::SE3Planner()
{
}

SE3Planner::~SE3Planner()
{
}

void SE3Planner::initialize(rclcpp::Node* parent, const std::string& plugin_name)
{
  is_map_ready_ = false;
  se3_bounds_ = std::make_shared<ompl::base::RealVectorBounds>(3);

  parent->declare_parameter(plugin_name + ".state_space_boundries.minx", -50.0);
  parent->declare_parameter(plugin_name + ".state_space_boundries.maxx", 50.0);
  parent->declare_parameter(plugin_name + ".state_space_boundries.miny", -10.0);
  parent->declare_parameter(plugin_name + ".state_space_boundries.maxy", 10.0);
  parent->declare_parameter(plugin_name + ".state_space_boundries.minz", -10.0);
  parent->declare_parameter(plugin_name + ".state_space_boundries.maxz", 10.0);

  parent->get_parameter("planner_name", planner_name_);
  parent->get_parameter("planner_timeout", planner_timeout_);
  parent->get_parameter("interpolation_parameter", interpolation_parameter_);
  parent->get_parameter("octomap_voxel_size", octomap_voxel_size_);

  se3_bounds_->setLow(0, parent->get_parameter(plugin_name + ".state_space_boundries.minx").as_double());
  se3_bounds_->setHigh(0, parent->get_parameter(plugin_name + ".state_space_boundries.maxx").as_double());
  se3_bounds_->setLow(1, parent->get_parameter(plugin_name + ".state_space_boundries.miny").as_double());
  se3_bounds_->setHigh(1, parent->get_parameter(plugin_name + ".state_space_boundries.maxy").as_double());
  se3_bounds_->setLow(2, parent->get_parameter(plugin_name + ".state_space_boundries.minz").as_double());
  se3_bounds_->setHigh(2, parent->get_parameter(plugin_name + ".state_space_boundries.maxz").as_double());

  state_space_ = std::make_shared<ompl::base::SE3StateSpace>();

  state_space_->as<ompl::base::SE2StateSpace>()->setBounds(*se3_bounds_);
  simple_setup_ = std::make_shared<ompl::geometric::SimpleSetup>(state_space_);

  typedef std::shared_ptr<fcl::CollisionGeometryf> CollisionGeometryPtr_t;
  CollisionGeometryPtr_t robot_body_box(new fcl::Box<float>(parent->get_parameter("robot_body_dimens.x").as_double(),
                                                            parent->get_parameter("robot_body_dimens.y").as_double(),
                                                            parent->get_parameter("robot_body_dimens.z").as_double()));

  fcl::CollisionObjectf robot_body_box_object(robot_body_box, fcl::Transform3f());
  robot_collision_object_ = std::make_shared<fcl::CollisionObjectf>(robot_body_box_object);
  original_octomap_octree_ = std::make_shared<octomap::OcTree>(octomap_voxel_size_);

  // service hooks for robot localization fromll service
  get_map_client_node_ = std::make_shared<rclcpp::Node>("get_traversability_map_client_node");

  get_traversability_map_client_ =
      get_map_client_node_->create_client<vox_nav_msgs::srv::GetTraversabilityMap>("get_traversability_map");

  RCLCPP_INFO(logger_, "Selected planner is: %s", planner_name_.c_str());

  setupMap();
}

std::vector<geometry_msgs::msg::PoseStamped> SE3Planner::createPlan(const geometry_msgs::msg::PoseStamped& start,
                                                                    const geometry_msgs::msg::PoseStamped& goal)
{
  if (!is_map_ready_)
  {
    RCLCPP_WARN(logger_, "A valid map has not been receievd yet, Try later again.");
    return std::vector<geometry_msgs::msg::PoseStamped>();
  }

  ompl::base::ScopedState<ompl::base::SE3StateSpace> se3_start(state_space_), se3_goal(state_space_);

  start_ = start;
  goal_ = goal;

  se3_start->setXYZ(start.pose.position.x, start.pose.position.y, start.pose.position.z);
  se3_goal->setXYZ(goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);

  se3_start->as<ompl::base::SO3StateSpace::StateType>(1)->setIdentity();
  se3_goal->as<ompl::base::SO3StateSpace::StateType>(1)->setIdentity();

  simple_setup_->setStartAndGoalStates(se3_start, se3_goal, 0.2);

  // objective is to minimize the planned path
  ompl::base::OptimizationObjectivePtr objective(
      new ompl::base::PathLengthOptimizationObjective(simple_setup_->getSpaceInformation()));

  simple_setup_->setOptimizationObjective(objective);

  // create a planner for the defined space
  ompl::base::PlannerPtr planner;
  vox_nav_utilities::initializeSelectedPlanner(planner, planner_name_, simple_setup_->getSpaceInformation(), logger_);

  simple_setup_->setPlanner(planner);
  simple_setup_->setup();
  // print the settings for this space
  simple_setup_->print(std::cout);

  // attempt to solve the problem within one second of planning time
  ompl::base::PlannerStatus solved = simple_setup_->solve(planner_timeout_);
  std::vector<geometry_msgs::msg::PoseStamped> plan_poses;

  if (solved)
  {
    ompl::geometric::PathGeometric solution_path = simple_setup_->getSolutionPath();
    // Path smoothing using bspline
    ompl::geometric::PathSimplifier* path_simlifier =
        new ompl::geometric::PathSimplifier(simple_setup_->getSpaceInformation());

    path_simlifier->smoothBSpline(solution_path, 3);
    solution_path.interpolate(interpolation_parameter_);

    for (std::size_t path_idx = 0; path_idx < solution_path.getStateCount(); path_idx++)
    {
      // cast the abstract state type to the type we expect
      const ompl::base::SE3StateSpace::StateType* se3_state =
          solution_path.getState(path_idx)->as<ompl::base::SE3StateSpace::StateType>();

      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = start.header.frame_id;
      pose.header.stamp = rclcpp::Clock().now();
      pose.pose.position.x = se3_state->getX();
      pose.pose.position.y = se3_state->getY();
      pose.pose.position.z = se3_state->getZ();
      pose.pose.orientation.w = 1.0;
      plan_poses.push_back(pose);
    }
    RCLCPP_INFO(logger_, "Found A plan with %i poses", plan_poses.size());
  }
  else
  {
    RCLCPP_WARN(logger_, "No solution for requested path planning !");
  }
  return plan_poses;
}

bool SE3Planner::isStateValid(const ompl::base::State* state)
{
  // cast the abstract state type to the type we expect
  const ompl::base::SE3StateSpace::StateType* se3_state = state->as<ompl::base::SE3StateSpace::StateType>();
  // check validity of state Fdefined by pos & rot
  fcl::Vector3f translation(se3_state->getX(), se3_state->getY(), se3_state->getZ());
  tf2::Quaternion myQuaternion;
  myQuaternion.setRPY(0, 0, 0);
  fcl::Quaternionf rotation(myQuaternion.getX(), myQuaternion.getY(), myQuaternion.getZ(), myQuaternion.getW());
  robot_collision_object_->setTransform(rotation, translation);
  fcl::CollisionRequestf requestType(1, false, 1, false);
  fcl::CollisionResultf collisionResult;
  fcl::collide<float>(robot_collision_object_.get(), original_octomap_collision_object_.get(), requestType,
                      collisionResult);
  return !collisionResult.isCollision();
}

void SE3Planner::setupMap()
{
  const std::lock_guard<std::mutex> lock(octomap_mutex_);

  while (!is_map_ready_ && rclcpp::ok())
  {
    auto request = std::make_shared<vox_nav_msgs::srv::GetTraversabilityMap::Request>();

    while (!get_traversability_map_client_->wait_for_service(std::chrono::seconds(1)))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(logger_, "Interrupted while waiting for the get_traversability_map service. Exiting");
        return;
      }
      RCLCPP_INFO(logger_, "get_traversability_map service not available, waiting and trying again");
    }

    auto result_future = get_traversability_map_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(get_map_client_node_, result_future) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(logger_, "/get_traversability_map service call failed");
    }
    auto response = result_future.get();

    if (response->is_valid)
    {
      is_map_ready_ = true;
    }
    else
    {
      std::this_thread::sleep_for(std::chrono::seconds(1));
      RCLCPP_INFO(logger_, "Waiting for GetTraversabilityMap service to provide correct maps.");
      continue;
    }

    auto original_octomap_octree =
        dynamic_cast<octomap::OcTree*>(octomap_msgs::fullMsgToMap(response->original_octomap));
    original_octomap_octree_ = std::make_shared<octomap::OcTree>(*original_octomap_octree);

    delete original_octomap_octree;

    auto original_octomap_fcl_octree = std::make_shared<fcl::OcTreef>(original_octomap_octree_);
    original_octomap_collision_object_ =
        std::make_shared<fcl::CollisionObjectf>(std::shared_ptr<fcl::CollisionGeometryf>(original_octomap_fcl_octree));

    RCLCPP_INFO(logger_,
                "Recieved a valid Octomap with %d nodes, A FCL collision tree will be created from this "
                "octomap for state validity (aka collision check)",
                original_octomap_octree_->size());

    simple_setup_->setStateValidityChecker(std::bind(&SE3Planner::isStateValid, this, std::placeholders::_1));
  }
}

std::vector<geometry_msgs::msg::PoseStamped> SE3Planner::getOverlayedStartandGoal()
{
  RCLCPP_WARN(logger_, "getOverlayedStartandGoal is not implemented for SE3Planner yet");
  return std::vector<geometry_msgs::msg::PoseStamped>();
}
}  // namespace vox_nav_planning

PLUGINLIB_EXPORT_CLASS(vox_nav_planning::SE3Planner, vox_nav_planning::PlannerCore)
