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

#include "vox_nav_planning/plugins/elevation_planner.hpp"
#include <pluginlib/class_list_macros.hpp>

#include <string>
#include <memory>
#include <vector>
#include <random>

namespace vox_nav_planning
{

ElevationPlanner::ElevationPlanner()
{
}

ElevationPlanner::~ElevationPlanner()
{
}

void ElevationPlanner::initialize(
  rclcpp::Node * parent,
  const std::string & plugin_name)
{
  is_map_ready_ = false;
  state_space_bounds_ = std::make_shared<ompl::base::RealVectorBounds>(2);
  elevated_surfel_cloud_ = pcl::PointCloud<pcl::PointSurfel>::Ptr(
    new pcl::PointCloud<pcl::PointSurfel>);

  // declare only planner specific parameters here
  // common parameters are declared in server
  parent->declare_parameter(plugin_name + ".state_space_boundries.minx", -10.0);
  parent->declare_parameter(plugin_name + ".state_space_boundries.maxx", 10.0);
  parent->declare_parameter(plugin_name + ".state_space_boundries.miny", -10.0);
  parent->declare_parameter(plugin_name + ".state_space_boundries.maxy", 10.0);
  parent->declare_parameter(plugin_name + ".state_space_boundries.minz", -10.0);
  parent->declare_parameter(plugin_name + ".state_space_boundries.maxz", 10.0);

  parent->get_parameter("enabled", is_enabled_);
  parent->get_parameter("planner_name", planner_name_);
  parent->get_parameter("planner_timeout", planner_timeout_);
  parent->get_parameter("interpolation_parameter", interpolation_parameter_);
  parent->get_parameter("octomap_voxel_size", octomap_voxel_size_);

  state_space_bounds_->setLow(
    0, parent->get_parameter(plugin_name + ".state_space_boundries.minx").as_double());
  state_space_bounds_->setHigh(
    0, parent->get_parameter(plugin_name + ".state_space_boundries.maxx").as_double());
  state_space_bounds_->setLow(
    1, parent->get_parameter(plugin_name + ".state_space_boundries.miny").as_double());
  state_space_bounds_->setHigh(
    1, parent->get_parameter(plugin_name + ".state_space_boundries.maxy").as_double());

  auto z_bound = std::make_shared<ompl::base::RealVectorBounds>(1);

  z_bound->setLow(
    0, parent->get_parameter(plugin_name + ".state_space_boundries.minz").as_double());
  z_bound->setHigh(
    0, parent->get_parameter(plugin_name + ".state_space_boundries.maxz").as_double());

  typedef std::shared_ptr<fcl::CollisionGeometry> CollisionGeometryPtr_t;
  CollisionGeometryPtr_t robot_body_box(new fcl::Box(
      parent->get_parameter("robot_body_dimens.x").as_double(),
      parent->get_parameter("robot_body_dimens.y").as_double(),
      parent->get_parameter("robot_body_dimens.z").as_double()));

  fcl::CollisionObject robot_body_box_object(robot_body_box, fcl::Transform3f());
  robot_collision_object_ = std::make_shared<fcl::CollisionObject>(robot_body_box_object);

  ompl::base::StateSpacePtr space(new ompl::base::ElevationStateSpace());
  space->as<ompl::base::ElevationStateSpace>()->setBounds(*state_space_bounds_, *z_bound);

  state_space_ = space;
  simple_setup_ = std::make_shared<ompl::geometric::SimpleSetup>(state_space_);

  elevated_surfel_octomap_octree_ = std::make_shared<octomap::OcTree>(octomap_voxel_size_);
  original_octomap_octree_ = std::make_shared<octomap::OcTree>(octomap_voxel_size_);

  // service hooks for robot localization fromll service
  get_maps_and_surfels_client_node_ = std::make_shared
    <rclcpp::Node>("get_maps_and_surfels_client_node");

  get_maps_and_surfels_client_ =
    get_maps_and_surfels_client_node_->create_client
    <vox_nav_msgs::srv::GetMapsAndSurfels>(
    "get_maps_and_surfels");

  if (!is_enabled_) {
    RCLCPP_WARN(logger_, "ElevationPlanner plugin is disabled.");
  }
  RCLCPP_INFO(logger_, "Selected planner is: %s", planner_name_.c_str());

  setupMap();
}

std::vector<geometry_msgs::msg::PoseStamped> ElevationPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  if (!is_enabled_) {
    RCLCPP_WARN(
      logger_,
      "ElevationPlanner plugin is disabled. Not performing anything returning an empty path"
    );
    return std::vector<geometry_msgs::msg::PoseStamped>();
  }

  if (!is_map_ready_) {
    RCLCPP_WARN(
      logger_, "A valid Octomap has not been receievd yet, Try later again."
    );
    return std::vector<geometry_msgs::msg::PoseStamped>();
  }

  // set the start and goal states
  double start_yaw, goal_yaw, nan;
  vox_nav_utilities::getRPYfromMsgQuaternion(start.pose.orientation, nan, nan, start_yaw);
  vox_nav_utilities::getRPYfromMsgQuaternion(goal.pose.orientation, nan, nan, goal_yaw);

  ompl::base::ScopedState<ompl::base::ElevationStateSpace>
  se3_start(state_space_),
  se3_goal(state_space_);

  pcl::PointSurfel start_nearest_surfel, goal_nearest_surfel;
  pcl::PointSurfel start_actual, goal_acual;

  start_actual.x = start.pose.position.x;
  start_actual.y = start.pose.position.y;
  start_actual.z = start.pose.position.z;

  goal_acual.x = goal.pose.position.x;
  goal_acual.y = goal.pose.position.y;
  goal_acual.z = goal.pose.position.z;

  vox_nav_utilities::getNearstPoint<
    pcl::PointSurfel,
    pcl::PointCloud<pcl::PointSurfel>::Ptr>(
    start_nearest_surfel, start_actual,
    elevated_surfel_cloud_);

  vox_nav_utilities::getNearstPoint<
    pcl::PointSurfel,
    pcl::PointCloud<pcl::PointSurfel>::Ptr>(
    goal_nearest_surfel, goal_acual,
    elevated_surfel_cloud_);

  nearest_elevated_surfel_to_start_.pose.position.x = start_nearest_surfel.x;
  nearest_elevated_surfel_to_start_.pose.position.y = start_nearest_surfel.y;
  nearest_elevated_surfel_to_start_.pose.position.z = start_nearest_surfel.z;
  nearest_elevated_surfel_to_start_.pose.orientation = vox_nav_utilities::getMsgQuaternionfromRPY(
    start_nearest_surfel.normal_x,
    start_nearest_surfel.normal_y,
    start_nearest_surfel.normal_z
  );

  nearest_elevated_surfel_to_goal_.pose.position.x = goal_nearest_surfel.x;
  nearest_elevated_surfel_to_goal_.pose.position.y = goal_nearest_surfel.y;
  nearest_elevated_surfel_to_goal_.pose.position.z = goal_nearest_surfel.z;
  nearest_elevated_surfel_to_goal_.pose.orientation = vox_nav_utilities::getMsgQuaternionfromRPY(
    goal_nearest_surfel.normal_x,
    goal_nearest_surfel.normal_y,
    goal_nearest_surfel.normal_z);

  se3_start->setSE2(
    nearest_elevated_surfel_to_start_.pose.position.x,
    nearest_elevated_surfel_to_start_.pose.position.y, 0);
  se3_start->setZ(
    nearest_elevated_surfel_to_start_.pose.position.z);

  se3_goal->setSE2(
    nearest_elevated_surfel_to_goal_.pose.position.x,
    nearest_elevated_surfel_to_goal_.pose.position.y, 0);
  se3_goal->setZ(
    nearest_elevated_surfel_to_goal_.pose.position.z);


  simple_setup_->setStartAndGoalStates(se3_start, se3_goal);

  RCLCPP_INFO(logger_, "%.2f  %.2f ", se3_goal->getSE2()->getX(), se3_goal->getSE2()->getY());
  RCLCPP_INFO(logger_, "%.2f  %.2f ", se3_start->getSE2()->getX(), se3_start->getSE2()->getY());


  // create a planner for the defined space
  ompl::base::PlannerPtr planner;
  vox_nav_utilities::initializeSelectedPlanner(
    planner,
    planner_name_,
    simple_setup_->getSpaceInformation(),
    logger_);

  simple_setup_->setPlanner(planner);
  simple_setup_->setup();
  // print the settings for this space
  simple_setup_->print(std::cout);

  simple_setup_->getSpaceInformation()->setValidStateSamplerAllocator(
    std::bind(
      &ElevationPlanner::
      allocValidStateSampler, this, std::placeholders::_1));

  // attempt to solve the problem within one second of planning time
  ompl::base::PlannerStatus solved = simple_setup_->solve(planner_timeout_);
  std::vector<geometry_msgs::msg::PoseStamped> plan_poses;

  if (solved) {
    ompl::geometric::PathGeometric solution_path = simple_setup_->getSolutionPath();
    // Path smoothing using bspline
    ompl::geometric::PathSimplifier * path_simlifier =
      new ompl::geometric::PathSimplifier(simple_setup_->getSpaceInformation());

    //path_simlifier->smoothBSpline(solution_path, 4, 0.2);
    solution_path.interpolate(interpolation_parameter_);

    for (std::size_t path_idx = 0; path_idx < solution_path.getStateCount(); path_idx++) {
      const ompl::base::SE3StateSpace::StateType * se3state =
        solution_path.getState(path_idx)->as<ompl::base::SE3StateSpace::StateType>();
      // extract the second component of the state and cast it to what we expect
      const ompl::base::SO3StateSpace::StateType * rot =
        se3state->as<ompl::base::SO3StateSpace::StateType>(1);
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = start.header.frame_id;
      pose.header.stamp = rclcpp::Clock().now();
      pose.pose.position.x = se3state->getX();
      pose.pose.position.y = se3state->getY();
      pose.pose.position.z = se3state->getZ();
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

  simple_setup_->clear();
  return plan_poses;
}

bool ElevationPlanner::isStateValid(const ompl::base::State * state)
{

  const auto * cstate = state->as<ompl::base::ElevationStateSpace::StateType>();


  // cast the abstract state type to the type we expect
  const auto * dubins = cstate->as<ompl::base::DubinsStateSpace::StateType>(0);
  // extract the second component of the state and cast it to what we expect
  const auto * z = cstate->as<ompl::base::RealVectorStateSpace::StateType>(1);

  fcl::CollisionRequest requestType(1, false, 1, false);

  // check validity of state Fdefined by pos & rot
  fcl::Vec3f translation(dubins->getX(), dubins->getY(), z->values[0]);
  fcl::Quaternion3f rotation(1, 0, 0, 0);
  robot_collision_object_->setTransform(rotation, translation);

  fcl::CollisionResult collisionWithNodesResult, collisionWitFullMapResult;

  fcl::collide(
    robot_collision_object_.get(),
    elevated_surfels_collision_object_.get(), requestType, collisionWithNodesResult);

  fcl::collide(
    robot_collision_object_.get(),
    original_octomap_collision_object_.get(), requestType, collisionWitFullMapResult);

  return collisionWithNodesResult.isCollision() && !collisionWitFullMapResult.isCollision();

}

void ElevationPlanner::setupMap()
{
  const std::lock_guard<std::mutex> lock(octomap_mutex_);

  while (!is_map_ready_ && rclcpp::ok()) {

    auto request = std::make_shared<vox_nav_msgs::srv::GetMapsAndSurfels::Request>();

    while (!get_maps_and_surfels_client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
          logger_,
          "Interrupted while waiting for the get_maps_and_surfels service. Exiting");
        return;
      }
      RCLCPP_INFO(
        logger_,
        "get_maps_and_surfels service not available, waiting and trying again");
    }

    auto result_future = get_maps_and_surfels_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(
        get_maps_and_surfels_client_node_,
        result_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(logger_, "/get_maps_and_surfels service call failed");
    }
    auto response = result_future.get();

    if (response->is_valid) {
      is_map_ready_ = true;
    } else {
      std::this_thread::sleep_for(std::chrono::seconds(1));
      RCLCPP_INFO(
        logger_, "Waiting for GetMapsAndSurfels service to provide correct maps.");
      continue;
    }

    elevated_surfel_poses_msg_ = std::make_shared<geometry_msgs::msg::PoseArray>(
      response->elevated_surfel_poses);

    auto original_octomap_octree =
      dynamic_cast<octomap::OcTree *>(octomap_msgs::fullMsgToMap(response->original_octomap));
    original_octomap_octree_ = std::make_shared<octomap::OcTree>(*original_octomap_octree);

    auto elevated_surfel_octomap_octree =
      dynamic_cast<octomap::OcTree *>(octomap_msgs::fullMsgToMap(
        response->elevated_surfel_octomap));
    elevated_surfel_octomap_octree_ = std::make_shared<octomap::OcTree>(
      *elevated_surfel_octomap_octree);

    delete original_octomap_octree;
    delete elevated_surfel_octomap_octree;

    auto elevated_surfels_fcl_octree =
      std::make_shared<fcl::OcTree>(elevated_surfel_octomap_octree_);
    elevated_surfels_collision_object_ = std::make_shared<fcl::CollisionObject>(
      std::shared_ptr<fcl::CollisionGeometry>(elevated_surfels_fcl_octree));

    auto original_octomap_fcl_octree = std::make_shared<fcl::OcTree>(original_octomap_octree_);
    original_octomap_collision_object_ = std::make_shared<fcl::CollisionObject>(
      std::shared_ptr<fcl::CollisionGeometry>(original_octomap_fcl_octree));

    RCLCPP_INFO(
      logger_,
      "Recieved a valid Octomap with %d nodes, A FCL collision tree will be created from this "
      "octomap for state validity (aka collision check)", original_octomap_octree_->size());

    RCLCPP_INFO(
      logger_,
      "Recieved a valid Octomap which represents Elevated surfels with %d nodes,"
      " A FCL collision tree will be created from this "
      "octomap for state validity (aka collision check)",
      elevated_surfel_octomap_octree_->size());

    simple_setup_->setOptimizationObjective(getOptimizationObjective());
    simple_setup_->setStateValidityChecker(
      std::bind(&ElevationPlanner::isStateValid, this, std::placeholders::_1));

    for (auto && i : elevated_surfel_poses_msg_->poses) {
      pcl::PointSurfel surfel;
      surfel.x = i.position.x;
      surfel.y = i.position.y;
      surfel.z = i.position.z;
      double r, p, y;
      vox_nav_utilities::getRPYfromMsgQuaternion(i.orientation, r, p, y);
      surfel.normal_x = r;
      surfel.normal_y = p;
      surfel.normal_z = y;
      elevated_surfel_cloud_->points.push_back(surfel);
    }
  }
}

ompl::base::ValidStateSamplerPtr ElevationPlanner::allocValidStateSampler(
  const ompl::base::SpaceInformation * si)
{
  octocell_valid_state_sampler_ = std::make_shared<OctoCellValidStateSampler>(
    simple_setup_->getSpaceInformation(),
    nearest_elevated_surfel_to_start_,
    nearest_elevated_surfel_to_goal_,
    robot_collision_object_,
    original_octomap_collision_object_,
    elevated_surfel_poses_msg_);
  return octocell_valid_state_sampler_;
}

ompl::base::OptimizationObjectivePtr ElevationPlanner::getOptimizationObjective()
{
  // select a optimizatio objective
  ompl::base::OptimizationObjectivePtr length_objective(
    new ompl::base::PathLengthOptimizationObjective(simple_setup_->getSpaceInformation()));
  ompl::base::OptimizationObjectivePtr octocost_objective(
    new OctoCostOptimizationObjective(
      simple_setup_->getSpaceInformation(), elevated_surfel_octomap_octree_));

  ompl::base::MultiOptimizationObjective * multi_optimization =
    new ompl::base::MultiOptimizationObjective(simple_setup_->getSpaceInformation());
  multi_optimization->addObjective(length_objective, 1.0);
  multi_optimization->addObjective(octocost_objective, 2.0);

  return ompl::base::OptimizationObjectivePtr(multi_optimization);
}

std::vector<geometry_msgs::msg::PoseStamped> ElevationPlanner::getOverlayedStartandGoal()
{
  std::vector<geometry_msgs::msg::PoseStamped> start_pose_vector;
  start_pose_vector.push_back(nearest_elevated_surfel_to_start_);
  start_pose_vector.push_back(nearest_elevated_surfel_to_goal_);
  return start_pose_vector;
}
}  // namespace vox_nav_planning

PLUGINLIB_EXPORT_CLASS(vox_nav_planning::ElevationPlanner, vox_nav_planning::PlannerCore)
