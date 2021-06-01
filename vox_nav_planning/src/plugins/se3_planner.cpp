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
#include <random>

namespace vox_nav_planning
{

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
  node_poses_msg_ = std::make_shared<geometry_msgs::msg::PoseArray>();

  node_surfels_ = pcl::PointCloud<pcl::PointSurfel>::Ptr(
    new pcl::PointCloud<pcl::PointSurfel>);

  is_octomap_ready_ = false;
  is_node_poses_ready_ = false;

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

  fcl::CollisionObject robot_body_box_object(robot_body_box, fcl::Transform3f());
  robot_collision_object_ = std::make_shared<fcl::CollisionObject>(robot_body_box_object);

  octomap_subscriber_ = parent->create_subscription<octomap_msgs::msg::Octomap>(
    octomap_topic_, rclcpp::SystemDefaultsQoS(),
    std::bind(&SE3Planner::octomapCallback, this, std::placeholders::_1));

  node_poses_subscriber_ = parent->create_subscription<geometry_msgs::msg::PoseArray>(
    "node_poses", rclcpp::SystemDefaultsQoS(),
    std::bind(&SE3Planner::nodePosesCallback, this, std::placeholders::_1));

  state_space_ = std::make_shared<ompl::base::SE3StateSpace>();
  state_space_->as<ompl::base::SE3StateSpace>()->setBounds(*state_space_bounds_);
  simple_setup_ = std::make_shared<ompl::geometric::SimpleSetup>(state_space_);

  nodes_octree_ = std::make_shared<octomap::OcTree>(octomap_voxel_size_);
  full_map_octree_ = std::make_shared<octomap::OcTree>(octomap_voxel_size_);

  if (!is_enabled_) {
    RCLCPP_WARN(logger_, "SE3Planner plugin is disabled.");
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

  if (!is_octomap_ready_) {
    RCLCPP_WARN(
      logger_, "A valid Octomap has not been receievd yet, Try later again."
    );
    return std::vector<geometry_msgs::msg::PoseStamped>();
  }

  // set the start and goal states
  double start_yaw, goal_yaw, nan;
  vox_nav_utilities::getRPYfromMsgQuaternion(start.pose.orientation, nan, nan, start_yaw);
  vox_nav_utilities::getRPYfromMsgQuaternion(goal.pose.orientation, nan, nan, goal_yaw);

  ompl::base::ScopedState<ompl::base::SE3StateSpace>
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
    pcl::PointCloud<pcl::PointSurfel>::Ptr>(start_nearest_surfel, start_actual, node_surfels_);

  vox_nav_utilities::getNearstPoint<
    pcl::PointSurfel,
    pcl::PointCloud<pcl::PointSurfel>::Ptr>(goal_nearest_surfel, goal_acual, node_surfels_);

  nearest_node_to_start_.pose.position.x = start_nearest_surfel.x;
  nearest_node_to_start_.pose.position.y = start_nearest_surfel.y;
  nearest_node_to_start_.pose.position.z = start_nearest_surfel.z;

  nearest_node_to_goal_.pose.position.x = goal_nearest_surfel.x;
  nearest_node_to_goal_.pose.position.y = goal_nearest_surfel.y;
  nearest_node_to_goal_.pose.position.z = goal_nearest_surfel.z;

  nearest_node_to_start_.pose.orientation = vox_nav_utilities::getMsgQuaternionfromRPY(
    start_nearest_surfel.normal_x,
    start_nearest_surfel.normal_y,
    start_nearest_surfel.normal_z
  );

  nearest_node_to_goal_.pose.orientation = vox_nav_utilities::getMsgQuaternionfromRPY(
    goal_nearest_surfel.normal_x,
    goal_nearest_surfel.normal_y,
    goal_nearest_surfel.normal_z);

  se3_start->setXYZ(
    nearest_node_to_start_.pose.position.x,
    nearest_node_to_start_.pose.position.y,
    nearest_node_to_start_.pose.position.z);

  se3_goal->setXYZ(
    nearest_node_to_goal_.pose.position.x,
    nearest_node_to_goal_.pose.position.y,
    nearest_node_to_goal_.pose.position.z);

  se3_start->as<ompl::base::SO3StateSpace::StateType>(1)->x =
    nearest_node_to_start_.pose.orientation.x;
  se3_start->as<ompl::base::SO3StateSpace::StateType>(1)->y =
    nearest_node_to_start_.pose.orientation.y;
  se3_start->as<ompl::base::SO3StateSpace::StateType>(1)->z =
    nearest_node_to_start_.pose.orientation.z;
  se3_start->as<ompl::base::SO3StateSpace::StateType>(1)->w =
    nearest_node_to_start_.pose.orientation.w;

  se3_goal->as<ompl::base::SO3StateSpace::StateType>(1)->x =
    nearest_node_to_goal_.pose.orientation.x;
  se3_goal->as<ompl::base::SO3StateSpace::StateType>(1)->y =
    nearest_node_to_goal_.pose.orientation.y;
  se3_goal->as<ompl::base::SO3StateSpace::StateType>(1)->z =
    nearest_node_to_goal_.pose.orientation.z;
  se3_goal->as<ompl::base::SO3StateSpace::StateType>(1)->w =
    nearest_node_to_goal_.pose.orientation.w;

  simple_setup_->setStartAndGoalStates(se3_start, se3_goal);

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
      &SE3Planner::
      allocValidStateSampler, this, std::placeholders::_1));

  // attempt to solve the problem within one second of planning time
  ompl::base::PlannerStatus solved = simple_setup_->solve(planner_timeout_);
  std::vector<geometry_msgs::msg::PoseStamped> plan_poses;

  if (solved) {
    ompl::geometric::PathGeometric solution_path = simple_setup_->getSolutionPath();
    // Path smoothing using bspline
    ompl::geometric::PathSimplifier * path_simlifier =
      new ompl::geometric::PathSimplifier(simple_setup_->getSpaceInformation());

    path_simlifier->smoothBSpline(solution_path, 3);
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

bool SE3Planner::isStateValid(const ompl::base::State * state)
{
  if (is_octomap_ready_) {
    // cast the abstract state type to the type we expect
    const ompl::base::SE3StateSpace::StateType * se3state =
      state->as<ompl::base::SE3StateSpace::StateType>();
    // extract the second component of the state and cast it to what we expect
    const ompl::base::SO3StateSpace::StateType * rot =
      se3state->as<ompl::base::SO3StateSpace::StateType>(1);
    fcl::CollisionRequest requestType(1, false, 1, false);

    // check validity of state Fdefined by pos & rot
    fcl::Vec3f translation(se3state->getX(), se3state->getY(), se3state->getZ());
    fcl::Quaternion3f rotation(rot->w, rot->x, rot->y, rot->z);
    robot_collision_object_->setTransform(rotation, translation);

    fcl::CollisionResult collisionWithNodesResult, collisionWitFullMapResult;

    fcl::collide(
      robot_collision_object_.get(),
      fcl_nodes_collision_object_.get(), requestType, collisionWithNodesResult);

    fcl::collide(
      robot_collision_object_.get(),
      fcl_full_map_collision_object_.get(), requestType, collisionWitFullMapResult);

    return true; //collisionWithNodesResult.isCollision();
  } else {
    RCLCPP_ERROR(
      logger_,
      "The Octomap has not been recieved correctly, Collision check "
      "cannot be processed without a valid Octomap!");
    return false;
  }
}


void SE3Planner::nodePosesCallback(
  const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  if (!is_node_poses_ready_) {
    node_poses_msg_ = msg;
    is_node_poses_ready_ = true;
    RCLCPP_INFO(logger_, "Node poses has been recieved!");
    for (auto && i : node_poses_msg_->poses) {
      pcl::PointSurfel surfel;
      surfel.x = i.position.x;
      surfel.y = i.position.y;
      surfel.z = i.position.z;
      double r, p, y;
      vox_nav_utilities::getRPYfromMsgQuaternion(i.orientation, r, p, y);
      surfel.normal_x = r;
      surfel.normal_y = p;
      surfel.normal_z = y;
      node_surfels_->points.push_back(surfel);
    }
  }
}


void SE3Planner::octomapCallback(
  const octomap_msgs::msg::Octomap::ConstSharedPtr msg)
{
  const std::lock_guard<std::mutex> lock(octomap_mutex_);

  if (!is_octomap_ready_ && is_node_poses_ready_) {
    octomap_msg_ = msg;
    RCLCPP_INFO(logger_, "Octomap has been recieved!");
    try {

      auto raw_color_octomap_octree =
        dynamic_cast<octomap::ColorOcTree *>(octomap_msgs::fullMsgToMap(*octomap_msg_));

      auto full_map_and_nodes_octree =
        std::make_shared<octomap::ColorOcTree>(*raw_color_octomap_octree);
      delete raw_color_octomap_octree;

      for (auto it = full_map_and_nodes_octree->begin(),
        end = full_map_and_nodes_octree->end(); it != end; ++it)
      {
        auto crr_point_node_key = full_map_and_nodes_octree->coordToKey(it.getCoordinate());
        if (it->getValue() > 2.0) {
          nodes_octree_->setNodeValue(crr_point_node_key, it->getValue(), false);
        } else {
          full_map_octree_->setNodeValue(crr_point_node_key, it->getValue(), false);
        }
      }

      auto nodes_octree = std::make_shared<fcl::OcTree>(nodes_octree_);
      fcl_nodes_collision_object_ = std::make_shared<fcl::CollisionObject>(
        std::shared_ptr<fcl::CollisionGeometry>(nodes_octree));

      auto full_map_octree = std::make_shared<fcl::OcTree>(full_map_octree_);
      fcl_full_map_collision_object_ = std::make_shared<fcl::CollisionObject>(
        std::shared_ptr<fcl::CollisionGeometry>(full_map_octree));

      RCLCPP_INFO(
        logger_,
        "Recieved a valid Octomap with %d nodes, A FCL collision tree will be created from this "
        "octomap for state validity (aka collision check)", full_map_octree_->size());

      RCLCPP_INFO(
        logger_,
        "Collisison check Octomap with %d nodes", nodes_octree_->size());
      is_octomap_ready_ = true;

      simple_setup_->setOptimizationObjective(getOptimizationObjective());

      simple_setup_->setStateValidityChecker(
        std::bind(
          &SE3Planner::
          isStateValid, this, std::placeholders::_1));

    } catch (const std::exception & e) {
      std::cerr << e.what() << "\n";
      RCLCPP_ERROR(
        logger_,
        "Exception while converting octomap  %s:", e.what());
    }
  }
}

ompl::base::ValidStateSamplerPtr SE3Planner::allocValidStateSampler(
  const ompl::base::SpaceInformation * si)
{
  octocell_valid_state_sampler_ = std::make_shared<OctoCellValidStateSampler>(
    simple_setup_->getSpaceInformation(),
    nearest_node_to_start_, nearest_node_to_goal_,
    nodes_octree_,
    full_map_octree_,
    robot_collision_object_,
    fcl_full_map_collision_object_,
    fcl_nodes_collision_object_,
    node_poses_msg_);
  return octocell_valid_state_sampler_;
}

ompl::base::OptimizationObjectivePtr SE3Planner::getOptimizationObjective()
{
  // select a optimizatio objective
  ompl::base::OptimizationObjectivePtr length_objective(
    new ompl::base::PathLengthOptimizationObjective(simple_setup_->getSpaceInformation()));

  ompl::base::OptimizationObjectivePtr octocost_objective(
    new OctoCostOptimizationObjective(
      simple_setup_->getSpaceInformation(), nodes_octree_));

  octocost_optimization_ = octocost_objective;

  return length_objective;
}

std::vector<geometry_msgs::msg::PoseStamped> SE3Planner::getOverlayedStartandGoal()
{
  std::vector<geometry_msgs::msg::PoseStamped> start_pose_vector;
  start_pose_vector.push_back(nearest_node_to_start_);
  start_pose_vector.push_back(nearest_node_to_goal_);
  return start_pose_vector;
}
}  // namespace vox_nav_planning

PLUGINLIB_EXPORT_CLASS(vox_nav_planning::SE3Planner, vox_nav_planning::PlannerCore)
