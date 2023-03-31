// Copyright (c) 2023 Fetullah Atas, Norwegian University of Life Sciences
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

#include "vox_nav_planning/plugins/osm_elevation_planner.hpp"
#include <pluginlib/class_list_macros.hpp>

#include <memory>
#include <random>
#include <string>
#include <vector>

namespace vox_nav_planning
{

  OSMElevationPlanner::OSMElevationPlanner() {}

  OSMElevationPlanner::~OSMElevationPlanner() {}

  void OSMElevationPlanner::initialize(
    rclcpp::Node * parent,
    const std::string & plugin_name)
  {
    is_map_ready_ = false;
    se2_bounds_ = std::make_shared<ompl::base::RealVectorBounds>(2);
    z_bounds_ = std::make_shared<ompl::base::RealVectorBounds>(1);
    auto v_bounds = std::make_shared<ompl::base::RealVectorBounds>(1);

    // declare only planner specific parameters here
    // common parameters are declared in server
    parent->declare_parameter(plugin_name + ".se2_space", "REEDS");
    parent->declare_parameter(plugin_name + ".rho", 1.5);
    parent->declare_parameter(plugin_name + ".state_space_boundries.minx", -10.0);
    parent->declare_parameter(plugin_name + ".state_space_boundries.maxx", 10.0);
    parent->declare_parameter(plugin_name + ".state_space_boundries.miny", -10.0);
    parent->declare_parameter(plugin_name + ".state_space_boundries.maxy", 10.0);
    parent->declare_parameter(plugin_name + ".state_space_boundries.minz", -10.0);
    parent->declare_parameter(plugin_name + ".state_space_boundries.maxz", 10.0);

    parent->get_parameter("planner_name", planner_name_);
    parent->get_parameter("planner_timeout", planner_timeout_);
    parent->get_parameter("interpolation_parameter", interpolation_parameter_);
    parent->get_parameter(plugin_name + ".se2_space", selected_se2_space_name_);
    parent->get_parameter(plugin_name + ".rho", rho_);

    se2_bounds_->setLow(
      0, parent->get_parameter(plugin_name + ".state_space_boundries.minx")
      .as_double());
    se2_bounds_->setHigh(
      0, parent->get_parameter(plugin_name + ".state_space_boundries.maxx")
      .as_double());
    se2_bounds_->setLow(
      1, parent->get_parameter(plugin_name + ".state_space_boundries.miny")
      .as_double());
    se2_bounds_->setHigh(
      1, parent->get_parameter(plugin_name + ".state_space_boundries.maxy")
      .as_double());
    z_bounds_->setLow(
      0, parent->get_parameter(plugin_name + ".state_space_boundries.minz")
      .as_double());
    z_bounds_->setHigh(
      0, parent->get_parameter(plugin_name + ".state_space_boundries.maxz")
      .as_double());
    v_bounds->setLow(0, -0.5);
    v_bounds->setHigh(0, 0.5);

    if (selected_se2_space_name_ == "SE2") {
      se2_space_type_ = ompl::base::ElevationStateSpace::SE2StateType::SE2;
    } else if (selected_se2_space_name_ == "DUBINS") {
      se2_space_type_ = ompl::base::ElevationStateSpace::SE2StateType::DUBINS;
    } else {
      se2_space_type_ = ompl::base::ElevationStateSpace::SE2StateType::REDDSSHEEP;
    }

    typedef std::shared_ptr<fcl::CollisionGeometryf> CollisionGeometryPtr_t;
    CollisionGeometryPtr_t robot_body_box(
      new fcl::Box<float>(
        parent->get_parameter("robot_body_dimens.x").as_double(),
        parent->get_parameter("robot_body_dimens.y").as_double(),
        parent->get_parameter("robot_body_dimens.z").as_double()));

    fcl::CollisionObjectf robot_body_box_object(robot_body_box,
      fcl::Transform3f());

    get_map_client_node_ =
      std::make_shared<rclcpp::Node>("GetOSMRoadTopologyMap");

    get_osm_road_topology_map_client_ =
      get_map_client_node_
      ->create_client<vox_nav_msgs::srv::GetOSMRoadTopologyMap>(
      "get_osm_map");

    RCLCPP_INFO(logger_, "Selected planner is: %s", planner_name_.c_str());

    setupMap();

    // WARN elevated_surfel_poses_msg_ needs to be populated by setupMap();
    //state_space_ = std::make_shared<ompl::base::ElevationStateSpace>(
    // se2_space_type_, elevated_surfel_poses_msg_,
    // rho_ /*only valid for duins or reeds*/,
    // false /*only valid for dubins*/);

    state_space_->as<ompl::base::ElevationStateSpace>()->setBounds(
      *se2_bounds_,
      *z_bounds_,
      *v_bounds);

    simple_setup_ = std::make_shared<ompl::geometric::SimpleSetup>(state_space_);

    ompl::base::OptimizationObjectivePtr length_objective(
      new ompl::base::PathLengthOptimizationObjective(
        simple_setup_->getSpaceInformation()));

    simple_setup_->setStateValidityChecker(
      std::bind(&OSMElevationPlanner::isStateValid, this, std::placeholders::_1));
  }

  std::vector<geometry_msgs::msg::PoseStamped>
  OSMElevationPlanner::createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal)
  {
    if (!is_map_ready_) {
      RCLCPP_WARN(
        logger_,
        "A valid Octomap has not been recived yet, Try later again.");
      return std::vector<geometry_msgs::msg::PoseStamped>();
    }
    // set the start and goal states
    double start_yaw, goal_yaw, nan;
    vox_nav_utilities::getRPYfromMsgQuaternion(
      start.pose.orientation, nan, nan,
      start_yaw);
    vox_nav_utilities::getRPYfromMsgQuaternion(
      goal.pose.orientation, nan, nan,
      goal_yaw);

    ompl::base::ScopedState<ompl::base::ElevationStateSpace> se3_start(
      state_space_),
    se3_goal(state_space_);

    se3_start->setXYZV(
      start.pose.position.x,
      start.pose.position.y,
      start.pose.position.z, 0);
    se3_start->setSO2(start_yaw);

    se3_goal->setXYZV(
      goal.pose.position.x,
      goal.pose.position.y,
      goal.pose.position.z, 0);
    se3_goal->setSO2(goal_yaw);

    simple_setup_->setStartAndGoalStates(se3_start, se3_goal);

    auto si = simple_setup_->getSpaceInformation();
    // create a planner for the defined space
    ompl::base::PlannerPtr planner;
    vox_nav_utilities::initializeSelectedPlanner(
      planner, planner_name_, si, logger_);

    si->setValidStateSamplerAllocator(
      std::bind(
        &OSMElevationPlanner::allocValidStateSampler, this,
        std::placeholders::_1));

    simple_setup_->setPlanner(planner);
    simple_setup_->setup();

    // attempt to solve the problem within one second of planning time
    ompl::base::PlannerStatus solved = simple_setup_->solve(planner_timeout_);
    std::vector<geometry_msgs::msg::PoseStamped> plan_poses;

    if (solved) {
      ompl::geometric::PathGeometric solution_path =
        simple_setup_->getSolutionPath();
      ompl::geometric::PathSimplifier * path_simlifier =
        new ompl::geometric::PathSimplifier(si);
      solution_path.interpolate(interpolation_parameter_);
      path_simlifier->smoothBSpline(solution_path, 1, 0.1);

      for (std::size_t path_idx = 0; path_idx < solution_path.getStateCount();
        path_idx++)
      {
        const auto * cstate =
          solution_path.getState(path_idx)->as<ompl::base::ElevationStateSpace::StateType>();
        const auto * cstate_so2 = cstate->as<ompl::base::SO2StateSpace::StateType>(0);
        const auto * cstate_xyzv = cstate->as<ompl::base::RealVectorStateSpace::StateType>(1);
        double yaw = cstate_so2->value;
        double x = cstate_xyzv->values[0];
        double y = cstate_xyzv->values[1];
        double z = cstate_xyzv->values[2];
        tf2::Quaternion this_pose_quat;
        this_pose_quat.setRPY(0, 0, yaw);
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = start.header.frame_id;
        pose.header.stamp = rclcpp::Clock().now();
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = z;
        pose.pose.orientation.x = this_pose_quat.getX();
        pose.pose.orientation.y = this_pose_quat.getY();
        pose.pose.orientation.z = this_pose_quat.getZ();
        pose.pose.orientation.w = this_pose_quat.getW();
        plan_poses.push_back(pose);
      }

      RCLCPP_INFO(logger_, "Found A plan with %i poses", plan_poses.size());
      RCLCPP_INFO(logger_, "Path Length, %s, %.2f", planner_name_.c_str(), solution_path.length());
      RCLCPP_INFO(
        logger_, "Path Smoothness, %s, %.2f",
        planner_name_.c_str(), solution_path.smoothness());

    } else {
      RCLCPP_WARN(logger_, "No solution for requested path planning !");
    }

    simple_setup_->clear();
    return plan_poses;
  }

  bool OSMElevationPlanner::isStateValid(const ompl::base::State * state)
  {
    return true;
  }

  void OSMElevationPlanner::setupMap()
  {
    const std::lock_guard<std::mutex> lock(map_mutex_);

    while (!is_map_ready_ && rclcpp::ok()) {

      auto request =
        std::make_shared<vox_nav_msgs::srv::GetOSMRoadTopologyMap::Request>();

      while (!get_osm_road_topology_map_client_->wait_for_service(
          std::chrono::seconds(1)))
      {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(
            logger_, "Interrupted while waiting for the "
            "get_traversability_map service. Exiting");
          return;
        }
        RCLCPP_INFO(
          logger_, "get_traversability_map service not available, "
          "waiting and trying again");
      }

      auto result_future =
        get_osm_road_topology_map_client_->async_send_request(request);
      if (rclcpp::spin_until_future_complete(
          get_map_client_node_,
          result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
      {
        RCLCPP_ERROR(logger_, "/get_traversability_map service call failed");
      }
      auto response = result_future.get();

      if (response->is_valid) {
        is_map_ready_ = true;
      } else {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        RCLCPP_INFO(
          logger_,
          "Waiting for GetTraversabilityMap service to provide correct maps.");
        continue;
      }

    }
  }

  ompl::base::OptimizationObjectivePtr
  OSMElevationPlanner::getOptimizationObjective()
  {
    // select a optimizatio objective
    ompl::base::OptimizationObjectivePtr length_objective(
      new ompl::base::PathLengthOptimizationObjective(
        simple_setup_->getSpaceInformation()));

    ompl::base::MultiOptimizationObjective * multi_optimization =
      new ompl::base::MultiOptimizationObjective(
      simple_setup_->getSpaceInformation());
    multi_optimization->addObjective(length_objective, 1.0);

    return ompl::base::OptimizationObjectivePtr(multi_optimization);
  }

  ompl::base::ValidStateSamplerPtr OSMElevationPlanner::allocValidStateSampler(
    const ompl::base::SpaceInformation * si)
  {
    /*auto valid_sampler = std::make_shared<ompl::base::OctoCellValidStateSampler>(
      simple_setup_->getSpaceInformation(), nearest_elevated_surfel_to_start_,
      nearest_elevated_surfel_to_goal_, elevated_surfel_poses_msg_);
    return valid_sampler;*/
  }


} // namespace vox_nav_planning

PLUGINLIB_EXPORT_CLASS(
  vox_nav_planning::OSMElevationPlanner,
  vox_nav_planning::PlannerCore)
