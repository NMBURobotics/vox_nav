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
    auto control_bounds = std::make_shared<ompl::base::RealVectorBounds>(2);

    osm_road_topology_pcd_ = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    valid_poses_ = std::make_shared<geometry_msgs::msg::PoseArray>();

    // declare only planner specific parameters here
    // common parameters are declared in server
    parent->declare_parameter(plugin_name + ".se2_space", "REEDS");
    parent->declare_parameter(plugin_name + ".rho", 1.5);
    parent->declare_parameter(plugin_name + ".goal_tolerance", 2.0);
    parent->declare_parameter(plugin_name + ".state_space_boundries.minx", -10.0);
    parent->declare_parameter(plugin_name + ".state_space_boundries.maxx", 10.0);
    parent->declare_parameter(plugin_name + ".state_space_boundries.miny", -10.0);
    parent->declare_parameter(plugin_name + ".state_space_boundries.maxy", 10.0);
    parent->declare_parameter(plugin_name + ".state_space_boundries.minz", -10.0);
    parent->declare_parameter(plugin_name + ".state_space_boundries.maxz", 10.0);
    parent->declare_parameter(plugin_name + ".control_boundries.minv", -0.5);
    parent->declare_parameter(plugin_name + ".control_boundries.maxv", 0.5);
    parent->declare_parameter(plugin_name + ".control_boundries.minw", -0.5);
    parent->declare_parameter(plugin_name + ".control_boundries.maxw", 0.5);

    parent->get_parameter("planner_name", planner_name_);
    parent->get_parameter("planner_timeout", planner_timeout_);
    parent->get_parameter("interpolation_parameter", interpolation_parameter_);
    parent->get_parameter(plugin_name + ".se2_space", selected_se2_space_name_);
    parent->get_parameter(plugin_name + ".rho", rho_);
    parent->get_parameter(plugin_name + ".goal_tolerance", goal_tolerance_);

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
    control_bounds->setLow(
      0, parent->get_parameter(plugin_name + ".control_boundries.minv").as_double());
    control_bounds->setHigh(
      0, parent->get_parameter(plugin_name + ".control_boundries.maxv").as_double());
    control_bounds->setLow(
      1, parent->get_parameter(plugin_name + ".control_boundries.minw").as_double());
    control_bounds->setHigh(
      1, parent->get_parameter(plugin_name + ".control_boundries.maxw").as_double());

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
      get_map_client_node_->create_client<vox_nav_msgs::srv::GetOSMRoadTopologyMap>(
      "get_osm_map");

    RCLCPP_INFO(logger_, "Selected planner is: %s", planner_name_.c_str());

    setupMap();

    // WARN elevated_surfel_poses_msg_ needs to be populated by setupMap();
    state_space_ = std::make_shared<ompl::base::ElevationStateSpace>(
      se2_space_type_,
      rho_ /*only valid for duins or reeds*/,
      false /*only valid for dubins*/);

    state_space_->as<ompl::base::ElevationStateSpace>()->setBounds(
      *se2_bounds_,
      *z_bounds_,
      *v_bounds);
    state_space_->setLongestValidSegmentFraction(0.1);

    control_state_space_ = std::make_shared<ompl::control::RealVectorControlSpace>(state_space_, 2);
    control_state_space_->as<ompl::control::RealVectorControlSpace>()->setBounds(*control_bounds);

    control_simple_setup_ = std::make_shared<ompl::control::SimpleSetup>(control_state_space_);
    control_simple_setup_->setOptimizationObjective(getOptimizationObjective());
    control_simple_setup_->setStateValidityChecker(
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
    start_pose_ = start;
    goal_pose_ = goal;
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
      start.pose.position.z, 0);
    se3_goal->setSO2(goal_yaw);

    // print start and goal states
    state_space_->printState(se3_start.get(), std::cout);
    state_space_->printState(se3_goal.get(), std::cout);

    control_simple_setup_->setStartAndGoalStates(se3_start, se3_goal, goal_tolerance_);

    auto si = control_simple_setup_->getSpaceInformation();
    si->setMinMaxControlDuration(20, 30);
    si->setPropagationStepSize(0.025);

    control_simple_setup_->setStatePropagator(
      [this, si](const ompl::base::State * state, const ompl::control::Control * control,
      const double duration, ompl::base::State * result)
      {
        this->propagate(si.get(), state, control, duration, result);
      });

    // create a planner for the defined space
    ompl::base::PlannerPtr planner;
    initializeSelectedControlPlanner(
      planner, planner_name_, si, logger_);

    si->setValidStateSamplerAllocator(
      std::bind(
        &OSMElevationPlanner::allocValidStateSampler, this,
        std::placeholders::_1));

    control_simple_setup_->setPlanner(planner);
    control_simple_setup_->setup();

    // attempt to solve the problem within one second of planning time
    ompl::base::PlannerStatus solved = control_simple_setup_->solve(planner_timeout_);
    std::vector<geometry_msgs::msg::PoseStamped> plan_poses;

    if (solved) {

      ompl::control::PathControl solution_path(si);
      try {
        control_simple_setup_->getSolutionPath().printAsMatrix(std::cout);
        solution_path = control_simple_setup_->getSolutionPath();
      } catch (const std::exception & e) {
        std::cerr << e.what() << '\n';
        RCLCPP_WARN(
          logger_, "Exception occured while retrivieng control solution path %s",
          e.what());
        control_simple_setup_->clear();
        return plan_poses;
      }

      RCLCPP_INFO(
        logger_, "A solution was found, the simplified solution path includes %d poses.",
        static_cast<int>(solution_path.getStateCount()));

      ompl::geometric::PathSimplifier * path_simlifier = new ompl::geometric::PathSimplifier(si);
      //solution_path.interpolate(solution_path);
      //path_simlifier->smoothBSpline(solution_path, 1, 0.1);

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

    } else {
      RCLCPP_WARN(logger_, "No solution for requested path planning !");
    }

    control_simple_setup_->clear();
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
            "get_osm_map service. Exiting");
          return;
        }
        RCLCPP_INFO(
          logger_, "get_osm_map service not available, "
          "waiting and trying again");
      }

      auto result_future =
        get_osm_road_topology_map_client_->async_send_request(request);
      if (rclcpp::spin_until_future_complete(
          get_map_client_node_,
          result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
      {
        RCLCPP_ERROR(logger_, "/get_osm_map service call failed");
      }
      auto response = result_future.get();

      if (response->is_valid) {
        is_map_ready_ = true;
        pcl::fromROSMsg(response->osm_road_topology, *osm_road_topology_pcd_);
        for (auto & point : osm_road_topology_pcd_->points) {
          geometry_msgs::msg::Pose pose;
          pose.position.x = point.x;
          pose.position.y = point.y;
          pose.position.z = point.z;
          valid_poses_->poses.push_back(pose);
        }

        RCLCPP_INFO(
          logger_, "Received a valid map with %i points",
          osm_road_topology_pcd_->points.size());

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
        control_simple_setup_->getSpaceInformation()));

    ompl::base::MultiOptimizationObjective * multi_optimization =
      new ompl::base::MultiOptimizationObjective(
      control_simple_setup_->getSpaceInformation());
    multi_optimization->addObjective(length_objective, 1.0);

    return length_objective;
  }

  ompl::base::ValidStateSamplerPtr OSMElevationPlanner::allocValidStateSampler(
    const ompl::base::SpaceInformation * si)
  {
    auto valid_sampler = std::make_shared<ompl::base::OctoCellValidStateSampler>(
      control_simple_setup_->getSpaceInformation(),
      start_pose_,
      goal_pose_,
      valid_poses_);
    return valid_sampler;
  }

  std::vector<geometry_msgs::msg::PoseStamped> OSMElevationPlanner::getOverlayedStartandGoal()
  {
    std::vector<geometry_msgs::msg::PoseStamped> start_pose_vector;
    start_pose_vector.push_back(start_pose_);
    start_pose_vector.push_back(goal_pose_);
    return start_pose_vector;
  }


} // namespace vox_nav_planning

PLUGINLIB_EXPORT_CLASS(
  vox_nav_planning::OSMElevationPlanner,
  vox_nav_planning::PlannerCore)
