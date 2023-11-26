// Copyright (c) 2022 Fetullah Atas, Norwegian University of Life Sciences
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

#include "car_control_planners_benchmark.hpp"

namespace vox_nav_planning
{
CarControlPlannersBenchMarking::CarControlPlannersBenchMarking() : Node("car_control_planners_benchmark")
{
  RCLCPP_INFO(this->get_logger(), "Creating:");

  se2_bounds_ = std::make_shared<ompl::base::RealVectorBounds>(2);
  z_bounds_ = std::make_shared<ompl::base::RealVectorBounds>(1);
  auto v_bounds = std::make_shared<ompl::base::RealVectorBounds>(1);
  auto control_bounds = std::make_shared<ompl::base::RealVectorBounds>(2);
  elevated_surfel_poses_msg_ = std::make_shared<geometry_msgs::msg::PoseArray>();

  elevated_surfel_cloud_ = pcl::PointCloud<pcl::PointSurfel>::Ptr(new pcl::PointCloud<pcl::PointSurfel>);
  is_map_ready_ = false;

  this->declare_parameter("selected_planners", std::vector<std::string>({ "RRTstar", "PRMstar" }));
  this->declare_parameter("robot_mesh_path", "");
  this->declare_parameter("planner_timeout", 5.0);
  this->declare_parameter("interpolation_parameter", 50);
  this->declare_parameter("octomap_voxel_size", 0.2);
  this->declare_parameter("selected_state_space", "REEDS");
  this->declare_parameter("min_turning_radius", 2.5);
  this->declare_parameter("state_space_boundries.minx", -50.0);
  this->declare_parameter("state_space_boundries.maxx", 50.0);
  this->declare_parameter("state_space_boundries.miny", -10.0);
  this->declare_parameter("state_space_boundries.maxy", 10.0);
  this->declare_parameter("state_space_boundries.minz", -10.0);
  this->declare_parameter("state_space_boundries.maxz", 10.0);
  this->declare_parameter("state_space_boundries.minyaw", -3.14);
  this->declare_parameter("state_space_boundries.maxyaw", 3.14);
  this->declare_parameter("state_space_boundries.minv", -1.5);
  this->declare_parameter("state_space_boundries.maxv", 1.5);
  this->declare_parameter("control_boundries.minv", -0.5);
  this->declare_parameter("control_boundries.maxv", 0.5);
  this->declare_parameter("control_boundries.minw", -0.5);
  this->declare_parameter("control_boundries.maxw", 0.5);
  this->declare_parameter("robot_body_dimens.x", 1.5);
  this->declare_parameter("robot_body_dimens.y", 1.5);
  this->declare_parameter("robot_body_dimens.z", 0.4);
  this->declare_parameter("start.z", 0.0);
  this->declare_parameter("goal.z", 0.0);
  this->declare_parameter("goal_tolerance", 0.5);
  this->declare_parameter("min_euclidean_dist_start_to_goal", 25.0);
  this->declare_parameter("batch_size", 10);
  this->declare_parameter("epochs", 10);
  this->declare_parameter("max_memory", 2048);
  this->declare_parameter("results_output_dir", "/home/atas/");
  this->declare_parameter("results_file_regex", "SE2");
  this->declare_parameter("publish_a_sample_bencmark", true);
  this->declare_parameter("sample_bencmark_plans_topic", "benchmark_plan");
  this->declare_parameter("octomap_from_file", "");

  this->get_parameter("selected_planners", selected_planners_);
  this->get_parameter("robot_mesh_path", robot_mesh_path_);
  this->get_parameter("planner_timeout", planner_timeout_);
  this->get_parameter("interpolation_parameter", interpolation_parameter_);
  this->get_parameter("octomap_voxel_size", octomap_voxel_size_);
  this->get_parameter("selected_state_space", selected_se2_space_name_);
  this->get_parameter("min_turning_radius", min_turning_radius_);
  this->get_parameter("robot_body_dimens.x", robot_body_dimensions_.x);
  this->get_parameter("robot_body_dimens.y", robot_body_dimensions_.y);
  this->get_parameter("robot_body_dimens.z", robot_body_dimensions_.z);
  this->get_parameter("start.z", start_.z);
  this->get_parameter("goal.z", goal_.z);
  this->get_parameter("goal_tolerance", goal_tolerance_);
  this->get_parameter("min_euclidean_dist_start_to_goal", min_euclidean_dist_start_to_goal_);
  this->get_parameter("batch_size", batch_size_);
  this->get_parameter("epochs", epochs_);
  this->get_parameter("max_memory", max_memory_);
  this->get_parameter("results_output_dir", results_output_dir_);
  this->get_parameter("results_file_regex", results_file_regex_);
  this->get_parameter("publish_a_sample_bencmark", publish_a_sample_bencmark_);
  this->get_parameter("sample_bencmark_plans_topic", sample_bencmark_plans_topic_);
  this->get_parameter("octomap_from_file", octomap_from_file_);

  this->get_parameter("state_space_boundries.minx", se_bounds_.minx);
  this->get_parameter("state_space_boundries.maxx", se_bounds_.maxx);
  this->get_parameter("state_space_boundries.miny", se_bounds_.miny);
  this->get_parameter("state_space_boundries.maxy", se_bounds_.maxy);
  this->get_parameter("state_space_boundries.minz", se_bounds_.minz);
  this->get_parameter("state_space_boundries.maxz", se_bounds_.maxz);
  this->get_parameter("state_space_boundries.minyaw", se_bounds_.minyaw);
  this->get_parameter("state_space_boundries.maxyaw", se_bounds_.maxyaw);

  se2_bounds_->setLow(0, this->get_parameter("state_space_boundries.minx").as_double());
  se2_bounds_->setHigh(0, this->get_parameter("state_space_boundries.maxx").as_double());
  se2_bounds_->setLow(1, this->get_parameter("state_space_boundries.miny").as_double());
  se2_bounds_->setHigh(1, this->get_parameter("state_space_boundries.maxy").as_double());
  z_bounds_->setLow(0, this->get_parameter("state_space_boundries.minz").as_double());
  z_bounds_->setHigh(0, this->get_parameter("state_space_boundries.maxz").as_double());
  v_bounds->setLow(0, this->get_parameter("state_space_boundries.minv").as_double());
  v_bounds->setHigh(0, this->get_parameter("state_space_boundries.maxv").as_double());
  control_bounds->setLow(0, this->get_parameter("control_boundries.minv").as_double());
  control_bounds->setHigh(0, this->get_parameter("control_boundries.maxv").as_double());
  control_bounds->setLow(1, this->get_parameter("control_boundries.minw").as_double());
  control_bounds->setHigh(1, this->get_parameter("control_boundries.maxw").as_double());

  if (selected_se2_space_name_ == "SE2")
  {
    se2_space_type_ = ompl::base::ElevationStateSpace::SE2StateType::SE2;
  }
  else if (selected_se2_space_name_ == "DUBINS")
  {
    se2_space_type_ = ompl::base::ElevationStateSpace::SE2StateType::DUBINS;
  }
  else
  {
    se2_space_type_ = ompl::base::ElevationStateSpace::SE2StateType::REDDSSHEEP;
  }

  typedef std::shared_ptr<fcl::CollisionGeometryf> CollisionGeometryPtr_t;
  CollisionGeometryPtr_t robot_body_box(
      new fcl::Box<float>(robot_body_dimensions_.x, robot_body_dimensions_.y, robot_body_dimensions_.z));

  fcl::CollisionObjectf robot_body_box_object(robot_body_box, fcl::Transform3f());
  robot_collision_object_ = std::make_shared<fcl::CollisionObjectf>(robot_body_box_object);
  elevated_surfel_octomap_octree_ = std::make_shared<octomap::OcTree>(octomap_voxel_size_);
  original_octomap_octree_ = std::make_shared<octomap::OcTree>(octomap_voxel_size_);
  get_map_client_node_ = std::make_shared<rclcpp::Node>("get_traversability_map_client_node");

  get_traversability_map_client_ =
      get_map_client_node_->create_client<vox_nav_msgs::srv::GetTraversabilityMap>("get_traversability_map");

  octomap_publisher_ = this->create_publisher<octomap_msgs::msg::Octomap>("octomap", rclcpp::SensorDataQoS());
  setupMap();

  // WARN elevated_surfel_poses_msg_ needs to be populated by setupMap();
  state_space_ = std::make_shared<ompl::base::ElevationStateSpace>(
      se2_space_type_, rho_ /*only valid for dubins or reeds*/, false /*only valid for dubins*/);

  state_space_->as<ompl::base::ElevationStateSpace>()->setBounds(*se2_bounds_, *z_bounds_, *v_bounds);
  state_space_->setLongestValidSegmentFraction(0.001);

  control_state_space_ = std::make_shared<ompl::control::RealVectorControlSpace>(state_space_, 2);
  control_state_space_->as<ompl::control::RealVectorControlSpace>()->setBounds(*control_bounds);

  control_simple_setup_ = std::make_shared<ompl::control::SimpleSetup>(control_state_space_);
  control_simple_setup_->setOptimizationObjective(getOptimizationObjective());
  control_simple_setup_->setStateValidityChecker(
      std::bind(&CarControlPlannersBenchMarking::isStateValid, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Selected planners for benchmarking:");
  for (auto&& i : selected_planners_)
  {
    RCLCPP_INFO(this->get_logger(), " %s", i.c_str());
  }

  // Initialize pubs & subs
  plan_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(sample_bencmark_plans_topic_.c_str(),
                                                                                 rclcpp::SystemDefaultsQoS());

  start_goal_poses_publisher_ =
      this->create_publisher<geometry_msgs::msg::PoseArray>("start_goal_poses", rclcpp::SystemDefaultsQoS());
}

CarControlPlannersBenchMarking::~CarControlPlannersBenchMarking()
{
  RCLCPP_INFO(this->get_logger(), "Destroying:");
}

void CarControlPlannersBenchMarking::propagate(const ompl::control::SpaceInformation* si,
                                               const ompl::base::State* start, const ompl::control::Control* control,
                                               const double duration, ompl::base::State* result)
{
  const auto* ee_start = start->as<ompl::base::ElevationStateSpace::StateType>();
  const auto* ee_start_so2 = ee_start->as<ompl::base::SO2StateSpace::StateType>(0);
  const auto* ee_start_xyzv = ee_start->as<ompl::base::RealVectorStateSpace::StateType>(1);
  const double* ctrl = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;

  auto x = ee_start_xyzv->values[0];
  auto y = ee_start_xyzv->values[1];
  auto z = ee_start_xyzv->values[2];
  auto v = ee_start_xyzv->values[3];
  auto yaw = ee_start_so2->value;

  result->as<ompl::base::ElevationStateSpace::StateType>()->setXYZV(x + duration * v * std::cos(yaw) /*X*/,
                                                                    y + duration * v * std::sin(yaw) /*Y*/, z /*Z*/,
                                                                    v + duration * ctrl[0] /*V*/);
  result->as<ompl::base::ElevationStateSpace::StateType>()->setSO2(yaw + duration * ctrl[1] /*W*/);

  si->enforceBounds(result);
}

ompl::base::OptimizationObjectivePtr CarControlPlannersBenchMarking::getOptimizationObjective()
{
  // select a optimizatio objective
  ompl::base::OptimizationObjectivePtr length_objective(
      new ompl::base::PathLengthOptimizationObjective(control_simple_setup_->getSpaceInformation()));
  ompl::base::OptimizationObjectivePtr octocost_objective(new ompl::base::OctoCostOptimizationObjective(
      control_simple_setup_->getSpaceInformation(), elevated_surfel_octomap_octree_));

  ompl::base::MultiOptimizationObjective* multi_optimization =
      new ompl::base::MultiOptimizationObjective(control_simple_setup_->getSpaceInformation());
  multi_optimization->addObjective(length_objective, 1.0);
  multi_optimization->addObjective(octocost_objective, 1.0);

  return ompl::base::OptimizationObjectivePtr(length_objective);
}

std::map<int, ompl::control::PathControl> CarControlPlannersBenchMarking::doBenchMarking()
{
  ompl::base::ScopedState<ompl::base::ElevationStateSpace> random_start(state_space_), random_goal(state_space_);
  geometry_msgs::msg::PoseStamped start, goal;
  std::map<int, ompl::control::PathControl> paths_map;
  auto si = control_simple_setup_->getSpaceInformation();

  si->setMinMaxControlDuration(20, 30);
  si->setPropagationStepSize(0.025);
  control_simple_setup_->setStatePropagator(
      [this, si](const ompl::base::State* state, const ompl::control::Control* control, const double duration,
                 ompl::base::State* result) { this->propagate(si.get(), state, control, duration, result); });
  /*si->setValidStateSamplerAllocator(
      std::bind(
        &CarControlPlannersBenchMarking::allocValidStateSampler, this,
        std::placeholders::_1));*/

  std::stringstream ss;

  for (int i = 0; i < epochs_; i++)
  {
    paths_map.clear();

    // spin until a valid random start and goal poses are found. Also
    // make sure that a soluion exists for generated states
    double start_yaw, goal_yaw, nan;

    start_yaw = getRangedRandom(se_bounds_.minyaw, se_bounds_.maxyaw);
    goal_yaw = getRangedRandom(se_bounds_.minyaw, se_bounds_.maxyaw);

    start.pose.position.x = 7.9;  // getRangedRandom(se_bounds_.minx, se_bounds_.maxx);
    start.pose.position.y = 9.0;  // getRangedRandom(se_bounds_.miny, se_bounds_.maxy);
    start.pose.position.z = 0.5;  // getRangedRandom(se_bounds_.miny, se_bounds_.maxy);
    start.pose.orientation = vox_nav_utilities::getMsgQuaternionfromRPY(nan, nan, 1.57);

    goal.pose.position.x = 7.9;  // getRangedRandom(se_bounds_.minx, se_bounds_.maxx);
    goal.pose.position.y = 2.8;  // getRangedRandom(se_bounds_.miny, se_bounds_.maxy);
    goal.pose.position.z = 0.5;  // getRangedRandom(se_bounds_.miny, se_bounds_.maxy)
    goal.pose.orientation = vox_nav_utilities::getMsgQuaternionfromRPY(nan, nan, 1.57);

    /*vox_nav_utilities::determineValidNearestGoalStart(
      nearest_elevated_surfel_to_start_,
      nearest_elevated_surfel_to_goal_,
      start,
      goal,
      elevated_surfel_cloud_);*/

    nearest_elevated_surfel_to_start_.pose.position = start.pose.position;
    nearest_elevated_surfel_to_goal_.pose.position = goal.pose.position;
    nearest_elevated_surfel_to_start_.pose.orientation = start.pose.orientation;
    nearest_elevated_surfel_to_goal_.pose.orientation = goal.pose.orientation;

    random_start->setXYZV(nearest_elevated_surfel_to_start_.pose.position.x,
                          nearest_elevated_surfel_to_start_.pose.position.y,
                          nearest_elevated_surfel_to_start_.pose.position.z, 0);
    random_start->setSO2(start_yaw);

    random_goal->setXYZV(nearest_elevated_surfel_to_goal_.pose.position.x,
                         nearest_elevated_surfel_to_goal_.pose.position.y,
                         nearest_elevated_surfel_to_goal_.pose.position.z, 0);
    random_goal->setSO2(goal_yaw);

    start_and_goal_poses_.poses.clear();
    start_and_goal_poses_.poses.push_back(start.pose);
    start_and_goal_poses_.poses.push_back(goal.pose);

    control_simple_setup_->setStartAndGoalStates(random_start, random_goal, goal_tolerance_);

    std::mutex plan_mutex;

    int index(0);
    for (auto&& planner_name : selected_planners_)
    {
      ompl::base::PlannerPtr planner_ptr;
      initializeSelectedControlPlanner(planner_ptr, planner_name, si, logger_);

      if (publish_a_sample_bencmark_)
      {
        std::lock_guard<std::mutex> guard(plan_mutex);

        RCLCPP_INFO(this->get_logger(), "Creating sample plans.");

        control_simple_setup_->setPlanner(planner_ptr);
        control_simple_setup_->setup();
        // control_simple_setup_->print(std::cout);
        ompl::base::PlannerStatus solved = control_simple_setup_->solve(planner_timeout_);
        ompl::control::PathControl solution_path(si);
        try
        {
          solution_path = control_simple_setup_->getSolutionPath();
        }
        catch (const std::exception& e)
        {
          std::cerr << e.what() << '\n';
          RCLCPP_WARN(logger_, "Exception occured while retrivieng control solution path %s", e.what());
          control_simple_setup_->clear();
        }

        ss << planner_name.c_str() << " " << solved << " " << solution_path.length() << "\n";
        std::pair<int, ompl::control::PathControl> curr_pair(index, solution_path);
        if (solved == ompl::base::PlannerStatus::EXACT_SOLUTION ||
            solved == ompl::base::PlannerStatus::APPROXIMATE_SOLUTION || solved == ompl::base::PlannerStatus::TIMEOUT)
        {
          paths_map.insert(curr_pair);
        }
        control_simple_setup_->clear();
      }
      index++;
    }

    std::cout << ss.str() << std::endl;

    control_simple_setup_->clear();

    publishSamplePlans(paths_map);
  }
  return paths_map;
}

bool CarControlPlannersBenchMarking::isStateValid(const ompl::base::State* state)
{
  const auto* cstate = state->as<ompl::base::ElevationStateSpace::StateType>();
  // cast the abstract state type to the type we expect
  const auto* so2 = cstate->as<ompl::base::SO2StateSpace::StateType>(0);
  // extract the second component of the state and cast it to what we expect
  const auto* xyzv = cstate->as<ompl::base::RealVectorStateSpace::StateType>(1);
  fcl::CollisionRequestf requestType(1, false, 1, false);
  // check validity of state Fdefined by pos & rot
  fcl::Vector3f translation(xyzv->values[0], xyzv->values[1], xyzv->values[2]);
  tf2::Quaternion myQuaternion;
  myQuaternion.setRPY(0, 0, so2->value);
  fcl::Quaternionf rotation(myQuaternion.getX(), myQuaternion.getY(), myQuaternion.getZ(), myQuaternion.getW());

  robot_collision_object_->setTransform(rotation, translation);

  fcl::CollisionResultf collisionWithFullMapResult;

  fcl::collide<float>(robot_collision_object_.get(), original_octomap_collision_object_.get(), requestType,
                      collisionWithFullMapResult);

  return !collisionWithFullMapResult.isCollision();
}

void CarControlPlannersBenchMarking::publishSamplePlans(std::map<int, ompl::control::PathControl> sample_paths)
{
  if (!publish_a_sample_bencmark_)
  {
    RCLCPP_INFO(this->get_logger(), "Will not publish sample plans.");
  }
  visualization_msgs::msg::MarkerArray marker_array;
  int total_poses = 0;

  auto it = sample_paths.begin();
  while (it != sample_paths.end())
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = rclcpp::Clock().now();

    if (!robot_mesh_path_.empty())
    {
      marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
      marker.mesh_resource = robot_mesh_path_;
    }
    else
    {
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    }
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.lifetime = rclcpp::Duration::from_seconds(0);
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.id = total_poses;
    marker.color = getColorByIndex(it->first);
    marker.ns = "path" + std::to_string(it->first);

    for (std::size_t curr_path_state = 0; curr_path_state < it->second.getStateCount(); curr_path_state++)
    {
      const auto* cstate = it->second.getState(curr_path_state)->as<ompl::base::ElevationStateSpace::StateType>();
      const auto* so2 = cstate->as<ompl::base::SO2StateSpace::StateType>(0);
      const auto* xyzv = cstate->as<ompl::base::RealVectorStateSpace::StateType>(1);
      geometry_msgs::msg::Point p;
      p.x = xyzv->values[0];
      p.y = xyzv->values[1];
      p.z = xyzv->values[2];
      marker.points.push_back(p);
      marker.colors.push_back(getColorByIndex(it->first));
    }
    total_poses++;
    start_and_goal_poses_.header = marker.header;
    marker_array.markers.push_back(marker);
    it++;
  }

  start_goal_poses_publisher_->publish(start_and_goal_poses_);
  plan_publisher_->publish(marker_array);

  // Publish the octomap
  octomap_msgs::msg::Octomap octomap_msg;
  octomap_msgs::fullMapToMsg(*original_octomap_octree_, octomap_msg);
  octomap_msg.header.frame_id = "map";
  octomap_msg.header.stamp = rclcpp::Clock().now();
  octomap_publisher_->publish(octomap_msg);
}

void CarControlPlannersBenchMarking::setupMap()
{
  const std::lock_guard<std::mutex> lock(octomap_mutex_);

  if (!octomap_from_file_.empty())
  {
    // Read Octomap from file

    octomap::OcTree temp_tree(0.2);
    temp_tree.readBinary(octomap_from_file_);

    original_octomap_octree_ = std::make_shared<octomap::OcTree>(temp_tree);

    auto original_octomap_fcl_octree = std::make_shared<fcl::OcTreef>(original_octomap_octree_);
    original_octomap_collision_object_ =
        std::make_shared<fcl::CollisionObjectf>(std::shared_ptr<fcl::CollisionGeometryf>(original_octomap_fcl_octree));

    RCLCPP_INFO(this->get_logger(), "Read Octomap from file");

    // Publish the octomap
    octomap_msgs::msg::Octomap octomap_msg;
    octomap_msgs::fullMapToMsg(*original_octomap_octree_, octomap_msg);
    octomap_msg.header.frame_id = "map";
    octomap_msg.header.stamp = rclcpp::Clock().now();

    for (int i = 0; i < 100; i++)
    {
      octomap_publisher_->publish(octomap_msg);
    }

    RCLCPP_INFO(logger_,
                "Recieved a valid Octomap with %d nodes, A FCL collision tree will be created from this "
                "octomap for state validity (aka collision check)",
                original_octomap_octree_->size());

    is_map_ready_ = true;
  }
  else
  {
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

      auto elevated_surfel_octomap_octree =
          dynamic_cast<octomap::OcTree*>(octomap_msgs::fullMsgToMap(response->elevated_surfel_octomap));
      elevated_surfel_octomap_octree_ = std::make_shared<octomap::OcTree>(*elevated_surfel_octomap_octree);

      delete original_octomap_octree;
      delete elevated_surfel_octomap_octree;

      auto elevated_surfels_fcl_octree = std::make_shared<fcl::OcTreef>(elevated_surfel_octomap_octree_);
      elevated_surfels_collision_object_ = std::make_shared<fcl::CollisionObjectf>(
          std::shared_ptr<fcl::CollisionGeometryf>(elevated_surfels_fcl_octree));

      auto original_octomap_fcl_octree = std::make_shared<fcl::OcTreef>(original_octomap_octree_);
      original_octomap_collision_object_ = std::make_shared<fcl::CollisionObjectf>(
          std::shared_ptr<fcl::CollisionGeometryf>(original_octomap_fcl_octree));

      elevated_surfel_poses_msg_ = std::make_shared<geometry_msgs::msg::PoseArray>(response->elevated_surfel_poses);
      for (auto&& i : elevated_surfel_poses_msg_->poses)
      {
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

      RCLCPP_INFO(logger_,
                  "Recieved a valid Octomap with %d nodes, A FCL collision tree will be created from this "
                  "octomap for state validity (aka collision check)",
                  original_octomap_octree_->size());

      RCLCPP_INFO(logger_,
                  "Recieved a valid Octomap which represents Elevated surfels with %d nodes,"
                  " A FCL collision tree will be created from this "
                  "octomap for state validity (aka collision check)",
                  elevated_surfel_octomap_octree_->size());
    }
  }
}

ompl::base::ValidStateSamplerPtr
CarControlPlannersBenchMarking::allocValidStateSampler(const ompl::base::SpaceInformation* si)
{
  auto valid_sampler = std::make_shared<ompl::base::OctoCellValidStateSampler>(
      control_simple_setup_->getSpaceInformation(), nearest_elevated_surfel_to_start_, nearest_elevated_surfel_to_goal_,
      elevated_surfel_poses_msg_);
  return valid_sampler;
}

std_msgs::msg::ColorRGBA CarControlPlannersBenchMarking::getColorByIndex(int index)
{
  std_msgs::msg::ColorRGBA result;
  switch (index)
  {
    case 0:  // RED:
      result.r = 0.8;
      result.g = 0.1;
      result.b = 0.1;
      result.a = 1.0;
      break;
    case 1:  // GREEN:
      result.r = 0.1;
      result.g = 0.8;
      result.b = 0.1;
      result.a = 1.0;
      break;
    case 2:  // GREY:
      result.r = 0.9;
      result.g = 0.9;
      result.b = 0.9;
      result.a = 1.0;
      break;
    case 3:  // CYAN:
      result.r = 0.0;
      result.g = 1.0;
      result.b = 1.0;
      result.a = 1.0;
      break;
    case 4:  // YELLOW:
      result.r = 1.0;
      result.g = 1.0;
      result.b = 0.0;
      result.a = 1.0;
      break;
    case 5:  // ORANGE:
      result.r = 1.0;
      result.g = 0.5;
      result.b = 0.0;
      result.a = 1.0;
      break;
    case 6:  // Maroon:
      result.r = 0.5;
      result.g = 0.0;
      result.b = 0.0;
      result.a = 1.0;
      break;
    case 7:  // Olive:
      result.r = 0.5;
      result.g = 0.5;
      result.b = 0.0;
      result.a = 1.0;
      break;
    case 8:  // Navy:
      result.r = 0.0;
      result.g = 0.0;
      result.b = 0.5;
      result.a = 1.0;
      break;
    case 9:  // BLACK:
      result.r = 0.0;
      result.g = 0.0;
      result.b = 0.0;
      result.a = 1.0;
      break;
    case 10:  // YELLOW:
      result.r = 1.0;
      result.g = 1.0;
      result.b = 0.0;
      result.a = 1.0;
      break;
    case 11:  // BROWN:
      result.r = 0.597;
      result.g = 0.296;
      result.b = 0.0;
      result.a = 1.0;
      break;
    case 12:  // PINK:
      result.r = 1.0;
      result.g = 0.4;
      result.b = 1;
      result.a = 1.0;
      break;
    case 13:  // LIME_GREEN:
      result.r = 0.6;
      result.g = 1.0;
      result.b = 0.2;
      result.a = 1.0;
      break;
    case 14:  // PURPLE:
      result.r = 0.597;
      result.g = 0.0;
      result.b = 0.597;
      result.a = 1.0;
      break;
    case 15:  // CYAN:
      result.r = 0.0;
      result.g = 1.0;
      result.b = 1.0;
      result.a = 1.0;
      break;
    case 16:  // MAGENTA:
      result.r = 1.0;
      result.g = 0.0;
      result.b = 1.0;
      result.a = 1.0;
  }
  return result;
}

double CarControlPlannersBenchMarking::getRangedRandom(double min, double max)
{
  std::random_device rd;
  std::default_random_engine eng(rd());
  std::uniform_real_distribution<double> distr(min, max);

  return distr(eng);
}

}  // namespace vox_nav_planning

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vox_nav_planning::CarControlPlannersBenchMarking>();
  while (rclcpp::ok() && !node->is_map_ready_)
  {
    rclcpp::spin_some(node->get_node_base_interface());
    RCLCPP_INFO(node->get_logger(),
                "Waiting for octomap to be ready In order "
                "to run planner bencmarking... ");
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  RCLCPP_INFO(node->get_logger(), "Octomap ready, running bencmark with given configurations");
  auto paths_map = node->doBenchMarking();
  while (rclcpp::ok())
  {
    node->publishSamplePlans(paths_map);
    rclcpp::spin_some(node->get_node_base_interface());
    RCLCPP_INFO(node->get_logger(), "publishing planner bencmarking... press CTRL+C to stop");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  RCLCPP_INFO(node->get_logger(), "Benchmarking done , exiting successfully");
  rclcpp::shutdown();
  return 0;
}
