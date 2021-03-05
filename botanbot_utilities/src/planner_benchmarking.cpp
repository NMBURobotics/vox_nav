#include "botanbot_utilities/planner_benchmarking.hpp"

namespace botanbot_utilities
{
PlannerBenchMarking::PlannerBenchMarking()
: Node("planner_benchmarking_rclcpp_node")
{
  RCLCPP_INFO(this->get_logger(), "Creating:");

  is_octomap_ready_ = false;
  octomap_msg_ = std::make_shared<octomap_msgs::msg::Octomap>();

  this->declare_parameter("selected_planners", std::vector<std::string>({"RRTstar", "PRMstar"}));
  this->declare_parameter("planner_timeout", 5.0);
  this->declare_parameter("interpolation_parameter", 50);
  this->declare_parameter("octomap_topic", "octomap");
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
  this->declare_parameter("robot_body_dimens.x", 1.5);
  this->declare_parameter("robot_body_dimens.y", 1.5);
  this->declare_parameter("robot_body_dimens.z", 0.4);
  this->declare_parameter("start.z", 0.0);
  this->declare_parameter("goal.z", 0.0);
  this->declare_parameter("goal_tolerance", 0.2);
  this->declare_parameter("min_euclidean_dist_start_to_goal", 25.0);
  this->declare_parameter("num_benchmark_runs", 100);
  this->declare_parameter("max_memory", 2048);
  this->declare_parameter("results_output_file", "/home/user/get.log");
  this->declare_parameter("publish_a_sample_bencmark", true);
  this->declare_parameter("sample_bencmark_plans_topic", "benchmark_plan");

  this->get_parameter("selected_planners", selected_planners_);
  this->get_parameter("planner_timeout", planner_timeout_);
  this->get_parameter("interpolation_parameter", interpolation_parameter_);
  this->get_parameter("octomap_topic", octomap_topic_);
  this->get_parameter("octomap_voxel_size", octomap_voxel_size_);
  this->get_parameter("selected_state_space", selected_state_space_);
  this->get_parameter("min_turning_radius", min_turning_radius_);
  this->get_parameter("state_space_boundries.minx", se_bounds_.minx);
  this->get_parameter("state_space_boundries.maxx", se_bounds_.maxx);
  this->get_parameter("state_space_boundries.miny", se_bounds_.miny);
  this->get_parameter("state_space_boundries.maxy", se_bounds_.maxy);
  this->get_parameter("state_space_boundries.minz", se_bounds_.minz);
  this->get_parameter("state_space_boundries.maxz", se_bounds_.maxz);
  this->get_parameter("state_space_boundries.minyaw", se_bounds_.minyaw);
  this->get_parameter("state_space_boundries.maxyaw", se_bounds_.maxyaw);
  this->get_parameter("robot_body_dimens.x", robot_body_dimensions_.x);
  this->get_parameter("robot_body_dimens.y", robot_body_dimensions_.y);
  this->get_parameter("robot_body_dimens.z", robot_body_dimensions_.z);
  this->get_parameter("start.z", start_.z);
  this->get_parameter("goal.z", goal_.z);
  this->get_parameter("goal_tolerance", goal_tolerance_);
  this->get_parameter("min_euclidean_dist_start_to_goal", min_euclidean_dist_start_to_goal_);
  this->get_parameter("num_benchmark_runs", num_benchmark_runs_);
  this->get_parameter("max_memory", max_memory_);
  this->get_parameter("results_output_file", results_output_file_);
  this->get_parameter("publish_a_sample_bencmark", publish_a_sample_bencmark_);
  this->get_parameter("sample_bencmark_plans_topic", sample_bencmark_plans_topic_);

  if (selected_state_space_ == "REEDS") {
    ompl_se_bounds_ = std::make_shared<ompl::base::RealVectorBounds>(2);
    ompl_se_bounds_->setLow(0, se_bounds_.minx);
    ompl_se_bounds_->setHigh(0, se_bounds_.maxx);
    ompl_se_bounds_->setLow(1, se_bounds_.miny);
    ompl_se_bounds_->setHigh(1, se_bounds_.maxy);
    ompl_se_bounds_->setLow(2, se_bounds_.minyaw);
    ompl_se_bounds_->setHigh(2, se_bounds_.maxyaw);
    state_space_ = std::make_shared<ompl::base::ReedsSheppStateSpace>(min_turning_radius_);
    state_space_->as<ompl::base::ReedsSheppStateSpace>()->setBounds(*ompl_se_bounds_);
  } else if (selected_state_space_ == "DUBINS") {
    ompl_se_bounds_ = std::make_shared<ompl::base::RealVectorBounds>(2);
    ompl_se_bounds_->setLow(0, se_bounds_.minx);
    ompl_se_bounds_->setHigh(0, se_bounds_.maxx);
    ompl_se_bounds_->setLow(1, se_bounds_.miny);
    ompl_se_bounds_->setHigh(1, se_bounds_.maxy);
    ompl_se_bounds_->setLow(2, se_bounds_.minyaw);
    ompl_se_bounds_->setHigh(2, se_bounds_.maxyaw);
    state_space_ = std::make_shared<ompl::base::DubinsStateSpace>(min_turning_radius_, false);
    state_space_->as<ompl::base::DubinsStateSpace>()->setBounds(*ompl_se_bounds_);
  } else if (selected_state_space_ == "SE2") {
    ompl_se_bounds_ = std::make_shared<ompl::base::RealVectorBounds>(2);
    ompl_se_bounds_->setLow(0, se_bounds_.minx);
    ompl_se_bounds_->setHigh(0, se_bounds_.maxx);
    ompl_se_bounds_->setLow(1, se_bounds_.miny);
    ompl_se_bounds_->setHigh(1, se_bounds_.maxy);
    ompl_se_bounds_->setLow(2, se_bounds_.minyaw);
    ompl_se_bounds_->setHigh(2, se_bounds_.maxyaw);
    state_space_ = std::make_shared<ompl::base::SE2StateSpace>();
    state_space_->as<ompl::base::SE2StateSpace>()->setBounds(*ompl_se_bounds_);
  } else {
    ompl_se_bounds_ = std::make_shared<ompl::base::RealVectorBounds>(3);
    ompl_se_bounds_->setLow(0, se_bounds_.minx);
    ompl_se_bounds_->setHigh(0, se_bounds_.maxx);
    ompl_se_bounds_->setLow(1, se_bounds_.miny);
    ompl_se_bounds_->setHigh(1, se_bounds_.maxy);
    ompl_se_bounds_->setLow(2, se_bounds_.minz);
    ompl_se_bounds_->setHigh(2, se_bounds_.maxz);
    state_space_ = std::make_shared<ompl::base::SE3StateSpace>();
    state_space_->as<ompl::base::SE3StateSpace>()->setBounds(*ompl_se_bounds_);
  }

  typedef std::shared_ptr<fcl::CollisionGeometry> CollisionGeometryPtr_t;
  CollisionGeometryPtr_t robot_body_box(
    new fcl::Box(
      robot_body_dimensions_.x,
      robot_body_dimensions_.y,
      robot_body_dimensions_.z));
  fcl::Transform3f tf2;
  fcl::CollisionObject robot_body_box_object(robot_body_box, tf2);
  robot_collision_object_ = std::make_shared<fcl::CollisionObject>(robot_body_box_object);
  octomap_subscriber_ = this->create_subscription<octomap_msgs::msg::Octomap>(
    octomap_topic_, rclcpp::SystemDefaultsQoS(),
    std::bind(&PlannerBenchMarking::octomapCallback, this, std::placeholders::_1));

  // Initialize pubs & subs
  plan_publisher_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>(
    sample_bencmark_plans_topic_.c_str(), rclcpp::SystemDefaultsQoS());

  start_goal_poses_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
    "start_goal_poses", rclcpp::SystemDefaultsQoS());

  RCLCPP_INFO(this->get_logger(), "Selected planners for benchmarking:");
  for (auto && i : selected_planners_) {
    RCLCPP_INFO(this->get_logger(), " %s", i.c_str());
  }
}

PlannerBenchMarking::~PlannerBenchMarking()
{
  RCLCPP_INFO(this->get_logger(), "Destroying:");
}

std::map<int, ompl::geometric::PathGeometric> PlannerBenchMarking::doBenchMarking()
{
  ompl::geometric::SimpleSetup ss(state_space_);
  ompl::base::SpaceInformationPtr si = ss.getSpaceInformation();
  ompl::base::ScopedState<ompl::base::SE2StateSpace> random_start(state_space_),
  random_goal(state_space_);

  // spin until a valid random start and goal poses are found. Also
  // make sure that a soluion exists for generated states
  volatile bool found_valid_random_start_goal = false;
  while (!found_valid_random_start_goal) {
    random_start->setX(getRangedRandom(se_bounds_.minx, se_bounds_.maxx));
    random_start->setY(getRangedRandom(se_bounds_.miny, se_bounds_.maxy));
    random_start->setYaw(getRangedRandom(se_bounds_.minyaw, se_bounds_.maxyaw));

    random_goal->setX(getRangedRandom(se_bounds_.minx, se_bounds_.maxx));
    random_goal->setY(getRangedRandom(se_bounds_.miny, se_bounds_.maxy));
    random_goal->setYaw(getRangedRandom(se_bounds_.minyaw, se_bounds_.maxyaw));

    // the distance should be above a certain threshold
    double distance =
      std::sqrt(
      std::pow(random_goal->getX() - random_start->getX(), 2) +
      std::pow(random_goal->getY() - random_start->getY(), 2));

    // define start & goal states
    if ((selected_state_space_ == "REEDS") || (selected_state_space_ == "DUBINS") ||
      (selected_state_space_ == "SE2"))
    {
      ompl::base::ScopedState<ompl::base::SE2StateSpace> start(state_space_), goal(state_space_);
      start->setXY(random_start->getX(), random_start->getY());
      start->setYaw(random_start->getYaw());
      goal->setXY(random_goal->getX(), random_goal->getY());
      goal->setYaw(random_goal->getYaw());
      ss.setStartAndGoalStates(start, goal, goal_tolerance_);
      ss.setStateValidityChecker(
        [this](const ompl::base::State * state)
        {
          return isStateValidSE2(state);
        });
    } else {
      ompl::base::ScopedState<ompl::base::SE3StateSpace> start(state_space_), goal(state_space_);
      start->setXYZ(random_start->getX(), random_start->getY(), start_.z);
      start->as<ompl::base::SO3StateSpace::StateType>(1)->setAxisAngle(
        0, 0, 1, random_start->getYaw());
      goal->setXYZ(random_goal->getX(), random_goal->getY(), goal_.z);
      goal->as<ompl::base::SO3StateSpace::StateType>(1)->setAxisAngle(
        0, 0, 1, random_goal->getYaw());
      ss.setStartAndGoalStates(start, goal, goal_tolerance_);
      ss.setStateValidityChecker(
        [this](const ompl::base::State * state)
        {
          return isStateValidSE3(state);
        });
    }

    found_valid_random_start_goal =
      (isStateValidSE2(random_start.get()) && isStateValidSE2(random_goal.get()) &&
      distance > min_euclidean_dist_start_to_goal_);

    if (!found_valid_random_start_goal) {
      RCLCPP_INFO(
        this->get_logger(), "Still Looking to sample valid random start and goal states ... ");
      continue;
    }
    RCLCPP_INFO(
      this->get_logger(), "A valid random start and goal states has been found.");

    ss.setOptimizationObjective(std::make_shared<ompl::base::PathLengthOptimizationObjective>(si));
    si->setStateValidityCheckingResolution(1.0 / state_space_->getMaximumExtent());
    si->setup();

    // create a planner for the defined space
    ompl::base::PlannerPtr bitstar_planner;
    bitstar_planner = ompl::base::PlannerPtr(new ompl::geometric::BITstar(si));
    ss.setPlanner(bitstar_planner);

    RCLCPP_INFO(
      this->get_logger(),
      "Checking whether a solution exists for random start and goal states .");
    ompl::base::PlannerStatus has_solution = ss.solve(5.0);

    // if it gets to this point , that menas our random states are valid and already meets min dist requiremnets
    // but now there also has to be a solution for this problem
    found_valid_random_start_goal = has_solution;

    if (found_valid_random_start_goal) {
      RCLCPP_INFO(
        this->get_logger(),
        "Found valid states and a solution for the random problem!, proceeding to actual benchmark.");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  ss.clear();
  RCLCPP_INFO(this->get_logger(), "Created valid random start and goal states");

  start_.x = random_start->getX();
  start_.y = random_start->getY();
  start_.yaw = random_start->getYaw();
  goal_.x = random_goal->getX();
  goal_.y = random_goal->getY();
  goal_.yaw = random_goal->getYaw();

  // create a planner for the defined space
  ompl::base::PlannerPtr cforest_planner;
  cforest_planner = ompl::base::PlannerPtr(new ompl::geometric::CForest(si));
  ss.setPlanner(cforest_planner);
  RCLCPP_INFO(this->get_logger(), "Creating Ground truth plan wih CFOREST");
  ompl::base::PlannerStatus gt_plan_solved = ss.solve(10.0);
  ompl::geometric::PathGeometric gt_plan = ss.getSolutionPath();
  ss.clear();
  std::map<int, ompl::geometric::PathGeometric> paths_map;
  std::mutex plan_mutex;
  ompl::tools::Benchmark::Request request(planner_timeout_, max_memory_, num_benchmark_runs_);
  request.displayProgress = false;
  ompl::tools::Benchmark b(ss, "outdoor_plan_benchmarking");
  b.addExperimentParameter("gt_path_length", "REAL", std::to_string(gt_plan.length()));

  // this section is to visualize a sample benchmark into RVIZ
  int index(0);
  for (auto && planner_name : selected_planners_) {
    ompl::base::PlannerPtr planner_ptr;
    allocatePlannerbyName(planner_ptr, planner_name, si);
    b.addPlanner(planner_ptr);

    if (publish_a_sample_bencmark_) {
      try {
        std::lock_guard<std::mutex> guard(plan_mutex);
        ompl::geometric::PathGeometric curr_path = makeAPlan(planner_ptr, ss);
        if (curr_path.getStateCount() == 0) {
          RCLCPP_WARN(
            this->get_logger(),
            "An empty path detected!, looks like %s failed to produce a valid plan",
            planner_name.c_str());
        }

        std::pair<int, ompl::geometric::PathGeometric> curr_pair(index, curr_path);
        paths_map.insert(curr_pair);

        ss.clear();
      } catch (const std::exception & e) {
        std::cerr << e.what() << '\n';
      }
    }

    index++;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Created sample plans from each planner, "
    "Now performing actual benchmark, This might take some time.");
  b.benchmark(request);
  b.saveResultsToFile(results_output_file_.c_str());
  RCLCPP_INFO(
    this->get_logger(),
    "Bencmarking results saved to given directory: %s.", results_output_file_.c_str());
  return paths_map;
}

bool PlannerBenchMarking::isStateValidSE2(const ompl::base::State * state)
{
  if (is_octomap_ready_) {
    std::call_once(
      fcl_tree_from_octomap_once_, [this]() {
        std::shared_ptr<octomap::OcTree> octomap_octree =
        std::make_shared<octomap::OcTree>(octomap_voxel_size_);
        octomap_msgs::readTree<octomap::OcTree>(octomap_octree.get(), *octomap_msg_);
        fcl_octree_ = std::make_shared<fcl::OcTree>(octomap_octree);
        fcl_octree_collision_object_ = std::make_shared<fcl::CollisionObject>(
          std::shared_ptr<fcl::CollisionGeometry>(fcl_octree_));
        RCLCPP_INFO(
          this->get_logger(),
          "Recieved a valid Octomap, A FCL collision tree will be created from this "
          "octomap for state validity(aka collision check)");
      });
  } else {
    RCLCPP_ERROR(
      this->get_logger(),
      "The Octomap has not been recieved correctly, Collision check "
      "cannot be processed without a valid Octomap!");
    return false;
  }
  // cast the abstract state type to the type we expect
  const ompl::base::SE2StateSpace::StateType * se2_state =
    state->as<ompl::base::SE2StateSpace::StateType>();
  // check validity of state Fdefined by pos & rot
  fcl::Vec3f translation(se2_state->getX(), se2_state->getY(), start_.z);

  tf2::Quaternion myQuaternion;
  myQuaternion.setRPY(0, 0, se2_state->getYaw());
  fcl::Quaternion3f rotation(
    myQuaternion.getX(), myQuaternion.getY(),
    myQuaternion.getZ(), myQuaternion.getW());
  robot_collision_object_->setTransform(rotation, translation);
  fcl::CollisionRequest requestType(1, false, 1, false);
  fcl::CollisionResult collisionResult;
  fcl::collide(
    robot_collision_object_.get(),
    fcl_octree_collision_object_.get(), requestType, collisionResult);
  return !collisionResult.isCollision();
}

bool PlannerBenchMarking::isStateValidSE3(const ompl::base::State * state)
{
  if (is_octomap_ready_) {
    std::call_once(
      fcl_tree_from_octomap_once_, [this]() {
        std::shared_ptr<octomap::OcTree> octomap_octree =
        std::make_shared<octomap::OcTree>(octomap_voxel_size_);
        octomap_msgs::readTree<octomap::OcTree>(octomap_octree.get(), *octomap_msg_);


        fcl_octree_ = std::make_shared<fcl::OcTree>(octomap_octree);
        fcl_octree_collision_object_ = std::make_shared<fcl::CollisionObject>(
          std::shared_ptr<fcl::CollisionGeometry>(fcl_octree_));
        RCLCPP_INFO(
          this->get_logger(),
          "Recieved a valid Octomap, A FCL collision tree will be created from this "
          "octomap for state validity(aka collision check)");
      });
  } else {
    RCLCPP_ERROR(
      this->get_logger(),
      "The Octomap has not been recieved correctly, Collision check "
      "cannot be processed without a valid Octomap!");
    return false;
  }
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
}

void PlannerBenchMarking::octomapCallback(
  const octomap_msgs::msg::Octomap::ConstSharedPtr msg)
{
  const std::lock_guard<std::mutex> lock(octomap_mutex_);
  if (!is_octomap_ready_) {
    is_octomap_ready_ = true;
    octomap_msg_ = msg;
  }
}

ompl::geometric::PathGeometric PlannerBenchMarking::makeAPlan(
  const ompl::base::PlannerPtr & planner,
  ompl::geometric::SimpleSetup & ss)
{
  ss.setPlanner(planner);
  // attempt to solve the problem within one second of planning time
  ompl::base::PlannerStatus solved = ss.solve(planner_timeout_);

  if (!solved) {
    ompl::geometric::PathGeometric empty_path = ss.getSolutionPath();
    empty_path.clear();
    return empty_path;
  }

  ompl::geometric::PathGeometric original_path = ss.getSolutionPath();
  ompl::geometric::PathGeometric copy_path = ss.getSolutionPath();

  // Path smoothing using bspline
  ompl::geometric::PathSimplifier path_simlifier(ss.getSpaceInformation());
  path_simlifier.simplifyMax(original_path);
  path_simlifier.smoothBSpline(original_path);
  original_path.interpolate(interpolation_parameter_);
  return original_path;
}

void PlannerBenchMarking::publishSamplePlans(
  std::map<int, ompl::geometric::PathGeometric> sample_paths)
{
  visualization_msgs::msg::MarkerArray marker_array;
  int total_poses = 0;

  auto it = sample_paths.begin();
  while (it != sample_paths.end()) {
    for (std::size_t curr_path_state = 0; curr_path_state < it->second.getStateCount();
      curr_path_state++)
    {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = rclcpp::Clock().now();
      marker.type = visualization_msgs::msg::Marker::ARROW;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.lifetime = rclcpp::Duration::from_seconds(0);
      marker.scale.x = 0.4;
      marker.scale.y = 0.3;
      marker.scale.z = 0.3;
      marker.id = total_poses;
      marker.color = getColorByIndex(it->first);
      marker.ns = "path" + std::to_string(it->first);
      if (selected_state_space_ == "SE3") {
        auto se3_state =
          it->second.getState(curr_path_state)->as<ompl::base::SE3StateSpace::StateType>();
        geometry_msgs::msg::Point p;
        p.x = se3_state->getX();
        p.y = se3_state->getY();
        p.z = se3_state->getZ();
        marker.pose.position = p;
        marker.pose.orientation.x = se3_state->rotation().x;
        marker.pose.orientation.y = se3_state->rotation().y;
        marker.pose.orientation.z = se3_state->rotation().z;
        marker.pose.orientation.w = se3_state->rotation().w;
      } else {
        auto se2_state =
          it->second.getState(curr_path_state)->as<ompl::base::SE2StateSpace::StateType>();
        geometry_msgs::msg::Point p;
        p.x = se2_state->getX();
        p.y = se2_state->getY();
        p.z = start_.z;
        marker.pose.position = p;
        marker.pose.orientation =
          botanbot_utilities::getMsgQuaternionfromRPY(0, 0, se2_state->getYaw());
      }
      marker_array.markers.push_back(marker);
      total_poses++;
    }
    it++;
  }
  plan_publisher_->publish(marker_array);
  geometry_msgs::msg::PoseArray start_and_goal;
  start_and_goal.header = marker_array.markers.front().header;
  geometry_msgs::msg::Pose start, goal;
  start.position.x = start_.x;
  start.position.y = start_.y;
  start.position.z = start_.z;
  start.orientation = botanbot_utilities::getMsgQuaternionfromRPY(0, 0, start_.yaw);
  goal.position.x = goal_.x;
  goal.position.y = goal_.y;
  goal.position.z = start_.z;
  goal.orientation = botanbot_utilities::getMsgQuaternionfromRPY(0, 0, goal_.yaw);
  start_and_goal.poses.push_back(start);
  start_and_goal.poses.push_back(goal);
  start_goal_poses_publisher_->publish(start_and_goal);
}

std_msgs::msg::ColorRGBA PlannerBenchMarking::getColorByIndex(int index)
{
  std_msgs::msg::ColorRGBA result;
  switch (index) {
    case 0:   // RED
      result.r = 0.8;
      result.g = 0.1;
      result.b = 0.1;
      result.a = 1.0;
      break;
    case 1:   //GREEN
      result.r = 0.1;
      result.g = 0.8;
      result.b = 0.1;
      result.a = 1.0;
      break;
    case 2:   //BLUE
      result.r = 0.0;
      result.g = 0.0;
      result.b = 1.0;
      result.a = 1.0;
      break;
    case 3:   //WHITE
      result.r = 1.0;
      result.g = 1.0;
      result.b = 1.0;
      result.a = 1.0;
      break;
    case 4:   //YELLOW
      result.r = 1.0;
      result.g = 1.0;
      result.b = 0.0;
      result.a = 1.0;
      break;
    case 5:   //MAGENTA
      result.r = 1.0;
      result.g = 0.0;
      result.b = 1.0;
      result.a = 1.0;
      break;
    case 6:   //BLACK
      result.r = 0.0;
      result.g = 0.0;
      result.b = 0.0;
      result.a = 1.0;
      break;
    case 7:   //CYAN
      result.r = 0.0;
      result.g = 1.0;
      result.b = 1.0;
      result.a = 1.0;
      break;
    case 8:   //PINK
      result.r = 1.0;
      result.g = 0.4;
      result.b = 1;
      result.a = 1.0;
      break;
    case 9:   //PURPLE
      result.r = 0.597;
      result.g = 0.0;
      result.b = 0.597;
      result.a = 1.0;
      break;
  }
  return result;
}

void PlannerBenchMarking::allocatePlannerbyName(
  ompl::base::PlannerPtr & planner,
  const std::string & selected_planner_name,
  const ompl::base::SpaceInformationPtr & si)
{
  if (selected_planner_name == std::string("RRTstar")) {
    planner = ompl::base::PlannerPtr(new ompl::geometric::RRTstar(si));
  } else if (selected_planner_name == std::string("PRMstar")) {
    planner = ompl::base::PlannerPtr(new ompl::geometric::PRMstar(si));
  } else if (selected_planner_name == std::string("LazyPRMstar")) {
    planner = ompl::base::PlannerPtr(new ompl::geometric::LazyPRMstar(si));
  } else if (selected_planner_name == std::string("RRTXstatic")) {
    planner = ompl::base::PlannerPtr(new ompl::geometric::RRTXstatic(si));
  } else if (selected_planner_name == std::string("FMT")) {
    planner = ompl::base::PlannerPtr(new ompl::geometric::FMT(si));
  } else if (selected_planner_name == std::string("BITstar")) {
    planner = ompl::base::PlannerPtr(new ompl::geometric::BITstar(si));
  } else if (selected_planner_name == std::string("ABITstar")) {
    planner = ompl::base::PlannerPtr(new ompl::geometric::ABITstar(si));
  } else if (selected_planner_name == std::string("CForest")) {
    planner = ompl::base::PlannerPtr(new ompl::geometric::CForest(si));
  } else {
    RCLCPP_WARN(
      this->get_logger(),
      "Selected planner is not Found in available planners, using the default planner: RRTstar");
    planner = ompl::base::PlannerPtr(new ompl::geometric::RRTstar(si));
  }
}

double PlannerBenchMarking::getRangedRandom(double min, double max)
{
  std::random_device rd;
  std::default_random_engine eng(rd());
  std::uniform_real_distribution<double> distr(min, max);

  return distr(eng);
}

}  // namespace botanbot_utilities

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<botanbot_utilities::PlannerBenchMarking>();
  while (rclcpp::ok() && !node->is_octomap_ready_) {
    rclcpp::spin_some(node->get_node_base_interface());
    RCLCPP_INFO(
      node->get_logger(),
      "Waiting for octomap to be ready In order to run planner bencmarking... ");
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  RCLCPP_INFO(
    node->get_logger()
    ,
    "Octomap ready, running bencmark with given configurations");
  auto paths_map = node->doBenchMarking();
  while (rclcpp::ok()) {
    node->publishSamplePlans(paths_map);
    rclcpp::spin_some(node->get_node_base_interface());
    RCLCPP_INFO(
      node->get_logger(), "publishing planner bencmarking... CTRL +X to stop");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  RCLCPP_INFO(
    node->get_logger()
    ,
    "Benchmarking done , exiting successfully");
  rclcpp::shutdown();
  return 0;
}
