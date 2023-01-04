#include "vox_nav_planning/tools/quadrotor_control_planners_benchmark.hpp"

namespace vox_nav_planning
{
  QuadrotorControlPlannersBenchMarking::QuadrotorControlPlannersBenchMarking()
  : Node("quadrotor_control_benchmarking_rclcpp_node")
  {
    RCLCPP_INFO(this->get_logger(), "Creating:");

    is_map_ready_ = false;

    this->declare_parameter(
      "selected_planners", std::vector<std::string>({"RRTstar", "PRMstar"}));
    this->declare_parameter("robot_mesh_path", "");
    this->declare_parameter("planner_timeout", 5.0);

    //  state bounds
    this->declare_parameter("state_bounds.min_x_pos", -50.0);
    this->declare_parameter("state_bounds.max_x_pos", 50.0);
    this->declare_parameter("state_bounds.min_y_pos", -50.0);
    this->declare_parameter("state_bounds.max_y_pos", 50.0);
    this->declare_parameter("state_bounds.min_z_pos", -50.0);
    this->declare_parameter("state_bounds.max_z_pos", 50.0);
    this->declare_parameter("state_bounds.min_roll", -3.14);
    this->declare_parameter("state_bounds.max_roll", 3.14);
    this->declare_parameter("state_bounds.min_pitch", -3.14);
    this->declare_parameter("state_bounds.max_pitch", 3.14);
    this->declare_parameter("state_bounds.min_yaw", -3.14);
    this->declare_parameter("state_bounds.max_yaw", 3.14);
    this->declare_parameter("state_bounds.min_x_vel", -1.0);
    this->declare_parameter("state_bounds.max_x_vel", 1.0);
    this->declare_parameter("state_bounds.min_y_vel", -1.0);
    this->declare_parameter("state_bounds.max_y_vel", 1.0);
    this->declare_parameter("state_bounds.min_z_vel", -1.0);
    this->declare_parameter("state_bounds.max_z_vel", 1.0);
    this->declare_parameter("state_bounds.min_roll_vel", -1.14);
    this->declare_parameter("state_bounds.max_roll_vel", 1.14);
    this->declare_parameter("state_bounds.min_pitch_vel", -1.14);
    this->declare_parameter("state_bounds.max_pitch_vel", 1.14);
    this->declare_parameter("state_bounds.min_yaw_vel", -1.14);
    this->declare_parameter("state_bounds.max_yaw_vel", 1.14);
    this->declare_parameter("state_bounds.min_x_acc", -1.0);
    this->declare_parameter("state_bounds.max_x_acc", 1.0);
    this->declare_parameter("state_bounds.min_y_acc", -1.0);
    this->declare_parameter("state_bounds.max_y_acc", 1.0);
    this->declare_parameter("state_bounds.min_z_acc", -1.0);
    this->declare_parameter("state_bounds.max_z_acc", 1.0);

    // Control Bounds
    this->declare_parameter("control_bounds.min_z_pos", -1.0);
    this->declare_parameter("control_bounds.max_z_pos", 1.0);
    this->declare_parameter("control_bounds.min_z_vel", -1.0);
    this->declare_parameter("control_bounds.max_z_vel", 1.0);
    this->declare_parameter("control_bounds.min_yaw_vel", -1.14);
    this->declare_parameter("control_bounds.max_yaw_vel", 1.14);
    this->declare_parameter("control_bounds.min_x_acc", -1.0);
    this->declare_parameter("control_bounds.max_x_acc", 1.0);
    this->declare_parameter("control_bounds.min_y_acc", -1.14);
    this->declare_parameter("control_bounds.max_y_acc", 1.14);
    this->declare_parameter("control_bounds.min_z_acc", -1.14);
    this->declare_parameter("control_bounds.max_z_acc", 1.14);
    this->declare_parameter("robot_body_dimens.x", 1.5);
    this->declare_parameter("robot_body_dimens.y", 1.5);
    this->declare_parameter("robot_body_dimens.z", 0.4);
    this->declare_parameter("goal_tolerance", 0.5);
    this->declare_parameter("min_euclidean_dist_start_to_goal", 25.0);
    this->declare_parameter("batch_size", 10);
    this->declare_parameter("epochs", 10);
    this->declare_parameter("max_memory", 2048);
    this->declare_parameter("results_output_dir", "/home/atas/");
    this->declare_parameter("results_file_regex", "SE2");
    this->declare_parameter("publish_a_sample_bencmark", true);
    this->declare_parameter("sample_bencmark_plans_topic", "benchmark_plan");

    this->get_parameter("selected_planners", selected_planners_);
    this->get_parameter("robot_mesh_path", robot_mesh_path_);
    this->get_parameter("planner_timeout", planner_timeout_);
    this->get_parameter("interpolation_parameter", interpolation_parameter_);
    this->get_parameter("robot_body_dimens.x", robot_body_dimensions_.x);
    this->get_parameter("robot_body_dimens.y", robot_body_dimensions_.y);
    this->get_parameter("robot_body_dimens.z", robot_body_dimensions_.z);
    this->get_parameter("goal_tolerance", goal_tolerance_);
    this->get_parameter(
      "min_euclidean_dist_start_to_goal", min_euclidean_dist_start_to_goal_);
    this->get_parameter("batch_size", batch_size_);
    this->get_parameter("epochs", epochs_);
    this->get_parameter("max_memory", max_memory_);
    this->get_parameter("results_output_dir", results_output_dir_);
    this->get_parameter("results_file_regex", results_file_regex_);
    this->get_parameter("publish_a_sample_bencmark", publish_a_sample_bencmark_);
    this->get_parameter(
      "sample_bencmark_plans_topic", sample_bencmark_plans_topic_);

    // Place all state bounds in a struct
    this->get_parameter("state_bounds.min_x_pos", state_bounds_.x_pos.first);
    this->get_parameter("state_bounds.max_x_pos", state_bounds_.x_pos.second);
    this->get_parameter("state_bounds.min_y_pos", state_bounds_.y_pos.first);
    this->get_parameter("state_bounds.max_y_pos", state_bounds_.y_pos.second);
    this->get_parameter("state_bounds.min_z_pos", state_bounds_.z_pos.first);
    this->get_parameter("state_bounds.max_z_pos", state_bounds_.z_pos.second);
    this->get_parameter("state_bounds.min_roll", state_bounds_.roll.first);
    this->get_parameter("state_bounds.max_roll", state_bounds_.roll.second);
    this->get_parameter("state_bounds.min_pitch", state_bounds_.pitch.first);
    this->get_parameter("state_bounds.max_pitch", state_bounds_.pitch.second);
    this->get_parameter("state_bounds.min_yaw", state_bounds_.yaw.first);
    this->get_parameter("state_bounds.max_yaw", state_bounds_.yaw.second);
    this->get_parameter("state_bounds.min_x_vel", state_bounds_.x_vel.first);
    this->get_parameter("state_bounds.max_x_vel", state_bounds_.x_vel.second);
    this->get_parameter("state_bounds.min_y_vel", state_bounds_.y_vel.first);
    this->get_parameter("state_bounds.max_y_vel", state_bounds_.y_vel.second);
    this->get_parameter("state_bounds.min_z_vel", state_bounds_.z_vel.first);
    this->get_parameter("state_bounds.max_z_vel", state_bounds_.z_vel.second);
    this->get_parameter("state_bounds.min_roll_vel", state_bounds_.roll_vel.first);
    this->get_parameter("state_bounds.max_roll_vel", state_bounds_.roll_vel.second);
    this->get_parameter("state_bounds.min_pitch_vel", state_bounds_.pitch_vel.first);
    this->get_parameter("state_bounds.max_pitch_vel", state_bounds_.pitch_vel.second);
    this->get_parameter("state_bounds.min_yaw_vel", state_bounds_.yaw_vel.first);
    this->get_parameter("state_bounds.max_yaw_vel", state_bounds_.yaw_vel.second);
    this->get_parameter("state_bounds.min_x_acc", state_bounds_.x_acc.first);
    this->get_parameter("state_bounds.max_x_acc", state_bounds_.x_acc.second);
    this->get_parameter("state_bounds.min_y_acc", state_bounds_.y_acc.first);
    this->get_parameter("state_bounds.max_y_acc", state_bounds_.y_acc.second);
    this->get_parameter("state_bounds.min_z_acc", state_bounds_.z_acc.first);
    this->get_parameter("state_bounds.max_z_acc", state_bounds_.z_acc.second);

    // Place all control bounds in a struct
    this->get_parameter("control_bounds.min_z_pos", control_bounds_.z_pos.first);
    this->get_parameter("control_bounds.max_z_pos", control_bounds_.z_pos.second);
    this->get_parameter("control_bounds.min_z_vel", control_bounds_.z_vel.first);
    this->get_parameter("control_bounds.max_z_vel", control_bounds_.z_vel.second);
    this->get_parameter("control_bounds.min_yaw_vel", control_bounds_.yaw_vel.first);
    this->get_parameter("control_bounds.max_yaw_vel", control_bounds_.yaw_vel.second);
    this->get_parameter("control_bounds.min_x_acc", control_bounds_.x_acc.first);
    this->get_parameter("control_bounds.max_x_acc", control_bounds_.x_acc.second);
    this->get_parameter("control_bounds.min_y_acc", control_bounds_.y_acc.first);
    this->get_parameter("control_bounds.max_y_acc", control_bounds_.y_acc.second);
    this->get_parameter("control_bounds.min_z_acc", control_bounds_.z_acc.first);
    this->get_parameter("control_bounds.max_z_acc", control_bounds_.z_acc.second);

    typedef std::shared_ptr<fcl::CollisionGeometryf> CollisionGeometryPtr_t;
    CollisionGeometryPtr_t robot_body_box(new fcl::Box<float>(
        robot_body_dimensions_.x,
        robot_body_dimensions_.y,
        robot_body_dimensions_.z
    ));

    fcl::CollisionObjectf robot_body_box_object(robot_body_box, fcl::Transform3f());
    robot_collision_object_ = std::make_shared<fcl::CollisionObjectf>(robot_body_box_object);
    get_maps_and_surfels_client_node_ = std::make_shared
      <rclcpp::Node>("get_maps_and_surfels_client_node");

    get_maps_and_surfels_client_ =
      get_maps_and_surfels_client_node_->create_client<vox_nav_msgs::srv::GetMapsAndSurfels>(
      "get_maps_and_surfels");

    setupMap();

    auto state_bounds = std::make_shared<ompl::base::RealVectorBounds>(15);
    auto control_bounds = std::make_shared<ompl::base::RealVectorBounds>(6);

    // Set state bounds
    state_bounds->setLow(0, state_bounds_.x_pos.first);
    state_bounds->setHigh(0, state_bounds_.x_pos.second);
    state_bounds->setLow(1, state_bounds_.y_pos.first);
    state_bounds->setHigh(1, state_bounds_.y_pos.second);
    state_bounds->setLow(2, state_bounds_.z_pos.first);
    state_bounds->setHigh(2, state_bounds_.z_pos.second);
    state_bounds->setLow(3, state_bounds_.roll.first);
    state_bounds->setHigh(3, state_bounds_.roll.second);
    state_bounds->setLow(4, state_bounds_.pitch.first);
    state_bounds->setHigh(4, state_bounds_.pitch.second);
    state_bounds->setLow(5, state_bounds_.yaw.first);
    state_bounds->setHigh(5, state_bounds_.yaw.second);
    state_bounds->setLow(6, state_bounds_.x_vel.first);
    state_bounds->setHigh(6, state_bounds_.x_vel.second);
    state_bounds->setLow(7, state_bounds_.y_vel.first);
    state_bounds->setHigh(7, state_bounds_.y_vel.second);
    state_bounds->setLow(8, state_bounds_.z_vel.first);
    state_bounds->setHigh(8, state_bounds_.z_vel.second);
    state_bounds->setLow(9, state_bounds_.roll_vel.first);
    state_bounds->setHigh(9, state_bounds_.roll_vel.second);
    state_bounds->setLow(10, state_bounds_.pitch_vel.first);
    state_bounds->setHigh(10, state_bounds_.pitch_vel.second);
    state_bounds->setLow(11, state_bounds_.yaw_vel.first);
    state_bounds->setHigh(11, state_bounds_.yaw_vel.second);
    state_bounds->setLow(12, state_bounds_.x_acc.first);
    state_bounds->setHigh(12, state_bounds_.x_acc.second);
    state_bounds->setLow(13, state_bounds_.y_acc.first);
    state_bounds->setHigh(13, state_bounds_.y_acc.second);
    state_bounds->setLow(14, state_bounds_.z_acc.first);
    state_bounds->setHigh(14, state_bounds_.z_acc.second);

    // Set control bounds
    control_bounds->setLow(0, control_bounds_.z_pos.first);
    control_bounds->setHigh(0, control_bounds_.z_pos.second);
    control_bounds->setLow(1, control_bounds_.z_vel.first);
    control_bounds->setHigh(1, control_bounds_.z_vel.second);
    control_bounds->setLow(2, control_bounds_.yaw_vel.first);
    control_bounds->setHigh(2, control_bounds_.yaw_vel.second);
    control_bounds->setLow(3, control_bounds_.x_acc.first);
    control_bounds->setHigh(3, control_bounds_.x_acc.second);
    control_bounds->setLow(4, control_bounds_.y_acc.first);
    control_bounds->setHigh(4, control_bounds_.y_acc.second);
    control_bounds->setLow(5, control_bounds_.z_acc.first);
    control_bounds->setHigh(5, control_bounds_.z_acc.second);

    state_space_ = std::make_shared<ompl::base::RealVectorStateSpace>(15);
    state_space_->as<ompl::base::RealVectorStateSpace>()->setBounds(*state_bounds);

    control_state_space_ = std::make_shared<ompl::control::RealVectorControlSpace>(state_space_, 6);
    control_state_space_->as<ompl::control::RealVectorControlSpace>()->setBounds(*control_bounds);

    control_simple_setup_ = std::make_shared<ompl::control::SimpleSetup>(control_state_space_);
    control_simple_setup_->setOptimizationObjective(getOptimizationObjective());
    control_simple_setup_->setStateValidityChecker(
      std::bind(&QuadrotorControlPlannersBenchMarking::isStateValid, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Selected planners for benchmarking:");
    for (auto && i : selected_planners_) {
      RCLCPP_INFO(this->get_logger(), " %s", i.c_str());
    }

    // Initialize pubs & subs
    plan_publisher_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>(
      sample_bencmark_plans_topic_.c_str(), rclcpp::SystemDefaultsQoS());

    start_goal_poses_publisher_ =
      this->create_publisher<geometry_msgs::msg::PoseArray>(
      "start_goal_poses", rclcpp::SystemDefaultsQoS());
  }

  QuadrotorControlPlannersBenchMarking::~QuadrotorControlPlannersBenchMarking()
  {
    RCLCPP_INFO(this->get_logger(), "Destroying:");
  }

  void QuadrotorControlPlannersBenchMarking::propagate(
    const ompl::control::SpaceInformation * si,
    const ompl::base::State * start,
    const ompl::control::Control * control,
    const double duration,
    ompl::base::State * result)
  {
    const auto * states = start->as<ompl::base::RealVectorStateSpace::StateType>();
    const auto * controls = control->as<ompl::control::RealVectorControlSpace::ControlType>();

    // Extract all the states and propagate them
    double x_pos = states->values[0];
    double y_pos = states->values[1];
    double z_pos = states->values[2];
    double roll = states->values[3];
    double pitch = states->values[4];
    double yaw = states->values[5];
    double x_vel = states->values[6];
    double y_vel = states->values[7];
    double z_vel = states->values[8];
    double roll_vel = states->values[9];
    double pitch_vel = states->values[10];
    double yaw_vel = states->values[11];
    double x_acc = states->values[12];
    double y_acc = states->values[13];
    double z_acc = states->values[14];

    // Extract all the controls and propagate them
    double c_z_pos = controls->values[0];
    double c_z_vel = controls->values[1];
    double c_yaw_vel = controls->values[2];
    double c_x_acc = controls->values[3];
    double c_y_acc = controls->values[4];
    double c_z_acc = controls->values[5];

    // Propagate the states
    auto propogated_states = quadrotor_controller_.step(
      x_pos, y_pos, z_pos, roll, pitch, yaw, x_vel, y_vel, z_vel,
      roll_vel, pitch_vel, yaw_vel, x_acc, y_acc, z_acc,
      c_z_pos, c_z_vel, c_yaw_vel, c_x_acc, c_y_acc, c_z_acc, duration);

    // Set the result
    auto * result_states = result->as<ompl::base::RealVectorStateSpace::StateType>();
    std::copy(propogated_states.begin(), propogated_states.end(), result_states->values);

    // Enforce the bounds
    si->enforceBounds(result);
  }

  ompl::base::OptimizationObjectivePtr QuadrotorControlPlannersBenchMarking::
  getOptimizationObjective()
  {
    // select a optimizatio objective
    ompl::base::OptimizationObjectivePtr length_objective(
      new ompl::base::PathLengthOptimizationObjective(control_simple_setup_->getSpaceInformation()));

    return ompl::base::OptimizationObjectivePtr(length_objective);
  }

  std::map<int, ompl::control::PathControl>
  QuadrotorControlPlannersBenchMarking::doBenchMarking()
  {

    ompl::base::ScopedState<ompl::base::RealVectorStateSpace>
    random_start(state_space_), random_goal(state_space_);

    geometry_msgs::msg::PoseStamped start, goal;
    std::map<int, ompl::control::PathControl> paths_map;
    auto si = control_simple_setup_->getSpaceInformation();

    std::stringstream ss;

    for (int i = 0; i < epochs_; i++) {

      paths_map.clear();

      // spin until a valid random start and goal poses are found. Also
      // make sure that a soluion exists for generated states
      volatile bool found_valid_random_start_goal = false;
      double start_yaw, goal_yaw, nan;

      while (!found_valid_random_start_goal) {

        start_yaw = getRangedRandom(state_bounds_.yaw.first, state_bounds_.yaw.second);
        goal_yaw = getRangedRandom(state_bounds_.yaw.first, state_bounds_.yaw.second);

        start.pose.position.x = 1;//getRangedRandom(se_bounds_.minx, se_bounds_.maxx);
        start.pose.position.y = 0; //getRangedRandom(se_bounds_.miny, se_bounds_.maxy);
        start.pose.position.z = 2;         //getRangedRandom(se_bounds_.miny, se_bounds_.maxy);

        start.pose.orientation = vox_nav_utilities::getMsgQuaternionfromRPY(nan, nan, 0);

        goal.pose.position.x = 35;//getRangedRandom(se_bounds_.minx, se_bounds_.maxx);
        goal.pose.position.y = 35;//getRangedRandom(se_bounds_.miny, se_bounds_.maxy);
        goal.pose.position.z = 3;        //getRangedRandom(se_bounds_.miny, se_bounds_.maxy);

        goal.pose.orientation = vox_nav_utilities::getMsgQuaternionfromRPY(nan, nan, 0);

        random_start->values[0] = start.pose.position.x;
        random_start->values[1] = start.pose.position.y;
        random_start->values[2] = start.pose.position.z;
        random_start->values[3] = 0;
        random_start->values[4] = 0;
        random_start->values[5] = start_yaw;

        random_goal->values[0] = goal.pose.position.x;
        random_goal->values[1] = goal.pose.position.y;
        random_goal->values[2] = goal.pose.position.z;
        random_goal->values[3] = 0;
        random_goal->values[4] = 0;
        random_goal->values[5] = goal_yaw;

        // the distance should be above a certain threshold
        double distance = state_space_->distance(random_start.get(), random_goal.get());

        // create a planner for the defined space
        ompl::base::PlannerPtr rrtstar_planner;
        rrtstar_planner =
          ompl::base::PlannerPtr(new ompl::control::RRT(si));
        control_simple_setup_->setPlanner(rrtstar_planner);

        //ompl::base::PlannerStatus has_solution = control_simple_setup_->solve(20.0);

        // if it gets to this point , that menas our random states are valid and
        // already meets min dist requiremnets but now there also has to be a
        // solution for this problem
        /*found_valid_random_start_goal =
          (has_solution == ompl::base::PlannerStatus::EXACT_SOLUTION);*/


        found_valid_random_start_goal =
          (isStateValid(random_start.get()) && isStateValid(random_goal.get()) &&
          distance > min_euclidean_dist_start_to_goal_);

        if (!found_valid_random_start_goal) {
          RCLCPP_INFO(
            this->get_logger(),
            "Still Looking to sample valid random start and goal states ... ");
          continue;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }

      control_simple_setup_->setStartAndGoalStates(random_start, random_goal, goal_tolerance_);

      auto si = control_simple_setup_->getSpaceInformation();
      si->setMinMaxControlDuration(20, 30);
      si->setPropagationStepSize(0.01);


      control_simple_setup_->setStatePropagator(
        [this, si](const ompl::base::State * state, const ompl::control::Control * control,
        const double duration, ompl::base::State * result)
        {
          this->propagate(si.get(), state, control, duration, result);
        });

      /*RCLCPP_INFO(
        this->get_logger(),
        "A valid random start and goal states has been found.");*/

      ompl::tools::Benchmark benchmark(*control_simple_setup_, "benchmark");
      std::mutex plan_mutex;
      int index(0);
      for (auto && planner_name : selected_planners_) {
        ompl::base::PlannerPtr planner_ptr;
        initializeSelectedControlPlanner(
          planner_ptr,
          planner_name,
          si,
          logger_);

        if (publish_a_sample_bencmark_) {
          std::lock_guard<std::mutex> guard(plan_mutex);

          RCLCPP_INFO(this->get_logger(), "Creating sample plans.");

          control_simple_setup_->setPlanner(planner_ptr);
          control_simple_setup_->setup();
          control_simple_setup_->print(std::cout);
          ompl::base::PlannerStatus solved = control_simple_setup_->solve(planner_timeout_);
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
          }

          ss << planner_name.c_str() << " " << solved << " " << solution_path.length() << "\n";
          std::pair<int, ompl::control::PathControl> curr_pair(index, solution_path);
          paths_map.insert(curr_pair);
          control_simple_setup_->clear();
        }
        index++;
      }

      std::cout << ss.str() << std::endl;

      /*
      benchmark.addPlanner(planner_ptr);


      ompl::tools::Benchmark::Request request(planner_timeout_, max_memory_,
        batch_size_);
      request.displayProgress = true;

      RCLCPP_INFO(
        this->get_logger(),
        "Created sample plans from each planner, "
        "Now performing actual benchmark, This might take some time.");

      benchmark.benchmark(request);

      benchmark.saveResultsToFile(
        (results_output_dir_ + results_file_regex_ +
        "_" + std::to_string(i) + ".log")
        .c_str());

      RCLCPP_INFO(
        this->get_logger(),
        "Bencmarking results saved to given directory: %s",
        results_output_dir_.c_str());*/

      control_simple_setup_->clear();

      publishSamplePlans(paths_map);
    }
    return paths_map;
  }

  bool QuadrotorControlPlannersBenchMarking::isStateValid(const ompl::base::State * state)
  {
    const auto * cstate = state->as<ompl::base::RealVectorStateSpace::StateType>();
    auto x = cstate->values[0];
    auto y = cstate->values[1];
    auto z = cstate->values[2];
    auto roll = cstate->values[3];
    auto pitch = cstate->values[4];
    auto yaw = cstate->values[5];

    fcl::CollisionRequestf requestType(1, false, 1, false);
    // check validity of state Fdefined by pos & rot
    fcl::Vector3f translation(x, y, z);
    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY(roll, pitch, yaw);
    fcl::Quaternionf rotation(
      myQuaternion.getX(), myQuaternion.getY(),
      myQuaternion.getZ(), myQuaternion.getW());

    robot_collision_object_->setTransform(rotation, translation);

    fcl::CollisionResultf collisionWithFullMapResult;

    fcl::collide<float>(
      robot_collision_object_.get(),
      original_octomap_collision_object_.get(), requestType, collisionWithFullMapResult);

    return !collisionWithFullMapResult.isCollision();
  }

  void QuadrotorControlPlannersBenchMarking::publishSamplePlans(
    std::map<int, ompl::control::PathControl> sample_paths)
  {

    if (!publish_a_sample_bencmark_) {
      RCLCPP_INFO(this->get_logger(), "Will not publish sample plans.");
    }
    visualization_msgs::msg::MarkerArray marker_array;
    int total_poses = 0;

    auto it = sample_paths.begin();
    while (it != sample_paths.end()) {


      for (std::size_t curr_path_state = 0;
        curr_path_state < it->second.getStateCount(); curr_path_state++)
      {

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = rclcpp::Clock().now();

        if (!robot_mesh_path_.empty()) {
          marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
          marker.mesh_resource = robot_mesh_path_;
        } else {
          marker.type = visualization_msgs::msg::Marker::CUBE;
        }

        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.lifetime = rclcpp::Duration::from_seconds(0);
        marker.scale.x = robot_body_dimensions_.x;
        marker.scale.y = robot_body_dimensions_.y;
        marker.scale.z = robot_body_dimensions_.z;
        marker.id = total_poses;
        marker.color = getColorByIndex(it->first);
        marker.ns = "path" + std::to_string(it->first);

        const auto * cstate =
          it->second.getState(curr_path_state)->as<ompl::base::RealVectorStateSpace::StateType>();

        geometry_msgs::msg::Point p;
        p.x = cstate->values[0];
        p.y = cstate->values[1];
        p.z = cstate->values[2];
        //marker.points.push_back(p);
        //marker.colors.push_back(getColorByIndex(it->first));

        marker.pose.position = p;
        //marker.colors.push_back(getColorByIndex(it->first));

        visualization_msgs::msg::Marker text;
        text.header.frame_id = "map";
        text.header.stamp = rclcpp::Clock().now();
        text.ns = "path";
        text.id = total_poses + 1000;
        text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::msg::Marker::ADD;
        text.lifetime = rclcpp::Duration::from_seconds(0);
        text.text = std::to_string(total_poses);
        text.pose = marker.pose;
        text.pose.position.z += 0.5;
        text.scale.x = 0.3;
        text.scale.y = 0.3;
        text.scale.z = 0.3;
        text.color.a = 1.0;
        text.color.g = 1.0;
        text.color.r = 1.0;

        marker_array.markers.push_back(text);
        marker_array.markers.push_back(marker);
        total_poses++;
      }

      it++;
    }

    plan_publisher_->publish(marker_array);

    /*geometry_msgs::msg::PoseArray start_and_goal;
    start_and_goal.header = marker_array.markers.front().header;
    geometry_msgs::msg::Pose start, goal;
    start.position.x = start_.x;
    start.position.y = start_.y;
    start.position.z = start_.z;
    start.orientation =
      vox_nav_utilities::getMsgQuaternionfromRPY(0, 0, start_.yaw);
    goal.position.x = goal_.x;
    goal.position.y = goal_.y;
    goal.position.z = goal_.z;
    goal.orientation =
      vox_nav_utilities::getMsgQuaternionfromRPY(0, 0, goal_.yaw);
    start_and_goal.poses.push_back(start);
    start_and_goal.poses.push_back(goal);
    start_goal_poses_publisher_->publish(start_and_goal);*/

  }

  void QuadrotorControlPlannersBenchMarking::setupMap()
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

      auto original_octomap_octree =
        dynamic_cast<octomap::OcTree *>(octomap_msgs::fullMsgToMap(response->original_octomap));
      original_octomap_octree_ = std::make_shared<octomap::OcTree>(*original_octomap_octree);

      delete original_octomap_octree;

      auto original_octomap_fcl_octree = std::make_shared<fcl::OcTreef>(original_octomap_octree_);
      original_octomap_collision_object_ = std::make_shared<fcl::CollisionObjectf>(
        std::shared_ptr<fcl::CollisionGeometryf>(original_octomap_fcl_octree));

      RCLCPP_INFO(
        logger_,
        "Recieved a valid Octomap with %d nodes, A FCL collision tree will be created from this "
        "octomap for state validity (aka collision check)", original_octomap_octree_->size());
    }
  }

  std_msgs::msg::ColorRGBA QuadrotorControlPlannersBenchMarking::getColorByIndex(int index)
  {
    std_msgs::msg::ColorRGBA result;
    switch (index) {
      case 0: // RED:
        result.r = 0.8;
        result.g = 0.1;
        result.b = 0.1;
        result.a = 1.0;
        break;
      case 1: // GREEN:
        result.r = 0.1;
        result.g = 0.8;
        result.b = 0.1;
        result.a = 1.0;
        break;
      case 2: // GREY:
        result.r = 0.9;
        result.g = 0.9;
        result.b = 0.9;
        result.a = 1.0;
        break;
      case 3: // CYAN:
        result.r = 0.0;
        result.g = 1.0;
        result.b = 1.0;
        result.a = 1.0;
        break;
      case 4: // YELLOW:
        result.r = 1.0;
        result.g = 1.0;
        result.b = 0.0;
        result.a = 1.0;
        break;
      case 5: // ORANGE:
        result.r = 1.0;
        result.g = 0.5;
        result.b = 0.0;
        result.a = 1.0;
        break;
      case 6: // Maroon:
        result.r = 0.5;
        result.g = 0.0;
        result.b = 0.0;
        result.a = 1.0;
        break;
      case 7: // Olive:
        result.r = 0.5;
        result.g = 0.5;
        result.b = 0.0;
        result.a = 1.0;
        break;
      case 8: // Navy:
        result.r = 0.0;
        result.g = 0.0;
        result.b = 0.5;
        result.a = 1.0;
        break;
      case 9: // BLACK:
        result.r = 0.0;
        result.g = 0.0;
        result.b = 0.0;
        result.a = 1.0;
        break;
      case 10: // YELLOW:
        result.r = 1.0;
        result.g = 1.0;
        result.b = 0.0;
        result.a = 1.0;
        break;
      case 11: // BROWN:
        result.r = 0.597;
        result.g = 0.296;
        result.b = 0.0;
        result.a = 1.0;
        break;
      case 12: // PINK:
        result.r = 1.0;
        result.g = 0.4;
        result.b = 1;
        result.a = 1.0;
        break;
      case 13: // LIME_GREEN:
        result.r = 0.6;
        result.g = 1.0;
        result.b = 0.2;
        result.a = 1.0;
        break;
      case 14: // PURPLE:
        result.r = 0.597;
        result.g = 0.0;
        result.b = 0.597;
        result.a = 1.0;
        break;
      case 15: // CYAN:
        result.r = 0.0;
        result.g = 1.0;
        result.b = 1.0;
        result.a = 1.0;
        break;
      case 16: // MAGENTA:
        result.r = 1.0;
        result.g = 0.0;
        result.b = 1.0;
        result.a = 1.0;
    }
    return result;
  }

  double QuadrotorControlPlannersBenchMarking::getRangedRandom(double min, double max)
  {
    std::random_device rd;
    std::default_random_engine eng(rd());
    std::uniform_real_distribution<double> distr(min, max);

    return distr(eng);
  }

} // namespace vox_nav_planning

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vox_nav_planning::QuadrotorControlPlannersBenchMarking>();
  while (rclcpp::ok() && !node->is_map_ready_) {
    rclcpp::spin_some(node->get_node_base_interface());
    RCLCPP_INFO(
      node->get_logger(), "Waiting for octomap to be ready In order "
      "to run planner bencmarking... ");
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  RCLCPP_INFO(
    node->get_logger(),
    "Octomap ready, running bencmark with given configurations");
  auto paths_map = node->doBenchMarking();
  while (rclcpp::ok()) {
    node->publishSamplePlans(paths_map);
    rclcpp::spin_some(node->get_node_base_interface());
    RCLCPP_INFO(
      node->get_logger(),
      "publishing planner bencmarking... press CTRL+C to stop");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  RCLCPP_INFO(node->get_logger(), "Benchmarking done , exiting successfully");
  rclcpp::shutdown();
  return 0;
}
