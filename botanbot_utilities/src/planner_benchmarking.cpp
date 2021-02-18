#include "botanbot_utilities/planner_benchmarking.hpp"

namespace botanbot_utilities
{
PlannerBenchMarking::PlannerBenchMarking()
: Node("planner_benchmarking_rclcpp_node")
{
  is_octomap_ready_ = false;
  octomap_msg_ = std::make_shared<octomap_msgs::msg::Octomap>();

  this->declare_parameter("selected_planners", std::vector<std::string>({"PRMstar"}));
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
    state_space_information_ = std::make_shared<ompl::base::SpaceInformation>(state_space_);
    state_space_information_->setStateValidityChecker(
      std::bind(&PlannerBenchMarking::isStateValidSE2, this, std::placeholders::_1));
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
    state_space_information_ = std::make_shared<ompl::base::SpaceInformation>(state_space_);
    state_space_information_->setStateValidityChecker(
      std::bind(&PlannerBenchMarking::isStateValidSE2, this, std::placeholders::_1));
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
    state_space_information_ = std::make_shared<ompl::base::SpaceInformation>(state_space_);
    state_space_information_->setStateValidityChecker(
      std::bind(&PlannerBenchMarking::isStateValidSE2, this, std::placeholders::_1));
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
    state_space_information_ = std::make_shared<ompl::base::SpaceInformation>(state_space_);
    state_space_information_->setStateValidityChecker(
      std::bind(&PlannerBenchMarking::isStateValidSE3, this, std::placeholders::_1));
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
}

PlannerBenchMarking::~PlannerBenchMarking()
{

}

void PlannerBenchMarking::doBenchMarking()
{
  goal_.pose.position.y = 20;
  ompl::geometric::SimpleSetup ss(state_space_);
  // define start & goal states
  if ((selected_state_space_ == "REEDS") || (selected_state_space_ == "DUBINS") ||
    (selected_state_space_ == "SE2"))
  {
    ompl::base::ScopedState<ompl::base::SE2StateSpace> start(state_space_), goal(state_space_);
    double nan, yaw;
    botanbot_utilities::getRPYfromMsgQuaternion(start_.pose.orientation, nan, nan, yaw);
    start->setXY(start_.pose.position.x, start_.pose.position.y);
    start->setYaw(yaw);
    botanbot_utilities::getRPYfromMsgQuaternion(goal_.pose.orientation, nan, nan, yaw);
    goal->setXY(goal_.pose.position.x, goal_.pose.position.y);
    start->setYaw(yaw);
    ss.setStartAndGoalStates(start, goal);
    ss.setStateValidityChecker(
      [this](const ompl::base::State * state)
      {
        return isStateValidSE2(state);
      });
  } else {
    ompl::base::ScopedState<ompl::base::SE3StateSpace> start(state_space_), goal(state_space_);
    double nan, yaw;
    botanbot_utilities::getRPYfromMsgQuaternion(start_.pose.orientation, nan, nan, yaw);
    start->setXYZ(start_.pose.position.x, start_.pose.position.y, start_.pose.position.z);
    start->as<ompl::base::SO3StateSpace::StateType>(1)->setAxisAngle(0, 0, 1, yaw);
    botanbot_utilities::getRPYfromMsgQuaternion(goal_.pose.orientation, nan, nan, yaw);
    goal->setXYZ(goal_.pose.position.x, goal_.pose.position.y, goal_.pose.position.z);
    goal->as<ompl::base::SO3StateSpace::StateType>(1)->setAxisAngle(0, 0, 1, yaw);
    ss.setStartAndGoalStates(start, goal);
    ss.setStateValidityChecker(
      [this](const ompl::base::State * state)
      {
        return isStateValidSE3(state);
      });
  }

  // setting collision checking resolution to 0.05 (absolute)
  ss.getSpaceInformation()->setStateValidityCheckingResolution(0.2 / 50);
  ss.getProblemDefinition()->setOptimizationObjective(
    std::make_shared<ompl::base::PathLengthOptimizationObjective>(ss.getSpaceInformation()));

  // by default, use the Benchmark class
  double memoryLimit = 4096;
  int runCount = 100;

  ompl::tools::Benchmark::Request request(planner_timeout_, memoryLimit, runCount);
  ompl::tools::Benchmark b(ss, "outdoor_planbenchmarking");

  b.addPlanner(std::make_shared<ompl::geometric::PRMstar>(ss.getSpaceInformation()));
  b.addPlanner(std::make_shared<ompl::geometric::RRTstar>(ss.getSpaceInformation()));
  b.addPlanner(std::make_shared<ompl::geometric::RRTConnect>(ss.getSpaceInformation()));
  b.addPlanner(std::make_shared<ompl::geometric::KPIECE1>(ss.getSpaceInformation()));
  b.addPlanner(std::make_shared<ompl::geometric::SBL>(ss.getSpaceInformation()));
  b.addPlanner(std::make_shared<ompl::geometric::SST>(ss.getSpaceInformation()));
  b.addPlanner(std::make_shared<ompl::geometric::CForest>(ss.getSpaceInformation()));
  b.benchmark(request);
  b.saveResultsToFile("/home/ros2-foxy/outdoor_planbenchmarking.log");

}

bool PlannerBenchMarking::isStateValidSE2(const ompl::base::State * state)
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
  fcl::Vec3f translation(se2_state->getX(), se2_state->getY(), 0.5);
  tf2::Quaternion myQuaternion;
  myQuaternion.setRPY(0, 0, se2_state->getYaw());
  fcl::Quaternion3f rotation(myQuaternion.getX(), myQuaternion.getY(),
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
        std::make_shared<octomap::OcTree>(0.2);
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
}  // namespace botanbot_utilities

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<botanbot_utilities::PlannerBenchMarking>();
  while (rclcpp::ok() && !node->is_octomap_ready_) {
    rclcpp::spin_some(node->get_node_base_interface());
    RCLCPP_INFO(
      node->get_logger()
      ,
      "Waiting for octomap to be ready In order to run planner bencmarking... ");
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  RCLCPP_INFO(
    node->get_logger()
    ,
    "Octomap ready, runnin bencmark with given configurations");
  node->doBenchMarking();
  RCLCPP_INFO(
    node->get_logger()
    ,
    "Benchmarking done , exiting successfully");
  rclcpp::shutdown();
  return 0;
}
