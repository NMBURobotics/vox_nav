#include "botanbot_planner/botanbot_se2_planning_control_space.hpp"

namespace botanbot_planner
{
// a decomposition is only needed for SyclopRRT and SyclopEST
class MyDecomposition : public ompl::control::GridDecomposition
{
public:
  MyDecomposition(const int length, const ompl::base::RealVectorBounds & bounds)
  : GridDecomposition(length, 2, bounds)
  {
  }
  void project(const ompl::base::State * s, std::vector<double> & coord) const override
  {
    coord.resize(2);
    coord[0] = s->as<ompl::base::SE2StateSpace::StateType>()->getX();
    coord[1] = s->as<ompl::base::SE2StateSpace::StateType>()->getY();
  }

  void sampleFullState(
    const ompl::base::StateSamplerPtr & sampler, const std::vector<double> & coord,
    ompl::base::State * s) const override
  {
    sampler->sampleUniform(s);
    s->as<ompl::base::SE2StateSpace::StateType>()->setXY(coord[0], coord[1]);
  }
};


BotanbotSE2PlanningControlSpace::BotanbotSE2PlanningControlSpace()
: Node("botanbot_planner_rclcpp_node"),
  robot_collision_geometry_(new fcl::Box(1.0, 1.0, 0.8))
{
  robot_collision_object_ = std::make_shared<fcl::CollisionObject>(robot_collision_geometry_);

  vis_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker", 10);
  traj_pub_ =
    this->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>("waypoints", 10);
  octomap_pub_ =
    this->create_publisher<octomap_msgs::msg::Octomap>("octomap", 10);
  octomap_pointcloud_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "pointcloud2_octomap",
    rclcpp::SystemDefaultsQoS());

  const std::string filename = "/home/ros2-foxy/f.bt";

  octomap_octree_ = std::make_shared<octomap::OcTree>(0.1);
  octomap_octree_->readBinary(filename);

  fcl_octree_ = std::make_shared<fcl::OcTree>(
    std::shared_ptr<const octomap::OcTree>(
      octomap_octree_));

  fcl_octree_collision_object_ =
    std::make_shared<fcl::CollisionObject>(std::shared_ptr<fcl::CollisionGeometry>(fcl_octree_));

  sleep(5.0);

  plan();
}

BotanbotSE2PlanningControlSpace::~BotanbotSE2PlanningControlSpace()
{
}

void BotanbotSE2PlanningControlSpace::propagate(
  const ompl::base::State * start, const ompl::control::Control * control, const double duration,
  ompl::base::State * result)
{
  const auto * se2state = start->as<ompl::base::SE2StateSpace::StateType>();
  const double * pos = se2state->as<ompl::base::RealVectorStateSpace::StateType>(0)->values;
  const double rot = se2state->as<ompl::base::SO2StateSpace::StateType>(1)->value;
  const double * ctrl = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;

  result->as<ompl::base::SE2StateSpace::StateType>()->setXY(
    pos[0] + ctrl[0] * duration * cos(rot),
    pos[1] + ctrl[0] * duration * sin(rot));
  result->as<ompl::base::SE2StateSpace::StateType>()->setYaw(
    rot + ctrl[1] * duration);
}

bool BotanbotSE2PlanningControlSpace::isStateValid(const ompl::base::State * state)
{
  // cast the abstract state type to the type we expect
  const ompl::base::ReedsSheppStateSpace::StateType * red_state =
    state->as<ompl::base::ReedsSheppStateSpace::StateType>();
  // check validity of state Fdefined by pos & rot
  fcl::Vec3f translation(red_state->getX(), red_state->getY(), 0.5);

  tf2::Quaternion myQuaternion;
  myQuaternion.setRPY(0, 0, red_state->getYaw());
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

void BotanbotSE2PlanningControlSpace::plan()
{
  // construct the state space we are planning in
  auto space(std::make_shared<ompl::base::SE2StateSpace>());

  // set the bounds for the R^2 part of SE(2)
  ompl::base::RealVectorBounds bounds(2);
  bounds.setLow(-20);
  bounds.setHigh(20);
  space->setBounds(bounds);

  // create a control space
  auto cspace(std::make_shared<ompl::control::RealVectorControlSpace>(space, 2));

  // set the bounds for the control space
  ompl::base::RealVectorBounds cbounds(2);
  cbounds.setLow(-0.1);
  cbounds.setHigh(0.1);

  cspace->setBounds(cbounds);

  // construct an instance of  space information from this control space
  auto si(std::make_shared<ompl::control::SpaceInformation>(space, cspace));

  // set state validity checking for this space
  si->setStateValidityChecker(
    [this](const ompl::base::State * state) {return isStateValid(state);});

  // set the state propagation routine
  si->setStatePropagator(propagate);

  // create a start state
  ompl::base::ScopedState<ompl::base::SE2StateSpace> start(space);
  start->setX(0);
  start->setY(0.0);
  start->setYaw(0.0);

  // create a goal state
  ompl::base::ScopedState<ompl::base::SE2StateSpace> goal(start);
  goal->setX(18.5);

  // create a problem instance
  auto pdef(std::make_shared<ompl::base::ProblemDefinition>(si));

  // set the start and goal states
  pdef->setStartAndGoalStates(start, goal, 0.2);

  // create a planner for the defined space
  //auto planner(std::make_shared<ompl::control::RRT>(si));
  //auto planner(std::make_shared<ompl::control::EST>(si));
  //auto planner(std::make_shared<ompl::control::KPIECE1>(si));
  auto decomp(std::make_shared<MyDecomposition>(32, bounds));
  auto planner(std::make_shared<ompl::control::RRT>(si));
  //auto planner(std::make_shared<ompl::control::SyclopRRT>(si, decomp));

  // set the problem we are trying to solve for the planner
  planner->setProblemDefinition(pdef);

  // perform setup steps for the planner
  planner->setup();

  // print the settings for this space
  si->printSettings(std::cout);

  // print the problem settings
  pdef->print(std::cout);

  // attempt to solve the problem within ten seconds of planning time
  ompl::base::PlannerStatus solved = planner->ompl::base::Planner::solve(1.0);


  if (solved) {
    std::vector<double> reals;

    std::cout << "Found solution:" << std::endl;

    ompl::geometric::PathGeometric * pth =
      pdef->getSolutionPath()->as<ompl::geometric::PathGeometric>();
    pth->interpolate(50);


    //Publish path as markers
    visualization_msgs::msg::MarkerArray marker_array;

    for (std::size_t path_idx = 0; path_idx < pth->getStateCount(); path_idx++) {

      // cast the abstract state type to the type we expect
      const ompl::base::SE2StateSpace::StateType * red_state =
        pth->getState(path_idx)->as<ompl::base::SE2StateSpace::StateType>();

      // check validity of state Fdefined by pos & rot
      fcl::Vec3f translation(red_state->getX(), red_state->getY(), 0.5);

      tf2::Quaternion myQuaternion;
      myQuaternion.setRPY(0, 0, red_state->getYaw());
      fcl::Quaternion3f rotation(myQuaternion.getX(), myQuaternion.getY(),
        myQuaternion.getZ(), myQuaternion.getW());

      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = rclcpp::Clock().now();
      marker.ns = "path";
      marker.id = path_idx;
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.lifetime = rclcpp::Duration::from_seconds(0);
      marker.pose.position.x = red_state->getX();
      marker.pose.position.y = red_state->getY();
      marker.pose.position.z = 0.5;
      marker.pose.orientation.x = myQuaternion.getX();
      marker.pose.orientation.y = myQuaternion.getY();
      marker.pose.orientation.z = myQuaternion.getZ();
      marker.pose.orientation.w = myQuaternion.getW();
      marker.scale.x = 1.0;
      marker.scale.y = 1.0;
      marker.scale.z = 0.1;
      marker.color.a = 1.0;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker_array.markers.push_back(marker);
    }
    vis_pub_->publish(marker_array);
    vis_pub_->publish(marker_array);
    vis_pub_->publish(marker_array);
    vis_pub_->publish(marker_array);
    vis_pub_->publish(marker_array);
    vis_pub_->publish(marker_array);
    vis_pub_->publish(marker_array);
    vis_pub_->publish(marker_array);

    std::cout << "Published marker size " << marker_array.markers.size() << std::endl;
  } else {
    std::cout << "No solution found" << std::endl;
  }

  octomap_msgs::msg::Octomap map;
  map.header.frame_id = "map";
  map.header.stamp = rclcpp::Clock().now();
  if (octomap_msgs::fullMapToMsg(*octomap_octree_, map)) {
    octomap_pub_->publish(map);
  }
  pcl::PointCloud<pcl::PointXYZRGB> pclCloud;

  // now, traverse all leafs in the tree:
  for (auto it = octomap_octree_->begin(0),
    end = octomap_octree_->end(); it != end; ++it)
  {
    if (octomap_octree_->isNodeOccupied(*it)) {

      double half_size = it.getSize() / 2.0;

      double size = it.getSize();
      double x = it.getX();
      double y = it.getY();
      double z = it.getZ();

      int r = z * 10;
      int g = 0;
      int b = 0;

      // insert into pointcloud:
      pcl::PointXYZRGB _point = pcl::PointXYZRGB();
      _point.x = x;
      _point.y = y;
      _point.z = z;
      _point.r = r;
      _point.g = g;
      _point.b = b;
      pclCloud.push_back(_point);
    }
  }

  sensor_msgs::msg::PointCloud2 cloud;
  pcl::toROSMsg(pclCloud, cloud);
  cloud.header.frame_id = "map";
  cloud.header.stamp = rclcpp::Clock().now();
  octomap_pointcloud_pub_->publish(cloud);
  octomap_pointcloud_pub_->publish(cloud);
  octomap_pointcloud_pub_->publish(cloud);
  octomap_pointcloud_pub_->publish(cloud);
  octomap_pointcloud_pub_->publish(cloud);
  octomap_pointcloud_pub_->publish(cloud);
  octomap_pointcloud_pub_->publish(cloud);
  octomap_pointcloud_pub_->publish(cloud);
  octomap_pointcloud_pub_->publish(cloud);
  octomap_pointcloud_pub_->publish(cloud);
  octomap_pointcloud_pub_->publish(cloud);
  octomap_pointcloud_pub_->publish(cloud);
  octomap_pointcloud_pub_->publish(cloud);
  octomap_pointcloud_pub_->publish(cloud);
  octomap_pointcloud_pub_->publish(cloud);
  octomap_pointcloud_pub_->publish(cloud);
}

} // namespace botanbot_planner

int main(int argc, char ** argv)
{
  std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

  rclcpp::init(argc, argv);

  auto node = std::make_shared
    <botanbot_planner::BotanbotSE2PlanningControlSpace>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
