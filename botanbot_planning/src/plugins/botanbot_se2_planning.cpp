#include "botanbot_planning/plugins/botanbot_se2_planning.hpp"

namespace botanbot_planning
{
namespace ob = ompl::base;
namespace og = ompl::geometric;
BotanbotSE2Planning::BotanbotSE2Planning()
: Node("botanbot_planning_rclcpp_node"),
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

BotanbotSE2Planning::~BotanbotSE2Planning()
{
}

bool BotanbotSE2Planning::isStateValid(const ompl::base::State * state)
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

void BotanbotSE2Planning::plan()
{
  ob::StateSpacePtr space(std::make_shared<ob::ReedsSheppStateSpace>());

  if (0) {
    space = std::make_shared<ob::ReedsSheppStateSpace>();
  }

  ob::ScopedState<> start(space), goal(space);
  ob::RealVectorBounds bounds(2);
  bounds.setLow(-50);
  bounds.setHigh(50);

  space->as<ob::SE2StateSpace>()->setBounds(bounds);

  // define a simple setup class
  og::SimpleSetup ss(space);

  // set state validity checking for this space
  const ob::SpaceInformation * si = ss.getSpaceInformation().get();
  ss.setStateValidityChecker(
    [this](const ob::State * state)
    {
      return isStateValid(state);
    });

  // set the start and goal states
  start[0] = 0;
  start[1] = 1.0;
  start[2] = 0;
  goal[0] = 12;
  goal[1] = -12.0;
  goal[2] = 0.0;

  ss.setStartAndGoalStates(start, goal);

  // this call is optional, but we put it in to get more output information
  //ss.getSpaceInformation()->setStateValidityCheckingResolution(0.1);
  ss.setup();
  ss.print();

  // attempt to solve the problem within 30 seconds of planning time
  ompl::base::PlannerPtr planner(new ompl::geometric::PRMstar(ss.getSpaceInformation()));
  ss.setPlanner(planner);
  ob::PlannerStatus solved = ss.solve(1);

  if (solved) {
    std::vector<double> reals;

    std::cout << "Found solution:" << std::endl;
    ss.simplifySolution();
    og::PathGeometric path = ss.getSolutionPath();
    path.interpolate(50);

    //Publish path as markers
    visualization_msgs::msg::MarkerArray marker_array;

    for (std::size_t path_idx = 0; path_idx < path.getStateCount(); path_idx++) {

      // cast the abstract state type to the type we expect
      const ompl::base::ReedsSheppStateSpace::StateType * red_state =
        path.getState(path_idx)->as<ompl::base::ReedsSheppStateSpace::StateType>();

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

} // namespace botanbot_planning

int main(int argc, char ** argv)
{
  std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

  rclcpp::init(argc, argv);

  auto node = std::make_shared
    <botanbot_planning::BotanbotSE2Planning>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
