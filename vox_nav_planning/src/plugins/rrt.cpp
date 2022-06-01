#include "vox_nav_planning/plugins/rrt.hpp"

namespace vox_nav_planning
{

  RRT::RRT(
    Node start,
    Node goal,
    double expand_dis,
    double path_resolution,
    double goal_sample_rate,
    double max_iter,
    double robot_radius)
  : start_(start),
    goal_(goal),
    expand_dis_(expand_dis),
    path_resolution_(path_resolution),
    goal_sample_rate_(goal_sample_rate),
    max_iter_(max_iter),
    robot_radius_(robot_radius)
  {
    if (!nodes_) {
      nodes_ = pcl::make_shared<pcl::PointCloud<Node>>();
    }
  }

  void RRT::initialize(
    rclcpp::Node * parent,
    const std::string & plugin_name)
  {
    is_map_ready_ = false;

    elevated_surfel_cloud_ = pcl::PointCloud<pcl::PointSurfel>::Ptr(
      new pcl::PointCloud<pcl::PointSurfel>);

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
    parent->declare_parameter(plugin_name + ".state_space_boundries.minv", -1.5);
    parent->declare_parameter(plugin_name + ".state_space_boundries.maxv", 1.5);
    parent->declare_parameter(plugin_name + ".control_boundries.minv", -0.5);
    parent->declare_parameter(plugin_name + ".control_boundries.maxv", 0.5);
    parent->declare_parameter(plugin_name + ".control_boundries.minw", -0.5);
    parent->declare_parameter(plugin_name + ".control_boundries.maxw", 0.5);

    parent->get_parameter("planner_name", planner_name_);
    parent->get_parameter("planner_timeout", planner_timeout_);
    parent->get_parameter("interpolation_parameter", interpolation_parameter_);
    parent->get_parameter("octomap_voxel_size", octomap_voxel_size_);

    typedef std::shared_ptr<fcl::CollisionGeometry> CollisionGeometryPtr_t;
    CollisionGeometryPtr_t robot_body_box(new fcl::Box(
        parent->get_parameter("robot_body_dimens.x").as_double(),
        parent->get_parameter("robot_body_dimens.y").as_double(),
        parent->get_parameter("robot_body_dimens.z").as_double()));

    fcl::CollisionObject robot_body_box_object(robot_body_box, fcl::Transform3f());

    robot_collision_object_ = std::make_shared<fcl::CollisionObject>(robot_body_box_object);

    elevated_surfel_octomap_octree_ = std::make_shared<octomap::OcTree>(octomap_voxel_size_);
    original_octomap_octree_ = std::make_shared<octomap::OcTree>(octomap_voxel_size_);

    get_maps_and_surfels_client_node_ = std::make_shared
      <rclcpp::Node>("get_maps_and_surfels_client_node");

    get_maps_and_surfels_client_ =
      get_maps_and_surfels_client_node_->create_client<vox_nav_msgs::srv::GetMapsAndSurfels>(
      "get_maps_and_surfels");

    setupMap();

  }

  RRT::~RRT()
  {
  }

  int RRT::nearestNodeIndex(const pcl::PointCloud<Node>::Ptr nodes, const Node random_node)
  {
    pcl::KdTreeFLANN<Node> kdtree;
    kdtree.setInputCloud(nodes);
    // K nearest neighbor search
    int K = 1;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    if (kdtree.nearestKSearch(random_node, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
      return pointIdxNKNSearch[0];
    } else {
      return -1;
    }
  }

  bool RRT::isStateValid(const Node * state)
  {
    fcl::CollisionRequest requestType(1, false, 1, false);
    // check validity of state Fdefined by pos & rot
    fcl::Vec3f translation(state->x, state->y, state->z);
    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY(0, 0, state->psi);
    fcl::Quaternion3f rotation(
      myQuaternion.getX(), myQuaternion.getY(),
      myQuaternion.getZ(), myQuaternion.getW());
    robot_collision_object_->setTransform(rotation, translation);
    fcl::CollisionResult collisionWithFullMapResult;
    fcl::collide(
      robot_collision_object_.get(),
      original_octomap_collision_object_.get(), requestType, collisionWithFullMapResult);
    return !collisionWithFullMapResult.isCollision();
  }

  std::vector<geometry_msgs::msg::PoseStamped> RRT::createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal)
  {
    if (!is_map_ready_) {
      RCLCPP_WARN(
        logger_, "A valid Octomap has not been recived yet, Try later again."
      );
      return std::vector<geometry_msgs::msg::PoseStamped>();
    }
    // set the start and goal states
    double start_yaw, goal_yaw, nan;
    vox_nav_utilities::getRPYfromMsgQuaternion(start.pose.orientation, nan, nan, start_yaw);
    vox_nav_utilities::getRPYfromMsgQuaternion(goal.pose.orientation, nan, nan, goal_yaw);

    nodes_->points.push_back(start_);

    auto valid_sampler = std::make_shared<OctoCellValidStateSampler>(
      start,
      goal,
      elevated_surfel_poses_msg_);

    for (size_t i = 0; i < max_iter_; i++) {

    }

    std::vector<geometry_msgs::msg::PoseStamped> plan_poses;

    return plan_poses;
  }

  void RRT::setupMap()
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
      elevated_surfel_poses_msg_ = std::make_shared<geometry_msgs::msg::PoseArray>(
        response->elevated_surfel_poses);
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
    }
  }

}
int main(int argc, char const * argv[])
{
  pcl::PointCloud<Node> cloud;
  cloud.points.resize(2);
  cloud.width = 2;
  cloud.height = 1;

  cloud.points[0].v = 1;
  cloud.points[1].psi = 2;
  cloud.points[0].x = cloud.points[0].y = cloud.points[0].z = 0;
  cloud.points[1].x = cloud.points[1].y = cloud.points[1].z = 3;
  return 0;
}
