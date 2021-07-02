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

#include "vox_nav_planning/plugins/optimal_elevation_planner.hpp"
#include <pluginlib/class_list_macros.hpp>

#include <string>
#include <memory>
#include <vector>
#include <random>

namespace vox_nav_planning
{

  OptimalElevationPlanner::OptimalElevationPlanner()
  {
  }

  OptimalElevationPlanner::~OptimalElevationPlanner()
  {
  }

  void OptimalElevationPlanner::initialize(
    rclcpp::Node * parent,
    const std::string & plugin_name)
  {
    is_map_ready_ = false;
    se2_bounds_ = std::make_shared<ompl::base::RealVectorBounds>(2);
    z_bounds_ = std::make_shared<ompl::base::RealVectorBounds>(1);
    elevated_surfel_cloud_ = pcl::PointCloud<pcl::PointSurfel>::Ptr(
      new pcl::PointCloud<pcl::PointSurfel>);

    // declare only planner specific parameters here
    // common parameters are declared in server
    parent->declare_parameter(plugin_name + ".state_space_boundries.minx", -10.0);
    parent->declare_parameter(plugin_name + ".state_space_boundries.maxx", 10.0);
    parent->declare_parameter(plugin_name + ".state_space_boundries.miny", -10.0);
    parent->declare_parameter(plugin_name + ".state_space_boundries.maxy", 10.0);
    parent->declare_parameter(plugin_name + ".state_space_boundries.minz", -10.0);
    parent->declare_parameter(plugin_name + ".state_space_boundries.maxz", 10.0);

    parent->get_parameter("interpolation_parameter", interpolation_parameter_);
    parent->get_parameter("octomap_voxel_size", octomap_voxel_size_);

    se2_bounds_->setLow(
      0, parent->get_parameter(plugin_name + ".state_space_boundries.minx").as_double());
    se2_bounds_->setHigh(
      0, parent->get_parameter(plugin_name + ".state_space_boundries.maxx").as_double());
    se2_bounds_->setLow(
      1, parent->get_parameter(plugin_name + ".state_space_boundries.miny").as_double());
    se2_bounds_->setHigh(
      1, parent->get_parameter(plugin_name + ".state_space_boundries.maxy").as_double());
    z_bounds_->setLow(
      0, parent->get_parameter(plugin_name + ".state_space_boundries.minz").as_double());
    z_bounds_->setHigh(
      0, parent->get_parameter(plugin_name + ".state_space_boundries.maxz").as_double());

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
      get_maps_and_surfels_client_node_->create_client
      <vox_nav_msgs::srv::GetMapsAndSurfels>(
      "get_maps_and_surfels");

    super_voxel_adjacency_marker_pub_ =
      parent->create_publisher<visualization_msgs::msg::MarkerArray>(
      "supervoxel_adjacency_markers", rclcpp::SystemDefaultsQoS());

    RCLCPP_INFO(logger_, "Selected planner is: %s", planner_name_.c_str());

    setupMap();

    // WARN elevated_surfel_poses_msg_ needs to be populated by  setupMap();
    state_space_ = std::make_shared<ompl::base::ElevationStateSpace>(
      ompl::base::ElevationStateSpace::SE2,
      elevated_surfel_poses_msg_);

    state_space_->as<ompl::base::ElevationStateSpace>()->setBounds(
      *se2_bounds_,
      *z_bounds_);

    simple_setup_ = std::make_shared<ompl::geometric::SimpleSetup>(state_space_);
    simple_setup_->setStateValidityChecker(
      std::bind(&OptimalElevationPlanner::isStateValid, this, std::placeholders::_1));
  }

  std::vector<geometry_msgs::msg::PoseStamped> OptimalElevationPlanner::createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal)
  {
    if (!is_map_ready_) {
      RCLCPP_WARN(
        logger_, "A valid Octomap has not been recieved yet, Try later again."
      );
      return std::vector<geometry_msgs::msg::PoseStamped>();
    }
    // set the start and goal states
    double start_yaw, goal_yaw, nan;
    vox_nav_utilities::getRPYfromMsgQuaternion(start.pose.orientation, nan, nan, start_yaw);
    vox_nav_utilities::getRPYfromMsgQuaternion(goal.pose.orientation, nan, nan, goal_yaw);

    RCLCPP_INFO(logger_, "Updating search area");

    double radius = vox_nav_utilities::getEuclidianDistBetweenPoses(goal, start) / 2.0;
    auto search_point_pose = vox_nav_utilities::getLinearInterpolatedPose(goal, start);
    auto search_point_surfel = vox_nav_utilities::poseMsg2PCLSurfel(search_point_pose);

    auto search_area_surfels =
      vox_nav_utilities::get_subcloud_within_radius<pcl::PointSurfel>(
      elevated_surfel_cloud_, search_point_surfel,
      radius);

    RCLCPP_INFO(
      logger_, "Updated search area surfels has, %d surfels",
      search_area_surfels->points.size());

    auto search_area_point_cloud =
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);

    for (auto && i : search_area_surfels->points) {
      pcl::PointXYZRGBA point;
      point.x = i.x;
      point.y = i.y;
      point.z = i.z;
      search_area_point_cloud->points.push_back(point);
    }

    bool disable_transform = false;
    float voxel_resolution = 0.8;
    float seed_resolution = 1.0f;
    float color_importance = 0.0f;
    float spatial_importance = 1.0f;
    float normal_importance = 1.0f;

    auto super = vox_nav_utilities::super_voxelize_cloud<pcl::PointXYZRGBA>(
      search_area_point_cloud,
      disable_transform,
      voxel_resolution,
      seed_resolution,
      color_importance,
      spatial_importance,
      normal_importance);

    RCLCPP_INFO(logger_, "Extracting supervoxels!");
    super.extract(supervoxel_clusters_);
    RCLCPP_INFO(logger_, "Found %d supervoxels", supervoxel_clusters_.size());
    RCLCPP_INFO(logger_, "Getting supervoxel adjacency");

    std::multimap<std::uint32_t, std::uint32_t> supervoxel_adjacency;

    super.getSupervoxelAdjacency(supervoxel_adjacency);
    auto voxel_centroid_cloud = super.getVoxelCentroidCloud();
    auto labeled_voxel_cloud = super.getLabeledVoxelCloud();
    std_msgs::msg::Header header;
    header.frame_id = "map";
    header.stamp = rclcpp::Clock().now();

    visualization_msgs::msg::MarkerArray supervoxel_marker_array;
    vox_nav_utilities::fillSuperVoxelMarkersfromAdjacency(
      supervoxel_clusters_, supervoxel_adjacency, header, supervoxel_marker_array);
    super_voxel_adjacency_marker_pub_->publish(supervoxel_marker_array);

    struct VertexProperty
    {
      std::uint32_t label;
      std::string name;
    };

    // specify some types
    typedef boost::adjacency_list<
        boost::setS,            // edge
        boost::vecS,            // vertex
        boost::undirectedS,     // type
        VertexProperty,         // vertex property
        boost::property<boost::edge_weight_t, cost>> // edge property
      GraphT;

    typedef boost::property_map<GraphT, boost::edge_weight_t>::type WeightMap;
    typedef GraphT::vertex_descriptor vertex_descriptor;
    typedef GraphT::edge_descriptor edge_descriptor;
    typedef GraphT::vertex_iterator vertex_iterator;
    typedef std::pair<int, int> edge;

    GraphT g;
    WeightMap weightmap = get(boost::edge_weight, g);
    //Add a vertex for each label, store ids in map
    std::map<std::uint32_t, vertex_descriptor> supervoxel_label_id_map;
    for (auto it = supervoxel_adjacency.cbegin();
      it != supervoxel_adjacency.cend(); )
    {
      std::uint32_t supervoxel_label = it->first;
      vertex_descriptor supervoxel_id = boost::add_vertex(g);
      g[supervoxel_id].label = (supervoxel_label);
      supervoxel_label_id_map.insert(std::make_pair(supervoxel_label, supervoxel_id));
      it = supervoxel_adjacency.upper_bound(supervoxel_label);
    }

    for (auto it = supervoxel_adjacency.cbegin();
      it != supervoxel_adjacency.cend(); )
    {
      std::uint32_t supervoxel_label = it->first;
      auto supervoxel = supervoxel_clusters_.at(supervoxel_label);
      for (auto adjacent_it = supervoxel_adjacency.equal_range(supervoxel_label).first;
        adjacent_it != supervoxel_adjacency.equal_range(supervoxel_label).second; ++adjacent_it)
      {
        std::uint32_t neighbour_supervoxel_label = adjacent_it->second;
        auto neighbour_supervoxel = supervoxel_clusters_.at(neighbour_supervoxel_label);
        edge_descriptor e; bool edge_added;
        vertex_descriptor u = (supervoxel_label_id_map.find(supervoxel_label))->second;
        vertex_descriptor v = (supervoxel_label_id_map.find(neighbour_supervoxel_label))->second;
        boost::tie(e, edge_added) = boost::add_edge(u, v, g);
        //Calc distance between centers, set as edge weight
        if (edge_added) {
          pcl::PointXYZRGBA centroid_data = supervoxel->centroid_;
          pcl::PointXYZRGBA neighbour_centroid_data = neighbour_supervoxel->centroid_;
          float length = vox_nav_utilities::PCLPointEuclideanDist<pcl::PointXYZRGBA>(
            centroid_data,
            neighbour_centroid_data);
          weightmap[e] = length;
        }
      }
      it = supervoxel_adjacency.upper_bound(supervoxel_label);
    }

    RCLCPP_INFO(logger_, "Running astar,");
    RCLCPP_INFO(logger_, "Graph has %d vertices", boost::num_vertices(g));
    RCLCPP_INFO(logger_, "Graph has %d edges", boost::num_edges(g));

    pcl::PointXYZRGBA start_as_pcl_point, goal_as_pcl_point;
    start_as_pcl_point.x = start.pose.position.x;
    start_as_pcl_point.y = start.pose.position.y;
    start_as_pcl_point.z = start.pose.position.z;
    goal_as_pcl_point.x = goal.pose.position.x;
    goal_as_pcl_point.y = goal.pose.position.y;
    goal_as_pcl_point.z = goal.pose.position.z;

    vertex_descriptor st;
    vertex_descriptor gl;

    for (auto itr = supervoxel_label_id_map.begin(); itr != supervoxel_label_id_map.end(); ++itr) {
      double start_dist, goal_dist = INFINITY;
      auto sv_centroid = supervoxel_clusters_.at(itr->first)->centroid_;

      if (vox_nav_utilities::PCLPointEuclideanDist<>(
          start_as_pcl_point, sv_centroid) < start_dist)
      {
        start_dist = vox_nav_utilities::PCLPointEuclideanDist<>(start_as_pcl_point, sv_centroid);
        st = itr->second;
      }

      if (vox_nav_utilities::PCLPointEuclideanDist<>(goal_as_pcl_point, sv_centroid) < goal_dist) {
        goal_dist = vox_nav_utilities::PCLPointEuclideanDist<>(goal_as_pcl_point, sv_centroid);
        gl = itr->second;
      }
    }
    // TO BE CHANGED
    std::vector<vertex_descriptor> p(boost::num_vertices(g));
    std::vector<cost> d(boost::num_vertices(g));
    std::vector<geometry_msgs::msg::PoseStamped> plan_poses;
    ompl::geometric::PathGeometricPtr solution_path =
      std::make_shared<ompl::geometric::PathGeometric>(simple_setup_->getSpaceInformation());
    ompl::geometric::PathSimplifierPtr path_simlifier =
      std::make_shared<ompl::geometric::PathSimplifier>(simple_setup_->getSpaceInformation());

    try {
      // call astar named parameter interface
      auto heuristic = distance_heuristic<GraphT, cost,
          pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>(voxel_centroid_cloud, gl);
      auto visitor = astar_goal_visitor<vertex_descriptor>(gl);
      boost::astar_search_tree(
        g, st,
        heuristic,
        boost::predecessor_map(&p[0]).distance_map(&d[0]).visitor(visitor));
      RCLCPP_WARN(logger_, "AStar failed to find a valid path!");
    } catch (found_goal fg) {    // found a path to the goal
      std::list<vertex_descriptor> shortest_path;
      for (vertex_descriptor v = gl;; v = p[v]) {
        shortest_path.push_front(v);
        if (p[v] == v) {break;}
      }
      std::list<vertex_descriptor>::iterator spi = shortest_path.begin();
      for (++spi; spi != shortest_path.end(); ++spi) {
        std::uint32_t key;
        for (auto & i : supervoxel_label_id_map) {
          if (i.second == *spi) {
            key = i.first;
            break;
          }
        }
        auto state_position = supervoxel_clusters_.at(key)->centroid_;
        auto state = state_space_->allocState();
        auto * compound_state = state->as<ompl::base::ElevationStateSpace::StateType>();
        compound_state->setSE2(state_position.x, state_position.y, 0);
        compound_state->setZ(state_position.z);
        solution_path->append(state);
      }

      solution_path->interpolate(interpolation_parameter_);
      path_simlifier->smoothBSpline(*solution_path, 1, 0.1);

      for (std::size_t path_idx = 0; path_idx < solution_path->getStateCount(); path_idx++) {
        const auto * cstate =
          solution_path->getState(path_idx)->as<ompl::base::ElevationStateSpace::StateType>();
        const auto * se2 = cstate->as<ompl::base::SE2StateSpace::StateType>(0);
        const auto * z = cstate->as<ompl::base::RealVectorStateSpace::StateType>(1);
        tf2::Quaternion this_pose_quat;
        this_pose_quat.setRPY(0, 0, se2->getYaw());
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = start.header.frame_id;
        pose.header.stamp = rclcpp::Clock().now();
        pose.pose.position.x = se2->getX();
        pose.pose.position.y = se2->getY();
        pose.pose.position.z = z->values[0];
        pose.pose.orientation.x = this_pose_quat.getX();
        pose.pose.orientation.y = this_pose_quat.getY();
        pose.pose.orientation.z = this_pose_quat.getZ();
        pose.pose.orientation.w = this_pose_quat.getW();
        plan_poses.push_back(pose);
      }
    }

    RCLCPP_INFO(
      logger_, "Found path with astar %d which includes poses,", plan_poses.size());

    //simple_setup_->clear();
    return plan_poses;
  }

  bool OptimalElevationPlanner::isStateValid(const ompl::base::State * state)
  {
    const auto * cstate = state->as<ompl::base::ElevationStateSpace::StateType>();
    // cast the abstract state type to the type we expect
    const auto * se2 = cstate->as<ompl::base::SE2StateSpace::StateType>(0);
    // extract the second component of the state and cast it to what we expect
    const auto * z = cstate->as<ompl::base::RealVectorStateSpace::StateType>(1);
    fcl::CollisionRequest requestType(1, false, 1, false);
    // check validity of state Fdefined by pos & rot
    fcl::Vec3f translation(se2->getX(), se2->getY(), z->values[0]);
    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY(0, 0, se2->getYaw());
    fcl::Quaternion3f rotation(
      myQuaternion.getX(), myQuaternion.getY(),
      myQuaternion.getZ(), myQuaternion.getW());

    robot_collision_object_->setTransform(rotation, translation);
    fcl::CollisionResult collisionWithSurfelsResult, collisionWithFullMapResult;
    fcl::collide(
      robot_collision_object_.get(),
      elevated_surfels_collision_object_.get(), requestType, collisionWithSurfelsResult);
    fcl::collide(
      robot_collision_object_.get(),
      original_octomap_collision_object_.get(), requestType, collisionWithFullMapResult);

    return collisionWithSurfelsResult.isCollision() && !collisionWithFullMapResult.isCollision();
  }

  void OptimalElevationPlanner::setupMap()
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

  std::vector<geometry_msgs::msg::PoseStamped> OptimalElevationPlanner::getOverlayedStartandGoal()
  {
    std::vector<geometry_msgs::msg::PoseStamped> start_pose_vector;
    start_pose_vector.push_back(nearest_elevated_surfel_to_start_);
    start_pose_vector.push_back(nearest_elevated_surfel_to_goal_);
    return start_pose_vector;
  }
}  // namespace vox_nav_planning

PLUGINLIB_EXPORT_CLASS(vox_nav_planning::OptimalElevationPlanner, vox_nav_planning::PlannerCore)
