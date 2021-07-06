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
    parent->declare_parameter(plugin_name + ".supervoxel_disable_transform", false);
    parent->declare_parameter(plugin_name + ".supervoxel_resolution", 0.8);
    parent->declare_parameter(plugin_name + ".supervoxel_seed_resolution", 1.0);
    parent->declare_parameter(plugin_name + ".supervoxel_color_importance", 0.0);
    parent->declare_parameter(plugin_name + ".supervoxel_spatial_importance", 1.0);
    parent->declare_parameter(plugin_name + ".supervoxel_normal_importance", 1.0);
    parent->declare_parameter(plugin_name + ".distance_penalty_weight", 1.0);
    parent->declare_parameter(plugin_name + ".elevation_penalty_weight", 1.0);
    parent->declare_parameter(plugin_name + ".graph_search_method", "astar");

    parent->get_parameter("interpolation_parameter", interpolation_parameter_);
    parent->get_parameter("octomap_voxel_size", octomap_voxel_size_);
    parent->get_parameter(
      plugin_name + ".supervoxel_disable_transform", supervoxel_disable_transform_);
    parent->get_parameter(
      plugin_name + ".supervoxel_resolution", supervoxel_resolution_);
    parent->get_parameter(
      plugin_name + ".supervoxel_seed_resolution", supervoxel_seed_resolution_);
    parent->get_parameter(
      plugin_name + ".supervoxel_color_importance", supervoxel_color_importance_);
    parent->get_parameter(
      plugin_name + ".supervoxel_spatial_importance", supervoxel_spatial_importance_);
    parent->get_parameter(
      plugin_name + ".supervoxel_normal_importance", supervoxel_normal_importance_);
    parent->get_parameter(plugin_name + ".distance_penalty_weight", distance_penalty_weight_);
    parent->get_parameter(plugin_name + ".elevation_penalty_weight", elevation_penalty_weight_);
    parent->get_parameter(plugin_name + ".graph_search_method", graph_search_method_);

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

    RCLCPP_INFO(
      logger_,
      "Selected planner is: %s optimal planner, this dos not bases on OMPL ",
      graph_search_method_.c_str());
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
    // get a stamp of time to calculate how much time this function costs
    auto t1 = std::chrono::high_resolution_clock::now();

    if (!is_map_ready_) {
      RCLCPP_WARN(logger_, "A valid Octomap has not been recieved yet !, Try later again.");
      return std::vector<geometry_msgs::msg::PoseStamped>();
    }

    double radius = vox_nav_utilities::getEuclidianDistBetweenPoses(goal, start) / 2.0;
    auto search_point_pose = vox_nav_utilities::getLinearInterpolatedPose(goal, start);
    auto search_point_surfel = vox_nav_utilities::poseMsg2PCLSurfel(search_point_pose);

    // we determined a search point and radius, now with this info lets get
    // a sub point cloud that falls within boundries
    auto search_area_surfels =
      vox_nav_utilities::get_subcloud_within_radius<pcl::PointSurfel>(
      elevated_surfel_cloud_,
      search_point_surfel,
      radius);

    auto search_area_rgba_pointcloud =
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
    // Fill surfels that are in serach space as XYZRGBA pointcloud
    // This is required by pcl::SupervoxelClustering
    for (auto && i : search_area_surfels->points) {
      // neglect rgba fields
      pcl::PointXYZRGBA point;
      point.x = i.x;
      point.y = i.y;
      point.z = i.z;
      search_area_rgba_pointcloud->points.push_back(point);
    }

    auto super = vox_nav_utilities::super_voxelize_cloud<pcl::PointXYZRGBA>(
      search_area_rgba_pointcloud,
      supervoxel_disable_transform_,
      supervoxel_resolution_,
      supervoxel_seed_resolution_,
      supervoxel_color_importance_,
      supervoxel_spatial_importance_,
      supervoxel_normal_importance_);

    RCLCPP_INFO(logger_, "Extracting supervoxels!");
    super.extract(supervoxel_clusters_);
    RCLCPP_INFO(logger_, "Found %d supervoxels", supervoxel_clusters_.size());
    std::multimap<std::uint32_t, std::uint32_t> supervoxel_adjacency;
    super.getSupervoxelAdjacency(supervoxel_adjacency);

    // Lets visualize supervxoel centroids and its adjacency
    // Yeah this looks cool but certainly computationally expensive
    std_msgs::msg::Header header;
    header.frame_id = "map";
    header.stamp = rclcpp::Clock().now();
    visualization_msgs::msg::MarkerArray marker_array;
    // Publish empty to reset previous
    super_voxel_adjacency_marker_pub_->publish(marker_array);
    /*vox_nav_utilities::fillSuperVoxelMarkersfromAdjacency(
      supervoxel_clusters_, supervoxel_adjacency, header, marker_array);*/

    /////////////////////////////////////////////////////////////////////////////////////////////////
    // CAREFUL , GONNA BE FUNCTIONIZED
    int index = 0;
    // To make a graph of the supervoxel adjacency,
    // we need to iterate through the supervoxel adjacency multimap
    for (auto label_itr = supervoxel_adjacency.cbegin();
      label_itr != supervoxel_adjacency.cend(); )
    {
      // First get the label
      std::uint32_t supervoxel_label = label_itr->first;
      // Now get the supervoxel corresponding to the label
      auto supervoxel = supervoxel_clusters_.at(supervoxel_label);
      visualization_msgs::msg::Marker line_strip;
      line_strip.header = header;
      line_strip.ns = "supervoxel_markers_ns";
      line_strip.id = index;
      line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
      line_strip.action = visualization_msgs::msg::Marker::ADD;
      line_strip.scale.x = 0.1;
      geometry_msgs::msg::Point point;
      point.x = supervoxel->centroid_.x;
      point.y = supervoxel->centroid_.y;
      point.z = supervoxel->centroid_.z;
      std_msgs::msg::ColorRGBA yellow_color;
      yellow_color.r = 1.0;
      yellow_color.g = 1.0;
      yellow_color.a = 0.4;

      visualization_msgs::msg::Marker sphere;
      sphere.header = header;
      sphere.ns = "supervoxel_markers_ns";
      sphere.id = index + 10000;
      sphere.type = visualization_msgs::msg::Marker::SPHERE;
      sphere.action = visualization_msgs::msg::Marker::ADD;
      sphere.pose.position = point;
      sphere.scale.x = 0.3;
      sphere.scale.y = 0.3;
      sphere.scale.z = 0.3;
      sphere.color.a = 1.0;
      sphere.color.g = 1.0;
      sphere.color.b = 1.0;

      for (auto adjacent_itr = supervoxel_adjacency.equal_range(supervoxel_label).first;
        adjacent_itr != supervoxel_adjacency.equal_range(supervoxel_label).second; ++adjacent_itr)
      {
        auto neighbour_supervoxel = supervoxel_clusters_.at(adjacent_itr->second);
        if (isEdgeinCollision(supervoxel->centroid_, neighbour_supervoxel->centroid_)) {
          continue;
        }
        geometry_msgs::msg::Point n_point;
        n_point.x = neighbour_supervoxel->centroid_.x;
        n_point.y = neighbour_supervoxel->centroid_.y;
        n_point.z = neighbour_supervoxel->centroid_.z;
        line_strip.points.push_back(point);
        line_strip.colors.push_back(yellow_color);
        line_strip.points.push_back(n_point);
        line_strip.colors.push_back(yellow_color);
      }
      // Move iterator forward to next label
      label_itr = supervoxel_adjacency.upper_bound(supervoxel_label);
      index++;
      marker_array.markers.push_back(sphere);
      marker_array.markers.push_back(line_strip);
    }
    // CAREFUL , GONNA BE FUNCTIONIZED
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    super_voxel_adjacency_marker_pub_->publish(marker_array);

    // lets construct a boost::graph of supervoxels and the adjacency of them
    // we can then use all boost::graph algortihms on this graph
    // edge weights are set as distances or elevations, see the configration to
    // adjust the weights of penalties
    GraphT g;
    WeightMap weightmap = get(boost::edge_weight, g);
    //Add a vertex for each label, store ids in a map
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

    // fill edges acquired from supervoxel clustering
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
        if (isEdgeinCollision(supervoxel->centroid_, neighbour_supervoxel->centroid_)) {
          continue;
        }
        edge_descriptor e; bool edge_added;
        vertex_descriptor u = (supervoxel_label_id_map.find(supervoxel_label))->second;
        vertex_descriptor v = (supervoxel_label_id_map.find(neighbour_supervoxel_label))->second;
        boost::tie(e, edge_added) = boost::add_edge(u, v, g);
        // Calc distance between centers, set this as edge weight
        // the more distane the heavier final cost
        if (edge_added) {
          pcl::PointXYZRGBA centroid_data = supervoxel->centroid_;
          pcl::PointXYZRGBA neighbour_centroid_data = neighbour_supervoxel->centroid_;

          float absolute_distance = vox_nav_utilities::PCLPointEuclideanDist<>(
            centroid_data,
            neighbour_centroid_data);
          // Lets also add elevation as weight
          float absolute_elevation = std::abs(centroid_data.z - neighbour_centroid_data.z);
          weightmap[e] = distance_penalty_weight_ * absolute_distance +
            elevation_penalty_weight_ * absolute_elevation;
        }
      }
      it = supervoxel_adjacency.upper_bound(supervoxel_label);
    }

    RCLCPP_INFO(
      logger_,
      "Constructed a Boost Graph from supervoxel clustering with %d vertices and %d edges",
      boost::num_vertices(g),
      boost::num_edges(g));

    pcl::PointXYZRGBA start_as_pcl_point, goal_as_pcl_point;
    start_as_pcl_point.x = start.pose.position.x;
    start_as_pcl_point.y = start.pose.position.y;
    start_as_pcl_point.z = start.pose.position.z;
    goal_as_pcl_point.x = goal.pose.position.x;
    goal_as_pcl_point.y = goal.pose.position.y;
    goal_as_pcl_point.z = goal.pose.position.z;

    // Match requested start and goal poses with valid vertexes on Graph
    vertex_descriptor start_vertex, goal_vertex;

    // Simple O(N) algorithm to find closest vertex to start and goal poses on boost::graph g
    double start_dist_min = INFINITY, goal_dist_min = INFINITY;
    for (auto itr = supervoxel_label_id_map.begin(); itr != supervoxel_label_id_map.end(); ++itr) {

      auto voxel_centroid = supervoxel_clusters_.at(itr->first)->centroid_;
      auto start_dist_to_crr_voxel_centroid =
        vox_nav_utilities::PCLPointEuclideanDist<>(start_as_pcl_point, voxel_centroid);
      auto goal_dist_to_crr_voxel_centroid =
        vox_nav_utilities::PCLPointEuclideanDist<>(goal_as_pcl_point, voxel_centroid);

      if (start_dist_to_crr_voxel_centroid < start_dist_min) {
        start_dist_min = start_dist_to_crr_voxel_centroid;
        start_vertex = itr->second;
      }
      if (goal_dist_to_crr_voxel_centroid < goal_dist_min) {
        goal_dist_min = goal_dist_to_crr_voxel_centroid;
        goal_vertex = itr->second;
      }
    }
    std::vector<vertex_descriptor> p(boost::num_vertices(g));
    std::vector<Cost> d(boost::num_vertices(g));
    std::vector<geometry_msgs::msg::PoseStamped> plan_poses;
    ompl::geometric::PathGeometricPtr solution_path =
      std::make_shared<ompl::geometric::PathGeometric>(simple_setup_->getSpaceInformation());
    ompl::geometric::PathSimplifierPtr path_simlifier =
      std::make_shared<ompl::geometric::PathSimplifier>(simple_setup_->getSpaceInformation());
    RCLCPP_INFO(
      logger_, "Running %s search on Constructed a Boost Graph", graph_search_method_.c_str());
    auto a1 = std::chrono::high_resolution_clock::now();

    int num_visited_nodes = 0;
    try {
      auto heuristic =
        distance_heuristic<GraphT, Cost, SuperVoxelClusters *>(
        &supervoxel_clusters_, goal_vertex, g);
      auto c_visitor = custom_goal_visitor<vertex_descriptor>(goal_vertex, &num_visited_nodes);

      if (graph_search_method_ == "dijkstra") {
        boost::dijkstra_shortest_paths(
          g, start_vertex,
          boost::predecessor_map(&p[0]).distance_map(&d[0]).visitor(c_visitor));
      } else { // astar
        boost::astar_search_tree(
          g, start_vertex, heuristic /*only difference*/,
          boost::predecessor_map(&p[0]).distance_map(&d[0]).visitor(c_visitor));
      }

      // If a path found exception will be throns and code block here
      // Should not be eecuted. If code executed up until here,
      // A path was NOT found. Warn user about it
      RCLCPP_WARN(logger_, "%s search failed to find a valid path!", graph_search_method_.c_str());
    } catch (FoundGoal found_goal) {
      auto a2 = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double, std::milli> graph_search_ms_double = a2 - a1;
      RCLCPP_INFO(
        logger_, "Pure %s graph search took %.4f milliseconds.",
        graph_search_method_.c_str(), graph_search_ms_double.count());

      // Found a path to the goal, catch the exception
      std::list<vertex_descriptor> shortest_path;
      for (vertex_descriptor v = goal_vertex;; v = p[v]) {
        shortest_path.push_front(v);
        if (p[v] == v) {break;}
      }
      auto shortest_path_iterator = shortest_path.begin();
      for (++shortest_path_iterator; shortest_path_iterator != shortest_path.end();
        ++shortest_path_iterator)
      {
        // Match vertex_descriptor of current vertex in shortess_path to its eqiuvalent
        // In supervoxel_label_id_map, do this to get
        std::uint32_t label;
        for (auto & i : supervoxel_label_id_map) {
          if (i.second == *shortest_path_iterator) {
            label = g[*shortest_path_iterator].label;
            break;
          }
        }
        // Fill the solution vertex to OMPL path
        // tis is needed for path smoothing and interpolation
        auto solution_state_position = supervoxel_clusters_.at(label)->centroid_;
        auto solution_state = state_space_->allocState();
        auto * compound_elevation_state =
          solution_state->as<ompl::base::ElevationStateSpace::StateType>();
        compound_elevation_state->setSE2(
          solution_state_position.x, solution_state_position.y,
          0 /*assume a 0 yaw here*/);
        compound_elevation_state->setZ(solution_state_position.z);
        solution_path->append(compound_elevation_state);
      }
      /*solution_path->interpolate(interpolation_parameter_);*/    /*WARN TAKES A LOT OF TIME*/
      /*path_simlifier->smoothBSpline(*solution_path, 1, 1.0);*/   /*WARN TAKES A LOT OF TIME*/

      for (std::size_t path_idx = 0; path_idx < solution_path->getStateCount(); path_idx++) {
        const auto * compound_elevation_state =
          solution_path->getState(path_idx)->as<ompl::base::ElevationStateSpace::StateType>();
        const auto * se2 = compound_elevation_state->as<ompl::base::SE2StateSpace::StateType>(0);
        const auto * z =
          compound_elevation_state->as<ompl::base::RealVectorStateSpace::StateType>(1);
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = start.header.frame_id;
        pose.header.stamp = rclcpp::Clock().now();
        pose.pose.position.x = se2->getX();
        pose.pose.position.y = se2->getY();
        pose.pose.position.z = z->values[0];
        plan_poses.push_back(pose);
      }
    }

    for (size_t i = 1; i < plan_poses.size(); i++) {
      double dx = plan_poses[i - 1].pose.position.x - plan_poses[i].pose.position.x;
      double dy = plan_poses[i - 1].pose.position.y - plan_poses[i].pose.position.y;
      double dz = plan_poses[i - 1].pose.position.z - plan_poses[i].pose.position.z;
      double roll, pitch, yaw;
      yaw = std::atan2(dy, dx);
      pitch = -std::atan2(dz, std::sqrt(dx * dx + dy * dy));
      roll = 0 /*std::atan2(dz, dy)*/;
      plan_poses[i].pose.orientation = vox_nav_utilities::getMsgQuaternionfromRPY(roll, pitch, yaw);
    }

    RCLCPP_INFO(
      logger_, "A total of %d vertices were visited from a Boost Graph", num_visited_nodes);
    RCLCPP_INFO(
      logger_, "Found path with %s search %d which includes poses,",
      graph_search_method_.c_str(), plan_poses.size());

    auto t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> ms_double = t2 - t1;
    RCLCPP_INFO(
      logger_, "Whole %s path finding function took %.4f milliseconds.",
      graph_search_method_.c_str(), ms_double.count());

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

  bool OptimalElevationPlanner::isEdgeinCollision(
    const pcl::PointXYZRGBA & a,
    const pcl::PointXYZRGBA & b)
  {
    fcl::Vec3f edge_center(
      (a.x + b.x) / 2.0,
      (a.y + b.y) / 2.0,
      (a.z + b.z) / 2.0);

    double dx = a.x - b.x;
    double dy = a.y - b.y;
    double dz = a.z - b.z;

    double roll, pitch, yaw;
    roll = std::atan2((dz), std::sqrt(dy * dy + dx * dx));
    pitch = 0;
    yaw = std::atan2(dx, dy);

    tf2::Quaternion quat;
    quat.setRPY(roll, pitch, yaw);
    fcl::Quaternion3f rotation(quat.getX(), quat.getY(), quat.getZ(), quat.getW());

    robot_collision_object_->setTransform(rotation, edge_center);

    fcl::CollisionResult collisionWithFullMapResult;
    fcl::CollisionRequest requestType(1, false, 1, false);
    fcl::collide(
      robot_collision_object_.get(),
      original_octomap_collision_object_.get(), requestType, collisionWithFullMapResult);
    return collisionWithFullMapResult.isCollision();
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
