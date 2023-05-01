// Copyright (c) 2023 Norwegian University of Life Sciences, Fetullah Atas
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

#include "vox_nav_control/plan_refiner_plugins/traversability_based_plan_refiner.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace vox_nav_control
{

  TraversabilityBasedPlanRefiner::TraversabilityBasedPlanRefiner()
  {
  }

  TraversabilityBasedPlanRefiner::~TraversabilityBasedPlanRefiner()
  {
  }

  void TraversabilityBasedPlanRefiner::initialize(
    rclcpp::Node * parent,
    const std::string & plugin_name)
  {
    node_ = parent;

    traversability_map_ = std::make_shared<sensor_msgs::msg::PointCloud2>();

    // init tf2 buffer and listener
    tf_buffer_ptr_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ptr_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_ptr_);

    // Bind to traversability map and marker topics
    traversability_map_subscriber_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      "traversable_cloud", rclcpp::SensorDataQoS(),
      std::bind(
        &TraversabilityBasedPlanRefiner::traversabilityMapCallback, this,
        std::placeholders::_1));

    local_goal_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      "local_goal", rclcpp::SensorDataQoS());

    supervoxel_clusters_publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "supervoxel_clusters", rclcpp::SensorDataQoS());

    supervoxel_graph_publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "supervoxel_graph", rclcpp::SensorDataQoS());

    local_optimal_path_publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "local_optimal_path", rclcpp::SensorDataQoS());

    traversability_map_bbox_publisher_ = node_->create_publisher<vision_msgs::msg::Detection3D>(
      "traversability_map_bbox", rclcpp::SensorDataQoS());

    node_->declare_parameter(plugin_name + ".supervoxel_disable_transform", false);
    node_->declare_parameter(plugin_name + ".supervoxel_resolution", 0.25);
    node_->declare_parameter(plugin_name + ".supervoxel_seed_resolution", 0.5);
    node_->declare_parameter(plugin_name + ".supervoxel_color_importance", 0.0);
    node_->declare_parameter(plugin_name + ".supervoxel_spatial_importance", 1.0);
    node_->declare_parameter(plugin_name + ".supervoxel_normal_importance", 1.0);
    node_->declare_parameter(plugin_name + ".traversability_layer_name", "traversability");
    node_->declare_parameter(plugin_name + ".traversability_threshold", 0.5);

    node_->get_parameter(
      plugin_name + ".supervoxel_disable_transform",
      supervoxel_disable_transform_);
    node_->get_parameter(
      plugin_name + ".supervoxel_resolution",
      supervoxel_resolution_);
    node_->get_parameter(
      plugin_name + ".supervoxel_seed_resolution",
      supervoxel_seed_resolution_);
    node_->get_parameter(
      plugin_name + ".supervoxel_color_importance",
      supervoxel_color_importance_);
    node_->get_parameter(
      plugin_name + ".supervoxel_spatial_importance",
      supervoxel_spatial_importance_);
    node_->get_parameter(
      plugin_name + ".supervoxel_normal_importance",
      supervoxel_normal_importance_);
  }

  bool TraversabilityBasedPlanRefiner::refinePlan(
    const geometry_msgs::msg::PoseStamped & curr_pose,
    nav_msgs::msg::Path & plan_to_refine)
  {
    // Convert most recent traversability map to pcl pointcloud
    std::lock_guard<std::mutex> guard(global_mutex_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr traversability_map_pcl(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*traversability_map_, *traversability_map_pcl);

    // Fit a box around the traversability map
    vox_nav_msgs::msg::Object traversability_map_box;
    //vox_nav_utilities::fitBoxtoPointCloud(traversability_map_pcl, traversability_map_box);
    // get min and max values of the traversability map
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*traversability_map_pcl, min_pt, max_pt);
    // get the center of the traversability map
    geometry_msgs::msg::Point center;
    center.x = (min_pt.x + max_pt.x) / 2.0;
    center.y = (min_pt.y + max_pt.y) / 2.0;
    center.z = (min_pt.z + max_pt.z) / 2.0;
    // get the dimensions of the traversability map
    traversability_map_box.header.frame_id = traversability_map_->header.frame_id;
    traversability_map_box.header.stamp = node_->now();
    traversability_map_box.shape.type = shape_msgs::msg::SolidPrimitive::BOX;
    traversability_map_box.shape.dimensions.push_back(max_pt.x - min_pt.x);
    traversability_map_box.shape.dimensions.push_back(max_pt.y - min_pt.y);
    traversability_map_box.shape.dimensions.push_back(max_pt.z - min_pt.z);
    traversability_map_box.pose.position = center;
    traversability_map_box.pose.orientation.w = 1.0;

    // Get the first point outside of the traversability map box on the path
    geometry_msgs::msg::PointStamped closest_point_on_path;
    int closest_point_index = -1;
    int nearest_traj_pose_index = vox_nav_control::common::nearestStateIndex(
      plan_to_refine, curr_pose);

    // Check using FCL if the point is inside the traversability map box
    typedef std::shared_ptr<fcl::CollisionGeometryf> CollisionGeometryPtr_t;
    CollisionGeometryPtr_t map_box(
      new fcl::Box<float>(
        traversability_map_box.shape.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X],
        traversability_map_box.shape.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y],
        traversability_map_box.shape.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z]));
    fcl::CollisionObjectf map_box_object(map_box, fcl::Transform3f());
    fcl::Vector3f map_position(
      curr_pose.pose.position.x,
      curr_pose.pose.position.y,
      curr_pose.pose.position.z
    );
    fcl::Quaternionf map_orientation(
      traversability_map_box.pose.orientation.w,
      traversability_map_box.pose.orientation.x,
      traversability_map_box.pose.orientation.y,
      traversability_map_box.pose.orientation.z);
    map_box_object.setTransform(map_orientation, map_position);

    // publish Traversability Map Bounding Box
    vision_msgs::msg::Detection3D traversability_map_bbox;
    traversability_map_bbox.header.frame_id = "map";
    traversability_map_bbox.header.stamp = node_->now();
    traversability_map_bbox.results.resize(1);
    traversability_map_bbox.bbox.center.position.x = curr_pose.pose.position.x;
    traversability_map_bbox.bbox.center.position.y = curr_pose.pose.position.y;
    traversability_map_bbox.bbox.center.position.z = curr_pose.pose.position.z;
    traversability_map_bbox.bbox.center.orientation.w = 1.0;
    traversability_map_bbox.bbox.center.orientation.w = traversability_map_box.pose.orientation.w;
    traversability_map_bbox.bbox.center.orientation.x = traversability_map_box.pose.orientation.x;
    traversability_map_bbox.bbox.center.orientation.y = traversability_map_box.pose.orientation.y;
    traversability_map_bbox.bbox.center.orientation.z = traversability_map_box.pose.orientation.z;
    traversability_map_bbox.bbox.size.x =
      traversability_map_box.shape.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X];
    traversability_map_bbox.bbox.size.y =
      traversability_map_box.shape.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y];
    traversability_map_bbox.bbox.size.z =
      traversability_map_box.shape.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z];
    traversability_map_bbox.id = 1.0;
    traversability_map_bbox_publisher_->publish(traversability_map_bbox);

    bool traj_endpoint_inside = true;
    for (int i = nearest_traj_pose_index; i < plan_to_refine.poses.size(); i++) {
      geometry_msgs::msg::PoseStamped point = plan_to_refine.poses[i];

      CollisionGeometryPtr_t point_box(new fcl::Box<float>(0.1, 0.1, 1.0));
      fcl::CollisionObjectf point_box_object(point_box, fcl::Transform3f());
      fcl::Vector3f point_position(
        point.pose.position.x,
        point.pose.position.y,
        point.pose.position.z);
      fcl::Quaternionf point_orientatio(1.0, 0.0, 0.0, 0.0);
      point_box_object.setTransform(point_orientatio, point_position);

      fcl::CollisionRequestf request;
      fcl::CollisionResultf result;
      fcl::collide(&map_box_object, &point_box_object, request, result);

      if (result.isCollision()) {
        // If the point is inside the traversability map box, continue
        continue;
      } else {
        // If the point is outside the traversability map box, break
        closest_point_on_path.point = point.pose.position;
        closest_point_index = i;
        traj_endpoint_inside = false;
        break;
      }
    }

    if (traj_endpoint_inside) {
      // If the entire trajectory is inside the traversability map box,
      // The local goal is the last point on the trajectory
      closest_point_on_path.point = plan_to_refine.poses.back().pose.position;
    }

    // Lets publish the closest point on the path to the traversability map as local goal
    geometry_msgs::msg::PoseStamped local_goal;
    local_goal.header.frame_id = plan_to_refine.header.frame_id;
    local_goal.header.stamp = node_->now();
    local_goal.pose.position = closest_point_on_path.point;
    local_goal.pose.orientation = plan_to_refine.poses[closest_point_index].pose.orientation;
    local_goal_publisher_->publish(local_goal);

    // Now construct a local path from the current pose to the local goal using boost graph
    pcl::PointXYZRGBA nn_to_curr_pose, nn_to_local_goal;
    nn_to_curr_pose.x = curr_pose.pose.position.x;
    nn_to_curr_pose.y = curr_pose.pose.position.y;
    nn_to_curr_pose.z = curr_pose.pose.position.z;
    nn_to_local_goal.x = local_goal.pose.position.x;
    nn_to_local_goal.y = local_goal.pose.position.y;
    nn_to_local_goal.z = local_goal.pose.position.z;

    // Get the nearest vertex to the current pose, that will ne the start vertex
    vox_nav_utilities::vertex_descriptor start_vertex = get_nearest(g_, nn_to_curr_pose);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr graph_vertices(new pcl::PointCloud<pcl::PointXYZRGBA>);
    fillCloudfromGraph(g_, graph_vertices);

    std::list<vox_nav_utilities::vertex_descriptor> shortest_path;
    pcl::PointCloud<pcl::PointXYZRGBA> local_optimal_path;

    // Get the radius neighbors of the local goal vertex on the graph
    // see which nearest neighbor can be reached from the current pose
    pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtree;
    kdtree.setInputCloud(graph_vertices);
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    float radius = 3.5;
    pcl::PointXYZRGBA searchPoint = nn_to_local_goal;
    if (kdtree.radiusSearch(
        searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
    {
      shortest_path.clear();
      for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
        auto nn_id = pointIdxRadiusSearch[i];
        auto nn = graph_vertices->points[nn_id];
        vox_nav_utilities::vertex_descriptor goal_vertex = get_nearest(g_, nn);

        local_optimal_path.points.clear();
        // Push start and goal vertices to the local optimal path
        local_optimal_path.points.push_back(graph_vertices->points[start_vertex]);
        local_optimal_path.points.push_back(graph_vertices->points[goal_vertex]);
        // paint the points
        local_optimal_path.points[0].r = 255; local_optimal_path.points[0].a = 255;
        local_optimal_path.points[1].b = 255; local_optimal_path.points[1].a = 255;

        // Now we have a local optimal path from current pose to local goal
        // Lets publish it as a sensor_msgs::PointCloud2
        sensor_msgs::msg::PointCloud2 local_optimal_path_cloud;
        pcl::toROSMsg(local_optimal_path, local_optimal_path_cloud);
        local_optimal_path_cloud.header = traversability_map_->header;
        local_optimal_path_publisher_->publish(local_optimal_path_cloud);


        // See if there is a path from start to goal
        if (find_astar_path(g_, weightmap_, start_vertex, goal_vertex, shortest_path)) {
          break;
        }
      }
    }

    if (shortest_path.empty()) {
      RCLCPP_WARN(node_->get_logger(), "No path found from current pose to local goal");
      return false;
    }

    auto shortest_path_iterator = shortest_path.begin();
    for (++shortest_path_iterator; shortest_path_iterator != shortest_path.end();
      ++shortest_path_iterator)
    {
      auto path_point = g_[*shortest_path_iterator].point;
      // paint the points
      path_point.g = 255; path_point.a = 255;
      local_optimal_path.points.push_back(path_point);
    }

    // Now we have a local optimal path from current pose to local goal
    // Lets publish it as a sensor_msgs::PointCloud2
    sensor_msgs::msg::PointCloud2 local_optimal_path_cloud;
    pcl::toROSMsg(local_optimal_path, local_optimal_path_cloud);
    local_optimal_path_cloud.header = traversability_map_->header;
    local_optimal_path_publisher_->publish(local_optimal_path_cloud);

    RCLCPP_INFO(
      node_->get_logger(),
      "Local optimal path published with %d points", local_optimal_path.points.size());

    return true;
  }

  void TraversabilityBasedPlanRefiner::traversabilityMapCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    std::lock_guard<std::mutex> guard(global_mutex_);
    traversability_map_ = msg;

    // Transform the traversability map to the map frame
    geometry_msgs::msg::TransformStamped map_to_traversability_map_transform;
    try {
      map_to_traversability_map_transform = tf_buffer_ptr_->lookupTransform(
        traversability_map_->header.frame_id, "map",
        rclcpp::Time(0));
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(node_->get_logger(), "%s", ex.what());
      return;
    }
    pcl_ros::transformPointCloud(
      "map", map_to_traversability_map_transform, *traversability_map_, *traversability_map_);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_xyzrgba(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::fromROSMsg(*traversability_map_, *cloud_xyzrgba);

    // remove non-traversable points
    auto pure_traversable_pcl = vox_nav_utilities::get_traversable_points(cloud_xyzrgba);

    // print number of traversable points and cloud_xyzrgb
    RCLCPP_INFO(
      node_->get_logger(),
      "Number of traversable points: %d, Number of points in cloud: %d",
      pure_traversable_pcl->points.size(), cloud_xyzrgba->points.size());

    // Create pointXYZRGBA cloud for supervoxelization from traversable points
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::copyPointCloud(*pure_traversable_pcl, *cloud);

    auto super = vox_nav_utilities::supervoxelizeCloud<pcl::PointXYZRGBA>(
      cloud,
      supervoxel_disable_transform_,
      supervoxel_resolution_,
      supervoxel_seed_resolution_,
      supervoxel_color_importance_,
      supervoxel_spatial_importance_,
      supervoxel_normal_importance_);

    g_.clear();
    weightmap_ = get(boost::edge_weight, g_);

    super.extract(supervoxel_clusters_);
    RCLCPP_INFO(node_->get_logger(), "Found %d supervoxels", supervoxel_clusters_.size());

    pcl::PointCloud<pcl::PointXYZL>::Ptr supervoxel_cloud = super.getLabeledVoxelCloud();
    sensor_msgs::msg::PointCloud2 supervoxel_cloud_msg;
    pcl::toROSMsg(*supervoxel_cloud, supervoxel_cloud_msg);
    supervoxel_cloud_msg.header = msg->header;
    supervoxel_clusters_publisher_->publish(supervoxel_cloud_msg);

    // Get the supervoxel adjacency
    std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
    super.getSupervoxelAdjacency(supervoxel_adjacency);

    // Publish the supervoxel adjacency graph
    std_msgs::msg::Header header;
    header.frame_id = msg->header.frame_id;
    header.stamp = msg->header.stamp;
    visualization_msgs::msg::MarkerArray marker_array;
    // Publish empty to reset previous
    fillSuperVoxelMarkersfromAdjacency(
      supervoxel_clusters_, supervoxel_adjacency, header, marker_array);
    supervoxel_graph_publisher_->publish(marker_array);

    // Create a graph
    // Add a vertex for each label, store ids in a map
    std::map<std::uint32_t, vox_nav_utilities::vertex_descriptor> supervoxel_label_id_map;

    for (auto it = supervoxel_adjacency.cbegin();
      it != supervoxel_adjacency.cend(); )
    {
      std::uint32_t supervoxel_label = it->first;
      vox_nav_utilities::vertex_descriptor supervoxel_id = boost::add_vertex(g_);
      g_[supervoxel_id].label = (supervoxel_label);
      g_[supervoxel_id].point = supervoxel_clusters_.at(supervoxel_label)->centroid_;
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

        vox_nav_utilities::edge_descriptor e; bool edge_added;
        vox_nav_utilities::vertex_descriptor u =
          (supervoxel_label_id_map.find(supervoxel_label))->second;
        vox_nav_utilities::vertex_descriptor v =
          (supervoxel_label_id_map.find(neighbour_supervoxel_label))->second;

        try {
          boost::tie(e, edge_added) = boost::add_edge(u, v, g_);
        } catch (const std::exception & e) {
          RCLCPP_ERROR(
            node_->get_logger(),
            "Caught exception while adding edge between %d and %d: %s",
            supervoxel_label, neighbour_supervoxel_label, e.what());
          continue;
        }

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
          weightmap_[e] = 0.5 * absolute_distance + 0.5 * absolute_elevation;
        }
      }
      it = supervoxel_adjacency.upper_bound(supervoxel_label);
    }

    RCLCPP_INFO(
      node_->get_logger(),
      "Constructed a Boost Graph from supervoxel clustering with %d vertices and %d edges",
      boost::num_vertices(g_),
      boost::num_edges(g_));
  }

}      // namespace vox_nav_control

PLUGINLIB_EXPORT_CLASS(
  vox_nav_control::TraversabilityBasedPlanRefiner,
  vox_nav_control::PlanRefinerCore)
