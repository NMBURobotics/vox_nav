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
    traversability_marker_ = std::make_shared<visualization_msgs::msg::MarkerArray>();

    // init tf2 buffer and listener
    tf_buffer_ptr_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ptr_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_ptr_);

    // Bind to traversability map and marker topics
    traversability_map_subscriber_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      "traversable_cloud", rclcpp::SensorDataQoS(),
      std::bind(
        &TraversabilityBasedPlanRefiner::traversabilityMapCallback, this,
        std::placeholders::_1));

    traversability_marker_subscriber_ =
      node_->create_subscription<visualization_msgs::msg::MarkerArray>(
      "supervoxel_graph", rclcpp::SensorDataQoS(),
      std::bind(
        &TraversabilityBasedPlanRefiner::traversabilityMarkerCallback, this,
        std::placeholders::_1));

    local_goal_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      "local_goal", rclcpp::SensorDataQoS());

    node_->declare_parameter(plugin_name + ".traversability_layer_name", "traversability");
    node_->declare_parameter(plugin_name + ".traversability_threshold", 0.5);
    node_->get_parameter(plugin_name + ".traversability_layer_name", traversability_layer_name_);
    node_->get_parameter(plugin_name + ".traversability_threshold", traversability_threshold_);
  }

  bool TraversabilityBasedPlanRefiner::refinePlan(
    const geometry_msgs::msg::PoseStamped & curr_pose,
    nav_msgs::msg::Path & plan_to_refine)
  {
    // Get the closest point on the path to edge of the traversability map
    // and check if it is traversable

    // Convert most recent traversability map to pcl pointcloud
    std::lock_guard<std::mutex> guard(global_mutex_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr traversability_map_pcl(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*traversability_map_, *traversability_map_pcl);

    // Transform the traversability map to the map frame
    geometry_msgs::msg::TransformStamped map_to_traversability_map_transform;
    try {
      map_to_traversability_map_transform = tf_buffer_ptr_->lookupTransform(
        traversability_map_->header.frame_id, "map",
        rclcpp::Time(0));
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(node_->get_logger(), "%s", ex.what());
      return false;
    }
    pcl_ros::transformPointCloud(
      *traversability_map_pcl, *traversability_map_pcl, map_to_traversability_map_transform);

    // Fit a box around the traversability map
    vox_nav_msgs::msg::Object traversability_map_box;
    vox_nav_utilities::fitBoxtoPointCloud(traversability_map_pcl, traversability_map_box);

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
      traversability_map_box.pose.position.x,
      traversability_map_box.pose.position.y,
      traversability_map_box.pose.position.z);
    fcl::Quaternionf map_orientation(
      traversability_map_box.pose.orientation.w,
      traversability_map_box.pose.orientation.x,
      traversability_map_box.pose.orientation.y,
      traversability_map_box.pose.orientation.z);
    map_box_object.setTransform(map_orientation, map_position);

    for (int i = nearest_traj_pose_index; i < plan_to_refine.poses.size(); i++) {
      geometry_msgs::msg::PoseStamped point = plan_to_refine.poses[i];

      CollisionGeometryPtr_t point_box(new fcl::Box<float>(0.1, 0.1, 0.1));
      fcl::CollisionObjectf point_box_object(point_box, fcl::Transform3f());
      fcl::Vector3f point_position(
        point.pose.position.x,
        point.pose.position.y,
        point.pose.position.z);
      fcl::Quaternionf point_orientatio(0.0, 0.0, 0.0, 1.0);
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
        break;
      }
    }

    // Lets publish the closest point on the path to the traversability map as local goal
    geometry_msgs::msg::PoseStamped local_goal;
    local_goal.header.frame_id = plan_to_refine.header.frame_id;
    local_goal.header.stamp = node_->now();
    local_goal.pose.position = closest_point_on_path.point;
    local_goal.pose.orientation = plan_to_refine.poses[closest_point_index].pose.orientation;
    local_goal_publisher_->publish(local_goal);

    return true;
  }

  void TraversabilityBasedPlanRefiner::traversabilityMapCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    std::lock_guard<std::mutex> guard(global_mutex_);
    traversability_map_ = msg;
  }

  void TraversabilityBasedPlanRefiner::traversabilityMarkerCallback(
    const visualization_msgs::msg::MarkerArray::SharedPtr msg)
  {
    traversability_marker_ = msg;
  }

}      // namespace vox_nav_control

PLUGINLIB_EXPORT_CLASS(
  vox_nav_control::TraversabilityBasedPlanRefiner,
  vox_nav_control::PlanRefinerCore)
