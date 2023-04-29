// Copyright (c) 2023 Fetullah Atas, Norwegian University of Life Sciences
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

#include "vox_nav_misc/traversablity_estimator.hpp"

namespace vox_nav_misc
{
  TraversabilityEstimator::TraversabilityEstimator()
  : rclcpp::Node("traversability_estimator_rclcpp_node")
  {
    cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "points",
      rclcpp::SensorDataQoS(),
      std::bind(&TraversabilityEstimator::cloudCallback, this, std::placeholders::_1));

    supervoxel_clusters_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "supervoxel_clusters", rclcpp::SensorDataQoS());

    supervoxel_graph_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "supervoxel_graph", rclcpp::SensorDataQoS());

    traversable_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "traversable_cloud", rclcpp::SensorDataQoS());

    this->declare_parameter("supervoxel_disable_transform", false);
    this->declare_parameter("supervoxel_resolution", 0.8);
    this->declare_parameter("supervoxel_seed_resolution", 1.0);
    this->declare_parameter("supervoxel_color_importance", 0.0);
    this->declare_parameter("supervoxel_spatial_importance", 1.0);
    this->declare_parameter("supervoxel_normal_importance", 1.0);
    declare_parameter("uniform_sample_radius", 0.2);
    declare_parameter("surfel_radius", 0.8);
    declare_parameter("max_allowed_tilt", 40.0);
    declare_parameter("max_allowed_point_deviation", 0.2);
    declare_parameter("max_allowed_energy_gap", 0.2);
    declare_parameter("node_elevation_distance", 0.5);
    declare_parameter("plane_fit_threshold", 0.2);
    declare_parameter("robot_mass", 0.1);
    declare_parameter("average_speed", 1.0);
    declare_parameter("cost_critic_weights", std::vector<double>({0.8, 0.1, 0.1}));

    this->get_parameter("supervoxel_disable_transform", supervoxel_disable_transform_);
    this->get_parameter("supervoxel_resolution", supervoxel_resolution_);
    this->get_parameter("supervoxel_seed_resolution", supervoxel_seed_resolution_);
    this->get_parameter("supervoxel_color_importance", supervoxel_color_importance_);
    this->get_parameter("supervoxel_spatial_importance", supervoxel_spatial_importance_);
    this->get_parameter("supervoxel_normal_importance", supervoxel_normal_importance_);
    get_parameter("uniform_sample_radius", cost_params_.uniform_sample_radius);
    get_parameter("surfel_radius", cost_params_.surfel_radius);
    get_parameter("max_allowed_tilt", cost_params_.max_allowed_tilt);
    get_parameter("max_allowed_point_deviation", cost_params_.max_allowed_point_deviation);
    get_parameter("max_allowed_energy_gap", cost_params_.max_allowed_energy_gap);
    get_parameter("node_elevation_distance", cost_params_.node_elevation_distance);
    get_parameter("plane_fit_threshold", cost_params_.plane_fit_threshold);
    get_parameter("robot_mass", cost_params_.robot_mass);
    get_parameter("average_speed", cost_params_.average_speed);
    get_parameter("cost_critic_weights", cost_params_.cost_critic_weights);
    // setup TF buffer and listerner to read transforms
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(this->get_logger(), "Traversability Estimator Node is up and running!");
  }

  TraversabilityEstimator::~TraversabilityEstimator()
  {
    RCLCPP_INFO(this->get_logger(), "Traversability Estimator Node is shutting down!");
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr TraversabilityEstimator::regressCosts(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
    const std_msgs::msg::Header & header)
  {
    auto input_cloud =
      vox_nav_utilities::set_cloud_color(
      cloud, std::vector<double>({0.0, 255.0, 0.0}));

    auto pure_traversable_pcl = vox_nav_utilities::get_traversable_points(input_cloud);
    auto pure_non_traversable_pcl = vox_nav_utilities::get_non_traversable_points(input_cloud);

    auto uniformly_sampled_nodes = vox_nav_utilities::uniformlySampleCloud<pcl::PointXYZRGB>(
      pure_traversable_pcl,
      cost_params_.uniform_sample_radius /*uniform sampling*/);

    // This is basically vector of cloud segments, each segments includes points representing a cell
    // The first element of pair is surfel_center_point while the second is pointcloud itself
    auto surfels = vox_nav_utilities::surfelize_traversability_cloud(
      pure_traversable_pcl,
      uniformly_sampled_nodes,
      cost_params_.surfel_radius /*Surfel radius*/);

    pcl::PointCloud<pcl::PointXYZRGB> cost_regressed_cloud;

    for (auto && i : surfels) {
      auto surfel_center_point = i.first;
      auto surfel_cloud = i.second;
      // fit a plane to this surfel cloud, in order to et its orientation
      pcl::ModelCoefficients::Ptr plane_model(new pcl::ModelCoefficients);
      try {
        vox_nav_utilities::fit_plane_to_cloud(
          plane_model,
          surfel_cloud,
          cost_params_.plane_fit_threshold /*plane distance threshold*/
        );
      } catch (...) {
        RCLCPP_ERROR(
          get_logger(),
          "Cannot fit a plane to current surfel points, this may occur if cell size is too small");
        RCLCPP_ERROR(
          get_logger(),
          "Current surfel has %d points, Jumping to next surfel", surfel_cloud->points.size());
        continue;
      }

      // extract rpy from plane equation
      auto rpy = vox_nav_utilities::rpy_from_plane(*plane_model);

      // extract averge point deviation from surfel cloud this determines the roughness of cloud
      double average_point_deviation = vox_nav_utilities::average_point_deviation_from_plane(
        surfel_cloud,
        *plane_model);

      // extract max energy grap from surfel cloud, the higher this , the higher cost
      double max_energy_gap = vox_nav_utilities::max_energy_gap_in_cloud(
        surfel_cloud,
        cost_params_.robot_mass,
        cost_params_.average_speed);

      // regulate all costs to be less than 1.0
      double max_tilt = std::max(std::abs(rpy[0]), std::abs(rpy[1]));
      double slope_cost = std::min(max_tilt / cost_params_.max_allowed_tilt, 1.0) *
        cost_params_.max_color_range;

      double energy_gap_cost = std::min(max_energy_gap / cost_params_.max_allowed_energy_gap, 1.0) *
        cost_params_.max_color_range;

      double deviation_of_points_cost = std::min(
        average_point_deviation / cost_params_.max_allowed_point_deviation, 1.0) *
        cost_params_.max_color_range;

      double total_cost =
        cost_params_.cost_critic_weights[0] * slope_cost +
        cost_params_.cost_critic_weights[1] * deviation_of_points_cost +
        cost_params_.cost_critic_weights[2] * energy_gap_cost;

      // any roll or pitch thats higher than max_tilt will make that surfel NON traversable
      if (max_tilt > cost_params_.max_allowed_tilt) {
        surfel_cloud = vox_nav_utilities::set_cloud_color(
          surfel_cloud,
          std::vector<double>({255.0, 0, 0}));
      } else {
        surfel_cloud = vox_nav_utilities::set_cloud_color(
          surfel_cloud,
          std::vector<double>({0.0, cost_params_.max_color_range - total_cost, total_cost}));
      }
      cost_regressed_cloud += *surfel_cloud;
    }

    cost_regressed_cloud += *pure_non_traversable_pcl;

    // Downsample the clouddown
    auto cost_regressed_cloud_final = vox_nav_utilities::downsampleInputCloud<pcl::PointXYZRGB>(
      std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(cost_regressed_cloud),
      0.15);

    // Publish the cost regressor cloud
    sensor_msgs::msg::PointCloud2::SharedPtr cost_regressed_cloud_msg(
      new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(*cost_regressed_cloud_final, *cost_regressed_cloud_msg);
    cost_regressed_cloud_msg->header = header;

    traversable_cloud_publisher_->publish(*cost_regressed_cloud_msg);

    return cost_regressed_cloud_final;

  }

  void TraversabilityEstimator::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Convert ROS Msg to PCL Point Cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *cloud_xyzrgb);

    geometry_msgs::msg::PoseStamped curr_robot_pose;
    vox_nav_utilities::getCurrentPose(
      curr_robot_pose, *tf_buffer_, "odom", "base_link", 0.1);
    // Crop the cloud around the robot
    Eigen::Vector4f min_pt;
    min_pt[0] = curr_robot_pose.pose.position.x - 20.0;
    min_pt[1] = curr_robot_pose.pose.position.y - 20.0;
    min_pt[2] = curr_robot_pose.pose.position.z - 5.0;
    Eigen::Vector4f max_pt;
    max_pt[0] = curr_robot_pose.pose.position.x + 20.0;
    max_pt[1] = curr_robot_pose.pose.position.y + 20.0;
    max_pt[2] = curr_robot_pose.pose.position.z + 5.0;

    cloud_xyzrgb = vox_nav_utilities::cropBox<pcl::PointXYZRGB>(cloud_xyzrgb, min_pt, max_pt);

    // regress cost to cloud;
    cloud_xyzrgb = regressCosts(cloud_xyzrgb, msg->header);

    // remove non-traversable points
    auto pure_traversable_pcl = vox_nav_utilities::get_traversable_points(cloud_xyzrgb);

    // print number of traversable points and cloud_xyzrgb
    RCLCPP_INFO(
      get_logger(),
      "Number of traversable points: %d, Number of points in cloud: %d",
      pure_traversable_pcl->points.size(), cloud_xyzrgb->points.size());

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

    super.extract(supervoxel_clusters_);
    RCLCPP_INFO(get_logger(), "Found %d supervoxels", supervoxel_clusters_.size());

    pcl::PointCloud<pcl::PointXYZL>::Ptr supervoxel_cloud = super.getLabeledVoxelCloud();
    sensor_msgs::msg::PointCloud2 supervoxel_cloud_msg;
    pcl::toROSMsg(*supervoxel_cloud, supervoxel_cloud_msg);
    supervoxel_cloud_msg.header = msg->header;
    supervoxel_clusters_publisher_->publish(supervoxel_cloud_msg);

    // Get the supervoxel adjacency
    std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
    super.getSupervoxelAdjacency(supervoxel_adjacency);

    std_msgs::msg::Header header;
    header.frame_id = msg->header.frame_id;
    header.stamp = msg->header.stamp;
    visualization_msgs::msg::MarkerArray marker_array;
    // Publish empty to reset previous
    fillSuperVoxelMarkersfromAdjacency(
      supervoxel_clusters_, supervoxel_adjacency, header, marker_array);
    supervoxel_graph_publisher_->publish(marker_array);

  }
}   // namespace vox_nav_misc

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vox_nav_misc::TraversabilityEstimator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
