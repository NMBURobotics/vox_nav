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

#include "vox_nav_map_server/map_manager.hpp"

#include <string>
#include <vector>
#include <memory>
#include <algorithm>

namespace vox_nav_map_server
{
MapManager::MapManager()
: Node("vox_nav_map_manager_rclcpp_node"),
  map_configured_(false)
{
  RCLCPP_INFO(this->get_logger(), "Creating..");
  // initialize shared pointers asap
  pcd_map_gps_pose_ = std::make_shared<vox_nav_msgs::msg::OrientedNavSatFix>();
  original_octomap_msg_ = std::make_shared<octomap_msgs::msg::Octomap>();
  elevated_surfel_octomap_msg_ = std::make_shared<octomap_msgs::msg::Octomap>();
  elevated_surfel_poses_msg_ = std::make_shared<geometry_msgs::msg::PoseArray>();
  octomap_pointcloud_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
  octomap_markers_msg_ = std::make_shared<visualization_msgs::msg::MarkerArray>();
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Declare this node's parameters
  declare_parameter("pcd_map_filename", "/home/ros2-foxy/f.pcd");
  declare_parameter("octomap_voxel_size", 0.2);
  declare_parameter("octomap_publish_frequency", 10);
  declare_parameter("publish_octomap_visuals", true);
  declare_parameter("octomap_point_cloud_publish_topic", "octomap_pointcloud");
  declare_parameter("octomap_markers_publish_topic", "octomap_markers");
  declare_parameter("map_frame_id", "map");
  declare_parameter("utm_frame_id", "utm");
  declare_parameter("yaw_offset", 1.57);
  declare_parameter("map_coordinates.latitude", 49.0);
  declare_parameter("map_coordinates.longitude", 3.0);
  declare_parameter("map_coordinates.altitude", 0.5);
  declare_parameter("map_coordinates.quaternion.x", 0.0);
  declare_parameter("map_coordinates.quaternion.y", 0.0);
  declare_parameter("map_coordinates.quaternion.z", 0.0);
  declare_parameter("map_coordinates.quaternion.w", 1.0);
  declare_parameter("pcd_map_transform.translation.x", 0.0);
  declare_parameter("pcd_map_transform.translation.y", 0.0);
  declare_parameter("pcd_map_transform.translation.z", 0.0);
  declare_parameter("pcd_map_transform.rotation.r", 0.0);
  declare_parameter("pcd_map_transform.rotation.p", 0.0);
  declare_parameter("pcd_map_transform.rotation.y", 0.0);
  declare_parameter("apply_filters", true);
  declare_parameter("pcd_map_downsample_voxel_size", 0.1);
  declare_parameter("remove_outlier_mean_K", 10);
  declare_parameter("remove_outlier_stddev_threshold", 1.0);
  declare_parameter("remove_outlier_radius_search", 0.1);
  declare_parameter("remove_outlier_min_neighbors_in_radius", 1);
  declare_parameter("cell_radius", 0.8);
  declare_parameter("max_allowed_tilt", 40.0);
  declare_parameter("max_allowed_point_deviation", 0.2);
  declare_parameter("max_allowed_energy_gap", 0.2);
  declare_parameter("node_elevation_distance", 0.5);
  declare_parameter("plane_fit_threshold", 0.2);
  declare_parameter("robot_mass", 0.1);
  declare_parameter("average_speed", 1.0);
  declare_parameter("include_node_centers_in_cloud", true);
  declare_parameter("cost_critic_weights", std::vector<double>({0.8, 0.1, 0.1}));

  // get this node's parameters
  get_parameter("pcd_map_filename", pcd_map_filename_);
  get_parameter("octomap_voxel_size", octomap_voxel_size_);
  get_parameter("octomap_publish_frequency", octomap_publish_frequency_);
  get_parameter("publish_octomap_visuals", publish_octomap_visuals_);
  get_parameter("octomap_point_cloud_publish_topic", octomap_point_cloud_publish_topic_);
  get_parameter("octomap_markers_publish_topic", octomap_markers_publish_topic_);
  get_parameter("map_frame_id", map_frame_id_);
  get_parameter("utm_frame_id", utm_frame_id_);
  get_parameter("yaw_offset", yaw_offset_);
  get_parameter("map_coordinates.latitude", pcd_map_gps_pose_->position.latitude);
  get_parameter("map_coordinates.longitude", pcd_map_gps_pose_->position.longitude);
  get_parameter("map_coordinates.altitude", pcd_map_gps_pose_->position.altitude);
  get_parameter("map_coordinates.quaternion.x", pcd_map_gps_pose_->orientation.x);
  get_parameter("map_coordinates.quaternion.y", pcd_map_gps_pose_->orientation.y);
  get_parameter("map_coordinates.quaternion.z", pcd_map_gps_pose_->orientation.z);
  get_parameter("map_coordinates.quaternion.w", pcd_map_gps_pose_->orientation.w);
  get_parameter("pcd_map_transform.translation.x", pcd_map_transform_matrix_.translation_.x());
  get_parameter("pcd_map_transform.translation.y", pcd_map_transform_matrix_.translation_.y());
  get_parameter("pcd_map_transform.translation.z", pcd_map_transform_matrix_.translation_.z());
  get_parameter("pcd_map_transform.rotation.r", pcd_map_transform_matrix_.rpyIntrinsic_.x());
  get_parameter("pcd_map_transform.rotation.p", pcd_map_transform_matrix_.rpyIntrinsic_.y());
  get_parameter("pcd_map_transform.rotation.y", pcd_map_transform_matrix_.rpyIntrinsic_.z());
  get_parameter("cell_radius", cost_params_.cell_radius);
  get_parameter("max_allowed_tilt", cost_params_.max_allowed_tilt);
  get_parameter("max_allowed_point_deviation", cost_params_.max_allowed_point_deviation);
  get_parameter("max_allowed_energy_gap", cost_params_.max_allowed_energy_gap);
  get_parameter("node_elevation_distance", cost_params_.node_elevation_distance);
  get_parameter("plane_fit_threshold", cost_params_.plane_fit_threshold);
  get_parameter("robot_mass", cost_params_.robot_mass);
  get_parameter("average_speed", cost_params_.average_speed);
  get_parameter("include_node_centers_in_cloud", cost_params_.include_node_centers_in_cloud);
  get_parameter("cost_critic_weights", cost_params_.cost_critic_weights);
  get_parameter("apply_filters", preprocess_params_.apply_filters);
  get_parameter("pcd_map_downsample_voxel_size", preprocess_params_.pcd_map_downsample_voxel_size);
  get_parameter("remove_outlier_mean_K", preprocess_params_.remove_outlier_mean_K);
  get_parameter(
    "remove_outlier_stddev_threshold", preprocess_params_.remove_outlier_stddev_threshold);
  get_parameter("remove_outlier_radius_search", preprocess_params_.remove_outlier_radius_search);
  get_parameter(
    "remove_outlier_min_neighbors_in_radius",
    preprocess_params_.remove_outlier_min_neighbors_in_radius);

  // service hooks
  get_maps_and_surfels_service_ = this->create_service<vox_nav_msgs::srv::GetMapsAndSurfels>(
    std::string("get_maps_and_surfels"),
    std::bind(
      &MapManager::getGetMapsAndSurfelsCallback, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  robot_localization_fromLL_client_node_ = std::make_shared<rclcpp::Node>(
    "map_manager_fromll_client_node");
  robot_localization_fromLL_client_ =
    robot_localization_fromLL_client_node_->create_client<robot_localization::srv::FromLL>("/fromLL");

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000 / octomap_publish_frequency_)),
    std::bind(&MapManager::timerCallback, this));

  octomap_pointloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    octomap_point_cloud_publish_topic_, rclcpp::SystemDefaultsQoS());
  octomap_markers_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    octomap_markers_publish_topic_, rclcpp::SystemDefaultsQoS());

  pcd_map_pointcloud_ = vox_nav_utilities::loadPointcloudFromPcd(pcd_map_filename_.c_str());

  RCLCPP_INFO(
    this->get_logger(), "Loaded a PCD map with %d points",
    pcd_map_pointcloud_->points.size());
}

MapManager::~MapManager()
{
  RCLCPP_INFO(this->get_logger(), "Destroying");
}

void MapManager::timerCallback()
{
  // Since this is static map we need to georefence this only once not each time
  std::call_once(
    align_static_map_once_, [this]()
    {
      while (!tf_buffer_->canTransform(utm_frame_id_, map_frame_id_, rclcpp::Time(0))) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        RCLCPP_INFO(
          this->get_logger(), "Waiting for %s to %s Transform to be available.",
          utm_frame_id_.c_str(), map_frame_id_.c_str());
      }
      RCLCPP_INFO(
        get_logger(), "Configuring pcd map with given parameters,"
        " But the map and octomap will be published at %i frequency rate",
        octomap_publish_frequency_);

      transfromPCDfromGPS2Map();
      preProcessPCDMap();
      regressCosts();
      handleOriginalOctomap();
      RCLCPP_INFO(get_logger(), "Georeferenced given map, ready to publish");

      map_configured_ = true;
    });
  publishMapVisuals();
}

void MapManager::preProcessPCDMap()
{
  pcd_map_pointcloud_ = vox_nav_utilities::downsampleInputCloud(
    pcd_map_pointcloud_,
    preprocess_params_.pcd_map_downsample_voxel_size);
  RCLCPP_INFO(
    this->get_logger(), "PCD Map downsampled, it now has %d points"
    " adjust the parameters if the map looks off",
    pcd_map_pointcloud_->points.size());
  if (preprocess_params_.apply_filters) {
    pcd_map_pointcloud_ = vox_nav_utilities::removeOutliersFromInputCloud(
      pcd_map_pointcloud_,
      preprocess_params_.remove_outlier_mean_K,
      preprocess_params_.remove_outlier_stddev_threshold,
      vox_nav_utilities::OutlierRemovalType::StatisticalOutlierRemoval);
    pcd_map_pointcloud_ = vox_nav_utilities::removeOutliersFromInputCloud(
      pcd_map_pointcloud_,
      preprocess_params_.remove_outlier_min_neighbors_in_radius,
      preprocess_params_.remove_outlier_radius_search,
      vox_nav_utilities::OutlierRemovalType::RadiusOutlierRemoval);
  }
  // apply a rigid body transfrom if it was given one
  pcd_map_pointcloud_ = vox_nav_utilities::transformCloud(
    pcd_map_pointcloud_,
    vox_nav_utilities::getRigidBodyTransform(
      pcd_map_transform_matrix_.translation_,
      pcd_map_transform_matrix_.rpyIntrinsic_,
      get_logger()));
  // Experimental, this assumes we have no prior infromation of
  // segmentation, so mark all points as traversable
  // by painting them green > 0
  pcd_map_pointcloud_ =
    vox_nav_utilities::set_cloud_color(pcd_map_pointcloud_, std::vector<double>({0.0, 255.0, 0.0}));
}

void MapManager::publishMapVisuals()
{
  if (publish_octomap_visuals_) {
    octomap_pointcloud_msg_->header.frame_id = map_frame_id_;
    octomap_pointcloud_msg_->header.stamp = this->now();
    octomap_pointloud_publisher_->publish(*octomap_pointcloud_msg_);
    octomap_markers_publisher_->publish(*octomap_markers_msg_);
  }
}

void MapManager::transfromPCDfromGPS2Map()
{
  auto request = std::make_shared<robot_localization::srv::FromLL::Request>();
  auto response = std::make_shared<robot_localization::srv::FromLL::Response>();
  request->ll_point.latitude = pcd_map_gps_pose_->position.latitude;
  request->ll_point.longitude = pcd_map_gps_pose_->position.longitude;
  request->ll_point.altitude = pcd_map_gps_pose_->position.altitude;

  while (!robot_localization_fromLL_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Interrupted while waiting for the /fromLL service.Exiting");
      return;
    }
    RCLCPP_INFO(
      this->get_logger(), "/fromLL service not available, waiting and trying again");
  }

  auto result_future = robot_localization_fromLL_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(
      robot_localization_fromLL_client_node_,
      result_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "/fromLL service call failed");
  }
  auto result = result_future.get();
  response->map_point = result->map_point;

  // "/fromLL" service only accounts for translational transform
  // we still need to rotate the points according to yaw_offset
  // yaw_offset determines rotation between utm and map frame
  // Normally utm and map frmaes are aligned rotationally, but if there is yaw_offset set in
  // navsat_transfrom_node we have to account for that yaw_offset here as well
  // use classic rotation formula https://en.wikipedia.org/wiki/Rotation_matrix#In_two_dimensions;
  // The rotation only happens in x and y since it is round the z axis(yaw)
  double x = response->map_point.x;
  double y = response->map_point.y;
  double x_dot = x * std::cos(yaw_offset_) - y * std::sin(yaw_offset_);
  double y_dot = x * std::sin(yaw_offset_) + y * std::cos(yaw_offset_);

  // The translation from static_map origin to map is basically inverse of this transform
  tf2::Transform static_map_translation;
  static_map_translation.setOrigin(
    tf2::Vector3(x_dot, y_dot, response->map_point.z));

  // this is identity because map and utm frames are rotationally aligned
  static_map_translation.setRotation(tf2::Quaternion::getIdentity());

  tf2::Transform static_map_rotation;
  tf2::Quaternion static_map_quaternion;
  tf2::fromMsg(pcd_map_gps_pose_->orientation, static_map_quaternion);
  // First align the static map origin to map in translation
  // and then rotate the static map with its correct rotation
  static_map_rotation.setOrigin(tf2::Vector3(0, 0, 0));
  static_map_rotation.setRotation(static_map_quaternion);
  tf2::Transform static_map_to_map_transfrom = static_map_rotation *
    static_map_translation.inverse();

  pcl_ros::transformPointCloud(
    *pcd_map_pointcloud_, *pcd_map_pointcloud_, static_map_to_map_transfrom
  );
}

void MapManager::regressCosts()
{
  // seperate traversble points from non-traversable ones
  auto pure_traversable_pcl = vox_nav_utilities::get_traversable_points(pcd_map_pointcloud_);
  auto pure_non_traversable_pcl =
    vox_nav_utilities::get_non_traversable_points(pcd_map_pointcloud_);

  // uniformly sample nodes on top of traversable cloud
  auto uniformly_sampled_nodes = vox_nav_utilities::uniformly_sample_cloud(
    pure_traversable_pcl,
    cost_params_.cell_radius);

  // This is basically vector of cloud segments, each segments includes points representing a cell
  // The first element of pair is surfel_center_point while the second is pointcloud itself
  auto surfels = vox_nav_utilities::surfelize_traversability_cloud(
    pure_traversable_pcl,
    uniformly_sampled_nodes,
    cost_params_.cell_radius);

  pcl::PointCloud<pcl::PointXYZRGB> cost_regressed_cloud;
  pcl::PointCloud<pcl::PointXYZRGB> elevated_surfels_cloud;
  for (auto && i : surfels) {
    auto surfel_center_point = i.first;
    auto surfel_cloud = i.second;

    // fit a plane to this surfel cloud, in order to et its orientation
    auto plane_model = vox_nav_utilities::fit_plane_to_cloud(
      surfel_cloud,
      cost_params_.plane_fit_threshold);

    // extract rpy from plane equation
    auto rpy = vox_nav_utilities::rpy_from_plane(plane_model);

    // extract averge point deviation from surfel cloud this determines the roughness of cloud
    double average_point_deviation = vox_nav_utilities::average_point_deviation_from_plane(
      surfel_cloud,
      plane_model);

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

    // any roll or pitche thats higher than max_tilt will make that surfel NON traversable
    if (max_tilt > cost_params_.max_allowed_tilt) {
      surfel_cloud = vox_nav_utilities::set_cloud_color(
        surfel_cloud,
        std::vector<double>({255.0, 0, 0}));
    } else {
      surfel_cloud = vox_nav_utilities::set_cloud_color(
        surfel_cloud,
        std::vector<double>({0.0, cost_params_.max_color_range - total_cost, total_cost}));
      handleElevatedSurfels(surfel_center_point, elevated_surfels_cloud, plane_model, rpy);
    }
    cost_regressed_cloud += *surfel_cloud;
  }

  octomap::Pointcloud surfel_octocloud;
  for (auto && i : elevated_surfels_cloud.points) {
    surfel_octocloud.push_back(octomap::point3d(i.x, i.y, i.z));
  }
  auto elevated_surfels_octomap_octree = std::make_shared<octomap::OcTree>(octomap_voxel_size_);
  elevated_surfels_octomap_octree->insertPointCloud(surfel_octocloud, octomap::point3d(0, 0, 0));

  try {
    octomap_msgs::fullMapToMsg<octomap::OcTree>(
      *elevated_surfels_octomap_octree,
      *elevated_surfel_octomap_msg_);
    elevated_surfel_octomap_msg_->binary = false;
    elevated_surfel_octomap_msg_->resolution = octomap_voxel_size_;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_logger(), "Exception while converting binary octomap %s:", e.what());
  }

  if (cost_params_.include_node_centers_in_cloud) {
    cost_regressed_cloud += elevated_surfels_cloud;
  }
  cost_regressed_cloud += *pure_non_traversable_pcl;
  *pcd_map_pointcloud_ = cost_regressed_cloud;

  // overlapping sufels duplicates some points , get rid of them by downsampling
  pcd_map_pointcloud_ = vox_nav_utilities::downsampleInputCloud(
    pcd_map_pointcloud_,
    preprocess_params_.pcd_map_downsample_voxel_size);
}


void MapManager::handleOriginalOctomap()
{
  pcl::toROSMsg(*pcd_map_pointcloud_, *octomap_pointcloud_msg_);
  octomap::Pointcloud octocloud;
  for (auto && i : pcd_map_pointcloud_->points) {
    octocloud.push_back(octomap::point3d(i.x, i.y, i.z));
  }
  auto original_octomap_octree = std::make_shared<octomap::OcTree>(octomap_voxel_size_);
  original_octomap_octree->insertPointCloud(octocloud, octomap::point3d(0, 0, 0));

  for (auto && i : pcd_map_pointcloud_->points) {
    double value = static_cast<double>(i.b / 255.0) -
      static_cast<double>(i.g / 255.0);
    if (i.r == 255) {
      value = 2.0;
    }
    original_octomap_octree->setNodeValue(i.x, i.y, i.z, std::max(0.0, value));
  }
  auto header = std::make_shared<std_msgs::msg::Header>();
  header->frame_id = map_frame_id_;
  header->stamp = this->now();
  vox_nav_utilities::fillOctomapMarkers(octomap_markers_msg_, header, original_octomap_octree);
  try {
    octomap_msgs::fullMapToMsg<octomap::OcTree>(
      *original_octomap_octree,
      *original_octomap_msg_);
    original_octomap_msg_->binary = false;
    original_octomap_msg_->resolution = octomap_voxel_size_;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_logger(), "Exception while converting binary octomap %s:", e.what());
  }
}

void MapManager::handleElevatedSurfels(
  pcl::PointXYZRGB & surfel_center_point,
  pcl::PointCloud<pcl::PointXYZRGB> & elevated_nodes_cloud,
  const pcl::ModelCoefficients plane_model,
  const std::vector<double> & rpy)
{
  pcl::PointXYZRGB elevated_node;
  elevated_node.x = surfel_center_point.x + cost_params_.node_elevation_distance *
    plane_model.values[0];
  elevated_node.y = surfel_center_point.y + cost_params_.node_elevation_distance *
    plane_model.values[1];
  elevated_node.z = surfel_center_point.z + cost_params_.node_elevation_distance *
    plane_model.values[2];
  elevated_node.r = cost_params_.max_color_range;
  elevated_node.g = cost_params_.max_color_range;
  elevated_nodes_cloud.points.push_back(elevated_node);
  geometry_msgs::msg::Pose elevated_node_pose;
  elevated_node_pose.position.x = elevated_node.x;
  elevated_node_pose.position.y = elevated_node.y;
  elevated_node_pose.position.z = elevated_node.z;
  elevated_node_pose.orientation = vox_nav_utilities::getMsgQuaternionfromRPY(
    rpy[0],
    rpy[1],
    rpy[2]);
  elevated_surfel_poses_msg_->poses.push_back(elevated_node_pose);
}

void MapManager::getGetMapsAndSurfelsCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<vox_nav_msgs::srv::GetMapsAndSurfels::Request> request,
  std::shared_ptr<vox_nav_msgs::srv::GetMapsAndSurfels::Response> response)
{
  if (!map_configured_) {
    RCLCPP_INFO(
      get_logger(), "Map has not been configured yet cannot handle GetMapsAndSurfels request");
  }
  RCLCPP_INFO(get_logger(), "Handling GetMapsAndSurfels request");
  response->original_octomap = *original_octomap_msg_;
  response->elevated_surfel_octomap = *elevated_surfel_octomap_msg_;
  response->elevated_surfel_poses = *elevated_surfel_poses_msg_;
}
}   // namespace vox_nav_map_server

/**
 * @brief
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char const * argv[])
{
  rclcpp::init(argc, argv);
  auto map_manager_node = std::make_shared
    <vox_nav_map_server::MapManager>();
  rclcpp::spin(map_manager_node);
  rclcpp::shutdown();
  return 0;
}
