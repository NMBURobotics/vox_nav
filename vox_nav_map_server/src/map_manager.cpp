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
: Node("vox_nav_map_manager_rclcpp_node")
{
  RCLCPP_INFO(this->get_logger(), "Creating..");
  // This is populated by params, soinitialize pointer asap
  static_map_gps_pose_ = std::make_shared<vox_nav_msgs::msg::OrientedNavSatFix>();
  node_poses_ = std::make_shared<geometry_msgs::msg::PoseArray>();

  // Declare this node's parameters
  declare_parameter("pcd_map_filename", "/home/ros2-foxy/f.pcd");
  declare_parameter("octomap_publish_topic_name", "octomap");
  declare_parameter("octomap_voxel_size", 0.2);
  declare_parameter("octomap_publish_frequency", 10);
  declare_parameter("publish_octomap_as_pointcloud", true);
  declare_parameter("publish_octomap_markers", true);
  declare_parameter("octomap_point_cloud_publish_topic", "octomap_pointcloud");
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
  get_parameter("octomap_publish_topic_name", octomap_publish_topic_name_);
  get_parameter("octomap_voxel_size", octomap_voxel_size_);
  get_parameter("octomap_publish_frequency", octomap_publish_frequency_);
  get_parameter("publish_octomap_as_pointcloud", publish_octomap_as_pointcloud_);
  get_parameter("publish_octomap_markers", publish_octomap_markers_);
  get_parameter("octomap_point_cloud_publish_topic", octomap_point_cloud_publish_topic_);
  get_parameter("map_frame_id", map_frame_id_);
  get_parameter("utm_frame_id", utm_frame_id_);
  get_parameter("yaw_offset", yaw_offset_);
  get_parameter("map_coordinates.latitude", static_map_gps_pose_->position.latitude);
  get_parameter("map_coordinates.longitude", static_map_gps_pose_->position.longitude);
  get_parameter("map_coordinates.altitude", static_map_gps_pose_->position.altitude);
  get_parameter("map_coordinates.quaternion.x", static_map_gps_pose_->orientation.x);
  get_parameter("map_coordinates.quaternion.y", static_map_gps_pose_->orientation.y);
  get_parameter("map_coordinates.quaternion.z", static_map_gps_pose_->orientation.z);
  get_parameter("map_coordinates.quaternion.w", static_map_gps_pose_->orientation.w);
  get_parameter("pcd_map_transform.translation.x", pcd_map_transform_matrix_.translation_.x());
  get_parameter("pcd_map_transform.translation.y", pcd_map_transform_matrix_.translation_.y());
  get_parameter("pcd_map_transform.translation.z", pcd_map_transform_matrix_.translation_.z());
  get_parameter("pcd_map_transform.rotation.r", pcd_map_transform_matrix_.rpyIntrinsic_.x());
  get_parameter("pcd_map_transform.rotation.p", pcd_map_transform_matrix_.rpyIntrinsic_.y());
  get_parameter("pcd_map_transform.rotation.y", pcd_map_transform_matrix_.rpyIntrinsic_.z());
  get_parameter("apply_filters", apply_filters_);
  get_parameter("pcd_map_downsample_voxel_size", pcd_map_downsample_voxel_size_);
  get_parameter("remove_outlier_mean_K", remove_outlier_mean_K_);
  get_parameter("remove_outlier_stddev_threshold", remove_outlier_stddev_threshold_);
  get_parameter("remove_outlier_radius_search", remove_outlier_radius_search_);
  get_parameter("remove_outlier_min_neighbors_in_radius", remove_outlier_min_neighbors_in_radius_);
  get_parameter("cell_radius", cell_radius_);
  get_parameter("max_allowed_tilt", max_allowed_tilt_);
  get_parameter("max_allowed_point_deviation", max_allowed_point_deviation_);
  get_parameter("max_allowed_energy_gap", max_allowed_energy_gap_);
  get_parameter("node_elevation_distance", node_elevation_distance_);
  get_parameter("plane_fit_threshold", plane_fit_threshold_);
  get_parameter("robot_mass", robot_mass_);
  get_parameter("average_speed", average_speed_);
  get_parameter("include_node_centers_in_cloud", include_node_centers_in_cloud_);
  get_parameter("cost_critic_weights", cost_critic_weights_);

  octomap_octree_ = std::make_shared<octomap::ColorOcTree>(octomap_voxel_size_);
  octomap_ros_msg_ = std::make_shared<octomap_msgs::msg::Octomap>();
  octomap_pointcloud_ros_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();

  robot_localization_fromLL_client_node_ = std::make_shared<rclcpp::Node>(
    "map_manager_fromll_client_node");
  octomap_publisher_ = this->create_publisher<octomap_msgs::msg::Octomap>(
    octomap_publish_topic_name_, rclcpp::SystemDefaultsQoS());
  octomap_pointloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    octomap_point_cloud_publish_topic_, rclcpp::SystemDefaultsQoS());
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000 / octomap_publish_frequency_)),
    std::bind(&MapManager::timerCallback, this));
  robot_localization_fromLL_client_ =
    robot_localization_fromLL_client_node_->
    create_client<robot_localization::srv::FromLL>("/fromLL");
  // setup TF buffer and listerner to read transforms
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  octomap_markers_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "octomap_markers", rclcpp::SystemDefaultsQoS());
  node_poses_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
    "node_poses", rclcpp::SystemDefaultsQoS());

  pcd_map_pointcloud_ = vox_nav_utilities::loadPointcloudFromPcd(pcd_map_filename_.c_str());

  RCLCPP_INFO(
    this->get_logger(), "Loading a PCD map with %d points",
    pcd_map_pointcloud_->points.size());

  pcd_map_pointcloud_ = vox_nav_utilities::downsampleInputCloud(
    pcd_map_pointcloud_,
    pcd_map_downsample_voxel_size_);

  RCLCPP_INFO(
    this->get_logger(), "PCD Map downsampled, it now has %d points",
    pcd_map_pointcloud_->points.size());

  if (apply_filters_) {
    pcd_map_pointcloud_ = vox_nav_utilities::removeOutliersFromInputCloud(
      pcd_map_pointcloud_,
      remove_outlier_mean_K_,
      remove_outlier_stddev_threshold_,
      vox_nav_utilities::OutlierRemovalType::StatisticalOutlierRemoval);
    pcd_map_pointcloud_ = vox_nav_utilities::removeOutliersFromInputCloud(
      pcd_map_pointcloud_,
      remove_outlier_min_neighbors_in_radius_,
      remove_outlier_radius_search_,
      vox_nav_utilities::OutlierRemovalType::RadiusOutlierRemoval);
  }

  pcd_map_pointcloud_ = vox_nav_utilities::transformCloud(
    pcd_map_pointcloud_,
    vox_nav_utilities::getRigidBodyTransform(
      pcd_map_transform_matrix_.translation_,
      pcd_map_transform_matrix_.rpyIntrinsic_,
      get_logger()));

  bool is_octomap_successfully_converted(false);
  try {
    is_octomap_successfully_converted = octomap_msgs::fullMapToMsg<octomap::ColorOcTree>(
      *octomap_octree_, *octomap_ros_msg_);
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
    RCLCPP_ERROR(
      get_logger(),
      "Exception while converting octomap %s:", e.what());
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Created an Instance of MapManager");
}

MapManager::~MapManager()
{
  RCLCPP_INFO(
    this->get_logger(),
    "Destroyed an Instance of MapManager");
}

void MapManager::timerCallback()
{
  // Since this is static map we need to georefence this only once not each time
  std::call_once(
    align_static_map_once_, [this]() {
      while (!tf_buffer_->canTransform(utm_frame_id_, map_frame_id_, rclcpp::Time(0))) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        RCLCPP_INFO(
          this->get_logger(), "Waiting for %s to %s Transform to be available.",
          utm_frame_id_.c_str(), map_frame_id_.c_str() );
      }
      RCLCPP_INFO(
        get_logger(), "Going to align the static map to map frames once,"
        " But the map and octomap will be published at %i frequency rate",
        octomap_publish_frequency_);
      RCLCPP_INFO(get_logger(), "Depending on the size of map , this might take a while..");
      auto request = std::make_shared<robot_localization::srv::FromLL::Request>();
      auto response = std::make_shared<robot_localization::srv::FromLL::Response>();
      request->ll_point.latitude = static_map_gps_pose_->position.latitude;
      request->ll_point.longitude = static_map_gps_pose_->position.longitude;
      request->ll_point.altitude = static_map_gps_pose_->position.altitude;
      fromGPSPoseToMapPose(request, response);

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
      tf2::fromMsg(static_map_gps_pose_->orientation, static_map_quaternion);
      // First align the static map origin to map in translation
      // and then rotate the static map with its correct rotation
      static_map_rotation.setOrigin(tf2::Vector3(0, 0, 0));
      static_map_rotation.setRotation(static_map_quaternion);

      tf2::Transform static_map_to_map_transfrom = static_map_rotation *
      static_map_translation.inverse();

      RCLCPP_INFO(this->get_logger(), "Regressing costs to given pcd map");
      regressCosts();
      alignStaticMapToMap(static_map_to_map_transfrom);
      fillOctomapMarkers();
      RCLCPP_INFO(get_logger(), "Georeferenced given map, ready to publish");
    });

  publishAlignedMap();
}

void MapManager::publishAlignedMap()
{
  octomap_ros_msg_->header.stamp = this->now();
  octomap_ros_msg_->header.frame_id = map_frame_id_;
  octomap_publisher_->publish(*octomap_ros_msg_);
  if (publish_octomap_as_pointcloud_) {
    octomap_pointcloud_ros_msg_->header.frame_id = map_frame_id_;
    octomap_pointcloud_ros_msg_->header.stamp = this->now();
    octomap_pointloud_publisher_->publish(*octomap_pointcloud_ros_msg_);
  }
  if (publish_octomap_markers_) {
    octomap_markers_publisher_->publish(octomap_markers_);
  }
  node_poses_publisher_->publish(*node_poses_);
}

void MapManager::fromGPSPoseToMapPose(
  const robot_localization::srv::FromLL::Request::SharedPtr request,
  robot_localization::srv::FromLL::Response::SharedPtr response)
{
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
}

void MapManager::regressCosts()
{
  // EXPERIMENTAL, THIS ASSUMES WE HAVE NO PRIOR INFORMATION OF
  // SEGMENTATION, SO MARK ALL POINTS AS TRAVERSABLE
  // BY PAINTING THEM GREEN > 0
  pcd_map_pointcloud_ =
    set_cloud_color(pcd_map_pointcloud_, std::vector<double>({0.0, 255.0, 0.0}));

  // REMOVE NON TRAVERSABLE POINTS(RED POINTS)
  auto pure_traversable_pcl = get_traversable_points(pcd_map_pointcloud_);
  auto pure_non_traversable_pcl = get_non_traversable_points(pcd_map_pointcloud_);

  // UNIFORMLY SAMPLE NODES ON TOP OF TRAVERSABLE CLOUD
  auto uniformly_sampled_nodes = uniformly_sample_cloud(
    pure_traversable_pcl,
    cell_radius_);

  // THIS IS BASICALLY VECTOR OF CLOUD SEGMENTS, EACH SEGMENT INCLUDES POINTS REPRESENTING CELL,
  // THE FIRST ELEMNET OF PAIR IS CENTROID WHILE SECOND IS THE POINTCLOUD ITSELF
  auto decomposed_cells = decompose_traversability_cloud(
    pure_traversable_pcl,
    uniformly_sampled_nodes,
    cell_radius_);

  pcl::PointCloud<pcl::PointXYZRGB> cost_regressd_cloud;
  pcl::PointCloud<pcl::PointXYZRGB> elevated_nodes_cloud;
  for (auto && i : decomposed_cells) {
    auto plane_model = fit_plane_to_cloud(i.second, plane_fit_threshold_);
    auto rpy = absolute_rpy_from_plane(plane_model);
    double average_point_deviation = average_point_deviation_from_plane(i.second, plane_model);
    double max_energy_gap = max_energy_gap_in_cloud(i.second, robot_mass_, average_speed_);

    double max_tilt = std::max(rpy[0], rpy[1]);
    double slope_cost = std::min(
      max_tilt /
      max_allowed_tilt_, 1.0) *
      kMaxColorRange;
    double energy_gap_cost = std::min(
      max_energy_gap /
      max_allowed_energy_gap_, 1.0) * kMaxColorRange;
    double deviation_of_points_cost = std::min(
      average_point_deviation /
      max_allowed_point_deviation_, 1.0) * kMaxColorRange;

    double total_cost =
      cost_critic_weights_[0] * slope_cost +
      cost_critic_weights_[1] * deviation_of_points_cost +
      cost_critic_weights_[2] * energy_gap_cost;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_fitted_cell(
      new pcl::PointCloud<pcl::PointXYZRGB>);

    if (max_tilt > max_allowed_tilt_) {
      plane_fitted_cell = set_cloud_color(
        i.second,
        std::vector<double>({255.0, 0, 0}));
    } else {
      plane_fitted_cell = set_cloud_color(
        i.second,
        std::vector<double>({0.0, kMaxColorRange - total_cost, total_cost}));
      pcl::PointXYZRGB elevated_node;
      elevated_node.x = i.first.x + node_elevation_distance_ * plane_model.values[0];
      elevated_node.y = i.first.y + node_elevation_distance_ * plane_model.values[1];
      elevated_node.z = i.first.z + node_elevation_distance_ * plane_model.values[2];
      elevated_node.r = kMaxColorRange;
      elevated_node.g = kMaxColorRange;
      elevated_nodes_cloud.points.push_back(elevated_node);

      geometry_msgs::msg::Pose elevated_node_pose;
      elevated_node_pose.position.x = elevated_node.x;
      elevated_node_pose.position.y = elevated_node.y;
      elevated_node_pose.position.z = elevated_node.z;
      const double kRAD2DEG = 180.0 / M_PI;

      elevated_node_pose.orientation = vox_nav_utilities::getMsgQuaternionfromRPY(
        rpy[0] / kRAD2DEG, rpy[1] / kRAD2DEG, rpy[2] / kRAD2DEG);
      node_poses_->poses.push_back(elevated_node_pose);
    }

    cost_regressd_cloud += *plane_fitted_cell;
  }

  if (include_node_centers_in_cloud_) {
    cost_regressd_cloud += elevated_nodes_cloud;
  }
  cost_regressd_cloud += *pure_non_traversable_pcl;
  *pcd_map_pointcloud_ = cost_regressd_cloud;

  pcd_map_pointcloud_ = vox_nav_utilities::downsampleInputCloud(
    pcd_map_pointcloud_,
    pcd_map_downsample_voxel_size_);
}


void MapManager::alignStaticMapToMap(const tf2::Transform & static_map_to_map_transfrom)
{
  pcl_ros::transformPointCloud(
    *pcd_map_pointcloud_, *pcd_map_pointcloud_, static_map_to_map_transfrom
  );
  pcl::toROSMsg(*pcd_map_pointcloud_, *octomap_pointcloud_ros_msg_);

  octomap::Pointcloud octocloud;
  octomap::point3d sensorOrigin(0, 0, 0);

  for (auto && i : pcd_map_pointcloud_->points) {
    octocloud.push_back(octomap::point3d(i.x, i.y, i.z));
  }

  octomap_octree_->insertPointCloud(octocloud, sensorOrigin);

  for (auto && i : pcd_map_pointcloud_->points) {
    double value = static_cast<double>(i.b / 255.0) -
      static_cast<double>(i.g / 255.0);
    if (i.r == 255) {
      value = 2.0;
    }
    if (i.r == 255 && i.g == 255) {
      value = 3.0;
    }
    octomap_octree_->setNodeValue(i.x, i.y, i.z, std::max(0.0, value));
  }

  try {
    octomap_msgs::fullMapToMsg<octomap::ColorOcTree>(*octomap_octree_, *octomap_ros_msg_);
    octomap_ros_msg_->binary = false;
    octomap_ros_msg_->resolution = octomap_voxel_size_;
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
    RCLCPP_ERROR(
      get_logger(),
      "Exception while converting binary octomap %s:", e.what());
  }
}
void MapManager::fillOctomapMarkers()
{
  auto tree_depth = octomap_octree_->getTreeDepth();
  octomap_markers_.markers.resize(tree_depth + 1);
  // now, traverse all leafs in the tree:
  for (auto it = octomap_octree_->begin(tree_depth),
    end = octomap_octree_->end(); it != end; ++it)
  {
    unsigned idx = it.getDepth();
    assert(idx < octomap_markers_.markers.size());
    geometry_msgs::msg::Point cubeCenter;
    cubeCenter.x = it.getCoordinate().x();
    cubeCenter.y = it.getCoordinate().y();
    cubeCenter.z = it.getCoordinate().z();
    octomap_markers_.markers[idx].points.push_back(cubeCenter);

    std_msgs::msg::ColorRGBA color;

    color.g = 1.0 - it->getValue();
    color.b = it->getValue();
    color.a = 1.0;

    if (it->getValue() == 2.0) {
      color.r = 1.0;
      color.g = 0.0;
      color.b = 0.0;
    }

    if (it->getValue() == 3.0) {
      color.r = 1.0;
      color.g = 1.0;
      color.b = 0.0;
    }

    if (!octomap_octree_->isNodeOccupied(*it)) {
      color.r = 0.0;
      color.g = 0.0;
      color.b = 0.0;
      color.a = 0.04;
    }

    octomap_markers_.markers[idx].colors.push_back(color);
  }

  for (unsigned i = 0; i < octomap_markers_.markers.size(); ++i) {
    double size = octomap_octree_->getNodeSize(i);

    octomap_markers_.markers[i].header.frame_id = map_frame_id_;
    octomap_markers_.markers[i].header.stamp = this->now();
    octomap_markers_.markers[i].ns = map_frame_id_;
    octomap_markers_.markers[i].id = i;
    octomap_markers_.markers[i].type =
      visualization_msgs::msg::Marker::CUBE_LIST;
    octomap_markers_.markers[i].scale.x = size;
    octomap_markers_.markers[i].scale.y = size;
    octomap_markers_.markers[i].scale.z = size;

    if (octomap_markers_.markers[i].points.size() > 0) {
      octomap_markers_.markers[i].action =
        visualization_msgs::msg::Marker::ADD;
    } else {
      octomap_markers_.markers[i].action =
        visualization_msgs::msg::Marker::DELETE;
    }
  }
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
  auto gps_waypoint_follower_client_node = std::make_shared
    <vox_nav_map_server::MapManager>();
  rclcpp::spin(gps_waypoint_follower_client_node);
  rclcpp::shutdown();
  return 0;
}
