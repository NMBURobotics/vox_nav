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

#include "vox_nav_map_server/osm_map_manager.hpp"
#include "boost/progress.hpp"
#include "rviz_default_plugins/displays/pointcloud/point_cloud_helpers.hpp"
#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <iostream>
#include <sstream>

namespace vox_nav_map_server
{
  OSMMapManager::OSMMapManager()
  : Node("vox_nav_osm_map_manager_rclcpp_node"),
    map_configured_(false)
  {

    // initialize shared pointers asap
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    static_transform_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    map_gps_pose_ = std::make_shared<vox_nav_msgs::msg::OrientedNavSatFix>();

    // Declare this node's parameters
    declare_parameter("osm_road_topologies_pcd_filename", "/home/username/osm_road_topologies.pcd");
    declare_parameter("osm_road_colliders_pcd_filename", "/home/username/osm_road_colliders.pcd");
    declare_parameter("osm_buildings_pcd_filename", "/home/username/osm_road_colliders.pcd");
    declare_parameter("publish_frequency", 10);
    declare_parameter("map_frame_id", "map");
    declare_parameter("utm_frame_id", "utm");
    declare_parameter("map_datum.latitude", 49.0);
    declare_parameter("map_datum.longitude", 3.0);
    declare_parameter("map_datum.altitude", 0.5);
    declare_parameter("map_datum.quaternion.x", 0.0);
    declare_parameter("map_datum.quaternion.y", 0.0);
    declare_parameter("map_datum.quaternion.z", 0.0);
    declare_parameter("map_datum.quaternion.w", 1.0);
    declare_parameter("pcd_map_transform.translation.x", 0.0);
    declare_parameter("pcd_map_transform.translation.y", 0.0);
    declare_parameter("pcd_map_transform.translation.z", 0.0);
    declare_parameter("pcd_map_transform.rotation.r", 0.0);
    declare_parameter("pcd_map_transform.rotation.p", 0.0);
    declare_parameter("pcd_map_transform.rotation.y", 0.0);
    declare_parameter("apply_filters", true);
    declare_parameter("downsample_leaf_size", 0.1);

    // get this node's parameters
    get_parameter("osm_road_topologies_pcd_filename", osm_road_topologies_pcd_filename_);
    get_parameter("osm_road_colliders_pcd_filename", osm_road_colliders_pcd_filename_);
    get_parameter("osm_buildings_pcd_filename", osm_buildings_pcd_filename_);
    get_parameter("publish_frequency", publish_frequency_);
    get_parameter("map_frame_id", map_frame_id_);
    get_parameter("utm_frame_id", utm_frame_id_);
    get_parameter("map_datum.latitude", map_gps_pose_->position.latitude);
    get_parameter("map_datum.longitude", map_gps_pose_->position.longitude);
    get_parameter("map_datum.altitude", map_gps_pose_->position.altitude);
    get_parameter("map_datum.quaternion.x", map_gps_pose_->orientation.x);
    get_parameter("map_datum.quaternion.y", map_gps_pose_->orientation.y);
    get_parameter("map_datum.quaternion.z", map_gps_pose_->orientation.z);
    get_parameter("map_datum.quaternion.w", map_gps_pose_->orientation.w);
    get_parameter("pcd_map_transform.translation.x", pcd_map_transform_matrix_.translation_.x());
    get_parameter("pcd_map_transform.translation.y", pcd_map_transform_matrix_.translation_.y());
    get_parameter("pcd_map_transform.translation.z", pcd_map_transform_matrix_.translation_.z());
    get_parameter("pcd_map_transform.rotation.r", pcd_map_transform_matrix_.rpyIntrinsic_.x());
    get_parameter("pcd_map_transform.rotation.p", pcd_map_transform_matrix_.rpyIntrinsic_.y());
    get_parameter("pcd_map_transform.rotation.y", pcd_map_transform_matrix_.rpyIntrinsic_.z());
    get_parameter("downsample_leaf_size", downsample_leaf_size_);

    // service hooks for get maps and surfels
    get_maps_service_ = this->create_service
      <vox_nav_msgs::srv::GetMapsAndSurfels>(
      std::string("get_maps_and_surfels"),
      std::bind(
        &OSMMapManager::getGetOSMMapCallback,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3));

    // service hooks for robot localization fromll service
    robot_localization_fromLL_client_node_ = std::make_shared
      <rclcpp::Node>("osm_map_manager_fromll_client_node");

    robot_localization_fromLL_client_ =
      robot_localization_fromLL_client_node_->create_client
      <robot_localization::srv::FromLL>(
      "/fromLL");

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000 / publish_frequency_)),
      std::bind(&OSMMapManager::timerCallback, this));

    osm_road_topologies_pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "osm_road_topologies_pointcloud", rclcpp::SystemDefaultsQoS());
    osm_road_colliders_pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "osm_road_colliders_pointcloud", rclcpp::SystemDefaultsQoS());
    osm_buildings_pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "osm_buildings_pointcloud", rclcpp::SystemDefaultsQoS());

    osm_road_topologies_pointcloud_ = vox_nav_utilities::loadPointcloudFromPcd(
      osm_road_topologies_pcd_filename_.c_str());
    osm_road_colliders_pointcloud_ = vox_nav_utilities::loadPointcloudFromPcd(
      osm_road_colliders_pcd_filename_.c_str());
    osm_buildings_pointcloud_ = vox_nav_utilities::loadPointcloudFromPcd(
      osm_buildings_pcd_filename_.c_str());

    RCLCPP_INFO(
      this->get_logger(), "OSM Road topologies cloud has %d points",
      static_cast<int>(osm_road_topologies_pointcloud_->points.size()));
    RCLCPP_INFO(
      this->get_logger(), "OSM Road colliders cloud has %d points",
      static_cast<int>(osm_road_colliders_pointcloud_->points.size()));
    RCLCPP_INFO(
      this->get_logger(), "OSM Buildings cloud has %d points",
      static_cast<int>(osm_buildings_pointcloud_->points.size()));

    RCLCPP_INFO(this->get_logger(), "Creating.");
  }

  OSMMapManager::~OSMMapManager()
  {
    RCLCPP_INFO(this->get_logger(), "Destroying");
  }

  void OSMMapManager::fixMapRoadsElevation()
  {
    // Road topology points are wrong in elevation, so we need to fix them using the road colliders which are correct
    RCLCPP_INFO(
      this->get_logger(),
      "Road topology points are wrong in elevation, so we will fix them with road colliders");

    boost::progress_display show_progress(osm_road_topologies_pointcloud_->points.size());

    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    kdtree.setInputCloud(osm_road_colliders_pointcloud_);

    for (auto & map_road_point : osm_road_topologies_pointcloud_->points) {
      std::vector<int> pointIdxNKNSearch(1);
      std::vector<float> pointNKNSquaredDistance(1);
      if (kdtree.nearestKSearch(
          map_road_point, 1, pointIdxNKNSearch,
          pointNKNSquaredDistance) > 0)
      {
        map_road_point.z = osm_road_colliders_pointcloud_->points[pointIdxNKNSearch[0]].z;
      }
      ++show_progress;
    }
  }

  void OSMMapManager::extractSemanticLabels()
  {

    sensor_msgs::msg::PointCloud2::SharedPtr osm_road_topologies_pointcloud_msg =
      std::make_shared<sensor_msgs::msg::PointCloud2>();

    pcl::toROSMsg(*osm_road_topologies_pointcloud_, *osm_road_topologies_pointcloud_msg);

    int32_t index = rviz_default_plugins::findChannelIndex(
      osm_road_topologies_pointcloud_msg, "rgb");

    const uint32_t offset = osm_road_topologies_pointcloud_msg->fields[index].offset;
    const uint8_t type = osm_road_topologies_pointcloud_msg->fields[index].datatype;
    const uint32_t point_step = osm_road_topologies_pointcloud_msg->point_step;
    const uint32_t num_points = osm_road_topologies_pointcloud_msg->width *
      osm_road_topologies_pointcloud_msg->height;

    float min_intensity = 9999999.0f;
    float max_intensity = -9999999.0f;

    for (uint32_t i = 0; i < num_points; ++i) {
      float val = rviz_default_plugins::valueFromCloud<float>(
        osm_road_topologies_pointcloud_msg,
        offset, type, point_step, i);
      min_intensity = std::min(val, min_intensity);
      max_intensity = std::max(val, max_intensity);
    }

    min_intensity = std::max(-999999.0f, min_intensity);
    max_intensity = std::min(999999.0f, max_intensity);

    float diff_intensity = max_intensity - min_intensity;
    if (diff_intensity == 0) {
      // If min and max are equal, set the diff to something huge so
      // when we divide by it, we effectively get zero.  That way the
      // point cloud coloring will be predictably uniform when min and
      // max are equal.
      diff_intensity = 1e20f;
    }

    for (uint32_t i = 0; i < num_points; ++i) {
      float val = rviz_default_plugins::valueFromCloud<float>(
        osm_road_topologies_pointcloud_msg,
        offset, type, point_step, i);
      float value = 1.0f - (val - min_intensity) / diff_intensity;

      getRainbowColor(value, osm_road_topologies_pointcloud_->points[i]);
    }


    // Each differnt color of point cloud is a different semantic label
    int r = -1, g = -1, b = -1;

    // A map of semantic labels and their corresponding color
    std::unordered_map<std::string, int> semantic_labels_map;
    semantic_labels_map.clear();

    for (auto & point : osm_road_topologies_pointcloud_->points) {
      if (point.r != r && point.g != g && point.b != b) {

        // encode the color values to a string
        std::stringstream ss;

        ss << std::setfill('0') << std::setw(3) << std::to_string(point.r)
           << std::setfill('0') << std::setw(3) << std::to_string(point.g)
           << std::setfill('0') << std::setw(3) << std::to_string(point.b);

        // add this color to the map
        semantic_labels_map[ss.str()] += 1;

      }
    }

    int curr_index = 4;
    for (auto x : semantic_labels_map) {
      std::cout << x.first.c_str() << " " << x.second << std::endl;
      // Label the semantic labels to each point cloud

      for (auto & point : osm_road_topologies_pointcloud_->points) {
        if (point.r == std::stoi(x.first.substr(0, 3)) &&
          point.g == std::stoi(x.first.substr(3, 3)) &&
          point.b == std::stoi(x.first.substr(6, 3)))
        {
          point.r = vox_nav_utilities::getColorByIndexEig(curr_index).x() * 255;
          point.g = vox_nav_utilities::getColorByIndexEig(curr_index).y() * 255;
          point.b = vox_nav_utilities::getColorByIndexEig(curr_index).z() * 255;
        }
      }
      curr_index++;
    }
  }

  void OSMMapManager::timerCallback()
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
          "But the map and octomap will be published at %i frequency rate",
          publish_frequency_);

        preProcessPCDMap();

        transfromPCDfromGPS2Map();

        fixMapRoadsElevation();

        extractSemanticLabels();

        RCLCPP_INFO(get_logger(), "Georeferenced given map, ready to publish");

        map_configured_ = true;
      });
    publishOSMPointClouds();
  }

  void OSMMapManager::transfromPCDfromGPS2Map()
  {
    auto request = std::make_shared<robot_localization::srv::FromLL::Request>();
    auto response = std::make_shared<robot_localization::srv::FromLL::Response>();
    request->ll_point.latitude = map_gps_pose_->position.latitude;
    request->ll_point.longitude = map_gps_pose_->position.longitude;
    request->ll_point.altitude = map_gps_pose_->position.altitude;

    while (!robot_localization_fromLL_client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Interrupted while waiting for the fromLL service.Exiting");
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

    // The translation from static_map origin to map is basically inverse of this transform
    tf2::Transform static_map_to_map_transfrom;
    static_map_to_map_transfrom.setOrigin(
      tf2::Vector3(
        response->map_point.x,
        response->map_point.y,
        response->map_point.z));

    tf2::Quaternion static_map_quaternion;
    tf2::fromMsg(map_gps_pose_->orientation, static_map_quaternion);
    static_map_to_map_transfrom.setRotation(static_map_quaternion);

    geometry_msgs::msg::TransformStamped stamped;
    stamped.child_frame_id = "static_map";
    stamped.header.frame_id = "map";
    stamped.header.stamp = this->get_clock()->now();
    stamped.transform.rotation = map_gps_pose_->orientation;
    geometry_msgs::msg::Vector3 translation;
    translation.x = response->map_point.x;
    translation.y = response->map_point.y;
    translation.z = response->map_point.z;
    stamped.transform.translation = translation;
    static_transform_broadcaster_->sendTransform(stamped);

    // Geo reference all the point clouds
    pcl_ros::transformPointCloud(
      *osm_road_topologies_pointcloud_, *osm_road_topologies_pointcloud_, static_map_to_map_transfrom
    );
    pcl_ros::transformPointCloud(
      *osm_road_colliders_pointcloud_, *osm_road_colliders_pointcloud_, static_map_to_map_transfrom
    );
    pcl_ros::transformPointCloud(
      *osm_buildings_pointcloud_, *osm_buildings_pointcloud_, static_map_to_map_transfrom
    );

  }

  void OSMMapManager::preProcessPCDMap()
  {
    if (downsample_leaf_size_ > 0.0) {
      osm_road_topologies_pointcloud_ = vox_nav_utilities::downsampleInputCloud<pcl::PointXYZRGB>(
        osm_road_topologies_pointcloud_, downsample_leaf_size_);

      osm_road_colliders_pointcloud_ = vox_nav_utilities::downsampleInputCloud<pcl::PointXYZRGB>(
        osm_road_colliders_pointcloud_, downsample_leaf_size_);

      osm_buildings_pointcloud_ = vox_nav_utilities::downsampleInputCloud<pcl::PointXYZRGB>(
        osm_buildings_pointcloud_, downsample_leaf_size_);
    }

    Eigen::Affine3d bt = vox_nav_utilities::getRigidBodyTransform(
      pcd_map_transform_matrix_.translation_,
      pcd_map_transform_matrix_.rpyIntrinsic_,
      get_logger());
    auto final_tr = tf2::eigenToTransform(bt);

    // Apply user requested transform (if any) to the point clouds
    pcl_ros::transformPointCloud(
      *osm_road_topologies_pointcloud_, *osm_road_topologies_pointcloud_, final_tr
    );
    pcl_ros::transformPointCloud(
      *osm_road_colliders_pointcloud_, *osm_road_colliders_pointcloud_, final_tr
    );
    pcl_ros::transformPointCloud(
      *osm_buildings_pointcloud_, *osm_buildings_pointcloud_, final_tr
    );
  }

  void OSMMapManager::publishOSMPointClouds()
  {
    // Publish the Osm point clouds
    sensor_msgs::msg::PointCloud2 osm_road_topologies_pointcloud_msg;
    pcl::toROSMsg(*osm_road_topologies_pointcloud_, osm_road_topologies_pointcloud_msg);
    osm_road_topologies_pointcloud_msg.header.frame_id = "map";
    osm_road_topologies_pointcloud_msg.header.stamp = this->get_clock()->now();
    osm_road_topologies_pointcloud_pub_->publish(osm_road_topologies_pointcloud_msg);

    sensor_msgs::msg::PointCloud2 osm_road_colliders_pointcloud_msg;
    pcl::toROSMsg(*osm_road_colliders_pointcloud_, osm_road_colliders_pointcloud_msg);
    osm_road_colliders_pointcloud_msg.header.frame_id = "map";
    osm_road_colliders_pointcloud_msg.header.stamp = this->get_clock()->now();
    osm_road_colliders_pointcloud_pub_->publish(osm_road_colliders_pointcloud_msg);

    sensor_msgs::msg::PointCloud2 osm_buildings_pointcloud_msg;
    pcl::toROSMsg(*osm_buildings_pointcloud_, osm_buildings_pointcloud_msg);
    osm_buildings_pointcloud_msg.header.frame_id = "map";
    osm_buildings_pointcloud_msg.header.stamp = this->get_clock()->now();
    osm_buildings_pointcloud_pub_->publish(osm_buildings_pointcloud_msg);
  }

  void OSMMapManager::getGetOSMMapCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<vox_nav_msgs::srv::GetMapsAndSurfels::Request> request,
    std::shared_ptr<vox_nav_msgs::srv::GetMapsAndSurfels::Response> response) {}


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
    <vox_nav_map_server::OSMMapManager>();
  rclcpp::spin(map_manager_node);
  rclcpp::shutdown();
  return 0;
}
