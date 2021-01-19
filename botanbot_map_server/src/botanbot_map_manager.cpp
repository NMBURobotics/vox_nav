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

#include <string>
#include <vector>
#include <memory>
#include "botanbot_map_server/botanbot_map_manager.hpp"

namespace botanbot_map_server
{
BotanbotMapManager::BotanbotMapManager()
: Node("botanbot_map_manager_rclcpp_node"),
  tf_broadcaster_(*this),
  is_robot_datum_ready_(false)
{
  RCLCPP_INFO(
    this->get_logger(),
    "Creating..");

  gps_waypoint_collector_node_ = std::make_shared<botanbot_utilities::GPSWaypointCollector>();
  octomap_ros_msg_ = std::make_shared<octomap_msgs::msg::Octomap>();
  octomap_pointcloud_ros_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
  map_datum_ = std::make_shared<botanbot_msgs::msg::OrientedNavSatFix>();
  robot_datum_ = std::make_shared<botanbot_msgs::msg::OrientedNavSatFix>();

  // Declare this node's parameters
  declare_parameter("octomap_filename", "/home/ros2-foxy/f.bt");
  declare_parameter("octomap_publish_topic_name", "octomap");
  declare_parameter("octomap_voxel_size", 0.2);
  declare_parameter("octomap_publish_frequency", 10);
  declare_parameter("provide_utm_to_map_transform", true);
  declare_parameter("publish_octomap_as_pointcloud", true);
  declare_parameter("octomap_point_cloud_publish_topic", "octomap_pointcloud");
  declare_parameter("octomap_frame_id", "map");
  declare_parameter("map_coordinates.latitude", 49.0);
  declare_parameter("map_coordinates.longitude", 3.0);
  declare_parameter("map_coordinates.altitude", 0.5);
  declare_parameter("map_coordinates.quaternion.x", 0.0);
  declare_parameter("map_coordinates.quaternion.y", 0.0);
  declare_parameter("map_coordinates.quaternion.z", 0.0);
  declare_parameter("map_coordinates.quaternion.w", 1.0);

  // get this node's parameters
  get_parameter("octomap_filename", octomap_filename_);
  get_parameter("octomap_publish_topic_name", octomap_publish_topic_name_);
  get_parameter("octomap_voxel_size", octomap_voxel_size_);
  get_parameter("octomap_publish_frequency", octomap_publish_frequency_);
  get_parameter("provide_utm_to_map_transform", provide_utm_to_map_transform_);
  get_parameter("publish_octomap_as_pointcloud", publish_octomap_as_pointcloud_);
  get_parameter("octomap_point_cloud_publish_topic", octomap_point_cloud_publish_topic_);
  get_parameter("octomap_frame_id", octomap_frame_id_);
  get_parameter("map_coordinates.latitude", map_datum_->position.latitude);
  get_parameter("map_coordinates.longitude", map_datum_->position.longitude);
  get_parameter("map_coordinates.altitude", map_datum_->position.altitude);
  get_parameter("map_coordinates.quaternion.x", map_datum_->orientation.x);
  get_parameter("map_coordinates.quaternion.y", map_datum_->orientation.y);
  get_parameter("map_coordinates.quaternion.z", map_datum_->orientation.z);
  get_parameter("map_coordinates.quaternion.w", map_datum_->orientation.w);

  octomap_octree_ = std::make_shared<octomap::OcTree>(octomap_voxel_size_);

  octomap_publisher_ = this->create_publisher<octomap_msgs::msg::Octomap>(
    octomap_publish_topic_name_, rclcpp::SystemDefaultsQoS());
  octomap_pointloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    octomap_point_cloud_publish_topic_, rclcpp::SystemDefaultsQoS());
  map_aligned_gps_odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
    "/odometry/gps/map_aligned", rclcpp::SystemDefaultsQoS());

  octomap_octree_->readBinary(octomap_filename_);
  bool is_octomap_successfully_converted(false);
  try {
    is_octomap_successfully_converted = octomap_msgs::binaryMapToMsg<octomap::OcTree>(
      *octomap_octree_, *octomap_ros_msg_);
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
    RCLCPP_ERROR(
      get_logger(),
      "Exception while converting bnary octomap  %s:", e.what());
  }

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000 / octomap_publish_frequency_)),
    std::bind(&BotanbotMapManager::timerCallback, this));

  gps_odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odometry/gps", rclcpp::SystemDefaultsQoS(),
    std::bind(&BotanbotMapManager::gpsOdomCallback, this, std::placeholders::_1));

  // used for transfroming orientation of GPS poses to map frame
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  RCLCPP_INFO(
    this->get_logger(),
    "Created an Instance of BotanbotMapManager");
}

BotanbotMapManager::~BotanbotMapManager()
{
  RCLCPP_INFO(
    this->get_logger(),
    "Destroyed an Instance of BotanbotMapManager");
}

void BotanbotMapManager::gpsOdomCallback(
  const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> guard(global_mutex_);
  gps_odom_ = *msg;
}

void BotanbotMapManager::timerCallback()
{
  std::lock_guard<std::mutex> guard(global_mutex_);
  auto stamp = this->now();
  octomap_ros_msg_->header.stamp = stamp;
  octomap_ros_msg_->header.frame_id = octomap_frame_id_;
  octomap_publisher_->publish(*octomap_ros_msg_);
  octomap_pointloud_publisher_->publish(*octomap_pointcloud_ros_msg_);

  double altitude = map_datum_->position.altitude;
  double longitude = map_datum_->position.longitude;
  double latitude = map_datum_->position.latitude;
  double utmY, utmX;
  std::string utm_zone_tmp;
  botanbot_utilities::navsat_conversions::LLtoUTM(latitude, longitude, utmY, utmX, utm_zone_tmp);
  tf2::Transform map_origin;
  tf2::Quaternion map_rot;
  map_rot.setX(map_datum_->orientation.x);
  map_rot.setY(map_datum_->orientation.y);
  map_rot.setZ(map_datum_->orientation.z);
  map_rot.setW(map_datum_->orientation.w);

  geometry_msgs::msg::TransformStamped map_utm_transform_stamped;
  map_origin.setOrigin(tf2::Vector3(utmX, utmY, altitude));
  map_origin.setRotation(map_rot);

  map_utm_transform_stamped.header.stamp = stamp;
  map_utm_transform_stamped.header.frame_id = "utm";
  map_utm_transform_stamped.child_frame_id = octomap_frame_id_;
  map_utm_transform_stamped.transform = tf2::toMsg(map_origin);
  tf_broadcaster_.sendTransform(map_utm_transform_stamped);
  fillPointCloudfromOctomap(octomap_octree_, octomap_pointcloud_ros_msg_);

  if (!gps_waypoint_collector_node_->isOrientedGPSDataReady()) {
    RCLCPP_WARN(
      get_logger(),
      "Robot GPS coordinates are not recieved yet, the initial pose of robot is unknown!, "
      "spinning gps waypoint collector node and trying again ...");
    rclcpp::spin_some(gps_waypoint_collector_node_);
    is_robot_datum_ready_ = false;
    rclcpp::sleep_for(std::chrono::seconds(1));
    return;
  } else {
    std::call_once(
      gps_data_recieved_flag_, [this]() {
        robot_datum_->position = gps_waypoint_collector_node_->getLatestOrientedGPSCoordinates().first;
        robot_datum_->orientation = gps_waypoint_collector_node_->getLatestOrientedGPSCoordinates().second.orientation;
        is_robot_datum_ready_ = true;
        RCLCPP_INFO(
          get_logger(), "Recived inital GPS pose of robot, this will be set as robot datum");
      });
  }

  if (is_robot_datum_ready_) {
    altitude = robot_datum_->position.altitude;
    longitude = robot_datum_->position.longitude;
    latitude = robot_datum_->position.latitude;
    botanbot_utilities::navsat_conversions::LLtoUTM(
      latitude, longitude, utmY, utmX,
      utm_zone_tmp);
    tf2::Transform robot_datum_origin;
    tf2::Quaternion robot_datum_rot;
    robot_datum_rot.setX(robot_datum_->orientation.x);
    robot_datum_rot.setY(robot_datum_->orientation.y);
    robot_datum_rot.setZ(robot_datum_->orientation.z);
    robot_datum_rot.setW(robot_datum_->orientation.w);

    geometry_msgs::msg::TransformStamped robot_utm_transform_stamped;
    robot_datum_origin.setOrigin(tf2::Vector3(utmX, utmY, altitude));
    robot_datum_origin.setRotation(robot_datum_rot);

    robot_utm_transform_stamped.header.stamp = stamp;
    robot_utm_transform_stamped.header.frame_id = "utm";
    robot_utm_transform_stamped.child_frame_id = "robot_datum";
    robot_utm_transform_stamped.transform = tf2::toMsg(robot_datum_origin);
    tf_broadcaster_.sendTransform(robot_utm_transform_stamped);

    geometry_msgs::msg::PoseStamped robot_datum;
    robot_datum.header.frame_id = "robot_datum";
    robot_datum.header.stamp = stamp;
    robot_datum.pose.orientation = robot_datum_->orientation;

    geometry_msgs::msg::PoseStamped robot_datum_in_map;
    robot_datum_in_map.header.frame_id = "map";
    robot_datum_in_map.header.stamp = stamp;

    try {
      tf_buffer_->transform(
        robot_datum, robot_datum_in_map, "map",
        tf2::durationFromSec(0.1));
      RCLCPP_INFO(get_logger(), "Transfromed gps odometry from root_datum to map datum");
      RCLCPP_INFO(
        get_logger(), "Ex values: %.4f, %.4f,  New val: %.4f,%.4f ",
        robot_datum.pose.position.x,
        robot_datum.pose.position.y,
        robot_datum_in_map.pose.position.x,
        robot_datum_in_map.pose.position.y);

      nav_msgs::msg::Odometry map_aligned_gps_odom;
      map_aligned_gps_odom.pose.pose.position = robot_datum_in_map.pose.position;
      map_aligned_gps_odom.pose.pose.orientation = robot_datum_in_map.pose.orientation;
      map_aligned_gps_odom.header = robot_datum_in_map.header;

      map_aligned_gps_odom_publisher_->publish(map_aligned_gps_odom);
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(
        this->get_logger(), "Exception in robot_datum -> map transform: %s",
        ex.what());
    }
  }
}

void BotanbotMapManager::fillPointCloudfromOctomap(
  const std::shared_ptr<octomap::OcTree> octomap_octree,
  sensor_msgs::msg::PointCloud2::SharedPtr cloud)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud =
    pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  // traverse all leafs in the tree:
  for (auto it = octomap_octree_->begin(0), end = octomap_octree_->end(); it != end; ++it) {
    if (octomap_octree_->isNodeOccupied(*it)) {
      double half_size = it.getSize() / 2.0;
      double size = it.getSize();
      double x = it.getX();
      double y = it.getY();
      double z = it.getZ();
      // insert into pointcloud:
      pcl::PointXYZ point = pcl::PointXYZ();
      point.x = x;
      point.y = y;
      point.z = z;
      pcl_cloud->points.push_back(point);
    }
  }
  pcl::toROSMsg(*pcl_cloud, *cloud);
  cloud->header.frame_id = octomap_frame_id_;
  cloud->header.stamp = this->now();
}

void BotanbotMapManager::publishUTMMapTransfrom()
{
}

}  // namespace botanbot_map_server

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
    <botanbot_map_server::BotanbotMapManager>();
  rclcpp::spin(gps_waypoint_follower_client_node);
  rclcpp::shutdown();
  return 0;
}
