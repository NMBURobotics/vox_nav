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
  is_robot_initial_gps_pose_ready_(false)
{
  RCLCPP_INFO(
    this->get_logger(),
    "Creating..");

  gps_waypoint_collector_node_ = std::make_shared<botanbot_utilities::GPSWaypointCollector>();
  octomap_ros_msg_ = std::make_shared<octomap_msgs::msg::Octomap>();
  octomap_pointcloud_ros_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
  static_map_gps_pose_ = std::make_shared<botanbot_msgs::msg::OrientedNavSatFix>();
  robot_initial_gps_pose_ = std::make_shared<botanbot_msgs::msg::OrientedNavSatFix>();

  // Declare this node's parameters
  declare_parameter("octomap_filename", "/home/ros2-foxy/f.bt");
  declare_parameter("octomap_publish_topic_name", "octomap");
  declare_parameter("octomap_voxel_size", 0.2);
  declare_parameter("octomap_publish_frequency", 10);
  declare_parameter("publish_octomap_as_pointcloud", true);
  declare_parameter("octomap_point_cloud_publish_topic", "octomap_pointcloud");
  declare_parameter("map_frame_id", "map");
  declare_parameter("static_map_frame_id", "static_map");
  declare_parameter("utm_frame_id", "utm");
  declare_parameter("map_coordinates.latitude", 49.0);
  declare_parameter("map_coordinates.longitude", 3.0);
  declare_parameter("map_coordinates.altitude", 0.5);
  declare_parameter("map_coordinates.quaternion.x", 0.0);
  declare_parameter("map_coordinates.quaternion.y", 0.0);
  declare_parameter("map_coordinates.quaternion.z", 0.0);
  declare_parameter("map_coordinates.quaternion.w", 1.0);
  declare_parameter("use_robot_datum", false);
  declare_parameter("robot_datum_coordinates.latitude", 49.0);
  declare_parameter("robot_datum_coordinates.longitude", 3.0);
  declare_parameter("robot_datum_coordinates.altitude", 0.5);
  declare_parameter("robot_datum_coordinates.quaternion.x", 0.0);
  declare_parameter("robot_datum_coordinates.quaternion.y", 0.0);
  declare_parameter("robot_datum_coordinates.quaternion.z", 0.0);
  declare_parameter("robot_datum_coordinates.quaternion.w", 1.0);

  // get this node's parameters
  get_parameter("octomap_filename", octomap_filename_);
  get_parameter("octomap_publish_topic_name", octomap_publish_topic_name_);
  get_parameter("octomap_voxel_size", octomap_voxel_size_);
  get_parameter("octomap_publish_frequency", octomap_publish_frequency_);
  get_parameter("publish_octomap_as_pointcloud", publish_octomap_as_pointcloud_);
  get_parameter("octomap_point_cloud_publish_topic", octomap_point_cloud_publish_topic_);
  get_parameter("map_frame_id", map_frame_id_);
  get_parameter("static_map_frame_id", static_map_frame_id_);
  get_parameter("utm_frame_id", utm_frame_id_);
  get_parameter("map_coordinates.latitude", static_map_gps_pose_->position.latitude);
  get_parameter("map_coordinates.longitude", static_map_gps_pose_->position.longitude);
  get_parameter("map_coordinates.altitude", static_map_gps_pose_->position.altitude);
  get_parameter("map_coordinates.quaternion.x", static_map_gps_pose_->orientation.x);
  get_parameter("map_coordinates.quaternion.y", static_map_gps_pose_->orientation.y);
  get_parameter("map_coordinates.quaternion.z", static_map_gps_pose_->orientation.z);
  get_parameter("map_coordinates.quaternion.w", static_map_gps_pose_->orientation.w);
  get_parameter("use_robot_datum", use_robot_datum_);
  if (use_robot_datum_) {
    get_parameter("robot_datum_coordinates.latitude", robot_initial_gps_pose_->position.latitude);
    get_parameter("robot_datum_coordinates.longitude", robot_initial_gps_pose_->position.longitude);
    get_parameter("robot_datum_coordinates.altitude", robot_initial_gps_pose_->position.altitude);
    get_parameter("robot_datum_coordinates.quaternion.x", robot_initial_gps_pose_->orientation.x);
    get_parameter("robot_datum_coordinates.quaternion.y", robot_initial_gps_pose_->orientation.y);
    get_parameter("robot_datum_coordinates.quaternion.z", robot_initial_gps_pose_->orientation.z);
    get_parameter("robot_datum_coordinates.quaternion.w", robot_initial_gps_pose_->orientation.w);
    is_robot_initial_gps_pose_ready_ = true;
  }

  octomap_octree_ = std::make_shared<octomap::OcTree>(octomap_voxel_size_);
  octomap_publisher_ = this->create_publisher<octomap_msgs::msg::Octomap>(
    octomap_publish_topic_name_, rclcpp::SystemDefaultsQoS());
  octomap_pointloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    octomap_point_cloud_publish_topic_, rclcpp::SystemDefaultsQoS());
  octomap_octree_->readBinary(octomap_filename_);

  bool is_octomap_successfully_converted(false);
  try {
    is_octomap_successfully_converted = octomap_msgs::binaryMapToMsg<octomap::OcTree>(
      *octomap_octree_, *octomap_ros_msg_);
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
    RCLCPP_ERROR(
      get_logger(),
      "Exception while converting binary octomap  %s:", e.what());
  }

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000 / octomap_publish_frequency_)),
    std::bind(&BotanbotMapManager::timerCallback, this));
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

void BotanbotMapManager::timerCallback()
{
  if (!gps_waypoint_collector_node_->isOrientedGPSDataReady()) {
    RCLCPP_WARN(
      get_logger(),
      "Robot GPS coordinates are not recieved yet, the initial pose of robot is unknown!, "
      "spinning gps waypoint collector node and trying again ...");
    rclcpp::spin_some(gps_waypoint_collector_node_);
    is_robot_initial_gps_pose_ready_ = false;
    rclcpp::sleep_for(std::chrono::seconds(1));
    return;
  } else {
    std::call_once(
      robot_initial_gps_pose_recieved_flag_,
      [this]() {
        robot_initial_gps_pose_->position = gps_waypoint_collector_node_->getLatestOrientedGPSCoordinates().first;
        robot_initial_gps_pose_->orientation = gps_waypoint_collector_node_->getLatestOrientedGPSCoordinates().second.orientation;
        is_robot_initial_gps_pose_ready_ = true;
        RCLCPP_INFO(
          get_logger(), "Recived inital GPS pose of robot, this will be set as robot datum");
      });
  }

  rclcpp::Time stamp = this->now();
  broadcastUtmStaticMapTransform(stamp, utm_frame_id_, static_map_frame_id_);
  broadcastUtmMapTransform(stamp, utm_frame_id_, map_frame_id_);

  try {
    geometry_msgs::msg::TransformStamped static_map_to_map_trans = tf_buffer_->lookupTransform(
      static_map_frame_id_,
      map_frame_id_,
      tf2::TimePointZero
    );
    publishOctomap(stamp, map_frame_id_, static_map_to_map_trans);
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
    RCLCPP_ERROR(
      get_logger(), "Caught exception: %s \n", e.what());
  }
}

void BotanbotMapManager::publishOctomap(
  rclcpp::Time stamp, std::string map_frame_id,
  geometry_msgs::msg::TransformStamped static_map_to_map_trans)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_octomap_cloud =
    pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  std::shared_ptr<octomap::OcTree> aligned_octomap =
    std::make_shared<octomap::OcTree>(octomap_voxel_size_);
  octomap::Pointcloud temp_octocloud;

  for (auto it = octomap_octree_->begin(0), end = octomap_octree_->end(); it != end; ++it) {
    if (octomap_octree_->isNodeOccupied(*it)) {
      double half_size = it.getSize() / 2.0;
      double size = it.getSize();
      double x = it.getX() - static_map_to_map_trans.transform.translation.x;
      double y = it.getY() - static_map_to_map_trans.transform.translation.y;
      double z = it.getZ() - static_map_to_map_trans.transform.translation.z;
      // insert into pointcloud:
      pcl::PointXYZ point = pcl::PointXYZ();
      point.x = x;
      point.y = y;
      point.z = z;
      aligned_octomap_cloud->points.push_back(point);
      octomap::point3d octocloud_point(x, y, z);
      temp_octocloud.push_back(octocloud_point);
    }
  }
  pcl::toROSMsg(*aligned_octomap_cloud, *octomap_pointcloud_ros_msg_);
  octomap_pointcloud_ros_msg_->header.frame_id = map_frame_id;
  octomap_pointcloud_ros_msg_->header.stamp = stamp;
  octomap::point3d sensorOrigin(0, 0, 0);
  aligned_octomap->insertPointCloud(temp_octocloud, sensorOrigin);
  try {
    octomap_msgs::binaryMapToMsg<octomap::OcTree>(*aligned_octomap, *octomap_ros_msg_);
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
    RCLCPP_ERROR(
      get_logger(),
      "Exception while converting binary octomap  %s:", e.what());
  }
  octomap_ros_msg_->header.stamp = stamp;
  octomap_ros_msg_->header.frame_id = map_frame_id;
  octomap_pointloud_publisher_->publish(*octomap_pointcloud_ros_msg_);
  octomap_publisher_->publish(*octomap_ros_msg_);
}

void BotanbotMapManager::broadcastUtmStaticMapTransform(
  rclcpp::Time stamp, std::string utm_frame_id,
  std::string static_map_frame_id)
{
  double altitude = static_map_gps_pose_->position.altitude;
  double longitude = static_map_gps_pose_->position.longitude;
  double latitude = static_map_gps_pose_->position.latitude;
  double utmY, utmX;
  std::string utm_zone_tmp;

  botanbot_utilities::navsat_conversions::LLtoUTM(
    latitude, longitude, utmY, utmX,
    utm_zone_tmp);

  tf2::Transform utm_to_static_map_trans;

  utm_to_static_map_trans.setOrigin(tf2::Vector3(utmX, utmY, altitude));
  tf2::Quaternion map_rot;
  map_rot.setX(static_map_gps_pose_->orientation.x);
  map_rot.setY(static_map_gps_pose_->orientation.y);
  map_rot.setZ(static_map_gps_pose_->orientation.z);
  map_rot.setW(static_map_gps_pose_->orientation.w);
  utm_to_static_map_trans.setRotation(map_rot);

  geometry_msgs::msg::TransformStamped utm_map_transform_stamped;
  utm_map_transform_stamped.header.stamp = stamp;
  utm_map_transform_stamped.header.frame_id = utm_frame_id;
  utm_map_transform_stamped.child_frame_id = static_map_frame_id;
  utm_map_transform_stamped.transform = tf2::toMsg(utm_to_static_map_trans);
  tf_broadcaster_.sendTransform(utm_map_transform_stamped);
}

void BotanbotMapManager::broadcastUtmMapTransform(
  rclcpp::Time stamp, std::string utm_frame_id,
  std::string map_frame_id)
{
  // First we need to make sure we have already recieved robot sinitial pose
  if (is_robot_initial_gps_pose_ready_) {
    double altitude = robot_initial_gps_pose_->position.altitude;
    double longitude = robot_initial_gps_pose_->position.longitude;
    double latitude = robot_initial_gps_pose_->position.latitude;
    double utmY, utmX;
    std::string utm_zone_tmp;
    botanbot_utilities::navsat_conversions::LLtoUTM(
      latitude, longitude, utmY, utmX, utm_zone_tmp);
    tf2::Transform utm_to_map_trans;
    utm_to_map_trans.setOrigin(tf2::Vector3(utmX, utmY, altitude));
    tf2::Quaternion robot_rot;
    robot_rot.setX(robot_initial_gps_pose_->orientation.x);
    robot_rot.setY(robot_initial_gps_pose_->orientation.y);
    robot_rot.setZ(robot_initial_gps_pose_->orientation.z);
    robot_rot.setW(robot_initial_gps_pose_->orientation.w);
    utm_to_map_trans.setRotation(robot_rot);

    geometry_msgs::msg::TransformStamped utm_robot_transform_stamped;
    utm_robot_transform_stamped.header.stamp = stamp;
    utm_robot_transform_stamped.header.frame_id = utm_frame_id;
    utm_robot_transform_stamped.child_frame_id = map_frame_id;
    utm_robot_transform_stamped.transform = tf2::toMsg(utm_to_map_trans);
    tf_broadcaster_.sendTransform(utm_robot_transform_stamped);
  }
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
