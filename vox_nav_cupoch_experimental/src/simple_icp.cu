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

/*
DISCLAIMER: some parts of code has been taken from; https://github.com/appinho/SARosPerceptionKitti
Credits to author: Simon Appel, https://github.com/appinho
*/

#include "vox_nav_cupoch_experimental/simple_icp.hpp"

#include <string>
#include <vector>
#include <memory>
#include <algorithm>

using namespace vox_nav_cupoch_experimental;

SimpleICP::SimpleICP()
: Node("cloud_clustering_rclcpp_node")
{

  cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/ouster/points",
    rclcpp::SensorDataQoS(),
    std::bind(
      &SimpleICP::cloudCallback, this, std::placeholders::_1));

  map_cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "vox_nav/map_server/octomap_pointcloud",
    rclcpp::SensorDataQoS(),
    std::bind(
      &SimpleICP::mapCloudCallback, this, std::placeholders::_1));

  gps_odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odometry/gps",
    rclcpp::SensorDataQoS(),
    std::bind(&SimpleICP::gpsOdomCallback, this, std::placeholders::_1));

  live_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/vox_nav/live_cloud_crop", rclcpp::SystemDefaultsQoS());

  map_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/vox_nav/map_cloud_crop", rclcpp::SystemDefaultsQoS());

  base_to_map_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/vox_nav/icp", rclcpp::SystemDefaultsQoS());

  new_robot_pose_publisher_ =
    this->create_publisher<geometry_msgs::msg::PoseArray>(
    "/vox_nav/robot_pose_icp", rclcpp::SystemDefaultsQoS());

  // setup TF buffer and listerner to read transforms
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  RCLCPP_INFO(get_logger(), "Creating...");
}

SimpleICP::~SimpleICP()
{
  RCLCPP_INFO(get_logger(), "Destroying...");
}

void SimpleICP::gpsOdomCallback(
  const nav_msgs::msg::Odometry::ConstSharedPtr odom)
{
  std::lock_guard<std::mutex> guard(latest_gps_odom_mutex_);
  latest_gps_odom_ = *odom;
  RCLCPP_INFO(get_logger(), "odom gps");

}

void SimpleICP::cloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
{
  if (map_configured_) {
    std::lock_guard<std::mutex> guard(latest_gps_odom_mutex_);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_curr(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*cloud, *pcl_curr);

    RCLCPP_INFO(get_logger(), "Live Cloud with %d points...", pcl_curr->points.size());

    auto croppped_live_cloud = vox_nav_utilities::cropBox<pcl::PointXYZRGB>(
      pcl_curr,
      Eigen::Vector4f(-10, -10, -5, 1),
      Eigen::Vector4f(10, 10, 5, 1));

    pcl_ros::transformPointCloud(
      "base_link", *croppped_live_cloud, *croppped_live_cloud,
      *tf_buffer_);

    geometry_msgs::msg::PoseStamped curr_robot_pose;

    curr_robot_pose.header.frame_id = "map";
    curr_robot_pose.header.stamp = cloud->header.stamp;
    curr_robot_pose.pose = latest_gps_odom_.pose.pose;

    /*vox_nav_utilities::getCurrentPose(
      curr_robot_pose, *tf_buffer_, "map", "base_link", 0.1);*/

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr map(new pcl::PointCloud<pcl::PointXYZRGB>(map_));

    auto croppped_map_cloud = vox_nav_utilities::cropBox<pcl::PointXYZRGB>(
      map,
      Eigen::Vector4f(
        -10 + curr_robot_pose.pose.position.x,
        -10 + curr_robot_pose.pose.position.y, -5, 1),

      Eigen::Vector4f(
        10 + curr_robot_pose.pose.position.x,
        10 + curr_robot_pose.pose.position.y, 5, 1));

    croppped_map_cloud->header.stamp = pcl_curr->header.stamp;
    croppped_map_cloud->header.seq = pcl_curr->header.seq;

    pcl_ros::transformPointCloud(
      "base_link", *croppped_map_cloud, *croppped_map_cloud,
      *tf_buffer_);

    croppped_map_cloud =
      vox_nav_utilities::downsampleInputCloud<pcl::PointXYZRGB>(croppped_map_cloud, 0.2);
    croppped_live_cloud =
      vox_nav_utilities::downsampleInputCloud<pcl::PointXYZRGB>(croppped_live_cloud, 0.2);

    thrust::host_vector<Eigen::Vector3f> map_points, live_points;

    for (int i = 0; i < croppped_map_cloud->points.size(); ++i) {
      auto p = croppped_map_cloud->points[i];
      Eigen::Vector3f point_eig(p.x, p.y, p.z);
      map_points.push_back(point_eig);
    }

    for (int i = 0; i < croppped_live_cloud->points.size(); ++i) {
      auto p = croppped_live_cloud->points[i];
      Eigen::Vector3f point_eig(p.x, p.y, p.z);
      live_points.push_back(point_eig);
    }

    auto map_points_cupoch = std::make_shared<cupoch::geometry::PointCloud>();
    auto live_points_cupoch = std::make_shared<cupoch::geometry::PointCloud>();
    map_points_cupoch->SetPoints(map_points);
    live_points_cupoch->SetPoints(live_points);

    // ICP
    Eigen::Matrix4f eye = Eigen::Matrix4f::Identity();
    auto point_to_point =
      cupoch::registration::TransformationEstimationPointToPoint();
    cupoch::registration::ICPConvergenceCriteria criteria;
    criteria.max_iteration_ = 20;
    auto res = cupoch::registration::RegistrationICP(
      *live_points_cupoch, *map_points_cupoch, 3.0, eye,
      point_to_point, criteria);
    live_points_cupoch->Transform(res.transformation_);

    Eigen::Affine3f T;
    T.matrix() = res.transformation_;
    Eigen::Affine3d T_d = T.cast<double>();
    auto transformation = tf2::eigenToTransform(T_d);

    geometry_msgs::msg::PoseStamped a, b;
    tf2::doTransform(a, b, transformation);

    b.header.frame_id = "base_link";
    b.header.stamp = curr_robot_pose.header.stamp;
    a.header.stamp = curr_robot_pose.header.stamp;

    rclcpp::Duration transform_tolerance(0, 500);

    auto result = vox_nav_utilities::transformPose(
      tf_buffer_, "map", b, a, transform_tolerance);

    geometry_msgs::msg::PoseArray icp_robot_poses;
    icp_robot_poses.header.frame_id = "map";
    icp_robot_poses.header.stamp = now();
    icp_robot_poses.poses.push_back(curr_robot_pose.pose);
    icp_robot_poses.poses.push_back(a.pose);
    new_robot_pose_publisher_->publish(icp_robot_poses);

    geometry_msgs::msg::PoseWithCovarianceStamped icp_pose;
    icp_pose.header.stamp = curr_robot_pose.header.stamp;
    icp_pose.header.frame_id = "map";
    icp_pose.pose.pose = a.pose;
    base_to_map_pose_pub_->publish(icp_pose);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr live_cloud_crop(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr map_cloud_crop(new pcl::PointCloud<pcl::PointXYZRGB>());

    for (auto && i : live_points_cupoch->GetPoints()) {
      pcl::PointXYZRGB p;
      p.x = i.x();
      p.y = i.y();
      p.z = i.z();
      p.r = 255;
      p.a = 200;
      live_cloud_crop->points.push_back(p);
    }

    for (auto && i : map_points_cupoch->GetPoints()) {
      pcl::PointXYZRGB p;
      p.x = i.x();
      p.y = i.y();
      p.z = i.z();
      p.b = 255;
      p.a = 200;
      map_cloud_crop->points.push_back(p);
    }

    sensor_msgs::msg::PointCloud2 live_cloud_crop_msg, map_cloud_crop_msg;

    pcl::toROSMsg(*live_cloud_crop, live_cloud_crop_msg);
    pcl::toROSMsg(*map_cloud_crop, map_cloud_crop_msg);
    live_cloud_crop_msg.header = cloud->header;
    live_cloud_crop_msg.header.frame_id = "base_link";
    map_cloud_crop_msg.header = cloud->header;
    map_cloud_crop_msg.header.frame_id = "base_link";

    live_cloud_pub_->publish(live_cloud_crop_msg);
    map_cloud_pub_->publish(map_cloud_crop_msg);

    std::cout << res.transformation_ << std::endl;
  }
}

void SimpleICP::mapCloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
{
  std::call_once(
    get_map_cloud_once_, [&]()
    {
      pcl::fromROSMsg(*cloud, map_);
      map_configured_ = true;
      RCLCPP_INFO(get_logger(), "Map Cloud with %d points...", map_.points.size());
    });
}

int main(int argc, char const * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimpleICP>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
