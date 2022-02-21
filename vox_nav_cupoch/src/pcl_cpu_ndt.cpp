// Copyright (c) 2022 Fetullah Atas, Norwegian University of Life Sciences
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

#include "vox_nav_cupoch/pcl_cpu_ndt.hpp"

#include <string>
#include <vector>
#include <memory>
#include <algorithm>

using namespace vox_nav_cupoch;

PCLCPUNDT::PCLCPUNDT()
: Node("pcl_cpu_ndt_rclcpp_node")
{

  live_cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/ouster/points",
    rclcpp::SensorDataQoS(),
    std::bind(
      &PCLCPUNDT::liveCloudCallback, this, std::placeholders::_1));

  map_cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "vox_nav/map_server/octomap_pointcloud",
    rclcpp::SensorDataQoS(),
    std::bind(
      &PCLCPUNDT::mapCloudCallback, this, std::placeholders::_1));

  gps_odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odometry/gps",
    rclcpp::SensorDataQoS(),
    std::bind(&PCLCPUNDT::gpsOdomCallback, this, std::placeholders::_1));

  live_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "vox_nav/cupoch/live_cloud_crop", rclcpp::SystemDefaultsQoS());

  map_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "vox_nav/cupoch/map_cloud_crop", rclcpp::SystemDefaultsQoS());

  base_to_map_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "vox_nav/cupoch/icp_base_to_map_pose", rclcpp::SystemDefaultsQoS());

  new_robot_pose_publisher_ =
    this->create_publisher<geometry_msgs::msg::PoseArray>(
    "vox_nav/cupoch/icp_robot_pose", rclcpp::SystemDefaultsQoS());

  // setup TF buffer and listerner to read transforms
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  latest_gps_odom_ = std::make_shared<nav_msgs::msg::Odometry>();
  map_cloud_ = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();


  // Define parameters
  declare_parameter("x_bound", params_.x_bound);
  declare_parameter("y_bound", params_.y_bound);
  declare_parameter("z_bound", params_.z_bound);
  declare_parameter("downsample_voxel_size", params_.downsample_voxel_size);
  declare_parameter("max_icp_iter", params_.max_icp_iter);
  declare_parameter("transformation_epsilon", params_.transformation_epsilon);
  declare_parameter("step_size", params_.step_size);
  declare_parameter("resolution", params_.resolution);
  declare_parameter("max_correspondence_distance", params_.max_correspondence_distance);
  declare_parameter("debug", params_.debug);

  get_parameter("x_bound", params_.x_bound);
  get_parameter("y_bound", params_.y_bound);
  get_parameter("z_bound", params_.z_bound);
  get_parameter("downsample_voxel_size", params_.downsample_voxel_size);
  get_parameter("max_icp_iter", params_.max_icp_iter);
  get_parameter("transformation_epsilon", params_.transformation_epsilon);
  get_parameter("step_size", params_.step_size);
  get_parameter("resolution", params_.resolution);
  get_parameter("max_correspondence_distance", params_.max_correspondence_distance);
  get_parameter("debug", params_.debug);

  // Print parameters
  RCLCPP_INFO_STREAM(get_logger(), "x_bound " << params_.x_bound);
  RCLCPP_INFO_STREAM(get_logger(), "y_bound " << params_.y_bound);
  RCLCPP_INFO_STREAM(get_logger(), "z_bound " << params_.z_bound);
  RCLCPP_INFO_STREAM(get_logger(), "downsample_voxel_size " << params_.downsample_voxel_size);
  RCLCPP_INFO_STREAM(get_logger(), "max_icp_iter " << params_.max_icp_iter);
  RCLCPP_INFO_STREAM(get_logger(), "transformation_epsilon " << params_.transformation_epsilon);
  RCLCPP_INFO_STREAM(get_logger(), "step_size " << params_.step_size);
  RCLCPP_INFO_STREAM(get_logger(), "resolution " << params_.resolution);

  RCLCPP_INFO_STREAM(
    get_logger(), "max_correspondence_distance " << params_.max_correspondence_distance);
  RCLCPP_INFO_STREAM(get_logger(), "debug " << params_.debug);

  RCLCPP_INFO(get_logger(), "Creating...");
}

PCLCPUNDT::~PCLCPUNDT()
{
  RCLCPP_INFO(get_logger(), "Destroying...");
}

void PCLCPUNDT::gpsOdomCallback(
  const nav_msgs::msg::Odometry::ConstSharedPtr odom)
{
  std::lock_guard<std::mutex> guard(latest_gps_odom_mutex_);
  latest_gps_odom_ = std::make_shared<nav_msgs::msg::Odometry>(*odom);
}

void PCLCPUNDT::liveCloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
{
  if (map_configured_) {
    std::lock_guard<std::mutex> guard(latest_gps_odom_mutex_);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_curr(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*cloud, *pcl_curr);


    auto croppped_live_cloud = vox_nav_utilities::cropBox<pcl::PointXYZRGB>(
      pcl_curr,
      Eigen::Vector4f(-params_.x_bound, -params_.y_bound, -params_.z_bound, 1),
      Eigen::Vector4f(params_.x_bound, params_.y_bound, params_.z_bound, 1));

    pcl_ros::transformPointCloud(
      "base_link", *croppped_live_cloud, *croppped_live_cloud, *tf_buffer_);

    geometry_msgs::msg::PoseStamped curr_robot_pose;

    curr_robot_pose.header.frame_id = "map";
    curr_robot_pose.header.stamp = cloud->header.stamp;
    curr_robot_pose.pose = latest_gps_odom_->pose.pose;

    auto croppped_map_cloud = vox_nav_utilities::cropBox<pcl::PointXYZRGB>(
      map_cloud_,
      Eigen::Vector4f(
        -params_.x_bound + curr_robot_pose.pose.position.x,
        -params_.y_bound + curr_robot_pose.pose.position.y,
        -params_.z_bound + curr_robot_pose.pose.position.z, 1),

      Eigen::Vector4f(
        params_.x_bound + curr_robot_pose.pose.position.x,
        params_.y_bound + curr_robot_pose.pose.position.y,
        params_.z_bound + curr_robot_pose.pose.position.z, 1));

    croppped_map_cloud->header.stamp = pcl_curr->header.stamp;
    croppped_map_cloud->header.seq = pcl_curr->header.seq;

    pcl_ros::transformPointCloud(
      "base_link", *croppped_map_cloud, *croppped_map_cloud, *tf_buffer_);

    croppped_map_cloud =
      vox_nav_utilities::downsampleInputCloud<pcl::PointXYZRGB>(
      croppped_map_cloud, params_.downsample_voxel_size);
    croppped_live_cloud =
      vox_nav_utilities::downsampleInputCloud<pcl::PointXYZRGB>(
      croppped_live_cloud, params_.downsample_voxel_size);

    pcl::NormalDistributionsTransform<pcl::PointXYZRGB, pcl::PointXYZRGB> ndt;
    // Setting scale dependent NDT parameters
    // Setting minimum transformation difference for termination condition.
    ndt.setTransformationEpsilon(params_.transformation_epsilon);
    // Setting maximum step size for More-Thuente line search.
    ndt.setStepSize(params_.step_size);
    //Setting Resolution of NDT grid structure (VoxelGridCovariance).
    ndt.setResolution(params_.resolution);

    // Setting max number of registration iterations.
    ndt.setMaximumIterations(params_.max_icp_iter);

    // Setting point cloud to be aligned.
    ndt.setInputSource(croppped_live_cloud);
    // Setting point cloud to be aligned to.
    ndt.setInputTarget(croppped_map_cloud);

    // Set initial alignment estimate found using robot odometry.
    Eigen::AngleAxisf init_rotation(0.0, Eigen::Vector3f::UnitZ());
    Eigen::Translation3f init_translation(0.0, 0.0, 0.0);
    Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();

    // Calculating required rigid transform to align the input cloud to the target cloud.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    ndt.align(*output_cloud, init_guess);


    Eigen::Affine3f T;
    T.matrix() = ndt.getFinalTransformation();
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

    sensor_msgs::msg::PointCloud2 live_cloud_crop_msg, map_cloud_crop_msg;

    pcl::toROSMsg(*output_cloud, live_cloud_crop_msg);
    pcl::toROSMsg(*croppped_map_cloud, map_cloud_crop_msg);
    live_cloud_crop_msg.header = cloud->header;
    live_cloud_crop_msg.header.frame_id = "base_link";
    map_cloud_crop_msg.header = cloud->header;
    map_cloud_crop_msg.header.frame_id = "base_link";

    live_cloud_pub_->publish(live_cloud_crop_msg);
    map_cloud_pub_->publish(map_cloud_crop_msg);

    if (params_.debug) {
      RCLCPP_INFO(
        get_logger(), "Did ICP with Live Cloud of %d points...", output_cloud->points.size());
      RCLCPP_INFO(
        get_logger(), "Did ICP with Map Cloud of %d points...", croppped_map_cloud->points.size());
      std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged() <<
        " score: " << ndt.getFitnessScore() << std::endl;
      std::cout << "Resulting transfrom: \n:" << ndt.getFinalTransformation() << std::endl;
    }
  }
}

void PCLCPUNDT::mapCloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
{
  std::call_once(
    get_map_cloud_once_, [&]()
    {
      pcl::fromROSMsg(*cloud, *map_cloud_);
      map_configured_ = true;
      RCLCPP_INFO(get_logger(), "Map Cloud with %d points...", map_cloud_->points.size());
    });
}

int main(int argc, char const * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PCLCPUNDT>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
