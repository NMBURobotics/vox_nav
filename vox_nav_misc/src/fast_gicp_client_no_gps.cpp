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

#include "vox_nav_misc/fast_gicp_client_no_gps.hpp"

#include <string>
#include <vector>
#include <memory>
#include <algorithm>

using namespace vox_nav_misc;

FastGICPClientNoGPS::FastGICPClientNoGPS() : Node("fast_gicp_client_rclcpp_node")
{
  live_cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/ouster/points", rclcpp::SensorDataQoS(),
      std::bind(&FastGICPClientNoGPS::liveCloudCallback, this, std::placeholders::_1));

  map_cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "vox_nav/map_server/octomap_pointcloud", rclcpp::SensorDataQoS(),
      std::bind(&FastGICPClientNoGPS::mapCloudCallback, this, std::placeholders::_1));

  // reliable qos with keep last 5
  auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
  initial_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", qos, std::bind(&FastGICPClientNoGPS::initialPoseCallback, this, std::placeholders::_1));

  live_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("vox_nav/cupoch/live_cloud_crop",
                                                                          rclcpp::SystemDefaultsQoS());

  map_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("vox_nav/cupoch/map_cloud_crop",
                                                                         rclcpp::SystemDefaultsQoS());

  base_to_map_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "vox_nav/cupoch/icp_base_to_map_pose", rclcpp::SystemDefaultsQoS());

  new_robot_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("vox_nav/cupoch/icp_robot_pose",
                                                                                    rclcpp::SystemDefaultsQoS());

  // setup TF buffer and listerner to read transforms
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  map_cloud_ = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  // Define parameters
  declare_parameter("x_bound", params_.x_bound);
  declare_parameter("y_bound", params_.y_bound);
  declare_parameter("z_bound", params_.z_bound);
  declare_parameter("downsample_voxel_size", params_.downsample_voxel_size);
  declare_parameter("max_icp_iter", params_.max_icp_iter);
  declare_parameter("max_correspondence_distance", params_.max_correspondence_distance);
  declare_parameter("method", params_.method);
  declare_parameter("num_threads", params_.num_threads);
  declare_parameter("debug", params_.debug);

  get_parameter("x_bound", params_.x_bound);
  get_parameter("y_bound", params_.y_bound);
  get_parameter("z_bound", params_.z_bound);
  get_parameter("downsample_voxel_size", params_.downsample_voxel_size);
  get_parameter("max_icp_iter", params_.max_icp_iter);
  get_parameter("max_correspondence_distance", params_.max_correspondence_distance);
  get_parameter("method", params_.method);
  get_parameter("num_threads", params_.num_threads);
  get_parameter("debug", params_.debug);

  // Print parameters
  RCLCPP_INFO_STREAM(get_logger(), "x_bound " << params_.x_bound);
  RCLCPP_INFO_STREAM(get_logger(), "y_bound " << params_.y_bound);
  RCLCPP_INFO_STREAM(get_logger(), "z_bound " << params_.z_bound);
  RCLCPP_INFO_STREAM(get_logger(), "downsample_voxel_size " << params_.downsample_voxel_size);
  RCLCPP_INFO_STREAM(get_logger(), "max_icp_iter " << params_.max_icp_iter);
  RCLCPP_INFO_STREAM(get_logger(), "max_correspondence_distance " << params_.max_correspondence_distance);
  RCLCPP_INFO_STREAM(get_logger(), "method " << params_.method);
  RCLCPP_INFO_STREAM(get_logger(), "num_threads " << params_.num_threads);
  RCLCPP_INFO_STREAM(get_logger(), "debug " << params_.debug);

  RCLCPP_INFO(get_logger(), "PLEASE PROVIDE INITIAL POSE FROM RVIZ");

  RCLCPP_INFO(get_logger(), "Creating...");
}

FastGICPClientNoGPS::~FastGICPClientNoGPS()
{
  RCLCPP_INFO(get_logger(), "Destroying...");
}

void FastGICPClientNoGPS::initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
{
  initial_pose_ = std::make_shared<geometry_msgs::msg::PoseWithCovariance>(msg->pose);
  initial_pose_recieved_ = true;
  RCLCPP_INFO(get_logger(), "Receieved Initial pose");
}

void FastGICPClientNoGPS::liveCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
{
  if (map_configured_ && initial_pose_recieved_)
  {
    // Transform live cloud to base_link frame
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_curr(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*cloud, *pcl_curr);

    /*if (!pcl_ros::transformPointCloud("base_link", *pcl_curr, *pcl_curr, *tf_buffer_))
    {
      RCLCPP_WARN(get_logger(), "Error Encountered at transfrom, doing nothing");
      return;
    }*/

    // Crop live cloud
    auto croppped_live_cloud = vox_nav_utilities::cropBox<pcl::PointXYZ>(
        pcl_curr, Eigen::Vector4f(-params_.x_bound, -params_.y_bound, -params_.z_bound, 1),
        Eigen::Vector4f(params_.x_bound, params_.y_bound, params_.z_bound, 1));

    // This will be the initial guess for robot pose
    geometry_msgs::msg::PoseStamped curr_robot_pose;
    curr_robot_pose.header.frame_id = "map";
    curr_robot_pose.header.stamp = cloud->header.stamp;
    curr_robot_pose.pose = initial_pose_->pose;

    auto croppped_map_cloud = vox_nav_utilities::cropBox<pcl::PointXYZ>(
        map_cloud_,
        Eigen::Vector4f(-params_.x_bound + curr_robot_pose.pose.position.x,
                        -params_.y_bound + curr_robot_pose.pose.position.y,
                        -params_.z_bound + curr_robot_pose.pose.position.z, 1),

        Eigen::Vector4f(params_.x_bound + curr_robot_pose.pose.position.x,
                        params_.y_bound + curr_robot_pose.pose.position.y,
                        params_.z_bound + curr_robot_pose.pose.position.z, 1));

    croppped_map_cloud->header.stamp = pcl_curr->header.stamp;
    croppped_map_cloud->header.seq = pcl_curr->header.seq;

    // convert tf transform to eigen transform
    Eigen::Affine3d initial_pose_eigen;
    tf2::fromMsg(initial_pose_->pose, initial_pose_eigen);

    // convert eigen transform to eigen matrix
    Eigen::Matrix4d initial_pose_eigen_matrix = initial_pose_eigen.matrix();

    // finally transfrom live cloud to map frame
    pcl::transformPointCloud(*croppped_live_cloud, *croppped_live_cloud, initial_pose_eigen_matrix);

    croppped_map_cloud =
        vox_nav_utilities::downsampleInputCloud<pcl::PointXYZ>(croppped_map_cloud, params_.downsample_voxel_size);
    croppped_live_cloud =
        vox_nav_utilities::downsampleInputCloud<pcl::PointXYZ>(croppped_live_cloud, params_.downsample_voxel_size);

    auto aligned = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>::Ptr reg = createRegistration(params_.method, params_.num_threads);
    if (reg == nullptr)
    {
      std::cerr << " registration method is null!, is CUDA available ?" << std::endl;
      return;
    }
    reg->setMaxCorrespondenceDistance(params_.max_correspondence_distance);
    reg->setMaximumIterations(params_.max_icp_iter);
    reg->setInputSource(croppped_live_cloud);
    reg->setInputTarget(croppped_map_cloud);

    reg->align(*aligned);

    auto res_transformation = reg->getFinalTransformation();

    Eigen::Affine3f T;
    T.matrix() = res_transformation;
    Eigen::Affine3d T_d = T.cast<double>();
    auto final_transform = tf2::eigenToTransform(T_d);

    geometry_msgs::msg::PoseStamped identity, resulting_pose;
    tf2::doTransform(identity, resulting_pose, final_transform);

    resulting_pose.header.frame_id = "map";
    resulting_pose.header.stamp = curr_robot_pose.header.stamp;

    geometry_msgs::msg::PoseArray icp_robot_poses;
    icp_robot_poses.header.frame_id = "map";
    icp_robot_poses.header.stamp = curr_robot_pose.header.stamp;
    icp_robot_poses.poses.push_back(resulting_pose.pose);
    new_robot_pose_publisher_->publish(icp_robot_poses);

    geometry_msgs::msg::PoseWithCovarianceStamped icp_pose;
    icp_pose.header.stamp = curr_robot_pose.header.stamp;
    icp_pose.header.frame_id = "map";
    icp_pose.pose.pose = resulting_pose.pose;
    base_to_map_pose_pub_->publish(icp_pose);

    sensor_msgs::msg::PointCloud2 live_cloud_crop_msg, map_cloud_crop_msg;

    pcl::toROSMsg(*aligned, live_cloud_crop_msg);
    pcl::toROSMsg(*croppped_map_cloud, map_cloud_crop_msg);

    live_cloud_crop_msg.header = cloud->header;
    live_cloud_crop_msg.header.frame_id = "map";
    map_cloud_crop_msg.header = cloud->header;
    map_cloud_crop_msg.header.frame_id = "map";

    live_cloud_pub_->publish(live_cloud_crop_msg);
    map_cloud_pub_->publish(map_cloud_crop_msg);

    // initial_pose_->pose = resulting_pose.pose;

    if (params_.debug)
    {
      RCLCPP_INFO(get_logger(), "Did %s with Live Cloud of %d points...", params_.method.c_str(),
                  aligned->points.size());
      RCLCPP_INFO(get_logger(), "Did %s with Map Cloud of %d points...", params_.method.c_str(),
                  croppped_map_cloud->points.size());
      std::cout << "Resulting transfrom: \n" << res_transformation << std::endl;
    }
  }
}

void FastGICPClientNoGPS::mapCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
{
  std::call_once(get_map_cloud_once_, [&]() {
    pcl::fromROSMsg(*cloud, *map_cloud_);
    map_configured_ = true;
    RCLCPP_INFO(get_logger(), "Map Cloud with %d points...", map_cloud_->points.size());
  });
}

pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>::Ptr FastGICPClientNoGPS::createRegistration(std::string method,
                                                                                             int num_threads)
{
  if (method == "GICP")
  {
    auto gicp = pcl::make_shared<fast_gicp::FastGICP<pcl::PointXYZ, pcl::PointXYZ>>();
    gicp->setNumThreads(num_threads);
    return gicp;
  }
  else if (method == "VGICP")
  {
    auto vgicp = pcl::make_shared<fast_gicp::FastVGICP<pcl::PointXYZ, pcl::PointXYZ>>();
    vgicp->setNumThreads(num_threads);
    return vgicp;
  }
  else if (method == "VGICP_CUDA")
  {
    auto vgicp_cuda = pcl::make_shared<fast_gicp::FastVGICPCuda<pcl::PointXYZ, pcl::PointXYZ>>();
    vgicp_cuda->setNearestNeighborSearchMethod(fast_gicp::NearestNeighborMethod::GPU_RBF_KERNEL);
    vgicp_cuda->setResolution(0.1);
    return vgicp_cuda;
  }
  else if (method == "NDT_CUDA")
  {
    auto ndt = pcl::make_shared<fast_gicp::NDTCuda<pcl::PointXYZ, pcl::PointXYZ>>();
    ndt->setResolution(0.1);
    ndt->setDistanceMode(fast_gicp::NDTDistanceMode::P2D);
    ndt->setNeighborSearchMethod(fast_gicp::NeighborSearchMethod::DIRECT_RADIUS, 0.8);
    return ndt;
  }
  std::cerr << "unknown registration method:" << method << std::endl;
  return nullptr;
}

void FastGICPClientNoGPS::swap_source_and_target(pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>::Ptr reg)
{
  fast_gicp::LsqRegistration<pcl::PointXYZ, pcl::PointXYZ>* lsq_reg =
      dynamic_cast<fast_gicp::LsqRegistration<pcl::PointXYZ, pcl::PointXYZ>*>(reg.get());
  if (lsq_reg != nullptr)
  {
    lsq_reg->swapSourceAndTarget();
    return;
  }

  std::cerr << "failed to swap source and target" << std::endl;
}

int main(int argc, char const* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FastGICPClientNoGPS>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
