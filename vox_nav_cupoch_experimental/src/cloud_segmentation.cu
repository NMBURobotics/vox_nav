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

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "message_filters/synchronizer.h"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <pcl/conversions.h> //I believe you were using pcl/ros/conversion.h
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl_ros/transforms.hpp>

#include <queue>
#include <ostream>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <fstream>

#include "cupoch_conversions/cupoch_conversions.hpp"
#include "cupoch/cupoch.h"
#include "vox_nav_cupoch_experimental/visibility_control.h"
#include "vox_nav_utilities/pcl_helpers.hpp"
#include "vox_nav_utilities/tf_helpers.hpp"
#include "vox_nav_utilities/map_manager_helpers.hpp"

class CloudSegmentation : public rclcpp::Node
{

public:
  typedef std::tuple<sensor_msgs::msg::PointCloud2::SharedPtr,
      nav_msgs::msg::Odometry::SharedPtr,
      sensor_msgs::msg::Imu::SharedPtr> data_captute_t;

  CloudSegmentation();
  ~CloudSegmentation();

  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::PointCloud2,
      nav_msgs::msg::Odometry,
      sensor_msgs::msg::Imu>
    CloudOdomApprxTimeSyncPolicy;
  typedef message_filters::Synchronizer<CloudOdomApprxTimeSyncPolicy>
    CloudOdomApprxTimeSyncer;

  void cloudOdomCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud,
    const nav_msgs::msg::Odometry::ConstSharedPtr & odom,
    const sensor_msgs::msg::Imu::ConstSharedPtr & imu);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr denoise_segmented_cloud(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double radius,
    double tolerated_divergence_rate, int min_num_neighbours);

  void shoot(std::vector<data_captute_t> & cloud_vector);

private:
  rclcpp::TimerBase::SharedPtr timer_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_subscriber_;
  message_filters::Subscriber<nav_msgs::msg::Odometry> odom_subscriber_;
  message_filters::Subscriber<sensor_msgs::msg::Imu> imu_subscriber_;
  std::shared_ptr<CloudOdomApprxTimeSyncer> cloud_odom_data_approx_time_syncher_;

  double dt_;
  double sensor_height_;

  std::vector<data_captute_t> cloud_odom_vector_;

  bool recieved_first_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr dyn_point_pub_;

  rclcpp::Time last_recieved_msg_stamp_;
  rclcpp::Time stamp_;
};

CloudSegmentation::CloudSegmentation()
: Node("dynamic_points_node"),
  recieved_first_(false)
{
  cloud_subscriber_.subscribe(this, "points", rmw_qos_profile_sensor_data);
  odom_subscriber_.subscribe(this, "odom", rmw_qos_profile_sensor_data);
  imu_subscriber_.subscribe(this, "imu", rmw_qos_profile_sensor_data);

  declare_parameter("dt", 0.0);
  get_parameter("dt", dt_);

  declare_parameter("sensor_height", 0.0);
  get_parameter("sensor_height", sensor_height_);

  cloud_odom_data_approx_time_syncher_.reset(
    new CloudOdomApprxTimeSyncer(
      CloudOdomApprxTimeSyncPolicy(3000),
      cloud_subscriber_,
      odom_subscriber_,
      imu_subscriber_));

  cloud_odom_data_approx_time_syncher_->registerCallback(
    std::bind(
      &CloudSegmentation::cloudOdomCallback, this,
      std::placeholders::_1,
      std::placeholders::_2,
      std::placeholders::_3));

  pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "merged", rclcpp::SystemDefaultsQoS());

  dyn_point_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "dyn_point_pub", rclcpp::SystemDefaultsQoS());

  last_recieved_msg_stamp_ = now();

  auto captured_data = std::make_tuple<>(
    std::make_shared<sensor_msgs::msg::PointCloud2>(sensor_msgs::msg::PointCloud2()),
    std::make_shared<nav_msgs::msg::Odometry>(nav_msgs::msg::Odometry()),
    std::make_shared<sensor_msgs::msg::Imu>(sensor_msgs::msg::Imu()));

  cloud_odom_vector_ = std::vector<data_captute_t>(2, captured_data);

}

CloudSegmentation::~CloudSegmentation()
{
}

void CloudSegmentation::cloudOdomCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud,
  const nav_msgs::msg::Odometry::ConstSharedPtr & odom,
  const sensor_msgs::msg::Imu::ConstSharedPtr & imu)
{
  last_recieved_msg_stamp_ = cloud->header.stamp;
  // convert to pcl type
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr curr_pcl(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PCLPointCloud2 pcl_pc;
  pcl_conversions::toPCL(*cloud, pcl_pc);
  pcl::fromPCLPointCloud2(pcl_pc, *curr_pcl);

  // remove ground points
  sensor_msgs::PointCloud2ConstIterator<float> iter_label(*cloud, "x");
  std::vector<u_int8_t> labels;
  pcl::IndicesPtr ground_points_indices(new std::vector<int>);
  int couter = 0;
  for (; (iter_label != iter_label.end()); ++iter_label) {
    labels.push_back(iter_label[3]);
    if (iter_label[3] == 40) { // ground point label
      ground_points_indices->push_back(couter);
    }
    couter++;
  }
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud(curr_pcl);
  extract.setIndices(ground_points_indices);
  extract.setNegative(true);
  extract.filter(*curr_pcl);

  auto denoised_cloud = vox_nav_utilities::removeOutliersFromInputCloud(
    curr_pcl, 4, 0.3,
    vox_nav_utilities::OutlierRemovalType::RadiusOutlierRemoval);

  sensor_msgs::msg::PointCloud2 denoised_cloud_msg;
  pcl::toROSMsg(*denoised_cloud, denoised_cloud_msg);
  denoised_cloud_msg.header = cloud->header;
  RCLCPP_INFO(get_logger(), " Writing a cloud with %d points", denoised_cloud->points.size());

  auto curr_cloud_odom_pair =
    std::make_tuple<>(
    std::make_shared<sensor_msgs::msg::PointCloud2>(denoised_cloud_msg),
    std::make_shared<nav_msgs::msg::Odometry>(*odom),
    std::make_shared<sensor_msgs::msg::Imu>(*imu));

  auto source = std::make_shared<cupoch::geometry::PointCloud>();
  cupoch_conversions::rosToCupoch(
    std::make_shared<sensor_msgs::msg::PointCloud2>(denoised_cloud_msg),
    source);

  // START VOXEL STUFF
  double voxel_size = 0.2;
  auto voxel_source = cupoch::geometry::VoxelGrid::CreateFromPointCloud(
    *source, voxel_size);

  auto cupoch_processed = std::make_shared<cupoch::geometry::PointCloud>(*source);

  sensor_msgs::msg::PointCloud2 cupoch_processed_msg;
  cupoch_conversions::cupochToRos(cupoch_processed, cupoch_processed_msg);
  cupoch_processed_msg.header = denoised_cloud_msg.header;


  dyn_point_pub_->publish(cupoch_processed_msg);
  pub_->publish(denoised_cloud_msg);
}

void CloudSegmentation::shoot(std::vector<data_captute_t> & cloud_vector)
{
}


int main(int argc, char const * argv[])
{
  rclcpp::init(argc, argv);
  cupoch::utility::InitializeAllocator();
  auto node = std::make_shared<CloudSegmentation>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
