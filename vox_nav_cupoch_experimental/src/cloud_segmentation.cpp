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

class CloudSegmentation : public rclcpp::Node
{

public:
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

  template<typename KeyType, typename ValueType>
  std::pair<KeyType, ValueType> get_max(const std::map<KeyType, ValueType> & x)
  {
    using pairtype = std::pair<KeyType, ValueType>;
    return *std::max_element(
      x.begin(), x.end(), [](const pairtype & p1, const pairtype & p2) {
        return p1.second < p2.second;
      });
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_subscriber_;
  message_filters::Subscriber<nav_msgs::msg::Odometry> odom_subscriber_;
  message_filters::Subscriber<sensor_msgs::msg::Imu> imu_subscriber_;
  std::shared_ptr<CloudOdomApprxTimeSyncer> cloud_odom_data_approx_time_syncher_;

  double dt_;
  double sensor_height_;

  bool recieved_first_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
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

  last_recieved_msg_stamp_ = now();

}

CloudSegmentation::~CloudSegmentation()
{
}

void CloudSegmentation::cloudOdomCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud,
  const nav_msgs::msg::Odometry::ConstSharedPtr & odom,
  const sensor_msgs::msg::Imu::ConstSharedPtr & imu)
{

  stamp_ = cloud->header.stamp;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr curr_pcl(new pcl::PointCloud<pcl::PointXYZRGB>());

  pcl::PCLPointCloud2 pcl_pc;
  pcl_conversions::toPCL(*cloud, pcl_pc);
  pcl::fromPCLPointCloud2(pcl_pc, *curr_pcl);

  RCLCPP_INFO(get_logger(), " Writing a cloud with %d points", curr_pcl->points.size());


  curr_pcl = vox_nav_utilities::downsampleInputCloud<pcl::PointXYZRGB>(curr_pcl, 0.1);

  auto denois = denoise_segmented_cloud(curr_pcl, 2.0, 0.2, 120);

  sensor_msgs::msg::PointCloud2 pcl_msg;
  pcl::toROSMsg(*denois, pcl_msg);
  pcl_msg.header = cloud->header;
  pub_->publish(pcl_msg);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr CloudSegmentation::denoise_segmented_cloud(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double radius,
  double tolerated_divergence_rate, int min_num_neighbours)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr denoised_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());


  pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
  kdtree.setInputCloud(cloud);

  for (size_t i = 0; i < cloud->points.size(); i++) {

    pcl::PointXYZRGB searchPoint = cloud->points[i];
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    if (kdtree.nearestKSearch(
        searchPoint, min_num_neighbours, pointIdxRadiusSearch,
        pointRadiusSquaredDistance) > 0)
    {
      int serach_point_key =
        ((int)searchPoint.r) << 16 | ((int)searchPoint.g) << 8 | ((int)searchPoint.b);

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr neighbours(new pcl::PointCloud<pcl::PointXYZRGB>);
      for (std::size_t j = 0; j < pointIdxRadiusSearch.size(); ++j) {
        neighbours->points.push_back(cloud->points[pointIdxRadiusSearch[j]]);
      }

      std::vector<int> neighbour_labels;
      for (std::size_t j = 0; j < neighbours->points.size(); ++j) {
        int r = neighbours->points[j].r;
        int g = neighbours->points[j].g;
        int b = neighbours->points[j].b;
        int rgb = ((int)r) << 16 | ((int)g) << 8 | ((int)b);
        neighbour_labels.push_back(rgb);
      }

      if (neighbour_labels.size() > 200) {
        continue;
      }

      std::map<int, int> M;
      for (int k = 0; k < neighbour_labels.size(); k++) {
        if (M.find(neighbour_labels[k]) == M.end()) {
          M[neighbour_labels[k]] = 1;
        } else {
          M[neighbour_labels[k]]++;
        }
      }

      auto max = get_max<int, int>(M);
      int num_search_point_neigbours_same_class;
      auto search_point_neigbours_same_class = M.find(serach_point_key);
      if (search_point_neigbours_same_class == M.end()) {
        RCLCPP_ERROR(get_logger(), "Not suitable search point");
        continue;
      } else {
        num_search_point_neigbours_same_class = search_point_neigbours_same_class->second;
      }

      if (max.second > num_search_point_neigbours_same_class) {
        std::uint8_t r = (max.first >> 16) & 0x0000ff;
        std::uint8_t g = (max.first >> 8) & 0x0000ff;
        std::uint8_t b = (max.first) & 0x0000ff;
        searchPoint.r = r;
        searchPoint.g = g;
        searchPoint.b = b;
      }
    }
    searchPoint.a = 255;
    denoised_cloud->points.push_back(searchPoint);
  }

  denoised_cloud->height = 1;
  denoised_cloud->width = denoised_cloud->points.size();
  return denoised_cloud;
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
