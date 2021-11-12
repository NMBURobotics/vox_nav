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
#include "vox_nav_utilities/pcl_helpers.hpp"
#include "vox_nav_utilities/tf_helpers.hpp"
#include "vox_nav_utilities/map_manager_helpers.hpp"
#include <pcl/common/common.h>
#include <pcl_ros/transforms.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/crop_box.h>
#include <vox_nav_utilities/pointnet2_utils.hpp>
#include <tf2_eigen/tf2_eigen.h>

#include <queue>

namespace vox_nav_utilities
{

  class DynamicPoints : public rclcpp::Node
  {

  public:
    DynamicPoints();
    ~DynamicPoints();

    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::PointCloud2,
        nav_msgs::msg::Odometry,
        sensor_msgs::msg::Imu> CloudOdomApprxTimeSyncPolicy;
    typedef message_filters::Synchronizer<CloudOdomApprxTimeSyncPolicy>
      CloudOdomApprxTimeSyncer;

    void cloudOdomCallback(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud,
      const nav_msgs::msg::Odometry::ConstSharedPtr & odom,
      const sensor_msgs::msg::Imu::ConstSharedPtr & imu);

    void shoot(
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_vector
    );

  private:
    rclcpp::TimerBase::SharedPtr timer_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_subscriber_;
    message_filters::Subscriber<nav_msgs::msg::Odometry> odom_subscriber_;
    message_filters::Subscriber<sensor_msgs::msg::Imu> imu_subscriber_;
    std::shared_ptr<CloudOdomApprxTimeSyncer> cloud_odom_data_approx_time_syncher_;

    int sequence_horizon_;
    double dt_;

    std::vector<std::tuple<sensor_msgs::msg::PointCloud2,
      nav_msgs::msg::Odometry, sensor_msgs::msg::Imu>> cloud_odom_vector_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

    rclcpp::Time last_recieved_msg_stamp_;
    rclcpp::Time stamp_;

  };

  DynamicPoints::DynamicPoints()
  : Node("dynamic_points_node")
  {
    cloud_subscriber_.subscribe(this, "points", rmw_qos_profile_sensor_data);
    odom_subscriber_.subscribe(this, "odom", rmw_qos_profile_sensor_data);
    imu_subscriber_.subscribe(this, "imu", rmw_qos_profile_sensor_data);

    declare_parameter("sequence_horizon", 0);
    get_parameter("sequence_horizon", sequence_horizon_);

    declare_parameter("dt", 0.0);
    get_parameter("dt", dt_);

    cloud_odom_data_approx_time_syncher_.reset(
      new CloudOdomApprxTimeSyncer(
        CloudOdomApprxTimeSyncPolicy(100),
        cloud_subscriber_,
        odom_subscriber_,
        imu_subscriber_));

    cloud_odom_data_approx_time_syncher_->registerCallback(
      std::bind(
        &DynamicPoints::cloudOdomCallback, this,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3));

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "merged", rclcpp::SystemDefaultsQoS());
  }

  DynamicPoints::~DynamicPoints()
  {
  }

  void DynamicPoints::cloudOdomCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud,
    const nav_msgs::msg::Odometry::ConstSharedPtr & odom,
    const sensor_msgs::msg::Imu::ConstSharedPtr & imu)
  {
    last_recieved_msg_stamp_ = cloud->header.stamp;

    RCLCPP_INFO(get_logger(), "Recieved a msg");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr curr_pcl(new pcl::PointCloud<pcl::PointXYZRGB>());

    pcl::fromROSMsg(*cloud, *curr_pcl);

    // Define cropBox limit
    Eigen::Vector4f min_pt(-20.0f, -20.0f, -4.0f, 1.0f);
    Eigen::Vector4f max_pt(20.0f, 20.0f, 4.0f, 1.0f);

    // Test the PointCloud<PointT> method
    pcl::CropBox<pcl::PointXYZRGB> cropBoxFilter(false);
    cropBoxFilter.setInputCloud(curr_pcl);
    // Cropbox slightly bigger then bounding box of points
    cropBoxFilter.setNegative(false);
    cropBoxFilter.setMin(min_pt);
    cropBoxFilter.setMax(max_pt);
    cropBoxFilter.filter(*curr_pcl);

    curr_pcl->is_dense = false;
    boost::shared_ptr<std::vector<int>> indices(new std::vector<int>);
    pcl::removeNaNFromPointCloud(*curr_pcl, *indices);
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(curr_pcl);
    extract.setIndices(indices);
    extract.setNegative(false);
    extract.filter(*curr_pcl);

    curr_pcl = vox_nav_utilities::downsampleInputCloud<pcl::PointXYZRGB>(curr_pcl, 0.2);
    curr_pcl = vox_nav_utilities::set_cloud_color(curr_pcl, {200, 0, 0});
    RCLCPP_INFO(get_logger(), "Cloud has %d points", curr_pcl->points.size());

    shoot(curr_pcl);
    /*sensor_msgs::msg::PointCloud2 pcl_msg;
    pcl::toROSMsg(*merged, pcl_msg);
    pcl_msg.header = std::get<0>(cloud_odom_vector_.back()).header;
    pub_->publish(pcl_msg);
    cloud_odom_vector_.erase(cloud_odom_vector_.begin());*/

  }

  void DynamicPoints::shoot(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_vector)
  {
    int kFPS = 128;
    int kMAX_N_POINTS_IN_RADIUS = 8;
    double kRADIUS = 0.3;
    torch::Device cuda_device = torch::kCUDA;
    auto this_scan_as_tensor = pointnet2_utils::load_pcl_as_torch_tensor(
      cloud_vector,
      cloud_vector->points.size(),
      cuda_device);

    // to test FPS(Furthest-point-sampleing algorithm) =============================
    at::Tensor fps_sampled_tensor_indices = pointnet2_utils::farthest_point_sample(
      this_scan_as_tensor,
      kFPS);

    at::Tensor fps_sampled_tensor = pointnet2_utils::index_points(
      this_scan_as_tensor,
      fps_sampled_tensor_indices
    );

    at::Tensor group_idx = pointnet2_utils::query_ball_point(
      kRADIUS,
      kMAX_N_POINTS_IN_RADIUS,
      this_scan_as_tensor,
      fps_sampled_tensor);
    auto grouped_xyz = pointnet2_utils::index_points(this_scan_as_tensor, group_idx);
  }

}  // namespace

int main(int argc, char const * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vox_nav_utilities::DynamicPoints>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
