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
#include <pcl/common/common.h>
#include <pcl_ros/transforms.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/crop_box.h>
#include <tf2_eigen/tf2_eigen.h>

#include <queue>

#include "cupoch_conversions/cupoch_conversions.hpp"
#include "cupoch/cupoch.h"
#include "vox_nav_cupoch_experimental/visibility_control.h"

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
      std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> & cloud_vector
    );

  private:
    rclcpp::TimerBase::SharedPtr timer_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_subscriber_;
    message_filters::Subscriber<nav_msgs::msg::Odometry> odom_subscriber_;
    message_filters::Subscriber<sensor_msgs::msg::Imu> imu_subscriber_;
    std::shared_ptr<CloudOdomApprxTimeSyncer> cloud_odom_data_approx_time_syncher_;

    int sequence_horizon_;
    double dt_;
    double sensor_height_;

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

    declare_parameter("sensor_height", 0.0);
    get_parameter("sensor_height", sensor_height_);

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

    if (cloud_odom_vector_.size() == 0) {
      auto curr_cloud_odom_pair = std::make_tuple<>(*cloud, *odom, *imu);
      cloud_odom_vector_.push_back(curr_cloud_odom_pair);
      stamp_ = std::get<0>(cloud_odom_vector_.back()).header.stamp;
    }

    if (cloud_odom_vector_.size() < sequence_horizon_ &&
      (last_recieved_msg_stamp_ - stamp_).seconds() > dt_)
    {
      auto curr_cloud_odom_pair = std::make_tuple<>(*cloud, *odom, *imu);
      cloud_odom_vector_.push_back(curr_cloud_odom_pair);
      stamp_ = std::get<0>(cloud_odom_vector_.back()).header.stamp;
    }

    if (cloud_odom_vector_.size() == sequence_horizon_) {
      std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> transformed_pcl_sequences;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged(new pcl::PointCloud<pcl::PointXYZRGB>());

      for (int i = 0; i < cloud_odom_vector_.size(); i++) {
        tf2::Transform T;
        auto dist = tf2::Vector3(
          std::get<1>(cloud_odom_vector_.back()).pose.pose.position.x -
          std::get<1>(cloud_odom_vector_[i]).pose.pose.position.x,
          std::get<1>(cloud_odom_vector_.back()).pose.pose.position.y -
          std::get<1>(cloud_odom_vector_[i]).pose.pose.position.y,
          std::get<1>(cloud_odom_vector_.back()).pose.pose.position.z -
          std::get<1>(cloud_odom_vector_[i]).pose.pose.position.z).length();

        double yaw_latest, pitch_latest, roll_latest;
        double yaw, pitch, roll;


        // Define cropBox limit
        Eigen::Vector4f min_pt(-20.0f, -15.0f, -4.0f, 1.0f);
        Eigen::Vector4f max_pt(20.0f, 15.0f, 4.0f, 1.0f);


      }

      shoot(transformed_pcl_sequences);

      for (auto && i : transformed_pcl_sequences) {
        *merged += *i;
      }

      sensor_msgs::msg::PointCloud2 pcl_msg;
      //pcl::toROSMsg(*merged, pcl_msg);
      pcl_msg.header = std::get<0>(cloud_odom_vector_.back()).header;
      pub_->publish(pcl_msg);

      cloud_odom_vector_.erase(cloud_odom_vector_.begin());
    }
  }

  void DynamicPoints::shoot(
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> & cloud_vector)
  {

    int kFPS = 512;
    int kMAX_N_POINTS_IN_RADIUS = 20;
    double kRADIUS = 0.8;

    RCLCPP_INFO(
      get_logger(), "CLASHING %d POINT CLOUDS", cloud_vector.size());


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
