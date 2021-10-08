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

      for (int i = 0; i < cloud_odom_vector_.size() - 1; i++) {
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

        vox_nav_utilities::getRPYfromMsgQuaternion(
          std::get<2>(cloud_odom_vector_.back()).orientation,
          roll_latest,
          pitch_latest,
          yaw_latest);
        vox_nav_utilities::getRPYfromMsgQuaternion(
          std::get<2>(cloud_odom_vector_[i]).orientation,
          roll,
          pitch,
          yaw);

        tf2::Quaternion quat;
        quat = vox_nav_utilities::getTFQuaternionfromRPY(
          roll_latest - roll,
          pitch_latest - pitch,
          yaw_latest - yaw);

        T.setRotation(quat);
        T.setOrigin(
          tf2::Vector3(
            dist * cos(yaw_latest - yaw),
            dist * sin(yaw_latest - yaw),
            0));

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr curr_pcl(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::fromROSMsg(std::get<0>(cloud_odom_vector_[i]), *curr_pcl);
        pcl_ros::transformPointCloud(*curr_pcl, *curr_pcl, T.inverse());

        if (i == 0) {
          curr_pcl = vox_nav_utilities::set_cloud_color(curr_pcl, {200, 0, 0});
        } else if (i == 1) {
          curr_pcl = vox_nav_utilities::set_cloud_color(curr_pcl, {0, 200, 0});
        } else if (i == 2) {
          curr_pcl = vox_nav_utilities::set_cloud_color(curr_pcl, {0, 0, 200});
        } else if (i == 3) {
          curr_pcl = vox_nav_utilities::set_cloud_color(curr_pcl, {200, 0, 200});
        }

        transformed_pcl_sequences.push_back(curr_pcl);
      }

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr latest_pcl(new pcl::PointCloud<pcl::PointXYZRGB>());
      pcl::fromROSMsg(std::get<0>(cloud_odom_vector_.back()), *latest_pcl);
      latest_pcl = vox_nav_utilities::set_cloud_color(latest_pcl, {255, 255, 255});
      transformed_pcl_sequences.push_back(latest_pcl);

      shoot(transformed_pcl_sequences);

      for (auto && i : transformed_pcl_sequences) {
        *merged += *i;
      }

      sensor_msgs::msg::PointCloud2 pcl_msg;
      pcl::toROSMsg(*merged, pcl_msg);
      pcl_msg.header = std::get<0>(cloud_odom_vector_.back()).header;
      pub_->publish(pcl_msg);

      cloud_odom_vector_.erase(cloud_odom_vector_.begin());
    }
  }


  void DynamicPoints::shoot(
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> & cloud_vector)
  {

    RCLCPP_INFO(
      get_logger(), "CLASHIN %d POINT CLOUDS", cloud_vector.size());

    for (size_t f = 0; f < cloud_vector.size() - 1; f++) {

      pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
      kdtree.setInputCloud(cloud_vector[f + 1]);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr static_points(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::PointIndices::Ptr static_point_indices(new pcl::PointIndices());
      pcl::ExtractIndices<pcl::PointXYZRGB> extract;
      extract.setInputCloud(cloud_vector[f + 1]);

      std::unordered_set<int> unique_indices;

      RCLCPP_INFO(
        get_logger(), "GONNA PROCESS A CLOUD WITH %d POINTS", cloud_vector[f + 1]->points.size());

      for (auto && k : cloud_vector[f]->points) {
        pcl::PointXYZRGB search_point = k;
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        float radius = 0.05;

        if (kdtree.radiusSearch(
            search_point, radius, pointIdxRadiusSearch,
            pointRadiusSquaredDistance) > 0)
        {
          for (std::size_t j = 0; j < pointIdxRadiusSearch.size(); ++j) {
            unique_indices.insert(pointIdxRadiusSearch[j]);
          }
        }
      }

      for (auto && t : unique_indices) {
        static_point_indices->indices.push_back(t);
      }

      extract.setIndices(static_point_indices);
      extract.setNegative(false);
      extract.filter(*cloud_vector[f + 1]);
    }
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
