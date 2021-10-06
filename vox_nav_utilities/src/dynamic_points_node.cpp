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
#include <nav_msgs/msg/odometry.hpp>
#include "message_filters/synchronizer.h"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "vox_nav_utilities/pcl_helpers.hpp"
#include "vox_nav_utilities/tf_helpers.hpp"
#include <pcl/common/common.h>

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
        nav_msgs::msg::Odometry> CloudOdomApprxTimeSyncPolicy;
    typedef message_filters::Synchronizer<CloudOdomApprxTimeSyncPolicy>
      CloudOdomApprxTimeSyncer;

    void cloudOdomCallback(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud,
      const nav_msgs::msg::Odometry::ConstSharedPtr & odom);

  private:
    rclcpp::TimerBase::SharedPtr timer_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_subscriber_;
    message_filters::Subscriber<nav_msgs::msg::Odometry> odom_subscriber_;
    std::shared_ptr<CloudOdomApprxTimeSyncer> cloud_odom_data_approx_time_syncher_;

    int sequence_horizon_;

    std::vector<std::pair<sensor_msgs::msg::PointCloud2,
      nav_msgs::msg::Odometry>> cloud_odom_vector_;
  };

  DynamicPoints::DynamicPoints()
  : Node("dynamic_points_node")
  {
    cloud_subscriber_.subscribe(this, "/ouster/points", rmw_qos_profile_sensor_data);
    odom_subscriber_.subscribe(this, "/odometry/global", rmw_qos_profile_sensor_data);

    declare_parameter("sequence_horizon", 3);
    get_parameter("sequence_horizon", sequence_horizon_);

    cloud_odom_data_approx_time_syncher_.reset(
      new CloudOdomApprxTimeSyncer(
        CloudOdomApprxTimeSyncPolicy(100),
        cloud_subscriber_,
        odom_subscriber_));

    cloud_odom_data_approx_time_syncher_->registerCallback(
      std::bind(
        &DynamicPoints::cloudOdomCallback, this, std::placeholders::_1,
        std::placeholders::_2));
  }

  DynamicPoints::~DynamicPoints()
  {
  }

  void DynamicPoints::cloudOdomCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud,
    const nav_msgs::msg::Odometry::ConstSharedPtr & odom)
  {

    if (cloud_odom_vector_.size() < sequence_horizon_) {
      auto curr_cloud_odom_pair = std::make_pair<>(*cloud, *odom);
      cloud_odom_vector_.push_back(curr_cloud_odom_pair);
    }
    if (cloud_odom_vector_.size() > sequence_horizon_) {
      cloud_odom_vector_.erase(cloud_odom_vector_.begin());
    }

    RCLCPP_INFO(
      get_logger(), "Currently cloud_odom_vector_ has %d elements ", cloud_odom_vector_.size());

    if (cloud_odom_vector_.size() == sequence_horizon_) {


      for (int i = cloud_odom_vector_.size() - 1; i == 0; i--) {

        tf2::Transform T;
        T.setOrigin(
          tf2::Vector3(
            cloud_odom_vector_[i].second.pose.pose.position.x -
            cloud_odom_vector_.back().second.pose.pose.position.x,
            cloud_odom_vector_[i].second.pose.pose.position.y -
            cloud_odom_vector_.back().second.pose.pose.position.y,
            cloud_odom_vector_[i].second.pose.pose.position.z -
            cloud_odom_vector_.back().second.pose.pose.position.z)
        );
        double yaw, pitch, roll;
        double yaw_latest, pitch_latest, roll_latest;

        vox_nav_utilities::getRPYfromMsgQuaternion(
          cloud_odom_vector_[i].second.pose.pose.orientation, roll, pitch, yaw);
        vox_nav_utilities::getRPYfromMsgQuaternion(
          cloud_odom_vector_.back().second.pose.pose.orientation, roll_latest, pitch_latest,
          yaw_latest);
        
          

      }
    }

  }
}

int main(int argc, char const * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vox_nav_utilities::DynamicPoints>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
