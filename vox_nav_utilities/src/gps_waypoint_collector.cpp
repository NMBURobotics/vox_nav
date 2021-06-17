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

#include <iostream>
#include <memory>
#include <utility>

#include "vox_nav_utilities/gps_waypoint_collector.hpp"

namespace vox_nav_utilities
{

  GPSWaypointCollector::GPSWaypointCollector()
  : Node("gps_waypoint_collector_rclcpp_node"), is_first_msg_recieved_(false)
  {
    navsat_fix_subscriber_.subscribe(this, "/gps/fix", rmw_qos_profile_sensor_data);
    imu_subscriber_.subscribe(this, "/imu/data", rmw_qos_profile_sensor_data);

    sensor_data_approx_time_syncher_.reset(
      new SensorDataApprxTimeSyncer(
        SensorDataApprxTimeSyncPolicy(100),
        navsat_fix_subscriber_,
        imu_subscriber_));

    sensor_data_approx_time_syncher_->registerCallback(
      std::bind(
        &GPSWaypointCollector::sensorDataCallback, this, std::placeholders::_1,
        std::placeholders::_2));
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&GPSWaypointCollector::timerCallback, this));
  }

  GPSWaypointCollector::~GPSWaypointCollector()
  {
  }

  void GPSWaypointCollector::timerCallback()
  {
    RCLCPP_INFO(this->get_logger(), "Entering to timer callback!, this is periodicly called");
    if (is_first_msg_recieved_) {
      std::lock_guard<std::mutex> guard(global_mutex_);
      tf2::Quaternion q(
        reusable_imu_msg_.orientation.x,
        reusable_imu_msg_.orientation.y,
        reusable_imu_msg_.orientation.z,
        reusable_imu_msg_.orientation.w);
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      RCLCPP_INFO(
        this->get_logger(),
        "curr_gps_waypoint: [%.8f, %.8f, %.8f, %.8f]", reusable_navsat_msg_.latitude,
        reusable_navsat_msg_.longitude, reusable_navsat_msg_.altitude, yaw);
    }
  }

  void GPSWaypointCollector::sensorDataCallback(
    const sensor_msgs::msg::NavSatFix::ConstSharedPtr & gps,
    const sensor_msgs::msg::Imu::ConstSharedPtr & imu)
  {
    std::lock_guard<std::mutex> guard(global_mutex_);
    reusable_navsat_msg_ = *gps;
    reusable_imu_msg_ = *imu;
    is_first_msg_recieved_ = true;
  }

  std::pair<sensor_msgs::msg::NavSatFix,
    sensor_msgs::msg::Imu> GPSWaypointCollector::getLatestOrientedGPSCoordinates() const
  {
    std::pair<sensor_msgs::msg::NavSatFix, sensor_msgs::msg::Imu>
    latest_oriented_gps_coordinates(reusable_navsat_msg_, reusable_imu_msg_);
    return latest_oriented_gps_coordinates;
  }

  bool GPSWaypointCollector::isOrientedGPSDataReady() const
  {
    return is_first_msg_recieved_;
  }

}  // namespace vox_nav_utilities
