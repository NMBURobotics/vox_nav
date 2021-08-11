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

#ifndef VOX_NAV_UTILITIES__GPS_WAYPOINT_COLLECTOR_HPP_
#define VOX_NAV_UTILITIES__GPS_WAYPOINT_COLLECTOR_HPP_

#include <mutex>
#include <utility>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "message_filters/synchronizer.h"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace vox_nav_utilities
{


  class GPSWaypointCollector : public rclcpp::Node
  {
  public:
    // IF ABSOLUTE HEADING COMES FROM IMU
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::NavSatFix,
        sensor_msgs::msg::Imu> GpsImuDataApprxTimeSyncPolicy;
    typedef message_filters::Synchronizer<GpsImuDataApprxTimeSyncPolicy>
      GpsImuDataApprxTimeSyncer;

    // IF ABSOLUTE HEADING COMES FROM NMEA DRIVER
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::NavSatFix,
        geometry_msgs::msg::QuaternionStamped> GpsQuaternionDataApprxTimeSyncPolicy;
    typedef message_filters::Synchronizer<GpsQuaternionDataApprxTimeSyncPolicy>
      GpsQuaternionDataApprxTimeSyncer;

    /**
     * @brief Construct a new GPSWaypointCollector object
     *
     */
    GPSWaypointCollector();

    /**
     * @brief Destroy the GPSWaypointCollector object
     *
     */
    ~GPSWaypointCollector();

    /**
   * @brief periodically called function to log latest gps and imu data
   *
   */
    void timerCallback();

    /**
     * @brief subscribe to imu anf gps and update global sensor data vars with latest recieved msgs
     *
     * @param gps
     * @param imu
     */
    void gpsImuDataCallback(
      const sensor_msgs::msg::NavSatFix::ConstSharedPtr & gps,
      const sensor_msgs::msg::Imu::ConstSharedPtr & imu);


    /**
     * @brief subscribe to imu anf gps and update global sensor data vars with latest recieved msgs
     *
     * @param gps
     * @param quat
     */
    void gpsQuaternionDataCallback(
      const sensor_msgs::msg::NavSatFix::ConstSharedPtr & gps,
      const geometry_msgs::msg::QuaternionStamped::ConstSharedPtr & quat);

    /**
     * @brief Get the Latest Oriented GPS Coordinates object
     *
     * @return std::pair<sensor_msgs::msg::NavSatFix,
     * sensor_msgs::msg::Imu>
     */
    std::pair<sensor_msgs::msg::NavSatFix,
      sensor_msgs::msg::Imu> getLatestOrientedGPSCoordinates() const;

    /**
     * @brief
     *
     * @return true
     * @return false
     */
    bool isOrientedGPSDataReady() const;

  private:
    rclcpp::TimerBase::SharedPtr timer_;
    message_filters::Subscriber<sensor_msgs::msg::NavSatFix> navsat_fix_subscriber_;
    message_filters::Subscriber<sensor_msgs::msg::Imu> imu_subscriber_;
    message_filters::Subscriber<geometry_msgs::msg::QuaternionStamped> quaternion_subscriber_;
    std::shared_ptr<GpsImuDataApprxTimeSyncer> gps_imu_data_approx_time_syncher_;
    std::shared_ptr<GpsQuaternionDataApprxTimeSyncer> gps_quaternion_data_approx_time_syncher_;

    sensor_msgs::msg::NavSatFix reusable_navsat_msg_;
    sensor_msgs::msg::Imu reusable_imu_msg_;
    geometry_msgs::msg::QuaternionStamped reusable_quaternion_msg_;

    // to ensure safety when accessing global
    std::mutex global_mutex_;
    bool is_first_msg_recieved_;
  };

}  // namespace vox_nav_utilities

#endif  // VOX_NAV_UTILITIES__GPS_WAYPOINT_COLLECTOR_HPP_
