// Copyright (c) 2020 Fetullah Atas
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

#ifndef BOTANBOT_UTILITIES__GPS_WAYPOINT_COLLECTOR_HPP_
#define BOTANBOT_UTILITIES__GPS_WAYPOINT_COLLECTOR_HPP_

#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

namespace botanbot_utilities
{

class GPSWaypointCollector : public rclcpp::Node
{
private:
  void timerCallback();
  void navSatFixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr navsat_fix_subscriber_;
  sensor_msgs::msg::NavSatFix reusable_navsat_msg_;
  // to ensure safety when accessing global
  std::mutex global_mutex_;
  int gps_msg_index_;
  bool is_first_msg_recieved_;

public:
  GPSWaypointCollector(/* args */);
  ~GPSWaypointCollector();
};

} // namespace botanbot_utilities

#endif  // BOTANBOT_UTILITIES__GPS_WAYPOINT_COLLECTOR_HPP_
