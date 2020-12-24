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

#ifndef BOTANBOT_GRID_MAP__GPS_WAYPOINT_FOLLOWER_CLIENT_HPP_
#define BOTANBOT_GRID_MAP__GPS_WAYPOINT_FOLLOWER_CLIENT_HPP_

#include <vector>
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/point32.hpp"

/**
 * @brief namespace for way point following, points are from a yaml file
 *
 */
namespace botanbot_grid_map
{


/**
 * @brief A ros node that drives robot through gievn way points from YAML file
 *
 */
class BotanbotGridMap : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new WayPoint Folllower Demo object
   *
   */
  BotanbotGridMap();

  /**
   * @brief Destroy the Way Point Folllower Demo object
   *
   */
  ~BotanbotGridMap();

  /**
   * @brief send robot through each of the pose in poses vector
   *
   * @param poses
   */
  void startWaypointFollowing();

protected:
  bool goal_done_;
  rclcpp::TimerBase::SharedPtr timer_;

};
}  // namespace botanbot_grid_map

#endif  // BOTANBOT_GRID_MAP__GPS_WAYPOINT_FOLLOWER_CLIENT_HPP_
