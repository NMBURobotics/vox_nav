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

#include <string>
#include <vector>
#include <memory>
#include "botanbot_grid_map/botanbot_grid_map.hpp"

namespace botanbot_grid_map
{
BotanbotGridMap::BotanbotGridMap()
: Node("gps_waypoint_follower_rclcpp_node"), goal_done_(false)
{

  this->timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&BotanbotGridMap::startWaypointFollowing, this));

}

BotanbotGridMap::~BotanbotGridMap()
{
  RCLCPP_INFO(
    this->get_logger(),
    "Destroyed an Instance of BotanbotGridMap");
}

void BotanbotGridMap::startWaypointFollowing()
{
  using namespace std::placeholders;
  this->timer_->cancel();

}

}  // namespace botanbot_grid_map

/**
 * @brief
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char const * argv[])
{
  rclcpp::init(argc, argv);
  auto gps_waypoint_follower_client_node = std::make_shared
    <botanbot_grid_map::BotanbotGridMap>();
  while (!rclcpp::ok()) {
    rclcpp::spin_some(gps_waypoint_follower_client_node);
  }
  rclcpp::shutdown();
  return 0;
}
