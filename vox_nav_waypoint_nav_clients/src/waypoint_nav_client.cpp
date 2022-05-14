// Copyright (c) 2022 Fetullah Atas, Norwegian University of Life Sciences
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

#include "vox_nav_waypoint_nav_clients/waypoint_nav_client.hpp"

#include <string>
#include <vector>
#include <memory>
#include <algorithm>

namespace waypoint_nav_client
{

}  // namespace waypoint_nav_client

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
  auto map_manager_node = std::make_shared
    <waypoint_nav_client::MapManager>();
  rclcpp::spin(map_manager_node);
  rclcpp::shutdown();
  return 0;
}
