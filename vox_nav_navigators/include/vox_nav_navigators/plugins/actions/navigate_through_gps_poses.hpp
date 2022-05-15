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

#ifndef VOX_NAV_NAVIGATORS_PLUGINS__ACTIONS__NAVIGATE_THROUGH_GPS_POSES_HPP_
#define VOX_NAV_NAVIGATORS_PLUGINS__ACTIONS__NAVIGATE_THROUGH_GPS_POSES_HPP_

#include <string>

#include "vox_nav_msgs/action/navigate_through_gps_poses.hpp"
#include "nav_msgs/msg/path.h"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "vox_nav_navigators/plugins/actions/base_action_client_node.hpp"

namespace vox_nav_navigators
{
  using NavigateThroughGPSPoses = vox_nav_msgs::action::NavigateThroughGPSPoses;
  class NavigateThroughGPSPosesNode : public BaseActionClientNode<NavigateThroughGPSPoses>
  {
  public:
    NavigateThroughGPSPosesNode(
      const std::string & xml_tag_name,
      const std::string & action_name,
      const BT::NodeConfiguration & conf)
    : BaseActionClientNode<NavigateThroughGPSPoses>(xml_tag_name, action_name, conf)
    {
    }

    static BT::PortsList providedPorts()
    {
      return providedBasicPorts(
      {
        BT::InputPort<std::vector<sensor_msgs::msg::NavSatFix>>("gps_poses", "gps_poses"),
      });
    }

    void on_tick()
    {
      // Use the position and orientation fields from the XML attributes to initialize the goal
      std::vector<sensor_msgs::msg::NavSatFix> gps_poses;
      if (!getInput("gps_poses", gps_poses)) {
        RCLCPP_ERROR(
          node_->get_logger(),
          "NavigateThroughPosesNode: pose not provided");
        return;
      }
      goal_.gps_poses = gps_poses;
    }
  };

}  // namespace vox_nav_navigators

#endif  // VOX_NAV_NAVIGATORS_PLUGINS__ACTIONS__NAVIGATE_THROUGH_GPS_POSES_HPP_
