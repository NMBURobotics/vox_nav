// Copyright (c) 2022 Norwegian University of Life Sciences, Fetullah Atas
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

#ifndef VOX_NAV_WAYPOINT_NAV__NAVIGATE_THROUGH_GPS_POSES_CLIENT_HPP_
#define VOX_NAV_WAYPOINT_NAV__NAVIGATE_THROUGH_GPS_POSES_CLIENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <vox_nav_msgs/action/navigate_through_gps_poses.hpp>
#include <vox_nav_utilities/tf_helpers.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <vector>
#include <string>
#include <memory>
#include <mutex>

namespace vox_nav_waypoint_nav_clients
{
  enum class ActionStatus
  {
    UNKNOWN = 0,
    PROCESSING = 1,
    FAILED = 2,
    SUCCEEDED = 3
  };

/**
 * @brief A ros node that drives robot through gievn way points from YAML file
 *
 */
  class NavigateThroughGPSPosesClient : public rclcpp::Node
  {
  public:
    using ClientT = vox_nav_msgs::action::NavigateThroughGPSPoses;
    using NavigateThroughGPSPosesGoalHandle = rclcpp_action::ClientGoalHandle<ClientT>;

    /**
     * @brief Construct a new WayPoint Folllower Demo object
     *
     */
    NavigateThroughGPSPosesClient();

    /**
     * @brief Destroy the Way Point Folllower Demo object
     *
     */
    ~NavigateThroughGPSPosesClient();

    /**
     * @brief send robot through each of the pose in poses vector
     *
     * @param poses
     */
    void startGPSWaypointFollowing();

    /**
     * @brief
     *
     * @return true
     * @return false
     */
    bool is_goal_done() const;

    /**
   * @brief given a parameter name on the yaml file, loads this parameters as std::vector<sensor_msgs::msg::NavSatFix>
   *  Note that this parameter needs to be an array of doubles
   *
   * @return std::vector<sensor_msgs::msg::NavSatFix>
   */
    std::vector<sensor_msgs::msg::NavSatFix> loadGPSWaypointsFromYAML();

    void goalResponseCallback(NavigateThroughGPSPosesGoalHandle::SharedPtr goal_handle);

    void feedbackCallback(
      NavigateThroughGPSPosesGoalHandle::SharedPtr,
      const std::shared_ptr<const ClientT::Feedback> feedback);

    void resultCallback(const NavigateThroughGPSPosesGoalHandle::WrappedResult & result);

  protected:
    bool goal_done_;
    rclcpp::TimerBase::SharedPtr timer_;
    // client to connect navigate through poses action server
    rclcpp_action::Client<ClientT>::SharedPtr
      navigate_through_gps_poses_action_client_;

    // goal handler to query state of goal
    ClientT::Goal navigate_through_gps_poses_goal_;

    NavigateThroughGPSPosesGoalHandle::SharedPtr navigate_through_gps_poses_goalhandle_;

    std::vector<sensor_msgs::msg::NavSatFix> gps_poses_from_yaml_;

  };

}  // namespace vox_nav_waypoint_nav_clients

#endif  // VOX_NAV_WAYPOINT_NAV__NAVIGATE_THROUGH_GPS_POSES_CLIENT_HPP_
