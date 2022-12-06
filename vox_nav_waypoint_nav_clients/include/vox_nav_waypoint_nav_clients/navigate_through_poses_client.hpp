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

#ifndef VOX_NAV_WAYPOINT_NAV__NAVIGATE_THROUGH_POSES_CLIENT_HPP_
#define VOX_NAV_WAYPOINT_NAV__NAVIGATE_THROUGH_POSES_CLIENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <vox_nav_msgs/action/navigate_through_poses.hpp>
#include <vox_nav_utilities/tf_helpers.hpp>
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
  class NavigateThroughPosesClient : public rclcpp::Node
  {
  public:
    using ClientT = vox_nav_msgs::action::NavigateThroughPoses;
    using NavigateThroughPosesGoalHandle = rclcpp_action::ClientGoalHandle<ClientT>;

    /**
     * @brief Construct a new WayPoint Folllower Demo object
     *
     */
    NavigateThroughPosesClient();

    /**
     * @brief Destroy the Way Point Folllower Demo object
     *
     */
    ~NavigateThroughPosesClient();

    /**
     * @brief send robot through each of the pose in poses vector
     *
     * @param poses
     */
    void startWaypointFollowing();

    /**
     * @brief
     *
     * @return true
     * @return false
     */
    bool is_goal_done() const;

    /**
   * @brief given a parameter name on the yaml file, loads this parameter as geometry_msgs_msgs::msg::Pose
   *  Note that this parameter needs to be an array of doubles
   *
   * @return geometry_msgs_msgs::msg::Pose
   */
    std::vector<geometry_msgs::msg::PoseStamped> loadWaypointsFromYAML();

    void goalResponseCallback(NavigateThroughPosesGoalHandle::SharedPtr goal_handle);

    void feedbackCallback(
      NavigateThroughPosesGoalHandle::SharedPtr,
      const std::shared_ptr<const ClientT::Feedback> feedback);

    void resultCallback(const NavigateThroughPosesGoalHandle::WrappedResult & result);

  protected:
    bool goal_done_;
    rclcpp::TimerBase::SharedPtr timer_;
    // client to connect navigate through poses action server
    rclcpp_action::Client<ClientT>::SharedPtr
      navigate_through_poses_action_client_;

    // goal handler to query state of goal
    ClientT::Goal navigate_through_poses_goal_;

    NavigateThroughPosesGoalHandle::SharedPtr navigate_through_poses_goalhandle_;

    std::vector<geometry_msgs::msg::PoseStamped> poses_from_yaml_;

  };

}  // namespace vox_nav_waypoint_nav_clients

#endif  // VOX_NAV_WAYPOINT_NAV__NAVIGATE_THROUGH_POSES_CLIENT_HPP_
