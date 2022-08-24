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

#ifndef VOX_NAV_PLANNING__PLUGINS__POLYTUNNEL_PLANNER_HPP_
#define VOX_NAV_PLANNING__PLUGINS__POLYTUNNEL_PLANNER_HPP_

#include "vox_nav_planning/planner_core.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "vox_nav_utilities/elevation_state_space.hpp"
#include "vox_nav_utilities/pcl_helpers.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/transform_datatypes.h"
#include "tf2_ros/create_timer_interface.h"

#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/tracking.hpp"

#include <vector>
#include <string>
#include <memory>
#include <algorithm>

namespace vox_nav_planning
{

  class PolyTunnelPlanner : public vox_nav_planning::PlannerCore
  {

  public:
/**
 * @brief Construct a new PolyTunnelPlanner object
 *
 */
    PolyTunnelPlanner();

/**
 * @brief Destroy the PolyTunnelPlanner object
 *
 */
    ~PolyTunnelPlanner();

    /**
     * @brief
     *
     */
    void initialize(
      rclcpp::Node * parent,
      const std::string & plugin_name) override;

    /**
     * @brief Method create the plan from a starting and ending goal.
     *
     * @param start The starting pose of the robot
     * @param goal  The goal pose of the robot
     * @return std::vector<geometry_msgs::msg::PoseStamped>   The sequence of poses to get from start to goal, if any
     */
    std::vector<geometry_msgs::msg::PoseStamped> createPlan(
      const geometry_msgs::msg::PoseStamped & start,
      const geometry_msgs::msg::PoseStamped & goal) override;

    /**
     * @brief Get the Overlayed Startand Goal object
     *
     * @return std::vector<geometry_msgs::msg::PoseStamped>
     */
    std::vector<geometry_msgs::msg::PoseStamped> getOverlayedStartandGoal() override;

    /**
     * @brief
     *
     * @param msg
     */
    void polytunnelCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    /**
     * @brief Not implemented
     *
     */
    void setupMap() override
    {
    }

    /**
     * @brief Not implemented
     *
     * @param state
     * @return true
     * @return false
     */
    bool isStateValid(const ompl::base::State * state) override
    {
    }

  protected:
    rclcpp::Logger logger_{rclcpp::get_logger("elevation_control_planner")};
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr polytunnel_cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr polytunnel_cloud_pub_;
    sensor_msgs::msg::PointCloud2 polytunnel_cloud_;

    // tf buffer to get transfroms
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    // tf listner for tf transforms
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;


    ompl::base::StateSpacePtr state_space_;
    std::string selected_se2_space_name_;
    // curve radius for reeds and dubins only
    double rho_;
    ompl::base::SpaceInformationPtr state_space_information_;
  };
}  // namespace vox_nav_planning

#endif  // VOX_NAV_PLANNING__PLUGINS__POLYTUNNEL_PLANNER_HPP_
