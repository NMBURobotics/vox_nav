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

#include <rclcpp/rclcpp.hpp>
#include <vox_nav_control/controller_core.hpp>
#include <vox_nav_control/common.hpp>
#include <vox_nav_utilities/tf_helpers.hpp>
#include <vox_nav_msgs/msg/object_array.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <string>
#include <memory>
#include <vector>

#ifndef VOX_NAV_CONTROL__LYAPUNOV_CONTROLLER__LYAPUNOV_CONTROLLER_ROS_HPP_
#define VOX_NAV_CONTROL__LYAPUNOV_CONTROLLER__LYAPUNOV_CONTROLLER_ROS_HPP_

namespace vox_nav_control
{

  namespace lyapunov_controller
  {
/**
 * @brief A ROS wrapper around the Core MPC controller class.
 *        This class interfaces MPC controller with ControllerCore which is base class for plugins.
 *        Derivatives of ControllerCore class are used to expose Controller server for use
 *        The controller server is an ROS2 action named follow_path
 *
 */
    class LyapunovControllerROS : public vox_nav_control::ControllerCore
    {
    public:
      /**
       * @brief Construct a new LyapunovControllerROS object
       *
       */
      LyapunovControllerROS();

      /**
       * @brief Destroy the LyapunovControllerROS object
       *
       */
      ~LyapunovControllerROS();

      /**
       * @brief gets all parameters required by MPC controller. Sets up an ROS2 stuff, publisher etc.
       *
       * @param parent
       * @param plugin_name
       */
      void initialize(
        rclcpp::Node * parent,
        const std::string & plugin_name) override;

      /**
       * @brief Set the Plan from controller server
       *
       * @param path
       */
      void setPlan(const nav_msgs::msg::Path & path) override;

      /**
       * @brief Compute required velocity commands to drive the robot along the reference_trajectory_
       *
       * @param curr_robot_pose
       * @return geometry_msgs::msg::Twist
       */
      geometry_msgs::msg::Twist computeVelocityCommands(
        geometry_msgs::msg::PoseStamped curr_robot_pose)
      override;

      /**
       * @brief Compute required velocity commands recorrect heading. This function is used when robot already satisfies
       * goal_tolerance_distance
       *
       * @param curr_robot_pose
       * @return geometry_msgs::msg::Twist
       */
      geometry_msgs::msg::Twist computeHeadingCorrectionCommands(
        geometry_msgs::msg::PoseStamped curr_robot_pose)
      override;

      /**
       * @brief
       *
       * @param msg
       */
      void obstacleTracksCallback(const vox_nav_msgs::msg::ObjectArray::SharedPtr msg);

      /**
       * @brief Convert ROS msg type of ObjectsArray to vector of Ellipsoid to feed to MPC controller
       *
       * @param tracks
       * @return std::vector<Ellipsoid>
       */
      std::vector<vox_nav_control::common::Ellipsoid> trackMsg2Ellipsoids(
        const vox_nav_msgs::msg::ObjectArray & tracks,
        const geometry_msgs::msg::PoseStamped & curr_robot_pose);

    private:
      // Given refernce traj to follow, this is set got from planner
      nav_msgs::msg::Path reference_traj_;
      // computed velocities as result of MPC
      geometry_msgs::msg::Twist computed_velocity_;
      // MPC core controller object
      // parameters struct used for MPC controller
      vox_nav_control::common::Parameters mpc_parameters_;
      // Publish local goal controller
      rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        curr_goal_publisher_;

      rclcpp::Node * parent_;
      rclcpp::Subscription<vox_nav_msgs::msg::ObjectArray>::SharedPtr obstacle_tracks_sub_;
      vox_nav_msgs::msg::ObjectArray obstacle_tracks_;
      std::mutex obstacle_tracks_mutex_;

      double k1_;
      double k2_;
      int lookahead_n_waypoints_;
    };

  } // namespace lyapunov_controller
}  // namespace vox_nav_control

#endif  // VOX_NAV_CONTROL__LYAPUNOV_CONTROLLER__LYAPUNOV_CONTROLLER_ROS_HPP_
