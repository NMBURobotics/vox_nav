// Copyright (c) 2021 Fetullah Atas, Norwegian University of Life Sciences
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
#include <vox_nav_control/mpc_controller/mpc_controller_core.hpp>
#include <vox_nav_utilities/tf_helpers.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/ScopedState.h>

#include <string>
#include <memory>
#include <vector>


#ifndef VOX_NAV_CONTROL__MPC_CONTROLLER__MPC_CONTROLLER_ROS_HPP_
#define VOX_NAV_CONTROL__MPC_CONTROLLER__MPC_CONTROLLER_ROS_HPP_

namespace vox_nav_control
{
  namespace mpc_controller
  {
/**
 * @brief A ROS wrapper around the Core MPC controller class.
 *        This class interfaces MPC controller with ControllerCore which is base class for plugins.
 *        Derivatives of ControllerCore class are used to expose Controller server for use
 *        The controller server is an ROS2 action named follow_path
 *
 */
    class MPCControllerROS : public vox_nav_control::ControllerCore
    {
    public:
      /**
       * @brief Construct a new MPCControllerROS object
       *
       */
      MPCControllerROS();

      /**
       * @brief Destroy the MPCControllerROS object
       *
       */
      ~MPCControllerROS();

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
       * @brief get the index of nearest trajectory state to current robot pose
       *
       * @param reference_traj
       * @param curr_robot_pose
       * @return int
       */
      int nearestStateIndex(
        nav_msgs::msg::Path reference_traj,
        geometry_msgs::msg::PoseStamped curr_robot_pose)
      override;

      /**
       * @brief Get the Interpolated Reference States, This is determined by the look ahead distance set in the parameters.
       *        As well as number of time Horizons(N)
       *
       * @param curr_robot_pose
       * @return std::vector<MPCControllerCore::States>
       */
      std::vector<MPCControllerCore::States> getLocalInterpolatedReferenceStates(
        geometry_msgs::msg::PoseStamped curr_robot_pose);

      /**
       * @brief return closest reference trajectory to be feed into conrol sceheme
       *        The number of returned states will be determined according to time horizon(N)
       *
       * @param interpolated_ref_traj
       */
      void publishLocalInterpolatedRefernceStates(
        std::vector<MPCControllerCore::States> interpolated_ref_traj);

    private:
      // Given refernce traj to follow, this is set got from planner
      nav_msgs::msg::Path reference_traj_;
      // computed velocities as result of MPC
      geometry_msgs::msg::Twist computed_velocity_;
      // MPC core controller object
      std::shared_ptr<MPCControllerCore> mpc_controller_;
      // parameters struct used for MPC controller
      MPCControllerCore::Parameters mpc_parameters_;
      // Keep a copy of previously applied control inputs, this is neded
      // by MPC algorithm
      MPCControllerCore::ControlInput previous_control_;
      // Publish local trajecory currently being fed to controller
      rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        interpolated_local_reference_traj_publisher_;
      // while following global plan whats the max look ahead distance ?, thats global_plan_look_ahead_distance_
      double global_plan_look_ahead_distance_;
    };

  } // namespace mpc_controller
}  // namespace vox_nav_control

#endif  // VOX_NAV_CONTROL__MPC_CONTROLLER__MPC_CONTROLLER_ROS_HPP_
