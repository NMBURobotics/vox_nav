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

#include <nav_msgs/msg/path.hpp>
#include <vox_nav_control/lyapunov_controller/lyapunov_controller_ros.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <memory>
#include <vector>
#include <string>

namespace vox_nav_control
{
  namespace lyapunov_controller
  {
    LyapunovControllerROS::LyapunovControllerROS()
    {
    }

    LyapunovControllerROS::~LyapunovControllerROS()
    {
    }
    void LyapunovControllerROS::initialize(
      rclcpp::Node * parent,
      const std::string & plugin_name)
    {
      parent_ = parent;

      parent->declare_parameter(plugin_name + ".V_MIN", -2.0);
      parent->declare_parameter(plugin_name + ".V_MAX", 2.0);
      parent->declare_parameter(plugin_name + ".DF_MIN", -0.5);
      parent->declare_parameter(plugin_name + ".DF_MAX", 0.5);
      parent->declare_parameter(plugin_name + ".k1", -0.5);
      parent->declare_parameter(plugin_name + ".k2", 0.5);
      parent->get_parameter(plugin_name + ".V_MIN", mpc_parameters_.V_MIN);
      parent->get_parameter(plugin_name + ".V_MAX", mpc_parameters_.V_MAX);
      parent->get_parameter(plugin_name + ".DF_MIN", mpc_parameters_.DF_MIN);
      parent->get_parameter(plugin_name + ".DF_MAX", mpc_parameters_.DF_MAX);
      parent->get_parameter(plugin_name + ".k1", k1_);
      parent->get_parameter(plugin_name + ".k2", k2_);

      interpolated_local_reference_traj_publisher_ =
        parent->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/vox_nav/controller/plan",
        1);

      obstacle_tracks_sub_ = parent->create_subscription<vox_nav_msgs::msg::ObjectArray>(
        "/vox_nav/tracking/objects", rclcpp::SystemDefaultsQoS(),
        std::bind(&LyapunovControllerROS::obstacleTracksCallback, this, std::placeholders::_1));

    }

    geometry_msgs::msg::Twist LyapunovControllerROS::computeVelocityCommands(
      geometry_msgs::msg::PoseStamped curr_robot_pose)
    {

      double robot_roll, robot_pitch, robot_yaw;
      vox_nav_utilities::getRPYfromMsgQuaternion(
        curr_robot_pose.pose.orientation, robot_roll, robot_pitch, robot_yaw);


      auto nearest_state_index = common::nearestStateIndex(reference_traj_, curr_robot_pose);
      nearest_state_index += 1;

      if (nearest_state_index == reference_traj_.poses.size()) {
        nearest_state_index = reference_traj_.poses.size() - 1;
      }

      auto curr_goal = reference_traj_.poses[nearest_state_index].pose;

      double goal_roll, goal_pitch, goal_yaw;
      vox_nav_utilities::getRPYfromMsgQuaternion(
        curr_goal.orientation, goal_roll, goal_pitch, goal_yaw);

      auto target_x = curr_goal.position.x;
      auto target_y = curr_goal.position.y;

      auto z1 = curr_robot_pose.pose.position.x;
      auto z2 = curr_robot_pose.pose.position.y;
      auto z3 = robot_yaw;

      // state in polar coordinates
      auto e = std::sqrt(std::pow((z1 - target_x), 2) + std::pow((z2 - target_y), 2));
      auto goal_heading = std::atan2((z2 - target_y), (z1 - target_x));

      auto k1 = k1_;
      auto k2 = k2_;

      if ((z1 - target_x) < 0) {
        k1 = -k1;
        if (z2 - target_y < 0) {
          goal_heading += M_PI;
        } else {
          goal_heading -= M_PI;
        }
      }

      auto a = goal_heading - z3;
      auto tht = goal_heading;

      auto u1 = k1 * (e * std::cos(a));
      auto u2 = k2 * (a + (std::cos(a) * std::sin(a) / a)) * (a + tht);

      computed_velocity_.linear.x = std::clamp<double>(
        u1, mpc_parameters_.V_MIN,
        mpc_parameters_.V_MAX);
      computed_velocity_.angular.z = std::clamp<double>(
        u2, mpc_parameters_.DF_MIN,
        mpc_parameters_.DF_MAX);

      std_msgs::msg::ColorRGBA red_color, blue_color;
      red_color.r = 1.0;
      red_color.a = 1.0;

      std::vector<vox_nav_control::common::States> local_interpolated_reference_states;
      vox_nav_control::common::States goal_state;
      goal_state.x = target_x;
      goal_state.y = target_y;
      goal_state.psi = goal_yaw;
      local_interpolated_reference_states.push_back(goal_state);

      vox_nav_control::common::publishTrajStates(
        local_interpolated_reference_states, red_color, "ref_traj",
        interpolated_local_reference_traj_publisher_);

      return computed_velocity_;
    }

    geometry_msgs::msg::Twist LyapunovControllerROS::computeHeadingCorrectionCommands(
      geometry_msgs::msg::PoseStamped curr_robot_pose)
    {

      // we dont really need roll and pitch here
      double nan, robot_psi;
      vox_nav_utilities::getRPYfromMsgQuaternion(
        curr_robot_pose.pose.orientation, nan, nan, robot_psi);

      vox_nav_control::common::States curr_states;
      curr_states.x = curr_robot_pose.pose.position.x;
      curr_states.y = curr_robot_pose.pose.position.y;
      curr_states.psi = robot_psi;
      curr_states.v = 0.0;

      auto u1 = 0;
      auto u2 = 0;

      computed_velocity_.linear.x = std::clamp<double>(
        u1, mpc_parameters_.V_MIN,
        mpc_parameters_.V_MAX);
      computed_velocity_.angular.z = std::clamp<double>(
        u2, mpc_parameters_.DF_MIN,
        mpc_parameters_.DF_MAX);

      return computed_velocity_;
    }

    void LyapunovControllerROS::setPlan(const nav_msgs::msg::Path & path)
    {
      reference_traj_ = path;
    }

    void LyapunovControllerROS::obstacleTracksCallback(
      const vox_nav_msgs::msg::ObjectArray::SharedPtr msg)
    {
      std::lock_guard<std::mutex> guard(obstacle_tracks_mutex_);
      obstacle_tracks_ = *msg;
    }

    std::vector<vox_nav_control::common::Ellipsoid> LyapunovControllerROS::trackMsg2Ellipsoids(
      const vox_nav_msgs::msg::ObjectArray & tracks,
      const geometry_msgs::msg::PoseStamped & curr_robot_pose)
    {

      double robot_roll, robot_pitch, robot_psi;
      vox_nav_utilities::getRPYfromMsgQuaternion(
        curr_robot_pose.pose.orientation, robot_roll, robot_pitch, robot_psi);

      std::vector<vox_nav_control::common::Ellipsoid> ellipsoids;
      for (auto && i : tracks.objects) {

        // We use dynamic weigthig matrix,
        // If the given goal is behind robots current heading, adjust parameters so that we take best maneuver
        Eigen::Vector3f curr_robot_vec(
          curr_robot_pose.pose.position.x,
          curr_robot_pose.pose.position.y,
          curr_robot_pose.pose.position.z);

        Eigen::Vector3f obstacle_center_vec(
          i.world_pose.point.x,
          i.world_pose.point.y,
          i.world_pose.point.z);

        Eigen::Vector3f obstacle_head_vec(
          i.world_pose.point.x + i.length / 2.0,
          i.world_pose.point.y,
          i.world_pose.point.z);

        float heading_to_robot_angle =
          std::acos(
          vox_nav_control::common::dot(
            obstacle_center_vec - obstacle_head_vec,
            obstacle_center_vec - curr_robot_vec) /
          (vox_nav_control::common::mag(obstacle_center_vec - obstacle_head_vec) *
          vox_nav_control::common::mag(obstacle_center_vec - curr_robot_vec)));

        /*
        https://math.stackexchange.com/questions/426150/what-is-the-general-equation-of-the-ellipse-that-is-not-in-the-origin-and-rotate

        std::pow(
          (x - center.x()) * std::cos(i.heading) + (y - center.y()) * std::sin(i.heading),
          2) / std::pow(a, 2) +
        std::pow(
          (x - center.x()) * std::sin(i.heading) - (y - center.y()) * std::cos(i.heading),
          2) / std::pow(b, 2) = 1;
        */
        Eigen::Vector2f center(i.world_pose.point.x, i.world_pose.point.y);
        double a = i.length;
        double b = i.width;
        vox_nav_control::common::Ellipsoid e;
        e.heading = i.heading;
        e.is_dynamic = i.is_dynamic;
        e.center = center;
        e.axes = Eigen::Vector2f(a, b);
        e.heading_to_robot_angle = heading_to_robot_angle - M_PI_2;
        ellipsoids.push_back(e);
      }

      return ellipsoids;

    }

  } // namespace lyapunov_controller
  PLUGINLIB_EXPORT_CLASS(
    lyapunov_controller::LyapunovControllerROS,
    vox_nav_control::ControllerCore)
}  // namespace vox_nav_control
