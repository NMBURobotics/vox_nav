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

#ifndef VOX_NAV_CONTROL__COMMON_HPP_
#define VOX_NAV_CONTROL__COMMON_HPP_

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <Eigen/Eigen>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/ScopedState.h>

#include <vox_nav_utilities/tf_helpers.hpp>
#include <vox_nav_msgs/msg/object_array.hpp>

namespace vox_nav_control
{
  namespace common
  {

    enum STATE_ENUM
    {
      kX = 0,
      kY = 1,
      kPsi = 2,
      kV = 3,
      kObs = 4,
    };

    enum INPUT_ENUM
    {
      kacc = 0,
      kdf = 1,
    };

    /**
       * @brief We model obstacles with ellipsoids
       * They are then used in opti stack
       *
       */
    struct Ellipsoid
    {
      Eigen::Vector2f center;
      Eigen::Vector2f axes;
      bool is_dynamic;
      double heading;
      Ellipsoid()
      : center(1000.0, 1000.0),
        axes(0.1, 0.1),
        is_dynamic(false),
        heading(0.0) {}
    };

    /**
     * @brief a struct to keep states organized
     *
     */
    struct States
    {
      double x;   // X position
      double y;   // Y position
      double psi;   // heading angle
      double v;   // linear velocity
      States()
      : x(0.0),
        y(0.0),
        psi(0.0),
        v(0.0) {}
    };

    /**
     * @brief a struct to keep control inputs organized
     *
     */
    struct ControlInput
    {
      double acc;   // accelration command
      double df;   // steering angle command
      ControlInput()
      : acc(0.0),
        df(0.0) {}
    };

    /**
   * @brief all parameters used by MPC class,
   * user needs to create and reconfigure this
   *
   */
    struct Parameters
    {
      // timesteps in MPC Horizon
      int N;
      // discretization time between timesteps(s)
      double DT;
      // distance from CoG to front axle(m)
      double L_F;
      // distance from CoG to rear axle(m)
      double L_R;
      // min / max velocity constraint(m / s)
      double V_MIN;
      double V_MAX;
      // min / max acceleration constraint(m / s ^ 2)
      double A_MIN;
      double A_MAX;
      // min / max front steer angle constraint(rad)
      double DF_MIN;
      double DF_MAX;
      // min / max jerk constraint(m / s ^ 3)
      double A_DOT_MIN;
      double A_DOT_MAX;
      // min / max front steer angle rate constraint(rad / s)
      double DF_DOT_MIN;
      double DF_DOT_MAX;
      // weights on x, y, psi, and v.
      std::vector<double> Q;
      // weights on jerk and skew rate(steering angle derivative)
      std::vector<double> R;
      // enable/disable debug messages
      bool debug_mode;
      // set this true only if user figured the configration
      bool params_configured;

      int max_obstacles;
      double robot_radius;
      double obstacle_cost;

      // Assign meaningful default values to this parameters
      Parameters()
      : N(10),
        DT(0.1),
        L_F(0.65),
        L_R(0.65),
        V_MIN(-10.0),
        V_MAX(10.0),
        A_MIN(-1.0),
        A_MAX(1.0),
        DF_MIN(-0.5),
        DF_MAX(0.5),
        A_DOT_MIN(-1.0),
        A_DOT_MAX(1.0),
        DF_DOT_MIN(-0.5),
        DF_DOT_MAX(0.5),
        Q({100.0, 100.0, 10.0, 0.1}),
        R({10.0, 10.0}),
        debug_mode(true),
        params_configured(false),
        max_obstacles(1),
        robot_radius(0.5),
        obstacle_cost(1.0) {}
    };

    static int nearestStateIndex(
      const nav_msgs::msg::Path & reference_traj,
      const geometry_msgs::msg::PoseStamped & curr_robot_pose)
    {
      int closest_state_index = -1;
      int closest_state_distance = 10000.0;
      for (int i = 0; i < reference_traj.poses.size(); i++) {

        double curr_distance =
          vox_nav_utilities::getEuclidianDistBetweenPoses(reference_traj.poses[i], curr_robot_pose);

        if (curr_distance < closest_state_distance) {
          closest_state_distance = curr_distance;
          closest_state_index = i;
        }
      }
      return closest_state_index;
    }

    static std::vector<States> getLocalInterpolatedReferenceStates(
      const geometry_msgs::msg::PoseStamped & curr_robot_pose,
      const Parameters & mpc_parameters,
      const nav_msgs::msg::Path & reference_traj,
      const double global_plan_look_ahead_distance,
      std::shared_ptr<ompl::base::SpaceInformation> si)
    {
      double robot_roll, robot_pitch, robot_yaw;
      geometry_msgs::msg::PoseStamped front_axle_pose;
      vox_nav_utilities::getRPYfromMsgQuaternion(
        curr_robot_pose.pose.orientation, robot_roll, robot_pitch, robot_yaw);

      front_axle_pose.pose.position.x =
        curr_robot_pose.pose.position.x + mpc_parameters.L_F * std::cos(robot_yaw);
      front_axle_pose.pose.position.y =
        curr_robot_pose.pose.position.y + mpc_parameters.L_F * std::sin(robot_yaw);
      front_axle_pose.pose.position.z = 0;

      // Now lets find nearest trajectory point to robot base
      int nearsest_traj_state_index = nearestStateIndex(reference_traj, curr_robot_pose);

      // Auto calculate interpolation steps
      double path_euclidian_length = 0.0;
      for (size_t i = 1; i < reference_traj.poses.size(); i++) {
        path_euclidian_length += vox_nav_utilities::getEuclidianDistBetweenPoses(
          reference_traj.poses[i], reference_traj.poses[i - 1]);
      }

      double interpolation_step_size = path_euclidian_length / reference_traj.poses.size();
      int states_to_see_horizon = global_plan_look_ahead_distance / interpolation_step_size;
      int local_goal_state_index = nearsest_traj_state_index + states_to_see_horizon;
      if (local_goal_state_index >= reference_traj.poses.size() - 1) {
        local_goal_state_index = reference_traj.poses.size() - 1;
      }
      // Define a state space, we basically need this only because we want to use OMPL's
      // geometric path, And then we can interpolate this path
      ompl::geometric::PathGeometric path(si);

      ompl::base::ScopedState<>
      closest_ref_traj_state(si->getStateSpace()),
      ompl_local_goal_state(si->getStateSpace());

      // Feed initial state, which is closest ref trajectory state
      double void_var, yaw;

      if (local_goal_state_index - nearsest_traj_state_index < mpc_parameters.N) {
        // Feed Intermediate state , which is nearest state in ref traj
        vox_nav_utilities::getRPYfromMsgQuaternion(
          curr_robot_pose.pose.orientation, void_var, void_var, yaw);
        closest_ref_traj_state[0] = curr_robot_pose.pose.position.x;
        closest_ref_traj_state[1] = curr_robot_pose.pose.position.y;
        closest_ref_traj_state[2] = yaw;
        path.append(static_cast<ompl::base::State *>(closest_ref_traj_state.get()));
      } else {
        // Feed Intermediate state , which is nearest state in ref traj
        vox_nav_utilities::getRPYfromMsgQuaternion(
          reference_traj.poses[nearsest_traj_state_index].pose.orientation, void_var, void_var,
          yaw);
        closest_ref_traj_state[0] =
          reference_traj.poses[nearsest_traj_state_index].pose.position.x;
        closest_ref_traj_state[1] =
          reference_traj.poses[nearsest_traj_state_index].pose.position.y;
        closest_ref_traj_state[2] = yaw;
        path.append(static_cast<ompl::base::State *>(closest_ref_traj_state.get()));
      }

      // Feed the final state, which the local goal for the current control effort.
      // This is basically the state in the ref trajectory, which is closest to global_plan_look_ahead_distance_
      vox_nav_utilities::getRPYfromMsgQuaternion(
        reference_traj.poses[local_goal_state_index].pose.orientation, void_var, void_var, yaw);
      ompl_local_goal_state[0] = reference_traj.poses[local_goal_state_index].pose.position.x;
      ompl_local_goal_state[1] = reference_traj.poses[local_goal_state_index].pose.position.y;
      ompl_local_goal_state[2] = yaw;
      path.append(static_cast<ompl::base::State *>(ompl_local_goal_state.get()));

      // The local ref traj now contains only 3 states, we will interpolate this states with OMPL
      // The count of states after interpolation must be same as horizon defined for the control problem , hence
      // it should be mpc_parameters_.N
      path.interpolate(mpc_parameters.N);

      // Now the local ref traj is interpolated from current robot state up to state at global look ahead distance
      // Lets fill the native MPC type ref states and return to caller
      std::vector<States> interpolated_reference_states;
      for (std::size_t path_idx = 0; path_idx < path.getStateCount(); path_idx++) {
        // cast the abstract state type to the type we expect
        const ompl::base::ReedsSheppStateSpace::StateType * interpolated_state =
          path.getState(path_idx)->as<ompl::base::ReedsSheppStateSpace::StateType>();
        States curr_interpolated_state;
        curr_interpolated_state.v = mpc_parameters.V_MAX;
        curr_interpolated_state.x = interpolated_state->getX();
        curr_interpolated_state.y = interpolated_state->getY();
        curr_interpolated_state.psi = interpolated_state->getYaw();
        interpolated_reference_states.push_back(curr_interpolated_state);
      }
      return interpolated_reference_states;
    }

    static void publishTrajStates(
      const std::vector<States> & interpolated_reference_states,
      const std_msgs::msg::ColorRGBA & color,
      const std::string & ns,
      const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher)
    {
      visualization_msgs::msg::MarkerArray marker_array;
      for (int i = 0; i < interpolated_reference_states.size(); i++) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = rclcpp::Clock().now();
        marker.ns = ns;
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.lifetime = rclcpp::Duration::from_seconds(0);
        marker.pose.position.x = interpolated_reference_states[i].x;
        marker.pose.position.y = interpolated_reference_states[i].y;
        marker.pose.position.z = 1.3;
        marker.pose.orientation = vox_nav_utilities::getMsgQuaternionfromRPY(
          0, 0, interpolated_reference_states[i].psi);
        marker.scale.x = 0.25;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color = color;

        marker_array.markers.push_back(marker);
      }
      publisher->publish(marker_array);
    }

    template<typename T>
    std::vector<T> slice(std::vector<T> const & v, int m, int n)
    {
      auto first = v.cbegin() + m;
      auto last = v.cbegin() + n + 1;

      std::vector<T> vec(first, last);
      return vec;
    }

    static float dot(Eigen::Vector3f a, Eigen::Vector3f b)
    {
      float value = a.x() * b.x() + a.y() * b.y() + a.z() * b.z();
      return value;
    }

    static float mag(Eigen::Vector3f a)
    {
      return std::sqrt(a.x() * a.x() + a.y() * a.y() + a.z() * a.z());
    }

    static vox_nav_msgs::msg::ObjectArray::SharedPtr  trimObstaclesToN(
      const vox_nav_msgs::msg::ObjectArray & obstacle_tracks,
      const geometry_msgs::msg::PoseStamped & curr_robot_pose,
      int N)
    {
      auto trimmed_N_obstacles =
        std::make_shared<vox_nav_msgs::msg::ObjectArray>(obstacle_tracks);

      if (obstacle_tracks.objects.size() < N) {

        for (size_t i = 0; i < N - obstacle_tracks.objects.size(); i++) {
          vox_nav_msgs::msg::Object ghost_obstacle;
          ghost_obstacle.world_pose.point.x = 20000.0;
          ghost_obstacle.world_pose.point.y = 20000.0;
          ghost_obstacle.world_pose.point.z = 20000.0;
          ghost_obstacle.length = 0.1;
          ghost_obstacle.width = 0.1;
          ghost_obstacle.height = 0.1;
          trimmed_N_obstacles->objects.push_back(ghost_obstacle);
        }
      } else {
        trimmed_N_obstacles->objects.clear();
        trimmed_N_obstacles->objects.resize(N);

        std::vector<double> distances;
        for (auto && obs : obstacle_tracks.objects) {
          double dist = vox_nav_utilities::getEuclidianDistBetweenPoints(
            obs.world_pose.point,
            curr_robot_pose.pose.position);
          distances.push_back(dist);
        }

        std::vector<int> sorted_indices(distances.size());
        std::iota(sorted_indices.begin(), sorted_indices.end(), 0);
        auto comparator = [&distances](int a, int b) {return distances[a] < distances[b];};
        std::sort(sorted_indices.begin(), sorted_indices.end(), comparator);

        for (size_t i = 0; i < N; i++) {
          trimmed_N_obstacles->objects[i] = obstacle_tracks.objects[sorted_indices[i]];
        }
      }

      return trimmed_N_obstacles;
    }

    static geometry_msgs::msg::Point solve(Ellipsoid e, geometry_msgs::msg::Point p)
    {

      auto px = std::abs(p.x - e.center.x());
      auto py = std::abs(p.y - e.center.y());
      auto t = M_PI_4;
      auto a = e.axes.x();
      auto b = e.axes.y();
      double x, y;

      for (int i = 0; i < 3; i++) {
        x = a * std::cos(t);
        y = b * std::sin(t);
        auto ex = (a * a - b * b) * std::pow(std::cos(t), 3) / a;
        auto ey = (b * b - a * a) * std::pow(std::sin(t), 3) / b;
        auto rx = x - ex;
        auto ry = y - ey;
        auto qx = px - ex;
        auto qy = py - ey;
        auto r = std::hypot(ry, rx);
        auto q = std::hypot(qy, qx);
        auto delta_c = r * std::asin((rx * qy - ry * qx) / (r * q));
        auto delta_t = delta_c / std::sqrt(a * a + b * b - x * x - y * y);
        t += delta_t;
        t = std::min(M_PI_2, std::max(0.0, t));
      }

      geometry_msgs::msg::Point at;
      at.x = std::copysign(x, p.x);
      at.y = std::copysign(y, p.y);
      at.z = 0;

      return at;
    }


  }  //   namespace common

}  // namespace vox_nav_control

#endif  // VOX_NAV_CONTROL__COMMON_HPP_
