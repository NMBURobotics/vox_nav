// Copyright (c) 2020 Fetullah Atas, Norwegian University of Life Sciences
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
#include <vox_nav_control/mpc_controller/mpc_controller_ros.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <memory>
#include <vector>
#include <string>

namespace vox_nav_control
{
  namespace mpc_controller
  {
    MPCControllerROS::MPCControllerROS()
    {
    }

    MPCControllerROS::~MPCControllerROS()
    {
    }
    void MPCControllerROS::initialize(
      rclcpp::Node * parent,
      const std::string & plugin_name)
    {
      parent_ = parent;
      parent->declare_parameter("global_plan_look_ahead_distance", 2.5);
      parent->declare_parameter(plugin_name + ".N", 10);
      parent->declare_parameter(plugin_name + ".DT", 0.1);
      parent->declare_parameter(plugin_name + ".L_F", 0.66);
      parent->declare_parameter(plugin_name + ".L_R", 0.66);
      parent->declare_parameter(plugin_name + ".V_MIN", -2.0);
      parent->declare_parameter(plugin_name + ".V_MAX", 2.0);
      parent->declare_parameter(plugin_name + ".A_MIN", -1.0);
      parent->declare_parameter(plugin_name + ".A_MAX", 1.0);
      parent->declare_parameter(plugin_name + ".DF_MIN", -0.5);
      parent->declare_parameter(plugin_name + ".DF_MAX", 0.5);
      parent->declare_parameter(plugin_name + ".A_DOT_MIN", -1.0);
      parent->declare_parameter(plugin_name + ".A_DOT_MAX", 1.0);
      parent->declare_parameter(plugin_name + ".DF_DOT_MIN", -0.4);
      parent->declare_parameter(plugin_name + ".DF_DOT_MAX", 0.4);
      parent->declare_parameter(plugin_name + ".Q", std::vector<double>({10.0, 10.0, 10.0, 0.0}));
      parent->declare_parameter(plugin_name + ".R", std::vector<double>({10.0, 100.0}));
      parent->declare_parameter(plugin_name + ".debug_mode", false);
      parent->declare_parameter(plugin_name + ".params_configured", false);

      parent->get_parameter("global_plan_look_ahead_distance", global_plan_look_ahead_distance_);
      parent->get_parameter(plugin_name + ".N", mpc_parameters_.N);
      parent->get_parameter(plugin_name + ".DT", mpc_parameters_.DT);
      parent->get_parameter(plugin_name + ".L_F", mpc_parameters_.L_F);
      parent->get_parameter(plugin_name + ".L_R", mpc_parameters_.L_R);
      parent->get_parameter(plugin_name + ".V_MIN", mpc_parameters_.V_MIN);
      parent->get_parameter(plugin_name + ".V_MAX", mpc_parameters_.V_MAX);
      parent->get_parameter(plugin_name + ".A_MIN", mpc_parameters_.A_MIN);
      parent->get_parameter(plugin_name + ".A_MAX", mpc_parameters_.A_MAX);
      parent->get_parameter(plugin_name + ".DF_MIN", mpc_parameters_.DF_MIN);
      parent->get_parameter(plugin_name + ".DF_MAX", mpc_parameters_.DF_MAX);
      parent->get_parameter(plugin_name + ".A_DOT_MIN", mpc_parameters_.A_DOT_MIN);
      parent->get_parameter(plugin_name + ".A_DOT_MAX", mpc_parameters_.A_DOT_MAX);
      parent->get_parameter(plugin_name + ".DF_DOT_MIN", mpc_parameters_.DF_DOT_MIN);
      parent->get_parameter(plugin_name + ".DF_DOT_MAX", mpc_parameters_.DF_DOT_MAX);
      parent->get_parameter(plugin_name + ".Q", mpc_parameters_.Q);
      parent->get_parameter(plugin_name + ".R", mpc_parameters_.R);
      parent->get_parameter(plugin_name + ".debug_mode", mpc_parameters_.debug_mode);
      parent->get_parameter(plugin_name + ".params_configured", mpc_parameters_.params_configured);

      interpolated_local_reference_traj_publisher_ =
        parent->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/vox_nav/controller/plan",
        1);

      obstacle_tracks_sub_ = parent->create_subscription<vox_nav_msgs::msg::ObjectArray>(
        "/vox_nav/tracking/objects", rclcpp::SystemDefaultsQoS(),
        std::bind(&MPCControllerROS::obstacleTracksCallback, this, std::placeholders::_1));

      mpc_controller_ = std::make_shared<MPCControllerCore>(mpc_parameters_);
    }

    geometry_msgs::msg::Twist MPCControllerROS::computeVelocityCommands(
      geometry_msgs::msg::PoseStamped curr_robot_pose)
    {

      auto regulate_max_speed = [this]() {
          if (computed_velocity_.linear.x > mpc_parameters_.V_MAX) {
            computed_velocity_.linear.x = mpc_parameters_.V_MAX;
          } else if (computed_velocity_.linear.x < mpc_parameters_.V_MIN) {
            computed_velocity_.linear.x = mpc_parameters_.V_MIN;
          }
        };

      double dt = mpc_parameters_.DT;
      double kTARGET_SPEED = 0.0;
      // distance from rear to front axle(m)
      double rear_axle_tofront_dist = mpc_parameters_.L_R + mpc_parameters_.L_F;

      double roll, pitch, psi;
      vox_nav_utilities::getRPYfromMsgQuaternion(
        curr_robot_pose.pose.orientation, roll, pitch, psi);

      MPCControllerCore::States curr_states;
      curr_states.x = curr_robot_pose.pose.position.x;
      curr_states.y = curr_robot_pose.pose.position.y;
      curr_states.psi = psi;
      curr_states.v = kTARGET_SPEED;

      std::vector<MPCControllerCore::States> local_interpolated_reference_states =
        getLocalInterpolatedReferenceStates(curr_robot_pose);
      mpc_controller_->updateCurrentStates(curr_states);
      mpc_controller_->updateReferences(local_interpolated_reference_states);
      mpc_controller_->updatePreviousControlInput(previous_control_);
      auto obstacles = trackMsg2Ellipsoids(obstacle_tracks_);
      MPCControllerCore::SolutionResult res = mpc_controller_->solve(obstacles);

      //  The control output is acceleration but we need to publish speed
      computed_velocity_.linear.x += res.control_input.acc * (dt);
      //  The control output is steeering angle but we need to publish angular velocity
      computed_velocity_.angular.z = (computed_velocity_.linear.x * res.control_input.df) /
        rear_axle_tofront_dist;

      regulate_max_speed();
      publishLocalInterpolatedRefernceStates(local_interpolated_reference_states);
      previous_control_ = res.control_input;
      return computed_velocity_;
    }

    geometry_msgs::msg::Twist MPCControllerROS::computeHeadingCorrectionCommands(
      geometry_msgs::msg::PoseStamped curr_robot_pose)
    {

      double dt = mpc_parameters_.DT;
      double kTARGET_SPEED = 0.0;

      // we dont really need roll and pitch here
      double nan, psi;
      vox_nav_utilities::getRPYfromMsgQuaternion(
        curr_robot_pose.pose.orientation, nan, nan, psi);

      MPCControllerCore::States curr_states;
      curr_states.x = curr_robot_pose.pose.position.x;
      curr_states.y = curr_robot_pose.pose.position.y;
      curr_states.psi = psi;
      curr_states.v = kTARGET_SPEED;

      std::vector<MPCControllerCore::States> local_interpolated_reference_states =
        getLocalInterpolatedReferenceStates(curr_robot_pose);

      mpc_controller_->updateCurrentStates(curr_states);
      mpc_controller_->updateReferences(local_interpolated_reference_states);
      mpc_controller_->updatePreviousControlInput(previous_control_);

      std::lock_guard<std::mutex> guard(obstacle_tracks_mutex_);
      auto obstacles = trackMsg2Ellipsoids(obstacle_tracks_);
      MPCControllerCore::SolutionResult res = mpc_controller_->solve(obstacles);

      computed_velocity_.linear.x = 0;
      computed_velocity_.angular.z += res.control_input.df * (dt);
      previous_control_ = res.control_input;

      return computed_velocity_;
    }

    void MPCControllerROS::setPlan(const nav_msgs::msg::Path & path)
    {
      reference_traj_ = path;
    }

    int MPCControllerROS::nearestStateIndex(
      nav_msgs::msg::Path reference_traj, geometry_msgs::msg::PoseStamped curr_robot_pose)
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

    std::vector<MPCControllerCore::States> MPCControllerROS::getLocalInterpolatedReferenceStates(
      geometry_msgs::msg::PoseStamped curr_robot_pose)
    {
      double kTARGETSPEED = 0.0;
      // Now lets find nearest trajectory point to robot base
      int nearsest_traj_state_index = nearestStateIndex(reference_traj_, curr_robot_pose);

      // Auto calculate interpolation steps
      double path_euclidian_length = 0.0;
      for (size_t i = 1; i < reference_traj_.poses.size(); i++) {
        path_euclidian_length += vox_nav_utilities::getEuclidianDistBetweenPoses(
          reference_traj_.poses[i], reference_traj_.poses[i - 1]);
      }

      double interpolation_step_size = path_euclidian_length / reference_traj_.poses.size();
      int states_to_see_horizon = global_plan_look_ahead_distance_ / interpolation_step_size;
      int local_goal_state_index = nearsest_traj_state_index + states_to_see_horizon;
      if (local_goal_state_index >= reference_traj_.poses.size() - 1) {
        local_goal_state_index = reference_traj_.poses.size() - 1;
      }
      // Define a state space, we basically need this only because we want to use OMPL's
      // geometric path, And then we can interpolate this path
      std::shared_ptr<ompl::base::RealVectorBounds> state_space_bounds =
        std::make_shared<ompl::base::RealVectorBounds>(2);
      ompl::base::StateSpacePtr state_space =
        std::make_shared<ompl::base::SE2StateSpace>();
      state_space->as<ompl::base::SE2StateSpace>()->setBounds(*state_space_bounds);
      ompl::base::SpaceInformationPtr state_space_information =
        std::make_shared<ompl::base::SpaceInformation>(state_space);
      ompl::geometric::PathGeometric path(state_space_information);

      ompl::base::ScopedState<ompl::base::SE2StateSpace>
      closest_ref_traj_state(state_space),
      ompl_local_goal_state(state_space);

      // Feed initial state, which is closest ref trajectory state
      double void_var, yaw;

      // Feed Intermediate state , which is nearest state in ref traj
      vox_nav_utilities::getRPYfromMsgQuaternion(
        reference_traj_.poses[nearsest_traj_state_index].pose.orientation, void_var, void_var, yaw);
      closest_ref_traj_state[0] = reference_traj_.poses[nearsest_traj_state_index].pose.position.x;
      closest_ref_traj_state[1] = reference_traj_.poses[nearsest_traj_state_index].pose.position.y;
      closest_ref_traj_state[2] = yaw;
      path.append(static_cast<ompl::base::State *>(closest_ref_traj_state.get()));

      // Feed the final state, which the local goal for the current control effort.
      // This is basically the state in the ref trajectory, which is closest to global_plan_look_ahead_distance_
      vox_nav_utilities::getRPYfromMsgQuaternion(
        reference_traj_.poses[local_goal_state_index].pose.orientation, void_var, void_var, yaw);
      ompl_local_goal_state[0] = reference_traj_.poses[local_goal_state_index].pose.position.x;
      ompl_local_goal_state[1] = reference_traj_.poses[local_goal_state_index].pose.position.y;
      ompl_local_goal_state[2] = yaw;
      path.append(static_cast<ompl::base::State *>(ompl_local_goal_state.get()));

      // The local ref traj now contains only 3 states, we will interpolate this states with OMPL
      // The count of states after interpolation must be same as horizon defined for the control problem , hence
      // it should be mpc_parameters_.N
      path.interpolate(mpc_parameters_.N);

      // Now the local ref traj is interpolated from current robot state up to state at global look ahead distance
      // Lets fill the native MPC type ref states and return to caller
      std::vector<MPCControllerCore::States> interpolated_reference_states;
      for (std::size_t path_idx = 0; path_idx < path.getStateCount(); path_idx++) {
        // cast the abstract state type to the type we expect
        const ompl::base::ReedsSheppStateSpace::StateType * interpolated_state =
          path.getState(path_idx)->as<ompl::base::ReedsSheppStateSpace::StateType>();
        MPCControllerCore::States curr_interpolated_state;
        curr_interpolated_state.v = kTARGETSPEED;
        curr_interpolated_state.x = interpolated_state->getX();
        curr_interpolated_state.y = interpolated_state->getY();
        curr_interpolated_state.psi = interpolated_state->getYaw();
        interpolated_reference_states.push_back(curr_interpolated_state);
      }
      return interpolated_reference_states;
    }

    void
    MPCControllerROS::publishLocalInterpolatedRefernceStates(
      std::vector<MPCControllerCore::States> interpolated_reference_states)
    {
      visualization_msgs::msg::MarkerArray marker_array;
      for (int i = 0; i < interpolated_reference_states.size(); i++) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = rclcpp::Clock().now();
        marker.ns = "path";
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.lifetime = rclcpp::Duration::from_seconds(0);
        marker.pose.position.x = interpolated_reference_states[i].x;
        marker.pose.position.y = interpolated_reference_states[i].y;
        marker.pose.position.z = 0.6;
        marker.pose.orientation = vox_nav_utilities::getMsgQuaternionfromRPY(
          0, 0, interpolated_reference_states[i].psi);
        marker.scale.x = 0.25;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        // Make it colorful
        marker.color.r = 1.0 * static_cast<double>(rclcpp::Clock().now().nanoseconds() % 10);
        marker.color.g = 0.5 * static_cast<double>(rclcpp::Clock().now().nanoseconds() % 10);
        marker.color.b = 0.2 * static_cast<double>(rclcpp::Clock().now().nanoseconds() % 10);
        marker_array.markers.push_back(marker);
      }
      interpolated_local_reference_traj_publisher_->publish(marker_array);
    }

    void MPCControllerROS::obstacleTracksCallback(
      const vox_nav_msgs::msg::ObjectArray::SharedPtr msg)
    {
      std::lock_guard<std::mutex> guard(obstacle_tracks_mutex_);
      obstacle_tracks_ = *msg;

      /*RCLCPP_INFO(
        parent_->get_logger(), "Recieved Tracks [%d]", int(obstacle_tracks_.objects.size()));*/
    }

    std::vector<Ellipsoid> MPCControllerROS::trackMsg2Ellipsoids(
      const vox_nav_msgs::msg::ObjectArray & tracks)
    {
      std::vector<Ellipsoid> ellipsoids;
      for (auto && i : tracks.objects) {
        // Now lets put the ellipsoid into form of ;
        // ax^2+2bxy+cy^2=1,
        // The shape matrix P is;
        // [a b
        //  b c]
        /*
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
        Ellipsoid e;
        e.heading = i.heading;
        e.is_dynamic = i.is_dynamic;
        e.center = center;
        e.axes = Eigen::Vector2f(a, b);
        ellipsoids.push_back(e);
      }

      return ellipsoids;

    }

  } // namespace mpc_controller
  PLUGINLIB_EXPORT_CLASS(mpc_controller::MPCControllerROS, vox_nav_control::ControllerCore)
}  // namespace vox_nav_control
