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
#include <botanbot_control/mpc_controller/mpc_controller_ros.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <memory>
#include <vector>
#include <string>

namespace botanbot_control
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

  interpolated_ref_traj_publisher_ = parent->create_publisher<visualization_msgs::msg::MarkerArray>(
    "interpolated_plan", 1);

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

  tf2::Quaternion q;
  tf2::fromMsg(curr_robot_pose.pose.orientation, q);
  tf2::Matrix3x3 m(q);
  double roll, pitch, psi;
  m.getRPY(roll, pitch, psi);

  MPCControllerCore::States curr_states;
  curr_states.x = curr_robot_pose.pose.position.x;
  curr_states.y = curr_robot_pose.pose.position.y;
  curr_states.psi = psi;
  curr_states.v = kTARGET_SPEED;

  std::vector<MPCControllerCore::States> interpolated_reference_states =
    getInterpolatedRefernceStates(reference_traj_, curr_robot_pose);

  mpc_controller_->updateCurrentStates(curr_states);
  mpc_controller_->updateReferences(interpolated_reference_states);
  mpc_controller_->updatePreviousControlInput(previous_control_);
  MPCControllerCore::SolutionResult res = mpc_controller_->solve();

  //  The control output is acceleration but we need to publish speed
  computed_velocity_.linear.x += res.control_input.acc * (dt);
  //  The control output is steeering angle but we need to publish angular velocity
  computed_velocity_.angular.z = (computed_velocity_.linear.x * res.control_input.df) /
    rear_axle_tofront_dist;

  regulate_max_speed();
  publishInterpolatedRefernceStates(interpolated_reference_states);
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
      std::sqrt(
      std::pow(reference_traj.poses[i].pose.position.x - curr_robot_pose.pose.position.x, 2) +
      std::pow(reference_traj.poses[i].pose.position.y - curr_robot_pose.pose.position.y, 2));
    if (curr_distance < closest_state_distance) {
      closest_state_distance = curr_distance;
      closest_state_index = i;
    }
  }
  return closest_state_index;
}

std::vector<MPCControllerCore::States> MPCControllerROS::getInterpolatedRefernceStates(
  const nav_msgs::msg::Path ref_traj,
  geometry_msgs::msg::PoseStamped curr_robot_pose)
{
  int kINTERPOLATION = mpc_parameters_.N;
  double kTARGETSPEED = 0.0;

  std::shared_ptr<ompl::base::RealVectorBounds> state_space_bounds =
    std::make_shared<ompl::base::RealVectorBounds>(2);
  state_space_bounds->setLow(-100);
  state_space_bounds->setHigh(100);
  ompl::base::StateSpacePtr state_space =
    std::make_shared<ompl::base::ReedsSheppStateSpace>();
  state_space->as<ompl::base::ReedsSheppStateSpace>()->setBounds(*state_space_bounds);
  ompl::base::SpaceInformationPtr state_space_information =
    std::make_shared<ompl::base::SpaceInformation>(state_space);

  ompl::geometric::PathGeometric path(state_space_information);

  ompl::base::ScopedState<ompl::base::ReedsSheppStateSpace> ompl_robot_curr_state(state_space);
  // Now lets find nearest trajectory point to robot base
  int nearsest_traj_state_index =
    nearestStateIndex(ref_traj, curr_robot_pose);

  double nan, yaw;
  botanbot_utilities::getRPYfromQuaternion(
    ref_traj.poses[nearsest_traj_state_index].pose.orientation,
    nan, nan, yaw);
  ompl_robot_curr_state[0] = ref_traj.poses[nearsest_traj_state_index].pose.position.x;
  ompl_robot_curr_state[1] = ref_traj.poses[nearsest_traj_state_index].pose.position.y;
  ompl_robot_curr_state[2] = yaw;
  // Feed first state , which is the current robo state.
  path.append(static_cast<ompl::base::State *>(ompl_robot_curr_state.get()));

  ompl::base::ScopedState<ompl::base::ReedsSheppStateSpace> ompl_local_goal_state(state_space);

  double distance_to_goal = botanbot_utilities::getEuclidianDistBetweenPoses(
    ref_traj.poses.front(), ref_traj.poses.back());
  double interpolation_step = distance_to_goal / ref_traj.poses.size();

  double kLOOKUPPATHHORIZON = 2.0;
  int states_to_see_horizon = kLOOKUPPATHHORIZON / interpolation_step;

  int local_goal_stat_index = nearsest_traj_state_index + states_to_see_horizon;

  if (local_goal_stat_index >= ref_traj.poses.size() - 1) {
    std::cout << "MPC Controller approaching to goal state..." << std::endl;
    local_goal_stat_index = ref_traj.poses.size() - 1;
  }
  botanbot_utilities::getRPYfromQuaternion(
    ref_traj.poses[local_goal_stat_index].pose.orientation,
    nan, nan, yaw);
  ompl_local_goal_state[0] = ref_traj.poses[local_goal_stat_index].pose.position.x;
  ompl_local_goal_state[1] = ref_traj.poses[local_goal_stat_index].pose.position.y;
  ompl_local_goal_state[2] = yaw;

  path.append(static_cast<ompl::base::State *>(ompl_local_goal_state.get()));
  path.interpolate(kINTERPOLATION);

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

void MPCControllerROS::fillOMPLStateFromPose(
  ompl::base::ScopedState<ompl::base::ReedsSheppStateSpace> state,
  const geometry_msgs::msg::PoseStamped * pose)
{
  double nan, yaw;
  botanbot_utilities::getRPYfromQuaternion(pose->pose.orientation, nan, nan, yaw);
  state[0] = pose->pose.position.x;
  state[1] = pose->pose.position.y;
  state[2] = yaw;
}

void
MPCControllerROS::publishInterpolatedRefernceStates(
  std::vector<MPCControllerCore::States> interpolated_reference_states)
{
  visualization_msgs::msg::MarkerArray marker_array;
  // iterae through Ref raj Xs
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
    tf2::Quaternion curr_ref_traj_sample_quat;
    curr_ref_traj_sample_quat.setRPY(0, 0, interpolated_reference_states[i].psi);
    marker.pose.orientation = tf2::toMsg(curr_ref_traj_sample_quat);
    marker.scale.x = 0.25;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 0.8;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker_array.markers.push_back(marker);
  }
  interpolated_ref_traj_publisher_->publish(marker_array);
}

}  // namespace mpc_controller
PLUGINLIB_EXPORT_CLASS(mpc_controller::MPCControllerROS, botanbot_control::ControllerCore)
}  // namespace botanbot_control
