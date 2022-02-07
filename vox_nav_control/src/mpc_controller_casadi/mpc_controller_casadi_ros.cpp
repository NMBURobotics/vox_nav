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
#include <vox_nav_control/mpc_controller_casadi/mpc_controller_casadi_ros.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <memory>
#include <vector>
#include <string>

namespace vox_nav_control
{
  namespace mpc_controller_casadi
  {
    MPCControllerCasadiROS::MPCControllerCasadiROS()
    {
    }

    MPCControllerCasadiROS::~MPCControllerCasadiROS()
    {
    }
    void MPCControllerCasadiROS::initialize(
      rclcpp::Node * parent,
      const std::string & plugin_name)
    {
      parent_ = parent;
      parent->declare_parameter("global_plan_look_ahead_distance", 2.5);
      parent->declare_parameter("ref_traj_se2_space", "REEDS");
      parent->declare_parameter("rho", 2.5);
      parent->declare_parameter("robot_radius", 0.5);
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
      parent->declare_parameter(plugin_name + ".max_obstacles", 1);
      parent->declare_parameter(plugin_name + ".obstacle_cost", 1.0);


      parent->get_parameter("global_plan_look_ahead_distance", global_plan_look_ahead_distance_);
      parent->get_parameter("ref_traj_se2_space", selected_se2_space_name_);
      parent->get_parameter("rho", rho_);
      parent->get_parameter("robot_radius", mpc_parameters_.robot_radius);
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
      parent->get_parameter(plugin_name + ".max_obstacles", mpc_parameters_.max_obstacles);
      parent->get_parameter(plugin_name + ".obstacle_cost", mpc_parameters_.obstacle_cost);


      interpolated_local_reference_traj_publisher_ =
        parent->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/vox_nav/controller/plan",
        1);

      mpc_computed_traj_publisher_ =
        parent->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/vox_nav/controller/mpc_computed_traj",
        1);

      obstacle_tracks_sub_ = parent->create_subscription<vox_nav_msgs::msg::ObjectArray>(
        "/vox_nav/tracking/objects", rclcpp::SystemDefaultsQoS(),
        std::bind(&MPCControllerCasadiROS::obstacleTracksCallback, this, std::placeholders::_1));

      mpc_controller_ = std::make_shared<MPCControllerCasadiCore>(mpc_parameters_);


      std::shared_ptr<ompl::base::RealVectorBounds> state_space_bounds =
        std::make_shared<ompl::base::RealVectorBounds>(2);
      if (selected_se2_space_name_ == "SE2") {
        state_space_ = std::make_shared<ompl::base::SE2StateSpace>();
        state_space_->as<ompl::base::SE2StateSpace>()->setBounds(*state_space_bounds);
      } else if (selected_se2_space_name_ == "DUBINS") {
        state_space_ = std::make_shared<ompl::base::DubinsStateSpace>(rho_, false);
        state_space_->as<ompl::base::DubinsStateSpace>()->setBounds(*state_space_bounds);
      } else {
        state_space_ = std::make_shared<ompl::base::ReedsSheppStateSpace>(rho_);
        state_space_->as<ompl::base::ReedsSheppStateSpace>()->setBounds(*state_space_bounds);
      }
      state_space_information_ = std::make_shared<ompl::base::SpaceInformation>(state_space_);

      solved_at_least_once_ = false;

    }

    geometry_msgs::msg::Twist MPCControllerCasadiROS::computeVelocityCommands(
      geometry_msgs::msg::PoseStamped curr_robot_pose)
    {

      double robot_roll, robot_pitch, robot_psi;
      vox_nav_utilities::getRPYfromMsgQuaternion(
        curr_robot_pose.pose.orientation, robot_roll, robot_pitch, robot_psi);

      vox_nav_control::common::States curr_states;
      curr_states.x = curr_robot_pose.pose.position.x;
      curr_states.y = curr_robot_pose.pose.position.y;
      curr_states.psi = robot_psi;
      curr_states.v = 0.0;

      std::vector<vox_nav_control::common::States> local_interpolated_reference_states =
        vox_nav_control::common::getLocalInterpolatedReferenceStates(
        curr_robot_pose, mpc_parameters_, reference_traj_,
        global_plan_look_ahead_distance_, state_space_information_);

      // There is a limit of number of obstacles we can handle,
      // There will be always a fixed amount of obstacles
      // If there is no obstacles at all just fill with ghost obstacles(all zeros)
      std::lock_guard<std::mutex> guard(obstacle_tracks_mutex_);

      vox_nav_msgs::msg::ObjectArray trimmed_N_obstacles =
        *vox_nav_control::common::trimObstaclesToN(
        obstacle_tracks_,
        curr_robot_pose,
        mpc_parameters_.max_obstacles);

      mpc_controller_->updateCurrentStates(curr_states);
      mpc_controller_->updateReferences(local_interpolated_reference_states);
      mpc_controller_->updatePreviousControlInput(previous_control_);
      auto obstacles = trackMsg2Ellipsoids(trimmed_N_obstacles, curr_robot_pose);
      mpc_controller_->updateObstacles(obstacles);
      MPCControllerCasadiCore::SolutionResult res = mpc_controller_->solve(obstacles);

      // distance from rear to front axle(m)
      double rear_axle_tofront_dist = mpc_parameters_.L_R + mpc_parameters_.L_F;
      //  The control output is acceleration but we need to publish speed
      computed_velocity_.linear.x += res.control_input.acc * (mpc_parameters_.DT);
      //  The control output is steeering angle but we need to publish angular velocity
      computed_velocity_.angular.z = (computed_velocity_.linear.x * res.control_input.df) /
        rear_axle_tofront_dist;

      vox_nav_control::common::regulateMaxSpeed(computed_velocity_, mpc_parameters_);

      std_msgs::msg::ColorRGBA red_color, blue_color;
      red_color.r = 1.0;
      red_color.a = 1.0;
      blue_color.b = 1.0;
      blue_color.a = 1.0;
      vox_nav_control::common::publishTrajStates(
        local_interpolated_reference_states, red_color, "ref_traj",
        interpolated_local_reference_traj_publisher_);
      vox_nav_control::common::publishTrajStates(
        res.actual_computed_states, blue_color, "actual_traj",
        mpc_computed_traj_publisher_);

      previous_control_ = res.control_input;

      return computed_velocity_;
    }

    geometry_msgs::msg::Twist MPCControllerCasadiROS::computeHeadingCorrectionCommands(
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

      std::vector<vox_nav_control::common::States> local_interpolated_reference_states =
        vox_nav_control::common::getLocalInterpolatedReferenceStates(
        curr_robot_pose, mpc_parameters_, reference_traj_,
        global_plan_look_ahead_distance_, state_space_information_);

      mpc_controller_->updateCurrentStates(curr_states);
      mpc_controller_->updateReferences(local_interpolated_reference_states);
      mpc_controller_->updatePreviousControlInput(previous_control_);

      std::lock_guard<std::mutex> guard(obstacle_tracks_mutex_);
      auto obstacles = trackMsg2Ellipsoids(obstacle_tracks_, curr_robot_pose);
      MPCControllerCasadiCore::SolutionResult res = mpc_controller_->solve(obstacles);

      computed_velocity_.linear.x = 0;
      computed_velocity_.angular.z += res.control_input.df * (mpc_parameters_.DT);

      vox_nav_control::common::regulateMaxSpeed(computed_velocity_, mpc_parameters_);

      previous_control_ = res.control_input;

      return computed_velocity_;
    }

    void MPCControllerCasadiROS::setPlan(const nav_msgs::msg::Path & path)
    {
      reference_traj_ = path;
    }

    void MPCControllerCasadiROS::obstacleTracksCallback(
      const vox_nav_msgs::msg::ObjectArray::SharedPtr msg)
    {
      std::lock_guard<std::mutex> guard(obstacle_tracks_mutex_);
      obstacle_tracks_ = *msg;

    }

    std::vector<vox_nav_control::common::Ellipsoid> MPCControllerCasadiROS::trackMsg2Ellipsoids(
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

  } // namespace mpc_controller_casadi
  PLUGINLIB_EXPORT_CLASS(
    mpc_controller_casadi::MPCControllerCasadiROS,
    vox_nav_control::ControllerCore)
}  // namespace vox_nav_control
