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
#include <vox_nav_control/mpc_controller_acado/mpc_controller_acado_ros.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <memory>
#include <vector>
#include <string>

/* Global variables used by the solver. */
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

namespace vox_nav_control
{
  namespace mpc_controller_acado
  {
    MPCControllerAcadoROS::MPCControllerAcadoROS()
    {
    }

    MPCControllerAcadoROS::~MPCControllerAcadoROS()
    {
    }
    void MPCControllerAcadoROS::initialize(
      rclcpp::Node * parent,
      const std::string & plugin_name)
    {
      parent_ = parent;
      parent->declare_parameter("global_plan_look_ahead_distance", 2.5);
      parent->declare_parameter("ref_traj_se2_space", "REEDS");
      parent->declare_parameter("rho", 2.5);
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

      parent->get_parameter("global_plan_look_ahead_distance", global_plan_look_ahead_distance_);
      parent->get_parameter("ref_traj_se2_space", selected_se2_space_name_);
      parent->get_parameter("rho", rho_);
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

      interpolated_local_reference_traj_publisher_ =
        parent->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/vox_nav/controller/plan", 1);

      mpc_computed_traj_publisher_ =
        parent->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/vox_nav/controller/mpc_computed_traj", 1);

      obstacle_tracks_sub_ = parent->create_subscription<vox_nav_msgs::msg::ObjectArray>(
        "/vox_nav/tracking/objects", rclcpp::SystemDefaultsQoS(),
        std::bind(&MPCControllerAcadoROS::obstacleTracksCallback, this, std::placeholders::_1));

      //Sets initial states, inputs to zeros and makes solver ready
      initAcadoStuff();

      // use parameters in mpc_parameters_.Q and mpc_parameters_.R
      initAcadoWeights();

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

    }

    geometry_msgs::msg::Twist MPCControllerAcadoROS::computeVelocityCommands(
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
      double kTARGET_SPEED = mpc_parameters_.V_MAX;

      double robot_roll, robot_pitch, robot_psi;
      vox_nav_utilities::getRPYfromMsgQuaternion(
        curr_robot_pose.pose.orientation, robot_roll, robot_pitch, robot_psi);

      vox_nav_control::common::States curr_states;
      curr_states.x = curr_robot_pose.pose.position.x;
      curr_states.y = curr_robot_pose.pose.position.y;
      curr_states.psi = robot_psi;
      curr_states.v = kTARGET_SPEED;

      std::vector<vox_nav_control::common::States> local_interpolated_reference_states =
        getLocalInterpolatedReferenceStates(
        curr_robot_pose, mpc_parameters_, reference_traj_,
        global_plan_look_ahead_distance_, state_space_information_);

      setRefrenceStates(local_interpolated_reference_states, previous_control_);
      updateCurrentStates(curr_states);

      acado_preparationStep();
      auto ret = acado_feedbackStep();

      auto obstacles = trackMsg2Ellipsoids(obstacle_tracks_);

      if (mpc_parameters_.debug_mode) {
        std::cout << "Return code from acado_feedbackStep(): " << ret << std::endl;
        acado_printDifferentialVariables();
        acado_printControlVariables();
      }

      std::vector<vox_nav_control::common::ControlInput> computed_controls =
        getPredictedControlsFromAcado();
      std::vector<vox_nav_control::common::States> computed_states = getPredictedStatesFromAcado();

      if (std::isnan(computed_controls.begin()->acc) || std::isnan(computed_controls.begin()->df)) {
        RCLCPP_WARN(parent_->get_logger(), "NAN Control outputs from Acado!");
      } else {
        //  The control output is acceleration but we need to publish speed
        computed_velocity_.linear.x += computed_controls.begin()->acc * (dt);
        //  The control output is steeering angle but we need to publish angular velocity
        /*computed_velocity_.angular.z = (computed_velocity_.linear.x * computed_controls.begin()->df) /
          (mpc_parameters_.L_R + mpc_parameters_.L_F);*/
        computed_velocity_.angular.z = computed_controls.begin()->df;
      }

      regulate_max_speed();

      std_msgs::msg::ColorRGBA red_color, blue_color;
      red_color.r = 1.0;
      red_color.a = 1.0;
      blue_color.b = 1.0;
      blue_color.a = 1.0;
      publishTrajStates(
        local_interpolated_reference_states, red_color, "ref_traj",
        interpolated_local_reference_traj_publisher_);

      publishTrajStates(
        computed_states, blue_color, "actual_traj",
        mpc_computed_traj_publisher_);

      previous_control_ = computed_controls;

      return computed_velocity_;
    }

    geometry_msgs::msg::Twist MPCControllerAcadoROS::computeHeadingCorrectionCommands(
      geometry_msgs::msg::PoseStamped curr_robot_pose)
    {
      double dt = mpc_parameters_.DT;
      double kTARGET_SPEED = mpc_parameters_.V_MAX;

      double robot_roll, robot_pitch, robot_psi;
      vox_nav_utilities::getRPYfromMsgQuaternion(
        curr_robot_pose.pose.orientation, robot_roll, robot_pitch, robot_psi);

      vox_nav_control::common::States curr_states;
      curr_states.x = curr_robot_pose.pose.position.x;
      curr_states.y = curr_robot_pose.pose.position.y;
      curr_states.psi = robot_psi;
      curr_states.v = kTARGET_SPEED;
      std::vector<vox_nav_control::common::States> local_interpolated_reference_states =
        getLocalInterpolatedReferenceStates(
        curr_robot_pose, mpc_parameters_, reference_traj_,
        global_plan_look_ahead_distance_, state_space_information_);

      setRefrenceStates(local_interpolated_reference_states, previous_control_);
      updateCurrentStates(curr_states);

      acado_preparationStep();
      auto ret = acado_feedbackStep();

      auto obstacles = trackMsg2Ellipsoids(obstacle_tracks_);

      if (mpc_parameters_.debug_mode) {
        std::cout << "Return code from acado_feedbackStep(): " << ret << std::endl;
        acado_printDifferentialVariables();
        acado_printControlVariables();
      }

      std::vector<vox_nav_control::common::ControlInput> computed_controls =
        getPredictedControlsFromAcado();
      std::vector<vox_nav_control::common::States> computed_states = getPredictedStatesFromAcado();

      //  The control output is acceleration but we need to publish speed
      computed_velocity_.linear.x += 0;
      //  The control output is steeering angle but we need to publish angular velocity
      computed_velocity_.angular.z += computed_controls.begin()->df * dt;

      return computed_velocity_;
    }

    void MPCControllerAcadoROS::setPlan(const nav_msgs::msg::Path & path)
    {
      reference_traj_ = path;
    }

    void MPCControllerAcadoROS::obstacleTracksCallback(
      const vox_nav_msgs::msg::ObjectArray::SharedPtr msg)
    {
      std::lock_guard<std::mutex> guard(obstacle_tracks_mutex_);
      obstacle_tracks_ = *msg;

      /*RCLCPP_INFO(
        parent_->get_logger(), "Recieved Tracks [%d]", int(obstacle_tracks_.objects.size()));*/
    }

    std::vector<vox_nav_control::common::Ellipsoid> MPCControllerAcadoROS::trackMsg2Ellipsoids(
      const vox_nav_msgs::msg::ObjectArray & tracks)
    {
      std::vector<vox_nav_control::common::Ellipsoid> ellipsoids;
      for (auto && i : tracks.objects) {
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
        ellipsoids.push_back(e);
      }

      return ellipsoids;

    }

    void MPCControllerAcadoROS::initAcadoStuff()
    {
      // Initialize the solver.
      acado_initializeSolver();
      // Initialize the states and controls.
      for (int i = 0; i < ACADO_NX * (ACADO_N + 1); ++i) {
        acadoVariables.x[i] = 0.0;
      }
      for (int i = 0; i < ACADO_NU * ACADO_N; ++i) {
        acadoVariables.u[i] = 0.0;
      }
      for (int i = 0; i < ACADO_NY * ACADO_N; ++i) {
        acadoVariables.y[i] = 0.0;
      }
      for (int i = 0; i < ACADO_NYN; ++i) {
        acadoVariables.yN[i] = 0.0;
      }
      for (int i = 0; i < ACADO_N; i++) {
        previous_control_.push_back(vox_nav_control::common::ControlInput());
      }

      for (size_t i = 0; i < ACADO_NOD * (ACADO_N + 1); i++) {
        acadoVariables.od[i] = 0.1;
      }

      // Prepare first step
      acado_preparationStep();
    }

    void MPCControllerAcadoROS::initAcadoWeights()
    {
      double w_x = mpc_parameters_.Q[vox_nav_control::common::STATE_ENUM::kX];
      double w_y = mpc_parameters_.Q[vox_nav_control::common::STATE_ENUM::kY];
      double w_yaw = mpc_parameters_.Q[vox_nav_control::common::STATE_ENUM::kPsi];
      double w_vel = mpc_parameters_.Q[vox_nav_control::common::STATE_ENUM::kV];
      double w_obs = mpc_parameters_.Q[vox_nav_control::common::STATE_ENUM::kObs];
      double w_acc = mpc_parameters_.R[vox_nav_control::common::INPUT_ENUM::kacc];
      double w_df = mpc_parameters_.R[vox_nav_control::common::INPUT_ENUM::kdf];
      for (int i = 0; i < ACADO_N; i++) {
        // Setup diagonal entries
        acadoVariables.W[ACADO_NY * ACADO_NY * i + (ACADO_NY + 1) * 0] = w_x;
        acadoVariables.W[ACADO_NY * ACADO_NY * i + (ACADO_NY + 1) * 1] = w_y;
        acadoVariables.W[ACADO_NY * ACADO_NY * i + (ACADO_NY + 1) * 2] = w_yaw;
        acadoVariables.W[ACADO_NY * ACADO_NY * i + (ACADO_NY + 1) * 3] = w_vel;
        acadoVariables.W[ACADO_NY * ACADO_NY * i + (ACADO_NY + 1) * 4] = w_acc;
        acadoVariables.W[ACADO_NY * ACADO_NY * i + (ACADO_NY + 1) * 5] = w_df;
        acadoVariables.W[ACADO_NY * ACADO_NY * i + (ACADO_NY + 1) * 6] = w_obs;
      }
      acadoVariables.WN[(ACADO_NYN + 1) * 0] = w_x;
      acadoVariables.WN[(ACADO_NYN + 1) * 1] = w_y;
      acadoVariables.WN[(ACADO_NYN + 1) * 2] = w_yaw;
      acadoVariables.WN[(ACADO_NYN + 1) * 3] = w_vel;

      for (int i = 0; i < ACADO_N * ACADO_NY; i++) {
        //acadoVariables.W[i] = 1;
      }
    }

    void MPCControllerAcadoROS::setRefrenceStates(
      const std::vector<vox_nav_control::common::States> & ref_states,
      const std::vector<vox_nav_control::common::ControlInput> & prev_controls)
    {

      for (int i = 0; i < ACADO_NY * ACADO_N; ++i) {
        int state = i % ACADO_NY;
        int index = i / ACADO_NY;
        if (state == vox_nav_control::common::STATE_ENUM::kX) {
          acadoVariables.y[i] = ref_states[index].x;
        } else if (state == vox_nav_control::common::STATE_ENUM::kY) {
          acadoVariables.y[i] = ref_states[index].y;
        } else if (state == vox_nav_control::common::STATE_ENUM::kPsi) {
          acadoVariables.y[i] = ref_states[index].psi;
        } else if (state == vox_nav_control::common::STATE_ENUM::kV) {
          acadoVariables.y[i] = ref_states[index].v;
        } else if (state == 4) {
          acadoVariables.y[i] = prev_controls.begin()->acc;
        } else if (state == 5) {
          acadoVariables.y[i] = prev_controls.begin()->df;
        }
      }

      // Set the Terminal Reference
      for (int i = 0; i < ACADO_NYN; ++i) {
        auto index = ref_states.size() - 1;
        if (i == vox_nav_control::common::STATE_ENUM::kX) {
          acadoVariables.yN[i] = ref_states[index].x;
        } else if (i == vox_nav_control::common::STATE_ENUM::kY) {
          acadoVariables.yN[i] = ref_states[index].y;
        } else if (i == vox_nav_control::common::STATE_ENUM::kPsi) {
          acadoVariables.yN[i] = ref_states[index].psi;
        } else if (i == vox_nav_control::common::STATE_ENUM::kV) {
          acadoVariables.yN[i] = ref_states[index].v;
        }
      }

      for (int i = 0; i < ACADO_NOD * (ACADO_N + 1); ++i) {
        int index = i % (ACADO_N + 1);

        index = index % 3;
        if (index == vox_nav_control::common::STATE_ENUM::kX) {
          acadoVariables.od[i] = 0.0;
        } else if (index == vox_nav_control::common::STATE_ENUM::kY) {
          acadoVariables.od[i] = -5.0;
        } else {
          acadoVariables.od[i] = 2.0;
        }
      }
    }

    void MPCControllerAcadoROS::updateCurrentStates(vox_nav_control::common::States curr_states)
    {
      // MPC: set the current state feedback
      acadoVariables.x0[0] = curr_states.x;
      acadoVariables.x0[1] = curr_states.y;
      acadoVariables.x0[2] = curr_states.psi;
      acadoVariables.x0[3] = curr_states.v;
    }

    std::vector<vox_nav_control::common::ControlInput> MPCControllerAcadoROS::
    getPredictedControlsFromAcado()
    {
      std::vector<vox_nav_control::common::ControlInput> computed_controls;
      real_t * u = acado_getVariablesU();
      for (int i = 0; i < ACADO_N; ++i) {
        vox_nav_control::common::ControlInput curr;
        for (int j = 0; j < ACADO_NU; ++j) {
          if (j == 0) {
            curr.acc = (double)u[i * ACADO_NU + j];
          } else {
            curr.df = (double)u[i * ACADO_NU + j];
          }
        }
        computed_controls.push_back(curr);
      }
      return computed_controls;
    }

    std::vector<vox_nav_control::common::States> MPCControllerAcadoROS::getPredictedStatesFromAcado()
    {
      std::vector<vox_nav_control::common::States> computed_states;
      real_t * x = acado_getVariablesX();
      for (int i = 0; i < ACADO_N; ++i) {
        vox_nav_control::common::States curr;
        for (int j = 0; j < ACADO_NX; ++j) {

          if (j == vox_nav_control::common::STATE_ENUM::kX) {
            curr.x = (double)x[i * ACADO_NX + j];
          } else if (j == vox_nav_control::common::STATE_ENUM::kY) {
            curr.y = (double)x[i * ACADO_NX + j];
          } else if (j == vox_nav_control::common::STATE_ENUM::kPsi) {
            curr.psi = (double)x[i * ACADO_NX + j];
          } else {
            curr.v = (double)x[i * ACADO_NX + j];
          }
        }
        computed_states.push_back(curr);
      }
      return computed_states;
    }

  }   // namespace mpc_controller_acado
  PLUGINLIB_EXPORT_CLASS(
    mpc_controller_acado::MPCControllerAcadoROS,
    vox_nav_control::ControllerCore)
}   // namespace vox_nav_control
