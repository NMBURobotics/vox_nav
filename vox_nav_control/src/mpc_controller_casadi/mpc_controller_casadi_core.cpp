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

// Equations and logic parts of this code was based on ;
// https://github.com/MPC-Berkeley/genesis_path_follower
// Also refer to https://github.com/MPC-Berkeley/barc/wiki/Car-Model

#include <vox_nav_control/mpc_controller_casadi/mpc_controller_casadi_core.hpp>

#include <memory>
#include <vector>

namespace vox_nav_control
{
  namespace mpc_controller_casadi
  {

    MPCControllerCasadiCore::MPCControllerCasadiCore(vox_nav_control::common::Parameters params)
    {

      initial_solution_found_ = false;
      params_ = params;
      opti_ = std::make_shared<casadi::Opti>();

      // make weight matrixes diagonal
      Q = Q.diag(params_.Q);
      R = R.diag(params_.R);

      u_prev_ = opti_->parameter(2);
      z_curr_ = opti_->parameter(4);

      // reference traj that we would like to follow
      z_ref_ = opti_->parameter(params_.N, 4);
      x_ref_ = z_ref_(slice_all_, 0);
      y_ref_ = z_ref_(slice_all_, 1);
      psi_ref_ = z_ref_(slice_all_, 2);
      v_ref_ = z_ref_(slice_all_, 3);

      // obstacles
      z_obs_ = opti_->parameter(params_.max_obstacles, 5);
      i_obs_ = z_obs_(slice_all_, 0);
      h_obs_ = z_obs_(slice_all_, 1);
      a_obs_ = z_obs_(slice_all_, 2);
      b_obs_ = z_obs_(slice_all_, 3);
      angle_obs_ = z_obs_(slice_all_, 4);

      // Decision vars
      z_dv_ = opti_->variable(params_.N + 1, 4);
      x_dv_ = z_dv_(slice_all_, 0);
      y_dv_ = z_dv_(slice_all_, 1);
      psi_dv_ = z_dv_(slice_all_, 2);
      v_dv_ = z_dv_(slice_all_, 3);

      // Control inputs used to achieve z_dv
      u_dv_ = opti_->variable(params_.N, 2);
      acc_dv_ = u_dv_(slice_all_, 0);
      df_dv_ = u_dv_(slice_all_, 1);

      // Slack variables used to relax input rate constraints.
      sl_dv_ = opti_->variable(params_.N, 2);
      sl_acc_dv_ = sl_dv_(slice_all_, 0);
      sl_df_dv_ = sl_dv_(slice_all_, 1);

      // Add cost and constraints to optimal control problem
      addConstraints();
      addCost();

      // Kind of a test , set all current states to zero
      vox_nav_control::common::States curr_states;
      updateCurrentStates(curr_states);

      // Kind of a test, set test obstacles
      vox_nav_control::common::Ellipsoid mock_obstacle;
      std::vector<vox_nav_control::common::Ellipsoid> mock_obstacles(params_.max_obstacles,
        mock_obstacle);
      updateObstacles(mock_obstacles);

      // Kind of a test, set reference states in horizon to zeros
      vox_nav_control::common::States mock_reference_state;
      std::vector<vox_nav_control::common::States> mock_reference_states(params_.N,
        mock_reference_state);
      updateReferences(mock_reference_states);

      // Initialize actual states in horizon and set them all to zeros
      vox_nav_control::common::States initial_actual_state;
      std::vector<vox_nav_control::common::States> initial_actual_states(params_.N + 1,
        initial_actual_state);
      initializeActualStates(initial_actual_states);

      // Kind of a test, just set previous control inputs to zeros
      vox_nav_control::common::ControlInput previous_control_input;
      updatePreviousControlInput(previous_control_input);

      // initiazlize actual control inputs , all zeros
      vox_nav_control::common::ControlInput initial_actual_control_input;
      std::vector<vox_nav_control::common::ControlInput> initial_actual_control_inputs(params_.N,
        initial_actual_control_input);
      initializeActualControlInputs(initial_actual_control_inputs);

      // Initialize slack variables, set them to zeros
      SlackVars slack_vars;
      std::vector<SlackVars> initial_slack_vars(params_.N, slack_vars);
      initializeSlackVars(initial_slack_vars);

      // set print level to zeros.
      casadi::Dict opts = {
        {"ipopt.print_level", 0},
        {"ipopt.max_cpu_time", 1.0},
        {"expand", true},
        {"print_time", false}};

      opti_->solver("ipopt", opts);

      // This needs to be called at least once , then we can retrieve opti->debug() information
      std::vector<vox_nav_control::common::Ellipsoid> no_obstackles;
      solve(no_obstackles);

      if (params_.debug_mode) {
        std::cout << "State Weight Matrix Q: \n" << Q << std::endl;
        std::cout << "Control Weight Matrix R: \n" << R << std::endl;
        std::cout << "Initial States z_curr_: \n" << opti_->debug().value(z_curr_) << std::endl;
        std::cout << "Initial Refernce Traj states z_ref_: \n" << opti_->debug().value(z_ref_) <<
          std::endl;
        std::cout << "Initial Obstacles z_obs_: \n" << opti_->debug().value(z_obs_) <<
          std::endl;
        std::cout << "Initial Actual Traj states z_dv_: \n" << opti_->debug().value(z_dv_) <<
          std::endl;
        std::cout << "Initial Control inputs u_dv_: \n" << opti_->debug().value(u_dv_) << std::endl;
        std::cout << "Previous Control inputs u_prev_: \n" << opti_->debug().value(u_prev_) <<
          std::endl;
        std::cout << "Initial Slack Vars sl_dv_: \n" << opti_->debug().value(sl_dv_) << std::endl;
        std::cout << "Constructed MPC controller instance." << std::endl;
      }
      if (!params_.params_configured) {
        std::cerr <<
          "MPC Controller parameters are not Configured !!!, Using default parameters. " <<
          std::endl;
      }
    }

    MPCControllerCasadiCore::~MPCControllerCasadiCore()
    {
    }

    void MPCControllerCasadiCore::addConstraints()
    {
      //  State Bound Constraints
      opti_->subject_to(opti_->bounded(params_.V_MIN, v_dv_, params_.V_MAX));


      // state dynamics constraints
      // see here for the equtions: https://github.com/MPC-Berkeley/barc/wiki/Car-Model
      for (int i = 0; i < params_.N; i++) {
        // Slip angle

        //FULL ACKERMAN MODEL
        auto beta =
          casadi::MX::atan(
          params_.L_R / (params_.L_F + params_.L_R) *
          casadi::MX::tan(df_dv_(i)));
        opti_->subject_to(
          x_dv_(i + 1) ==
          x_dv_(i) + params_.DT * (v_dv_(i) * casadi::MX::cos(psi_dv_(i) + beta)));
        opti_->subject_to(
          y_dv_(i + 1) ==
          y_dv_(i) + params_.DT * (v_dv_(i) * casadi::MX::sin(psi_dv_(i) + beta)));
        opti_->subject_to(
          psi_dv_(i + 1) ==
          psi_dv_(i) + params_.DT * (v_dv_(i) / params_.L_R * casadi::MX::sin(beta)));
        opti_->subject_to(
          v_dv_(i + 1) ==
          v_dv_(i) + params_.DT * acc_dv_(i));

      }

      //  Input Bound Constraints
      opti_->subject_to(opti_->bounded(params_.A_MIN, acc_dv_, params_.A_MAX));
      opti_->subject_to(opti_->bounded(params_.DF_MIN, df_dv_, params_.DF_MAX));

      opti_->subject_to(x_dv_(0) == z_curr_(0));
      opti_->subject_to(y_dv_(0) == z_curr_(1));
      opti_->subject_to(psi_dv_(0) == z_curr_(2));
      opti_->subject_to(v_dv_(0) == z_curr_(3));

      //  Input Rate Bound Constraints
      opti_->subject_to(
        opti_->bounded(
          params_.A_DOT_MIN * params_.DT - sl_acc_dv_(0),
          acc_dv_(0) - u_prev_(0),
          params_.A_DOT_MAX * params_.DT + sl_acc_dv_(0)));

      opti_->subject_to(
        opti_->bounded(
          params_.DF_DOT_MIN * params_.DT - sl_df_dv_(0),
          df_dv_(0) - u_prev_(1),
          params_.DF_DOT_MAX * params_.DT + sl_df_dv_(0)));

      for (int i = 0; i < params_.N - 1; i++) {
        opti_->subject_to(
          opti_->bounded(
            params_.A_DOT_MIN * params_.DT - sl_acc_dv_(i + 1),
            acc_dv_(i + 1) - acc_dv_(i),
            params_.A_DOT_MAX * params_.DT + sl_acc_dv_(i + 1)));

        opti_->subject_to(
          opti_->bounded(
            params_.DF_DOT_MIN * params_.DT - sl_df_dv_(i + 1),
            df_dv_(i + 1) - df_dv_(i),
            params_.DF_DOT_MAX * params_.DT + sl_df_dv_(i + 1)));
      }

      //  Other Constraints
      opti_->subject_to(0 <= sl_df_dv_);
      opti_->subject_to(0 <= sl_acc_dv_);


    }

    void MPCControllerCasadiCore::addCost()
    {
      auto quad_form = [](casadi::MX z, casadi::Matrix<double> P)
        {
          return casadi::MX::mtimes(z, casadi::MX::mtimes(P, z.T()));
        };
      casadi::MX cost = 0.0;

      // tracking cost cost
      for (int i = 0; i < params_.N; i++) {
        cost += quad_form(z_dv_(i + 1, slice_all_) - z_ref_(i, slice_all_), Q);
      }
      // input derivative cost
      for (int i = 0; i < params_.N - 1; i++) {
        cost += quad_form(u_dv_(i + 1, slice_all_) - u_dv_(i, slice_all_), R);
      }
      // slack cost
      cost += (casadi::MX::sum1(sl_df_dv_) + casadi::MX::sum1(sl_acc_dv_));

      auto casadi_hypot = [](casadi::MX & a, casadi::MX & b) {
          return casadi::MX::sqrt(casadi::MX::pow(a, 2) + casadi::MX::pow(b, 2));
        };

      auto casadi_min = [](casadi::MX & a, casadi::MX & b) {
          return casadi::MX::if_else((a < b), a, b);
        };

      auto casadi_max = [](casadi::MX & a, casadi::MX & b) {
          return casadi::MX::if_else((a > b), a, b);
        };

      casadi::MX obstacle_cost = 0.0;
      for (int o = 0; o < params_.max_obstacles; o++) {
        auto a = a_obs_(o) / 2.0;
        auto b = b_obs_(o) / 2.0;
        auto i = i_obs_(o);
        auto h = h_obs_(o);

        auto is_inside =
          casadi::MX::pow((x_dv_ - i), 2) / casadi::MX::pow(a + params_.robot_radius, 2) +
          casadi::MX::pow((y_dv_ - h), 2) / casadi::MX::pow(b + params_.robot_radius, 2);

        obstacle_cost += casadi::MX::sum1(casadi::MX::exp(1.0 / is_inside));

      }
      cost += params_.obstacle_cost * obstacle_cost;

      opti_->minimize(cost);
    }


    MPCControllerCasadiCore::SolutionResult MPCControllerCasadiCore::solve(
      const std::vector<vox_nav_control::common::Ellipsoid> & obstacles)
    {
      bool is_solution_optimal(false);

      auto start = std::chrono::high_resolution_clock::now();
      casadi::native_DM u_mpc, z_mpc, sl_mpc, z_ref;
      try {
        auto sol = opti_->solve();
        u_mpc = sol.value(u_dv_);
        z_mpc = sol.value(z_dv_);
        sl_mpc = sol.value(sl_dv_);
        z_ref = sol.value(z_ref_);
        is_solution_optimal = true;
        initial_solution_found_ = true;
      } catch (const std::exception & e) {
        std::cerr << "MPC CONTROL: NON OPTIMAL SOLUTION FOUND!" << '\n';
        u_mpc = opti_->debug().value(u_dv_);
        z_mpc = opti_->debug().value(z_dv_);
        sl_mpc = opti_->debug().value(sl_dv_);
        z_ref = opti_->debug().value(z_ref_);
      }
      auto end = std::chrono::high_resolution_clock::now();
      auto execution_time =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

      // get actual_computed_states
      std::vector<vox_nav_control::common::States> actual_computed_states;
      for (int i = 0; i < params_.N + 1; i++) {
        vox_nav_control::common::States ith_computed_state;
        ith_computed_state.x = z_mpc(i, 0).scalar();
        ith_computed_state.y = z_mpc(i, 1).scalar();
        ith_computed_state.psi = z_mpc(i, 2).scalar();
        ith_computed_state.v = z_mpc(i, 3).scalar();
        actual_computed_states.push_back(ith_computed_state);
      }

      vox_nav_control::common::ControlInput control_input;
      control_input.acc = u_mpc(0, 0).scalar();
      control_input.df = u_mpc(0, 1).scalar();

      SolutionResult result;
      result.solve_time_ms = execution_time;
      result.is_optimal = is_solution_optimal;
      result.control_input = control_input;
      result.actual_computed_states = actual_computed_states;

      if (params_.debug_mode) {
        std::cout << "Current States z_curr_: \n " << opti_->debug().value(z_curr_) << std::endl;
        std::cout << "Refernce Traj states z_ref_: \n" << opti_->debug().value(z_ref_) <<
          std::endl;
        std::cout << "Actual Traj states z_dv_: \n" << opti_->debug().value(z_dv_) << std::endl;
        std::cout << "Control inputs u_dv_: \n" << opti_->debug().value(u_dv_) << std::endl;
        std::cout << "Previous Control inputs u_prev_: \n" << opti_->debug().value(u_prev_) <<
          std::endl;
        std::cout << "Solve took: " << execution_time << " ms" << std::endl;
        std::cout << "acceleration cmd " << result.control_input.acc << std::endl;
        std::cout << "steering angle " << result.control_input.df << std::endl;
      }

      return result;
    }

    void MPCControllerCasadiCore::updateCurrentStates(vox_nav_control::common::States curr_states)
    {
      opti_->set_value(z_curr_, {curr_states.x, curr_states.y, curr_states.psi, curr_states.v});
    }

    void MPCControllerCasadiCore::updateReferences(
      std::vector<vox_nav_control::common::States> reference_states)
    {
      std::vector<double> x_ref, y_ref, psi_ref, v_ref;
      for (int i = 0; i < reference_states.size(); i++) {
        x_ref.push_back(reference_states[i].x);
        y_ref.push_back(reference_states[i].y);
        psi_ref.push_back(reference_states[i].psi);
        v_ref.push_back(reference_states[i].v);
      }
      opti_->set_value(x_ref_, x_ref);
      opti_->set_value(y_ref_, y_ref);
      opti_->set_value(psi_ref_, psi_ref);
      opti_->set_value(v_ref_, v_ref);
    }

    void MPCControllerCasadiCore::updateObstacles(
      std::vector<vox_nav_control::common::Ellipsoid> obstacles)
    {
      std::vector<double> i, h, a, b, angle;
      for (int o = 0; o < obstacles.size(); o++) {
        i.push_back(obstacles[o].center.x());
        h.push_back(obstacles[o].center.y());
        a.push_back(obstacles[o].axes.x());
        b.push_back(obstacles[o].axes.y());
        angle.push_back(obstacles[o].heading_to_robot_angle);

      }
      opti_->set_value(i_obs_, i);
      opti_->set_value(h_obs_, h);
      opti_->set_value(a_obs_, a);
      opti_->set_value(b_obs_, b);
      opti_->set_value(angle_obs_, angle);
    }

    void MPCControllerCasadiCore::updatePreviousControlInput(
      vox_nav_control::common::ControlInput previous_control_input)
    {
      opti_->set_value(u_prev_, {previous_control_input.acc, previous_control_input.df});
    }

    void MPCControllerCasadiCore::initializeSlackVars(
      std::vector<SlackVars> initial_slack_vars)
    {
      std::vector<double> sl_acc_dv, sl_df_dv;
      for (int i = 0; i < initial_slack_vars.size(); i++) {
        sl_acc_dv.push_back(initial_slack_vars[i].sl_acc);
        sl_df_dv.push_back(initial_slack_vars[i].sl_df);
      }
      opti_->set_initial(sl_dv_(slice_all_, 0), sl_acc_dv);
      opti_->set_initial(sl_dv_(slice_all_, 1), sl_df_dv);
    }

    void MPCControllerCasadiCore::initializeActualControlInputs(
      std::vector<vox_nav_control::common::ControlInput> initial_actual_control_inputs)
    {
      std::vector<double> initial_acc_dv, initial_df_dv;
      for (int i = 0; i < initial_actual_control_inputs.size(); i++) {
        initial_acc_dv.push_back(initial_actual_control_inputs[i].acc);
        initial_df_dv.push_back(initial_actual_control_inputs[i].df);
      }
      opti_->set_initial(u_dv_(slice_all_, 0), initial_acc_dv);
      opti_->set_initial(u_dv_(slice_all_, 1), initial_df_dv);
    }

    void MPCControllerCasadiCore::initializeActualStates(
      std::vector<vox_nav_control::common::States> initial_actual_states)
    {
      std::vector<double> x_dv, y_dv, psi_dv, v_dv;
      for (int i = 0; i < initial_actual_states.size(); i++) {
        x_dv.push_back(initial_actual_states[i].x);
        y_dv.push_back(initial_actual_states[i].y);
        psi_dv.push_back(initial_actual_states[i].psi);
        v_dv.push_back(initial_actual_states[i].v);
      }
      opti_->set_initial(x_dv_, x_dv);
      opti_->set_initial(y_dv_, y_dv);
      opti_->set_initial(psi_dv_, psi_dv);
      opti_->set_initial(v_dv_, v_dv);
    }

  }   // namespace mpc_controller_casadi
}   // namespace vox_nav_control
