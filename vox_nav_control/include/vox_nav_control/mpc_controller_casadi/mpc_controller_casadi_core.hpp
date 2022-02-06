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

// Equations and logic parts of this code was based on ;
// https://github.com/MPC-Berkeley/genesis_path_follower
// Also refer to;
// https://github.com/MPC-Berkeley/barc/wiki/Car-Model

#ifndef VOX_NAV_CONTROL__MPC_CONTROLLER__MPC_CONTROLLER_CORE_HPP_
#define VOX_NAV_CONTROL__MPC_CONTROLLER__MPC_CONTROLLER_CORE_HPP_

#include <casadi/casadi.hpp>

#include <vector>
#include <memory>
#include <chrono>
#include <Eigen/Eigen>
#include <vox_nav_control/common.hpp>

namespace vox_nav_control
{
  namespace mpc_controller_casadi
  {

/**
 * @brief CASADI based MOdel Predcitive Control for Ackermann Vehicle
 *
 */
    class MPCControllerCasadiCore
    {
    public:
      /**
       * @brief structure for slack variables, these are used to relax input rate constraints
       *
       */
      struct SlackVars
      {
        double sl_acc;
        double sl_df;
        SlackVars()
        : sl_acc(0.0),
          sl_df(0.0) {}
      };

      /**
       * @brief result is passed as this struct
       *
       */
      struct SolutionResult
      {
        // whether the found solution was optimal
        bool is_optimal;
        // Execution ime took by solver in milliseconds
        int solve_time_ms;
        // computed control input as result of optimal control
        vox_nav_control::common::ControlInput control_input;
        // actual states that are going to be achieved in the time horizon
        std::vector<vox_nav_control::common::States> actual_computed_states;
        SolutionResult()
        : is_optimal(false),
          solve_time_ms(0)
        {}
      };

      /**
       * @brief Construct a new MPCControllerCasadiCore object
       *
       * @param params
       */
      MPCControllerCasadiCore(vox_nav_control::common::Parameters params);

      /**
       * @brief Destroy the MPCControllerCasadiCore object
       *
       */
      ~MPCControllerCasadiCore();

      /**
       * @brief state dynamcs constraints, control input
       * constraints and boundries are defined through this function
       *
       */
      void addConstraints();

      /**
       * @brief objective function and cost is defined with this function
       *
       */
      void addCost();

      /**
       * @brief update the current states
       *
       * @param curr_states
       */
      void updateCurrentStates(vox_nav_control::common::States curr_states);

      /**
       * @brief update current refernce states in horizon
       *
       * @param reference_states
       */
      void updateReferences(std::vector<vox_nav_control::common::States> reference_states);

      /**
       * @brief update current obstacles in horizon
       *
       * @param obstacles
       */
      void updateObstacles(std::vector<vox_nav_control::common::Ellipsoid> obstacles);

      /**
       * @brief update previous control input
       *
       * @param previous_control_input
       */
      void updatePreviousControlInput(vox_nav_control::common::ControlInput previous_control_input);

      /**
       * @brief initialize slack variables, mostly just equalizing them to zero
       *
       * @param initial_slack_vars
       */
      void initializeSlackVars(std::vector<SlackVars> initial_slack_vars);

      /**
       * @brief  initialize actual states, mostly just equalizing them to zero.
       * These cannot be updated
       * they are variables of optimal control problem itself
       *
       * @param initial_actual_states
       */
      void initializeActualStates(
        std::vector<vox_nav_control::common::States> initial_actual_states);

      /**
     * @brief  initialize control inputs, mostly just equalizing them to zero. These cannot be updated
     *  they are variables of optimal control problem itself
     *
     * @param initial_actual_control_inputs
     */
      void initializeActualControlInputs(
        std::vector<vox_nav_control::common::ControlInput> initial_actual_control_inputs);

      /**
       * @brief solve the actual optiml control problem
       *
       * @return SolutionResult, includes resulting control inputs
       * as well as optimal control variables; computed actual states
       */
      SolutionResult solve(const std::vector<vox_nav_control::common::Ellipsoid> & obstacles);

    private:
      std::shared_ptr<casadi::Opti> opti_;
      // used to slice casadi matrixes
      casadi::Slice slice_all_;
      casadi::Matrix<double> Q;
      casadi::Matrix<double> R;

      casadi::MX u_prev_; // previous input: [u_{acc, -1}, u_{df, -1}]
      casadi::MX z_curr_; // current state:  [x_0, y_0, psi_0, v_0]

      // reference traj that we would like to follow
      // First index corresponds to our desired state at timestep k+1:
      // i.e. z_ref_[0,:] = z_{desired, 1}.
      // second index selects the state element from [x_k, y_k, psi_k, v_k].
      casadi::MX z_ref_;
      casadi::MX x_ref_;
      casadi::MX y_ref_;
      casadi::MX psi_ref_;
      casadi::MX v_ref_;

      // Actual trajectory we will follow given the optimal solution.
      // First index is the timestep k, i.e. self.z_dv[0,:] is z_0.
      // It has N+1 timesteps since we go from z_0, ..., z_N.
      // Second index is the state element, as detailed below.
      casadi::MX z_dv_;
      casadi::MX x_dv_;
      casadi::MX y_dv_;
      casadi::MX psi_dv_;
      casadi::MX v_dv_;

      // Control inputs used to achieve z_dv according to dynamics.
      // First index is the timestep k, i.e. self.u_dv[0,:] is u_0.
      // Second index is the input element as detailed below.
      casadi::MX u_dv_;
      casadi::MX acc_dv_;
      casadi::MX df_dv_;

      // Slack variables used to relax input rate constraints.
      casadi::MX sl_dv_;
      casadi::MX sl_acc_dv_;
      casadi::MX sl_df_dv_;

      // this will be updated through the constructor.
      vox_nav_control::common::Parameters params_;

      casadi::MX z_obs_;
      casadi::MX i_obs_;
      casadi::MX h_obs_;
      casadi::MX a_obs_;
      casadi::MX b_obs_;
      casadi::MX angle_obs_;
      bool initial_solution_found_;

    };
  } // namespace mpc_controller_casadi
}  // namespace vox_nav_control

#endif  // VOX_NAV_CONTROL__MPC_CONTROLLER__MPC_CONTROLLER_CORE_HPP_
