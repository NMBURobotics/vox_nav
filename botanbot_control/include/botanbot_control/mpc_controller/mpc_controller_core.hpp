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

#ifndef BOTANBOT_CONTROL__MPC_CONTROLLER__MPC_CONTROLLER_CORE_HPP_
#define BOTANBOT_CONTROL__MPC_CONTROLLER__MPC_CONTROLLER_CORE_HPP_

#include <casadi/casadi.hpp>

#include <vector>
#include <memory>
#include <chrono>

namespace botanbot_control
{
namespace mpc_controller
{

class MPCControllerCore
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
   * @brief a struct to keep states organized
   *
   */
  struct States
  {
    double x;     // X position
    double y;     // Y position
    double psi;   // heading angle
    double v;     // linear velocity
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
    double df;    // steering angle command
    ControlInput()
    : acc(0.0),
      df(0.0) {}
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
    ControlInput control_input;
    // actual states that are going to be achieved in the time horizon
    std::vector<States> actual_computed_states;
    SolutionResult()
    : is_optimal(false),
      solve_time_ms(0)
    {}
  };

  /**
   * @brief all parameters usd by MPC class,
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
    // weights on jerk and slew rate(steering angle derivative)
    std::vector<double> R;
    // enable/disable debug messages
    bool debug_mode;
    // set this true only if user figured the configration
    bool params_configured;

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
      params_configured(false) {}
  };

  /**
   * @brief Construct a new MPCControllerCore object
   *
   * @param params
   */
  MPCControllerCore(Parameters params);

  /**
   * @brief Destroy the MPCControllerCore object
   *
   */
  ~MPCControllerCore();

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
  void updateCurrentStates(States curr_states);

  /**
   * @brief update current refernce states in horizon
   *
   * @param reference_states
   */
  void updateReferences(std::vector<States> reference_states);

  /**
   * @brief update previous control input
   *
   * @param previous_control_input
   */
  void updatePreviousControlInput(ControlInput previous_control_input);

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
  void initializeActualStates(std::vector<States> initial_actual_states);

  /**
 * @brief  initialize control inputs, mostly just equalizing them to zero. These cannot be updated
 *  they are variables of optimal control problem itself
 *
 * @param initial_actual_control_inputs
 */
  void initializeActualControlInputs(std::vector<ControlInput> initial_actual_control_inputs);

  /**
   * @brief solve the actual optiml control problem
   *
   * @return SolutionResult, includes resulting control inputs
   * as well as optimal control variables; computed actual states
   */
  SolutionResult solve();

private:
  std::shared_ptr<casadi::Opti> opti_;
  // used to slice casadi matrixes
  casadi::Slice slice_all_;
  casadi::Matrix<double> Q;
  casadi::Matrix<double> R;

  casadi::MX u_prev_;   // previous input: [u_{acc, -1}, u_{df, -1}]
  casadi::MX z_curr_;   // current state:  [x_0, y_0, psi_0, v_0]

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
  Parameters params_;
};
}  // namespace mpc_controller
}  // namespace botanbot_control

#endif  // BOTANBOT_CONTROL__MPC_CONTROLLER__MPC_CONTROLLER_CORE_HPP_
