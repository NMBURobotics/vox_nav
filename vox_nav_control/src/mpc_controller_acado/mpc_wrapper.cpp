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

#include <vox_nav_control/mpc_controller_acado/mpc_wrapper.hpp>

/* Global variables used by the solver. */
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

namespace vox_nav_control
{
  namespace mpc_controller_acado
  {
    template<typename T>
    MPCWrapper<T>::MPCWrapper()
    : acado_reference_states_{acadoVariables.y},
      acado_reference_end_state_{acadoVariables.yN},
      acado_initial_state_{acadoVariables.x0},
      acado_states_{acadoVariables.x},
      acado_inputs_{acadoVariables.u},
      acado_online_data_{acadoVariables.od},
      acado_W_{acadoVariables.W},
      acado_W_end_{acadoVariables.WN},
      acado_lower_bounds_{acadoVariables.lbValues},
      acado_upper_bounds_{acadoVariables.ubValues}
    {
      memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
      memset(&acadoVariables, 0, sizeof( acadoVariables ));

      acado_initializeSolver();

      // Initialize the states
      const Eigen::Matrix<T, kStateSize, 1> initial_states =
        (Eigen::Matrix<T, kStateSize, 1>() <<
        0.0, 0.0, 0.0, 0.0).finished();

      // Initialize states x and xN and input u.
      acado_initial_state_ = initial_states.template cast<float>();

      acado_states_ = initial_states.replicate(1, kSamples + 1).template cast<float>();

      acado_inputs_ = kZeroInput_.replicate(1, kSamples).template cast<float>();

      // Initialize references y and yN.
      acado_reference_states_.block(0, 0, kStateSize, kSamples) =
        initial_states.replicate(1, kSamples).template cast<float>();

      acado_reference_states_.block(kStateSize, 0, kCostSize - kStateSize, kSamples) =
        Eigen::Matrix<float, kCostSize - kStateSize, kSamples>::Zero();

      acado_reference_states_.block(kCostSize, 0, kInputSize, kSamples) =
        kZeroInput_.replicate(1, kSamples);

      acado_reference_end_state_.segment(0, kStateSize) =
        initial_states.template cast<float>();

      acado_reference_end_state_.segment(kStateSize, kCostSize - kStateSize) =
        Eigen::Matrix<float, kCostSize - kStateSize, 1>::Zero();

      // Initialize Cost matrix W and WN.
      if (!(acado_W_.trace() > 0.0)) {
        acado_W_ = W_.replicate(1, kSamples).template cast<float>();
        acado_W_end_ = WN_.template cast<float>();
      }

      // Initialize online data.
      Eigen::Matrix<T, 3, 1> p_B_C(0, 0, 0);
      Eigen::Quaternion<T> q_B_C(1, 0, 0, 0);
      Eigen::Matrix<T, 3, 1> point_of_interest(0, 0, -1000);

      // Initialize solver.
      acado_initializeNodesByForwardSimulation();
      acado_preparationStep();
      acado_is_prepared_ = true;
    }

    template<typename T>
    MPCWrapper<T>::~MPCWrapper()
    {
    }

    // Constructor with cost matrices as arguments.
    template<typename T>
    MPCWrapper<T>::MPCWrapper(
      const Eigen::Ref<const Eigen::Matrix<T, kCostSize, kCostSize>> Q,
      const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kInputSize>> R)
    {
      setCosts(Q, R);
      MPCWrapper();
    }

    // Set cost matrices with optional scaling.
    template<typename T>
    bool MPCWrapper<T>::setCosts(
      const Eigen::Ref<const Eigen::Matrix<T, kCostSize, kCostSize>> Q,
      const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kInputSize>> R,
      const T state_cost_scaling, const T input_cost_scaling)
    {
      if (state_cost_scaling < 0.0 || input_cost_scaling < 0.0) {
        std::cerr << ("MPC: Cost scaling is wrong, must be non-negative!");
        return false;
      }
      W_.block(0, 0, kCostSize, kCostSize) = Q;
      W_.block(kCostSize, kCostSize, kInputSize, kInputSize) = R;
      WN_ = W_.block(0, 0, kCostSize, kCostSize);

      float state_scale{1.0};
      float input_scale{1.0};
      for (int i = 0; i < kSamples; i++) {
        state_scale = exp(
          -float(i) / float(kSamples) *
          float(state_cost_scaling));
        input_scale = exp(
          -float(i) / float(kSamples) *
          float(input_cost_scaling));
        acado_W_.block(0, i * kRefSize, kCostSize, kCostSize) =
          W_.block(0, 0, kCostSize, kCostSize).template cast<float>() *
          state_scale;
        acado_W_.block(kCostSize, i * kRefSize + kCostSize, kInputSize, kInputSize) =
          W_.block(
          kCostSize, kCostSize, kInputSize, kInputSize
          ).template cast<float>() * input_scale;
      }
      acado_W_end_ = WN_.template cast<float>() * state_scale;

      return true;
    }

    template<typename T>
    bool MPCWrapper<T>::setLimits(
      T min_v_dv, T max_v_dv,
      T min_acc_dv, T max_acc_dv,
      T min_df_dv, T max_df_dv)
    {
      // Set input boundaries.
      Eigen::Matrix<T, 3, 1> lower_bounds = Eigen::Matrix<T, 3, 1>::Zero();
      Eigen::Matrix<T, 3, 1> upper_bounds = Eigen::Matrix<T, 3, 1>::Zero();
      lower_bounds <<
        min_v_dv, min_acc_dv, min_df_dv;
      upper_bounds <<
        max_v_dv, max_acc_dv, max_df_dv;

      acado_lower_bounds_ =
        lower_bounds.replicate(1, kSamples).template cast<float>();

      acado_upper_bounds_ =
        upper_bounds.replicate(1, kSamples).template cast<float>();
      return true;
    }

    // Set camera extrinsics.
    template<typename T>
    bool MPCWrapper<T>::setOnlineData(
      const Eigen::Ref<const Eigen::Matrix<T, 3, 1>> & obs)
    {
      acado_online_data_.block(0, 0, 3, ACADO_N + 1) =
        obs.replicate(1, ACADO_N + 1).template cast<float>();
      return true;
    }

    // Set a reference pose.
    template<typename T>
    bool MPCWrapper<T>::setReferencePose(
      const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state)
    {
      acado_reference_states_.block(0, 0, kStateSize, kSamples) =
        state.replicate(1, kSamples).template cast<float>();

      acado_reference_states_.block(kStateSize, 0, kCostSize - kStateSize, kSamples) =
        Eigen::Matrix<float, kCostSize - kStateSize, kSamples>::Zero();

      acado_reference_states_.block(kCostSize, 0, kInputSize, kSamples) =
        kZeroInput_.replicate(1, kSamples);

      acado_reference_end_state_.segment(0, kStateSize) =
        state.template cast<float>();

      acado_reference_end_state_.segment(kStateSize, kCostSize - kStateSize) =
        Eigen::Matrix<float, kCostSize - kStateSize, 1>::Zero();

      acado_initializeNodesByForwardSimulation();
      return true;
    }

    // Set a reference trajectory.
    template<typename T>
    bool MPCWrapper<T>::setTrajectory(
      const Eigen::Ref<const Eigen::Matrix<T, kStateSize, kSamples + 1>> states,
      const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kSamples + 1>> inputs)
    {
      Eigen::Map<Eigen::Matrix<float, kRefSize, kSamples, Eigen::ColMajor>>
      y(const_cast<float *>(acadoVariables.y));

      acado_reference_states_.block(0, 0, kStateSize, kSamples) =
        states.block(0, 0, kStateSize, kSamples).template cast<float>();

      acado_reference_states_.block(kStateSize, 0, kCostSize - kStateSize, kSamples) =
        Eigen::Matrix<float, kCostSize - kStateSize, kSamples>::Zero();

      acado_reference_states_.block(kCostSize, 0, kInputSize, kSamples) =
        inputs.block(0, 0, kInputSize, kSamples).template cast<float>();

      acado_reference_end_state_.segment(0, kStateSize) =
        states.col(kSamples).template cast<float>();

      acado_reference_end_state_.segment(kStateSize, kCostSize - kStateSize) =
        Eigen::Matrix<float, kCostSize - kStateSize, 1>::Zero();

      return true;
    }

    // Reset states and inputs and calculate new solution.
    template<typename T>
    bool MPCWrapper<T>::solve(
      const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state)
    {
      acado_states_ = state.replicate(1, kSamples + 1).template cast<float>();

      acado_inputs_ = kZeroInput_.replicate(1, kSamples);

      return update(state);
    }


// Calculate new solution from last known solution.
    template<typename T>
    bool MPCWrapper<T>::update(
      const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state,
      bool do_preparation)
    {
      if (!acado_is_prepared_) {
        std::cout << ("MPC: Solver was triggered without preparation, abort!");
        return false;
      }

      // Check if estimated and reference quaternion live in sthe same hemisphere.
      acado_initial_state_ = state.template cast<float>();

      /*if (acado_initial_state_.segment(3, 4).dot(
          Eigen::Vector4f(acado_reference_states_.block(3, 0, 4, 1))) < (T)0.0)
      {
        acado_initial_state_.segment(3, 4) = -acado_initial_state_.segment(3, 4);
      }*/

      // Perform feedback step and reset preparation check.
      acado_feedbackStep();
      acado_is_prepared_ = false;

      // Prepare if the solver if wanted
      if (do_preparation) {
        acado_preparationStep();
        acado_is_prepared_ = true;
      }

      return true;
    }

    // Prepare the solver.
    template<typename T>
    bool MPCWrapper<T>::prepare()
    {
      acado_preparationStep();
      acado_is_prepared_ = true;

      return true;
    }

    template<typename T>
    void MPCWrapper<T>::getState(
      const int node_index,
      Eigen::Ref<Eigen::Matrix<T, kStateSize, 1>> return_state)
    {
      return_state = acado_states_.col(node_index).cast<T>();
    }

    template<typename T>
    void MPCWrapper<T>::getStates(
      Eigen::Ref<Eigen::Matrix<T, kStateSize, kSamples + 1>> return_states)
    {
      return_states = acado_states_.cast<T>();
    }

    template<typename T>
    void MPCWrapper<T>::getInput(
      const int node_index,
      Eigen::Ref<Eigen::Matrix<T, kInputSize, 1>> return_input)
    {
      return_input = acado_inputs_.col(node_index).cast<T>();
    }

    template<typename T>
    void MPCWrapper<T>::getInputs(
      Eigen::Ref<Eigen::Matrix<T, kInputSize, kSamples>> return_inputs)
    {
      return_inputs = acado_inputs_.cast<T>();
    }

    template class MPCWrapper<float>;
    template class MPCWrapper<double>;

  }  // namespace mpc_controller_acado
}   // namespace vox_nav_control
