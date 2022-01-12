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

#ifndef VOX_NAV_CONTROL__MPC_CONTROLLER_ACADO__MPC_WRAPPER_HPP_
#define VOX_NAV_CONTROL__MPC_CONTROLLER_ACADO__MPC_WRAPPER_HPP_


#include <Eigen/Eigen>
#include <iostream>
#include <vox_nav_control/mpc_controller_acado/auto_gen/acado_common.h>
#include <vox_nav_control/mpc_controller_acado/auto_gen/acado_auxiliary_functions.h>

namespace vox_nav_control
{
  namespace mpc_controller_acado
  {

    static constexpr int kSamples = ACADO_N;  // number of samples
    static constexpr int kStateSize = ACADO_NX; // number of states
    static constexpr int kRefSize = ACADO_NY; // number of reference states
    static constexpr int kEndRefSize = ACADO_NYN; // number of end reference states
    static constexpr int kInputSize = ACADO_NU; // number of inputs
    static constexpr int kCostSize = ACADO_NY - ACADO_NU; // number of state costs
    static constexpr int kOdSize = ACADO_NOD; // number of online data

    template<typename T>
    class MPCWrapper
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      /**
       * @brief Construct a new MPCWrapper object
       *
       */
      MPCWrapper();

      /**
       * @brief Construct a new MPCWrapper object with weights
       *
       * @param Q
       * @param R
       */
      MPCWrapper(
        const Eigen::Ref<const Eigen::Matrix<T, kCostSize, kCostSize>> Q,
        const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kInputSize>> R);

      /**
       * @brief Set the Costs object
       *
       * @param Q
       * @param R
       * @param state_cost_scaling
       * @param input_cost_scaling
       * @return true
       * @return false
       */
      bool setCosts(
        const Eigen::Ref<const Eigen::Matrix<T, kCostSize, kCostSize>> Q,
        const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kInputSize>> R,
        const T state_cost_scaling = 0.0, const T input_cost_scaling = 0.0);

      /**
       * @brief Destroy the MPCWrapper object
       *
       */
      ~MPCWrapper();

      /**
       * @brief Set the Limits object
       *
       * @param min_v_dv
       * @param max_v_dv
       * @param min_acc_dv
       * @param max_acc_dv
       * @param min_df_dv
       * @param max_df_dv
       * @return true
       * @return false
       */
      bool setLimits(
        T min_v_dv, T max_v_dv,
        T min_acc_dv, T max_acc_dv,
        T min_df_dv, T max_df_dv);

      /**
       * @brief Set the Online Data object, Where these is obstacles
       *
       * @param obs
       * @return true
       * @return false
       */
      bool setOnlineData(const Eigen::Ref<const Eigen::Matrix<T, 3, 1>> & obs);

      /**
       * @brief Set the Reference Pose object
       * Fill all refernce states by a single state
       *
       * @param state
       * @return true
       * @return false
       */
      bool setReferencePose(
        const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state);

      /**
       * @brief Set the Trajectory object
       *
       * @param states
       * @param inputs
       * @return true
       * @return false
       */
      bool setTrajectory(
        const Eigen::Ref<const Eigen::Matrix<T, kStateSize, kSamples + 1>> states,
        const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kSamples + 1>> inputs);

      /**
       * @brief RUN mpc cycle
       *
       * @param state
       * @return true
       * @return false
       */
      bool solve(const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state);

      /**
       * @brief Get the feedback for the next cycle
       *
       * @param state
       * @param do_preparation
       * @return true
       * @return false
       */
      bool update(
        const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state,
        bool do_preparation = true);

      /**
       * @brief Acado specific call for acado_preparationStep()
       *
       * @return true
       * @return false
       */
      bool prepare();

      /**
       * @brief Get the State object
       *
       * @param node_index
       * @param return_state
       */
      void getState(
        const int node_index,
        Eigen::Ref<Eigen::Matrix<T, kStateSize, 1>> return_state);

      /**
       * @brief Get the States object
       *
       * @param return_states
       */
      void getStates(
        Eigen::Ref<Eigen::Matrix<T, kStateSize, kSamples + 1>> return_states);

      /**
       * @brief Get the Input object
       *
       * @param node_index
       * @param return_input
       */
      void getInput(
        const int node_index,
        Eigen::Ref<Eigen::Matrix<T, kInputSize, 1>> return_input);

      /**
       * @brief Get the Inputs object
       *
       * @param return_input
       */
      void getInputs(
        Eigen::Ref<Eigen::Matrix<T, kInputSize, kSamples>> return_input);

      /**
       * @brief Get the Timestep object, dt
       *
       * @return T
       */
      T getTimestep() {return dt_;}

    private:
      Eigen::Map<Eigen::Matrix<float, kRefSize, kSamples, Eigen::ColMajor>>
      acado_reference_states_{acadoVariables.y};

      Eigen::Map<Eigen::Matrix<float, kEndRefSize, 1, Eigen::ColMajor>>
      acado_reference_end_state_{acadoVariables.yN};

      Eigen::Map<Eigen::Matrix<float, kStateSize, 1, Eigen::ColMajor>>
      acado_initial_state_{acadoVariables.x0};

      Eigen::Map<Eigen::Matrix<float, kStateSize, kSamples + 1, Eigen::ColMajor>>
      acado_states_{acadoVariables.x};

      Eigen::Map<Eigen::Matrix<float, kInputSize, kSamples, Eigen::ColMajor>>
      acado_inputs_{acadoVariables.u};

      Eigen::Map<Eigen::Matrix<float, kOdSize, kSamples + 1, Eigen::ColMajor>>
      acado_online_data_{acadoVariables.od};

      Eigen::Map<Eigen::Matrix<float, kRefSize, kRefSize * kSamples>>
      acado_W_{acadoVariables.W};

      Eigen::Map<Eigen::Matrix<float, kEndRefSize, kEndRefSize>>
      acado_W_end_{acadoVariables.WN};

      Eigen::Map<Eigen::Matrix<float, 3, kSamples, Eigen::ColMajor>>
      acado_lower_bounds_{acadoVariables.lbValues};

      Eigen::Map<Eigen::Matrix<float, 3, kSamples, Eigen::ColMajor>>
      acado_upper_bounds_{acadoVariables.ubValues};

      Eigen::Matrix<T, kRefSize, kRefSize> W_ = (Eigen::Matrix<T, kRefSize, 1>() <<
        10 * Eigen::Matrix<T, 4, 1>::Ones(),
        100 * Eigen::Matrix<T, 2, 1>::Ones()).finished().asDiagonal();

      Eigen::Matrix<T, kEndRefSize, kEndRefSize> WN_ =
        W_.block(0, 0, kEndRefSize, kEndRefSize);

      bool acado_is_prepared_{false};
      const T dt_{0.1};

      const Eigen::Matrix<real_t, kInputSize, 1> kZeroInput_ =
        (Eigen::Matrix<real_t, kInputSize, 1>() << 0.0, 0.0).finished();
    };


  }  // namespace mpc_controller_acado
}   // namespace vox_nav_control

#endif  // VOX_NAV_CONTROL__MPC_CONTROLLER_ACADO__MPC_WRAPPER_HPP_
