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

#ifndef VOX_NAV_PLANNING__RRT__LQRPLANNER_HPP_
#define VOX_NAV_PLANNING__RRT__LQRPLANNER_HPP_

#include "vox_nav_planning/planner_core.hpp"
#include "vox_nav_utilities/elevation_state_space.hpp"
#include "nav_msgs/msg/path.hpp"

#include "ompl/control/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/base/spaces/SE2StateSpace.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/objectives/MinimaxObjective.h"
#include "ompl/base/objectives/MaximizeMinClearanceObjective.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/base/objectives/MechanicalWorkOptimizationObjective.h"
#include "ompl/tools/config/SelfConfig.h"

#include <limits>

namespace ompl
{
  namespace control
  {

    class LQRPlanner : public base::Planner
    {
    public:
      /** \brief Constructor */
      LQRPlanner(const SpaceInformationPtr & si);

      ~LQRPlanner() override;

      void setup() override;

      /** \brief Continue solving for some amount of time. Return true if solution was found. */
      base::PlannerStatus solve(const base::PlannerTerminationCondition & ptc) override;

      void getPlannerData(base::PlannerData & data) const override;

      /** \brief Clear datastructures. Call this function if the
          input data to the planner has changed and you do not
          want to continue planning */
      void clear() override;

    protected:
      /** \brief Free the memory allocated by this planner */
      void freeMemory();

      /** \brief State sampler */
      base::StateSamplerPtr sampler_;

      const SpaceInformation * siC_;

      rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rrt_nodes_pub_;

      rclcpp::Node::SharedPtr node_;

      double dt_{0.05};

      double max_time_{5.0};

      std::tuple<double, double> lqr_control(
        const Eigen::MatrixXd & A,
        const Eigen::MatrixXd & B,
        const Eigen::MatrixXd & X)
      {
        auto res = dlqr(
          A, B,
          Eigen::MatrixXd::Identity(2, 2),
          Eigen::MatrixXd::Identity(1, 1));

        auto K = std::get<0>(res);

        double phi = (K * X)(0);

        phi = std::clamp<double>(phi, -0.6, 0.6);
        double v_r = 1.5;

        return std::make_tuple(v_r, phi);
      }


      /**
       * @brief authored by: Horibe Takamasa; https://github.com/TakaHoribe
       *        DisContinous time Riccati Eq. solver
       */
      bool solve_dare(
        const Eigen::MatrixXd & Ad,
        const Eigen::MatrixXd & Bd,
        const Eigen::MatrixXd & Q,
        const Eigen::MatrixXd & R,
        Eigen::MatrixXd & P,
        const double & tolerance = 0.01,
        const uint iter_max = 150)
      {
        P = Q; // initialize
        Eigen::MatrixXd P_next;
        Eigen::MatrixXd AdT = Ad.transpose();
        Eigen::MatrixXd BdT = Bd.transpose();
        Eigen::MatrixXd Rinv = R.inverse();
        double diff;
        for (uint i = 0; i < iter_max; ++i) {
          // -- discrete solver --
          P_next = AdT * P * Ad -
            AdT * P * Bd * (R + BdT * P * Bd).inverse() * BdT * P * Ad + Q;

          diff = fabs((P_next - P).maxCoeff());
          P = P_next;
          if (diff < tolerance) {
            std::cout << "iteration mumber = " << i << std::endl;
            return true;
          }
        }
        return false; // over iteration limit
      }

      /**
       * @brief
       * Solve the discrete time lqr controller.
       * x[k+1] = A x[k] + B u[k]
       * cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
       * ref Bertsekas, p.151

       * @param Ad
       * @param Bd
       * @param Q
       * @param R
       * @return std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::VectorXcd>, K, X, EIGENVALUES
       */
      std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::VectorXcd> dlqr(
        const Eigen::MatrixXd & Ad,
        const Eigen::MatrixXd & Bd,
        const Eigen::MatrixXd & Q,
        const Eigen::MatrixXd & R)
      {
        Eigen::MatrixXd X, K;
        bool solved_dare = solve_dare(Ad, Bd, Q, R, X);
        K = (Bd.transpose() * X * Bd + R).inverse() * (Bd.transpose() * X * Ad);
        auto eig = (Ad - Bd * K).eigenvalues();
        return std::make_tuple(K, X, eig);
      }
    };

  }
}

#endif  // VOX_NAV_PLANNING__RRT__LQRPLANNER_HPP_
