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
#include "ompl/base/spaces/SE2StateSpace.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/tools/config/SelfConfig.h"

#include <limits>

namespace ompl
{
  namespace control
  {

    class LQRPlanner : public base::Planner
    {
    public:
      /** \brief Plan from start to pose by LQR for linearized car model
       *  https://ieeexplore.ieee.org/document/7553742
       */
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

      /** \brief Free the memory allocated by this planner */
      void freeMemory();

      rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rrt_nodes_pub_;

      rclcpp::Node::SharedPtr node_;

      double dt_{0.25};
      double max_time_{5.0};
      double q1_{1};
      double q2_{10};
      double r_{1};
      double v_r_{1.0};
      double L_{0.8};
      double phi_bound_{0.4};
      double goal_tolerance_{0.5};

      void update_params(
        double dt,
        double max_time,
        double q1,
        double q2,
        double r,
        double v_r,
        double L,
        double phi_bound,
        double goal_tolerance)
      {
        dt_ = dt;
        max_time_ = max_time;
        q1_ = q1;
        q2_ = q2;
        r_ = r;
        v_r_ = v_r;
        L_ = L;
        phi_bound_ = phi_bound;
        goal_tolerance_ = goal_tolerance;
      }

      std::tuple<Eigen::Matrix2d, Eigen::Vector2d,
        Eigen::Matrix2d, double> getABQR()
      {
        Eigen::Matrix2d A(2, 2);
        A << 0, -v_r_,
          0, 0;
        Eigen::Vector2d B(0, -v_r_ / L_);
        Eigen::Matrix2d Q(2, 2);
        Q << q1_, 0,
          0, q2_;
        double R(r_);
        return std::make_tuple(A, B, Q, R);
      }

      std::vector<base::State *> compute_LQR_plan(
        base::State * start_state,
        base::State * goal_state,
        std::vector<base::State *> & resulting_path)
      {
        double start_to_goal_dist = si_->distance(start_state, goal_state);

        double time = 0.0;
        double last_dist_to_goal = INFINITY;
        resulting_path.push_back(start_state);

        const auto * start_cstate = start_state->as<ompl::base::ElevationStateSpace::StateType>();
        const auto * start_se2 = start_cstate->as<ompl::base::SE2StateSpace::StateType>(0);
        const auto * start_z = start_cstate->as<ompl::base::RealVectorStateSpace::StateType>(1);
        const auto * goal_cstate = goal_state->as<ompl::base::ElevationStateSpace::StateType>();
        const auto * goal_se2 = goal_cstate->as<ompl::base::SE2StateSpace::StateType>(0);
        const auto * goal_z = goal_cstate->as<ompl::base::RealVectorStateSpace::StateType>(1);

        std::tuple<Eigen::Matrix2d, Eigen::Vector2d,
          Eigen::Matrix2d, double> ABQR = getABQR();
        Eigen::Matrix2d A = std::get<0>(ABQR);
        Eigen::Vector2d B = std::get<1>(ABQR);
        Eigen::Matrix2d Q = std::get<2>(ABQR);
        double R = std::get<3>(ABQR);

        int iter = 0;

        while (time < max_time_) {

          auto * latest_cstate =
            resulting_path.back()->as<ompl::base::ElevationStateSpace::StateType>();
          const auto * latest_se2 = latest_cstate->as<ompl::base::SE2StateSpace::StateType>(0);
          const auto * latest_z = latest_cstate->as<ompl::base::RealVectorStateSpace::StateType>(1);

          double xc = latest_se2->getX();
          double yc = latest_se2->getY();
          double thetac = latest_se2->getYaw();

          double theta_r = std::atan2(
            (yc - start_se2->getY()),
            (xc - start_se2->getX()));

          Eigen::MatrixXd T(3, 3);
          T <<
            -std::cos(theta_r), -std::sin(theta_r), 0,
            std::sin(theta_r), -std::cos(theta_r), 0,
            0, 0, 1;

          Eigen::VectorXd e(3);
          e(0) = xc - goal_se2->getX();
          e(1) = yc - goal_se2->getY();
          e(2) = thetac - theta_r;

          Eigen::Vector3d Te_dynamics = T * e;
          Eigen::Vector2d X(Te_dynamics(1), Te_dynamics(2));

          double phi = lqr_control(A, B, Q, R, X);
          phi = std::clamp<double>(phi, -phi_bound_, phi_bound_);

          Eigen::VectorXd U(2);
          U(0) = v_r_;
          U(1) = phi;

          // Propogate the states with computed optimal control3
          // Store the state in the resulting path as that really is
          auto * this_state = si_->allocState();
          auto * this_cstate = this_state->as<ompl::base::ElevationStateSpace::StateType>();

          /*propogate according to car-like dynamics*/
          this_cstate->setSE2(
            xc + dt_ * U(0) * std::cos(thetac),
            yc + dt_ * U(0) * std::sin(thetac),
            thetac + dt_ * (U(0) * std::tan(U(1)) / L_)
          );
          // linear inetrpolation for intermediate z values
          double z_avg = (goal_z->values[0] + start_z->values[0]) / 2.0;
          this_cstate->setZ(z_avg);

          auto d_to_goal = si_->distance(this_state, goal_state);
          auto d_to_start = si_->distance(this_state, start_state);

          // linear inetrpolation for intermediate z values
          double true_z = ((start_z->values[0] * d_to_goal) + (goal_z->values[0] * d_to_start)) /
            (d_to_start + d_to_goal );
          this_cstate->setZ(true_z);

          if (si_->distance(this_state, goal_state) < goal_tolerance_) {
            // if Reached the goal or max time reached, break the loop
            U(0)  = 0;
            break;
          }

          resulting_path.push_back(this_state);

          // TIME STEP INCREASE
          time += dt_;

          iter++;

        }

        return resulting_path;
      }

      // return optimal steering angle
      double lqr_control(
        const Eigen::Matrix2d & A,
        const Eigen::Vector2d & B,
        const Eigen::Matrix2d & Q,
        const double & R,
        const Eigen::Vector2d & X)
      {
        std::tuple<Eigen::Vector2d, Eigen::Matrix2d, Eigen::VectorXcd> res = dlqr(A, B, Q, R);
        Eigen::Vector2d K = std::get<0>(res);
        double phi = (K.transpose() * X).value();
        return phi;
      }

      /**
       * @brief authored by: Horibe Takamasa; https://github.com/TakaHoribe
       *        Continous time Riccati Eq. solver
       */
      bool solve_care(
        const Eigen::Matrix2d & A,
        const Eigen::Vector2d & B,
        const Eigen::Matrix2d & Q,
        const double & R,
        Eigen::Matrix2d & P,
        const double dt = 0.001,
        const double & tolerance = 1.E-5,
        const uint iter_max = 100000)
      {
        P = Q; // initialize
        Eigen::Matrix2d P_next;
        Eigen::Matrix2d AT = A.transpose();
        double Rinv = 1.0 / R;

        double diff;
        for (uint i = 0; i < iter_max; ++i) {
          P_next = P + (P * A + AT * P - P * B * Rinv * B.transpose() * P + Q) * dt;
          diff = fabs((P_next - P).maxCoeff());
          P = P_next;
          if (diff < tolerance) {
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
       std::tuple<Eigen::Vector2d, Eigen::Matrix2d, Eigen::VectorXcd> dlqr(
        const Eigen::Matrix2d & A,
        const Eigen::Vector2d & B,
        const Eigen::Matrix2d & Q,
        const double & R)
      {
        Eigen::Matrix2d P;
        bool solved_dare = solve_care(A, B, Q, R, P);
        Eigen::Vector2d K = (1 / R) * (B.transpose() * P);
        Eigen::VectorXcd eig/* = (A - B.transpose() * K).eigenvalues()*/;
        return std::make_tuple(K, P, eig);
      }
    };

  }
}

#endif  // VOX_NAV_PLANNING__RRT__LQRPLANNER_HPP_
