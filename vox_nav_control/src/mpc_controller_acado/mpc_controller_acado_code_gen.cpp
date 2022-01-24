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

#include <acado_toolkit.hpp>
#include <acado_code_generation.hpp>
#include <acado_gnuplot.hpp>

#include <stdlib.h>
#include <stdio.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{

  rclcpp::init(argc, argv);

  int N;
  int Ni;
  int max_obstacles; // should be the nearest ones
  double DT;
  double L_F;
  double L_R;
  double min_acc_dv;
  double max_acc_dv;
  double min_df_dv;
  double max_df_dv;
  double robot_radius;

  auto node = std::make_shared<rclcpp::Node>("vox_nav_controller_server_rclcpp_node");

  RCLCPP_INFO(node->get_logger(), "Trying to acquire paramaters for %s", node->get_name());

  std::string plugin_name = "MPCControllerAcadoROS";
  node->declare_parameter("robot_radius", 0.8);
  node->declare_parameter(plugin_name + ".N", 10);
  node->declare_parameter(plugin_name + ".Ni", 1);
  node->declare_parameter(plugin_name + ".max_obstacles", 1);
  node->declare_parameter(plugin_name + ".DT", 0.1);
  node->declare_parameter(plugin_name + ".L_F", 0.66);
  node->declare_parameter(plugin_name + ".L_R", 0.66);
  node->declare_parameter(plugin_name + ".A_MIN", -1.0);
  node->declare_parameter(plugin_name + ".A_MAX", 1.0);
  node->declare_parameter(plugin_name + ".DF_MIN", -0.5);
  node->declare_parameter(plugin_name + ".DF_MAX", 0.5);

  node->get_parameter("robot_radius", robot_radius);
  node->get_parameter(plugin_name + ".N", N);
  node->get_parameter(plugin_name + ".Ni", Ni);
  node->get_parameter(plugin_name + ".max_obstacles", max_obstacles);
  node->get_parameter(plugin_name + ".DT", DT);
  node->get_parameter(plugin_name + ".L_F", L_F);
  node->get_parameter(plugin_name + ".L_R", L_R);
  node->get_parameter(plugin_name + ".A_MIN", min_acc_dv);
  node->get_parameter(plugin_name + ".A_MAX", max_acc_dv);
  node->get_parameter(plugin_name + ".DF_MIN", min_df_dv);
  node->get_parameter(plugin_name + ".DF_MAX", max_df_dv);

  RCLCPP_INFO(node->get_logger(), "Generating acado code with following parameters");

  RCLCPP_INFO_STREAM(node->get_logger(), "robot_radius " << robot_radius);
  RCLCPP_INFO_STREAM(node->get_logger(), "N " << N);
  RCLCPP_INFO_STREAM(node->get_logger(), "Ni " << Ni);
  RCLCPP_INFO_STREAM(node->get_logger(), "max_obstacles " << max_obstacles);
  RCLCPP_INFO_STREAM(node->get_logger(), "DT " << DT);
  RCLCPP_INFO_STREAM(node->get_logger(), "L_F " << L_F);
  RCLCPP_INFO_STREAM(node->get_logger(), "L_R " << L_R);
  RCLCPP_INFO_STREAM(node->get_logger(), "A_MIN " << min_acc_dv);
  RCLCPP_INFO_STREAM(node->get_logger(), "A_MAX " << max_acc_dv);
  RCLCPP_INFO_STREAM(node->get_logger(), "DF_MIN " << min_df_dv);
  RCLCPP_INFO_STREAM(node->get_logger(), "DF_MAX " << max_df_dv);

  // INTRODUCE THE VARIABLES (acadoVariables.x):
  // -------------------------
  ACADO::DifferentialState x_dv;
  ACADO::DifferentialState y_dv;
  ACADO::DifferentialState psi_dv;
  ACADO::DifferentialState v_dv;

  ACADO::Control acc_dv;
  ACADO::Control df_dv;

  // DEFINE A DIFFERENTIAL EQUATION:
  // -------------------------------
  ACADO::DifferentialEquation f;

  /*// FULL ACKERMAN MODEL
  auto beta = atan(L_R / (L_R + L_F) * tan(df_dv));
  f << dot(x_dv) == v_dv * cos(psi_dv + beta);
  f << dot(y_dv) == v_dv * sin(psi_dv + beta);
  f << dot(v_dv) == acc_dv;
  f << dot(psi_dv) == (v_dv / L_R * sin(beta));*/

  // Simlistic model for acceleration and angular speed control
  f << dot(x_dv) == v_dv * cos(psi_dv);
  f << dot(y_dv) == v_dv * sin(psi_dv);
  f << dot(psi_dv) == df_dv; // and angular speed
  f << dot(v_dv) == acc_dv;  // control acceleration and

  struct Obstacle // Defined by ellipses TODO(jediofgever), ellipses
  {
    ACADO::OnlineData h; // center x
    ACADO::OnlineData k; // center y
    ACADO::OnlineData a; // length along x axis
    ACADO::OnlineData b; // length along y axis
  };

  std::vector<Obstacle> obstacles;
  ACADO::DifferentialState obstacle_cost;

  //Costs related to obstacles
  for (size_t i = 0; i < max_obstacles; i++) {
    Obstacle obs;
    auto obs_expression =
      ((pow((x_dv - obs.h), 2) / pow(obs.a, 2) +
      pow((y_dv - obs.k), 2) / pow(obs.b, 2) - 1.0));

    obs_expression = obs_expression.getPowInt(4);
    obs_expression = exp(1.0 / obs_expression);

    obstacle_cost += obs_expression;
    obstacles.push_back(obs);
  }

  ACADO::Function rf;
  ACADO::Function rfN;

  rf << x_dv << y_dv << psi_dv << v_dv << acc_dv << df_dv << obstacle_cost;
  rfN << x_dv << y_dv << psi_dv << v_dv;

  ACADO::OCP ocp(0, N * DT, N);
  // dynamics
  ocp.subjectTo(f);
  // control constraints
  ocp.subjectTo(min_acc_dv <= acc_dv <= max_acc_dv);
  ocp.subjectTo(min_df_dv <= df_dv <= max_df_dv);

  // obstacle constraints
  for (auto && i : obstacles) {
    ocp.subjectTo(
      pow((x_dv - i.h), 2) / pow(i.a, 2) +
      pow((y_dv - i.k), 2) / pow(i.b, 2) - (robot_radius + 1.0) >= 0.0);
  }

  // Provide defined weighting matrices:
  ACADO::BMatrix W = ACADO::eye<bool>(rf.getDim());
  ACADO::BMatrix WN = ACADO::eye<bool>(rfN.getDim());

  ocp.minimizeLSQ(W, rf);
  ocp.minimizeLSQEndTerm(WN, rfN);
  ocp.setNOD(obstacles.size() * 4);

  ACADO::OCPexport mpc(ocp);
  USING_NAMESPACE_ACADO

  mpc.set(HESSIAN_APPROXIMATION, GAUSS_NEWTON);
  mpc.set(DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);
  mpc.set(INTEGRATOR_TYPE, INT_IRK_GL4);
  mpc.set(NUM_INTEGRATOR_STEPS, N * Ni);
  mpc.set(SPARSE_QP_SOLUTION, FULL_CONDENSING);
  mpc.set(QP_SOLVER, QP_QPOASES);
  mpc.set(HOTSTART_QP, YES);
  mpc.set(GENERATE_TEST_FILE, YES);
  mpc.set(CG_USE_OPENMP, YES);
  mpc.set(GENERATE_MAKE_FILE, NO);
  mpc.set(CG_USE_VARIABLE_WEIGHTING_MATRIX, YES);
  mpc.set(FIX_INITIAL_STATE, YES);

  std::string home = std::getenv("HOME");
  std::string full_path = home +
    "/colcon_ws/src/vox_nav/vox_nav_control/include/vox_nav_control/mpc_controller_acado/auto_gen";

  if (mpc.exportCode(full_path.c_str()) != ACADO::SUCCESSFUL_RETURN) {
    exit(EXIT_FAILURE);
  }

  mpc.printDimensionsQP();

  RCLCPP_INFO(
    node->get_logger(),
    "Dumped auto generated code to: %s, Note that it needs to be under ::vox_nav/vox_nav_control/include/vox_nav_control/mpc_controller_acado/auto_gen::",
    full_path.c_str()
  );

  return EXIT_SUCCESS;
}
