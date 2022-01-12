#include <acado_toolkit.hpp>
#include <acado_code_generation.hpp>
#include <acado_gnuplot.hpp>

#include <stdlib.h>
#include <stdio.h>

int main()
{

  double L_R = 0.66;
  double L_F = 0.66;
  const int N = 8;
  const int Ni = 4;
  const double Ts = 0.1;
  const double min_acc_dv = -1.0;
  const double max_acc_dv = 1.0;
  const double min_v_dv = -1.0;
  const double max_v_dv = 1.0;
  const double min_df_dv = -M_PI / 2;
  const double max_df_dv = M_PI / 2;

  // INTRODUCE THE VARIABLES (acadoVariables.x):
  // -------------------------
  ACADO::DifferentialState x_dv;
  ACADO::DifferentialState y_dv;
  ACADO::DifferentialState psi_dv;
  ACADO::DifferentialState v_dv;
  ACADO::Control acc_dv;
  ACADO::Control df_dv;
  ACADO::OnlineData obs_x;
  ACADO::OnlineData obs_y;
  ACADO::OnlineData obs_radius;

  // DEFINE A DIFFERENTIAL EQUATION:
  // -------------------------------
  ACADO::DifferentialEquation f;

  auto beta = atan(L_R / (L_R + L_F) * tan(df_dv));

  f << dot(x_dv) == v_dv * cos(psi_dv + beta);
  f << dot(y_dv) == v_dv * sin(psi_dv + beta);
  f << dot(psi_dv) == (v_dv / L_R * sin(beta));
  f << dot(v_dv) == acc_dv;

  ACADO::Function rf;
  ACADO::Function rfN;

  rf << x_dv << y_dv << psi_dv << v_dv << acc_dv << df_dv;
  rfN << x_dv << y_dv << psi_dv << v_dv;

  ACADO::OCP ocp(0, N * Ts, N);

  ACADO::BMatrix Q_sparse(rf.getDim(), rf.getDim());
  Q_sparse.setIdentity();
  ACADO::BMatrix QN_sparse(rfN.getDim(), rfN.getDim());
  QN_sparse.setIdentity();
  ocp.minimizeLSQ(Q_sparse, rf);
  ocp.minimizeLSQEndTerm(QN_sparse, rfN);

  ocp.subjectTo(f);

  // obstacle contraints
  ocp.subjectTo(
    sqrt(
      (x_dv - obs_x) * (x_dv - obs_x) +
      (y_dv - obs_y) * (y_dv - obs_y)) >= 2.0);

  // control constraints
  ocp.subjectTo(min_v_dv <= v_dv <= max_v_dv);
  ocp.subjectTo(min_acc_dv <= acc_dv <= max_acc_dv);
  ocp.subjectTo(min_df_dv <= df_dv <= max_df_dv);

  ocp.setNOD(3);

  ACADO::OCPexport mpc(ocp);

  mpc.set(ACADO::HESSIAN_APPROXIMATION, ACADO::GAUSS_NEWTON);           // is robust, stable
  mpc.set(ACADO::DISCRETIZATION_TYPE, ACADO::MULTIPLE_SHOOTING);        // good convergence
  mpc.set(ACADO::SPARSE_QP_SOLUTION, ACADO::FULL_CONDENSING_N2);        // due to qpOASES
  mpc.set(ACADO::INTEGRATOR_TYPE, ACADO::INT_IRK_GL4);                  // accurate
  mpc.set(ACADO::NUM_INTEGRATOR_STEPS, N);
  mpc.set(ACADO::QP_SOLVER, ACADO::QP_QPOASES);                         // free, source code
  mpc.set(ACADO::HOTSTART_QP, YES);
  mpc.set(ACADO::CG_USE_OPENMP, YES);                            // paralellization
  mpc.set(ACADO::CG_HARDCODE_CONSTRAINT_VALUES, NO);             // set on runtime
  mpc.set(ACADO::CG_USE_VARIABLE_WEIGHTING_MATRIX, YES);         // time-varying costs
  mpc.set(ACADO::CG_MODULE_NAME, "private_namespace");
  mpc.set(ACADO::USE_SINGLE_PRECISION, YES);                     // Single precision
  mpc.set(ACADO::GENERATE_TEST_FILE, YES);
  mpc.set(ACADO::GENERATE_MAKE_FILE, NO);
  mpc.set(ACADO::GENERATE_MATLAB_INTERFACE, NO);
  mpc.set(ACADO::GENERATE_SIMULINK_INTERFACE, NO);

  std::string home = std::getenv("HOME");

  if (mpc.exportCode(
      (home +
      "/colcon_ws/src/vox_nav/vox_nav_control/include/vox_nav_control/mpc_controller_acado/auto_gen")
      .c_str()) !=
    ACADO::SUCCESSFUL_RETURN)
  {
    exit(EXIT_FAILURE);
  }

  mpc.printDimensionsQP();

  return EXIT_SUCCESS;
}
