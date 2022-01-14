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
  const int Ni = 1;
  const double Ts = 0.1;
  const double min_acc_dv = -1.0;
  const double max_acc_dv = 1.0;
  const double min_v_dv = -1.0;
  const double max_v_dv = 1.0;
  const double min_df_dv = -M_PI_2 / 4;
  const double max_df_dv = M_PI_2 / 4;

  // INTRODUCE THE VARIABLES (acadoVariables.x):
  // -------------------------
  ACADO::DifferentialState x_dv;
  ACADO::DifferentialState y_dv;
  ACADO::DifferentialState v_dv;
  ACADO::DifferentialState psi_dv;
  ACADO::Control acc_dv;
  ACADO::Control df_dv;
  //ACADO::OnlineData obs_x;
  //ACADO::OnlineData obs_y;
  //ACADO::OnlineData obs_radius;

  // DEFINE A DIFFERENTIAL EQUATION:
  // -------------------------------
  ACADO::DifferentialEquation f;

  //auto beta = atan(L_R / (L_R + L_F) * tan(df_dv));

  f << dot(x_dv) == v_dv * cos(psi_dv /*+ beta*/);
  f << dot(y_dv) == v_dv * sin(psi_dv /*+ beta*/);
  //f << dot(psi_dv) == (v_dv / L_R * sin(beta));
  f << dot(v_dv) == acc_dv;
  f << dot(psi_dv) == v_dv * tan(df_dv) / L_R;

  ACADO::Function rf;
  ACADO::Function rfN;

  rf << x_dv << y_dv << v_dv << psi_dv << acc_dv << df_dv;
  rfN << x_dv << y_dv << v_dv << psi_dv;

  ACADO::OCP ocp(0, N * Ts, N);

  ocp.subjectTo(f);

  // obstacle contraints
  /*ocp.subjectTo(
    sqrt(
      (x_dv - obs_x) * (x_dv - obs_x) +
      (y_dv - obs_y) * (y_dv - obs_y)) >= 2.0);*/

  // control constraints
  ocp.subjectTo(min_acc_dv <= acc_dv <= max_acc_dv);
  ocp.subjectTo(min_df_dv <= df_dv <= max_df_dv);
  //ocp.subjectTo(min_v_dv <= v_dv <= max_v_dv);

  // Provide defined weighting matrices:
  ACADO::BMatrix W = ACADO::eye<bool>(rf.getDim());
  ACADO::BMatrix WN = ACADO::eye<bool>(rfN.getDim());

  ocp.minimizeLSQ(W, rf);
  ocp.minimizeLSQEndTerm(WN, rfN);

  //ocp.setNOD(3);

  ACADO::OCPexport mpc(ocp);

  USING_NAMESPACE_ACADO

  mpc.set(HESSIAN_APPROXIMATION, GAUSS_NEWTON);
  mpc.set(DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);
  mpc.set(INTEGRATOR_TYPE, INT_IRK_RIIA3);
  mpc.set(NUM_INTEGRATOR_STEPS, N * Ni);
  mpc.set(SPARSE_QP_SOLUTION, FULL_CONDENSING);
  mpc.set(QP_SOLVER, QP_QPOASES);
  mpc.set(HOTSTART_QP, YES);
  mpc.set(GENERATE_TEST_FILE, YES);
  mpc.set(GENERATE_MAKE_FILE, NO);
  mpc.set(GENERATE_MATLAB_INTERFACE, NO);
  mpc.set(CG_USE_VARIABLE_WEIGHTING_MATRIX, YES);
  mpc.set(FIX_INITIAL_STATE, YES);

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
