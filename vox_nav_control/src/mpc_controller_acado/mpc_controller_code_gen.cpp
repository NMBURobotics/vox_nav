#include <acado_toolkit.hpp>
#include <acado_code_generation.hpp>

int main()
{

  double L_R = 0.66;
  double L_F = 0.66;
  const int N = 5;
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

  rf << x_dv << y_dv << psi_dv << v_dv;
  rfN << x_dv << y_dv << psi_dv << v_dv;


  // Provide defined weighting matrices:
  ACADO::DMatrix W = ACADO::eye<double>(rf.getDim());
  ACADO::DMatrix WN = ACADO::eye<double>(rfN.getDim());

  W(0, 0) = 0.1;
  W(1, 1) = 0.1;
  W(2, 2) = 0.05;
  WN(0, 0) = 1.0;
  WN(1, 1) = 1.0;
  WN(2, 2) = 0.05;

  ACADO::OCP ocp(0, N * Ts, N);

  ocp.subjectTo(f);

  // obstacle contraints
  // for(int i = 0; i < NUM_OBST; i++){
  //   ocp.subjectTo(sqrt((x[0]-obst[i][0])*(x[0]-obst[i][0]) + (x[2]-obst[i][1])*(x[2]-obst[i][1])) >= OBST_THRS);
  // }

  ocp.minimizeLSQ(W, rf);
  ocp.minimizeLSQEndTerm(WN, rfN);

  // control constraints
  ocp.subjectTo(min_v_dv <= v_dv <= max_v_dv);
  ocp.subjectTo(min_acc_dv <= acc_dv <= max_acc_dv);
  ocp.subjectTo(min_df_dv <= df_dv <= max_df_dv);

  //
  // Export the code:
  //
  ACADO::OCPexport mpc(ocp);
  mpc.set(ACADO::HESSIAN_APPROXIMATION, ACADO::GAUSS_NEWTON);
  //mpc.set(DISCRETIZATION_TYPE, SINGLE_SHOOTING);
  mpc.set(ACADO::DISCRETIZATION_TYPE, ACADO::MULTIPLE_SHOOTING);
  //mpc.set(INTEGRATOR_TYPE, INT_RK4);
  mpc.set(ACADO::INTEGRATOR_TYPE, ACADO::INT_IRK_RIIA3);
  mpc.set(ACADO::NUM_INTEGRATOR_STEPS, N * Ni);
  mpc.set(ACADO::SPARSE_QP_SOLUTION, ACADO::FULL_CONDENSING);
  mpc.set(ACADO::QP_SOLVER, ACADO::QP_QPOASES);
  mpc.set(ACADO::HOTSTART_QP, YES);
  //	mpc.set(SPARSE_QP_SOLUTION, SPARSE_SOLVER);
  //	mpc.set(LEVENBERG_MARQUARDT, 1.0e-10);
  mpc.set(ACADO::GENERATE_TEST_FILE, YES);
  mpc.set(ACADO::GENERATE_MAKE_FILE, NO);
  mpc.set(ACADO::GENERATE_MATLAB_INTERFACE, NO);
  //	mpc.set(USE_SINGLE_PRECISION, YES);
  //mpc.set( CG_HARDCODE_CONSTRAINT_VALUES, NO);
  //	mpc.set(CG_USE_OPENMP, YES);
  // NOTE: This is crucial for export of MHE!
  //mpc.set(SPARSE_QP_SOLUTION, CONDENSING);
  mpc.set(ACADO::FIX_INITIAL_STATE, YES);


  if (mpc.exportCode("/home/atas/colcon_ws/auto_gen") != ACADO::SUCCESSFUL_RETURN) {
    exit(EXIT_FAILURE);
  }

  mpc.printDimensionsQP();

  return EXIT_SUCCESS;
}
