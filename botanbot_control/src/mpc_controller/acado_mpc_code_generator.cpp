#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>


int main()
{
  USING_NAMESPACE_ACADO

  // — state variables (acadoVariables.x)—
  DifferentialState x;     // x position of rear axle
  DifferentialState y;     // y position of rear axle
  DifferentialState v;     // v vehicle speed [m/s]
  DifferentialState yaw;   // vehicle orientation [rad]
  DifferentialState delta;

  // — control inputs —
  Control a;                 // acceleration[m/s^2]
  Control deltarate;         // steering angle rate

  // —- differential equations —-
  double L = 1.32;            // vehicle wheel base
  DifferentialEquation f;
  f << dot(x) == v * cos(yaw);
  f << dot(y) == v * sin(yaw);
  f << dot(v) == a;
  f << dot(yaw) == v * tan(delta) / L;
  f << dot(delta) == deltarate;

  // — reference functions (acadoVariables.y) —
  Function rf;
  Function rfN;
  rf << x << y << v << yaw << delta;
  rfN << x << y << v << yaw << delta;
  // — constraints, weighting matrices for the reference functions —
  // N=number of prediction time steps, Ts=step time interval
  // Provide defined weighting matrices:
  BMatrix W = eye<bool>(rf.getDim());
  BMatrix WN = eye<bool>(rfN.getDim());

  // 5 second time horizon
  const int N = 30;  // Number of steps
  const double Ts = 0.1;

  OCP ocp(0, N * Ts, N);

  ocp.subjectTo(f);
  ocp.subjectTo(-3.0 <= a <= 3.0);
  ocp.subjectTo(-M_PI <= delta <= M_PI);
  ocp.minimizeLSQ(W, rf);
  ocp.minimizeLSQEndTerm(WN, rfN);

  // — generate MPC code —
  OCPexport mpc(ocp);
  mpc.set(HESSIAN_APPROXIMATION, GAUSS_NEWTON);
  mpc.set(DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);
  mpc.set(INTEGRATOR_TYPE, INT_RK45);
  mpc.set(SPARSE_QP_SOLUTION, FULL_CONDENSING);
  mpc.set(QP_SOLVER, QP_QPOASES);
  mpc.set(MAX_NUM_QP_ITERATIONS, 999);
  mpc.set(HOTSTART_QP, YES);
  mpc.set(CG_HARDCODE_CONSTRAINT_VALUES, NO);
  mpc.set(CG_USE_VARIABLE_WEIGHTING_MATRIX, YES);
  mpc.set(LEVENBERG_MARQUARDT, 1.0E-3);
  mpc.set(GENERATE_TEST_FILE, NO);
  mpc.set(GENERATE_MAKE_FILE, NO);
  mpc.set(GENERATE_MATLAB_INTERFACE, NO);

  mpc.exportCode("/home/ros2-foxy/mpc_acado/");

  return EXIT_SUCCESS;
}
