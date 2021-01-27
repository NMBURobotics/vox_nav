#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>


int main()
{
  USING_NAMESPACE_ACADO

  // — state variables (acadoVariables.x)—
  DifferentialState x;       // x position of rear axle
  DifferentialState y;       // y position of rear axle
  DifferentialState theta;   // vehicle orientation [rad]

  // — control inputs —
  Control v;                   // speed[m/s]
  Control phi;                 // steering angle

  // —- differential equations —-
  double L = 1.32;            // vehicle wheel base
  DifferentialEquation f;
  f << dot(x) == v * cos(theta);
  f << dot(y) == v * sin(theta);
  f << dot(theta) == (v / L) * tan(phi);

  // — reference functions (acadoVariables.y) —
  Function rf;
  Function rfN;
  rf << x << y << theta;
  rfN << x << y << theta;
  // — constraints, weighting matrices for the reference functions —
  // N=number of prediction time steps, Ts=step time interval
  // Provide defined weighting matrices:
  BMatrix W = eye<bool>(rf.getDim());
  BMatrix WN = eye<bool>(rfN.getDim());

  // Number of steps
  const int N = 10;
  const double Ts = 0.1;

  OCP ocp(0, N * Ts, N);

  ocp.subjectTo(f);
  // limit velocity(m/s) to [-1.0,1.0]
  ocp.subjectTo(-1.0 <= v <= 1.0);

  // limit steering angle(rad) to [-0.78,0.78]
  ocp.subjectTo(-M_PI / 4.0 <= phi <= M_PI / 4.0);
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
