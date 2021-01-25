#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>


int main()
{
  USING_NAMESPACE_ACADO

  // — state variables (acadoVariables.x)—
  DifferentialState x;
  DifferentialState y;
  DifferentialState v;
  DifferentialState phi;
  // — control inputs —
  Control a;
  Control delta;
  // —- differential equations —-
  double L = 1.32;            // vehicle wheel base
  DifferentialEquation f;
  f << dot(x) == v * cos(phi);
  f << dot(y) == v * sin(phi);
  f << dot(v) == a;
  f << dot(phi) == v * tan(delta) / L;
  // — reference functions (acadoVariables.y) —
  Function rf;
  Function rfN;
  rf << x << y << v << phi;
  rfN << x << y << v << phi;
  // — constraints, weighting matrices for the reference functions —
  // N=number of prediction time steps, Ts=step time interval
  // Provide defined weighting matrices:
  BMatrix W = eye<bool>(rf.getDim());
  BMatrix WN = eye<bool>(rfN.getDim());
  // define real-time optimal control problem
  OCP ocp(0, 100 * 0.001, 100);
  ocp.subjectTo(f);
  ocp.subjectTo(-1 <= a <= 1);
  ocp.subjectTo(-M_PI <= delta <= M_PI);
  ocp.minimizeLSQ(W, rf);
  ocp.minimizeLSQEndTerm(WN, rfN);
  // — generate MPC code —
  OCPexport mpc(ocp);
  mpc.exportCode("/home/ros2-foxy/mpc_acado/", "mpc_controller", "int", 16);


  return EXIT_SUCCESS;
}
