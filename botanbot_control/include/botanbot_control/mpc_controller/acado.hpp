#ifndef ACADO_H
#define ACADO_H
#include <iostream>
#include <stdio.h>
#include <vector>
#include <Eigen/Core>
#include <Eigen/QR>
#include <botanbot_control/mpc_controller/mpc_acado/acado_auxiliary_functions.h>
#include <botanbot_control/mpc_controller/mpc_acado/acado_common.h>

using namespace std;

#define NX   ACADO_NX              /* Number of differential state variables.  */
#define NXA  ACADO_NXA             /* Number of algebraic variables. */
#define NU   ACADO_NU              /* Number of control inputs. */
#define NOD  ACADO_NOD             /* Number of online data values. */

#define NY   ACADO_NY              /* Number of measurements/references on nodes 0..N - 1. */
#define NYN  ACADO_NYN             /* Number of measurements/references on node N. */

#define N    ACADO_N               /* Number of intervals in the horizon. */

#define NUM_STEPS 10               /* Number of real-time iterations. */
#define VERBOSE 1                  /* Show iterations: 1, silent: 0.  */
vector<vector<double>> init_acado();
vector<vector<double>> run_mpc_acado(
  vector<double> states, vector<double> ref_states,
  vector<vector<double>> previous_u);
vector<double> motion_prediction(
  const vector<double> & cur_states,
  const vector<vector<double>> & prev_u);
vector<double> calculate_ref_states(const Eigen::VectorXd & coeff, const double & ref_v);
vector<double> update_states(vector<double> state, double acceleration_cmd, double steering_cmd);
#define Ts 0.1 // sampling time
#define Lf 2.6

#endif
