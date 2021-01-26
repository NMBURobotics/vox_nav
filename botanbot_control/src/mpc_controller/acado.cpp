#include <botanbot_control/mpc_controller/acado.hpp>

/* Global variables used by the solver. */
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;
using namespace std;

void init_weight()
{
  double w_x = 10;
  double w_y = 200;
  double w_vel = 10;
  double w_yaw = 100000;
  double w_delta = 50;

  for (int i = 0; i < N; i++) {
    // Setup diagonal entries
    acadoVariables.W[NY * NY * i + (NY + 1) * 0] = w_x;
    acadoVariables.W[NY * NY * i + (NY + 1) * 1] = w_y;
    acadoVariables.W[NY * NY * i + (NY + 1) * 2] = w_vel;
    acadoVariables.W[NY * NY * i + (NY + 1) * 3] = w_yaw;
    acadoVariables.W[NY * NY * i + (NY + 1) * 4] = w_delta;
  }
  acadoVariables.WN[(NYN + 1) * 0] = w_x;
  acadoVariables.WN[(NYN + 1) * 1] = w_y;
  acadoVariables.WN[(NYN + 1) * 2] = w_vel;
  acadoVariables.WN[(NYN + 1) * 3] = w_yaw;
  acadoVariables.WN[(NYN + 1) * 4] = w_delta;

  for (int i = 0; i < N * NY; i++) {
    acadoVariables.W[i] = 1;
  }
}

vector<vector<double>> init_acado()
{
  /* Initialize the solver. */
  acado_initializeSolver();
  for (int i = 0; i < NX * (N + 1); ++i) {
    acadoVariables.x[i] = 0.0;
  }
  for (int i = 0; i < NU * N; ++i) {
    acadoVariables.u[i] = 0.0;
  }
  for (int i = 0; i < NY * N; ++i) {
    acadoVariables.y[i] = 0.0;
  }
  for (int i = 0; i < NYN; ++i) {
    acadoVariables.yN[i] = 0.0;
  }

  acado_preparationStep();
  vector<double> control_output_acceleration;
  vector<double> control_output_steering_rate;
  for (int i = 0; i < ACADO_N; ++i) {
    for (int j = 0; j < ACADO_NU; ++j) {
      if (j == 0) {
        control_output_acceleration.push_back(acadoVariables.u[i * ACADO_NU + j]);
      } else {               // only two output. therefore j == 1 means steering command output
        control_output_steering_rate.push_back(acadoVariables.u[i * ACADO_NU + j]);
      }
    }
  }
  init_weight();
  return {control_output_acceleration, control_output_steering_rate};
}

vector<vector<double>> run_mpc_acado(
  vector<double> states, vector<double> ref_states,
  vector<vector<double>> previous_u)
{
  /* Some temporary variables. */
  int i, iter;
  acado_timer t;

  for (i = 0; i < NX * (N + 1); ++i) {
    acadoVariables.x[i] = (real_t) states[i];
  }
  for (i = 0; i < NX; ++i) {
    acadoVariables.x0[i] = (real_t)states[i];
  }

  /* Initialize the measurements/reference. */
  for (i = 0; i < NY * N; ++i) {
    acadoVariables.y[i] = (real_t)ref_states[i];
  }
  for (i = 0; i < NYN; ++i) {
    acadoVariables.yN[i] = (real_t)ref_states[NY * (N - 1) + i];
  }

  /* MPC: initialize the current state feedback. */
  // #if ACADO_INITIAL_STATE_FIXED
  //    for (i = 0; i < NX; ++i)
  //            acadoVariables.x0[i] = 0.1;
  // #endif

  if (VERBOSE) {
    acado_printHeader();
  }

  // /* Prepare first step */
  acado_preparationStep();

  /* Get the time before start of the loop. */
  acado_tic(&t);

  /* The "real-time iterations" loop. */

  /* Perform the feedback step. */
  acado_feedbackStep();
  // acado_preparationStep();

  /* Apply the new control immediately to the process, first NU components. */

  if (VERBOSE) {
    printf("\tReal-Time Iteration %d:  KKT Tolerance = %.3e\n\n", iter, acado_getKKT());
  }

  /* Optional: shift the initialization (look at acado_common.h). */
  /* acado_shiftStates(2, 0, 0); */
  /* acado_shiftControls( 0 ); */

  /* Read the elapsed time. */
  real_t te = acado_toc(&t);

  if (VERBOSE) {
    printf("\n\nEnd of the RTI loop. \n\n\n");
  }

  /* Eye-candy. */

  if (!VERBOSE) {
    printf(
      "\n\n Average time of one real-time iteration:   %.3g microseconds\n\n",
      1e6 * te / NUM_STEPS);
  }

  acado_printDifferentialVariables();
  acado_printControlVariables();

  vector<double> control_output_acceleration;
  vector<double> control_output_steering_rate;
  real_t * u = acado_getVariablesU();
  for (int i = 0; i < ACADO_N; ++i) {
    for (int j = 0; j < ACADO_NU; ++j) {
      if (j == 0) {
        control_output_acceleration.push_back((double)u[i * ACADO_NU + j]);
        printf("accel cmd: %lf, \n", (double)u[i * ACADO_NU + j]);
      } else {               // only two output. therefore j == 1 means steering command output
        control_output_steering_rate.push_back((double)u[i * ACADO_NU + j]);
        printf("steering cmd: %lf, \n", (double)u[i * ACADO_NU + j]);
      }
    }
  }
  return {control_output_acceleration, control_output_steering_rate};
}

vector<double> calculate_ref_states(const Eigen::VectorXd & coeff, const double & reference_v)
{
  vector<double> ref_x;
  vector<double> ref_y;
  vector<double> ref_v;
  vector<double> ref_yaw;
  vector<double> ref_yaw_rate;

  double x0 = 0;
  double y0 = coeff[0] + coeff[1] * x0 + coeff[2] * pow(x0, 2) + coeff[3] * pow(x0, 3);
  double yaw0 = atan(coeff[1] + 2 * coeff[2] * x0 + 3 * coeff[3] * pow(x0, 2));
  double yaw_rate0 = 0;

  ref_x.push_back(x0);
  ref_y.push_back(y0);
  ref_yaw.push_back(yaw0);
  ref_v.push_back(reference_v);
  ref_yaw_rate.push_back(yaw_rate0);

  double d = reference_v * Ts;

  for (int i = 0; i < N - 1; i++) {
    // current states
    double cur_x = ref_x[i];
    double cur_y = ref_y[i];
    double cur_yaw = ref_yaw[i];
    double dx = d * cos(ref_yaw[i]);
    double dy = d * sin(ref_yaw[i]);
    double dyaw = d * cur_yaw;

    ref_x.push_back(ref_x[i] + dx);
    ref_y.push_back(ref_y[i] + dy);
    double next_yaw =
      atan(coeff[1] + 2 * coeff[2] * ref_x[i + 1] + 3 * coeff[3] * pow(ref_x[i + 1], 2));
    ref_yaw.push_back(next_yaw);
    ref_yaw_rate.push_back(ref_yaw[i] + dyaw);
    ref_v.push_back(reference_v);
  }
  vector<vector<double>> ref_states = {ref_x, ref_y, ref_v, ref_yaw, ref_yaw_rate};
  vector<double> result;
  for (int i = 0; i < ACADO_N; ++i) {
    for (int j = 0; j < NY; ++j) {
      result.push_back(ref_states[j][i]);
    }
  }
  return result;
}
vector<double> motion_prediction(
  const vector<double> & cur_states,
  const vector<vector<double>> & prev_u)
{
  vector<double> old_acceleration_cmd = prev_u[0];
  vector<double> old_steering_rate_cmd = prev_u[1];
  vector<vector<double>> predicted_states;
  predicted_states.push_back(cur_states);
  for (int i = 0; i < N; i++) {
    printf("i: %d, old_acceleration_cmd: %lf \n", i, old_acceleration_cmd[i]);
    printf("i: %d, old_steering_rate_cmd: %lf \n", i, old_steering_rate_cmd[i]);
  }
  for (int i = 0; i < N; i++) {
    vector<double> cur_state = predicted_states[i];
    // yaw angle compensation of overflow
    if (cur_state[3] > M_PI) {
      cur_state[3] -= 2 * M_PI;
    }
    if (cur_state[3] < -M_PI) {
      cur_state[3] += 2. * M_PI;
    }
    vector<double> next_state = update_states(
      cur_state, old_acceleration_cmd[i],
      old_steering_rate_cmd[i]);
    predicted_states.push_back(next_state);
  }
  printf("-------- motion prediction -------- \n");
  for (int i = 0; i < N + 1; i++) {
    printf(
      "i: %d, x: %lf, y: %lf, v: %lf, yaw: %lf, yaw rate: %lf\n", i, predicted_states[i][0],
      predicted_states[i][1], predicted_states[i][2], predicted_states[i][3],
      predicted_states[i][4]);
  }
  vector<double> result;
  for (int i = 0; i < (ACADO_N + 1); ++i) {
    for (int j = 0; j < NY; ++j) {
      result.push_back(predicted_states[i][j]);
    }
  }
  return result;
}

vector<double> update_states(vector<double> state, double acceleration_cmd, double deltarate)
{
  // based on kinematic model
  double x0 = state[0];
  double y0 = state[1];
  double v0 = state[2];
  double yaw0 = state[3];
  double delta0 = state[4];

  double x1 = x0 + v0 * cos(yaw0) * Ts;
  double y1 = y0 + v0 * sin(yaw0) * Ts;
  double v1 = v0 + acceleration_cmd * Ts;
  double yaw1 = yaw0 + v0 / Lf * std::tan(delta0) * Ts;
  double delta1 = delta0 + deltarate * Ts;
  return {x1, y1, v1, yaw1, delta1};
}
