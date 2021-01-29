#include <botanbot_control/mpc_controller/acado.hpp>

/* Global variables used by the solver. */
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;
using namespace std;

void init_weight()
{
  double w_x = 10;
  double w_y = 200;
  double w_theta = 200;
  double w_v = 200;


  for (int i = 0; i < N; i++) {
    // Setup diagonal entries
    acadoVariables.W[NY * NY * i + (NY + 1) * 0] = w_x;
    acadoVariables.W[NY * NY * i + (NY + 1) * 1] = w_y;
    acadoVariables.W[NY * NY * i + (NY + 1) * 2] = w_theta;
    acadoVariables.W[NY * NY * i + (NY + 1) * 3] = w_v;
  }
  acadoVariables.WN[(NYN + 1) * 0] = w_x;
  acadoVariables.WN[(NYN + 1) * 1] = w_y;
  acadoVariables.WN[(NYN + 1) * 2] = w_theta;
  acadoVariables.WN[(NYN + 1) * 2] = w_v;
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
  vector<double> control_output_steering_angle;
  for (int i = 0; i < ACADO_N; ++i) {
    for (int j = 0; j < ACADO_NU; ++j) {
      if (j == 0) {
        control_output_acceleration.push_back(acadoVariables.u[i * ACADO_NU + j]);
      } else {               // only two output. therefore j == 1 means steering command output
        control_output_steering_angle.push_back(acadoVariables.u[i * ACADO_NU + j]);
      }
    }
  }
  init_weight();
  return {control_output_acceleration, control_output_steering_angle};
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

  // /* Prepare first step */
  acado_preparationStep();

  /* Get the time before start of the loop. */
  acado_tic(&t);

  /* The "real-time iterations" loop. */

  /* Perform the feedback step. */
  acado_feedbackStep();
  // acado_preparationStep();


  /* Optional: shift the initialization (look at acado_common.h). */
  /* acado_shiftStates(2, 0, 0); */
  /* acado_shiftControls( 0 ); */

  /* Read the elapsed time. */
  real_t te = acado_toc(&t);

  /* Eye-candy. */

  if (!VERBOSE) {
    printf(
      "\n\n Average time of one real-time iteration:   %.3g microseconds\n\n",
      1e6 * te / NUM_STEPS);
  }

  //acado_printDifferentialVariables();
  //acado_printControlVariables();

  vector<double> control_output_acceleration;
  vector<double> control_output_steering_angle;
  real_t * u = acado_getVariablesU();
  for (int i = 0; i < ACADO_N; ++i) {
    for (int j = 0; j < ACADO_NU; ++j) {
      if (j == 0) {
        control_output_acceleration.push_back((double)u[i * ACADO_NU + j]);
        //printf("accel cmd: %lf, \n", (double)u[i * ACADO_NU + j]);
      } else {               // only two output. therefore j == 1 means steering command output
        control_output_steering_angle.push_back((double)u[i * ACADO_NU + j]);
        //printf("steering cmd: %lf, \n", (double)u[i * ACADO_NU + j]);
      }
    }
  }
  return {control_output_acceleration, control_output_steering_angle};
}

vector<double> calculate_ref_states(
  const Eigen::VectorXd & coeff, const double & reference_v,
  double curr_speed)
{
  vector<double> ref_x;
  vector<double> ref_y;
  vector<double> ref_theta;
  vector<double> ref_v;

  double x0 = 0;
  double y0 = coeff[0] + coeff[1] * x0 + coeff[2] * pow(x0, 2) + coeff[3] * pow(x0, 3);
  double theta0 = atan(coeff[1] + 2 * coeff[2] * x0 + 3 * coeff[3] * pow(x0, 2));
  double v0 = curr_speed;

  ref_x.push_back(x0);
  ref_y.push_back(y0);
  ref_theta.push_back(theta0);
  ref_v.push_back(v0);


  double d = reference_v * Ts;
  for (int i = 0; i < N - 1; i++) {
    // current states
    double cur_x = ref_x[i];
    double cur_y = ref_y[i];
    double cur_theta = ref_theta[i];
    double cur_v = ref_v[i];


    double dx = d * cos(ref_theta[i]);
    double dy = d * sin(ref_theta[i]);
    double dyaw = d * cur_theta;
    double dv = d * cur_v;

    ref_x.push_back(ref_x[i] + dx);
    ref_y.push_back(ref_y[i] + dy);
    double next_theta =
      atan(coeff[1] + 2 * coeff[2] * ref_x[i + 1] + 3 * coeff[3] * pow(ref_x[i + 1], 2));
    ref_theta.push_back(next_theta);
    ref_v.push_back(ref_v[i] + dv);

  }
  vector<vector<double>> ref_states = {ref_x, ref_y, ref_theta, ref_v};
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
  vector<double> old_velocity_cmd = prev_u[0];
  vector<double> old_steering_angle_cmd = prev_u[1];
  vector<vector<double>> predicted_states;

  predicted_states.push_back(cur_states);
  for (int i = 0; i < N; i++) {
    //printf("i: %d, old_velocity_cmd: %lf \n", i, old_velocity_cmd[i]);
    //printf("i: %d, old_steering_angle_cmd: %lf \n", i, old_steering_angle_cmd[i]);
  }

  for (int i = 0; i < N; i++) {
    vector<double> cur_state = predicted_states[i];
    // yaw angle compensation of overflow
    if (cur_state[2] > M_PI) {
      cur_state[2] -= 2 * M_PI;
    }
    if (cur_state[2] < -M_PI) {
      cur_state[2] += 2. * M_PI;
    }
    vector<double> next_state = update_states(
      cur_state, old_velocity_cmd[i],
      old_steering_angle_cmd[i]);

    predicted_states.push_back(next_state);
  }
  for (int i = 0; i < N + 1; i++) {
    /*printf(
      "i: %d, x: %lf, y: %lf, v: %lf, yaw: %lf, yaw rate: %lf\n", i, predicted_states[i][0],
      predicted_states[i][1], predicted_states[i][2], predicted_states[i][3],
      predicted_states[i][4]);*/
  }
  vector<double> result;
  for (int i = 0; i < (ACADO_N + 1); ++i) {
    for (int j = 0; j < NY; ++j) {
      result.push_back(predicted_states[i][j]);
    }
  }
  return result;
}

vector<double> update_states(
  vector<double> state, double acceleration_cmd,
  double streering_angle_cmd)
{
  double kMAX_STEER = 0.6;   // maximum steering angle
  double kMAX_ACCEL = 2.0;    // min
  double kMIN_ACCEL = -2.0;  // max

  if (streering_angle_cmd > kMAX_STEER) {
    streering_angle_cmd = kMAX_STEER;
  } else if (streering_angle_cmd < -kMAX_STEER) {
    streering_angle_cmd = -kMAX_STEER;
  }

  if (acceleration_cmd > kMAX_ACCEL) {
    acceleration_cmd = kMAX_ACCEL;
  } else if (acceleration_cmd < kMIN_ACCEL) {
    acceleration_cmd = kMIN_ACCEL;
  }

  // based on kinematic model
  double x0 = state[0];
  double y0 = state[1];
  double theta0 = state[2];
  double v0 = state[3];

  double x1 = x0 + v0 * cos(theta0) * Ts;
  double y1 = y0 + v0 * sin(theta0) * Ts;
  double theta1 = theta0 + v0 / Lf * std::tan(streering_angle_cmd) * Ts;
  double v1 = v0 + acceleration_cmd * Ts;

  return {x1, y1, theta1, v1};
}

int calculate_nearest_state_index(
  vector<double> ptsx, vector<double> ptsy, double currx,
  double curry)
{
  int closest_point_index = -1;
  int closest_point_distance = 10000.0;

  for (size_t i = 0; i < ptsx.size(); i++) {
    double curr_distance = std::sqrt(std::pow(currx - ptsx[i], 2) + std::pow(curry - ptsy[i], 2));
    if (curr_distance < closest_point_distance) {
      closest_point_distance = curr_distance;
      closest_point_index = i;
    }
  }
  return closest_point_index;
}
