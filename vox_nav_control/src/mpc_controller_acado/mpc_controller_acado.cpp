#include <vox_nav_control/mpc_controller_acado/auto_gen/acado_common.h>
#include <vox_nav_control/mpc_controller_acado/auto_gen/acado_auxiliary_functions.h>
#include <stdio.h>

/* Some convenient definitions. */
#define NX          ACADO_NX  /* Number of differential state variables.  */
#define NXA         ACADO_NXA /* Number of algebraic variables. */
#define NU          ACADO_NU  /* Number of control inputs. */
#define NOD         ACADO_NOD  /* Number of online data values. */

#define NY          ACADO_NY  /* Number of measurements/references on nodes 0..N - 1. */
#define NYN         ACADO_NYN /* Number of measurements/references on node N. */

#define N           ACADO_N   /* Number of intervals in the horizon. */

#define NUM_STEPS   10        /* Number of real-time iterations. */
#define VERBOSE     1         /* Show iterations: 1, silent: 0.  */

/* Global variables used by the solver. */
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

int main()
{

  /* Some temporary variables. */
  int i, iter;
  acado_timer t;

  /* Initialize the solver. */
  acado_initializeSolver();

  /* Initialize the states and controls. */
  for (i = 0; i < NX * (N + 1); ++i) {
    acadoVariables.x[i] = 0.0;
  }
  for (i = 0; i < NU * N; ++i) {
    acadoVariables.u[i] = 0.0;
  }

  /* Initialize the measurements/reference. */
  for (i = 0; i < NY * N; ++i) {
    acadoVariables.y[i] = 0.0;
  }
  for (i = 0; i < NYN; ++i) {
    acadoVariables.yN[i] = 0.0;
  }

  /* MPC: initialize the current state feedback. */
#if ACADO_INITIAL_STATE_FIXED
  for (i = 0; i < NX; ++i) {
    acadoVariables.x0[i] = 0.1;
  }
#endif

  if (VERBOSE) {acado_printHeader();}

  /* Prepare first step */
  acado_preparationStep();

  /* Get the time before start of the loop. */
  acado_tic(&t);

  /* The "real-time iterations" loop. */
  for (iter = 0; iter < NUM_STEPS; ++iter) {
    /* Perform the feedback step. */
    acado_feedbackStep();

    /* Apply the new control immediately to the process, first NU components. */

    if (VERBOSE) {
      printf("\tReal-Time Iteration %d:  KKT Tolerance = %.3e\n\n", iter, acado_getKKT() );
    }

    /* Optional: shift the initialization (look at acado_common.h). */
    /* acado_shiftStates(2, 0, 0); */
    /* acado_shiftControls( 0 ); */

    /* Prepare for the next step. */
    acado_preparationStep();
  }
  /* Read the elapsed time. */
  real_t te = acado_toc(&t);

  if (VERBOSE) {printf("\n\nEnd of the RTI loop. \n\n\n");}

  /* Eye-candy. */

  if (!VERBOSE) {
    printf(
      "\n\n Average time of one real-time iteration:   %.3g microseconds\n\n",
      1e6 * te / NUM_STEPS);
  }

  acado_printDifferentialVariables();
  acado_printControlVariables();


}
