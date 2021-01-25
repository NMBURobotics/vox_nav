/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#ifndef ACADO_COMMON_H
#define ACADO_COMMON_H

#include <math.h>
#include <string.h>

#ifndef __MATLAB__
#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */
#endif /* __MATLAB__ */

/** \defgroup ACADO ACADO CGT generated module. */
/** @{ */

/** qpOASES QP solver indicator. */
#define ACADO_QPOASES  0
#define ACADO_QPOASES3 1
/** FORCES QP solver indicator.*/
#define ACADO_FORCES   2
/** qpDUNES QP solver indicator.*/
#define ACADO_QPDUNES  3
/** HPMPC QP solver indicator. */
#define ACADO_HPMPC    4
#define ACADO_GENERIC    5

/** Indicator for determining the QP solver used by the ACADO solver code. */
#define ACADO_QP_SOLVER ACADO_QPOASES

#include "acado_qpoases_interface.hpp"


/*
 * Common definitions
 */
/** User defined block based condensing. */
#define ACADO_BLOCK_CONDENSING 0
/** Compute covariance matrix of the last state estimate. */
#define ACADO_COMPUTE_COVARIANCE_MATRIX 0
/** Flag indicating whether constraint values are hard-coded or not. */
#define ACADO_HARDCODED_CONSTRAINT_VALUES 1
/** Indicator for fixed initial state. */
#define ACADO_INITIAL_STATE_FIXED 1
/** Number of control/estimation intervals. */
#define ACADO_N 100
/** Number of online data values. */
#define ACADO_NOD 0
/** Number of path constraints. */
#define ACADO_NPAC 0
/** Number of control variables. */
#define ACADO_NU 2
/** Number of differential variables. */
#define ACADO_NX 4
/** Number of algebraic variables. */
#define ACADO_NXA 0
/** Number of differential derivative variables. */
#define ACADO_NXD 0
/** Number of references/measurements per node on the first N nodes. */
#define ACADO_NY 4
/** Number of references/measurements on the last (N + 1)st node. */
#define ACADO_NYN 4
/** Total number of QP optimization variables. */
#define ACADO_QP_NV 200
/** Number of integration steps per shooting interval. */
#define ACADO_RK_NIS 1
/** Number of Runge-Kutta stages per integration step. */
#define ACADO_RK_NSTAGES 4
/** Providing interface for arrival cost. */
#define ACADO_USE_ARRIVAL_COST 0
/** Indicator for usage of non-hard-coded linear terms in the objective. */
#define ACADO_USE_LINEAR_TERMS 0
/** Indicator for type of fixed weighting matrices. */
#define ACADO_WEIGHTING_MATRICES_TYPE 1


/*
 * Globally used structure definitions
 */

/** The structure containing the user data.
 * 
 *  Via this structure the user "communicates" with the solver code.
 */
typedef struct ACADOvariables_
{
int dummy;
/** Matrix of size: 101 x 4 (row major format)
 * 
 *  Matrix containing 101 differential variable vectors.
 */
mpc_controller x[ 404 ];

/** Matrix of size: 100 x 2 (row major format)
 * 
 *  Matrix containing 100 control variable vectors.
 */
mpc_controller u[ 200 ];

/** Column vector of size: 400
 * 
 *  Matrix containing 100 reference/measurement vectors of size 4 for first 100 nodes.
 */
mpc_controller y[ 400 ];

/** Column vector of size: 4
 * 
 *  Reference/measurement vector for the 101. node.
 */
mpc_controller yN[ 4 ];

/** Matrix of size: 4 x 4 (row major format) */
mpc_controller W[ 16 ];

/** Matrix of size: 4 x 4 (row major format) */
mpc_controller WN[ 16 ];

/** Column vector of size: 4
 * 
 *  Current state feedback vector.
 */
mpc_controller x0[ 4 ];


} ACADOvariables;

/** Private workspace used by the auto-generated code.
 * 
 *  Data members of this structure are private to the solver.
 *  In other words, the user code should not modify values of this 
 *  structure. 
 */
typedef struct ACADOworkspace_
{
/** Column vector of size: 19 */
mpc_controller rhs_aux[ 19 ];

mpc_controller rk_ttt;

/** Row vector of size: 30 */
mpc_controller rk_xxx[ 30 ];

/** Matrix of size: 4 x 28 (row major format) */
mpc_controller rk_kkk[ 112 ];

/** Row vector of size: 30 */
mpc_controller state[ 30 ];

/** Column vector of size: 400 */
mpc_controller d[ 400 ];

/** Column vector of size: 400 */
mpc_controller Dy[ 400 ];

/** Column vector of size: 4 */
mpc_controller DyN[ 4 ];

/** Matrix of size: 400 x 4 (row major format) */
mpc_controller evGx[ 1600 ];

/** Matrix of size: 400 x 2 (row major format) */
mpc_controller evGu[ 800 ];

/** Row vector of size: 6 */
mpc_controller objValueIn[ 6 ];

/** Row vector of size: 4 */
mpc_controller objValueOut[ 4 ];

/** Matrix of size: 400 x 4 (row major format) */
mpc_controller Q1[ 1600 ];

/** Matrix of size: 400 x 4 (row major format) */
mpc_controller Q2[ 1600 ];

/** Matrix of size: 4 x 4 (row major format) */
mpc_controller QN1[ 16 ];

/** Matrix of size: 4 x 4 (row major format) */
mpc_controller QN2[ 16 ];

/** Column vector of size: 4 */
mpc_controller Dx0[ 4 ];

/** Matrix of size: 4 x 4 (row major format) */
mpc_controller T[ 16 ];

/** Matrix of size: 20200 x 2 (row major format) */
mpc_controller E[ 40400 ];

/** Matrix of size: 20200 x 2 (row major format) */
mpc_controller QE[ 40400 ];

/** Column vector of size: 400 */
mpc_controller Qd[ 400 ];

/** Column vector of size: 404 */
mpc_controller QDy[ 404 ];

/** Matrix of size: 200 x 4 (row major format) */
mpc_controller H10[ 800 ];

/** Matrix of size: 200 x 200 (row major format) */
mpc_controller H[ 40000 ];

/** Column vector of size: 200 */
mpc_controller g[ 200 ];

/** Column vector of size: 200 */
mpc_controller lb[ 200 ];

/** Column vector of size: 200 */
mpc_controller ub[ 200 ];

/** Column vector of size: 200 */
mpc_controller x[ 200 ];

/** Column vector of size: 200 */
mpc_controller y[ 200 ];


} ACADOworkspace;

/* 
 * Forward function declarations. 
 */


/** Performs the integration and sensitivity propagation for one shooting interval.
 *
 *  \param rk_eta Working array to pass the input values and return the results.
 *  \param resetIntegrator The internal memory of the integrator can be reset.
 *
 *  \return Status code of the integrator.
 */
int acado_integrate( mpc_controller* const rk_eta, int resetIntegrator );

/** Export of an ACADO symbolic function.
 *
 *  \param in Input to the exported function.
 *  \param out Output of the exported function.
 */
void acado_rhs_forw(const mpc_controller* in, mpc_controller* out);

/** Preparation step of the RTI scheme.
 *
 *  \return Status of the integration module. =0: OK, otherwise the error code.
 */
int acado_preparationStep(  );

/** Feedback/estimation step of the RTI scheme.
 *
 *  \return Status code of the qpOASES QP solver.
 */
int acado_feedbackStep(  );

/** Solver initialization. Must be called once before any other function call.
 *
 *  \return =0: OK, otherwise an error code of a QP solver.
 */
int acado_initializeSolver(  );

/** Initialize shooting nodes by a forward simulation starting from the first node.
 */
void acado_initializeNodesByForwardSimulation(  );

/** Shift differential variables vector by one interval.
 *
 *  \param strategy Shifting strategy: 1. Initialize node 101 with xEnd. 2. Initialize node 101 by forward simulation.
 *  \param xEnd Value for the x vector on the last node. If =0 the old value is used.
 *  \param uEnd Value for the u vector on the second to last node. If =0 the old value is used.
 */
void acado_shiftStates( int strategy, mpc_controller* const xEnd, mpc_controller* const uEnd );

/** Shift controls vector by one interval.
 *
 *  \param uEnd Value for the u vector on the second to last node. If =0 the old value is used.
 */
void acado_shiftControls( mpc_controller* const uEnd );

/** Get the KKT tolerance of the current iterate.
 *
 *  \return The KKT tolerance value.
 */
mpc_controller acado_getKKT(  );

/** Calculate the objective value.
 *
 *  \return Value of the objective function.
 */
mpc_controller acado_getObjective(  );


/* 
 * Extern declarations. 
 */

extern ACADOworkspace acadoWorkspace;
extern ACADOvariables acadoVariables;

/** @} */

#ifndef __MATLAB__
#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */
#endif /* __MATLAB__ */

#endif /* ACADO_COMMON_H */
