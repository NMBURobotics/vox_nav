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


#include "acado_common.h"




/******************************************************************************/
/*                                                                            */
/* ACADO code generation                                                      */
/*                                                                            */
/******************************************************************************/


int acado_modelSimulation(  )
{
int ret;

int lRun1;
ret = 0;
for (lRun1 = 0; lRun1 < 10; ++lRun1)
{
acadoWorkspace.state[0] = acadoVariables.x[lRun1 * 3];
acadoWorkspace.state[1] = acadoVariables.x[lRun1 * 3 + 1];
acadoWorkspace.state[2] = acadoVariables.x[lRun1 * 3 + 2];

acadoWorkspace.state[18] = acadoVariables.u[lRun1 * 2];
acadoWorkspace.state[19] = acadoVariables.u[lRun1 * 2 + 1];

ret = acado_integrate(acadoWorkspace.state, 1);

acadoWorkspace.d[lRun1 * 3] = acadoWorkspace.state[0] - acadoVariables.x[lRun1 * 3 + 3];
acadoWorkspace.d[lRun1 * 3 + 1] = acadoWorkspace.state[1] - acadoVariables.x[lRun1 * 3 + 4];
acadoWorkspace.d[lRun1 * 3 + 2] = acadoWorkspace.state[2] - acadoVariables.x[lRun1 * 3 + 5];

acadoWorkspace.evGx[lRun1 * 9] = acadoWorkspace.state[3];
acadoWorkspace.evGx[lRun1 * 9 + 1] = acadoWorkspace.state[4];
acadoWorkspace.evGx[lRun1 * 9 + 2] = acadoWorkspace.state[5];
acadoWorkspace.evGx[lRun1 * 9 + 3] = acadoWorkspace.state[6];
acadoWorkspace.evGx[lRun1 * 9 + 4] = acadoWorkspace.state[7];
acadoWorkspace.evGx[lRun1 * 9 + 5] = acadoWorkspace.state[8];
acadoWorkspace.evGx[lRun1 * 9 + 6] = acadoWorkspace.state[9];
acadoWorkspace.evGx[lRun1 * 9 + 7] = acadoWorkspace.state[10];
acadoWorkspace.evGx[lRun1 * 9 + 8] = acadoWorkspace.state[11];

acadoWorkspace.evGu[lRun1 * 6] = acadoWorkspace.state[12];
acadoWorkspace.evGu[lRun1 * 6 + 1] = acadoWorkspace.state[13];
acadoWorkspace.evGu[lRun1 * 6 + 2] = acadoWorkspace.state[14];
acadoWorkspace.evGu[lRun1 * 6 + 3] = acadoWorkspace.state[15];
acadoWorkspace.evGu[lRun1 * 6 + 4] = acadoWorkspace.state[16];
acadoWorkspace.evGu[lRun1 * 6 + 5] = acadoWorkspace.state[17];
}
return ret;
}

void acado_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
}

void acado_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
}

void acado_setObjQ1Q2( real_t* const tmpObjS, real_t* const tmpQ1, real_t* const tmpQ2 )
{
tmpQ2[0] = +tmpObjS[0];
tmpQ2[1] = +tmpObjS[1];
tmpQ2[2] = +tmpObjS[2];
tmpQ2[3] = +tmpObjS[3];
tmpQ2[4] = +tmpObjS[4];
tmpQ2[5] = +tmpObjS[5];
tmpQ2[6] = +tmpObjS[6];
tmpQ2[7] = +tmpObjS[7];
tmpQ2[8] = +tmpObjS[8];
tmpQ1[0] = + tmpQ2[0];
tmpQ1[1] = + tmpQ2[1];
tmpQ1[2] = + tmpQ2[2];
tmpQ1[3] = + tmpQ2[3];
tmpQ1[4] = + tmpQ2[4];
tmpQ1[5] = + tmpQ2[5];
tmpQ1[6] = + tmpQ2[6];
tmpQ1[7] = + tmpQ2[7];
tmpQ1[8] = + tmpQ2[8];
}

void acado_setObjQN1QN2( real_t* const tmpObjSEndTerm, real_t* const tmpQN1, real_t* const tmpQN2 )
{
tmpQN2[0] = +tmpObjSEndTerm[0];
tmpQN2[1] = +tmpObjSEndTerm[1];
tmpQN2[2] = +tmpObjSEndTerm[2];
tmpQN2[3] = +tmpObjSEndTerm[3];
tmpQN2[4] = +tmpObjSEndTerm[4];
tmpQN2[5] = +tmpObjSEndTerm[5];
tmpQN2[6] = +tmpObjSEndTerm[6];
tmpQN2[7] = +tmpObjSEndTerm[7];
tmpQN2[8] = +tmpObjSEndTerm[8];
tmpQN1[0] = + tmpQN2[0];
tmpQN1[1] = + tmpQN2[1];
tmpQN1[2] = + tmpQN2[2];
tmpQN1[3] = + tmpQN2[3];
tmpQN1[4] = + tmpQN2[4];
tmpQN1[5] = + tmpQN2[5];
tmpQN1[6] = + tmpQN2[6];
tmpQN1[7] = + tmpQN2[7];
tmpQN1[8] = + tmpQN2[8];
}

void acado_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 10; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj * 3];
acadoWorkspace.objValueIn[1] = acadoVariables.x[runObj * 3 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[runObj * 3 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.u[runObj * 2];
acadoWorkspace.objValueIn[4] = acadoVariables.u[runObj * 2 + 1];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[runObj * 3] = acadoWorkspace.objValueOut[0];
acadoWorkspace.Dy[runObj * 3 + 1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.Dy[runObj * 3 + 2] = acadoWorkspace.objValueOut[2];

acado_setObjQ1Q2( &(acadoVariables.W[ runObj * 9 ]), &(acadoWorkspace.Q1[ runObj * 9 ]), &(acadoWorkspace.Q2[ runObj * 9 ]) );

}
acadoWorkspace.objValueIn[0] = acadoVariables.x[30];
acadoWorkspace.objValueIn[1] = acadoVariables.x[31];
acadoWorkspace.objValueIn[2] = acadoVariables.x[32];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2];

acado_setObjQN1QN2( acadoVariables.WN, acadoWorkspace.QN1, acadoWorkspace.QN2 );

}

void acado_multGxd( real_t* const dOld, real_t* const Gx1, real_t* const dNew )
{
dNew[0] += + Gx1[0]*dOld[0] + Gx1[1]*dOld[1] + Gx1[2]*dOld[2];
dNew[1] += + Gx1[3]*dOld[0] + Gx1[4]*dOld[1] + Gx1[5]*dOld[2];
dNew[2] += + Gx1[6]*dOld[0] + Gx1[7]*dOld[1] + Gx1[8]*dOld[2];
}

void acado_moveGxT( real_t* const Gx1, real_t* const Gx2 )
{
Gx2[0] = Gx1[0];
Gx2[1] = Gx1[1];
Gx2[2] = Gx1[2];
Gx2[3] = Gx1[3];
Gx2[4] = Gx1[4];
Gx2[5] = Gx1[5];
Gx2[6] = Gx1[6];
Gx2[7] = Gx1[7];
Gx2[8] = Gx1[8];
}

void acado_multGxGx( real_t* const Gx1, real_t* const Gx2, real_t* const Gx3 )
{
Gx3[0] = + Gx1[0]*Gx2[0] + Gx1[1]*Gx2[3] + Gx1[2]*Gx2[6];
Gx3[1] = + Gx1[0]*Gx2[1] + Gx1[1]*Gx2[4] + Gx1[2]*Gx2[7];
Gx3[2] = + Gx1[0]*Gx2[2] + Gx1[1]*Gx2[5] + Gx1[2]*Gx2[8];
Gx3[3] = + Gx1[3]*Gx2[0] + Gx1[4]*Gx2[3] + Gx1[5]*Gx2[6];
Gx3[4] = + Gx1[3]*Gx2[1] + Gx1[4]*Gx2[4] + Gx1[5]*Gx2[7];
Gx3[5] = + Gx1[3]*Gx2[2] + Gx1[4]*Gx2[5] + Gx1[5]*Gx2[8];
Gx3[6] = + Gx1[6]*Gx2[0] + Gx1[7]*Gx2[3] + Gx1[8]*Gx2[6];
Gx3[7] = + Gx1[6]*Gx2[1] + Gx1[7]*Gx2[4] + Gx1[8]*Gx2[7];
Gx3[8] = + Gx1[6]*Gx2[2] + Gx1[7]*Gx2[5] + Gx1[8]*Gx2[8];
}

void acado_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[2] + Gx1[2]*Gu1[4];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[1]*Gu1[3] + Gx1[2]*Gu1[5];
Gu2[2] = + Gx1[3]*Gu1[0] + Gx1[4]*Gu1[2] + Gx1[5]*Gu1[4];
Gu2[3] = + Gx1[3]*Gu1[1] + Gx1[4]*Gu1[3] + Gx1[5]*Gu1[5];
Gu2[4] = + Gx1[6]*Gu1[0] + Gx1[7]*Gu1[2] + Gx1[8]*Gu1[4];
Gu2[5] = + Gx1[6]*Gu1[1] + Gx1[7]*Gu1[3] + Gx1[8]*Gu1[5];
}

void acado_moveGuE( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = Gu1[0];
Gu2[1] = Gu1[1];
Gu2[2] = Gu1[2];
Gu2[3] = Gu1[3];
Gu2[4] = Gu1[4];
Gu2[5] = Gu1[5];
}

void acado_setBlockH11( int iRow, int iCol, real_t* const Gu1, real_t* const Gu2 )
{
acadoWorkspace.H[(iRow * 40) + (iCol * 2)] += + Gu1[0]*Gu2[0] + Gu1[2]*Gu2[2] + Gu1[4]*Gu2[4];
acadoWorkspace.H[(iRow * 40) + (iCol * 2 + 1)] += + Gu1[0]*Gu2[1] + Gu1[2]*Gu2[3] + Gu1[4]*Gu2[5];
acadoWorkspace.H[(iRow * 40 + 20) + (iCol * 2)] += + Gu1[1]*Gu2[0] + Gu1[3]*Gu2[2] + Gu1[5]*Gu2[4];
acadoWorkspace.H[(iRow * 40 + 20) + (iCol * 2 + 1)] += + Gu1[1]*Gu2[1] + Gu1[3]*Gu2[3] + Gu1[5]*Gu2[5];
}

void acado_setBlockH11_R1( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 40) + (iCol * 2)] = + (real_t)1.0000000000000000e-03;
acadoWorkspace.H[(iRow * 40) + (iCol * 2 + 1)] = 0.0;
acadoWorkspace.H[(iRow * 40 + 20) + (iCol * 2)] = 0.0;
acadoWorkspace.H[(iRow * 40 + 20) + (iCol * 2 + 1)] = + (real_t)1.0000000000000000e-03;
}

void acado_zeroBlockH11( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 40) + (iCol * 2)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 40) + (iCol * 2 + 1)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 40 + 20) + (iCol * 2)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 40 + 20) + (iCol * 2 + 1)] = 0.0000000000000000e+00;
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 40) + (iCol * 2)] = acadoWorkspace.H[(iCol * 40) + (iRow * 2)];
acadoWorkspace.H[(iRow * 40) + (iCol * 2 + 1)] = acadoWorkspace.H[(iCol * 40 + 20) + (iRow * 2)];
acadoWorkspace.H[(iRow * 40 + 20) + (iCol * 2)] = acadoWorkspace.H[(iCol * 40) + (iRow * 2 + 1)];
acadoWorkspace.H[(iRow * 40 + 20) + (iCol * 2 + 1)] = acadoWorkspace.H[(iCol * 40 + 20) + (iRow * 2 + 1)];
}

void acado_multQ1d( real_t* const Gx1, real_t* const dOld, real_t* const dNew )
{
dNew[0] = + Gx1[0]*dOld[0] + Gx1[1]*dOld[1] + Gx1[2]*dOld[2];
dNew[1] = + Gx1[3]*dOld[0] + Gx1[4]*dOld[1] + Gx1[5]*dOld[2];
dNew[2] = + Gx1[6]*dOld[0] + Gx1[7]*dOld[1] + Gx1[8]*dOld[2];
}

void acado_multQN1d( real_t* const QN1, real_t* const dOld, real_t* const dNew )
{
dNew[0] = + acadoWorkspace.QN1[0]*dOld[0] + acadoWorkspace.QN1[1]*dOld[1] + acadoWorkspace.QN1[2]*dOld[2];
dNew[1] = + acadoWorkspace.QN1[3]*dOld[0] + acadoWorkspace.QN1[4]*dOld[1] + acadoWorkspace.QN1[5]*dOld[2];
dNew[2] = + acadoWorkspace.QN1[6]*dOld[0] + acadoWorkspace.QN1[7]*dOld[1] + acadoWorkspace.QN1[8]*dOld[2];
}

void acado_multRDy( real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = 0.0;
;
RDy1[1] = 0.0;
;
}

void acado_multQDy( real_t* const Q2, real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = + Q2[0]*Dy1[0] + Q2[1]*Dy1[1] + Q2[2]*Dy1[2];
QDy1[1] = + Q2[3]*Dy1[0] + Q2[4]*Dy1[1] + Q2[5]*Dy1[2];
QDy1[2] = + Q2[6]*Dy1[0] + Q2[7]*Dy1[1] + Q2[8]*Dy1[2];
}

void acado_multEQDy( real_t* const E1, real_t* const QDy1, real_t* const U1 )
{
U1[0] += + E1[0]*QDy1[0] + E1[2]*QDy1[1] + E1[4]*QDy1[2];
U1[1] += + E1[1]*QDy1[0] + E1[3]*QDy1[1] + E1[5]*QDy1[2];
}

void acado_multQETGx( real_t* const E1, real_t* const Gx1, real_t* const H101 )
{
H101[0] += + E1[0]*Gx1[0] + E1[2]*Gx1[3] + E1[4]*Gx1[6];
H101[1] += + E1[0]*Gx1[1] + E1[2]*Gx1[4] + E1[4]*Gx1[7];
H101[2] += + E1[0]*Gx1[2] + E1[2]*Gx1[5] + E1[4]*Gx1[8];
H101[3] += + E1[1]*Gx1[0] + E1[3]*Gx1[3] + E1[5]*Gx1[6];
H101[4] += + E1[1]*Gx1[1] + E1[3]*Gx1[4] + E1[5]*Gx1[7];
H101[5] += + E1[1]*Gx1[2] + E1[3]*Gx1[5] + E1[5]*Gx1[8];
}

void acado_zeroBlockH10( real_t* const H101 )
{
{ int lCopy; for (lCopy = 0; lCopy < 6; lCopy++) H101[ lCopy ] = 0; }
}

void acado_multEDu( real_t* const E1, real_t* const U1, real_t* const dNew )
{
dNew[0] += + E1[0]*U1[0] + E1[1]*U1[1];
dNew[1] += + E1[2]*U1[0] + E1[3]*U1[1];
dNew[2] += + E1[4]*U1[0] + E1[5]*U1[1];
}

void acado_macETSlu( real_t* const E0, real_t* const g1 )
{
g1[0] += 0.0;
;
g1[1] += 0.0;
;
}

void acado_condensePrep(  )
{
acado_moveGuE( acadoWorkspace.evGu, acadoWorkspace.E );
acado_moveGxT( &(acadoWorkspace.evGx[ 9 ]), acadoWorkspace.T );
acado_multGxd( acadoWorkspace.d, &(acadoWorkspace.evGx[ 9 ]), &(acadoWorkspace.d[ 3 ]) );
acado_multGxGx( acadoWorkspace.T, acadoWorkspace.evGx, &(acadoWorkspace.evGx[ 9 ]) );

acado_multGxGu( acadoWorkspace.T, acadoWorkspace.E, &(acadoWorkspace.E[ 6 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 6 ]), &(acadoWorkspace.E[ 12 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 18 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 3 ]), &(acadoWorkspace.evGx[ 18 ]), &(acadoWorkspace.d[ 6 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 9 ]), &(acadoWorkspace.evGx[ 18 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 6 ]), &(acadoWorkspace.E[ 18 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 12 ]), &(acadoWorkspace.E[ 24 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 12 ]), &(acadoWorkspace.E[ 30 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 27 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 6 ]), &(acadoWorkspace.evGx[ 27 ]), &(acadoWorkspace.d[ 9 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 18 ]), &(acadoWorkspace.evGx[ 27 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 18 ]), &(acadoWorkspace.E[ 36 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 24 ]), &(acadoWorkspace.E[ 42 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 30 ]), &(acadoWorkspace.E[ 48 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 18 ]), &(acadoWorkspace.E[ 54 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 36 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 9 ]), &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.d[ 12 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 27 ]), &(acadoWorkspace.evGx[ 36 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 36 ]), &(acadoWorkspace.E[ 60 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 42 ]), &(acadoWorkspace.E[ 66 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.E[ 72 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 54 ]), &(acadoWorkspace.E[ 78 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 24 ]), &(acadoWorkspace.E[ 84 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 45 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 12 ]), &(acadoWorkspace.evGx[ 45 ]), &(acadoWorkspace.d[ 15 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.evGx[ 45 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.E[ 90 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 66 ]), &(acadoWorkspace.E[ 96 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.E[ 102 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 78 ]), &(acadoWorkspace.E[ 108 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.E[ 114 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 30 ]), &(acadoWorkspace.E[ 120 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 54 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 15 ]), &(acadoWorkspace.evGx[ 54 ]), &(acadoWorkspace.d[ 18 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 45 ]), &(acadoWorkspace.evGx[ 54 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.E[ 126 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.E[ 132 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 102 ]), &(acadoWorkspace.E[ 138 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.E[ 144 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 114 ]), &(acadoWorkspace.E[ 150 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.E[ 156 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 36 ]), &(acadoWorkspace.E[ 162 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 63 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 18 ]), &(acadoWorkspace.evGx[ 63 ]), &(acadoWorkspace.d[ 21 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 54 ]), &(acadoWorkspace.evGx[ 63 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 126 ]), &(acadoWorkspace.E[ 168 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 132 ]), &(acadoWorkspace.E[ 174 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 138 ]), &(acadoWorkspace.E[ 180 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.E[ 186 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 150 ]), &(acadoWorkspace.E[ 192 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 156 ]), &(acadoWorkspace.E[ 198 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 162 ]), &(acadoWorkspace.E[ 204 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 42 ]), &(acadoWorkspace.E[ 210 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 72 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 21 ]), &(acadoWorkspace.evGx[ 72 ]), &(acadoWorkspace.d[ 24 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 63 ]), &(acadoWorkspace.evGx[ 72 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.E[ 216 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 174 ]), &(acadoWorkspace.E[ 222 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.E[ 228 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 186 ]), &(acadoWorkspace.E[ 234 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.E[ 240 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 198 ]), &(acadoWorkspace.E[ 246 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 204 ]), &(acadoWorkspace.E[ 252 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 210 ]), &(acadoWorkspace.E[ 258 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 48 ]), &(acadoWorkspace.E[ 264 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 81 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 24 ]), &(acadoWorkspace.evGx[ 81 ]), &(acadoWorkspace.d[ 27 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 72 ]), &(acadoWorkspace.evGx[ 81 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.E[ 270 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 222 ]), &(acadoWorkspace.E[ 276 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 228 ]), &(acadoWorkspace.E[ 282 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 234 ]), &(acadoWorkspace.E[ 288 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.E[ 294 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 246 ]), &(acadoWorkspace.E[ 300 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 252 ]), &(acadoWorkspace.E[ 306 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 258 ]), &(acadoWorkspace.E[ 312 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 264 ]), &(acadoWorkspace.E[ 318 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 54 ]), &(acadoWorkspace.E[ 324 ]) );

acado_multGxGu( &(acadoWorkspace.Q1[ 9 ]), acadoWorkspace.E, acadoWorkspace.QE );
acado_multGxGu( &(acadoWorkspace.Q1[ 18 ]), &(acadoWorkspace.E[ 6 ]), &(acadoWorkspace.QE[ 6 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 18 ]), &(acadoWorkspace.E[ 12 ]), &(acadoWorkspace.QE[ 12 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 27 ]), &(acadoWorkspace.E[ 18 ]), &(acadoWorkspace.QE[ 18 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 27 ]), &(acadoWorkspace.E[ 24 ]), &(acadoWorkspace.QE[ 24 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 27 ]), &(acadoWorkspace.E[ 30 ]), &(acadoWorkspace.QE[ 30 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 36 ]), &(acadoWorkspace.E[ 36 ]), &(acadoWorkspace.QE[ 36 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 36 ]), &(acadoWorkspace.E[ 42 ]), &(acadoWorkspace.QE[ 42 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 36 ]), &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QE[ 48 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 36 ]), &(acadoWorkspace.E[ 54 ]), &(acadoWorkspace.QE[ 54 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 45 ]), &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.QE[ 60 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 45 ]), &(acadoWorkspace.E[ 66 ]), &(acadoWorkspace.QE[ 66 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 45 ]), &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.QE[ 72 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 45 ]), &(acadoWorkspace.E[ 78 ]), &(acadoWorkspace.QE[ 78 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 45 ]), &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.QE[ 84 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 54 ]), &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.QE[ 90 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 54 ]), &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QE[ 96 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 54 ]), &(acadoWorkspace.E[ 102 ]), &(acadoWorkspace.QE[ 102 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 54 ]), &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.QE[ 108 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 54 ]), &(acadoWorkspace.E[ 114 ]), &(acadoWorkspace.QE[ 114 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 54 ]), &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QE[ 120 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 63 ]), &(acadoWorkspace.E[ 126 ]), &(acadoWorkspace.QE[ 126 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 63 ]), &(acadoWorkspace.E[ 132 ]), &(acadoWorkspace.QE[ 132 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 63 ]), &(acadoWorkspace.E[ 138 ]), &(acadoWorkspace.QE[ 138 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 63 ]), &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QE[ 144 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 63 ]), &(acadoWorkspace.E[ 150 ]), &(acadoWorkspace.QE[ 150 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 63 ]), &(acadoWorkspace.E[ 156 ]), &(acadoWorkspace.QE[ 156 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 63 ]), &(acadoWorkspace.E[ 162 ]), &(acadoWorkspace.QE[ 162 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 72 ]), &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.QE[ 168 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 72 ]), &(acadoWorkspace.E[ 174 ]), &(acadoWorkspace.QE[ 174 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 72 ]), &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 180 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 72 ]), &(acadoWorkspace.E[ 186 ]), &(acadoWorkspace.QE[ 186 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 72 ]), &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QE[ 192 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 72 ]), &(acadoWorkspace.E[ 198 ]), &(acadoWorkspace.QE[ 198 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 72 ]), &(acadoWorkspace.E[ 204 ]), &(acadoWorkspace.QE[ 204 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 72 ]), &(acadoWorkspace.E[ 210 ]), &(acadoWorkspace.QE[ 210 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 81 ]), &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.QE[ 216 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 81 ]), &(acadoWorkspace.E[ 222 ]), &(acadoWorkspace.QE[ 222 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 81 ]), &(acadoWorkspace.E[ 228 ]), &(acadoWorkspace.QE[ 228 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 81 ]), &(acadoWorkspace.E[ 234 ]), &(acadoWorkspace.QE[ 234 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 81 ]), &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 240 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 81 ]), &(acadoWorkspace.E[ 246 ]), &(acadoWorkspace.QE[ 246 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 81 ]), &(acadoWorkspace.E[ 252 ]), &(acadoWorkspace.QE[ 252 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 81 ]), &(acadoWorkspace.E[ 258 ]), &(acadoWorkspace.QE[ 258 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 81 ]), &(acadoWorkspace.E[ 264 ]), &(acadoWorkspace.QE[ 264 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 270 ]), &(acadoWorkspace.QE[ 270 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.QE[ 276 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 282 ]), &(acadoWorkspace.QE[ 282 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 288 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 294 ]), &(acadoWorkspace.QE[ 294 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.QE[ 300 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 306 ]), &(acadoWorkspace.QE[ 306 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 312 ]), &(acadoWorkspace.QE[ 312 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 318 ]), &(acadoWorkspace.QE[ 318 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 324 ]), &(acadoWorkspace.QE[ 324 ]) );

acado_zeroBlockH10( acadoWorkspace.H10 );
acado_multQETGx( acadoWorkspace.QE, acadoWorkspace.evGx, acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 6 ]), &(acadoWorkspace.evGx[ 9 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 18 ]), &(acadoWorkspace.evGx[ 18 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 36 ]), &(acadoWorkspace.evGx[ 27 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 60 ]), &(acadoWorkspace.evGx[ 36 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 90 ]), &(acadoWorkspace.evGx[ 45 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 126 ]), &(acadoWorkspace.evGx[ 54 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 168 ]), &(acadoWorkspace.evGx[ 63 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 216 ]), &(acadoWorkspace.evGx[ 72 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 270 ]), &(acadoWorkspace.evGx[ 81 ]), acadoWorkspace.H10 );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 6 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 12 ]), &(acadoWorkspace.evGx[ 9 ]), &(acadoWorkspace.H10[ 6 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 24 ]), &(acadoWorkspace.evGx[ 18 ]), &(acadoWorkspace.H10[ 6 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 42 ]), &(acadoWorkspace.evGx[ 27 ]), &(acadoWorkspace.H10[ 6 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 66 ]), &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.H10[ 6 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 96 ]), &(acadoWorkspace.evGx[ 45 ]), &(acadoWorkspace.H10[ 6 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 132 ]), &(acadoWorkspace.evGx[ 54 ]), &(acadoWorkspace.H10[ 6 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 174 ]), &(acadoWorkspace.evGx[ 63 ]), &(acadoWorkspace.H10[ 6 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 222 ]), &(acadoWorkspace.evGx[ 72 ]), &(acadoWorkspace.H10[ 6 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 276 ]), &(acadoWorkspace.evGx[ 81 ]), &(acadoWorkspace.H10[ 6 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 30 ]), &(acadoWorkspace.evGx[ 18 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 48 ]), &(acadoWorkspace.evGx[ 27 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 72 ]), &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 102 ]), &(acadoWorkspace.evGx[ 45 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 138 ]), &(acadoWorkspace.evGx[ 54 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 180 ]), &(acadoWorkspace.evGx[ 63 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 228 ]), &(acadoWorkspace.evGx[ 72 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 282 ]), &(acadoWorkspace.evGx[ 81 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 18 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 54 ]), &(acadoWorkspace.evGx[ 27 ]), &(acadoWorkspace.H10[ 18 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 78 ]), &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.H10[ 18 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 108 ]), &(acadoWorkspace.evGx[ 45 ]), &(acadoWorkspace.H10[ 18 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 144 ]), &(acadoWorkspace.evGx[ 54 ]), &(acadoWorkspace.H10[ 18 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 186 ]), &(acadoWorkspace.evGx[ 63 ]), &(acadoWorkspace.H10[ 18 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 234 ]), &(acadoWorkspace.evGx[ 72 ]), &(acadoWorkspace.H10[ 18 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 288 ]), &(acadoWorkspace.evGx[ 81 ]), &(acadoWorkspace.H10[ 18 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 84 ]), &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 114 ]), &(acadoWorkspace.evGx[ 45 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 150 ]), &(acadoWorkspace.evGx[ 54 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 192 ]), &(acadoWorkspace.evGx[ 63 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 240 ]), &(acadoWorkspace.evGx[ 72 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 294 ]), &(acadoWorkspace.evGx[ 81 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 30 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 120 ]), &(acadoWorkspace.evGx[ 45 ]), &(acadoWorkspace.H10[ 30 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 156 ]), &(acadoWorkspace.evGx[ 54 ]), &(acadoWorkspace.H10[ 30 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 198 ]), &(acadoWorkspace.evGx[ 63 ]), &(acadoWorkspace.H10[ 30 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 246 ]), &(acadoWorkspace.evGx[ 72 ]), &(acadoWorkspace.H10[ 30 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 300 ]), &(acadoWorkspace.evGx[ 81 ]), &(acadoWorkspace.H10[ 30 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 36 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 162 ]), &(acadoWorkspace.evGx[ 54 ]), &(acadoWorkspace.H10[ 36 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 204 ]), &(acadoWorkspace.evGx[ 63 ]), &(acadoWorkspace.H10[ 36 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 252 ]), &(acadoWorkspace.evGx[ 72 ]), &(acadoWorkspace.H10[ 36 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 306 ]), &(acadoWorkspace.evGx[ 81 ]), &(acadoWorkspace.H10[ 36 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 42 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 210 ]), &(acadoWorkspace.evGx[ 63 ]), &(acadoWorkspace.H10[ 42 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 258 ]), &(acadoWorkspace.evGx[ 72 ]), &(acadoWorkspace.H10[ 42 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 312 ]), &(acadoWorkspace.evGx[ 81 ]), &(acadoWorkspace.H10[ 42 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 264 ]), &(acadoWorkspace.evGx[ 72 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 318 ]), &(acadoWorkspace.evGx[ 81 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 54 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 324 ]), &(acadoWorkspace.evGx[ 81 ]), &(acadoWorkspace.H10[ 54 ]) );

acado_setBlockH11_R1( 0, 0 );
acado_setBlockH11( 0, 0, acadoWorkspace.E, acadoWorkspace.QE );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 6 ]), &(acadoWorkspace.QE[ 6 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 18 ]), &(acadoWorkspace.QE[ 18 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 36 ]), &(acadoWorkspace.QE[ 36 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.QE[ 60 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.QE[ 90 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 126 ]), &(acadoWorkspace.QE[ 126 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.QE[ 168 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.QE[ 216 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 270 ]), &(acadoWorkspace.QE[ 270 ]) );

acado_zeroBlockH11( 0, 1 );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 6 ]), &(acadoWorkspace.QE[ 12 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 18 ]), &(acadoWorkspace.QE[ 24 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 36 ]), &(acadoWorkspace.QE[ 42 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.QE[ 66 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.QE[ 96 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 126 ]), &(acadoWorkspace.QE[ 132 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.QE[ 174 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.QE[ 222 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 270 ]), &(acadoWorkspace.QE[ 276 ]) );

acado_zeroBlockH11( 0, 2 );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 18 ]), &(acadoWorkspace.QE[ 30 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 36 ]), &(acadoWorkspace.QE[ 48 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.QE[ 72 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.QE[ 102 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 126 ]), &(acadoWorkspace.QE[ 138 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.QE[ 180 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.QE[ 228 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 270 ]), &(acadoWorkspace.QE[ 282 ]) );

acado_zeroBlockH11( 0, 3 );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 36 ]), &(acadoWorkspace.QE[ 54 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.QE[ 78 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.QE[ 108 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 126 ]), &(acadoWorkspace.QE[ 144 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.QE[ 186 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.QE[ 234 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 270 ]), &(acadoWorkspace.QE[ 288 ]) );

acado_zeroBlockH11( 0, 4 );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.QE[ 84 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.QE[ 114 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 126 ]), &(acadoWorkspace.QE[ 150 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.QE[ 192 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.QE[ 240 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 270 ]), &(acadoWorkspace.QE[ 294 ]) );

acado_zeroBlockH11( 0, 5 );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.QE[ 120 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 126 ]), &(acadoWorkspace.QE[ 156 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.QE[ 198 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.QE[ 246 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 270 ]), &(acadoWorkspace.QE[ 300 ]) );

acado_zeroBlockH11( 0, 6 );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 126 ]), &(acadoWorkspace.QE[ 162 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.QE[ 204 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.QE[ 252 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 270 ]), &(acadoWorkspace.QE[ 306 ]) );

acado_zeroBlockH11( 0, 7 );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.QE[ 210 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.QE[ 258 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 270 ]), &(acadoWorkspace.QE[ 312 ]) );

acado_zeroBlockH11( 0, 8 );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.QE[ 264 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 270 ]), &(acadoWorkspace.QE[ 318 ]) );

acado_zeroBlockH11( 0, 9 );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 270 ]), &(acadoWorkspace.QE[ 324 ]) );

acado_setBlockH11_R1( 1, 1 );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 12 ]), &(acadoWorkspace.QE[ 12 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 24 ]), &(acadoWorkspace.QE[ 24 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 42 ]), &(acadoWorkspace.QE[ 42 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 66 ]), &(acadoWorkspace.QE[ 66 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QE[ 96 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 132 ]), &(acadoWorkspace.QE[ 132 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 174 ]), &(acadoWorkspace.QE[ 174 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 222 ]), &(acadoWorkspace.QE[ 222 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.QE[ 276 ]) );

acado_zeroBlockH11( 1, 2 );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 24 ]), &(acadoWorkspace.QE[ 30 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 42 ]), &(acadoWorkspace.QE[ 48 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 66 ]), &(acadoWorkspace.QE[ 72 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QE[ 102 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 132 ]), &(acadoWorkspace.QE[ 138 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 174 ]), &(acadoWorkspace.QE[ 180 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 222 ]), &(acadoWorkspace.QE[ 228 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.QE[ 282 ]) );

acado_zeroBlockH11( 1, 3 );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 42 ]), &(acadoWorkspace.QE[ 54 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 66 ]), &(acadoWorkspace.QE[ 78 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QE[ 108 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 132 ]), &(acadoWorkspace.QE[ 144 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 174 ]), &(acadoWorkspace.QE[ 186 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 222 ]), &(acadoWorkspace.QE[ 234 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.QE[ 288 ]) );

acado_zeroBlockH11( 1, 4 );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 66 ]), &(acadoWorkspace.QE[ 84 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QE[ 114 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 132 ]), &(acadoWorkspace.QE[ 150 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 174 ]), &(acadoWorkspace.QE[ 192 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 222 ]), &(acadoWorkspace.QE[ 240 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.QE[ 294 ]) );

acado_zeroBlockH11( 1, 5 );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QE[ 120 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 132 ]), &(acadoWorkspace.QE[ 156 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 174 ]), &(acadoWorkspace.QE[ 198 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 222 ]), &(acadoWorkspace.QE[ 246 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.QE[ 300 ]) );

acado_zeroBlockH11( 1, 6 );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 132 ]), &(acadoWorkspace.QE[ 162 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 174 ]), &(acadoWorkspace.QE[ 204 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 222 ]), &(acadoWorkspace.QE[ 252 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.QE[ 306 ]) );

acado_zeroBlockH11( 1, 7 );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 174 ]), &(acadoWorkspace.QE[ 210 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 222 ]), &(acadoWorkspace.QE[ 258 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.QE[ 312 ]) );

acado_zeroBlockH11( 1, 8 );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 222 ]), &(acadoWorkspace.QE[ 264 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.QE[ 318 ]) );

acado_zeroBlockH11( 1, 9 );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.QE[ 324 ]) );

acado_setBlockH11_R1( 2, 2 );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 30 ]), &(acadoWorkspace.QE[ 30 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QE[ 48 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.QE[ 72 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 102 ]), &(acadoWorkspace.QE[ 102 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 138 ]), &(acadoWorkspace.QE[ 138 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 180 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 228 ]), &(acadoWorkspace.QE[ 228 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 282 ]), &(acadoWorkspace.QE[ 282 ]) );

acado_zeroBlockH11( 2, 3 );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QE[ 54 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.QE[ 78 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 102 ]), &(acadoWorkspace.QE[ 108 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 138 ]), &(acadoWorkspace.QE[ 144 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 186 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 228 ]), &(acadoWorkspace.QE[ 234 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 282 ]), &(acadoWorkspace.QE[ 288 ]) );

acado_zeroBlockH11( 2, 4 );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.QE[ 84 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 102 ]), &(acadoWorkspace.QE[ 114 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 138 ]), &(acadoWorkspace.QE[ 150 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 192 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 228 ]), &(acadoWorkspace.QE[ 240 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 282 ]), &(acadoWorkspace.QE[ 294 ]) );

acado_zeroBlockH11( 2, 5 );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 102 ]), &(acadoWorkspace.QE[ 120 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 138 ]), &(acadoWorkspace.QE[ 156 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 198 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 228 ]), &(acadoWorkspace.QE[ 246 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 282 ]), &(acadoWorkspace.QE[ 300 ]) );

acado_zeroBlockH11( 2, 6 );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 138 ]), &(acadoWorkspace.QE[ 162 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 204 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 228 ]), &(acadoWorkspace.QE[ 252 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 282 ]), &(acadoWorkspace.QE[ 306 ]) );

acado_zeroBlockH11( 2, 7 );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 210 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 228 ]), &(acadoWorkspace.QE[ 258 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 282 ]), &(acadoWorkspace.QE[ 312 ]) );

acado_zeroBlockH11( 2, 8 );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 228 ]), &(acadoWorkspace.QE[ 264 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 282 ]), &(acadoWorkspace.QE[ 318 ]) );

acado_zeroBlockH11( 2, 9 );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 282 ]), &(acadoWorkspace.QE[ 324 ]) );

acado_setBlockH11_R1( 3, 3 );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 54 ]), &(acadoWorkspace.QE[ 54 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 78 ]), &(acadoWorkspace.QE[ 78 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.QE[ 108 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QE[ 144 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 186 ]), &(acadoWorkspace.QE[ 186 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 234 ]), &(acadoWorkspace.QE[ 234 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 288 ]) );

acado_zeroBlockH11( 3, 4 );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 78 ]), &(acadoWorkspace.QE[ 84 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.QE[ 114 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QE[ 150 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 186 ]), &(acadoWorkspace.QE[ 192 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 234 ]), &(acadoWorkspace.QE[ 240 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 294 ]) );

acado_zeroBlockH11( 3, 5 );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.QE[ 120 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QE[ 156 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 186 ]), &(acadoWorkspace.QE[ 198 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 234 ]), &(acadoWorkspace.QE[ 246 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 300 ]) );

acado_zeroBlockH11( 3, 6 );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QE[ 162 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 186 ]), &(acadoWorkspace.QE[ 204 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 234 ]), &(acadoWorkspace.QE[ 252 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 306 ]) );

acado_zeroBlockH11( 3, 7 );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 186 ]), &(acadoWorkspace.QE[ 210 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 234 ]), &(acadoWorkspace.QE[ 258 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 312 ]) );

acado_zeroBlockH11( 3, 8 );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 234 ]), &(acadoWorkspace.QE[ 264 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 318 ]) );

acado_zeroBlockH11( 3, 9 );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 324 ]) );

acado_setBlockH11_R1( 4, 4 );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.QE[ 84 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 114 ]), &(acadoWorkspace.QE[ 114 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 150 ]), &(acadoWorkspace.QE[ 150 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QE[ 192 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 240 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 294 ]), &(acadoWorkspace.QE[ 294 ]) );

acado_zeroBlockH11( 4, 5 );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 114 ]), &(acadoWorkspace.QE[ 120 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 150 ]), &(acadoWorkspace.QE[ 156 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QE[ 198 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 246 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 294 ]), &(acadoWorkspace.QE[ 300 ]) );

acado_zeroBlockH11( 4, 6 );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 150 ]), &(acadoWorkspace.QE[ 162 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QE[ 204 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 252 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 294 ]), &(acadoWorkspace.QE[ 306 ]) );

acado_zeroBlockH11( 4, 7 );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QE[ 210 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 258 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 294 ]), &(acadoWorkspace.QE[ 312 ]) );

acado_zeroBlockH11( 4, 8 );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 264 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 294 ]), &(acadoWorkspace.QE[ 318 ]) );

acado_zeroBlockH11( 4, 9 );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 294 ]), &(acadoWorkspace.QE[ 324 ]) );

acado_setBlockH11_R1( 5, 5 );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QE[ 120 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 156 ]), &(acadoWorkspace.QE[ 156 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 198 ]), &(acadoWorkspace.QE[ 198 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 246 ]), &(acadoWorkspace.QE[ 246 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.QE[ 300 ]) );

acado_zeroBlockH11( 5, 6 );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 156 ]), &(acadoWorkspace.QE[ 162 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 198 ]), &(acadoWorkspace.QE[ 204 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 246 ]), &(acadoWorkspace.QE[ 252 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.QE[ 306 ]) );

acado_zeroBlockH11( 5, 7 );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 198 ]), &(acadoWorkspace.QE[ 210 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 246 ]), &(acadoWorkspace.QE[ 258 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.QE[ 312 ]) );

acado_zeroBlockH11( 5, 8 );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 246 ]), &(acadoWorkspace.QE[ 264 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.QE[ 318 ]) );

acado_zeroBlockH11( 5, 9 );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.QE[ 324 ]) );

acado_setBlockH11_R1( 6, 6 );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 162 ]), &(acadoWorkspace.QE[ 162 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 204 ]), &(acadoWorkspace.QE[ 204 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 252 ]), &(acadoWorkspace.QE[ 252 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 306 ]), &(acadoWorkspace.QE[ 306 ]) );

acado_zeroBlockH11( 6, 7 );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 204 ]), &(acadoWorkspace.QE[ 210 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 252 ]), &(acadoWorkspace.QE[ 258 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 306 ]), &(acadoWorkspace.QE[ 312 ]) );

acado_zeroBlockH11( 6, 8 );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 252 ]), &(acadoWorkspace.QE[ 264 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 306 ]), &(acadoWorkspace.QE[ 318 ]) );

acado_zeroBlockH11( 6, 9 );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 306 ]), &(acadoWorkspace.QE[ 324 ]) );

acado_setBlockH11_R1( 7, 7 );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 210 ]), &(acadoWorkspace.QE[ 210 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 258 ]), &(acadoWorkspace.QE[ 258 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 312 ]), &(acadoWorkspace.QE[ 312 ]) );

acado_zeroBlockH11( 7, 8 );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 258 ]), &(acadoWorkspace.QE[ 264 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 312 ]), &(acadoWorkspace.QE[ 318 ]) );

acado_zeroBlockH11( 7, 9 );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 312 ]), &(acadoWorkspace.QE[ 324 ]) );

acado_setBlockH11_R1( 8, 8 );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 264 ]), &(acadoWorkspace.QE[ 264 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 318 ]), &(acadoWorkspace.QE[ 318 ]) );

acado_zeroBlockH11( 8, 9 );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 318 ]), &(acadoWorkspace.QE[ 324 ]) );

acado_setBlockH11_R1( 9, 9 );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 324 ]), &(acadoWorkspace.QE[ 324 ]) );


acado_copyHTH( 1, 0 );
acado_copyHTH( 2, 0 );
acado_copyHTH( 2, 1 );
acado_copyHTH( 3, 0 );
acado_copyHTH( 3, 1 );
acado_copyHTH( 3, 2 );
acado_copyHTH( 4, 0 );
acado_copyHTH( 4, 1 );
acado_copyHTH( 4, 2 );
acado_copyHTH( 4, 3 );
acado_copyHTH( 5, 0 );
acado_copyHTH( 5, 1 );
acado_copyHTH( 5, 2 );
acado_copyHTH( 5, 3 );
acado_copyHTH( 5, 4 );
acado_copyHTH( 6, 0 );
acado_copyHTH( 6, 1 );
acado_copyHTH( 6, 2 );
acado_copyHTH( 6, 3 );
acado_copyHTH( 6, 4 );
acado_copyHTH( 6, 5 );
acado_copyHTH( 7, 0 );
acado_copyHTH( 7, 1 );
acado_copyHTH( 7, 2 );
acado_copyHTH( 7, 3 );
acado_copyHTH( 7, 4 );
acado_copyHTH( 7, 5 );
acado_copyHTH( 7, 6 );
acado_copyHTH( 8, 0 );
acado_copyHTH( 8, 1 );
acado_copyHTH( 8, 2 );
acado_copyHTH( 8, 3 );
acado_copyHTH( 8, 4 );
acado_copyHTH( 8, 5 );
acado_copyHTH( 8, 6 );
acado_copyHTH( 8, 7 );
acado_copyHTH( 9, 0 );
acado_copyHTH( 9, 1 );
acado_copyHTH( 9, 2 );
acado_copyHTH( 9, 3 );
acado_copyHTH( 9, 4 );
acado_copyHTH( 9, 5 );
acado_copyHTH( 9, 6 );
acado_copyHTH( 9, 7 );
acado_copyHTH( 9, 8 );

acado_multQ1d( &(acadoWorkspace.Q1[ 9 ]), acadoWorkspace.d, acadoWorkspace.Qd );
acado_multQ1d( &(acadoWorkspace.Q1[ 18 ]), &(acadoWorkspace.d[ 3 ]), &(acadoWorkspace.Qd[ 3 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 27 ]), &(acadoWorkspace.d[ 6 ]), &(acadoWorkspace.Qd[ 6 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 36 ]), &(acadoWorkspace.d[ 9 ]), &(acadoWorkspace.Qd[ 9 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 45 ]), &(acadoWorkspace.d[ 12 ]), &(acadoWorkspace.Qd[ 12 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 54 ]), &(acadoWorkspace.d[ 15 ]), &(acadoWorkspace.Qd[ 15 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 63 ]), &(acadoWorkspace.d[ 18 ]), &(acadoWorkspace.Qd[ 18 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 72 ]), &(acadoWorkspace.d[ 21 ]), &(acadoWorkspace.Qd[ 21 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 81 ]), &(acadoWorkspace.d[ 24 ]), &(acadoWorkspace.Qd[ 24 ]) );
acado_multQN1d( acadoWorkspace.QN1, &(acadoWorkspace.d[ 27 ]), &(acadoWorkspace.Qd[ 27 ]) );

acado_macETSlu( acadoWorkspace.QE, acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 6 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 18 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 36 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 60 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 90 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 126 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 168 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 216 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 270 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 12 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 24 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 42 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 66 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 96 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 132 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 174 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 222 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 276 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 30 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 48 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 72 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 102 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 138 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 180 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 228 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 282 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 54 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 78 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 108 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 144 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 186 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 234 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 288 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 84 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 114 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 150 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 192 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 240 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 294 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 120 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 156 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 198 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 246 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 300 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 162 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 204 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 252 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 306 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 210 ]), &(acadoWorkspace.g[ 14 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 258 ]), &(acadoWorkspace.g[ 14 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 312 ]), &(acadoWorkspace.g[ 14 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 264 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 318 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 324 ]), &(acadoWorkspace.g[ 18 ]) );
}

void acado_condenseFdb(  )
{
acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
acadoWorkspace.Dx0[1] = acadoVariables.x0[1] - acadoVariables.x[1];
acadoWorkspace.Dx0[2] = acadoVariables.x0[2] - acadoVariables.x[2];

acadoWorkspace.Dy[0] -= acadoVariables.y[0];
acadoWorkspace.Dy[1] -= acadoVariables.y[1];
acadoWorkspace.Dy[2] -= acadoVariables.y[2];
acadoWorkspace.Dy[3] -= acadoVariables.y[3];
acadoWorkspace.Dy[4] -= acadoVariables.y[4];
acadoWorkspace.Dy[5] -= acadoVariables.y[5];
acadoWorkspace.Dy[6] -= acadoVariables.y[6];
acadoWorkspace.Dy[7] -= acadoVariables.y[7];
acadoWorkspace.Dy[8] -= acadoVariables.y[8];
acadoWorkspace.Dy[9] -= acadoVariables.y[9];
acadoWorkspace.Dy[10] -= acadoVariables.y[10];
acadoWorkspace.Dy[11] -= acadoVariables.y[11];
acadoWorkspace.Dy[12] -= acadoVariables.y[12];
acadoWorkspace.Dy[13] -= acadoVariables.y[13];
acadoWorkspace.Dy[14] -= acadoVariables.y[14];
acadoWorkspace.Dy[15] -= acadoVariables.y[15];
acadoWorkspace.Dy[16] -= acadoVariables.y[16];
acadoWorkspace.Dy[17] -= acadoVariables.y[17];
acadoWorkspace.Dy[18] -= acadoVariables.y[18];
acadoWorkspace.Dy[19] -= acadoVariables.y[19];
acadoWorkspace.Dy[20] -= acadoVariables.y[20];
acadoWorkspace.Dy[21] -= acadoVariables.y[21];
acadoWorkspace.Dy[22] -= acadoVariables.y[22];
acadoWorkspace.Dy[23] -= acadoVariables.y[23];
acadoWorkspace.Dy[24] -= acadoVariables.y[24];
acadoWorkspace.Dy[25] -= acadoVariables.y[25];
acadoWorkspace.Dy[26] -= acadoVariables.y[26];
acadoWorkspace.Dy[27] -= acadoVariables.y[27];
acadoWorkspace.Dy[28] -= acadoVariables.y[28];
acadoWorkspace.Dy[29] -= acadoVariables.y[29];
acadoWorkspace.DyN[0] -= acadoVariables.yN[0];
acadoWorkspace.DyN[1] -= acadoVariables.yN[1];
acadoWorkspace.DyN[2] -= acadoVariables.yN[2];

acado_multRDy( acadoWorkspace.Dy, acadoWorkspace.g );
acado_multRDy( &(acadoWorkspace.Dy[ 3 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 6 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 9 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 12 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 15 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 18 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 21 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 24 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 27 ]), &(acadoWorkspace.g[ 18 ]) );

acado_multQDy( acadoWorkspace.Q2, acadoWorkspace.Dy, acadoWorkspace.QDy );
acado_multQDy( &(acadoWorkspace.Q2[ 9 ]), &(acadoWorkspace.Dy[ 3 ]), &(acadoWorkspace.QDy[ 3 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 18 ]), &(acadoWorkspace.Dy[ 6 ]), &(acadoWorkspace.QDy[ 6 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 27 ]), &(acadoWorkspace.Dy[ 9 ]), &(acadoWorkspace.QDy[ 9 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 36 ]), &(acadoWorkspace.Dy[ 12 ]), &(acadoWorkspace.QDy[ 12 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 45 ]), &(acadoWorkspace.Dy[ 15 ]), &(acadoWorkspace.QDy[ 15 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 54 ]), &(acadoWorkspace.Dy[ 18 ]), &(acadoWorkspace.QDy[ 18 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 63 ]), &(acadoWorkspace.Dy[ 21 ]), &(acadoWorkspace.QDy[ 21 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 72 ]), &(acadoWorkspace.Dy[ 24 ]), &(acadoWorkspace.QDy[ 24 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 81 ]), &(acadoWorkspace.Dy[ 27 ]), &(acadoWorkspace.QDy[ 27 ]) );

acadoWorkspace.QDy[30] = + acadoWorkspace.QN2[0]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[1]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[2]*acadoWorkspace.DyN[2];
acadoWorkspace.QDy[31] = + acadoWorkspace.QN2[3]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[4]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[5]*acadoWorkspace.DyN[2];
acadoWorkspace.QDy[32] = + acadoWorkspace.QN2[6]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[7]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[8]*acadoWorkspace.DyN[2];

acadoWorkspace.QDy[3] += acadoWorkspace.Qd[0];
acadoWorkspace.QDy[4] += acadoWorkspace.Qd[1];
acadoWorkspace.QDy[5] += acadoWorkspace.Qd[2];
acadoWorkspace.QDy[6] += acadoWorkspace.Qd[3];
acadoWorkspace.QDy[7] += acadoWorkspace.Qd[4];
acadoWorkspace.QDy[8] += acadoWorkspace.Qd[5];
acadoWorkspace.QDy[9] += acadoWorkspace.Qd[6];
acadoWorkspace.QDy[10] += acadoWorkspace.Qd[7];
acadoWorkspace.QDy[11] += acadoWorkspace.Qd[8];
acadoWorkspace.QDy[12] += acadoWorkspace.Qd[9];
acadoWorkspace.QDy[13] += acadoWorkspace.Qd[10];
acadoWorkspace.QDy[14] += acadoWorkspace.Qd[11];
acadoWorkspace.QDy[15] += acadoWorkspace.Qd[12];
acadoWorkspace.QDy[16] += acadoWorkspace.Qd[13];
acadoWorkspace.QDy[17] += acadoWorkspace.Qd[14];
acadoWorkspace.QDy[18] += acadoWorkspace.Qd[15];
acadoWorkspace.QDy[19] += acadoWorkspace.Qd[16];
acadoWorkspace.QDy[20] += acadoWorkspace.Qd[17];
acadoWorkspace.QDy[21] += acadoWorkspace.Qd[18];
acadoWorkspace.QDy[22] += acadoWorkspace.Qd[19];
acadoWorkspace.QDy[23] += acadoWorkspace.Qd[20];
acadoWorkspace.QDy[24] += acadoWorkspace.Qd[21];
acadoWorkspace.QDy[25] += acadoWorkspace.Qd[22];
acadoWorkspace.QDy[26] += acadoWorkspace.Qd[23];
acadoWorkspace.QDy[27] += acadoWorkspace.Qd[24];
acadoWorkspace.QDy[28] += acadoWorkspace.Qd[25];
acadoWorkspace.QDy[29] += acadoWorkspace.Qd[26];
acadoWorkspace.QDy[30] += acadoWorkspace.Qd[27];
acadoWorkspace.QDy[31] += acadoWorkspace.Qd[28];
acadoWorkspace.QDy[32] += acadoWorkspace.Qd[29];

acado_multEQDy( acadoWorkspace.E, &(acadoWorkspace.QDy[ 3 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 6 ]), &(acadoWorkspace.QDy[ 6 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 18 ]), &(acadoWorkspace.QDy[ 9 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 36 ]), &(acadoWorkspace.QDy[ 12 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.QDy[ 15 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.QDy[ 18 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 126 ]), &(acadoWorkspace.QDy[ 21 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.QDy[ 24 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.QDy[ 27 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 270 ]), &(acadoWorkspace.QDy[ 30 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 12 ]), &(acadoWorkspace.QDy[ 6 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 24 ]), &(acadoWorkspace.QDy[ 9 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 42 ]), &(acadoWorkspace.QDy[ 12 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 66 ]), &(acadoWorkspace.QDy[ 15 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QDy[ 18 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 132 ]), &(acadoWorkspace.QDy[ 21 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 174 ]), &(acadoWorkspace.QDy[ 24 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 222 ]), &(acadoWorkspace.QDy[ 27 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.QDy[ 30 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 30 ]), &(acadoWorkspace.QDy[ 9 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QDy[ 12 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.QDy[ 15 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 102 ]), &(acadoWorkspace.QDy[ 18 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 138 ]), &(acadoWorkspace.QDy[ 21 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QDy[ 24 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 228 ]), &(acadoWorkspace.QDy[ 27 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 282 ]), &(acadoWorkspace.QDy[ 30 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 54 ]), &(acadoWorkspace.QDy[ 12 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 78 ]), &(acadoWorkspace.QDy[ 15 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.QDy[ 18 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QDy[ 21 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 186 ]), &(acadoWorkspace.QDy[ 24 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 234 ]), &(acadoWorkspace.QDy[ 27 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QDy[ 30 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.QDy[ 15 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 114 ]), &(acadoWorkspace.QDy[ 18 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 150 ]), &(acadoWorkspace.QDy[ 21 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QDy[ 24 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QDy[ 27 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 294 ]), &(acadoWorkspace.QDy[ 30 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QDy[ 18 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 156 ]), &(acadoWorkspace.QDy[ 21 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 198 ]), &(acadoWorkspace.QDy[ 24 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 246 ]), &(acadoWorkspace.QDy[ 27 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.QDy[ 30 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 162 ]), &(acadoWorkspace.QDy[ 21 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 204 ]), &(acadoWorkspace.QDy[ 24 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 252 ]), &(acadoWorkspace.QDy[ 27 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 306 ]), &(acadoWorkspace.QDy[ 30 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 210 ]), &(acadoWorkspace.QDy[ 24 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 258 ]), &(acadoWorkspace.QDy[ 27 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 312 ]), &(acadoWorkspace.QDy[ 30 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 264 ]), &(acadoWorkspace.QDy[ 27 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 318 ]), &(acadoWorkspace.QDy[ 30 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 324 ]), &(acadoWorkspace.QDy[ 30 ]), &(acadoWorkspace.g[ 18 ]) );

acadoWorkspace.g[0] += + acadoWorkspace.H10[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[2]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[1] += + acadoWorkspace.H10[3]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[4]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[5]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[2] += + acadoWorkspace.H10[6]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[7]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[8]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[3] += + acadoWorkspace.H10[9]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[10]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[11]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[4] += + acadoWorkspace.H10[12]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[13]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[14]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[5] += + acadoWorkspace.H10[15]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[16]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[17]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[6] += + acadoWorkspace.H10[18]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[19]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[20]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[7] += + acadoWorkspace.H10[21]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[22]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[23]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[8] += + acadoWorkspace.H10[24]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[25]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[26]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[9] += + acadoWorkspace.H10[27]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[28]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[29]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[10] += + acadoWorkspace.H10[30]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[31]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[32]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[11] += + acadoWorkspace.H10[33]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[34]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[35]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[12] += + acadoWorkspace.H10[36]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[37]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[38]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[13] += + acadoWorkspace.H10[39]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[40]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[41]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[14] += + acadoWorkspace.H10[42]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[43]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[44]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[15] += + acadoWorkspace.H10[45]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[46]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[47]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[16] += + acadoWorkspace.H10[48]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[49]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[50]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[17] += + acadoWorkspace.H10[51]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[52]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[53]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[18] += + acadoWorkspace.H10[54]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[55]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[56]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[19] += + acadoWorkspace.H10[57]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[58]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[59]*acadoWorkspace.Dx0[2];

acadoWorkspace.lb[0] = acadoVariables.lbValues[0] - acadoVariables.u[0];
acadoWorkspace.lb[1] = acadoVariables.lbValues[1] - acadoVariables.u[1];
acadoWorkspace.lb[2] = acadoVariables.lbValues[2] - acadoVariables.u[2];
acadoWorkspace.lb[3] = acadoVariables.lbValues[3] - acadoVariables.u[3];
acadoWorkspace.lb[4] = acadoVariables.lbValues[4] - acadoVariables.u[4];
acadoWorkspace.lb[5] = acadoVariables.lbValues[5] - acadoVariables.u[5];
acadoWorkspace.lb[6] = acadoVariables.lbValues[6] - acadoVariables.u[6];
acadoWorkspace.lb[7] = acadoVariables.lbValues[7] - acadoVariables.u[7];
acadoWorkspace.lb[8] = acadoVariables.lbValues[8] - acadoVariables.u[8];
acadoWorkspace.lb[9] = acadoVariables.lbValues[9] - acadoVariables.u[9];
acadoWorkspace.lb[10] = acadoVariables.lbValues[10] - acadoVariables.u[10];
acadoWorkspace.lb[11] = acadoVariables.lbValues[11] - acadoVariables.u[11];
acadoWorkspace.lb[12] = acadoVariables.lbValues[12] - acadoVariables.u[12];
acadoWorkspace.lb[13] = acadoVariables.lbValues[13] - acadoVariables.u[13];
acadoWorkspace.lb[14] = acadoVariables.lbValues[14] - acadoVariables.u[14];
acadoWorkspace.lb[15] = acadoVariables.lbValues[15] - acadoVariables.u[15];
acadoWorkspace.lb[16] = acadoVariables.lbValues[16] - acadoVariables.u[16];
acadoWorkspace.lb[17] = acadoVariables.lbValues[17] - acadoVariables.u[17];
acadoWorkspace.lb[18] = acadoVariables.lbValues[18] - acadoVariables.u[18];
acadoWorkspace.lb[19] = acadoVariables.lbValues[19] - acadoVariables.u[19];
acadoWorkspace.ub[0] = acadoVariables.ubValues[0] - acadoVariables.u[0];
acadoWorkspace.ub[1] = acadoVariables.ubValues[1] - acadoVariables.u[1];
acadoWorkspace.ub[2] = acadoVariables.ubValues[2] - acadoVariables.u[2];
acadoWorkspace.ub[3] = acadoVariables.ubValues[3] - acadoVariables.u[3];
acadoWorkspace.ub[4] = acadoVariables.ubValues[4] - acadoVariables.u[4];
acadoWorkspace.ub[5] = acadoVariables.ubValues[5] - acadoVariables.u[5];
acadoWorkspace.ub[6] = acadoVariables.ubValues[6] - acadoVariables.u[6];
acadoWorkspace.ub[7] = acadoVariables.ubValues[7] - acadoVariables.u[7];
acadoWorkspace.ub[8] = acadoVariables.ubValues[8] - acadoVariables.u[8];
acadoWorkspace.ub[9] = acadoVariables.ubValues[9] - acadoVariables.u[9];
acadoWorkspace.ub[10] = acadoVariables.ubValues[10] - acadoVariables.u[10];
acadoWorkspace.ub[11] = acadoVariables.ubValues[11] - acadoVariables.u[11];
acadoWorkspace.ub[12] = acadoVariables.ubValues[12] - acadoVariables.u[12];
acadoWorkspace.ub[13] = acadoVariables.ubValues[13] - acadoVariables.u[13];
acadoWorkspace.ub[14] = acadoVariables.ubValues[14] - acadoVariables.u[14];
acadoWorkspace.ub[15] = acadoVariables.ubValues[15] - acadoVariables.u[15];
acadoWorkspace.ub[16] = acadoVariables.ubValues[16] - acadoVariables.u[16];
acadoWorkspace.ub[17] = acadoVariables.ubValues[17] - acadoVariables.u[17];
acadoWorkspace.ub[18] = acadoVariables.ubValues[18] - acadoVariables.u[18];
acadoWorkspace.ub[19] = acadoVariables.ubValues[19] - acadoVariables.u[19];

}

void acado_expand(  )
{
acadoVariables.u[0] += acadoWorkspace.x[0];
acadoVariables.u[1] += acadoWorkspace.x[1];
acadoVariables.u[2] += acadoWorkspace.x[2];
acadoVariables.u[3] += acadoWorkspace.x[3];
acadoVariables.u[4] += acadoWorkspace.x[4];
acadoVariables.u[5] += acadoWorkspace.x[5];
acadoVariables.u[6] += acadoWorkspace.x[6];
acadoVariables.u[7] += acadoWorkspace.x[7];
acadoVariables.u[8] += acadoWorkspace.x[8];
acadoVariables.u[9] += acadoWorkspace.x[9];
acadoVariables.u[10] += acadoWorkspace.x[10];
acadoVariables.u[11] += acadoWorkspace.x[11];
acadoVariables.u[12] += acadoWorkspace.x[12];
acadoVariables.u[13] += acadoWorkspace.x[13];
acadoVariables.u[14] += acadoWorkspace.x[14];
acadoVariables.u[15] += acadoWorkspace.x[15];
acadoVariables.u[16] += acadoWorkspace.x[16];
acadoVariables.u[17] += acadoWorkspace.x[17];
acadoVariables.u[18] += acadoWorkspace.x[18];
acadoVariables.u[19] += acadoWorkspace.x[19];

acadoVariables.x[0] += acadoWorkspace.Dx0[0];
acadoVariables.x[1] += acadoWorkspace.Dx0[1];
acadoVariables.x[2] += acadoWorkspace.Dx0[2];

acadoVariables.x[3] += + acadoWorkspace.evGx[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[2]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[0];
acadoVariables.x[4] += + acadoWorkspace.evGx[3]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[4]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[5]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[1];
acadoVariables.x[5] += + acadoWorkspace.evGx[6]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[7]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[8]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[2];
acadoVariables.x[6] += + acadoWorkspace.evGx[9]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[10]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[11]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[3];
acadoVariables.x[7] += + acadoWorkspace.evGx[12]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[13]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[14]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[4];
acadoVariables.x[8] += + acadoWorkspace.evGx[15]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[16]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[17]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[5];
acadoVariables.x[9] += + acadoWorkspace.evGx[18]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[19]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[20]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[6];
acadoVariables.x[10] += + acadoWorkspace.evGx[21]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[22]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[23]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[7];
acadoVariables.x[11] += + acadoWorkspace.evGx[24]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[25]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[26]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[8];
acadoVariables.x[12] += + acadoWorkspace.evGx[27]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[28]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[29]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[9];
acadoVariables.x[13] += + acadoWorkspace.evGx[30]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[31]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[32]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[10];
acadoVariables.x[14] += + acadoWorkspace.evGx[33]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[34]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[35]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[11];
acadoVariables.x[15] += + acadoWorkspace.evGx[36]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[37]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[38]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[12];
acadoVariables.x[16] += + acadoWorkspace.evGx[39]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[40]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[41]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[13];
acadoVariables.x[17] += + acadoWorkspace.evGx[42]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[43]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[44]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[14];
acadoVariables.x[18] += + acadoWorkspace.evGx[45]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[46]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[47]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[15];
acadoVariables.x[19] += + acadoWorkspace.evGx[48]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[49]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[50]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[16];
acadoVariables.x[20] += + acadoWorkspace.evGx[51]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[52]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[53]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[17];
acadoVariables.x[21] += + acadoWorkspace.evGx[54]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[55]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[56]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[18];
acadoVariables.x[22] += + acadoWorkspace.evGx[57]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[58]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[59]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[19];
acadoVariables.x[23] += + acadoWorkspace.evGx[60]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[61]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[62]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[20];
acadoVariables.x[24] += + acadoWorkspace.evGx[63]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[64]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[65]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[21];
acadoVariables.x[25] += + acadoWorkspace.evGx[66]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[67]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[68]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[22];
acadoVariables.x[26] += + acadoWorkspace.evGx[69]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[70]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[71]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[23];
acadoVariables.x[27] += + acadoWorkspace.evGx[72]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[73]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[74]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[24];
acadoVariables.x[28] += + acadoWorkspace.evGx[75]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[76]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[77]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[25];
acadoVariables.x[29] += + acadoWorkspace.evGx[78]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[79]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[80]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[26];
acadoVariables.x[30] += + acadoWorkspace.evGx[81]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[82]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[83]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[27];
acadoVariables.x[31] += + acadoWorkspace.evGx[84]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[85]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[86]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[28];
acadoVariables.x[32] += + acadoWorkspace.evGx[87]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[88]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[89]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[29];

acado_multEDu( acadoWorkspace.E, acadoWorkspace.x, &(acadoVariables.x[ 3 ]) );
acado_multEDu( &(acadoWorkspace.E[ 6 ]), acadoWorkspace.x, &(acadoVariables.x[ 6 ]) );
acado_multEDu( &(acadoWorkspace.E[ 12 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 6 ]) );
acado_multEDu( &(acadoWorkspace.E[ 18 ]), acadoWorkspace.x, &(acadoVariables.x[ 9 ]) );
acado_multEDu( &(acadoWorkspace.E[ 24 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 9 ]) );
acado_multEDu( &(acadoWorkspace.E[ 30 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 9 ]) );
acado_multEDu( &(acadoWorkspace.E[ 36 ]), acadoWorkspace.x, &(acadoVariables.x[ 12 ]) );
acado_multEDu( &(acadoWorkspace.E[ 42 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 12 ]) );
acado_multEDu( &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 12 ]) );
acado_multEDu( &(acadoWorkspace.E[ 54 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 12 ]) );
acado_multEDu( &(acadoWorkspace.E[ 60 ]), acadoWorkspace.x, &(acadoVariables.x[ 15 ]) );
acado_multEDu( &(acadoWorkspace.E[ 66 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 15 ]) );
acado_multEDu( &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 15 ]) );
acado_multEDu( &(acadoWorkspace.E[ 78 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 15 ]) );
acado_multEDu( &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 15 ]) );
acado_multEDu( &(acadoWorkspace.E[ 90 ]), acadoWorkspace.x, &(acadoVariables.x[ 18 ]) );
acado_multEDu( &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 18 ]) );
acado_multEDu( &(acadoWorkspace.E[ 102 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 18 ]) );
acado_multEDu( &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 18 ]) );
acado_multEDu( &(acadoWorkspace.E[ 114 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 18 ]) );
acado_multEDu( &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 18 ]) );
acado_multEDu( &(acadoWorkspace.E[ 126 ]), acadoWorkspace.x, &(acadoVariables.x[ 21 ]) );
acado_multEDu( &(acadoWorkspace.E[ 132 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 21 ]) );
acado_multEDu( &(acadoWorkspace.E[ 138 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 21 ]) );
acado_multEDu( &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 21 ]) );
acado_multEDu( &(acadoWorkspace.E[ 150 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 21 ]) );
acado_multEDu( &(acadoWorkspace.E[ 156 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 21 ]) );
acado_multEDu( &(acadoWorkspace.E[ 162 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 21 ]) );
acado_multEDu( &(acadoWorkspace.E[ 168 ]), acadoWorkspace.x, &(acadoVariables.x[ 24 ]) );
acado_multEDu( &(acadoWorkspace.E[ 174 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 24 ]) );
acado_multEDu( &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 24 ]) );
acado_multEDu( &(acadoWorkspace.E[ 186 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 24 ]) );
acado_multEDu( &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 24 ]) );
acado_multEDu( &(acadoWorkspace.E[ 198 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 24 ]) );
acado_multEDu( &(acadoWorkspace.E[ 204 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 24 ]) );
acado_multEDu( &(acadoWorkspace.E[ 210 ]), &(acadoWorkspace.x[ 14 ]), &(acadoVariables.x[ 24 ]) );
acado_multEDu( &(acadoWorkspace.E[ 216 ]), acadoWorkspace.x, &(acadoVariables.x[ 27 ]) );
acado_multEDu( &(acadoWorkspace.E[ 222 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 27 ]) );
acado_multEDu( &(acadoWorkspace.E[ 228 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 27 ]) );
acado_multEDu( &(acadoWorkspace.E[ 234 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 27 ]) );
acado_multEDu( &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 27 ]) );
acado_multEDu( &(acadoWorkspace.E[ 246 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 27 ]) );
acado_multEDu( &(acadoWorkspace.E[ 252 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 27 ]) );
acado_multEDu( &(acadoWorkspace.E[ 258 ]), &(acadoWorkspace.x[ 14 ]), &(acadoVariables.x[ 27 ]) );
acado_multEDu( &(acadoWorkspace.E[ 264 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 27 ]) );
acado_multEDu( &(acadoWorkspace.E[ 270 ]), acadoWorkspace.x, &(acadoVariables.x[ 30 ]) );
acado_multEDu( &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 30 ]) );
acado_multEDu( &(acadoWorkspace.E[ 282 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 30 ]) );
acado_multEDu( &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 30 ]) );
acado_multEDu( &(acadoWorkspace.E[ 294 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 30 ]) );
acado_multEDu( &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 30 ]) );
acado_multEDu( &(acadoWorkspace.E[ 306 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 30 ]) );
acado_multEDu( &(acadoWorkspace.E[ 312 ]), &(acadoWorkspace.x[ 14 ]), &(acadoVariables.x[ 30 ]) );
acado_multEDu( &(acadoWorkspace.E[ 318 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 30 ]) );
acado_multEDu( &(acadoWorkspace.E[ 324 ]), &(acadoWorkspace.x[ 18 ]), &(acadoVariables.x[ 30 ]) );
}

int acado_preparationStep(  )
{
int ret;

ret = acado_modelSimulation();
acado_evaluateObjective(  );
acado_condensePrep(  );
return ret;
}

int acado_feedbackStep(  )
{
int tmp;

acado_condenseFdb(  );

tmp = acado_solve( );

acado_expand(  );
return tmp;
}

int acado_initializeSolver(  )
{
int ret;

/* This is a function which must be called once before any other function call! */


ret = 0;

memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
acadoVariables.lbValues[0] = -1.0000000000000000e+00;
acadoVariables.lbValues[1] = -7.8539816339744828e-01;
acadoVariables.lbValues[2] = -1.0000000000000000e+00;
acadoVariables.lbValues[3] = -7.8539816339744828e-01;
acadoVariables.lbValues[4] = -1.0000000000000000e+00;
acadoVariables.lbValues[5] = -7.8539816339744828e-01;
acadoVariables.lbValues[6] = -1.0000000000000000e+00;
acadoVariables.lbValues[7] = -7.8539816339744828e-01;
acadoVariables.lbValues[8] = -1.0000000000000000e+00;
acadoVariables.lbValues[9] = -7.8539816339744828e-01;
acadoVariables.lbValues[10] = -1.0000000000000000e+00;
acadoVariables.lbValues[11] = -7.8539816339744828e-01;
acadoVariables.lbValues[12] = -1.0000000000000000e+00;
acadoVariables.lbValues[13] = -7.8539816339744828e-01;
acadoVariables.lbValues[14] = -1.0000000000000000e+00;
acadoVariables.lbValues[15] = -7.8539816339744828e-01;
acadoVariables.lbValues[16] = -1.0000000000000000e+00;
acadoVariables.lbValues[17] = -7.8539816339744828e-01;
acadoVariables.lbValues[18] = -1.0000000000000000e+00;
acadoVariables.lbValues[19] = -7.8539816339744828e-01;
acadoVariables.ubValues[0] = 1.0000000000000000e+00;
acadoVariables.ubValues[1] = 7.8539816339744828e-01;
acadoVariables.ubValues[2] = 1.0000000000000000e+00;
acadoVariables.ubValues[3] = 7.8539816339744828e-01;
acadoVariables.ubValues[4] = 1.0000000000000000e+00;
acadoVariables.ubValues[5] = 7.8539816339744828e-01;
acadoVariables.ubValues[6] = 1.0000000000000000e+00;
acadoVariables.ubValues[7] = 7.8539816339744828e-01;
acadoVariables.ubValues[8] = 1.0000000000000000e+00;
acadoVariables.ubValues[9] = 7.8539816339744828e-01;
acadoVariables.ubValues[10] = 1.0000000000000000e+00;
acadoVariables.ubValues[11] = 7.8539816339744828e-01;
acadoVariables.ubValues[12] = 1.0000000000000000e+00;
acadoVariables.ubValues[13] = 7.8539816339744828e-01;
acadoVariables.ubValues[14] = 1.0000000000000000e+00;
acadoVariables.ubValues[15] = 7.8539816339744828e-01;
acadoVariables.ubValues[16] = 1.0000000000000000e+00;
acadoVariables.ubValues[17] = 7.8539816339744828e-01;
acadoVariables.ubValues[18] = 1.0000000000000000e+00;
acadoVariables.ubValues[19] = 7.8539816339744828e-01;
return ret;
}

void acado_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 10; ++index)
{
acadoWorkspace.state[0] = acadoVariables.x[index * 3];
acadoWorkspace.state[1] = acadoVariables.x[index * 3 + 1];
acadoWorkspace.state[2] = acadoVariables.x[index * 3 + 2];
acadoWorkspace.state[18] = acadoVariables.u[index * 2];
acadoWorkspace.state[19] = acadoVariables.u[index * 2 + 1];

acado_integrate(acadoWorkspace.state, index == 0);

acadoVariables.x[index * 3 + 3] = acadoWorkspace.state[0];
acadoVariables.x[index * 3 + 4] = acadoWorkspace.state[1];
acadoVariables.x[index * 3 + 5] = acadoWorkspace.state[2];
}
}

void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 10; ++index)
{
acadoVariables.x[index * 3] = acadoVariables.x[index * 3 + 3];
acadoVariables.x[index * 3 + 1] = acadoVariables.x[index * 3 + 4];
acadoVariables.x[index * 3 + 2] = acadoVariables.x[index * 3 + 5];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[30] = xEnd[0];
acadoVariables.x[31] = xEnd[1];
acadoVariables.x[32] = xEnd[2];
}
else if (strategy == 2) 
{
acadoWorkspace.state[0] = acadoVariables.x[30];
acadoWorkspace.state[1] = acadoVariables.x[31];
acadoWorkspace.state[2] = acadoVariables.x[32];
if (uEnd != 0)
{
acadoWorkspace.state[18] = uEnd[0];
acadoWorkspace.state[19] = uEnd[1];
}
else
{
acadoWorkspace.state[18] = acadoVariables.u[18];
acadoWorkspace.state[19] = acadoVariables.u[19];
}

acado_integrate(acadoWorkspace.state, 1);

acadoVariables.x[30] = acadoWorkspace.state[0];
acadoVariables.x[31] = acadoWorkspace.state[1];
acadoVariables.x[32] = acadoWorkspace.state[2];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 9; ++index)
{
acadoVariables.u[index * 2] = acadoVariables.u[index * 2 + 2];
acadoVariables.u[index * 2 + 1] = acadoVariables.u[index * 2 + 3];
}

if (uEnd != 0)
{
acadoVariables.u[18] = uEnd[0];
acadoVariables.u[19] = uEnd[1];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19];
kkt = fabs( kkt );
for (index = 0; index < 20; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
return kkt;
}

real_t acado_getObjective(  )
{
real_t objVal;

int lRun1;
/** Row vector of size: 3 */
real_t tmpDy[ 3 ];

/** Row vector of size: 3 */
real_t tmpDyN[ 3 ];

for (lRun1 = 0; lRun1 < 10; ++lRun1)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1 * 3];
acadoWorkspace.objValueIn[1] = acadoVariables.x[lRun1 * 3 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[lRun1 * 3 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.u[lRun1 * 2];
acadoWorkspace.objValueIn[4] = acadoVariables.u[lRun1 * 2 + 1];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[lRun1 * 3] = acadoWorkspace.objValueOut[0] - acadoVariables.y[lRun1 * 3];
acadoWorkspace.Dy[lRun1 * 3 + 1] = acadoWorkspace.objValueOut[1] - acadoVariables.y[lRun1 * 3 + 1];
acadoWorkspace.Dy[lRun1 * 3 + 2] = acadoWorkspace.objValueOut[2] - acadoVariables.y[lRun1 * 3 + 2];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[30];
acadoWorkspace.objValueIn[1] = acadoVariables.x[31];
acadoWorkspace.objValueIn[2] = acadoVariables.x[32];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1] - acadoVariables.yN[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2] - acadoVariables.yN[2];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 10; ++lRun1)
{
tmpDy[0] = + acadoWorkspace.Dy[lRun1 * 3]*acadoVariables.W[lRun1 * 9] + acadoWorkspace.Dy[lRun1 * 3 + 1]*acadoVariables.W[lRun1 * 9 + 3] + acadoWorkspace.Dy[lRun1 * 3 + 2]*acadoVariables.W[lRun1 * 9 + 6];
tmpDy[1] = + acadoWorkspace.Dy[lRun1 * 3]*acadoVariables.W[lRun1 * 9 + 1] + acadoWorkspace.Dy[lRun1 * 3 + 1]*acadoVariables.W[lRun1 * 9 + 4] + acadoWorkspace.Dy[lRun1 * 3 + 2]*acadoVariables.W[lRun1 * 9 + 7];
tmpDy[2] = + acadoWorkspace.Dy[lRun1 * 3]*acadoVariables.W[lRun1 * 9 + 2] + acadoWorkspace.Dy[lRun1 * 3 + 1]*acadoVariables.W[lRun1 * 9 + 5] + acadoWorkspace.Dy[lRun1 * 3 + 2]*acadoVariables.W[lRun1 * 9 + 8];
objVal += + acadoWorkspace.Dy[lRun1 * 3]*tmpDy[0] + acadoWorkspace.Dy[lRun1 * 3 + 1]*tmpDy[1] + acadoWorkspace.Dy[lRun1 * 3 + 2]*tmpDy[2];
}

tmpDyN[0] = + acadoWorkspace.DyN[0]*acadoVariables.WN[0];
tmpDyN[1] = + acadoWorkspace.DyN[1]*acadoVariables.WN[4];
tmpDyN[2] = + acadoWorkspace.DyN[2]*acadoVariables.WN[8];
objVal += + acadoWorkspace.DyN[0]*tmpDyN[0] + acadoWorkspace.DyN[1]*tmpDyN[1] + acadoWorkspace.DyN[2]*tmpDyN[2];

objVal *= 0.5;
return objVal;
}

