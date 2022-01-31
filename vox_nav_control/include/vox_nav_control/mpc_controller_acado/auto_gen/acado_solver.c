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


/** Row vector of size: 62 */
real_t state[ 62 ];

int acado_modelSimulation(  )
{
int ret;

int lRun1;
ret = 0;
#pragma omp parallel for private(lRun1, state) shared(acadoWorkspace, acadoVariables)
for (lRun1 = 0; lRun1 < 20; ++lRun1)
{
state[0] = acadoVariables.x[lRun1 * 4];
state[1] = acadoVariables.x[lRun1 * 4 + 1];
state[2] = acadoVariables.x[lRun1 * 4 + 2];
state[3] = acadoVariables.x[lRun1 * 4 + 3];

state[28] = acadoVariables.u[lRun1 * 2];
state[29] = acadoVariables.u[lRun1 * 2 + 1];
state[30] = acadoVariables.od[lRun1 * 32];
state[31] = acadoVariables.od[lRun1 * 32 + 1];
state[32] = acadoVariables.od[lRun1 * 32 + 2];
state[33] = acadoVariables.od[lRun1 * 32 + 3];
state[34] = acadoVariables.od[lRun1 * 32 + 4];
state[35] = acadoVariables.od[lRun1 * 32 + 5];
state[36] = acadoVariables.od[lRun1 * 32 + 6];
state[37] = acadoVariables.od[lRun1 * 32 + 7];
state[38] = acadoVariables.od[lRun1 * 32 + 8];
state[39] = acadoVariables.od[lRun1 * 32 + 9];
state[40] = acadoVariables.od[lRun1 * 32 + 10];
state[41] = acadoVariables.od[lRun1 * 32 + 11];
state[42] = acadoVariables.od[lRun1 * 32 + 12];
state[43] = acadoVariables.od[lRun1 * 32 + 13];
state[44] = acadoVariables.od[lRun1 * 32 + 14];
state[45] = acadoVariables.od[lRun1 * 32 + 15];
state[46] = acadoVariables.od[lRun1 * 32 + 16];
state[47] = acadoVariables.od[lRun1 * 32 + 17];
state[48] = acadoVariables.od[lRun1 * 32 + 18];
state[49] = acadoVariables.od[lRun1 * 32 + 19];
state[50] = acadoVariables.od[lRun1 * 32 + 20];
state[51] = acadoVariables.od[lRun1 * 32 + 21];
state[52] = acadoVariables.od[lRun1 * 32 + 22];
state[53] = acadoVariables.od[lRun1 * 32 + 23];
state[54] = acadoVariables.od[lRun1 * 32 + 24];
state[55] = acadoVariables.od[lRun1 * 32 + 25];
state[56] = acadoVariables.od[lRun1 * 32 + 26];
state[57] = acadoVariables.od[lRun1 * 32 + 27];
state[58] = acadoVariables.od[lRun1 * 32 + 28];
state[59] = acadoVariables.od[lRun1 * 32 + 29];
state[60] = acadoVariables.od[lRun1 * 32 + 30];
state[61] = acadoVariables.od[lRun1 * 32 + 31];

ret = acado_integrate(state, 1);

acadoWorkspace.d[lRun1 * 4] = state[0] - acadoVariables.x[lRun1 * 4 + 4];
acadoWorkspace.d[lRun1 * 4 + 1] = state[1] - acadoVariables.x[lRun1 * 4 + 5];
acadoWorkspace.d[lRun1 * 4 + 2] = state[2] - acadoVariables.x[lRun1 * 4 + 6];
acadoWorkspace.d[lRun1 * 4 + 3] = state[3] - acadoVariables.x[lRun1 * 4 + 7];

acadoWorkspace.evGx[lRun1 * 16] = state[4];
acadoWorkspace.evGx[lRun1 * 16 + 1] = state[5];
acadoWorkspace.evGx[lRun1 * 16 + 2] = state[6];
acadoWorkspace.evGx[lRun1 * 16 + 3] = state[7];
acadoWorkspace.evGx[lRun1 * 16 + 4] = state[8];
acadoWorkspace.evGx[lRun1 * 16 + 5] = state[9];
acadoWorkspace.evGx[lRun1 * 16 + 6] = state[10];
acadoWorkspace.evGx[lRun1 * 16 + 7] = state[11];
acadoWorkspace.evGx[lRun1 * 16 + 8] = state[12];
acadoWorkspace.evGx[lRun1 * 16 + 9] = state[13];
acadoWorkspace.evGx[lRun1 * 16 + 10] = state[14];
acadoWorkspace.evGx[lRun1 * 16 + 11] = state[15];
acadoWorkspace.evGx[lRun1 * 16 + 12] = state[16];
acadoWorkspace.evGx[lRun1 * 16 + 13] = state[17];
acadoWorkspace.evGx[lRun1 * 16 + 14] = state[18];
acadoWorkspace.evGx[lRun1 * 16 + 15] = state[19];

acadoWorkspace.evGu[lRun1 * 8] = state[20];
acadoWorkspace.evGu[lRun1 * 8 + 1] = state[21];
acadoWorkspace.evGu[lRun1 * 8 + 2] = state[22];
acadoWorkspace.evGu[lRun1 * 8 + 3] = state[23];
acadoWorkspace.evGu[lRun1 * 8 + 4] = state[24];
acadoWorkspace.evGu[lRun1 * 8 + 5] = state[25];
acadoWorkspace.evGu[lRun1 * 8 + 6] = state[26];
acadoWorkspace.evGu[lRun1 * 8 + 7] = state[27];
}
return ret;
}

void acado_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 4;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
out[4] = u[0];
out[5] = u[1];
out[6] = xd[4];
}

void acado_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
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
tmpQ2[9] = +tmpObjS[9];
tmpQ2[10] = +tmpObjS[10];
tmpQ2[11] = +tmpObjS[11];
tmpQ2[12] = +tmpObjS[12];
tmpQ2[13] = +tmpObjS[13];
tmpQ2[14] = +tmpObjS[14];
tmpQ2[15] = +tmpObjS[15];
tmpQ2[16] = +tmpObjS[16];
tmpQ2[17] = +tmpObjS[17];
tmpQ2[18] = +tmpObjS[18];
tmpQ2[19] = +tmpObjS[19];
tmpQ2[20] = +tmpObjS[20];
tmpQ2[21] = +tmpObjS[21];
tmpQ2[22] = +tmpObjS[22];
tmpQ2[23] = +tmpObjS[23];
tmpQ2[24] = +tmpObjS[24];
tmpQ2[25] = +tmpObjS[25];
tmpQ2[26] = +tmpObjS[26];
tmpQ2[27] = +tmpObjS[27];
tmpQ1[0] = + tmpQ2[0];
tmpQ1[1] = + tmpQ2[1];
tmpQ1[2] = + tmpQ2[2];
tmpQ1[3] = + tmpQ2[3];
tmpQ1[4] = + tmpQ2[7];
tmpQ1[5] = + tmpQ2[8];
tmpQ1[6] = + tmpQ2[9];
tmpQ1[7] = + tmpQ2[10];
tmpQ1[8] = + tmpQ2[14];
tmpQ1[9] = + tmpQ2[15];
tmpQ1[10] = + tmpQ2[16];
tmpQ1[11] = + tmpQ2[17];
tmpQ1[12] = + tmpQ2[21];
tmpQ1[13] = + tmpQ2[22];
tmpQ1[14] = + tmpQ2[23];
tmpQ1[15] = + tmpQ2[24];
}

void acado_setObjR1R2( real_t* const tmpObjS, real_t* const tmpR1, real_t* const tmpR2 )
{
tmpR2[0] = +tmpObjS[28];
tmpR2[1] = +tmpObjS[29];
tmpR2[2] = +tmpObjS[30];
tmpR2[3] = +tmpObjS[31];
tmpR2[4] = +tmpObjS[32];
tmpR2[5] = +tmpObjS[33];
tmpR2[6] = +tmpObjS[34];
tmpR2[7] = +tmpObjS[35];
tmpR2[8] = +tmpObjS[36];
tmpR2[9] = +tmpObjS[37];
tmpR2[10] = +tmpObjS[38];
tmpR2[11] = +tmpObjS[39];
tmpR2[12] = +tmpObjS[40];
tmpR2[13] = +tmpObjS[41];
tmpR1[0] = + tmpR2[4];
tmpR1[1] = + tmpR2[5];
tmpR1[2] = + tmpR2[11];
tmpR1[3] = + tmpR2[12];
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
tmpQN2[9] = +tmpObjSEndTerm[9];
tmpQN2[10] = +tmpObjSEndTerm[10];
tmpQN2[11] = +tmpObjSEndTerm[11];
tmpQN2[12] = +tmpObjSEndTerm[12];
tmpQN2[13] = +tmpObjSEndTerm[13];
tmpQN2[14] = +tmpObjSEndTerm[14];
tmpQN2[15] = +tmpObjSEndTerm[15];
tmpQN1[0] = + tmpQN2[0];
tmpQN1[1] = + tmpQN2[1];
tmpQN1[2] = + tmpQN2[2];
tmpQN1[3] = + tmpQN2[3];
tmpQN1[4] = + tmpQN2[4];
tmpQN1[5] = + tmpQN2[5];
tmpQN1[6] = + tmpQN2[6];
tmpQN1[7] = + tmpQN2[7];
tmpQN1[8] = + tmpQN2[8];
tmpQN1[9] = + tmpQN2[9];
tmpQN1[10] = + tmpQN2[10];
tmpQN1[11] = + tmpQN2[11];
tmpQN1[12] = + tmpQN2[12];
tmpQN1[13] = + tmpQN2[13];
tmpQN1[14] = + tmpQN2[14];
tmpQN1[15] = + tmpQN2[15];
}

void acado_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 20; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj * 4];
acadoWorkspace.objValueIn[1] = acadoVariables.x[runObj * 4 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[runObj * 4 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[runObj * 4 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.u[runObj * 2];
acadoWorkspace.objValueIn[5] = acadoVariables.u[runObj * 2 + 1];
acadoWorkspace.objValueIn[6] = acadoVariables.od[runObj * 32];
acadoWorkspace.objValueIn[7] = acadoVariables.od[runObj * 32 + 1];
acadoWorkspace.objValueIn[8] = acadoVariables.od[runObj * 32 + 2];
acadoWorkspace.objValueIn[9] = acadoVariables.od[runObj * 32 + 3];
acadoWorkspace.objValueIn[10] = acadoVariables.od[runObj * 32 + 4];
acadoWorkspace.objValueIn[11] = acadoVariables.od[runObj * 32 + 5];
acadoWorkspace.objValueIn[12] = acadoVariables.od[runObj * 32 + 6];
acadoWorkspace.objValueIn[13] = acadoVariables.od[runObj * 32 + 7];
acadoWorkspace.objValueIn[14] = acadoVariables.od[runObj * 32 + 8];
acadoWorkspace.objValueIn[15] = acadoVariables.od[runObj * 32 + 9];
acadoWorkspace.objValueIn[16] = acadoVariables.od[runObj * 32 + 10];
acadoWorkspace.objValueIn[17] = acadoVariables.od[runObj * 32 + 11];
acadoWorkspace.objValueIn[18] = acadoVariables.od[runObj * 32 + 12];
acadoWorkspace.objValueIn[19] = acadoVariables.od[runObj * 32 + 13];
acadoWorkspace.objValueIn[20] = acadoVariables.od[runObj * 32 + 14];
acadoWorkspace.objValueIn[21] = acadoVariables.od[runObj * 32 + 15];
acadoWorkspace.objValueIn[22] = acadoVariables.od[runObj * 32 + 16];
acadoWorkspace.objValueIn[23] = acadoVariables.od[runObj * 32 + 17];
acadoWorkspace.objValueIn[24] = acadoVariables.od[runObj * 32 + 18];
acadoWorkspace.objValueIn[25] = acadoVariables.od[runObj * 32 + 19];
acadoWorkspace.objValueIn[26] = acadoVariables.od[runObj * 32 + 20];
acadoWorkspace.objValueIn[27] = acadoVariables.od[runObj * 32 + 21];
acadoWorkspace.objValueIn[28] = acadoVariables.od[runObj * 32 + 22];
acadoWorkspace.objValueIn[29] = acadoVariables.od[runObj * 32 + 23];
acadoWorkspace.objValueIn[30] = acadoVariables.od[runObj * 32 + 24];
acadoWorkspace.objValueIn[31] = acadoVariables.od[runObj * 32 + 25];
acadoWorkspace.objValueIn[32] = acadoVariables.od[runObj * 32 + 26];
acadoWorkspace.objValueIn[33] = acadoVariables.od[runObj * 32 + 27];
acadoWorkspace.objValueIn[34] = acadoVariables.od[runObj * 32 + 28];
acadoWorkspace.objValueIn[35] = acadoVariables.od[runObj * 32 + 29];
acadoWorkspace.objValueIn[36] = acadoVariables.od[runObj * 32 + 30];
acadoWorkspace.objValueIn[37] = acadoVariables.od[runObj * 32 + 31];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[runObj * 7] = acadoWorkspace.objValueOut[0];
acadoWorkspace.Dy[runObj * 7 + 1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.Dy[runObj * 7 + 2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.Dy[runObj * 7 + 3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.Dy[runObj * 7 + 4] = acadoWorkspace.objValueOut[4];
acadoWorkspace.Dy[runObj * 7 + 5] = acadoWorkspace.objValueOut[5];
acadoWorkspace.Dy[runObj * 7 + 6] = acadoWorkspace.objValueOut[6];

acado_setObjQ1Q2( &(acadoVariables.W[ runObj * 49 ]), &(acadoWorkspace.Q1[ runObj * 16 ]), &(acadoWorkspace.Q2[ runObj * 28 ]) );

acado_setObjR1R2( &(acadoVariables.W[ runObj * 49 ]), &(acadoWorkspace.R1[ runObj * 4 ]), &(acadoWorkspace.R2[ runObj * 14 ]) );

}
acadoWorkspace.objValueIn[0] = acadoVariables.x[80];
acadoWorkspace.objValueIn[1] = acadoVariables.x[81];
acadoWorkspace.objValueIn[2] = acadoVariables.x[82];
acadoWorkspace.objValueIn[3] = acadoVariables.x[83];
acadoWorkspace.objValueIn[4] = acadoVariables.od[640];
acadoWorkspace.objValueIn[5] = acadoVariables.od[641];
acadoWorkspace.objValueIn[6] = acadoVariables.od[642];
acadoWorkspace.objValueIn[7] = acadoVariables.od[643];
acadoWorkspace.objValueIn[8] = acadoVariables.od[644];
acadoWorkspace.objValueIn[9] = acadoVariables.od[645];
acadoWorkspace.objValueIn[10] = acadoVariables.od[646];
acadoWorkspace.objValueIn[11] = acadoVariables.od[647];
acadoWorkspace.objValueIn[12] = acadoVariables.od[648];
acadoWorkspace.objValueIn[13] = acadoVariables.od[649];
acadoWorkspace.objValueIn[14] = acadoVariables.od[650];
acadoWorkspace.objValueIn[15] = acadoVariables.od[651];
acadoWorkspace.objValueIn[16] = acadoVariables.od[652];
acadoWorkspace.objValueIn[17] = acadoVariables.od[653];
acadoWorkspace.objValueIn[18] = acadoVariables.od[654];
acadoWorkspace.objValueIn[19] = acadoVariables.od[655];
acadoWorkspace.objValueIn[20] = acadoVariables.od[656];
acadoWorkspace.objValueIn[21] = acadoVariables.od[657];
acadoWorkspace.objValueIn[22] = acadoVariables.od[658];
acadoWorkspace.objValueIn[23] = acadoVariables.od[659];
acadoWorkspace.objValueIn[24] = acadoVariables.od[660];
acadoWorkspace.objValueIn[25] = acadoVariables.od[661];
acadoWorkspace.objValueIn[26] = acadoVariables.od[662];
acadoWorkspace.objValueIn[27] = acadoVariables.od[663];
acadoWorkspace.objValueIn[28] = acadoVariables.od[664];
acadoWorkspace.objValueIn[29] = acadoVariables.od[665];
acadoWorkspace.objValueIn[30] = acadoVariables.od[666];
acadoWorkspace.objValueIn[31] = acadoVariables.od[667];
acadoWorkspace.objValueIn[32] = acadoVariables.od[668];
acadoWorkspace.objValueIn[33] = acadoVariables.od[669];
acadoWorkspace.objValueIn[34] = acadoVariables.od[670];
acadoWorkspace.objValueIn[35] = acadoVariables.od[671];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3];

acado_setObjQN1QN2( acadoVariables.WN, acadoWorkspace.QN1, acadoWorkspace.QN2 );

}

void acado_multGxd( real_t* const dOld, real_t* const Gx1, real_t* const dNew )
{
dNew[0] += + Gx1[0]*dOld[0] + Gx1[1]*dOld[1] + Gx1[2]*dOld[2] + Gx1[3]*dOld[3];
dNew[1] += + Gx1[4]*dOld[0] + Gx1[5]*dOld[1] + Gx1[6]*dOld[2] + Gx1[7]*dOld[3];
dNew[2] += + Gx1[8]*dOld[0] + Gx1[9]*dOld[1] + Gx1[10]*dOld[2] + Gx1[11]*dOld[3];
dNew[3] += + Gx1[12]*dOld[0] + Gx1[13]*dOld[1] + Gx1[14]*dOld[2] + Gx1[15]*dOld[3];
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
Gx2[9] = Gx1[9];
Gx2[10] = Gx1[10];
Gx2[11] = Gx1[11];
Gx2[12] = Gx1[12];
Gx2[13] = Gx1[13];
Gx2[14] = Gx1[14];
Gx2[15] = Gx1[15];
}

void acado_multGxGx( real_t* const Gx1, real_t* const Gx2, real_t* const Gx3 )
{
Gx3[0] = + Gx1[0]*Gx2[0] + Gx1[1]*Gx2[4] + Gx1[2]*Gx2[8] + Gx1[3]*Gx2[12];
Gx3[1] = + Gx1[0]*Gx2[1] + Gx1[1]*Gx2[5] + Gx1[2]*Gx2[9] + Gx1[3]*Gx2[13];
Gx3[2] = + Gx1[0]*Gx2[2] + Gx1[1]*Gx2[6] + Gx1[2]*Gx2[10] + Gx1[3]*Gx2[14];
Gx3[3] = + Gx1[0]*Gx2[3] + Gx1[1]*Gx2[7] + Gx1[2]*Gx2[11] + Gx1[3]*Gx2[15];
Gx3[4] = + Gx1[4]*Gx2[0] + Gx1[5]*Gx2[4] + Gx1[6]*Gx2[8] + Gx1[7]*Gx2[12];
Gx3[5] = + Gx1[4]*Gx2[1] + Gx1[5]*Gx2[5] + Gx1[6]*Gx2[9] + Gx1[7]*Gx2[13];
Gx3[6] = + Gx1[4]*Gx2[2] + Gx1[5]*Gx2[6] + Gx1[6]*Gx2[10] + Gx1[7]*Gx2[14];
Gx3[7] = + Gx1[4]*Gx2[3] + Gx1[5]*Gx2[7] + Gx1[6]*Gx2[11] + Gx1[7]*Gx2[15];
Gx3[8] = + Gx1[8]*Gx2[0] + Gx1[9]*Gx2[4] + Gx1[10]*Gx2[8] + Gx1[11]*Gx2[12];
Gx3[9] = + Gx1[8]*Gx2[1] + Gx1[9]*Gx2[5] + Gx1[10]*Gx2[9] + Gx1[11]*Gx2[13];
Gx3[10] = + Gx1[8]*Gx2[2] + Gx1[9]*Gx2[6] + Gx1[10]*Gx2[10] + Gx1[11]*Gx2[14];
Gx3[11] = + Gx1[8]*Gx2[3] + Gx1[9]*Gx2[7] + Gx1[10]*Gx2[11] + Gx1[11]*Gx2[15];
Gx3[12] = + Gx1[12]*Gx2[0] + Gx1[13]*Gx2[4] + Gx1[14]*Gx2[8] + Gx1[15]*Gx2[12];
Gx3[13] = + Gx1[12]*Gx2[1] + Gx1[13]*Gx2[5] + Gx1[14]*Gx2[9] + Gx1[15]*Gx2[13];
Gx3[14] = + Gx1[12]*Gx2[2] + Gx1[13]*Gx2[6] + Gx1[14]*Gx2[10] + Gx1[15]*Gx2[14];
Gx3[15] = + Gx1[12]*Gx2[3] + Gx1[13]*Gx2[7] + Gx1[14]*Gx2[11] + Gx1[15]*Gx2[15];
}

void acado_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[2] + Gx1[2]*Gu1[4] + Gx1[3]*Gu1[6];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[1]*Gu1[3] + Gx1[2]*Gu1[5] + Gx1[3]*Gu1[7];
Gu2[2] = + Gx1[4]*Gu1[0] + Gx1[5]*Gu1[2] + Gx1[6]*Gu1[4] + Gx1[7]*Gu1[6];
Gu2[3] = + Gx1[4]*Gu1[1] + Gx1[5]*Gu1[3] + Gx1[6]*Gu1[5] + Gx1[7]*Gu1[7];
Gu2[4] = + Gx1[8]*Gu1[0] + Gx1[9]*Gu1[2] + Gx1[10]*Gu1[4] + Gx1[11]*Gu1[6];
Gu2[5] = + Gx1[8]*Gu1[1] + Gx1[9]*Gu1[3] + Gx1[10]*Gu1[5] + Gx1[11]*Gu1[7];
Gu2[6] = + Gx1[12]*Gu1[0] + Gx1[13]*Gu1[2] + Gx1[14]*Gu1[4] + Gx1[15]*Gu1[6];
Gu2[7] = + Gx1[12]*Gu1[1] + Gx1[13]*Gu1[3] + Gx1[14]*Gu1[5] + Gx1[15]*Gu1[7];
}

void acado_moveGuE( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = Gu1[0];
Gu2[1] = Gu1[1];
Gu2[2] = Gu1[2];
Gu2[3] = Gu1[3];
Gu2[4] = Gu1[4];
Gu2[5] = Gu1[5];
Gu2[6] = Gu1[6];
Gu2[7] = Gu1[7];
}

void acado_setBlockH11( int iRow, int iCol, real_t* const Gu1, real_t* const Gu2 )
{
acadoWorkspace.H[(iRow * 80) + (iCol * 2)] += + Gu1[0]*Gu2[0] + Gu1[2]*Gu2[2] + Gu1[4]*Gu2[4] + Gu1[6]*Gu2[6];
acadoWorkspace.H[(iRow * 80) + (iCol * 2 + 1)] += + Gu1[0]*Gu2[1] + Gu1[2]*Gu2[3] + Gu1[4]*Gu2[5] + Gu1[6]*Gu2[7];
acadoWorkspace.H[(iRow * 80 + 40) + (iCol * 2)] += + Gu1[1]*Gu2[0] + Gu1[3]*Gu2[2] + Gu1[5]*Gu2[4] + Gu1[7]*Gu2[6];
acadoWorkspace.H[(iRow * 80 + 40) + (iCol * 2 + 1)] += + Gu1[1]*Gu2[1] + Gu1[3]*Gu2[3] + Gu1[5]*Gu2[5] + Gu1[7]*Gu2[7];
}

void acado_setBlockH11_R1( int iRow, int iCol, real_t* const R11 )
{
acadoWorkspace.H[(iRow * 80) + (iCol * 2)] = R11[0];
acadoWorkspace.H[(iRow * 80) + (iCol * 2 + 1)] = R11[1];
acadoWorkspace.H[(iRow * 80 + 40) + (iCol * 2)] = R11[2];
acadoWorkspace.H[(iRow * 80 + 40) + (iCol * 2 + 1)] = R11[3];
}

void acado_zeroBlockH11( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 80) + (iCol * 2)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 80) + (iCol * 2 + 1)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 80 + 40) + (iCol * 2)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 80 + 40) + (iCol * 2 + 1)] = 0.0000000000000000e+00;
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 80) + (iCol * 2)] = acadoWorkspace.H[(iCol * 80) + (iRow * 2)];
acadoWorkspace.H[(iRow * 80) + (iCol * 2 + 1)] = acadoWorkspace.H[(iCol * 80 + 40) + (iRow * 2)];
acadoWorkspace.H[(iRow * 80 + 40) + (iCol * 2)] = acadoWorkspace.H[(iCol * 80) + (iRow * 2 + 1)];
acadoWorkspace.H[(iRow * 80 + 40) + (iCol * 2 + 1)] = acadoWorkspace.H[(iCol * 80 + 40) + (iRow * 2 + 1)];
}

void acado_multQ1d( real_t* const Gx1, real_t* const dOld, real_t* const dNew )
{
dNew[0] = + Gx1[0]*dOld[0] + Gx1[1]*dOld[1] + Gx1[2]*dOld[2] + Gx1[3]*dOld[3];
dNew[1] = + Gx1[4]*dOld[0] + Gx1[5]*dOld[1] + Gx1[6]*dOld[2] + Gx1[7]*dOld[3];
dNew[2] = + Gx1[8]*dOld[0] + Gx1[9]*dOld[1] + Gx1[10]*dOld[2] + Gx1[11]*dOld[3];
dNew[3] = + Gx1[12]*dOld[0] + Gx1[13]*dOld[1] + Gx1[14]*dOld[2] + Gx1[15]*dOld[3];
}

void acado_multQN1d( real_t* const QN1, real_t* const dOld, real_t* const dNew )
{
dNew[0] = + acadoWorkspace.QN1[0]*dOld[0] + acadoWorkspace.QN1[1]*dOld[1] + acadoWorkspace.QN1[2]*dOld[2] + acadoWorkspace.QN1[3]*dOld[3];
dNew[1] = + acadoWorkspace.QN1[4]*dOld[0] + acadoWorkspace.QN1[5]*dOld[1] + acadoWorkspace.QN1[6]*dOld[2] + acadoWorkspace.QN1[7]*dOld[3];
dNew[2] = + acadoWorkspace.QN1[8]*dOld[0] + acadoWorkspace.QN1[9]*dOld[1] + acadoWorkspace.QN1[10]*dOld[2] + acadoWorkspace.QN1[11]*dOld[3];
dNew[3] = + acadoWorkspace.QN1[12]*dOld[0] + acadoWorkspace.QN1[13]*dOld[1] + acadoWorkspace.QN1[14]*dOld[2] + acadoWorkspace.QN1[15]*dOld[3];
}

void acado_multRDy( real_t* const R2, real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = + R2[0]*Dy1[0] + R2[1]*Dy1[1] + R2[2]*Dy1[2] + R2[3]*Dy1[3] + R2[4]*Dy1[4] + R2[5]*Dy1[5] + R2[6]*Dy1[6];
RDy1[1] = + R2[7]*Dy1[0] + R2[8]*Dy1[1] + R2[9]*Dy1[2] + R2[10]*Dy1[3] + R2[11]*Dy1[4] + R2[12]*Dy1[5] + R2[13]*Dy1[6];
}

void acado_multQDy( real_t* const Q2, real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = + Q2[0]*Dy1[0] + Q2[1]*Dy1[1] + Q2[2]*Dy1[2] + Q2[3]*Dy1[3] + Q2[4]*Dy1[4] + Q2[5]*Dy1[5] + Q2[6]*Dy1[6];
QDy1[1] = + Q2[7]*Dy1[0] + Q2[8]*Dy1[1] + Q2[9]*Dy1[2] + Q2[10]*Dy1[3] + Q2[11]*Dy1[4] + Q2[12]*Dy1[5] + Q2[13]*Dy1[6];
QDy1[2] = + Q2[14]*Dy1[0] + Q2[15]*Dy1[1] + Q2[16]*Dy1[2] + Q2[17]*Dy1[3] + Q2[18]*Dy1[4] + Q2[19]*Dy1[5] + Q2[20]*Dy1[6];
QDy1[3] = + Q2[21]*Dy1[0] + Q2[22]*Dy1[1] + Q2[23]*Dy1[2] + Q2[24]*Dy1[3] + Q2[25]*Dy1[4] + Q2[26]*Dy1[5] + Q2[27]*Dy1[6];
}

void acado_multEQDy( real_t* const E1, real_t* const QDy1, real_t* const U1 )
{
U1[0] += + E1[0]*QDy1[0] + E1[2]*QDy1[1] + E1[4]*QDy1[2] + E1[6]*QDy1[3];
U1[1] += + E1[1]*QDy1[0] + E1[3]*QDy1[1] + E1[5]*QDy1[2] + E1[7]*QDy1[3];
}

void acado_multQETGx( real_t* const E1, real_t* const Gx1, real_t* const H101 )
{
H101[0] += + E1[0]*Gx1[0] + E1[2]*Gx1[4] + E1[4]*Gx1[8] + E1[6]*Gx1[12];
H101[1] += + E1[0]*Gx1[1] + E1[2]*Gx1[5] + E1[4]*Gx1[9] + E1[6]*Gx1[13];
H101[2] += + E1[0]*Gx1[2] + E1[2]*Gx1[6] + E1[4]*Gx1[10] + E1[6]*Gx1[14];
H101[3] += + E1[0]*Gx1[3] + E1[2]*Gx1[7] + E1[4]*Gx1[11] + E1[6]*Gx1[15];
H101[4] += + E1[1]*Gx1[0] + E1[3]*Gx1[4] + E1[5]*Gx1[8] + E1[7]*Gx1[12];
H101[5] += + E1[1]*Gx1[1] + E1[3]*Gx1[5] + E1[5]*Gx1[9] + E1[7]*Gx1[13];
H101[6] += + E1[1]*Gx1[2] + E1[3]*Gx1[6] + E1[5]*Gx1[10] + E1[7]*Gx1[14];
H101[7] += + E1[1]*Gx1[3] + E1[3]*Gx1[7] + E1[5]*Gx1[11] + E1[7]*Gx1[15];
}

void acado_zeroBlockH10( real_t* const H101 )
{
{ int lCopy; for (lCopy = 0; lCopy < 8; lCopy++) H101[ lCopy ] = 0; }
}

void acado_multEDu( real_t* const E1, real_t* const U1, real_t* const dNew )
{
dNew[0] += + E1[0]*U1[0] + E1[1]*U1[1];
dNew[1] += + E1[2]*U1[0] + E1[3]*U1[1];
dNew[2] += + E1[4]*U1[0] + E1[5]*U1[1];
dNew[3] += + E1[6]*U1[0] + E1[7]*U1[1];
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
acado_moveGxT( &(acadoWorkspace.evGx[ 16 ]), acadoWorkspace.T );
acado_multGxd( acadoWorkspace.d, &(acadoWorkspace.evGx[ 16 ]), &(acadoWorkspace.d[ 4 ]) );
acado_multGxGx( acadoWorkspace.T, acadoWorkspace.evGx, &(acadoWorkspace.evGx[ 16 ]) );

acado_multGxGu( acadoWorkspace.T, acadoWorkspace.E, &(acadoWorkspace.E[ 8 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 8 ]), &(acadoWorkspace.E[ 16 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 32 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 4 ]), &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.d[ 8 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 16 ]), &(acadoWorkspace.evGx[ 32 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 8 ]), &(acadoWorkspace.E[ 24 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 16 ]), &(acadoWorkspace.E[ 32 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 16 ]), &(acadoWorkspace.E[ 40 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 48 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 8 ]), &(acadoWorkspace.evGx[ 48 ]), &(acadoWorkspace.d[ 12 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.evGx[ 48 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 24 ]), &(acadoWorkspace.E[ 48 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 32 ]), &(acadoWorkspace.E[ 56 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 40 ]), &(acadoWorkspace.E[ 64 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 24 ]), &(acadoWorkspace.E[ 72 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 64 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 12 ]), &(acadoWorkspace.evGx[ 64 ]), &(acadoWorkspace.d[ 16 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 48 ]), &(acadoWorkspace.evGx[ 64 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.E[ 80 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 56 ]), &(acadoWorkspace.E[ 88 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 64 ]), &(acadoWorkspace.E[ 96 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.E[ 104 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 32 ]), &(acadoWorkspace.E[ 112 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 80 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 16 ]), &(acadoWorkspace.evGx[ 80 ]), &(acadoWorkspace.d[ 20 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 64 ]), &(acadoWorkspace.evGx[ 80 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 80 ]), &(acadoWorkspace.E[ 120 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 88 ]), &(acadoWorkspace.E[ 128 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.E[ 136 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 104 ]), &(acadoWorkspace.E[ 144 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 112 ]), &(acadoWorkspace.E[ 152 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 40 ]), &(acadoWorkspace.E[ 160 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 96 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 20 ]), &(acadoWorkspace.evGx[ 96 ]), &(acadoWorkspace.d[ 24 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 80 ]), &(acadoWorkspace.evGx[ 96 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.E[ 168 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 128 ]), &(acadoWorkspace.E[ 176 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 136 ]), &(acadoWorkspace.E[ 184 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.E[ 192 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 152 ]), &(acadoWorkspace.E[ 200 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 160 ]), &(acadoWorkspace.E[ 208 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 48 ]), &(acadoWorkspace.E[ 216 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 112 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 24 ]), &(acadoWorkspace.evGx[ 112 ]), &(acadoWorkspace.d[ 28 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 96 ]), &(acadoWorkspace.evGx[ 112 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.E[ 224 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 176 ]), &(acadoWorkspace.E[ 232 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 184 ]), &(acadoWorkspace.E[ 240 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.E[ 248 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 200 ]), &(acadoWorkspace.E[ 256 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 208 ]), &(acadoWorkspace.E[ 264 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.E[ 272 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 56 ]), &(acadoWorkspace.E[ 280 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 128 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 28 ]), &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.d[ 32 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 112 ]), &(acadoWorkspace.evGx[ 128 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 224 ]), &(acadoWorkspace.E[ 288 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 232 ]), &(acadoWorkspace.E[ 296 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.E[ 304 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 248 ]), &(acadoWorkspace.E[ 312 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 256 ]), &(acadoWorkspace.E[ 320 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 264 ]), &(acadoWorkspace.E[ 328 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 272 ]), &(acadoWorkspace.E[ 336 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 280 ]), &(acadoWorkspace.E[ 344 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 64 ]), &(acadoWorkspace.E[ 352 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 144 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 32 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.d[ 36 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.evGx[ 144 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.E[ 360 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 296 ]), &(acadoWorkspace.E[ 368 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 304 ]), &(acadoWorkspace.E[ 376 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 312 ]), &(acadoWorkspace.E[ 384 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 320 ]), &(acadoWorkspace.E[ 392 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 328 ]), &(acadoWorkspace.E[ 400 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.E[ 408 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 344 ]), &(acadoWorkspace.E[ 416 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 352 ]), &(acadoWorkspace.E[ 424 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 72 ]), &(acadoWorkspace.E[ 432 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 160 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 36 ]), &(acadoWorkspace.evGx[ 160 ]), &(acadoWorkspace.d[ 40 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.evGx[ 160 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.E[ 440 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 368 ]), &(acadoWorkspace.E[ 448 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 376 ]), &(acadoWorkspace.E[ 456 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.E[ 464 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 392 ]), &(acadoWorkspace.E[ 472 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 400 ]), &(acadoWorkspace.E[ 480 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 408 ]), &(acadoWorkspace.E[ 488 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 416 ]), &(acadoWorkspace.E[ 496 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 424 ]), &(acadoWorkspace.E[ 504 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.E[ 512 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 80 ]), &(acadoWorkspace.E[ 520 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 176 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 40 ]), &(acadoWorkspace.evGx[ 176 ]), &(acadoWorkspace.d[ 44 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 160 ]), &(acadoWorkspace.evGx[ 176 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 440 ]), &(acadoWorkspace.E[ 528 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 448 ]), &(acadoWorkspace.E[ 536 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 456 ]), &(acadoWorkspace.E[ 544 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 464 ]), &(acadoWorkspace.E[ 552 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 472 ]), &(acadoWorkspace.E[ 560 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.E[ 568 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 488 ]), &(acadoWorkspace.E[ 576 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 496 ]), &(acadoWorkspace.E[ 584 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 504 ]), &(acadoWorkspace.E[ 592 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 512 ]), &(acadoWorkspace.E[ 600 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 520 ]), &(acadoWorkspace.E[ 608 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 88 ]), &(acadoWorkspace.E[ 616 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 192 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 44 ]), &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.d[ 48 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 176 ]), &(acadoWorkspace.evGx[ 192 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.E[ 624 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 536 ]), &(acadoWorkspace.E[ 632 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 544 ]), &(acadoWorkspace.E[ 640 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.E[ 648 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 560 ]), &(acadoWorkspace.E[ 656 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 568 ]), &(acadoWorkspace.E[ 664 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.E[ 672 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 584 ]), &(acadoWorkspace.E[ 680 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 592 ]), &(acadoWorkspace.E[ 688 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 600 ]), &(acadoWorkspace.E[ 696 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 608 ]), &(acadoWorkspace.E[ 704 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 616 ]), &(acadoWorkspace.E[ 712 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 96 ]), &(acadoWorkspace.E[ 720 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 208 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 48 ]), &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.d[ 52 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.evGx[ 208 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.E[ 728 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 632 ]), &(acadoWorkspace.E[ 736 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 640 ]), &(acadoWorkspace.E[ 744 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 648 ]), &(acadoWorkspace.E[ 752 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 656 ]), &(acadoWorkspace.E[ 760 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 664 ]), &(acadoWorkspace.E[ 768 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.E[ 776 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 680 ]), &(acadoWorkspace.E[ 784 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 688 ]), &(acadoWorkspace.E[ 792 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 696 ]), &(acadoWorkspace.E[ 800 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 704 ]), &(acadoWorkspace.E[ 808 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 712 ]), &(acadoWorkspace.E[ 816 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.E[ 824 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 104 ]), &(acadoWorkspace.E[ 832 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 224 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 52 ]), &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.d[ 56 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.evGx[ 224 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 728 ]), &(acadoWorkspace.E[ 840 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 736 ]), &(acadoWorkspace.E[ 848 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 744 ]), &(acadoWorkspace.E[ 856 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 752 ]), &(acadoWorkspace.E[ 864 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 760 ]), &(acadoWorkspace.E[ 872 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 768 ]), &(acadoWorkspace.E[ 880 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 776 ]), &(acadoWorkspace.E[ 888 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 784 ]), &(acadoWorkspace.E[ 896 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 792 ]), &(acadoWorkspace.E[ 904 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 800 ]), &(acadoWorkspace.E[ 912 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 808 ]), &(acadoWorkspace.E[ 920 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 816 ]), &(acadoWorkspace.E[ 928 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 824 ]), &(acadoWorkspace.E[ 936 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 832 ]), &(acadoWorkspace.E[ 944 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 112 ]), &(acadoWorkspace.E[ 952 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 240 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 56 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.d[ 60 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.evGx[ 240 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 840 ]), &(acadoWorkspace.E[ 960 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 848 ]), &(acadoWorkspace.E[ 968 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 856 ]), &(acadoWorkspace.E[ 976 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.E[ 984 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 872 ]), &(acadoWorkspace.E[ 992 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 880 ]), &(acadoWorkspace.E[ 1000 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 888 ]), &(acadoWorkspace.E[ 1008 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 896 ]), &(acadoWorkspace.E[ 1016 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 904 ]), &(acadoWorkspace.E[ 1024 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 912 ]), &(acadoWorkspace.E[ 1032 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 920 ]), &(acadoWorkspace.E[ 1040 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 928 ]), &(acadoWorkspace.E[ 1048 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 936 ]), &(acadoWorkspace.E[ 1056 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 944 ]), &(acadoWorkspace.E[ 1064 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 952 ]), &(acadoWorkspace.E[ 1072 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 120 ]), &(acadoWorkspace.E[ 1080 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 256 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 60 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.d[ 64 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.evGx[ 256 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.E[ 1088 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 968 ]), &(acadoWorkspace.E[ 1096 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 976 ]), &(acadoWorkspace.E[ 1104 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 984 ]), &(acadoWorkspace.E[ 1112 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 992 ]), &(acadoWorkspace.E[ 1120 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1000 ]), &(acadoWorkspace.E[ 1128 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1008 ]), &(acadoWorkspace.E[ 1136 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1016 ]), &(acadoWorkspace.E[ 1144 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1024 ]), &(acadoWorkspace.E[ 1152 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1032 ]), &(acadoWorkspace.E[ 1160 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1040 ]), &(acadoWorkspace.E[ 1168 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1048 ]), &(acadoWorkspace.E[ 1176 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1056 ]), &(acadoWorkspace.E[ 1184 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1064 ]), &(acadoWorkspace.E[ 1192 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1072 ]), &(acadoWorkspace.E[ 1200 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1080 ]), &(acadoWorkspace.E[ 1208 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 128 ]), &(acadoWorkspace.E[ 1216 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 272 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 64 ]), &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.d[ 68 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.evGx[ 272 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1088 ]), &(acadoWorkspace.E[ 1224 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1096 ]), &(acadoWorkspace.E[ 1232 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.E[ 1240 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1112 ]), &(acadoWorkspace.E[ 1248 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1120 ]), &(acadoWorkspace.E[ 1256 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1128 ]), &(acadoWorkspace.E[ 1264 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1136 ]), &(acadoWorkspace.E[ 1272 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1144 ]), &(acadoWorkspace.E[ 1280 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1152 ]), &(acadoWorkspace.E[ 1288 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1160 ]), &(acadoWorkspace.E[ 1296 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1168 ]), &(acadoWorkspace.E[ 1304 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1176 ]), &(acadoWorkspace.E[ 1312 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1184 ]), &(acadoWorkspace.E[ 1320 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1192 ]), &(acadoWorkspace.E[ 1328 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1200 ]), &(acadoWorkspace.E[ 1336 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1208 ]), &(acadoWorkspace.E[ 1344 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1216 ]), &(acadoWorkspace.E[ 1352 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 136 ]), &(acadoWorkspace.E[ 1360 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 288 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 68 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.d[ 72 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.evGx[ 288 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1224 ]), &(acadoWorkspace.E[ 1368 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1232 ]), &(acadoWorkspace.E[ 1376 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1240 ]), &(acadoWorkspace.E[ 1384 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1248 ]), &(acadoWorkspace.E[ 1392 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1256 ]), &(acadoWorkspace.E[ 1400 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1264 ]), &(acadoWorkspace.E[ 1408 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1272 ]), &(acadoWorkspace.E[ 1416 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1280 ]), &(acadoWorkspace.E[ 1424 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1288 ]), &(acadoWorkspace.E[ 1432 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.E[ 1440 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1304 ]), &(acadoWorkspace.E[ 1448 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1312 ]), &(acadoWorkspace.E[ 1456 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1320 ]), &(acadoWorkspace.E[ 1464 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1328 ]), &(acadoWorkspace.E[ 1472 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1336 ]), &(acadoWorkspace.E[ 1480 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1344 ]), &(acadoWorkspace.E[ 1488 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1352 ]), &(acadoWorkspace.E[ 1496 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1360 ]), &(acadoWorkspace.E[ 1504 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 144 ]), &(acadoWorkspace.E[ 1512 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 304 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 72 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.d[ 76 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.evGx[ 304 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1368 ]), &(acadoWorkspace.E[ 1520 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1376 ]), &(acadoWorkspace.E[ 1528 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1384 ]), &(acadoWorkspace.E[ 1536 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1392 ]), &(acadoWorkspace.E[ 1544 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1400 ]), &(acadoWorkspace.E[ 1552 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1408 ]), &(acadoWorkspace.E[ 1560 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1416 ]), &(acadoWorkspace.E[ 1568 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1424 ]), &(acadoWorkspace.E[ 1576 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1432 ]), &(acadoWorkspace.E[ 1584 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.E[ 1592 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1448 ]), &(acadoWorkspace.E[ 1600 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1456 ]), &(acadoWorkspace.E[ 1608 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1464 ]), &(acadoWorkspace.E[ 1616 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1472 ]), &(acadoWorkspace.E[ 1624 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1480 ]), &(acadoWorkspace.E[ 1632 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.E[ 1640 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1496 ]), &(acadoWorkspace.E[ 1648 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1504 ]), &(acadoWorkspace.E[ 1656 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1512 ]), &(acadoWorkspace.E[ 1664 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 152 ]), &(acadoWorkspace.E[ 1672 ]) );

acado_multGxGu( &(acadoWorkspace.Q1[ 16 ]), acadoWorkspace.E, acadoWorkspace.QE );
acado_multGxGu( &(acadoWorkspace.Q1[ 32 ]), &(acadoWorkspace.E[ 8 ]), &(acadoWorkspace.QE[ 8 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 32 ]), &(acadoWorkspace.E[ 16 ]), &(acadoWorkspace.QE[ 16 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 48 ]), &(acadoWorkspace.E[ 24 ]), &(acadoWorkspace.QE[ 24 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 48 ]), &(acadoWorkspace.E[ 32 ]), &(acadoWorkspace.QE[ 32 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 48 ]), &(acadoWorkspace.E[ 40 ]), &(acadoWorkspace.QE[ 40 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 64 ]), &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QE[ 48 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 64 ]), &(acadoWorkspace.E[ 56 ]), &(acadoWorkspace.QE[ 56 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 64 ]), &(acadoWorkspace.E[ 64 ]), &(acadoWorkspace.QE[ 64 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 64 ]), &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.QE[ 72 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 80 ]), &(acadoWorkspace.E[ 80 ]), &(acadoWorkspace.QE[ 80 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 80 ]), &(acadoWorkspace.E[ 88 ]), &(acadoWorkspace.QE[ 88 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 80 ]), &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QE[ 96 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 80 ]), &(acadoWorkspace.E[ 104 ]), &(acadoWorkspace.QE[ 104 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 80 ]), &(acadoWorkspace.E[ 112 ]), &(acadoWorkspace.QE[ 112 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 96 ]), &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QE[ 120 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 96 ]), &(acadoWorkspace.E[ 128 ]), &(acadoWorkspace.QE[ 128 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 96 ]), &(acadoWorkspace.E[ 136 ]), &(acadoWorkspace.QE[ 136 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 96 ]), &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QE[ 144 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 96 ]), &(acadoWorkspace.E[ 152 ]), &(acadoWorkspace.QE[ 152 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 96 ]), &(acadoWorkspace.E[ 160 ]), &(acadoWorkspace.QE[ 160 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 112 ]), &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.QE[ 168 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 112 ]), &(acadoWorkspace.E[ 176 ]), &(acadoWorkspace.QE[ 176 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 112 ]), &(acadoWorkspace.E[ 184 ]), &(acadoWorkspace.QE[ 184 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 112 ]), &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QE[ 192 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 112 ]), &(acadoWorkspace.E[ 200 ]), &(acadoWorkspace.QE[ 200 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 112 ]), &(acadoWorkspace.E[ 208 ]), &(acadoWorkspace.QE[ 208 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 112 ]), &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.QE[ 216 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 128 ]), &(acadoWorkspace.E[ 224 ]), &(acadoWorkspace.QE[ 224 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 128 ]), &(acadoWorkspace.E[ 232 ]), &(acadoWorkspace.QE[ 232 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 128 ]), &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 240 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 128 ]), &(acadoWorkspace.E[ 248 ]), &(acadoWorkspace.QE[ 248 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 128 ]), &(acadoWorkspace.E[ 256 ]), &(acadoWorkspace.QE[ 256 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 128 ]), &(acadoWorkspace.E[ 264 ]), &(acadoWorkspace.QE[ 264 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 128 ]), &(acadoWorkspace.E[ 272 ]), &(acadoWorkspace.QE[ 272 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 128 ]), &(acadoWorkspace.E[ 280 ]), &(acadoWorkspace.QE[ 280 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 288 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.E[ 296 ]), &(acadoWorkspace.QE[ 296 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.E[ 304 ]), &(acadoWorkspace.QE[ 304 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.E[ 312 ]), &(acadoWorkspace.QE[ 312 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.E[ 320 ]), &(acadoWorkspace.QE[ 320 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.E[ 328 ]), &(acadoWorkspace.QE[ 328 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QE[ 336 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.E[ 344 ]), &(acadoWorkspace.QE[ 344 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.E[ 352 ]), &(acadoWorkspace.QE[ 352 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 160 ]), &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 360 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 160 ]), &(acadoWorkspace.E[ 368 ]), &(acadoWorkspace.QE[ 368 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 160 ]), &(acadoWorkspace.E[ 376 ]), &(acadoWorkspace.QE[ 376 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 160 ]), &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.QE[ 384 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 160 ]), &(acadoWorkspace.E[ 392 ]), &(acadoWorkspace.QE[ 392 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 160 ]), &(acadoWorkspace.E[ 400 ]), &(acadoWorkspace.QE[ 400 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 160 ]), &(acadoWorkspace.E[ 408 ]), &(acadoWorkspace.QE[ 408 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 160 ]), &(acadoWorkspace.E[ 416 ]), &(acadoWorkspace.QE[ 416 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 160 ]), &(acadoWorkspace.E[ 424 ]), &(acadoWorkspace.QE[ 424 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 160 ]), &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QE[ 432 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 176 ]), &(acadoWorkspace.E[ 440 ]), &(acadoWorkspace.QE[ 440 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 176 ]), &(acadoWorkspace.E[ 448 ]), &(acadoWorkspace.QE[ 448 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 176 ]), &(acadoWorkspace.E[ 456 ]), &(acadoWorkspace.QE[ 456 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 176 ]), &(acadoWorkspace.E[ 464 ]), &(acadoWorkspace.QE[ 464 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 176 ]), &(acadoWorkspace.E[ 472 ]), &(acadoWorkspace.QE[ 472 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 176 ]), &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 480 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 176 ]), &(acadoWorkspace.E[ 488 ]), &(acadoWorkspace.QE[ 488 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 176 ]), &(acadoWorkspace.E[ 496 ]), &(acadoWorkspace.QE[ 496 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 176 ]), &(acadoWorkspace.E[ 504 ]), &(acadoWorkspace.QE[ 504 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 176 ]), &(acadoWorkspace.E[ 512 ]), &(acadoWorkspace.QE[ 512 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 176 ]), &(acadoWorkspace.E[ 520 ]), &(acadoWorkspace.QE[ 520 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 192 ]), &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.QE[ 528 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 192 ]), &(acadoWorkspace.E[ 536 ]), &(acadoWorkspace.QE[ 536 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 192 ]), &(acadoWorkspace.E[ 544 ]), &(acadoWorkspace.QE[ 544 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 192 ]), &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.QE[ 552 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 192 ]), &(acadoWorkspace.E[ 560 ]), &(acadoWorkspace.QE[ 560 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 192 ]), &(acadoWorkspace.E[ 568 ]), &(acadoWorkspace.QE[ 568 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 192 ]), &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 576 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 192 ]), &(acadoWorkspace.E[ 584 ]), &(acadoWorkspace.QE[ 584 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 192 ]), &(acadoWorkspace.E[ 592 ]), &(acadoWorkspace.QE[ 592 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 192 ]), &(acadoWorkspace.E[ 600 ]), &(acadoWorkspace.QE[ 600 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 192 ]), &(acadoWorkspace.E[ 608 ]), &(acadoWorkspace.QE[ 608 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 192 ]), &(acadoWorkspace.E[ 616 ]), &(acadoWorkspace.QE[ 616 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 208 ]), &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.QE[ 624 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 208 ]), &(acadoWorkspace.E[ 632 ]), &(acadoWorkspace.QE[ 632 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 208 ]), &(acadoWorkspace.E[ 640 ]), &(acadoWorkspace.QE[ 640 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 208 ]), &(acadoWorkspace.E[ 648 ]), &(acadoWorkspace.QE[ 648 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 208 ]), &(acadoWorkspace.E[ 656 ]), &(acadoWorkspace.QE[ 656 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 208 ]), &(acadoWorkspace.E[ 664 ]), &(acadoWorkspace.QE[ 664 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 208 ]), &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.QE[ 672 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 208 ]), &(acadoWorkspace.E[ 680 ]), &(acadoWorkspace.QE[ 680 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 208 ]), &(acadoWorkspace.E[ 688 ]), &(acadoWorkspace.QE[ 688 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 208 ]), &(acadoWorkspace.E[ 696 ]), &(acadoWorkspace.QE[ 696 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 208 ]), &(acadoWorkspace.E[ 704 ]), &(acadoWorkspace.QE[ 704 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 208 ]), &(acadoWorkspace.E[ 712 ]), &(acadoWorkspace.QE[ 712 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 208 ]), &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.QE[ 720 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 224 ]), &(acadoWorkspace.E[ 728 ]), &(acadoWorkspace.QE[ 728 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 224 ]), &(acadoWorkspace.E[ 736 ]), &(acadoWorkspace.QE[ 736 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 224 ]), &(acadoWorkspace.E[ 744 ]), &(acadoWorkspace.QE[ 744 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 224 ]), &(acadoWorkspace.E[ 752 ]), &(acadoWorkspace.QE[ 752 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 224 ]), &(acadoWorkspace.E[ 760 ]), &(acadoWorkspace.QE[ 760 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 224 ]), &(acadoWorkspace.E[ 768 ]), &(acadoWorkspace.QE[ 768 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 224 ]), &(acadoWorkspace.E[ 776 ]), &(acadoWorkspace.QE[ 776 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 224 ]), &(acadoWorkspace.E[ 784 ]), &(acadoWorkspace.QE[ 784 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 224 ]), &(acadoWorkspace.E[ 792 ]), &(acadoWorkspace.QE[ 792 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 224 ]), &(acadoWorkspace.E[ 800 ]), &(acadoWorkspace.QE[ 800 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 224 ]), &(acadoWorkspace.E[ 808 ]), &(acadoWorkspace.QE[ 808 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 224 ]), &(acadoWorkspace.E[ 816 ]), &(acadoWorkspace.QE[ 816 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 224 ]), &(acadoWorkspace.E[ 824 ]), &(acadoWorkspace.QE[ 824 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 224 ]), &(acadoWorkspace.E[ 832 ]), &(acadoWorkspace.QE[ 832 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 240 ]), &(acadoWorkspace.E[ 840 ]), &(acadoWorkspace.QE[ 840 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 240 ]), &(acadoWorkspace.E[ 848 ]), &(acadoWorkspace.QE[ 848 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 240 ]), &(acadoWorkspace.E[ 856 ]), &(acadoWorkspace.QE[ 856 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 240 ]), &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.QE[ 864 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 240 ]), &(acadoWorkspace.E[ 872 ]), &(acadoWorkspace.QE[ 872 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 240 ]), &(acadoWorkspace.E[ 880 ]), &(acadoWorkspace.QE[ 880 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 240 ]), &(acadoWorkspace.E[ 888 ]), &(acadoWorkspace.QE[ 888 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 240 ]), &(acadoWorkspace.E[ 896 ]), &(acadoWorkspace.QE[ 896 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 240 ]), &(acadoWorkspace.E[ 904 ]), &(acadoWorkspace.QE[ 904 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 240 ]), &(acadoWorkspace.E[ 912 ]), &(acadoWorkspace.QE[ 912 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 240 ]), &(acadoWorkspace.E[ 920 ]), &(acadoWorkspace.QE[ 920 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 240 ]), &(acadoWorkspace.E[ 928 ]), &(acadoWorkspace.QE[ 928 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 240 ]), &(acadoWorkspace.E[ 936 ]), &(acadoWorkspace.QE[ 936 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 240 ]), &(acadoWorkspace.E[ 944 ]), &(acadoWorkspace.QE[ 944 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 240 ]), &(acadoWorkspace.E[ 952 ]), &(acadoWorkspace.QE[ 952 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 256 ]), &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 960 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 256 ]), &(acadoWorkspace.E[ 968 ]), &(acadoWorkspace.QE[ 968 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 256 ]), &(acadoWorkspace.E[ 976 ]), &(acadoWorkspace.QE[ 976 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 256 ]), &(acadoWorkspace.E[ 984 ]), &(acadoWorkspace.QE[ 984 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 256 ]), &(acadoWorkspace.E[ 992 ]), &(acadoWorkspace.QE[ 992 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 256 ]), &(acadoWorkspace.E[ 1000 ]), &(acadoWorkspace.QE[ 1000 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 256 ]), &(acadoWorkspace.E[ 1008 ]), &(acadoWorkspace.QE[ 1008 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 256 ]), &(acadoWorkspace.E[ 1016 ]), &(acadoWorkspace.QE[ 1016 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 256 ]), &(acadoWorkspace.E[ 1024 ]), &(acadoWorkspace.QE[ 1024 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 256 ]), &(acadoWorkspace.E[ 1032 ]), &(acadoWorkspace.QE[ 1032 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 256 ]), &(acadoWorkspace.E[ 1040 ]), &(acadoWorkspace.QE[ 1040 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 256 ]), &(acadoWorkspace.E[ 1048 ]), &(acadoWorkspace.QE[ 1048 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 256 ]), &(acadoWorkspace.E[ 1056 ]), &(acadoWorkspace.QE[ 1056 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 256 ]), &(acadoWorkspace.E[ 1064 ]), &(acadoWorkspace.QE[ 1064 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 256 ]), &(acadoWorkspace.E[ 1072 ]), &(acadoWorkspace.QE[ 1072 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 256 ]), &(acadoWorkspace.E[ 1080 ]), &(acadoWorkspace.QE[ 1080 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 272 ]), &(acadoWorkspace.E[ 1088 ]), &(acadoWorkspace.QE[ 1088 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 272 ]), &(acadoWorkspace.E[ 1096 ]), &(acadoWorkspace.QE[ 1096 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 272 ]), &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QE[ 1104 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 272 ]), &(acadoWorkspace.E[ 1112 ]), &(acadoWorkspace.QE[ 1112 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 272 ]), &(acadoWorkspace.E[ 1120 ]), &(acadoWorkspace.QE[ 1120 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 272 ]), &(acadoWorkspace.E[ 1128 ]), &(acadoWorkspace.QE[ 1128 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 272 ]), &(acadoWorkspace.E[ 1136 ]), &(acadoWorkspace.QE[ 1136 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 272 ]), &(acadoWorkspace.E[ 1144 ]), &(acadoWorkspace.QE[ 1144 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 272 ]), &(acadoWorkspace.E[ 1152 ]), &(acadoWorkspace.QE[ 1152 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 272 ]), &(acadoWorkspace.E[ 1160 ]), &(acadoWorkspace.QE[ 1160 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 272 ]), &(acadoWorkspace.E[ 1168 ]), &(acadoWorkspace.QE[ 1168 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 272 ]), &(acadoWorkspace.E[ 1176 ]), &(acadoWorkspace.QE[ 1176 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 272 ]), &(acadoWorkspace.E[ 1184 ]), &(acadoWorkspace.QE[ 1184 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 272 ]), &(acadoWorkspace.E[ 1192 ]), &(acadoWorkspace.QE[ 1192 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 272 ]), &(acadoWorkspace.E[ 1200 ]), &(acadoWorkspace.QE[ 1200 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 272 ]), &(acadoWorkspace.E[ 1208 ]), &(acadoWorkspace.QE[ 1208 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 272 ]), &(acadoWorkspace.E[ 1216 ]), &(acadoWorkspace.QE[ 1216 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.E[ 1224 ]), &(acadoWorkspace.QE[ 1224 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.E[ 1232 ]), &(acadoWorkspace.QE[ 1232 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.E[ 1240 ]), &(acadoWorkspace.QE[ 1240 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.E[ 1248 ]), &(acadoWorkspace.QE[ 1248 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.E[ 1256 ]), &(acadoWorkspace.QE[ 1256 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.E[ 1264 ]), &(acadoWorkspace.QE[ 1264 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.E[ 1272 ]), &(acadoWorkspace.QE[ 1272 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.E[ 1280 ]), &(acadoWorkspace.QE[ 1280 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.E[ 1288 ]), &(acadoWorkspace.QE[ 1288 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.QE[ 1296 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.E[ 1304 ]), &(acadoWorkspace.QE[ 1304 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.E[ 1312 ]), &(acadoWorkspace.QE[ 1312 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.E[ 1320 ]), &(acadoWorkspace.QE[ 1320 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.E[ 1328 ]), &(acadoWorkspace.QE[ 1328 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.E[ 1336 ]), &(acadoWorkspace.QE[ 1336 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.E[ 1344 ]), &(acadoWorkspace.QE[ 1344 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.E[ 1352 ]), &(acadoWorkspace.QE[ 1352 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.E[ 1360 ]), &(acadoWorkspace.QE[ 1360 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 304 ]), &(acadoWorkspace.E[ 1368 ]), &(acadoWorkspace.QE[ 1368 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 304 ]), &(acadoWorkspace.E[ 1376 ]), &(acadoWorkspace.QE[ 1376 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 304 ]), &(acadoWorkspace.E[ 1384 ]), &(acadoWorkspace.QE[ 1384 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 304 ]), &(acadoWorkspace.E[ 1392 ]), &(acadoWorkspace.QE[ 1392 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 304 ]), &(acadoWorkspace.E[ 1400 ]), &(acadoWorkspace.QE[ 1400 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 304 ]), &(acadoWorkspace.E[ 1408 ]), &(acadoWorkspace.QE[ 1408 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 304 ]), &(acadoWorkspace.E[ 1416 ]), &(acadoWorkspace.QE[ 1416 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 304 ]), &(acadoWorkspace.E[ 1424 ]), &(acadoWorkspace.QE[ 1424 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 304 ]), &(acadoWorkspace.E[ 1432 ]), &(acadoWorkspace.QE[ 1432 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 304 ]), &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QE[ 1440 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 304 ]), &(acadoWorkspace.E[ 1448 ]), &(acadoWorkspace.QE[ 1448 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 304 ]), &(acadoWorkspace.E[ 1456 ]), &(acadoWorkspace.QE[ 1456 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 304 ]), &(acadoWorkspace.E[ 1464 ]), &(acadoWorkspace.QE[ 1464 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 304 ]), &(acadoWorkspace.E[ 1472 ]), &(acadoWorkspace.QE[ 1472 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 304 ]), &(acadoWorkspace.E[ 1480 ]), &(acadoWorkspace.QE[ 1480 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 304 ]), &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.QE[ 1488 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 304 ]), &(acadoWorkspace.E[ 1496 ]), &(acadoWorkspace.QE[ 1496 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 304 ]), &(acadoWorkspace.E[ 1504 ]), &(acadoWorkspace.QE[ 1504 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 304 ]), &(acadoWorkspace.E[ 1512 ]), &(acadoWorkspace.QE[ 1512 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 1520 ]), &(acadoWorkspace.QE[ 1520 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 1528 ]), &(acadoWorkspace.QE[ 1528 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.QE[ 1536 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 1544 ]), &(acadoWorkspace.QE[ 1544 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 1552 ]), &(acadoWorkspace.QE[ 1552 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 1560 ]), &(acadoWorkspace.QE[ 1560 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 1568 ]), &(acadoWorkspace.QE[ 1568 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 1576 ]), &(acadoWorkspace.QE[ 1576 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 1584 ]), &(acadoWorkspace.QE[ 1584 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 1592 ]), &(acadoWorkspace.QE[ 1592 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 1600 ]), &(acadoWorkspace.QE[ 1600 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 1608 ]), &(acadoWorkspace.QE[ 1608 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 1616 ]), &(acadoWorkspace.QE[ 1616 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 1624 ]), &(acadoWorkspace.QE[ 1624 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 1632 ]), &(acadoWorkspace.QE[ 1632 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 1640 ]), &(acadoWorkspace.QE[ 1640 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 1648 ]), &(acadoWorkspace.QE[ 1648 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 1656 ]), &(acadoWorkspace.QE[ 1656 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 1664 ]), &(acadoWorkspace.QE[ 1664 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 1672 ]), &(acadoWorkspace.QE[ 1672 ]) );

acado_zeroBlockH10( acadoWorkspace.H10 );
acado_multQETGx( acadoWorkspace.QE, acadoWorkspace.evGx, acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 8 ]), &(acadoWorkspace.evGx[ 16 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 24 ]), &(acadoWorkspace.evGx[ 32 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 48 ]), &(acadoWorkspace.evGx[ 48 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 80 ]), &(acadoWorkspace.evGx[ 64 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 120 ]), &(acadoWorkspace.evGx[ 80 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 168 ]), &(acadoWorkspace.evGx[ 96 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 224 ]), &(acadoWorkspace.evGx[ 112 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 288 ]), &(acadoWorkspace.evGx[ 128 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 360 ]), &(acadoWorkspace.evGx[ 144 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 440 ]), &(acadoWorkspace.evGx[ 160 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 528 ]), &(acadoWorkspace.evGx[ 176 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 624 ]), &(acadoWorkspace.evGx[ 192 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 728 ]), &(acadoWorkspace.evGx[ 208 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 840 ]), &(acadoWorkspace.evGx[ 224 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 960 ]), &(acadoWorkspace.evGx[ 240 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 1088 ]), &(acadoWorkspace.evGx[ 256 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 1224 ]), &(acadoWorkspace.evGx[ 272 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 1368 ]), &(acadoWorkspace.evGx[ 288 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 1520 ]), &(acadoWorkspace.evGx[ 304 ]), acadoWorkspace.H10 );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 8 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 16 ]), &(acadoWorkspace.evGx[ 16 ]), &(acadoWorkspace.H10[ 8 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 32 ]), &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.H10[ 8 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 56 ]), &(acadoWorkspace.evGx[ 48 ]), &(acadoWorkspace.H10[ 8 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 88 ]), &(acadoWorkspace.evGx[ 64 ]), &(acadoWorkspace.H10[ 8 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 128 ]), &(acadoWorkspace.evGx[ 80 ]), &(acadoWorkspace.H10[ 8 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 176 ]), &(acadoWorkspace.evGx[ 96 ]), &(acadoWorkspace.H10[ 8 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 232 ]), &(acadoWorkspace.evGx[ 112 ]), &(acadoWorkspace.H10[ 8 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 296 ]), &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.H10[ 8 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 368 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.H10[ 8 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 448 ]), &(acadoWorkspace.evGx[ 160 ]), &(acadoWorkspace.H10[ 8 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 536 ]), &(acadoWorkspace.evGx[ 176 ]), &(acadoWorkspace.H10[ 8 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 632 ]), &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.H10[ 8 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 736 ]), &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.H10[ 8 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 848 ]), &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.H10[ 8 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 968 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.H10[ 8 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1096 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.H10[ 8 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1232 ]), &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.H10[ 8 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1376 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 8 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1528 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.H10[ 8 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 16 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 40 ]), &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.H10[ 16 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 64 ]), &(acadoWorkspace.evGx[ 48 ]), &(acadoWorkspace.H10[ 16 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 96 ]), &(acadoWorkspace.evGx[ 64 ]), &(acadoWorkspace.H10[ 16 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 136 ]), &(acadoWorkspace.evGx[ 80 ]), &(acadoWorkspace.H10[ 16 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 184 ]), &(acadoWorkspace.evGx[ 96 ]), &(acadoWorkspace.H10[ 16 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 240 ]), &(acadoWorkspace.evGx[ 112 ]), &(acadoWorkspace.H10[ 16 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 304 ]), &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.H10[ 16 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 376 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.H10[ 16 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 456 ]), &(acadoWorkspace.evGx[ 160 ]), &(acadoWorkspace.H10[ 16 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 544 ]), &(acadoWorkspace.evGx[ 176 ]), &(acadoWorkspace.H10[ 16 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 640 ]), &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.H10[ 16 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 744 ]), &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.H10[ 16 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 856 ]), &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.H10[ 16 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 976 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.H10[ 16 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1104 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.H10[ 16 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1240 ]), &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.H10[ 16 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1384 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 16 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1536 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.H10[ 16 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 72 ]), &(acadoWorkspace.evGx[ 48 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 104 ]), &(acadoWorkspace.evGx[ 64 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 144 ]), &(acadoWorkspace.evGx[ 80 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 192 ]), &(acadoWorkspace.evGx[ 96 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 248 ]), &(acadoWorkspace.evGx[ 112 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 312 ]), &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 384 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 464 ]), &(acadoWorkspace.evGx[ 160 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 552 ]), &(acadoWorkspace.evGx[ 176 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 648 ]), &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 752 ]), &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 864 ]), &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 984 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1112 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1248 ]), &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1392 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1544 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 32 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 112 ]), &(acadoWorkspace.evGx[ 64 ]), &(acadoWorkspace.H10[ 32 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 152 ]), &(acadoWorkspace.evGx[ 80 ]), &(acadoWorkspace.H10[ 32 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 200 ]), &(acadoWorkspace.evGx[ 96 ]), &(acadoWorkspace.H10[ 32 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 256 ]), &(acadoWorkspace.evGx[ 112 ]), &(acadoWorkspace.H10[ 32 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 320 ]), &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.H10[ 32 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 392 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.H10[ 32 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 472 ]), &(acadoWorkspace.evGx[ 160 ]), &(acadoWorkspace.H10[ 32 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 560 ]), &(acadoWorkspace.evGx[ 176 ]), &(acadoWorkspace.H10[ 32 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 656 ]), &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.H10[ 32 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 760 ]), &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.H10[ 32 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 872 ]), &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.H10[ 32 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 992 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.H10[ 32 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1120 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.H10[ 32 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1256 ]), &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.H10[ 32 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1400 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 32 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1552 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.H10[ 32 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 40 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 160 ]), &(acadoWorkspace.evGx[ 80 ]), &(acadoWorkspace.H10[ 40 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 208 ]), &(acadoWorkspace.evGx[ 96 ]), &(acadoWorkspace.H10[ 40 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 264 ]), &(acadoWorkspace.evGx[ 112 ]), &(acadoWorkspace.H10[ 40 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 328 ]), &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.H10[ 40 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 400 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.H10[ 40 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 480 ]), &(acadoWorkspace.evGx[ 160 ]), &(acadoWorkspace.H10[ 40 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 568 ]), &(acadoWorkspace.evGx[ 176 ]), &(acadoWorkspace.H10[ 40 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 664 ]), &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.H10[ 40 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 768 ]), &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.H10[ 40 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 880 ]), &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.H10[ 40 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1000 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.H10[ 40 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1128 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.H10[ 40 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1264 ]), &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.H10[ 40 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1408 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 40 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1560 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.H10[ 40 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 216 ]), &(acadoWorkspace.evGx[ 96 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 272 ]), &(acadoWorkspace.evGx[ 112 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 336 ]), &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 408 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 488 ]), &(acadoWorkspace.evGx[ 160 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 576 ]), &(acadoWorkspace.evGx[ 176 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 672 ]), &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 776 ]), &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 888 ]), &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1008 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1136 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1272 ]), &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1416 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1568 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 56 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 280 ]), &(acadoWorkspace.evGx[ 112 ]), &(acadoWorkspace.H10[ 56 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 344 ]), &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.H10[ 56 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 416 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.H10[ 56 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 496 ]), &(acadoWorkspace.evGx[ 160 ]), &(acadoWorkspace.H10[ 56 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 584 ]), &(acadoWorkspace.evGx[ 176 ]), &(acadoWorkspace.H10[ 56 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 680 ]), &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.H10[ 56 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 784 ]), &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.H10[ 56 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 896 ]), &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.H10[ 56 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1016 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.H10[ 56 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1144 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.H10[ 56 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1280 ]), &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.H10[ 56 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1424 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 56 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1576 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.H10[ 56 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 64 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 352 ]), &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.H10[ 64 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 424 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.H10[ 64 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 504 ]), &(acadoWorkspace.evGx[ 160 ]), &(acadoWorkspace.H10[ 64 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 592 ]), &(acadoWorkspace.evGx[ 176 ]), &(acadoWorkspace.H10[ 64 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 688 ]), &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.H10[ 64 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 792 ]), &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.H10[ 64 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 904 ]), &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.H10[ 64 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1024 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.H10[ 64 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1152 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.H10[ 64 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1288 ]), &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.H10[ 64 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1432 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 64 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1584 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.H10[ 64 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 72 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 432 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.H10[ 72 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 512 ]), &(acadoWorkspace.evGx[ 160 ]), &(acadoWorkspace.H10[ 72 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 600 ]), &(acadoWorkspace.evGx[ 176 ]), &(acadoWorkspace.H10[ 72 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 696 ]), &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.H10[ 72 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 800 ]), &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.H10[ 72 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 912 ]), &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.H10[ 72 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1032 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.H10[ 72 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1160 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.H10[ 72 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1296 ]), &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.H10[ 72 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1440 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 72 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1592 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.H10[ 72 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 80 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 520 ]), &(acadoWorkspace.evGx[ 160 ]), &(acadoWorkspace.H10[ 80 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 608 ]), &(acadoWorkspace.evGx[ 176 ]), &(acadoWorkspace.H10[ 80 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 704 ]), &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.H10[ 80 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 808 ]), &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.H10[ 80 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 920 ]), &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.H10[ 80 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1040 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.H10[ 80 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1168 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.H10[ 80 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1304 ]), &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.H10[ 80 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1448 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 80 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1600 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.H10[ 80 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 88 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 616 ]), &(acadoWorkspace.evGx[ 176 ]), &(acadoWorkspace.H10[ 88 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 712 ]), &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.H10[ 88 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 816 ]), &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.H10[ 88 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 928 ]), &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.H10[ 88 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1048 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.H10[ 88 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1176 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.H10[ 88 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1312 ]), &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.H10[ 88 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1456 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 88 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1608 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.H10[ 88 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 720 ]), &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 824 ]), &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 936 ]), &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1056 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1184 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1320 ]), &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1464 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1616 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 104 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 832 ]), &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.H10[ 104 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 944 ]), &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.H10[ 104 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1064 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.H10[ 104 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1192 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.H10[ 104 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1328 ]), &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.H10[ 104 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1472 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 104 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1624 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.H10[ 104 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 112 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 952 ]), &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.H10[ 112 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1072 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.H10[ 112 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1200 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.H10[ 112 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1336 ]), &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.H10[ 112 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1480 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 112 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1632 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.H10[ 112 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 120 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1080 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.H10[ 120 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1208 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.H10[ 120 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1344 ]), &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.H10[ 120 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1488 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 120 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1640 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.H10[ 120 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 128 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1216 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.H10[ 128 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1352 ]), &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.H10[ 128 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1496 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 128 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1648 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.H10[ 128 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 136 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1360 ]), &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.H10[ 136 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1504 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 136 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1656 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.H10[ 136 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 144 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1512 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 144 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1664 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.H10[ 144 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 152 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1672 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.H10[ 152 ]) );

acado_setBlockH11_R1( 0, 0, acadoWorkspace.R1 );
acado_setBlockH11( 0, 0, acadoWorkspace.E, acadoWorkspace.QE );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 8 ]), &(acadoWorkspace.QE[ 8 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 24 ]), &(acadoWorkspace.QE[ 24 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QE[ 48 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 80 ]), &(acadoWorkspace.QE[ 80 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QE[ 120 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.QE[ 168 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 224 ]), &(acadoWorkspace.QE[ 224 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 288 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 360 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 440 ]), &(acadoWorkspace.QE[ 440 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.QE[ 528 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.QE[ 624 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 728 ]), &(acadoWorkspace.QE[ 728 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 840 ]), &(acadoWorkspace.QE[ 840 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 960 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 1088 ]), &(acadoWorkspace.QE[ 1088 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 1224 ]), &(acadoWorkspace.QE[ 1224 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 1368 ]), &(acadoWorkspace.QE[ 1368 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 1520 ]), &(acadoWorkspace.QE[ 1520 ]) );

acado_zeroBlockH11( 0, 1 );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 8 ]), &(acadoWorkspace.QE[ 16 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 24 ]), &(acadoWorkspace.QE[ 32 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QE[ 56 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 80 ]), &(acadoWorkspace.QE[ 88 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QE[ 128 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.QE[ 176 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 224 ]), &(acadoWorkspace.QE[ 232 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 296 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 368 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 440 ]), &(acadoWorkspace.QE[ 448 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.QE[ 536 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.QE[ 632 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 728 ]), &(acadoWorkspace.QE[ 736 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 840 ]), &(acadoWorkspace.QE[ 848 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 968 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 1088 ]), &(acadoWorkspace.QE[ 1096 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 1224 ]), &(acadoWorkspace.QE[ 1232 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 1368 ]), &(acadoWorkspace.QE[ 1376 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 1520 ]), &(acadoWorkspace.QE[ 1528 ]) );

acado_zeroBlockH11( 0, 2 );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 24 ]), &(acadoWorkspace.QE[ 40 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QE[ 64 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 80 ]), &(acadoWorkspace.QE[ 96 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QE[ 136 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.QE[ 184 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 224 ]), &(acadoWorkspace.QE[ 240 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 304 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 376 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 440 ]), &(acadoWorkspace.QE[ 456 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.QE[ 544 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.QE[ 640 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 728 ]), &(acadoWorkspace.QE[ 744 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 840 ]), &(acadoWorkspace.QE[ 856 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 976 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 1088 ]), &(acadoWorkspace.QE[ 1104 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 1224 ]), &(acadoWorkspace.QE[ 1240 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 1368 ]), &(acadoWorkspace.QE[ 1384 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 1520 ]), &(acadoWorkspace.QE[ 1536 ]) );

acado_zeroBlockH11( 0, 3 );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QE[ 72 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 80 ]), &(acadoWorkspace.QE[ 104 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QE[ 144 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.QE[ 192 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 224 ]), &(acadoWorkspace.QE[ 248 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 312 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 384 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 440 ]), &(acadoWorkspace.QE[ 464 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.QE[ 552 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.QE[ 648 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 728 ]), &(acadoWorkspace.QE[ 752 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 840 ]), &(acadoWorkspace.QE[ 864 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 984 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 1088 ]), &(acadoWorkspace.QE[ 1112 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 1224 ]), &(acadoWorkspace.QE[ 1248 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 1368 ]), &(acadoWorkspace.QE[ 1392 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 1520 ]), &(acadoWorkspace.QE[ 1544 ]) );

acado_zeroBlockH11( 0, 4 );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 80 ]), &(acadoWorkspace.QE[ 112 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QE[ 152 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.QE[ 200 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 224 ]), &(acadoWorkspace.QE[ 256 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 320 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 392 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 440 ]), &(acadoWorkspace.QE[ 472 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.QE[ 560 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.QE[ 656 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 728 ]), &(acadoWorkspace.QE[ 760 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 840 ]), &(acadoWorkspace.QE[ 872 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 992 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 1088 ]), &(acadoWorkspace.QE[ 1120 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 1224 ]), &(acadoWorkspace.QE[ 1256 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 1368 ]), &(acadoWorkspace.QE[ 1400 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 1520 ]), &(acadoWorkspace.QE[ 1552 ]) );

acado_zeroBlockH11( 0, 5 );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QE[ 160 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.QE[ 208 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 224 ]), &(acadoWorkspace.QE[ 264 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 328 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 400 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 440 ]), &(acadoWorkspace.QE[ 480 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.QE[ 568 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.QE[ 664 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 728 ]), &(acadoWorkspace.QE[ 768 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 840 ]), &(acadoWorkspace.QE[ 880 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 1000 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 1088 ]), &(acadoWorkspace.QE[ 1128 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 1224 ]), &(acadoWorkspace.QE[ 1264 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 1368 ]), &(acadoWorkspace.QE[ 1408 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 1520 ]), &(acadoWorkspace.QE[ 1560 ]) );

acado_zeroBlockH11( 0, 6 );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.QE[ 216 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 224 ]), &(acadoWorkspace.QE[ 272 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 336 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 408 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 440 ]), &(acadoWorkspace.QE[ 488 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.QE[ 576 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.QE[ 672 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 728 ]), &(acadoWorkspace.QE[ 776 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 840 ]), &(acadoWorkspace.QE[ 888 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 1008 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 1088 ]), &(acadoWorkspace.QE[ 1136 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 1224 ]), &(acadoWorkspace.QE[ 1272 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 1368 ]), &(acadoWorkspace.QE[ 1416 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 1520 ]), &(acadoWorkspace.QE[ 1568 ]) );

acado_zeroBlockH11( 0, 7 );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 224 ]), &(acadoWorkspace.QE[ 280 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 344 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 416 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 440 ]), &(acadoWorkspace.QE[ 496 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.QE[ 584 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.QE[ 680 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 728 ]), &(acadoWorkspace.QE[ 784 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 840 ]), &(acadoWorkspace.QE[ 896 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 1016 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 1088 ]), &(acadoWorkspace.QE[ 1144 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 1224 ]), &(acadoWorkspace.QE[ 1280 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 1368 ]), &(acadoWorkspace.QE[ 1424 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 1520 ]), &(acadoWorkspace.QE[ 1576 ]) );

acado_zeroBlockH11( 0, 8 );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 352 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 424 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 440 ]), &(acadoWorkspace.QE[ 504 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.QE[ 592 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.QE[ 688 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 728 ]), &(acadoWorkspace.QE[ 792 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 840 ]), &(acadoWorkspace.QE[ 904 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 1024 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 1088 ]), &(acadoWorkspace.QE[ 1152 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 1224 ]), &(acadoWorkspace.QE[ 1288 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 1368 ]), &(acadoWorkspace.QE[ 1432 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 1520 ]), &(acadoWorkspace.QE[ 1584 ]) );

acado_zeroBlockH11( 0, 9 );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 432 ]) );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 440 ]), &(acadoWorkspace.QE[ 512 ]) );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.QE[ 600 ]) );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.QE[ 696 ]) );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 728 ]), &(acadoWorkspace.QE[ 800 ]) );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 840 ]), &(acadoWorkspace.QE[ 912 ]) );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 1032 ]) );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 1088 ]), &(acadoWorkspace.QE[ 1160 ]) );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 1224 ]), &(acadoWorkspace.QE[ 1296 ]) );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 1368 ]), &(acadoWorkspace.QE[ 1440 ]) );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 1520 ]), &(acadoWorkspace.QE[ 1592 ]) );

acado_zeroBlockH11( 0, 10 );
acado_setBlockH11( 0, 10, &(acadoWorkspace.E[ 440 ]), &(acadoWorkspace.QE[ 520 ]) );
acado_setBlockH11( 0, 10, &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.QE[ 608 ]) );
acado_setBlockH11( 0, 10, &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.QE[ 704 ]) );
acado_setBlockH11( 0, 10, &(acadoWorkspace.E[ 728 ]), &(acadoWorkspace.QE[ 808 ]) );
acado_setBlockH11( 0, 10, &(acadoWorkspace.E[ 840 ]), &(acadoWorkspace.QE[ 920 ]) );
acado_setBlockH11( 0, 10, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 1040 ]) );
acado_setBlockH11( 0, 10, &(acadoWorkspace.E[ 1088 ]), &(acadoWorkspace.QE[ 1168 ]) );
acado_setBlockH11( 0, 10, &(acadoWorkspace.E[ 1224 ]), &(acadoWorkspace.QE[ 1304 ]) );
acado_setBlockH11( 0, 10, &(acadoWorkspace.E[ 1368 ]), &(acadoWorkspace.QE[ 1448 ]) );
acado_setBlockH11( 0, 10, &(acadoWorkspace.E[ 1520 ]), &(acadoWorkspace.QE[ 1600 ]) );

acado_zeroBlockH11( 0, 11 );
acado_setBlockH11( 0, 11, &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.QE[ 616 ]) );
acado_setBlockH11( 0, 11, &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.QE[ 712 ]) );
acado_setBlockH11( 0, 11, &(acadoWorkspace.E[ 728 ]), &(acadoWorkspace.QE[ 816 ]) );
acado_setBlockH11( 0, 11, &(acadoWorkspace.E[ 840 ]), &(acadoWorkspace.QE[ 928 ]) );
acado_setBlockH11( 0, 11, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 1048 ]) );
acado_setBlockH11( 0, 11, &(acadoWorkspace.E[ 1088 ]), &(acadoWorkspace.QE[ 1176 ]) );
acado_setBlockH11( 0, 11, &(acadoWorkspace.E[ 1224 ]), &(acadoWorkspace.QE[ 1312 ]) );
acado_setBlockH11( 0, 11, &(acadoWorkspace.E[ 1368 ]), &(acadoWorkspace.QE[ 1456 ]) );
acado_setBlockH11( 0, 11, &(acadoWorkspace.E[ 1520 ]), &(acadoWorkspace.QE[ 1608 ]) );

acado_zeroBlockH11( 0, 12 );
acado_setBlockH11( 0, 12, &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.QE[ 720 ]) );
acado_setBlockH11( 0, 12, &(acadoWorkspace.E[ 728 ]), &(acadoWorkspace.QE[ 824 ]) );
acado_setBlockH11( 0, 12, &(acadoWorkspace.E[ 840 ]), &(acadoWorkspace.QE[ 936 ]) );
acado_setBlockH11( 0, 12, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 1056 ]) );
acado_setBlockH11( 0, 12, &(acadoWorkspace.E[ 1088 ]), &(acadoWorkspace.QE[ 1184 ]) );
acado_setBlockH11( 0, 12, &(acadoWorkspace.E[ 1224 ]), &(acadoWorkspace.QE[ 1320 ]) );
acado_setBlockH11( 0, 12, &(acadoWorkspace.E[ 1368 ]), &(acadoWorkspace.QE[ 1464 ]) );
acado_setBlockH11( 0, 12, &(acadoWorkspace.E[ 1520 ]), &(acadoWorkspace.QE[ 1616 ]) );

acado_zeroBlockH11( 0, 13 );
acado_setBlockH11( 0, 13, &(acadoWorkspace.E[ 728 ]), &(acadoWorkspace.QE[ 832 ]) );
acado_setBlockH11( 0, 13, &(acadoWorkspace.E[ 840 ]), &(acadoWorkspace.QE[ 944 ]) );
acado_setBlockH11( 0, 13, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 1064 ]) );
acado_setBlockH11( 0, 13, &(acadoWorkspace.E[ 1088 ]), &(acadoWorkspace.QE[ 1192 ]) );
acado_setBlockH11( 0, 13, &(acadoWorkspace.E[ 1224 ]), &(acadoWorkspace.QE[ 1328 ]) );
acado_setBlockH11( 0, 13, &(acadoWorkspace.E[ 1368 ]), &(acadoWorkspace.QE[ 1472 ]) );
acado_setBlockH11( 0, 13, &(acadoWorkspace.E[ 1520 ]), &(acadoWorkspace.QE[ 1624 ]) );

acado_zeroBlockH11( 0, 14 );
acado_setBlockH11( 0, 14, &(acadoWorkspace.E[ 840 ]), &(acadoWorkspace.QE[ 952 ]) );
acado_setBlockH11( 0, 14, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 1072 ]) );
acado_setBlockH11( 0, 14, &(acadoWorkspace.E[ 1088 ]), &(acadoWorkspace.QE[ 1200 ]) );
acado_setBlockH11( 0, 14, &(acadoWorkspace.E[ 1224 ]), &(acadoWorkspace.QE[ 1336 ]) );
acado_setBlockH11( 0, 14, &(acadoWorkspace.E[ 1368 ]), &(acadoWorkspace.QE[ 1480 ]) );
acado_setBlockH11( 0, 14, &(acadoWorkspace.E[ 1520 ]), &(acadoWorkspace.QE[ 1632 ]) );

acado_zeroBlockH11( 0, 15 );
acado_setBlockH11( 0, 15, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 1080 ]) );
acado_setBlockH11( 0, 15, &(acadoWorkspace.E[ 1088 ]), &(acadoWorkspace.QE[ 1208 ]) );
acado_setBlockH11( 0, 15, &(acadoWorkspace.E[ 1224 ]), &(acadoWorkspace.QE[ 1344 ]) );
acado_setBlockH11( 0, 15, &(acadoWorkspace.E[ 1368 ]), &(acadoWorkspace.QE[ 1488 ]) );
acado_setBlockH11( 0, 15, &(acadoWorkspace.E[ 1520 ]), &(acadoWorkspace.QE[ 1640 ]) );

acado_zeroBlockH11( 0, 16 );
acado_setBlockH11( 0, 16, &(acadoWorkspace.E[ 1088 ]), &(acadoWorkspace.QE[ 1216 ]) );
acado_setBlockH11( 0, 16, &(acadoWorkspace.E[ 1224 ]), &(acadoWorkspace.QE[ 1352 ]) );
acado_setBlockH11( 0, 16, &(acadoWorkspace.E[ 1368 ]), &(acadoWorkspace.QE[ 1496 ]) );
acado_setBlockH11( 0, 16, &(acadoWorkspace.E[ 1520 ]), &(acadoWorkspace.QE[ 1648 ]) );

acado_zeroBlockH11( 0, 17 );
acado_setBlockH11( 0, 17, &(acadoWorkspace.E[ 1224 ]), &(acadoWorkspace.QE[ 1360 ]) );
acado_setBlockH11( 0, 17, &(acadoWorkspace.E[ 1368 ]), &(acadoWorkspace.QE[ 1504 ]) );
acado_setBlockH11( 0, 17, &(acadoWorkspace.E[ 1520 ]), &(acadoWorkspace.QE[ 1656 ]) );

acado_zeroBlockH11( 0, 18 );
acado_setBlockH11( 0, 18, &(acadoWorkspace.E[ 1368 ]), &(acadoWorkspace.QE[ 1512 ]) );
acado_setBlockH11( 0, 18, &(acadoWorkspace.E[ 1520 ]), &(acadoWorkspace.QE[ 1664 ]) );

acado_zeroBlockH11( 0, 19 );
acado_setBlockH11( 0, 19, &(acadoWorkspace.E[ 1520 ]), &(acadoWorkspace.QE[ 1672 ]) );

acado_setBlockH11_R1( 1, 1, &(acadoWorkspace.R1[ 4 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 16 ]), &(acadoWorkspace.QE[ 16 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 32 ]), &(acadoWorkspace.QE[ 32 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 56 ]), &(acadoWorkspace.QE[ 56 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 88 ]), &(acadoWorkspace.QE[ 88 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 128 ]), &(acadoWorkspace.QE[ 128 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 176 ]), &(acadoWorkspace.QE[ 176 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 232 ]), &(acadoWorkspace.QE[ 232 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 296 ]), &(acadoWorkspace.QE[ 296 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 368 ]), &(acadoWorkspace.QE[ 368 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 448 ]), &(acadoWorkspace.QE[ 448 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 536 ]), &(acadoWorkspace.QE[ 536 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 632 ]), &(acadoWorkspace.QE[ 632 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 736 ]), &(acadoWorkspace.QE[ 736 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 848 ]), &(acadoWorkspace.QE[ 848 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 968 ]), &(acadoWorkspace.QE[ 968 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 1096 ]), &(acadoWorkspace.QE[ 1096 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 1232 ]), &(acadoWorkspace.QE[ 1232 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 1376 ]), &(acadoWorkspace.QE[ 1376 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 1528 ]), &(acadoWorkspace.QE[ 1528 ]) );

acado_zeroBlockH11( 1, 2 );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 32 ]), &(acadoWorkspace.QE[ 40 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 56 ]), &(acadoWorkspace.QE[ 64 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 88 ]), &(acadoWorkspace.QE[ 96 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 128 ]), &(acadoWorkspace.QE[ 136 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 176 ]), &(acadoWorkspace.QE[ 184 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 232 ]), &(acadoWorkspace.QE[ 240 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 296 ]), &(acadoWorkspace.QE[ 304 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 368 ]), &(acadoWorkspace.QE[ 376 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 448 ]), &(acadoWorkspace.QE[ 456 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 536 ]), &(acadoWorkspace.QE[ 544 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 632 ]), &(acadoWorkspace.QE[ 640 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 736 ]), &(acadoWorkspace.QE[ 744 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 848 ]), &(acadoWorkspace.QE[ 856 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 968 ]), &(acadoWorkspace.QE[ 976 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 1096 ]), &(acadoWorkspace.QE[ 1104 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 1232 ]), &(acadoWorkspace.QE[ 1240 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 1376 ]), &(acadoWorkspace.QE[ 1384 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 1528 ]), &(acadoWorkspace.QE[ 1536 ]) );

acado_zeroBlockH11( 1, 3 );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 56 ]), &(acadoWorkspace.QE[ 72 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 88 ]), &(acadoWorkspace.QE[ 104 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 128 ]), &(acadoWorkspace.QE[ 144 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 176 ]), &(acadoWorkspace.QE[ 192 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 232 ]), &(acadoWorkspace.QE[ 248 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 296 ]), &(acadoWorkspace.QE[ 312 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 368 ]), &(acadoWorkspace.QE[ 384 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 448 ]), &(acadoWorkspace.QE[ 464 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 536 ]), &(acadoWorkspace.QE[ 552 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 632 ]), &(acadoWorkspace.QE[ 648 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 736 ]), &(acadoWorkspace.QE[ 752 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 848 ]), &(acadoWorkspace.QE[ 864 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 968 ]), &(acadoWorkspace.QE[ 984 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 1096 ]), &(acadoWorkspace.QE[ 1112 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 1232 ]), &(acadoWorkspace.QE[ 1248 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 1376 ]), &(acadoWorkspace.QE[ 1392 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 1528 ]), &(acadoWorkspace.QE[ 1544 ]) );

acado_zeroBlockH11( 1, 4 );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 88 ]), &(acadoWorkspace.QE[ 112 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 128 ]), &(acadoWorkspace.QE[ 152 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 176 ]), &(acadoWorkspace.QE[ 200 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 232 ]), &(acadoWorkspace.QE[ 256 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 296 ]), &(acadoWorkspace.QE[ 320 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 368 ]), &(acadoWorkspace.QE[ 392 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 448 ]), &(acadoWorkspace.QE[ 472 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 536 ]), &(acadoWorkspace.QE[ 560 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 632 ]), &(acadoWorkspace.QE[ 656 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 736 ]), &(acadoWorkspace.QE[ 760 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 848 ]), &(acadoWorkspace.QE[ 872 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 968 ]), &(acadoWorkspace.QE[ 992 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 1096 ]), &(acadoWorkspace.QE[ 1120 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 1232 ]), &(acadoWorkspace.QE[ 1256 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 1376 ]), &(acadoWorkspace.QE[ 1400 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 1528 ]), &(acadoWorkspace.QE[ 1552 ]) );

acado_zeroBlockH11( 1, 5 );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 128 ]), &(acadoWorkspace.QE[ 160 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 176 ]), &(acadoWorkspace.QE[ 208 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 232 ]), &(acadoWorkspace.QE[ 264 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 296 ]), &(acadoWorkspace.QE[ 328 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 368 ]), &(acadoWorkspace.QE[ 400 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 448 ]), &(acadoWorkspace.QE[ 480 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 536 ]), &(acadoWorkspace.QE[ 568 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 632 ]), &(acadoWorkspace.QE[ 664 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 736 ]), &(acadoWorkspace.QE[ 768 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 848 ]), &(acadoWorkspace.QE[ 880 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 968 ]), &(acadoWorkspace.QE[ 1000 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 1096 ]), &(acadoWorkspace.QE[ 1128 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 1232 ]), &(acadoWorkspace.QE[ 1264 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 1376 ]), &(acadoWorkspace.QE[ 1408 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 1528 ]), &(acadoWorkspace.QE[ 1560 ]) );

acado_zeroBlockH11( 1, 6 );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 176 ]), &(acadoWorkspace.QE[ 216 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 232 ]), &(acadoWorkspace.QE[ 272 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 296 ]), &(acadoWorkspace.QE[ 336 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 368 ]), &(acadoWorkspace.QE[ 408 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 448 ]), &(acadoWorkspace.QE[ 488 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 536 ]), &(acadoWorkspace.QE[ 576 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 632 ]), &(acadoWorkspace.QE[ 672 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 736 ]), &(acadoWorkspace.QE[ 776 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 848 ]), &(acadoWorkspace.QE[ 888 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 968 ]), &(acadoWorkspace.QE[ 1008 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 1096 ]), &(acadoWorkspace.QE[ 1136 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 1232 ]), &(acadoWorkspace.QE[ 1272 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 1376 ]), &(acadoWorkspace.QE[ 1416 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 1528 ]), &(acadoWorkspace.QE[ 1568 ]) );

acado_zeroBlockH11( 1, 7 );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 232 ]), &(acadoWorkspace.QE[ 280 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 296 ]), &(acadoWorkspace.QE[ 344 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 368 ]), &(acadoWorkspace.QE[ 416 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 448 ]), &(acadoWorkspace.QE[ 496 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 536 ]), &(acadoWorkspace.QE[ 584 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 632 ]), &(acadoWorkspace.QE[ 680 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 736 ]), &(acadoWorkspace.QE[ 784 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 848 ]), &(acadoWorkspace.QE[ 896 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 968 ]), &(acadoWorkspace.QE[ 1016 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 1096 ]), &(acadoWorkspace.QE[ 1144 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 1232 ]), &(acadoWorkspace.QE[ 1280 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 1376 ]), &(acadoWorkspace.QE[ 1424 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 1528 ]), &(acadoWorkspace.QE[ 1576 ]) );

acado_zeroBlockH11( 1, 8 );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 296 ]), &(acadoWorkspace.QE[ 352 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 368 ]), &(acadoWorkspace.QE[ 424 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 448 ]), &(acadoWorkspace.QE[ 504 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 536 ]), &(acadoWorkspace.QE[ 592 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 632 ]), &(acadoWorkspace.QE[ 688 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 736 ]), &(acadoWorkspace.QE[ 792 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 848 ]), &(acadoWorkspace.QE[ 904 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 968 ]), &(acadoWorkspace.QE[ 1024 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 1096 ]), &(acadoWorkspace.QE[ 1152 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 1232 ]), &(acadoWorkspace.QE[ 1288 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 1376 ]), &(acadoWorkspace.QE[ 1432 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 1528 ]), &(acadoWorkspace.QE[ 1584 ]) );

acado_zeroBlockH11( 1, 9 );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 368 ]), &(acadoWorkspace.QE[ 432 ]) );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 448 ]), &(acadoWorkspace.QE[ 512 ]) );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 536 ]), &(acadoWorkspace.QE[ 600 ]) );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 632 ]), &(acadoWorkspace.QE[ 696 ]) );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 736 ]), &(acadoWorkspace.QE[ 800 ]) );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 848 ]), &(acadoWorkspace.QE[ 912 ]) );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 968 ]), &(acadoWorkspace.QE[ 1032 ]) );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 1096 ]), &(acadoWorkspace.QE[ 1160 ]) );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 1232 ]), &(acadoWorkspace.QE[ 1296 ]) );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 1376 ]), &(acadoWorkspace.QE[ 1440 ]) );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 1528 ]), &(acadoWorkspace.QE[ 1592 ]) );

acado_zeroBlockH11( 1, 10 );
acado_setBlockH11( 1, 10, &(acadoWorkspace.E[ 448 ]), &(acadoWorkspace.QE[ 520 ]) );
acado_setBlockH11( 1, 10, &(acadoWorkspace.E[ 536 ]), &(acadoWorkspace.QE[ 608 ]) );
acado_setBlockH11( 1, 10, &(acadoWorkspace.E[ 632 ]), &(acadoWorkspace.QE[ 704 ]) );
acado_setBlockH11( 1, 10, &(acadoWorkspace.E[ 736 ]), &(acadoWorkspace.QE[ 808 ]) );
acado_setBlockH11( 1, 10, &(acadoWorkspace.E[ 848 ]), &(acadoWorkspace.QE[ 920 ]) );
acado_setBlockH11( 1, 10, &(acadoWorkspace.E[ 968 ]), &(acadoWorkspace.QE[ 1040 ]) );
acado_setBlockH11( 1, 10, &(acadoWorkspace.E[ 1096 ]), &(acadoWorkspace.QE[ 1168 ]) );
acado_setBlockH11( 1, 10, &(acadoWorkspace.E[ 1232 ]), &(acadoWorkspace.QE[ 1304 ]) );
acado_setBlockH11( 1, 10, &(acadoWorkspace.E[ 1376 ]), &(acadoWorkspace.QE[ 1448 ]) );
acado_setBlockH11( 1, 10, &(acadoWorkspace.E[ 1528 ]), &(acadoWorkspace.QE[ 1600 ]) );

acado_zeroBlockH11( 1, 11 );
acado_setBlockH11( 1, 11, &(acadoWorkspace.E[ 536 ]), &(acadoWorkspace.QE[ 616 ]) );
acado_setBlockH11( 1, 11, &(acadoWorkspace.E[ 632 ]), &(acadoWorkspace.QE[ 712 ]) );
acado_setBlockH11( 1, 11, &(acadoWorkspace.E[ 736 ]), &(acadoWorkspace.QE[ 816 ]) );
acado_setBlockH11( 1, 11, &(acadoWorkspace.E[ 848 ]), &(acadoWorkspace.QE[ 928 ]) );
acado_setBlockH11( 1, 11, &(acadoWorkspace.E[ 968 ]), &(acadoWorkspace.QE[ 1048 ]) );
acado_setBlockH11( 1, 11, &(acadoWorkspace.E[ 1096 ]), &(acadoWorkspace.QE[ 1176 ]) );
acado_setBlockH11( 1, 11, &(acadoWorkspace.E[ 1232 ]), &(acadoWorkspace.QE[ 1312 ]) );
acado_setBlockH11( 1, 11, &(acadoWorkspace.E[ 1376 ]), &(acadoWorkspace.QE[ 1456 ]) );
acado_setBlockH11( 1, 11, &(acadoWorkspace.E[ 1528 ]), &(acadoWorkspace.QE[ 1608 ]) );

acado_zeroBlockH11( 1, 12 );
acado_setBlockH11( 1, 12, &(acadoWorkspace.E[ 632 ]), &(acadoWorkspace.QE[ 720 ]) );
acado_setBlockH11( 1, 12, &(acadoWorkspace.E[ 736 ]), &(acadoWorkspace.QE[ 824 ]) );
acado_setBlockH11( 1, 12, &(acadoWorkspace.E[ 848 ]), &(acadoWorkspace.QE[ 936 ]) );
acado_setBlockH11( 1, 12, &(acadoWorkspace.E[ 968 ]), &(acadoWorkspace.QE[ 1056 ]) );
acado_setBlockH11( 1, 12, &(acadoWorkspace.E[ 1096 ]), &(acadoWorkspace.QE[ 1184 ]) );
acado_setBlockH11( 1, 12, &(acadoWorkspace.E[ 1232 ]), &(acadoWorkspace.QE[ 1320 ]) );
acado_setBlockH11( 1, 12, &(acadoWorkspace.E[ 1376 ]), &(acadoWorkspace.QE[ 1464 ]) );
acado_setBlockH11( 1, 12, &(acadoWorkspace.E[ 1528 ]), &(acadoWorkspace.QE[ 1616 ]) );

acado_zeroBlockH11( 1, 13 );
acado_setBlockH11( 1, 13, &(acadoWorkspace.E[ 736 ]), &(acadoWorkspace.QE[ 832 ]) );
acado_setBlockH11( 1, 13, &(acadoWorkspace.E[ 848 ]), &(acadoWorkspace.QE[ 944 ]) );
acado_setBlockH11( 1, 13, &(acadoWorkspace.E[ 968 ]), &(acadoWorkspace.QE[ 1064 ]) );
acado_setBlockH11( 1, 13, &(acadoWorkspace.E[ 1096 ]), &(acadoWorkspace.QE[ 1192 ]) );
acado_setBlockH11( 1, 13, &(acadoWorkspace.E[ 1232 ]), &(acadoWorkspace.QE[ 1328 ]) );
acado_setBlockH11( 1, 13, &(acadoWorkspace.E[ 1376 ]), &(acadoWorkspace.QE[ 1472 ]) );
acado_setBlockH11( 1, 13, &(acadoWorkspace.E[ 1528 ]), &(acadoWorkspace.QE[ 1624 ]) );

acado_zeroBlockH11( 1, 14 );
acado_setBlockH11( 1, 14, &(acadoWorkspace.E[ 848 ]), &(acadoWorkspace.QE[ 952 ]) );
acado_setBlockH11( 1, 14, &(acadoWorkspace.E[ 968 ]), &(acadoWorkspace.QE[ 1072 ]) );
acado_setBlockH11( 1, 14, &(acadoWorkspace.E[ 1096 ]), &(acadoWorkspace.QE[ 1200 ]) );
acado_setBlockH11( 1, 14, &(acadoWorkspace.E[ 1232 ]), &(acadoWorkspace.QE[ 1336 ]) );
acado_setBlockH11( 1, 14, &(acadoWorkspace.E[ 1376 ]), &(acadoWorkspace.QE[ 1480 ]) );
acado_setBlockH11( 1, 14, &(acadoWorkspace.E[ 1528 ]), &(acadoWorkspace.QE[ 1632 ]) );

acado_zeroBlockH11( 1, 15 );
acado_setBlockH11( 1, 15, &(acadoWorkspace.E[ 968 ]), &(acadoWorkspace.QE[ 1080 ]) );
acado_setBlockH11( 1, 15, &(acadoWorkspace.E[ 1096 ]), &(acadoWorkspace.QE[ 1208 ]) );
acado_setBlockH11( 1, 15, &(acadoWorkspace.E[ 1232 ]), &(acadoWorkspace.QE[ 1344 ]) );
acado_setBlockH11( 1, 15, &(acadoWorkspace.E[ 1376 ]), &(acadoWorkspace.QE[ 1488 ]) );
acado_setBlockH11( 1, 15, &(acadoWorkspace.E[ 1528 ]), &(acadoWorkspace.QE[ 1640 ]) );

acado_zeroBlockH11( 1, 16 );
acado_setBlockH11( 1, 16, &(acadoWorkspace.E[ 1096 ]), &(acadoWorkspace.QE[ 1216 ]) );
acado_setBlockH11( 1, 16, &(acadoWorkspace.E[ 1232 ]), &(acadoWorkspace.QE[ 1352 ]) );
acado_setBlockH11( 1, 16, &(acadoWorkspace.E[ 1376 ]), &(acadoWorkspace.QE[ 1496 ]) );
acado_setBlockH11( 1, 16, &(acadoWorkspace.E[ 1528 ]), &(acadoWorkspace.QE[ 1648 ]) );

acado_zeroBlockH11( 1, 17 );
acado_setBlockH11( 1, 17, &(acadoWorkspace.E[ 1232 ]), &(acadoWorkspace.QE[ 1360 ]) );
acado_setBlockH11( 1, 17, &(acadoWorkspace.E[ 1376 ]), &(acadoWorkspace.QE[ 1504 ]) );
acado_setBlockH11( 1, 17, &(acadoWorkspace.E[ 1528 ]), &(acadoWorkspace.QE[ 1656 ]) );

acado_zeroBlockH11( 1, 18 );
acado_setBlockH11( 1, 18, &(acadoWorkspace.E[ 1376 ]), &(acadoWorkspace.QE[ 1512 ]) );
acado_setBlockH11( 1, 18, &(acadoWorkspace.E[ 1528 ]), &(acadoWorkspace.QE[ 1664 ]) );

acado_zeroBlockH11( 1, 19 );
acado_setBlockH11( 1, 19, &(acadoWorkspace.E[ 1528 ]), &(acadoWorkspace.QE[ 1672 ]) );

acado_setBlockH11_R1( 2, 2, &(acadoWorkspace.R1[ 8 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 40 ]), &(acadoWorkspace.QE[ 40 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 64 ]), &(acadoWorkspace.QE[ 64 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QE[ 96 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 136 ]), &(acadoWorkspace.QE[ 136 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 184 ]), &(acadoWorkspace.QE[ 184 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 240 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 304 ]), &(acadoWorkspace.QE[ 304 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 376 ]), &(acadoWorkspace.QE[ 376 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 456 ]), &(acadoWorkspace.QE[ 456 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 544 ]), &(acadoWorkspace.QE[ 544 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 640 ]), &(acadoWorkspace.QE[ 640 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 744 ]), &(acadoWorkspace.QE[ 744 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 856 ]), &(acadoWorkspace.QE[ 856 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 976 ]), &(acadoWorkspace.QE[ 976 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QE[ 1104 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 1240 ]), &(acadoWorkspace.QE[ 1240 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 1384 ]), &(acadoWorkspace.QE[ 1384 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.QE[ 1536 ]) );

acado_zeroBlockH11( 2, 3 );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 64 ]), &(acadoWorkspace.QE[ 72 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QE[ 104 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 136 ]), &(acadoWorkspace.QE[ 144 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 184 ]), &(acadoWorkspace.QE[ 192 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 248 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 304 ]), &(acadoWorkspace.QE[ 312 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 376 ]), &(acadoWorkspace.QE[ 384 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 456 ]), &(acadoWorkspace.QE[ 464 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 544 ]), &(acadoWorkspace.QE[ 552 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 640 ]), &(acadoWorkspace.QE[ 648 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 744 ]), &(acadoWorkspace.QE[ 752 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 856 ]), &(acadoWorkspace.QE[ 864 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 976 ]), &(acadoWorkspace.QE[ 984 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QE[ 1112 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 1240 ]), &(acadoWorkspace.QE[ 1248 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 1384 ]), &(acadoWorkspace.QE[ 1392 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.QE[ 1544 ]) );

acado_zeroBlockH11( 2, 4 );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QE[ 112 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 136 ]), &(acadoWorkspace.QE[ 152 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 184 ]), &(acadoWorkspace.QE[ 200 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 256 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 304 ]), &(acadoWorkspace.QE[ 320 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 376 ]), &(acadoWorkspace.QE[ 392 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 456 ]), &(acadoWorkspace.QE[ 472 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 544 ]), &(acadoWorkspace.QE[ 560 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 640 ]), &(acadoWorkspace.QE[ 656 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 744 ]), &(acadoWorkspace.QE[ 760 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 856 ]), &(acadoWorkspace.QE[ 872 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 976 ]), &(acadoWorkspace.QE[ 992 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QE[ 1120 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 1240 ]), &(acadoWorkspace.QE[ 1256 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 1384 ]), &(acadoWorkspace.QE[ 1400 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.QE[ 1552 ]) );

acado_zeroBlockH11( 2, 5 );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 136 ]), &(acadoWorkspace.QE[ 160 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 184 ]), &(acadoWorkspace.QE[ 208 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 264 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 304 ]), &(acadoWorkspace.QE[ 328 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 376 ]), &(acadoWorkspace.QE[ 400 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 456 ]), &(acadoWorkspace.QE[ 480 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 544 ]), &(acadoWorkspace.QE[ 568 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 640 ]), &(acadoWorkspace.QE[ 664 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 744 ]), &(acadoWorkspace.QE[ 768 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 856 ]), &(acadoWorkspace.QE[ 880 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 976 ]), &(acadoWorkspace.QE[ 1000 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QE[ 1128 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 1240 ]), &(acadoWorkspace.QE[ 1264 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 1384 ]), &(acadoWorkspace.QE[ 1408 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.QE[ 1560 ]) );

acado_zeroBlockH11( 2, 6 );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 184 ]), &(acadoWorkspace.QE[ 216 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 272 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 304 ]), &(acadoWorkspace.QE[ 336 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 376 ]), &(acadoWorkspace.QE[ 408 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 456 ]), &(acadoWorkspace.QE[ 488 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 544 ]), &(acadoWorkspace.QE[ 576 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 640 ]), &(acadoWorkspace.QE[ 672 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 744 ]), &(acadoWorkspace.QE[ 776 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 856 ]), &(acadoWorkspace.QE[ 888 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 976 ]), &(acadoWorkspace.QE[ 1008 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QE[ 1136 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 1240 ]), &(acadoWorkspace.QE[ 1272 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 1384 ]), &(acadoWorkspace.QE[ 1416 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.QE[ 1568 ]) );

acado_zeroBlockH11( 2, 7 );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 280 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 304 ]), &(acadoWorkspace.QE[ 344 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 376 ]), &(acadoWorkspace.QE[ 416 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 456 ]), &(acadoWorkspace.QE[ 496 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 544 ]), &(acadoWorkspace.QE[ 584 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 640 ]), &(acadoWorkspace.QE[ 680 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 744 ]), &(acadoWorkspace.QE[ 784 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 856 ]), &(acadoWorkspace.QE[ 896 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 976 ]), &(acadoWorkspace.QE[ 1016 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QE[ 1144 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 1240 ]), &(acadoWorkspace.QE[ 1280 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 1384 ]), &(acadoWorkspace.QE[ 1424 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.QE[ 1576 ]) );

acado_zeroBlockH11( 2, 8 );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 304 ]), &(acadoWorkspace.QE[ 352 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 376 ]), &(acadoWorkspace.QE[ 424 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 456 ]), &(acadoWorkspace.QE[ 504 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 544 ]), &(acadoWorkspace.QE[ 592 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 640 ]), &(acadoWorkspace.QE[ 688 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 744 ]), &(acadoWorkspace.QE[ 792 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 856 ]), &(acadoWorkspace.QE[ 904 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 976 ]), &(acadoWorkspace.QE[ 1024 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QE[ 1152 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 1240 ]), &(acadoWorkspace.QE[ 1288 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 1384 ]), &(acadoWorkspace.QE[ 1432 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.QE[ 1584 ]) );

acado_zeroBlockH11( 2, 9 );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 376 ]), &(acadoWorkspace.QE[ 432 ]) );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 456 ]), &(acadoWorkspace.QE[ 512 ]) );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 544 ]), &(acadoWorkspace.QE[ 600 ]) );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 640 ]), &(acadoWorkspace.QE[ 696 ]) );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 744 ]), &(acadoWorkspace.QE[ 800 ]) );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 856 ]), &(acadoWorkspace.QE[ 912 ]) );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 976 ]), &(acadoWorkspace.QE[ 1032 ]) );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QE[ 1160 ]) );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 1240 ]), &(acadoWorkspace.QE[ 1296 ]) );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 1384 ]), &(acadoWorkspace.QE[ 1440 ]) );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.QE[ 1592 ]) );

acado_zeroBlockH11( 2, 10 );
acado_setBlockH11( 2, 10, &(acadoWorkspace.E[ 456 ]), &(acadoWorkspace.QE[ 520 ]) );
acado_setBlockH11( 2, 10, &(acadoWorkspace.E[ 544 ]), &(acadoWorkspace.QE[ 608 ]) );
acado_setBlockH11( 2, 10, &(acadoWorkspace.E[ 640 ]), &(acadoWorkspace.QE[ 704 ]) );
acado_setBlockH11( 2, 10, &(acadoWorkspace.E[ 744 ]), &(acadoWorkspace.QE[ 808 ]) );
acado_setBlockH11( 2, 10, &(acadoWorkspace.E[ 856 ]), &(acadoWorkspace.QE[ 920 ]) );
acado_setBlockH11( 2, 10, &(acadoWorkspace.E[ 976 ]), &(acadoWorkspace.QE[ 1040 ]) );
acado_setBlockH11( 2, 10, &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QE[ 1168 ]) );
acado_setBlockH11( 2, 10, &(acadoWorkspace.E[ 1240 ]), &(acadoWorkspace.QE[ 1304 ]) );
acado_setBlockH11( 2, 10, &(acadoWorkspace.E[ 1384 ]), &(acadoWorkspace.QE[ 1448 ]) );
acado_setBlockH11( 2, 10, &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.QE[ 1600 ]) );

acado_zeroBlockH11( 2, 11 );
acado_setBlockH11( 2, 11, &(acadoWorkspace.E[ 544 ]), &(acadoWorkspace.QE[ 616 ]) );
acado_setBlockH11( 2, 11, &(acadoWorkspace.E[ 640 ]), &(acadoWorkspace.QE[ 712 ]) );
acado_setBlockH11( 2, 11, &(acadoWorkspace.E[ 744 ]), &(acadoWorkspace.QE[ 816 ]) );
acado_setBlockH11( 2, 11, &(acadoWorkspace.E[ 856 ]), &(acadoWorkspace.QE[ 928 ]) );
acado_setBlockH11( 2, 11, &(acadoWorkspace.E[ 976 ]), &(acadoWorkspace.QE[ 1048 ]) );
acado_setBlockH11( 2, 11, &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QE[ 1176 ]) );
acado_setBlockH11( 2, 11, &(acadoWorkspace.E[ 1240 ]), &(acadoWorkspace.QE[ 1312 ]) );
acado_setBlockH11( 2, 11, &(acadoWorkspace.E[ 1384 ]), &(acadoWorkspace.QE[ 1456 ]) );
acado_setBlockH11( 2, 11, &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.QE[ 1608 ]) );

acado_zeroBlockH11( 2, 12 );
acado_setBlockH11( 2, 12, &(acadoWorkspace.E[ 640 ]), &(acadoWorkspace.QE[ 720 ]) );
acado_setBlockH11( 2, 12, &(acadoWorkspace.E[ 744 ]), &(acadoWorkspace.QE[ 824 ]) );
acado_setBlockH11( 2, 12, &(acadoWorkspace.E[ 856 ]), &(acadoWorkspace.QE[ 936 ]) );
acado_setBlockH11( 2, 12, &(acadoWorkspace.E[ 976 ]), &(acadoWorkspace.QE[ 1056 ]) );
acado_setBlockH11( 2, 12, &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QE[ 1184 ]) );
acado_setBlockH11( 2, 12, &(acadoWorkspace.E[ 1240 ]), &(acadoWorkspace.QE[ 1320 ]) );
acado_setBlockH11( 2, 12, &(acadoWorkspace.E[ 1384 ]), &(acadoWorkspace.QE[ 1464 ]) );
acado_setBlockH11( 2, 12, &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.QE[ 1616 ]) );

acado_zeroBlockH11( 2, 13 );
acado_setBlockH11( 2, 13, &(acadoWorkspace.E[ 744 ]), &(acadoWorkspace.QE[ 832 ]) );
acado_setBlockH11( 2, 13, &(acadoWorkspace.E[ 856 ]), &(acadoWorkspace.QE[ 944 ]) );
acado_setBlockH11( 2, 13, &(acadoWorkspace.E[ 976 ]), &(acadoWorkspace.QE[ 1064 ]) );
acado_setBlockH11( 2, 13, &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QE[ 1192 ]) );
acado_setBlockH11( 2, 13, &(acadoWorkspace.E[ 1240 ]), &(acadoWorkspace.QE[ 1328 ]) );
acado_setBlockH11( 2, 13, &(acadoWorkspace.E[ 1384 ]), &(acadoWorkspace.QE[ 1472 ]) );
acado_setBlockH11( 2, 13, &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.QE[ 1624 ]) );

acado_zeroBlockH11( 2, 14 );
acado_setBlockH11( 2, 14, &(acadoWorkspace.E[ 856 ]), &(acadoWorkspace.QE[ 952 ]) );
acado_setBlockH11( 2, 14, &(acadoWorkspace.E[ 976 ]), &(acadoWorkspace.QE[ 1072 ]) );
acado_setBlockH11( 2, 14, &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QE[ 1200 ]) );
acado_setBlockH11( 2, 14, &(acadoWorkspace.E[ 1240 ]), &(acadoWorkspace.QE[ 1336 ]) );
acado_setBlockH11( 2, 14, &(acadoWorkspace.E[ 1384 ]), &(acadoWorkspace.QE[ 1480 ]) );
acado_setBlockH11( 2, 14, &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.QE[ 1632 ]) );

acado_zeroBlockH11( 2, 15 );
acado_setBlockH11( 2, 15, &(acadoWorkspace.E[ 976 ]), &(acadoWorkspace.QE[ 1080 ]) );
acado_setBlockH11( 2, 15, &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QE[ 1208 ]) );
acado_setBlockH11( 2, 15, &(acadoWorkspace.E[ 1240 ]), &(acadoWorkspace.QE[ 1344 ]) );
acado_setBlockH11( 2, 15, &(acadoWorkspace.E[ 1384 ]), &(acadoWorkspace.QE[ 1488 ]) );
acado_setBlockH11( 2, 15, &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.QE[ 1640 ]) );

acado_zeroBlockH11( 2, 16 );
acado_setBlockH11( 2, 16, &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QE[ 1216 ]) );
acado_setBlockH11( 2, 16, &(acadoWorkspace.E[ 1240 ]), &(acadoWorkspace.QE[ 1352 ]) );
acado_setBlockH11( 2, 16, &(acadoWorkspace.E[ 1384 ]), &(acadoWorkspace.QE[ 1496 ]) );
acado_setBlockH11( 2, 16, &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.QE[ 1648 ]) );

acado_zeroBlockH11( 2, 17 );
acado_setBlockH11( 2, 17, &(acadoWorkspace.E[ 1240 ]), &(acadoWorkspace.QE[ 1360 ]) );
acado_setBlockH11( 2, 17, &(acadoWorkspace.E[ 1384 ]), &(acadoWorkspace.QE[ 1504 ]) );
acado_setBlockH11( 2, 17, &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.QE[ 1656 ]) );

acado_zeroBlockH11( 2, 18 );
acado_setBlockH11( 2, 18, &(acadoWorkspace.E[ 1384 ]), &(acadoWorkspace.QE[ 1512 ]) );
acado_setBlockH11( 2, 18, &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.QE[ 1664 ]) );

acado_zeroBlockH11( 2, 19 );
acado_setBlockH11( 2, 19, &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.QE[ 1672 ]) );

acado_setBlockH11_R1( 3, 3, &(acadoWorkspace.R1[ 12 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.QE[ 72 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 104 ]), &(acadoWorkspace.QE[ 104 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QE[ 144 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QE[ 192 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 248 ]), &(acadoWorkspace.QE[ 248 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 312 ]), &(acadoWorkspace.QE[ 312 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.QE[ 384 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 464 ]), &(acadoWorkspace.QE[ 464 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.QE[ 552 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 648 ]), &(acadoWorkspace.QE[ 648 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 752 ]), &(acadoWorkspace.QE[ 752 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.QE[ 864 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 984 ]), &(acadoWorkspace.QE[ 984 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 1112 ]), &(acadoWorkspace.QE[ 1112 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 1248 ]), &(acadoWorkspace.QE[ 1248 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 1392 ]), &(acadoWorkspace.QE[ 1392 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 1544 ]), &(acadoWorkspace.QE[ 1544 ]) );

acado_zeroBlockH11( 3, 4 );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 104 ]), &(acadoWorkspace.QE[ 112 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QE[ 152 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QE[ 200 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 248 ]), &(acadoWorkspace.QE[ 256 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 312 ]), &(acadoWorkspace.QE[ 320 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.QE[ 392 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 464 ]), &(acadoWorkspace.QE[ 472 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.QE[ 560 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 648 ]), &(acadoWorkspace.QE[ 656 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 752 ]), &(acadoWorkspace.QE[ 760 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.QE[ 872 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 984 ]), &(acadoWorkspace.QE[ 992 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 1112 ]), &(acadoWorkspace.QE[ 1120 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 1248 ]), &(acadoWorkspace.QE[ 1256 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 1392 ]), &(acadoWorkspace.QE[ 1400 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 1544 ]), &(acadoWorkspace.QE[ 1552 ]) );

acado_zeroBlockH11( 3, 5 );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QE[ 160 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QE[ 208 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 248 ]), &(acadoWorkspace.QE[ 264 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 312 ]), &(acadoWorkspace.QE[ 328 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.QE[ 400 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 464 ]), &(acadoWorkspace.QE[ 480 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.QE[ 568 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 648 ]), &(acadoWorkspace.QE[ 664 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 752 ]), &(acadoWorkspace.QE[ 768 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.QE[ 880 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 984 ]), &(acadoWorkspace.QE[ 1000 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 1112 ]), &(acadoWorkspace.QE[ 1128 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 1248 ]), &(acadoWorkspace.QE[ 1264 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 1392 ]), &(acadoWorkspace.QE[ 1408 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 1544 ]), &(acadoWorkspace.QE[ 1560 ]) );

acado_zeroBlockH11( 3, 6 );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QE[ 216 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 248 ]), &(acadoWorkspace.QE[ 272 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 312 ]), &(acadoWorkspace.QE[ 336 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.QE[ 408 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 464 ]), &(acadoWorkspace.QE[ 488 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.QE[ 576 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 648 ]), &(acadoWorkspace.QE[ 672 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 752 ]), &(acadoWorkspace.QE[ 776 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.QE[ 888 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 984 ]), &(acadoWorkspace.QE[ 1008 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 1112 ]), &(acadoWorkspace.QE[ 1136 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 1248 ]), &(acadoWorkspace.QE[ 1272 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 1392 ]), &(acadoWorkspace.QE[ 1416 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 1544 ]), &(acadoWorkspace.QE[ 1568 ]) );

acado_zeroBlockH11( 3, 7 );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 248 ]), &(acadoWorkspace.QE[ 280 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 312 ]), &(acadoWorkspace.QE[ 344 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.QE[ 416 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 464 ]), &(acadoWorkspace.QE[ 496 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.QE[ 584 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 648 ]), &(acadoWorkspace.QE[ 680 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 752 ]), &(acadoWorkspace.QE[ 784 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.QE[ 896 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 984 ]), &(acadoWorkspace.QE[ 1016 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 1112 ]), &(acadoWorkspace.QE[ 1144 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 1248 ]), &(acadoWorkspace.QE[ 1280 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 1392 ]), &(acadoWorkspace.QE[ 1424 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 1544 ]), &(acadoWorkspace.QE[ 1576 ]) );

acado_zeroBlockH11( 3, 8 );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 312 ]), &(acadoWorkspace.QE[ 352 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.QE[ 424 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 464 ]), &(acadoWorkspace.QE[ 504 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.QE[ 592 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 648 ]), &(acadoWorkspace.QE[ 688 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 752 ]), &(acadoWorkspace.QE[ 792 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.QE[ 904 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 984 ]), &(acadoWorkspace.QE[ 1024 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 1112 ]), &(acadoWorkspace.QE[ 1152 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 1248 ]), &(acadoWorkspace.QE[ 1288 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 1392 ]), &(acadoWorkspace.QE[ 1432 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 1544 ]), &(acadoWorkspace.QE[ 1584 ]) );

acado_zeroBlockH11( 3, 9 );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.QE[ 432 ]) );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 464 ]), &(acadoWorkspace.QE[ 512 ]) );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.QE[ 600 ]) );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 648 ]), &(acadoWorkspace.QE[ 696 ]) );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 752 ]), &(acadoWorkspace.QE[ 800 ]) );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.QE[ 912 ]) );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 984 ]), &(acadoWorkspace.QE[ 1032 ]) );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 1112 ]), &(acadoWorkspace.QE[ 1160 ]) );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 1248 ]), &(acadoWorkspace.QE[ 1296 ]) );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 1392 ]), &(acadoWorkspace.QE[ 1440 ]) );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 1544 ]), &(acadoWorkspace.QE[ 1592 ]) );

acado_zeroBlockH11( 3, 10 );
acado_setBlockH11( 3, 10, &(acadoWorkspace.E[ 464 ]), &(acadoWorkspace.QE[ 520 ]) );
acado_setBlockH11( 3, 10, &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.QE[ 608 ]) );
acado_setBlockH11( 3, 10, &(acadoWorkspace.E[ 648 ]), &(acadoWorkspace.QE[ 704 ]) );
acado_setBlockH11( 3, 10, &(acadoWorkspace.E[ 752 ]), &(acadoWorkspace.QE[ 808 ]) );
acado_setBlockH11( 3, 10, &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.QE[ 920 ]) );
acado_setBlockH11( 3, 10, &(acadoWorkspace.E[ 984 ]), &(acadoWorkspace.QE[ 1040 ]) );
acado_setBlockH11( 3, 10, &(acadoWorkspace.E[ 1112 ]), &(acadoWorkspace.QE[ 1168 ]) );
acado_setBlockH11( 3, 10, &(acadoWorkspace.E[ 1248 ]), &(acadoWorkspace.QE[ 1304 ]) );
acado_setBlockH11( 3, 10, &(acadoWorkspace.E[ 1392 ]), &(acadoWorkspace.QE[ 1448 ]) );
acado_setBlockH11( 3, 10, &(acadoWorkspace.E[ 1544 ]), &(acadoWorkspace.QE[ 1600 ]) );

acado_zeroBlockH11( 3, 11 );
acado_setBlockH11( 3, 11, &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.QE[ 616 ]) );
acado_setBlockH11( 3, 11, &(acadoWorkspace.E[ 648 ]), &(acadoWorkspace.QE[ 712 ]) );
acado_setBlockH11( 3, 11, &(acadoWorkspace.E[ 752 ]), &(acadoWorkspace.QE[ 816 ]) );
acado_setBlockH11( 3, 11, &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.QE[ 928 ]) );
acado_setBlockH11( 3, 11, &(acadoWorkspace.E[ 984 ]), &(acadoWorkspace.QE[ 1048 ]) );
acado_setBlockH11( 3, 11, &(acadoWorkspace.E[ 1112 ]), &(acadoWorkspace.QE[ 1176 ]) );
acado_setBlockH11( 3, 11, &(acadoWorkspace.E[ 1248 ]), &(acadoWorkspace.QE[ 1312 ]) );
acado_setBlockH11( 3, 11, &(acadoWorkspace.E[ 1392 ]), &(acadoWorkspace.QE[ 1456 ]) );
acado_setBlockH11( 3, 11, &(acadoWorkspace.E[ 1544 ]), &(acadoWorkspace.QE[ 1608 ]) );

acado_zeroBlockH11( 3, 12 );
acado_setBlockH11( 3, 12, &(acadoWorkspace.E[ 648 ]), &(acadoWorkspace.QE[ 720 ]) );
acado_setBlockH11( 3, 12, &(acadoWorkspace.E[ 752 ]), &(acadoWorkspace.QE[ 824 ]) );
acado_setBlockH11( 3, 12, &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.QE[ 936 ]) );
acado_setBlockH11( 3, 12, &(acadoWorkspace.E[ 984 ]), &(acadoWorkspace.QE[ 1056 ]) );
acado_setBlockH11( 3, 12, &(acadoWorkspace.E[ 1112 ]), &(acadoWorkspace.QE[ 1184 ]) );
acado_setBlockH11( 3, 12, &(acadoWorkspace.E[ 1248 ]), &(acadoWorkspace.QE[ 1320 ]) );
acado_setBlockH11( 3, 12, &(acadoWorkspace.E[ 1392 ]), &(acadoWorkspace.QE[ 1464 ]) );
acado_setBlockH11( 3, 12, &(acadoWorkspace.E[ 1544 ]), &(acadoWorkspace.QE[ 1616 ]) );

acado_zeroBlockH11( 3, 13 );
acado_setBlockH11( 3, 13, &(acadoWorkspace.E[ 752 ]), &(acadoWorkspace.QE[ 832 ]) );
acado_setBlockH11( 3, 13, &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.QE[ 944 ]) );
acado_setBlockH11( 3, 13, &(acadoWorkspace.E[ 984 ]), &(acadoWorkspace.QE[ 1064 ]) );
acado_setBlockH11( 3, 13, &(acadoWorkspace.E[ 1112 ]), &(acadoWorkspace.QE[ 1192 ]) );
acado_setBlockH11( 3, 13, &(acadoWorkspace.E[ 1248 ]), &(acadoWorkspace.QE[ 1328 ]) );
acado_setBlockH11( 3, 13, &(acadoWorkspace.E[ 1392 ]), &(acadoWorkspace.QE[ 1472 ]) );
acado_setBlockH11( 3, 13, &(acadoWorkspace.E[ 1544 ]), &(acadoWorkspace.QE[ 1624 ]) );

acado_zeroBlockH11( 3, 14 );
acado_setBlockH11( 3, 14, &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.QE[ 952 ]) );
acado_setBlockH11( 3, 14, &(acadoWorkspace.E[ 984 ]), &(acadoWorkspace.QE[ 1072 ]) );
acado_setBlockH11( 3, 14, &(acadoWorkspace.E[ 1112 ]), &(acadoWorkspace.QE[ 1200 ]) );
acado_setBlockH11( 3, 14, &(acadoWorkspace.E[ 1248 ]), &(acadoWorkspace.QE[ 1336 ]) );
acado_setBlockH11( 3, 14, &(acadoWorkspace.E[ 1392 ]), &(acadoWorkspace.QE[ 1480 ]) );
acado_setBlockH11( 3, 14, &(acadoWorkspace.E[ 1544 ]), &(acadoWorkspace.QE[ 1632 ]) );

acado_zeroBlockH11( 3, 15 );
acado_setBlockH11( 3, 15, &(acadoWorkspace.E[ 984 ]), &(acadoWorkspace.QE[ 1080 ]) );
acado_setBlockH11( 3, 15, &(acadoWorkspace.E[ 1112 ]), &(acadoWorkspace.QE[ 1208 ]) );
acado_setBlockH11( 3, 15, &(acadoWorkspace.E[ 1248 ]), &(acadoWorkspace.QE[ 1344 ]) );
acado_setBlockH11( 3, 15, &(acadoWorkspace.E[ 1392 ]), &(acadoWorkspace.QE[ 1488 ]) );
acado_setBlockH11( 3, 15, &(acadoWorkspace.E[ 1544 ]), &(acadoWorkspace.QE[ 1640 ]) );

acado_zeroBlockH11( 3, 16 );
acado_setBlockH11( 3, 16, &(acadoWorkspace.E[ 1112 ]), &(acadoWorkspace.QE[ 1216 ]) );
acado_setBlockH11( 3, 16, &(acadoWorkspace.E[ 1248 ]), &(acadoWorkspace.QE[ 1352 ]) );
acado_setBlockH11( 3, 16, &(acadoWorkspace.E[ 1392 ]), &(acadoWorkspace.QE[ 1496 ]) );
acado_setBlockH11( 3, 16, &(acadoWorkspace.E[ 1544 ]), &(acadoWorkspace.QE[ 1648 ]) );

acado_zeroBlockH11( 3, 17 );
acado_setBlockH11( 3, 17, &(acadoWorkspace.E[ 1248 ]), &(acadoWorkspace.QE[ 1360 ]) );
acado_setBlockH11( 3, 17, &(acadoWorkspace.E[ 1392 ]), &(acadoWorkspace.QE[ 1504 ]) );
acado_setBlockH11( 3, 17, &(acadoWorkspace.E[ 1544 ]), &(acadoWorkspace.QE[ 1656 ]) );

acado_zeroBlockH11( 3, 18 );
acado_setBlockH11( 3, 18, &(acadoWorkspace.E[ 1392 ]), &(acadoWorkspace.QE[ 1512 ]) );
acado_setBlockH11( 3, 18, &(acadoWorkspace.E[ 1544 ]), &(acadoWorkspace.QE[ 1664 ]) );

acado_zeroBlockH11( 3, 19 );
acado_setBlockH11( 3, 19, &(acadoWorkspace.E[ 1544 ]), &(acadoWorkspace.QE[ 1672 ]) );

acado_setBlockH11_R1( 4, 4, &(acadoWorkspace.R1[ 16 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 112 ]), &(acadoWorkspace.QE[ 112 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 152 ]), &(acadoWorkspace.QE[ 152 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 200 ]), &(acadoWorkspace.QE[ 200 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 256 ]), &(acadoWorkspace.QE[ 256 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 320 ]), &(acadoWorkspace.QE[ 320 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 392 ]), &(acadoWorkspace.QE[ 392 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 472 ]), &(acadoWorkspace.QE[ 472 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 560 ]), &(acadoWorkspace.QE[ 560 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 656 ]), &(acadoWorkspace.QE[ 656 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 760 ]), &(acadoWorkspace.QE[ 760 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 872 ]), &(acadoWorkspace.QE[ 872 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 992 ]), &(acadoWorkspace.QE[ 992 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 1120 ]), &(acadoWorkspace.QE[ 1120 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 1256 ]), &(acadoWorkspace.QE[ 1256 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 1400 ]), &(acadoWorkspace.QE[ 1400 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 1552 ]), &(acadoWorkspace.QE[ 1552 ]) );

acado_zeroBlockH11( 4, 5 );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 152 ]), &(acadoWorkspace.QE[ 160 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 200 ]), &(acadoWorkspace.QE[ 208 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 256 ]), &(acadoWorkspace.QE[ 264 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 320 ]), &(acadoWorkspace.QE[ 328 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 392 ]), &(acadoWorkspace.QE[ 400 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 472 ]), &(acadoWorkspace.QE[ 480 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 560 ]), &(acadoWorkspace.QE[ 568 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 656 ]), &(acadoWorkspace.QE[ 664 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 760 ]), &(acadoWorkspace.QE[ 768 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 872 ]), &(acadoWorkspace.QE[ 880 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 992 ]), &(acadoWorkspace.QE[ 1000 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 1120 ]), &(acadoWorkspace.QE[ 1128 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 1256 ]), &(acadoWorkspace.QE[ 1264 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 1400 ]), &(acadoWorkspace.QE[ 1408 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 1552 ]), &(acadoWorkspace.QE[ 1560 ]) );

acado_zeroBlockH11( 4, 6 );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 200 ]), &(acadoWorkspace.QE[ 216 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 256 ]), &(acadoWorkspace.QE[ 272 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 320 ]), &(acadoWorkspace.QE[ 336 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 392 ]), &(acadoWorkspace.QE[ 408 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 472 ]), &(acadoWorkspace.QE[ 488 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 560 ]), &(acadoWorkspace.QE[ 576 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 656 ]), &(acadoWorkspace.QE[ 672 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 760 ]), &(acadoWorkspace.QE[ 776 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 872 ]), &(acadoWorkspace.QE[ 888 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 992 ]), &(acadoWorkspace.QE[ 1008 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 1120 ]), &(acadoWorkspace.QE[ 1136 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 1256 ]), &(acadoWorkspace.QE[ 1272 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 1400 ]), &(acadoWorkspace.QE[ 1416 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 1552 ]), &(acadoWorkspace.QE[ 1568 ]) );

acado_zeroBlockH11( 4, 7 );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 256 ]), &(acadoWorkspace.QE[ 280 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 320 ]), &(acadoWorkspace.QE[ 344 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 392 ]), &(acadoWorkspace.QE[ 416 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 472 ]), &(acadoWorkspace.QE[ 496 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 560 ]), &(acadoWorkspace.QE[ 584 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 656 ]), &(acadoWorkspace.QE[ 680 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 760 ]), &(acadoWorkspace.QE[ 784 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 872 ]), &(acadoWorkspace.QE[ 896 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 992 ]), &(acadoWorkspace.QE[ 1016 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 1120 ]), &(acadoWorkspace.QE[ 1144 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 1256 ]), &(acadoWorkspace.QE[ 1280 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 1400 ]), &(acadoWorkspace.QE[ 1424 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 1552 ]), &(acadoWorkspace.QE[ 1576 ]) );

acado_zeroBlockH11( 4, 8 );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 320 ]), &(acadoWorkspace.QE[ 352 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 392 ]), &(acadoWorkspace.QE[ 424 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 472 ]), &(acadoWorkspace.QE[ 504 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 560 ]), &(acadoWorkspace.QE[ 592 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 656 ]), &(acadoWorkspace.QE[ 688 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 760 ]), &(acadoWorkspace.QE[ 792 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 872 ]), &(acadoWorkspace.QE[ 904 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 992 ]), &(acadoWorkspace.QE[ 1024 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 1120 ]), &(acadoWorkspace.QE[ 1152 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 1256 ]), &(acadoWorkspace.QE[ 1288 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 1400 ]), &(acadoWorkspace.QE[ 1432 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 1552 ]), &(acadoWorkspace.QE[ 1584 ]) );

acado_zeroBlockH11( 4, 9 );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 392 ]), &(acadoWorkspace.QE[ 432 ]) );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 472 ]), &(acadoWorkspace.QE[ 512 ]) );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 560 ]), &(acadoWorkspace.QE[ 600 ]) );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 656 ]), &(acadoWorkspace.QE[ 696 ]) );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 760 ]), &(acadoWorkspace.QE[ 800 ]) );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 872 ]), &(acadoWorkspace.QE[ 912 ]) );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 992 ]), &(acadoWorkspace.QE[ 1032 ]) );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 1120 ]), &(acadoWorkspace.QE[ 1160 ]) );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 1256 ]), &(acadoWorkspace.QE[ 1296 ]) );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 1400 ]), &(acadoWorkspace.QE[ 1440 ]) );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 1552 ]), &(acadoWorkspace.QE[ 1592 ]) );

acado_zeroBlockH11( 4, 10 );
acado_setBlockH11( 4, 10, &(acadoWorkspace.E[ 472 ]), &(acadoWorkspace.QE[ 520 ]) );
acado_setBlockH11( 4, 10, &(acadoWorkspace.E[ 560 ]), &(acadoWorkspace.QE[ 608 ]) );
acado_setBlockH11( 4, 10, &(acadoWorkspace.E[ 656 ]), &(acadoWorkspace.QE[ 704 ]) );
acado_setBlockH11( 4, 10, &(acadoWorkspace.E[ 760 ]), &(acadoWorkspace.QE[ 808 ]) );
acado_setBlockH11( 4, 10, &(acadoWorkspace.E[ 872 ]), &(acadoWorkspace.QE[ 920 ]) );
acado_setBlockH11( 4, 10, &(acadoWorkspace.E[ 992 ]), &(acadoWorkspace.QE[ 1040 ]) );
acado_setBlockH11( 4, 10, &(acadoWorkspace.E[ 1120 ]), &(acadoWorkspace.QE[ 1168 ]) );
acado_setBlockH11( 4, 10, &(acadoWorkspace.E[ 1256 ]), &(acadoWorkspace.QE[ 1304 ]) );
acado_setBlockH11( 4, 10, &(acadoWorkspace.E[ 1400 ]), &(acadoWorkspace.QE[ 1448 ]) );
acado_setBlockH11( 4, 10, &(acadoWorkspace.E[ 1552 ]), &(acadoWorkspace.QE[ 1600 ]) );

acado_zeroBlockH11( 4, 11 );
acado_setBlockH11( 4, 11, &(acadoWorkspace.E[ 560 ]), &(acadoWorkspace.QE[ 616 ]) );
acado_setBlockH11( 4, 11, &(acadoWorkspace.E[ 656 ]), &(acadoWorkspace.QE[ 712 ]) );
acado_setBlockH11( 4, 11, &(acadoWorkspace.E[ 760 ]), &(acadoWorkspace.QE[ 816 ]) );
acado_setBlockH11( 4, 11, &(acadoWorkspace.E[ 872 ]), &(acadoWorkspace.QE[ 928 ]) );
acado_setBlockH11( 4, 11, &(acadoWorkspace.E[ 992 ]), &(acadoWorkspace.QE[ 1048 ]) );
acado_setBlockH11( 4, 11, &(acadoWorkspace.E[ 1120 ]), &(acadoWorkspace.QE[ 1176 ]) );
acado_setBlockH11( 4, 11, &(acadoWorkspace.E[ 1256 ]), &(acadoWorkspace.QE[ 1312 ]) );
acado_setBlockH11( 4, 11, &(acadoWorkspace.E[ 1400 ]), &(acadoWorkspace.QE[ 1456 ]) );
acado_setBlockH11( 4, 11, &(acadoWorkspace.E[ 1552 ]), &(acadoWorkspace.QE[ 1608 ]) );

acado_zeroBlockH11( 4, 12 );
acado_setBlockH11( 4, 12, &(acadoWorkspace.E[ 656 ]), &(acadoWorkspace.QE[ 720 ]) );
acado_setBlockH11( 4, 12, &(acadoWorkspace.E[ 760 ]), &(acadoWorkspace.QE[ 824 ]) );
acado_setBlockH11( 4, 12, &(acadoWorkspace.E[ 872 ]), &(acadoWorkspace.QE[ 936 ]) );
acado_setBlockH11( 4, 12, &(acadoWorkspace.E[ 992 ]), &(acadoWorkspace.QE[ 1056 ]) );
acado_setBlockH11( 4, 12, &(acadoWorkspace.E[ 1120 ]), &(acadoWorkspace.QE[ 1184 ]) );
acado_setBlockH11( 4, 12, &(acadoWorkspace.E[ 1256 ]), &(acadoWorkspace.QE[ 1320 ]) );
acado_setBlockH11( 4, 12, &(acadoWorkspace.E[ 1400 ]), &(acadoWorkspace.QE[ 1464 ]) );
acado_setBlockH11( 4, 12, &(acadoWorkspace.E[ 1552 ]), &(acadoWorkspace.QE[ 1616 ]) );

acado_zeroBlockH11( 4, 13 );
acado_setBlockH11( 4, 13, &(acadoWorkspace.E[ 760 ]), &(acadoWorkspace.QE[ 832 ]) );
acado_setBlockH11( 4, 13, &(acadoWorkspace.E[ 872 ]), &(acadoWorkspace.QE[ 944 ]) );
acado_setBlockH11( 4, 13, &(acadoWorkspace.E[ 992 ]), &(acadoWorkspace.QE[ 1064 ]) );
acado_setBlockH11( 4, 13, &(acadoWorkspace.E[ 1120 ]), &(acadoWorkspace.QE[ 1192 ]) );
acado_setBlockH11( 4, 13, &(acadoWorkspace.E[ 1256 ]), &(acadoWorkspace.QE[ 1328 ]) );
acado_setBlockH11( 4, 13, &(acadoWorkspace.E[ 1400 ]), &(acadoWorkspace.QE[ 1472 ]) );
acado_setBlockH11( 4, 13, &(acadoWorkspace.E[ 1552 ]), &(acadoWorkspace.QE[ 1624 ]) );

acado_zeroBlockH11( 4, 14 );
acado_setBlockH11( 4, 14, &(acadoWorkspace.E[ 872 ]), &(acadoWorkspace.QE[ 952 ]) );
acado_setBlockH11( 4, 14, &(acadoWorkspace.E[ 992 ]), &(acadoWorkspace.QE[ 1072 ]) );
acado_setBlockH11( 4, 14, &(acadoWorkspace.E[ 1120 ]), &(acadoWorkspace.QE[ 1200 ]) );
acado_setBlockH11( 4, 14, &(acadoWorkspace.E[ 1256 ]), &(acadoWorkspace.QE[ 1336 ]) );
acado_setBlockH11( 4, 14, &(acadoWorkspace.E[ 1400 ]), &(acadoWorkspace.QE[ 1480 ]) );
acado_setBlockH11( 4, 14, &(acadoWorkspace.E[ 1552 ]), &(acadoWorkspace.QE[ 1632 ]) );

acado_zeroBlockH11( 4, 15 );
acado_setBlockH11( 4, 15, &(acadoWorkspace.E[ 992 ]), &(acadoWorkspace.QE[ 1080 ]) );
acado_setBlockH11( 4, 15, &(acadoWorkspace.E[ 1120 ]), &(acadoWorkspace.QE[ 1208 ]) );
acado_setBlockH11( 4, 15, &(acadoWorkspace.E[ 1256 ]), &(acadoWorkspace.QE[ 1344 ]) );
acado_setBlockH11( 4, 15, &(acadoWorkspace.E[ 1400 ]), &(acadoWorkspace.QE[ 1488 ]) );
acado_setBlockH11( 4, 15, &(acadoWorkspace.E[ 1552 ]), &(acadoWorkspace.QE[ 1640 ]) );

acado_zeroBlockH11( 4, 16 );
acado_setBlockH11( 4, 16, &(acadoWorkspace.E[ 1120 ]), &(acadoWorkspace.QE[ 1216 ]) );
acado_setBlockH11( 4, 16, &(acadoWorkspace.E[ 1256 ]), &(acadoWorkspace.QE[ 1352 ]) );
acado_setBlockH11( 4, 16, &(acadoWorkspace.E[ 1400 ]), &(acadoWorkspace.QE[ 1496 ]) );
acado_setBlockH11( 4, 16, &(acadoWorkspace.E[ 1552 ]), &(acadoWorkspace.QE[ 1648 ]) );

acado_zeroBlockH11( 4, 17 );
acado_setBlockH11( 4, 17, &(acadoWorkspace.E[ 1256 ]), &(acadoWorkspace.QE[ 1360 ]) );
acado_setBlockH11( 4, 17, &(acadoWorkspace.E[ 1400 ]), &(acadoWorkspace.QE[ 1504 ]) );
acado_setBlockH11( 4, 17, &(acadoWorkspace.E[ 1552 ]), &(acadoWorkspace.QE[ 1656 ]) );

acado_zeroBlockH11( 4, 18 );
acado_setBlockH11( 4, 18, &(acadoWorkspace.E[ 1400 ]), &(acadoWorkspace.QE[ 1512 ]) );
acado_setBlockH11( 4, 18, &(acadoWorkspace.E[ 1552 ]), &(acadoWorkspace.QE[ 1664 ]) );

acado_zeroBlockH11( 4, 19 );
acado_setBlockH11( 4, 19, &(acadoWorkspace.E[ 1552 ]), &(acadoWorkspace.QE[ 1672 ]) );

acado_setBlockH11_R1( 5, 5, &(acadoWorkspace.R1[ 20 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 160 ]), &(acadoWorkspace.QE[ 160 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 208 ]), &(acadoWorkspace.QE[ 208 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 264 ]), &(acadoWorkspace.QE[ 264 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 328 ]), &(acadoWorkspace.QE[ 328 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 400 ]), &(acadoWorkspace.QE[ 400 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 480 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 568 ]), &(acadoWorkspace.QE[ 568 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 664 ]), &(acadoWorkspace.QE[ 664 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 768 ]), &(acadoWorkspace.QE[ 768 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 880 ]), &(acadoWorkspace.QE[ 880 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 1000 ]), &(acadoWorkspace.QE[ 1000 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 1128 ]), &(acadoWorkspace.QE[ 1128 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 1264 ]), &(acadoWorkspace.QE[ 1264 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 1408 ]), &(acadoWorkspace.QE[ 1408 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 1560 ]), &(acadoWorkspace.QE[ 1560 ]) );

acado_zeroBlockH11( 5, 6 );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 208 ]), &(acadoWorkspace.QE[ 216 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 264 ]), &(acadoWorkspace.QE[ 272 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 328 ]), &(acadoWorkspace.QE[ 336 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 400 ]), &(acadoWorkspace.QE[ 408 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 488 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 568 ]), &(acadoWorkspace.QE[ 576 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 664 ]), &(acadoWorkspace.QE[ 672 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 768 ]), &(acadoWorkspace.QE[ 776 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 880 ]), &(acadoWorkspace.QE[ 888 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 1000 ]), &(acadoWorkspace.QE[ 1008 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 1128 ]), &(acadoWorkspace.QE[ 1136 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 1264 ]), &(acadoWorkspace.QE[ 1272 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 1408 ]), &(acadoWorkspace.QE[ 1416 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 1560 ]), &(acadoWorkspace.QE[ 1568 ]) );

acado_zeroBlockH11( 5, 7 );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 264 ]), &(acadoWorkspace.QE[ 280 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 328 ]), &(acadoWorkspace.QE[ 344 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 400 ]), &(acadoWorkspace.QE[ 416 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 496 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 568 ]), &(acadoWorkspace.QE[ 584 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 664 ]), &(acadoWorkspace.QE[ 680 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 768 ]), &(acadoWorkspace.QE[ 784 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 880 ]), &(acadoWorkspace.QE[ 896 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 1000 ]), &(acadoWorkspace.QE[ 1016 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 1128 ]), &(acadoWorkspace.QE[ 1144 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 1264 ]), &(acadoWorkspace.QE[ 1280 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 1408 ]), &(acadoWorkspace.QE[ 1424 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 1560 ]), &(acadoWorkspace.QE[ 1576 ]) );

acado_zeroBlockH11( 5, 8 );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 328 ]), &(acadoWorkspace.QE[ 352 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 400 ]), &(acadoWorkspace.QE[ 424 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 504 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 568 ]), &(acadoWorkspace.QE[ 592 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 664 ]), &(acadoWorkspace.QE[ 688 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 768 ]), &(acadoWorkspace.QE[ 792 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 880 ]), &(acadoWorkspace.QE[ 904 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 1000 ]), &(acadoWorkspace.QE[ 1024 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 1128 ]), &(acadoWorkspace.QE[ 1152 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 1264 ]), &(acadoWorkspace.QE[ 1288 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 1408 ]), &(acadoWorkspace.QE[ 1432 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 1560 ]), &(acadoWorkspace.QE[ 1584 ]) );

acado_zeroBlockH11( 5, 9 );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 400 ]), &(acadoWorkspace.QE[ 432 ]) );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 512 ]) );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 568 ]), &(acadoWorkspace.QE[ 600 ]) );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 664 ]), &(acadoWorkspace.QE[ 696 ]) );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 768 ]), &(acadoWorkspace.QE[ 800 ]) );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 880 ]), &(acadoWorkspace.QE[ 912 ]) );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 1000 ]), &(acadoWorkspace.QE[ 1032 ]) );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 1128 ]), &(acadoWorkspace.QE[ 1160 ]) );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 1264 ]), &(acadoWorkspace.QE[ 1296 ]) );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 1408 ]), &(acadoWorkspace.QE[ 1440 ]) );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 1560 ]), &(acadoWorkspace.QE[ 1592 ]) );

acado_zeroBlockH11( 5, 10 );
acado_setBlockH11( 5, 10, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 520 ]) );
acado_setBlockH11( 5, 10, &(acadoWorkspace.E[ 568 ]), &(acadoWorkspace.QE[ 608 ]) );
acado_setBlockH11( 5, 10, &(acadoWorkspace.E[ 664 ]), &(acadoWorkspace.QE[ 704 ]) );
acado_setBlockH11( 5, 10, &(acadoWorkspace.E[ 768 ]), &(acadoWorkspace.QE[ 808 ]) );
acado_setBlockH11( 5, 10, &(acadoWorkspace.E[ 880 ]), &(acadoWorkspace.QE[ 920 ]) );
acado_setBlockH11( 5, 10, &(acadoWorkspace.E[ 1000 ]), &(acadoWorkspace.QE[ 1040 ]) );
acado_setBlockH11( 5, 10, &(acadoWorkspace.E[ 1128 ]), &(acadoWorkspace.QE[ 1168 ]) );
acado_setBlockH11( 5, 10, &(acadoWorkspace.E[ 1264 ]), &(acadoWorkspace.QE[ 1304 ]) );
acado_setBlockH11( 5, 10, &(acadoWorkspace.E[ 1408 ]), &(acadoWorkspace.QE[ 1448 ]) );
acado_setBlockH11( 5, 10, &(acadoWorkspace.E[ 1560 ]), &(acadoWorkspace.QE[ 1600 ]) );

acado_zeroBlockH11( 5, 11 );
acado_setBlockH11( 5, 11, &(acadoWorkspace.E[ 568 ]), &(acadoWorkspace.QE[ 616 ]) );
acado_setBlockH11( 5, 11, &(acadoWorkspace.E[ 664 ]), &(acadoWorkspace.QE[ 712 ]) );
acado_setBlockH11( 5, 11, &(acadoWorkspace.E[ 768 ]), &(acadoWorkspace.QE[ 816 ]) );
acado_setBlockH11( 5, 11, &(acadoWorkspace.E[ 880 ]), &(acadoWorkspace.QE[ 928 ]) );
acado_setBlockH11( 5, 11, &(acadoWorkspace.E[ 1000 ]), &(acadoWorkspace.QE[ 1048 ]) );
acado_setBlockH11( 5, 11, &(acadoWorkspace.E[ 1128 ]), &(acadoWorkspace.QE[ 1176 ]) );
acado_setBlockH11( 5, 11, &(acadoWorkspace.E[ 1264 ]), &(acadoWorkspace.QE[ 1312 ]) );
acado_setBlockH11( 5, 11, &(acadoWorkspace.E[ 1408 ]), &(acadoWorkspace.QE[ 1456 ]) );
acado_setBlockH11( 5, 11, &(acadoWorkspace.E[ 1560 ]), &(acadoWorkspace.QE[ 1608 ]) );

acado_zeroBlockH11( 5, 12 );
acado_setBlockH11( 5, 12, &(acadoWorkspace.E[ 664 ]), &(acadoWorkspace.QE[ 720 ]) );
acado_setBlockH11( 5, 12, &(acadoWorkspace.E[ 768 ]), &(acadoWorkspace.QE[ 824 ]) );
acado_setBlockH11( 5, 12, &(acadoWorkspace.E[ 880 ]), &(acadoWorkspace.QE[ 936 ]) );
acado_setBlockH11( 5, 12, &(acadoWorkspace.E[ 1000 ]), &(acadoWorkspace.QE[ 1056 ]) );
acado_setBlockH11( 5, 12, &(acadoWorkspace.E[ 1128 ]), &(acadoWorkspace.QE[ 1184 ]) );
acado_setBlockH11( 5, 12, &(acadoWorkspace.E[ 1264 ]), &(acadoWorkspace.QE[ 1320 ]) );
acado_setBlockH11( 5, 12, &(acadoWorkspace.E[ 1408 ]), &(acadoWorkspace.QE[ 1464 ]) );
acado_setBlockH11( 5, 12, &(acadoWorkspace.E[ 1560 ]), &(acadoWorkspace.QE[ 1616 ]) );

acado_zeroBlockH11( 5, 13 );
acado_setBlockH11( 5, 13, &(acadoWorkspace.E[ 768 ]), &(acadoWorkspace.QE[ 832 ]) );
acado_setBlockH11( 5, 13, &(acadoWorkspace.E[ 880 ]), &(acadoWorkspace.QE[ 944 ]) );
acado_setBlockH11( 5, 13, &(acadoWorkspace.E[ 1000 ]), &(acadoWorkspace.QE[ 1064 ]) );
acado_setBlockH11( 5, 13, &(acadoWorkspace.E[ 1128 ]), &(acadoWorkspace.QE[ 1192 ]) );
acado_setBlockH11( 5, 13, &(acadoWorkspace.E[ 1264 ]), &(acadoWorkspace.QE[ 1328 ]) );
acado_setBlockH11( 5, 13, &(acadoWorkspace.E[ 1408 ]), &(acadoWorkspace.QE[ 1472 ]) );
acado_setBlockH11( 5, 13, &(acadoWorkspace.E[ 1560 ]), &(acadoWorkspace.QE[ 1624 ]) );

acado_zeroBlockH11( 5, 14 );
acado_setBlockH11( 5, 14, &(acadoWorkspace.E[ 880 ]), &(acadoWorkspace.QE[ 952 ]) );
acado_setBlockH11( 5, 14, &(acadoWorkspace.E[ 1000 ]), &(acadoWorkspace.QE[ 1072 ]) );
acado_setBlockH11( 5, 14, &(acadoWorkspace.E[ 1128 ]), &(acadoWorkspace.QE[ 1200 ]) );
acado_setBlockH11( 5, 14, &(acadoWorkspace.E[ 1264 ]), &(acadoWorkspace.QE[ 1336 ]) );
acado_setBlockH11( 5, 14, &(acadoWorkspace.E[ 1408 ]), &(acadoWorkspace.QE[ 1480 ]) );
acado_setBlockH11( 5, 14, &(acadoWorkspace.E[ 1560 ]), &(acadoWorkspace.QE[ 1632 ]) );

acado_zeroBlockH11( 5, 15 );
acado_setBlockH11( 5, 15, &(acadoWorkspace.E[ 1000 ]), &(acadoWorkspace.QE[ 1080 ]) );
acado_setBlockH11( 5, 15, &(acadoWorkspace.E[ 1128 ]), &(acadoWorkspace.QE[ 1208 ]) );
acado_setBlockH11( 5, 15, &(acadoWorkspace.E[ 1264 ]), &(acadoWorkspace.QE[ 1344 ]) );
acado_setBlockH11( 5, 15, &(acadoWorkspace.E[ 1408 ]), &(acadoWorkspace.QE[ 1488 ]) );
acado_setBlockH11( 5, 15, &(acadoWorkspace.E[ 1560 ]), &(acadoWorkspace.QE[ 1640 ]) );

acado_zeroBlockH11( 5, 16 );
acado_setBlockH11( 5, 16, &(acadoWorkspace.E[ 1128 ]), &(acadoWorkspace.QE[ 1216 ]) );
acado_setBlockH11( 5, 16, &(acadoWorkspace.E[ 1264 ]), &(acadoWorkspace.QE[ 1352 ]) );
acado_setBlockH11( 5, 16, &(acadoWorkspace.E[ 1408 ]), &(acadoWorkspace.QE[ 1496 ]) );
acado_setBlockH11( 5, 16, &(acadoWorkspace.E[ 1560 ]), &(acadoWorkspace.QE[ 1648 ]) );

acado_zeroBlockH11( 5, 17 );
acado_setBlockH11( 5, 17, &(acadoWorkspace.E[ 1264 ]), &(acadoWorkspace.QE[ 1360 ]) );
acado_setBlockH11( 5, 17, &(acadoWorkspace.E[ 1408 ]), &(acadoWorkspace.QE[ 1504 ]) );
acado_setBlockH11( 5, 17, &(acadoWorkspace.E[ 1560 ]), &(acadoWorkspace.QE[ 1656 ]) );

acado_zeroBlockH11( 5, 18 );
acado_setBlockH11( 5, 18, &(acadoWorkspace.E[ 1408 ]), &(acadoWorkspace.QE[ 1512 ]) );
acado_setBlockH11( 5, 18, &(acadoWorkspace.E[ 1560 ]), &(acadoWorkspace.QE[ 1664 ]) );

acado_zeroBlockH11( 5, 19 );
acado_setBlockH11( 5, 19, &(acadoWorkspace.E[ 1560 ]), &(acadoWorkspace.QE[ 1672 ]) );

acado_setBlockH11_R1( 6, 6, &(acadoWorkspace.R1[ 24 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.QE[ 216 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 272 ]), &(acadoWorkspace.QE[ 272 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QE[ 336 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 408 ]), &(acadoWorkspace.QE[ 408 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 488 ]), &(acadoWorkspace.QE[ 488 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 576 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.QE[ 672 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 776 ]), &(acadoWorkspace.QE[ 776 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 888 ]), &(acadoWorkspace.QE[ 888 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 1008 ]), &(acadoWorkspace.QE[ 1008 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 1136 ]), &(acadoWorkspace.QE[ 1136 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 1272 ]), &(acadoWorkspace.QE[ 1272 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 1416 ]), &(acadoWorkspace.QE[ 1416 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 1568 ]), &(acadoWorkspace.QE[ 1568 ]) );

acado_zeroBlockH11( 6, 7 );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 272 ]), &(acadoWorkspace.QE[ 280 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QE[ 344 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 408 ]), &(acadoWorkspace.QE[ 416 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 488 ]), &(acadoWorkspace.QE[ 496 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 584 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.QE[ 680 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 776 ]), &(acadoWorkspace.QE[ 784 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 888 ]), &(acadoWorkspace.QE[ 896 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 1008 ]), &(acadoWorkspace.QE[ 1016 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 1136 ]), &(acadoWorkspace.QE[ 1144 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 1272 ]), &(acadoWorkspace.QE[ 1280 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 1416 ]), &(acadoWorkspace.QE[ 1424 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 1568 ]), &(acadoWorkspace.QE[ 1576 ]) );

acado_zeroBlockH11( 6, 8 );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QE[ 352 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 408 ]), &(acadoWorkspace.QE[ 424 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 488 ]), &(acadoWorkspace.QE[ 504 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 592 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.QE[ 688 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 776 ]), &(acadoWorkspace.QE[ 792 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 888 ]), &(acadoWorkspace.QE[ 904 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 1008 ]), &(acadoWorkspace.QE[ 1024 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 1136 ]), &(acadoWorkspace.QE[ 1152 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 1272 ]), &(acadoWorkspace.QE[ 1288 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 1416 ]), &(acadoWorkspace.QE[ 1432 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 1568 ]), &(acadoWorkspace.QE[ 1584 ]) );

acado_zeroBlockH11( 6, 9 );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 408 ]), &(acadoWorkspace.QE[ 432 ]) );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 488 ]), &(acadoWorkspace.QE[ 512 ]) );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 600 ]) );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.QE[ 696 ]) );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 776 ]), &(acadoWorkspace.QE[ 800 ]) );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 888 ]), &(acadoWorkspace.QE[ 912 ]) );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 1008 ]), &(acadoWorkspace.QE[ 1032 ]) );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 1136 ]), &(acadoWorkspace.QE[ 1160 ]) );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 1272 ]), &(acadoWorkspace.QE[ 1296 ]) );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 1416 ]), &(acadoWorkspace.QE[ 1440 ]) );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 1568 ]), &(acadoWorkspace.QE[ 1592 ]) );

acado_zeroBlockH11( 6, 10 );
acado_setBlockH11( 6, 10, &(acadoWorkspace.E[ 488 ]), &(acadoWorkspace.QE[ 520 ]) );
acado_setBlockH11( 6, 10, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 608 ]) );
acado_setBlockH11( 6, 10, &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.QE[ 704 ]) );
acado_setBlockH11( 6, 10, &(acadoWorkspace.E[ 776 ]), &(acadoWorkspace.QE[ 808 ]) );
acado_setBlockH11( 6, 10, &(acadoWorkspace.E[ 888 ]), &(acadoWorkspace.QE[ 920 ]) );
acado_setBlockH11( 6, 10, &(acadoWorkspace.E[ 1008 ]), &(acadoWorkspace.QE[ 1040 ]) );
acado_setBlockH11( 6, 10, &(acadoWorkspace.E[ 1136 ]), &(acadoWorkspace.QE[ 1168 ]) );
acado_setBlockH11( 6, 10, &(acadoWorkspace.E[ 1272 ]), &(acadoWorkspace.QE[ 1304 ]) );
acado_setBlockH11( 6, 10, &(acadoWorkspace.E[ 1416 ]), &(acadoWorkspace.QE[ 1448 ]) );
acado_setBlockH11( 6, 10, &(acadoWorkspace.E[ 1568 ]), &(acadoWorkspace.QE[ 1600 ]) );

acado_zeroBlockH11( 6, 11 );
acado_setBlockH11( 6, 11, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 616 ]) );
acado_setBlockH11( 6, 11, &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.QE[ 712 ]) );
acado_setBlockH11( 6, 11, &(acadoWorkspace.E[ 776 ]), &(acadoWorkspace.QE[ 816 ]) );
acado_setBlockH11( 6, 11, &(acadoWorkspace.E[ 888 ]), &(acadoWorkspace.QE[ 928 ]) );
acado_setBlockH11( 6, 11, &(acadoWorkspace.E[ 1008 ]), &(acadoWorkspace.QE[ 1048 ]) );
acado_setBlockH11( 6, 11, &(acadoWorkspace.E[ 1136 ]), &(acadoWorkspace.QE[ 1176 ]) );
acado_setBlockH11( 6, 11, &(acadoWorkspace.E[ 1272 ]), &(acadoWorkspace.QE[ 1312 ]) );
acado_setBlockH11( 6, 11, &(acadoWorkspace.E[ 1416 ]), &(acadoWorkspace.QE[ 1456 ]) );
acado_setBlockH11( 6, 11, &(acadoWorkspace.E[ 1568 ]), &(acadoWorkspace.QE[ 1608 ]) );

acado_zeroBlockH11( 6, 12 );
acado_setBlockH11( 6, 12, &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.QE[ 720 ]) );
acado_setBlockH11( 6, 12, &(acadoWorkspace.E[ 776 ]), &(acadoWorkspace.QE[ 824 ]) );
acado_setBlockH11( 6, 12, &(acadoWorkspace.E[ 888 ]), &(acadoWorkspace.QE[ 936 ]) );
acado_setBlockH11( 6, 12, &(acadoWorkspace.E[ 1008 ]), &(acadoWorkspace.QE[ 1056 ]) );
acado_setBlockH11( 6, 12, &(acadoWorkspace.E[ 1136 ]), &(acadoWorkspace.QE[ 1184 ]) );
acado_setBlockH11( 6, 12, &(acadoWorkspace.E[ 1272 ]), &(acadoWorkspace.QE[ 1320 ]) );
acado_setBlockH11( 6, 12, &(acadoWorkspace.E[ 1416 ]), &(acadoWorkspace.QE[ 1464 ]) );
acado_setBlockH11( 6, 12, &(acadoWorkspace.E[ 1568 ]), &(acadoWorkspace.QE[ 1616 ]) );

acado_zeroBlockH11( 6, 13 );
acado_setBlockH11( 6, 13, &(acadoWorkspace.E[ 776 ]), &(acadoWorkspace.QE[ 832 ]) );
acado_setBlockH11( 6, 13, &(acadoWorkspace.E[ 888 ]), &(acadoWorkspace.QE[ 944 ]) );
acado_setBlockH11( 6, 13, &(acadoWorkspace.E[ 1008 ]), &(acadoWorkspace.QE[ 1064 ]) );
acado_setBlockH11( 6, 13, &(acadoWorkspace.E[ 1136 ]), &(acadoWorkspace.QE[ 1192 ]) );
acado_setBlockH11( 6, 13, &(acadoWorkspace.E[ 1272 ]), &(acadoWorkspace.QE[ 1328 ]) );
acado_setBlockH11( 6, 13, &(acadoWorkspace.E[ 1416 ]), &(acadoWorkspace.QE[ 1472 ]) );
acado_setBlockH11( 6, 13, &(acadoWorkspace.E[ 1568 ]), &(acadoWorkspace.QE[ 1624 ]) );

acado_zeroBlockH11( 6, 14 );
acado_setBlockH11( 6, 14, &(acadoWorkspace.E[ 888 ]), &(acadoWorkspace.QE[ 952 ]) );
acado_setBlockH11( 6, 14, &(acadoWorkspace.E[ 1008 ]), &(acadoWorkspace.QE[ 1072 ]) );
acado_setBlockH11( 6, 14, &(acadoWorkspace.E[ 1136 ]), &(acadoWorkspace.QE[ 1200 ]) );
acado_setBlockH11( 6, 14, &(acadoWorkspace.E[ 1272 ]), &(acadoWorkspace.QE[ 1336 ]) );
acado_setBlockH11( 6, 14, &(acadoWorkspace.E[ 1416 ]), &(acadoWorkspace.QE[ 1480 ]) );
acado_setBlockH11( 6, 14, &(acadoWorkspace.E[ 1568 ]), &(acadoWorkspace.QE[ 1632 ]) );

acado_zeroBlockH11( 6, 15 );
acado_setBlockH11( 6, 15, &(acadoWorkspace.E[ 1008 ]), &(acadoWorkspace.QE[ 1080 ]) );
acado_setBlockH11( 6, 15, &(acadoWorkspace.E[ 1136 ]), &(acadoWorkspace.QE[ 1208 ]) );
acado_setBlockH11( 6, 15, &(acadoWorkspace.E[ 1272 ]), &(acadoWorkspace.QE[ 1344 ]) );
acado_setBlockH11( 6, 15, &(acadoWorkspace.E[ 1416 ]), &(acadoWorkspace.QE[ 1488 ]) );
acado_setBlockH11( 6, 15, &(acadoWorkspace.E[ 1568 ]), &(acadoWorkspace.QE[ 1640 ]) );

acado_zeroBlockH11( 6, 16 );
acado_setBlockH11( 6, 16, &(acadoWorkspace.E[ 1136 ]), &(acadoWorkspace.QE[ 1216 ]) );
acado_setBlockH11( 6, 16, &(acadoWorkspace.E[ 1272 ]), &(acadoWorkspace.QE[ 1352 ]) );
acado_setBlockH11( 6, 16, &(acadoWorkspace.E[ 1416 ]), &(acadoWorkspace.QE[ 1496 ]) );
acado_setBlockH11( 6, 16, &(acadoWorkspace.E[ 1568 ]), &(acadoWorkspace.QE[ 1648 ]) );

acado_zeroBlockH11( 6, 17 );
acado_setBlockH11( 6, 17, &(acadoWorkspace.E[ 1272 ]), &(acadoWorkspace.QE[ 1360 ]) );
acado_setBlockH11( 6, 17, &(acadoWorkspace.E[ 1416 ]), &(acadoWorkspace.QE[ 1504 ]) );
acado_setBlockH11( 6, 17, &(acadoWorkspace.E[ 1568 ]), &(acadoWorkspace.QE[ 1656 ]) );

acado_zeroBlockH11( 6, 18 );
acado_setBlockH11( 6, 18, &(acadoWorkspace.E[ 1416 ]), &(acadoWorkspace.QE[ 1512 ]) );
acado_setBlockH11( 6, 18, &(acadoWorkspace.E[ 1568 ]), &(acadoWorkspace.QE[ 1664 ]) );

acado_zeroBlockH11( 6, 19 );
acado_setBlockH11( 6, 19, &(acadoWorkspace.E[ 1568 ]), &(acadoWorkspace.QE[ 1672 ]) );

acado_setBlockH11_R1( 7, 7, &(acadoWorkspace.R1[ 28 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 280 ]), &(acadoWorkspace.QE[ 280 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 344 ]), &(acadoWorkspace.QE[ 344 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 416 ]), &(acadoWorkspace.QE[ 416 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 496 ]), &(acadoWorkspace.QE[ 496 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 584 ]), &(acadoWorkspace.QE[ 584 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 680 ]), &(acadoWorkspace.QE[ 680 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 784 ]), &(acadoWorkspace.QE[ 784 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 896 ]), &(acadoWorkspace.QE[ 896 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 1016 ]), &(acadoWorkspace.QE[ 1016 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 1144 ]), &(acadoWorkspace.QE[ 1144 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 1280 ]), &(acadoWorkspace.QE[ 1280 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 1424 ]), &(acadoWorkspace.QE[ 1424 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 1576 ]), &(acadoWorkspace.QE[ 1576 ]) );

acado_zeroBlockH11( 7, 8 );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 344 ]), &(acadoWorkspace.QE[ 352 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 416 ]), &(acadoWorkspace.QE[ 424 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 496 ]), &(acadoWorkspace.QE[ 504 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 584 ]), &(acadoWorkspace.QE[ 592 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 680 ]), &(acadoWorkspace.QE[ 688 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 784 ]), &(acadoWorkspace.QE[ 792 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 896 ]), &(acadoWorkspace.QE[ 904 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 1016 ]), &(acadoWorkspace.QE[ 1024 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 1144 ]), &(acadoWorkspace.QE[ 1152 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 1280 ]), &(acadoWorkspace.QE[ 1288 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 1424 ]), &(acadoWorkspace.QE[ 1432 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 1576 ]), &(acadoWorkspace.QE[ 1584 ]) );

acado_zeroBlockH11( 7, 9 );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 416 ]), &(acadoWorkspace.QE[ 432 ]) );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 496 ]), &(acadoWorkspace.QE[ 512 ]) );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 584 ]), &(acadoWorkspace.QE[ 600 ]) );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 680 ]), &(acadoWorkspace.QE[ 696 ]) );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 784 ]), &(acadoWorkspace.QE[ 800 ]) );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 896 ]), &(acadoWorkspace.QE[ 912 ]) );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 1016 ]), &(acadoWorkspace.QE[ 1032 ]) );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 1144 ]), &(acadoWorkspace.QE[ 1160 ]) );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 1280 ]), &(acadoWorkspace.QE[ 1296 ]) );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 1424 ]), &(acadoWorkspace.QE[ 1440 ]) );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 1576 ]), &(acadoWorkspace.QE[ 1592 ]) );

acado_zeroBlockH11( 7, 10 );
acado_setBlockH11( 7, 10, &(acadoWorkspace.E[ 496 ]), &(acadoWorkspace.QE[ 520 ]) );
acado_setBlockH11( 7, 10, &(acadoWorkspace.E[ 584 ]), &(acadoWorkspace.QE[ 608 ]) );
acado_setBlockH11( 7, 10, &(acadoWorkspace.E[ 680 ]), &(acadoWorkspace.QE[ 704 ]) );
acado_setBlockH11( 7, 10, &(acadoWorkspace.E[ 784 ]), &(acadoWorkspace.QE[ 808 ]) );
acado_setBlockH11( 7, 10, &(acadoWorkspace.E[ 896 ]), &(acadoWorkspace.QE[ 920 ]) );
acado_setBlockH11( 7, 10, &(acadoWorkspace.E[ 1016 ]), &(acadoWorkspace.QE[ 1040 ]) );
acado_setBlockH11( 7, 10, &(acadoWorkspace.E[ 1144 ]), &(acadoWorkspace.QE[ 1168 ]) );
acado_setBlockH11( 7, 10, &(acadoWorkspace.E[ 1280 ]), &(acadoWorkspace.QE[ 1304 ]) );
acado_setBlockH11( 7, 10, &(acadoWorkspace.E[ 1424 ]), &(acadoWorkspace.QE[ 1448 ]) );
acado_setBlockH11( 7, 10, &(acadoWorkspace.E[ 1576 ]), &(acadoWorkspace.QE[ 1600 ]) );

acado_zeroBlockH11( 7, 11 );
acado_setBlockH11( 7, 11, &(acadoWorkspace.E[ 584 ]), &(acadoWorkspace.QE[ 616 ]) );
acado_setBlockH11( 7, 11, &(acadoWorkspace.E[ 680 ]), &(acadoWorkspace.QE[ 712 ]) );
acado_setBlockH11( 7, 11, &(acadoWorkspace.E[ 784 ]), &(acadoWorkspace.QE[ 816 ]) );
acado_setBlockH11( 7, 11, &(acadoWorkspace.E[ 896 ]), &(acadoWorkspace.QE[ 928 ]) );
acado_setBlockH11( 7, 11, &(acadoWorkspace.E[ 1016 ]), &(acadoWorkspace.QE[ 1048 ]) );
acado_setBlockH11( 7, 11, &(acadoWorkspace.E[ 1144 ]), &(acadoWorkspace.QE[ 1176 ]) );
acado_setBlockH11( 7, 11, &(acadoWorkspace.E[ 1280 ]), &(acadoWorkspace.QE[ 1312 ]) );
acado_setBlockH11( 7, 11, &(acadoWorkspace.E[ 1424 ]), &(acadoWorkspace.QE[ 1456 ]) );
acado_setBlockH11( 7, 11, &(acadoWorkspace.E[ 1576 ]), &(acadoWorkspace.QE[ 1608 ]) );

acado_zeroBlockH11( 7, 12 );
acado_setBlockH11( 7, 12, &(acadoWorkspace.E[ 680 ]), &(acadoWorkspace.QE[ 720 ]) );
acado_setBlockH11( 7, 12, &(acadoWorkspace.E[ 784 ]), &(acadoWorkspace.QE[ 824 ]) );
acado_setBlockH11( 7, 12, &(acadoWorkspace.E[ 896 ]), &(acadoWorkspace.QE[ 936 ]) );
acado_setBlockH11( 7, 12, &(acadoWorkspace.E[ 1016 ]), &(acadoWorkspace.QE[ 1056 ]) );
acado_setBlockH11( 7, 12, &(acadoWorkspace.E[ 1144 ]), &(acadoWorkspace.QE[ 1184 ]) );
acado_setBlockH11( 7, 12, &(acadoWorkspace.E[ 1280 ]), &(acadoWorkspace.QE[ 1320 ]) );
acado_setBlockH11( 7, 12, &(acadoWorkspace.E[ 1424 ]), &(acadoWorkspace.QE[ 1464 ]) );
acado_setBlockH11( 7, 12, &(acadoWorkspace.E[ 1576 ]), &(acadoWorkspace.QE[ 1616 ]) );

acado_zeroBlockH11( 7, 13 );
acado_setBlockH11( 7, 13, &(acadoWorkspace.E[ 784 ]), &(acadoWorkspace.QE[ 832 ]) );
acado_setBlockH11( 7, 13, &(acadoWorkspace.E[ 896 ]), &(acadoWorkspace.QE[ 944 ]) );
acado_setBlockH11( 7, 13, &(acadoWorkspace.E[ 1016 ]), &(acadoWorkspace.QE[ 1064 ]) );
acado_setBlockH11( 7, 13, &(acadoWorkspace.E[ 1144 ]), &(acadoWorkspace.QE[ 1192 ]) );
acado_setBlockH11( 7, 13, &(acadoWorkspace.E[ 1280 ]), &(acadoWorkspace.QE[ 1328 ]) );
acado_setBlockH11( 7, 13, &(acadoWorkspace.E[ 1424 ]), &(acadoWorkspace.QE[ 1472 ]) );
acado_setBlockH11( 7, 13, &(acadoWorkspace.E[ 1576 ]), &(acadoWorkspace.QE[ 1624 ]) );

acado_zeroBlockH11( 7, 14 );
acado_setBlockH11( 7, 14, &(acadoWorkspace.E[ 896 ]), &(acadoWorkspace.QE[ 952 ]) );
acado_setBlockH11( 7, 14, &(acadoWorkspace.E[ 1016 ]), &(acadoWorkspace.QE[ 1072 ]) );
acado_setBlockH11( 7, 14, &(acadoWorkspace.E[ 1144 ]), &(acadoWorkspace.QE[ 1200 ]) );
acado_setBlockH11( 7, 14, &(acadoWorkspace.E[ 1280 ]), &(acadoWorkspace.QE[ 1336 ]) );
acado_setBlockH11( 7, 14, &(acadoWorkspace.E[ 1424 ]), &(acadoWorkspace.QE[ 1480 ]) );
acado_setBlockH11( 7, 14, &(acadoWorkspace.E[ 1576 ]), &(acadoWorkspace.QE[ 1632 ]) );

acado_zeroBlockH11( 7, 15 );
acado_setBlockH11( 7, 15, &(acadoWorkspace.E[ 1016 ]), &(acadoWorkspace.QE[ 1080 ]) );
acado_setBlockH11( 7, 15, &(acadoWorkspace.E[ 1144 ]), &(acadoWorkspace.QE[ 1208 ]) );
acado_setBlockH11( 7, 15, &(acadoWorkspace.E[ 1280 ]), &(acadoWorkspace.QE[ 1344 ]) );
acado_setBlockH11( 7, 15, &(acadoWorkspace.E[ 1424 ]), &(acadoWorkspace.QE[ 1488 ]) );
acado_setBlockH11( 7, 15, &(acadoWorkspace.E[ 1576 ]), &(acadoWorkspace.QE[ 1640 ]) );

acado_zeroBlockH11( 7, 16 );
acado_setBlockH11( 7, 16, &(acadoWorkspace.E[ 1144 ]), &(acadoWorkspace.QE[ 1216 ]) );
acado_setBlockH11( 7, 16, &(acadoWorkspace.E[ 1280 ]), &(acadoWorkspace.QE[ 1352 ]) );
acado_setBlockH11( 7, 16, &(acadoWorkspace.E[ 1424 ]), &(acadoWorkspace.QE[ 1496 ]) );
acado_setBlockH11( 7, 16, &(acadoWorkspace.E[ 1576 ]), &(acadoWorkspace.QE[ 1648 ]) );

acado_zeroBlockH11( 7, 17 );
acado_setBlockH11( 7, 17, &(acadoWorkspace.E[ 1280 ]), &(acadoWorkspace.QE[ 1360 ]) );
acado_setBlockH11( 7, 17, &(acadoWorkspace.E[ 1424 ]), &(acadoWorkspace.QE[ 1504 ]) );
acado_setBlockH11( 7, 17, &(acadoWorkspace.E[ 1576 ]), &(acadoWorkspace.QE[ 1656 ]) );

acado_zeroBlockH11( 7, 18 );
acado_setBlockH11( 7, 18, &(acadoWorkspace.E[ 1424 ]), &(acadoWorkspace.QE[ 1512 ]) );
acado_setBlockH11( 7, 18, &(acadoWorkspace.E[ 1576 ]), &(acadoWorkspace.QE[ 1664 ]) );

acado_zeroBlockH11( 7, 19 );
acado_setBlockH11( 7, 19, &(acadoWorkspace.E[ 1576 ]), &(acadoWorkspace.QE[ 1672 ]) );

acado_setBlockH11_R1( 8, 8, &(acadoWorkspace.R1[ 32 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 352 ]), &(acadoWorkspace.QE[ 352 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 424 ]), &(acadoWorkspace.QE[ 424 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 504 ]), &(acadoWorkspace.QE[ 504 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 592 ]), &(acadoWorkspace.QE[ 592 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 688 ]), &(acadoWorkspace.QE[ 688 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 792 ]), &(acadoWorkspace.QE[ 792 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 904 ]), &(acadoWorkspace.QE[ 904 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 1024 ]), &(acadoWorkspace.QE[ 1024 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 1152 ]), &(acadoWorkspace.QE[ 1152 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 1288 ]), &(acadoWorkspace.QE[ 1288 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 1432 ]), &(acadoWorkspace.QE[ 1432 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 1584 ]), &(acadoWorkspace.QE[ 1584 ]) );

acado_zeroBlockH11( 8, 9 );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 424 ]), &(acadoWorkspace.QE[ 432 ]) );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 504 ]), &(acadoWorkspace.QE[ 512 ]) );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 592 ]), &(acadoWorkspace.QE[ 600 ]) );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 688 ]), &(acadoWorkspace.QE[ 696 ]) );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 792 ]), &(acadoWorkspace.QE[ 800 ]) );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 904 ]), &(acadoWorkspace.QE[ 912 ]) );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 1024 ]), &(acadoWorkspace.QE[ 1032 ]) );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 1152 ]), &(acadoWorkspace.QE[ 1160 ]) );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 1288 ]), &(acadoWorkspace.QE[ 1296 ]) );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 1432 ]), &(acadoWorkspace.QE[ 1440 ]) );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 1584 ]), &(acadoWorkspace.QE[ 1592 ]) );

acado_zeroBlockH11( 8, 10 );
acado_setBlockH11( 8, 10, &(acadoWorkspace.E[ 504 ]), &(acadoWorkspace.QE[ 520 ]) );
acado_setBlockH11( 8, 10, &(acadoWorkspace.E[ 592 ]), &(acadoWorkspace.QE[ 608 ]) );
acado_setBlockH11( 8, 10, &(acadoWorkspace.E[ 688 ]), &(acadoWorkspace.QE[ 704 ]) );
acado_setBlockH11( 8, 10, &(acadoWorkspace.E[ 792 ]), &(acadoWorkspace.QE[ 808 ]) );
acado_setBlockH11( 8, 10, &(acadoWorkspace.E[ 904 ]), &(acadoWorkspace.QE[ 920 ]) );
acado_setBlockH11( 8, 10, &(acadoWorkspace.E[ 1024 ]), &(acadoWorkspace.QE[ 1040 ]) );
acado_setBlockH11( 8, 10, &(acadoWorkspace.E[ 1152 ]), &(acadoWorkspace.QE[ 1168 ]) );
acado_setBlockH11( 8, 10, &(acadoWorkspace.E[ 1288 ]), &(acadoWorkspace.QE[ 1304 ]) );
acado_setBlockH11( 8, 10, &(acadoWorkspace.E[ 1432 ]), &(acadoWorkspace.QE[ 1448 ]) );
acado_setBlockH11( 8, 10, &(acadoWorkspace.E[ 1584 ]), &(acadoWorkspace.QE[ 1600 ]) );

acado_zeroBlockH11( 8, 11 );
acado_setBlockH11( 8, 11, &(acadoWorkspace.E[ 592 ]), &(acadoWorkspace.QE[ 616 ]) );
acado_setBlockH11( 8, 11, &(acadoWorkspace.E[ 688 ]), &(acadoWorkspace.QE[ 712 ]) );
acado_setBlockH11( 8, 11, &(acadoWorkspace.E[ 792 ]), &(acadoWorkspace.QE[ 816 ]) );
acado_setBlockH11( 8, 11, &(acadoWorkspace.E[ 904 ]), &(acadoWorkspace.QE[ 928 ]) );
acado_setBlockH11( 8, 11, &(acadoWorkspace.E[ 1024 ]), &(acadoWorkspace.QE[ 1048 ]) );
acado_setBlockH11( 8, 11, &(acadoWorkspace.E[ 1152 ]), &(acadoWorkspace.QE[ 1176 ]) );
acado_setBlockH11( 8, 11, &(acadoWorkspace.E[ 1288 ]), &(acadoWorkspace.QE[ 1312 ]) );
acado_setBlockH11( 8, 11, &(acadoWorkspace.E[ 1432 ]), &(acadoWorkspace.QE[ 1456 ]) );
acado_setBlockH11( 8, 11, &(acadoWorkspace.E[ 1584 ]), &(acadoWorkspace.QE[ 1608 ]) );

acado_zeroBlockH11( 8, 12 );
acado_setBlockH11( 8, 12, &(acadoWorkspace.E[ 688 ]), &(acadoWorkspace.QE[ 720 ]) );
acado_setBlockH11( 8, 12, &(acadoWorkspace.E[ 792 ]), &(acadoWorkspace.QE[ 824 ]) );
acado_setBlockH11( 8, 12, &(acadoWorkspace.E[ 904 ]), &(acadoWorkspace.QE[ 936 ]) );
acado_setBlockH11( 8, 12, &(acadoWorkspace.E[ 1024 ]), &(acadoWorkspace.QE[ 1056 ]) );
acado_setBlockH11( 8, 12, &(acadoWorkspace.E[ 1152 ]), &(acadoWorkspace.QE[ 1184 ]) );
acado_setBlockH11( 8, 12, &(acadoWorkspace.E[ 1288 ]), &(acadoWorkspace.QE[ 1320 ]) );
acado_setBlockH11( 8, 12, &(acadoWorkspace.E[ 1432 ]), &(acadoWorkspace.QE[ 1464 ]) );
acado_setBlockH11( 8, 12, &(acadoWorkspace.E[ 1584 ]), &(acadoWorkspace.QE[ 1616 ]) );

acado_zeroBlockH11( 8, 13 );
acado_setBlockH11( 8, 13, &(acadoWorkspace.E[ 792 ]), &(acadoWorkspace.QE[ 832 ]) );
acado_setBlockH11( 8, 13, &(acadoWorkspace.E[ 904 ]), &(acadoWorkspace.QE[ 944 ]) );
acado_setBlockH11( 8, 13, &(acadoWorkspace.E[ 1024 ]), &(acadoWorkspace.QE[ 1064 ]) );
acado_setBlockH11( 8, 13, &(acadoWorkspace.E[ 1152 ]), &(acadoWorkspace.QE[ 1192 ]) );
acado_setBlockH11( 8, 13, &(acadoWorkspace.E[ 1288 ]), &(acadoWorkspace.QE[ 1328 ]) );
acado_setBlockH11( 8, 13, &(acadoWorkspace.E[ 1432 ]), &(acadoWorkspace.QE[ 1472 ]) );
acado_setBlockH11( 8, 13, &(acadoWorkspace.E[ 1584 ]), &(acadoWorkspace.QE[ 1624 ]) );

acado_zeroBlockH11( 8, 14 );
acado_setBlockH11( 8, 14, &(acadoWorkspace.E[ 904 ]), &(acadoWorkspace.QE[ 952 ]) );
acado_setBlockH11( 8, 14, &(acadoWorkspace.E[ 1024 ]), &(acadoWorkspace.QE[ 1072 ]) );
acado_setBlockH11( 8, 14, &(acadoWorkspace.E[ 1152 ]), &(acadoWorkspace.QE[ 1200 ]) );
acado_setBlockH11( 8, 14, &(acadoWorkspace.E[ 1288 ]), &(acadoWorkspace.QE[ 1336 ]) );
acado_setBlockH11( 8, 14, &(acadoWorkspace.E[ 1432 ]), &(acadoWorkspace.QE[ 1480 ]) );
acado_setBlockH11( 8, 14, &(acadoWorkspace.E[ 1584 ]), &(acadoWorkspace.QE[ 1632 ]) );

acado_zeroBlockH11( 8, 15 );
acado_setBlockH11( 8, 15, &(acadoWorkspace.E[ 1024 ]), &(acadoWorkspace.QE[ 1080 ]) );
acado_setBlockH11( 8, 15, &(acadoWorkspace.E[ 1152 ]), &(acadoWorkspace.QE[ 1208 ]) );
acado_setBlockH11( 8, 15, &(acadoWorkspace.E[ 1288 ]), &(acadoWorkspace.QE[ 1344 ]) );
acado_setBlockH11( 8, 15, &(acadoWorkspace.E[ 1432 ]), &(acadoWorkspace.QE[ 1488 ]) );
acado_setBlockH11( 8, 15, &(acadoWorkspace.E[ 1584 ]), &(acadoWorkspace.QE[ 1640 ]) );

acado_zeroBlockH11( 8, 16 );
acado_setBlockH11( 8, 16, &(acadoWorkspace.E[ 1152 ]), &(acadoWorkspace.QE[ 1216 ]) );
acado_setBlockH11( 8, 16, &(acadoWorkspace.E[ 1288 ]), &(acadoWorkspace.QE[ 1352 ]) );
acado_setBlockH11( 8, 16, &(acadoWorkspace.E[ 1432 ]), &(acadoWorkspace.QE[ 1496 ]) );
acado_setBlockH11( 8, 16, &(acadoWorkspace.E[ 1584 ]), &(acadoWorkspace.QE[ 1648 ]) );

acado_zeroBlockH11( 8, 17 );
acado_setBlockH11( 8, 17, &(acadoWorkspace.E[ 1288 ]), &(acadoWorkspace.QE[ 1360 ]) );
acado_setBlockH11( 8, 17, &(acadoWorkspace.E[ 1432 ]), &(acadoWorkspace.QE[ 1504 ]) );
acado_setBlockH11( 8, 17, &(acadoWorkspace.E[ 1584 ]), &(acadoWorkspace.QE[ 1656 ]) );

acado_zeroBlockH11( 8, 18 );
acado_setBlockH11( 8, 18, &(acadoWorkspace.E[ 1432 ]), &(acadoWorkspace.QE[ 1512 ]) );
acado_setBlockH11( 8, 18, &(acadoWorkspace.E[ 1584 ]), &(acadoWorkspace.QE[ 1664 ]) );

acado_zeroBlockH11( 8, 19 );
acado_setBlockH11( 8, 19, &(acadoWorkspace.E[ 1584 ]), &(acadoWorkspace.QE[ 1672 ]) );

acado_setBlockH11_R1( 9, 9, &(acadoWorkspace.R1[ 36 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QE[ 432 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 512 ]), &(acadoWorkspace.QE[ 512 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 600 ]), &(acadoWorkspace.QE[ 600 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 696 ]), &(acadoWorkspace.QE[ 696 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 800 ]), &(acadoWorkspace.QE[ 800 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 912 ]), &(acadoWorkspace.QE[ 912 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 1032 ]), &(acadoWorkspace.QE[ 1032 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 1160 ]), &(acadoWorkspace.QE[ 1160 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.QE[ 1296 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QE[ 1440 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 1592 ]), &(acadoWorkspace.QE[ 1592 ]) );

acado_zeroBlockH11( 9, 10 );
acado_setBlockH11( 9, 10, &(acadoWorkspace.E[ 512 ]), &(acadoWorkspace.QE[ 520 ]) );
acado_setBlockH11( 9, 10, &(acadoWorkspace.E[ 600 ]), &(acadoWorkspace.QE[ 608 ]) );
acado_setBlockH11( 9, 10, &(acadoWorkspace.E[ 696 ]), &(acadoWorkspace.QE[ 704 ]) );
acado_setBlockH11( 9, 10, &(acadoWorkspace.E[ 800 ]), &(acadoWorkspace.QE[ 808 ]) );
acado_setBlockH11( 9, 10, &(acadoWorkspace.E[ 912 ]), &(acadoWorkspace.QE[ 920 ]) );
acado_setBlockH11( 9, 10, &(acadoWorkspace.E[ 1032 ]), &(acadoWorkspace.QE[ 1040 ]) );
acado_setBlockH11( 9, 10, &(acadoWorkspace.E[ 1160 ]), &(acadoWorkspace.QE[ 1168 ]) );
acado_setBlockH11( 9, 10, &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.QE[ 1304 ]) );
acado_setBlockH11( 9, 10, &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QE[ 1448 ]) );
acado_setBlockH11( 9, 10, &(acadoWorkspace.E[ 1592 ]), &(acadoWorkspace.QE[ 1600 ]) );

acado_zeroBlockH11( 9, 11 );
acado_setBlockH11( 9, 11, &(acadoWorkspace.E[ 600 ]), &(acadoWorkspace.QE[ 616 ]) );
acado_setBlockH11( 9, 11, &(acadoWorkspace.E[ 696 ]), &(acadoWorkspace.QE[ 712 ]) );
acado_setBlockH11( 9, 11, &(acadoWorkspace.E[ 800 ]), &(acadoWorkspace.QE[ 816 ]) );
acado_setBlockH11( 9, 11, &(acadoWorkspace.E[ 912 ]), &(acadoWorkspace.QE[ 928 ]) );
acado_setBlockH11( 9, 11, &(acadoWorkspace.E[ 1032 ]), &(acadoWorkspace.QE[ 1048 ]) );
acado_setBlockH11( 9, 11, &(acadoWorkspace.E[ 1160 ]), &(acadoWorkspace.QE[ 1176 ]) );
acado_setBlockH11( 9, 11, &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.QE[ 1312 ]) );
acado_setBlockH11( 9, 11, &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QE[ 1456 ]) );
acado_setBlockH11( 9, 11, &(acadoWorkspace.E[ 1592 ]), &(acadoWorkspace.QE[ 1608 ]) );

acado_zeroBlockH11( 9, 12 );
acado_setBlockH11( 9, 12, &(acadoWorkspace.E[ 696 ]), &(acadoWorkspace.QE[ 720 ]) );
acado_setBlockH11( 9, 12, &(acadoWorkspace.E[ 800 ]), &(acadoWorkspace.QE[ 824 ]) );
acado_setBlockH11( 9, 12, &(acadoWorkspace.E[ 912 ]), &(acadoWorkspace.QE[ 936 ]) );
acado_setBlockH11( 9, 12, &(acadoWorkspace.E[ 1032 ]), &(acadoWorkspace.QE[ 1056 ]) );
acado_setBlockH11( 9, 12, &(acadoWorkspace.E[ 1160 ]), &(acadoWorkspace.QE[ 1184 ]) );
acado_setBlockH11( 9, 12, &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.QE[ 1320 ]) );
acado_setBlockH11( 9, 12, &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QE[ 1464 ]) );
acado_setBlockH11( 9, 12, &(acadoWorkspace.E[ 1592 ]), &(acadoWorkspace.QE[ 1616 ]) );

acado_zeroBlockH11( 9, 13 );
acado_setBlockH11( 9, 13, &(acadoWorkspace.E[ 800 ]), &(acadoWorkspace.QE[ 832 ]) );
acado_setBlockH11( 9, 13, &(acadoWorkspace.E[ 912 ]), &(acadoWorkspace.QE[ 944 ]) );
acado_setBlockH11( 9, 13, &(acadoWorkspace.E[ 1032 ]), &(acadoWorkspace.QE[ 1064 ]) );
acado_setBlockH11( 9, 13, &(acadoWorkspace.E[ 1160 ]), &(acadoWorkspace.QE[ 1192 ]) );
acado_setBlockH11( 9, 13, &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.QE[ 1328 ]) );
acado_setBlockH11( 9, 13, &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QE[ 1472 ]) );
acado_setBlockH11( 9, 13, &(acadoWorkspace.E[ 1592 ]), &(acadoWorkspace.QE[ 1624 ]) );

acado_zeroBlockH11( 9, 14 );
acado_setBlockH11( 9, 14, &(acadoWorkspace.E[ 912 ]), &(acadoWorkspace.QE[ 952 ]) );
acado_setBlockH11( 9, 14, &(acadoWorkspace.E[ 1032 ]), &(acadoWorkspace.QE[ 1072 ]) );
acado_setBlockH11( 9, 14, &(acadoWorkspace.E[ 1160 ]), &(acadoWorkspace.QE[ 1200 ]) );
acado_setBlockH11( 9, 14, &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.QE[ 1336 ]) );
acado_setBlockH11( 9, 14, &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QE[ 1480 ]) );
acado_setBlockH11( 9, 14, &(acadoWorkspace.E[ 1592 ]), &(acadoWorkspace.QE[ 1632 ]) );

acado_zeroBlockH11( 9, 15 );
acado_setBlockH11( 9, 15, &(acadoWorkspace.E[ 1032 ]), &(acadoWorkspace.QE[ 1080 ]) );
acado_setBlockH11( 9, 15, &(acadoWorkspace.E[ 1160 ]), &(acadoWorkspace.QE[ 1208 ]) );
acado_setBlockH11( 9, 15, &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.QE[ 1344 ]) );
acado_setBlockH11( 9, 15, &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QE[ 1488 ]) );
acado_setBlockH11( 9, 15, &(acadoWorkspace.E[ 1592 ]), &(acadoWorkspace.QE[ 1640 ]) );

acado_zeroBlockH11( 9, 16 );
acado_setBlockH11( 9, 16, &(acadoWorkspace.E[ 1160 ]), &(acadoWorkspace.QE[ 1216 ]) );
acado_setBlockH11( 9, 16, &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.QE[ 1352 ]) );
acado_setBlockH11( 9, 16, &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QE[ 1496 ]) );
acado_setBlockH11( 9, 16, &(acadoWorkspace.E[ 1592 ]), &(acadoWorkspace.QE[ 1648 ]) );

acado_zeroBlockH11( 9, 17 );
acado_setBlockH11( 9, 17, &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.QE[ 1360 ]) );
acado_setBlockH11( 9, 17, &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QE[ 1504 ]) );
acado_setBlockH11( 9, 17, &(acadoWorkspace.E[ 1592 ]), &(acadoWorkspace.QE[ 1656 ]) );

acado_zeroBlockH11( 9, 18 );
acado_setBlockH11( 9, 18, &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QE[ 1512 ]) );
acado_setBlockH11( 9, 18, &(acadoWorkspace.E[ 1592 ]), &(acadoWorkspace.QE[ 1664 ]) );

acado_zeroBlockH11( 9, 19 );
acado_setBlockH11( 9, 19, &(acadoWorkspace.E[ 1592 ]), &(acadoWorkspace.QE[ 1672 ]) );

acado_setBlockH11_R1( 10, 10, &(acadoWorkspace.R1[ 40 ]) );
acado_setBlockH11( 10, 10, &(acadoWorkspace.E[ 520 ]), &(acadoWorkspace.QE[ 520 ]) );
acado_setBlockH11( 10, 10, &(acadoWorkspace.E[ 608 ]), &(acadoWorkspace.QE[ 608 ]) );
acado_setBlockH11( 10, 10, &(acadoWorkspace.E[ 704 ]), &(acadoWorkspace.QE[ 704 ]) );
acado_setBlockH11( 10, 10, &(acadoWorkspace.E[ 808 ]), &(acadoWorkspace.QE[ 808 ]) );
acado_setBlockH11( 10, 10, &(acadoWorkspace.E[ 920 ]), &(acadoWorkspace.QE[ 920 ]) );
acado_setBlockH11( 10, 10, &(acadoWorkspace.E[ 1040 ]), &(acadoWorkspace.QE[ 1040 ]) );
acado_setBlockH11( 10, 10, &(acadoWorkspace.E[ 1168 ]), &(acadoWorkspace.QE[ 1168 ]) );
acado_setBlockH11( 10, 10, &(acadoWorkspace.E[ 1304 ]), &(acadoWorkspace.QE[ 1304 ]) );
acado_setBlockH11( 10, 10, &(acadoWorkspace.E[ 1448 ]), &(acadoWorkspace.QE[ 1448 ]) );
acado_setBlockH11( 10, 10, &(acadoWorkspace.E[ 1600 ]), &(acadoWorkspace.QE[ 1600 ]) );

acado_zeroBlockH11( 10, 11 );
acado_setBlockH11( 10, 11, &(acadoWorkspace.E[ 608 ]), &(acadoWorkspace.QE[ 616 ]) );
acado_setBlockH11( 10, 11, &(acadoWorkspace.E[ 704 ]), &(acadoWorkspace.QE[ 712 ]) );
acado_setBlockH11( 10, 11, &(acadoWorkspace.E[ 808 ]), &(acadoWorkspace.QE[ 816 ]) );
acado_setBlockH11( 10, 11, &(acadoWorkspace.E[ 920 ]), &(acadoWorkspace.QE[ 928 ]) );
acado_setBlockH11( 10, 11, &(acadoWorkspace.E[ 1040 ]), &(acadoWorkspace.QE[ 1048 ]) );
acado_setBlockH11( 10, 11, &(acadoWorkspace.E[ 1168 ]), &(acadoWorkspace.QE[ 1176 ]) );
acado_setBlockH11( 10, 11, &(acadoWorkspace.E[ 1304 ]), &(acadoWorkspace.QE[ 1312 ]) );
acado_setBlockH11( 10, 11, &(acadoWorkspace.E[ 1448 ]), &(acadoWorkspace.QE[ 1456 ]) );
acado_setBlockH11( 10, 11, &(acadoWorkspace.E[ 1600 ]), &(acadoWorkspace.QE[ 1608 ]) );

acado_zeroBlockH11( 10, 12 );
acado_setBlockH11( 10, 12, &(acadoWorkspace.E[ 704 ]), &(acadoWorkspace.QE[ 720 ]) );
acado_setBlockH11( 10, 12, &(acadoWorkspace.E[ 808 ]), &(acadoWorkspace.QE[ 824 ]) );
acado_setBlockH11( 10, 12, &(acadoWorkspace.E[ 920 ]), &(acadoWorkspace.QE[ 936 ]) );
acado_setBlockH11( 10, 12, &(acadoWorkspace.E[ 1040 ]), &(acadoWorkspace.QE[ 1056 ]) );
acado_setBlockH11( 10, 12, &(acadoWorkspace.E[ 1168 ]), &(acadoWorkspace.QE[ 1184 ]) );
acado_setBlockH11( 10, 12, &(acadoWorkspace.E[ 1304 ]), &(acadoWorkspace.QE[ 1320 ]) );
acado_setBlockH11( 10, 12, &(acadoWorkspace.E[ 1448 ]), &(acadoWorkspace.QE[ 1464 ]) );
acado_setBlockH11( 10, 12, &(acadoWorkspace.E[ 1600 ]), &(acadoWorkspace.QE[ 1616 ]) );

acado_zeroBlockH11( 10, 13 );
acado_setBlockH11( 10, 13, &(acadoWorkspace.E[ 808 ]), &(acadoWorkspace.QE[ 832 ]) );
acado_setBlockH11( 10, 13, &(acadoWorkspace.E[ 920 ]), &(acadoWorkspace.QE[ 944 ]) );
acado_setBlockH11( 10, 13, &(acadoWorkspace.E[ 1040 ]), &(acadoWorkspace.QE[ 1064 ]) );
acado_setBlockH11( 10, 13, &(acadoWorkspace.E[ 1168 ]), &(acadoWorkspace.QE[ 1192 ]) );
acado_setBlockH11( 10, 13, &(acadoWorkspace.E[ 1304 ]), &(acadoWorkspace.QE[ 1328 ]) );
acado_setBlockH11( 10, 13, &(acadoWorkspace.E[ 1448 ]), &(acadoWorkspace.QE[ 1472 ]) );
acado_setBlockH11( 10, 13, &(acadoWorkspace.E[ 1600 ]), &(acadoWorkspace.QE[ 1624 ]) );

acado_zeroBlockH11( 10, 14 );
acado_setBlockH11( 10, 14, &(acadoWorkspace.E[ 920 ]), &(acadoWorkspace.QE[ 952 ]) );
acado_setBlockH11( 10, 14, &(acadoWorkspace.E[ 1040 ]), &(acadoWorkspace.QE[ 1072 ]) );
acado_setBlockH11( 10, 14, &(acadoWorkspace.E[ 1168 ]), &(acadoWorkspace.QE[ 1200 ]) );
acado_setBlockH11( 10, 14, &(acadoWorkspace.E[ 1304 ]), &(acadoWorkspace.QE[ 1336 ]) );
acado_setBlockH11( 10, 14, &(acadoWorkspace.E[ 1448 ]), &(acadoWorkspace.QE[ 1480 ]) );
acado_setBlockH11( 10, 14, &(acadoWorkspace.E[ 1600 ]), &(acadoWorkspace.QE[ 1632 ]) );

acado_zeroBlockH11( 10, 15 );
acado_setBlockH11( 10, 15, &(acadoWorkspace.E[ 1040 ]), &(acadoWorkspace.QE[ 1080 ]) );
acado_setBlockH11( 10, 15, &(acadoWorkspace.E[ 1168 ]), &(acadoWorkspace.QE[ 1208 ]) );
acado_setBlockH11( 10, 15, &(acadoWorkspace.E[ 1304 ]), &(acadoWorkspace.QE[ 1344 ]) );
acado_setBlockH11( 10, 15, &(acadoWorkspace.E[ 1448 ]), &(acadoWorkspace.QE[ 1488 ]) );
acado_setBlockH11( 10, 15, &(acadoWorkspace.E[ 1600 ]), &(acadoWorkspace.QE[ 1640 ]) );

acado_zeroBlockH11( 10, 16 );
acado_setBlockH11( 10, 16, &(acadoWorkspace.E[ 1168 ]), &(acadoWorkspace.QE[ 1216 ]) );
acado_setBlockH11( 10, 16, &(acadoWorkspace.E[ 1304 ]), &(acadoWorkspace.QE[ 1352 ]) );
acado_setBlockH11( 10, 16, &(acadoWorkspace.E[ 1448 ]), &(acadoWorkspace.QE[ 1496 ]) );
acado_setBlockH11( 10, 16, &(acadoWorkspace.E[ 1600 ]), &(acadoWorkspace.QE[ 1648 ]) );

acado_zeroBlockH11( 10, 17 );
acado_setBlockH11( 10, 17, &(acadoWorkspace.E[ 1304 ]), &(acadoWorkspace.QE[ 1360 ]) );
acado_setBlockH11( 10, 17, &(acadoWorkspace.E[ 1448 ]), &(acadoWorkspace.QE[ 1504 ]) );
acado_setBlockH11( 10, 17, &(acadoWorkspace.E[ 1600 ]), &(acadoWorkspace.QE[ 1656 ]) );

acado_zeroBlockH11( 10, 18 );
acado_setBlockH11( 10, 18, &(acadoWorkspace.E[ 1448 ]), &(acadoWorkspace.QE[ 1512 ]) );
acado_setBlockH11( 10, 18, &(acadoWorkspace.E[ 1600 ]), &(acadoWorkspace.QE[ 1664 ]) );

acado_zeroBlockH11( 10, 19 );
acado_setBlockH11( 10, 19, &(acadoWorkspace.E[ 1600 ]), &(acadoWorkspace.QE[ 1672 ]) );

acado_setBlockH11_R1( 11, 11, &(acadoWorkspace.R1[ 44 ]) );
acado_setBlockH11( 11, 11, &(acadoWorkspace.E[ 616 ]), &(acadoWorkspace.QE[ 616 ]) );
acado_setBlockH11( 11, 11, &(acadoWorkspace.E[ 712 ]), &(acadoWorkspace.QE[ 712 ]) );
acado_setBlockH11( 11, 11, &(acadoWorkspace.E[ 816 ]), &(acadoWorkspace.QE[ 816 ]) );
acado_setBlockH11( 11, 11, &(acadoWorkspace.E[ 928 ]), &(acadoWorkspace.QE[ 928 ]) );
acado_setBlockH11( 11, 11, &(acadoWorkspace.E[ 1048 ]), &(acadoWorkspace.QE[ 1048 ]) );
acado_setBlockH11( 11, 11, &(acadoWorkspace.E[ 1176 ]), &(acadoWorkspace.QE[ 1176 ]) );
acado_setBlockH11( 11, 11, &(acadoWorkspace.E[ 1312 ]), &(acadoWorkspace.QE[ 1312 ]) );
acado_setBlockH11( 11, 11, &(acadoWorkspace.E[ 1456 ]), &(acadoWorkspace.QE[ 1456 ]) );
acado_setBlockH11( 11, 11, &(acadoWorkspace.E[ 1608 ]), &(acadoWorkspace.QE[ 1608 ]) );

acado_zeroBlockH11( 11, 12 );
acado_setBlockH11( 11, 12, &(acadoWorkspace.E[ 712 ]), &(acadoWorkspace.QE[ 720 ]) );
acado_setBlockH11( 11, 12, &(acadoWorkspace.E[ 816 ]), &(acadoWorkspace.QE[ 824 ]) );
acado_setBlockH11( 11, 12, &(acadoWorkspace.E[ 928 ]), &(acadoWorkspace.QE[ 936 ]) );
acado_setBlockH11( 11, 12, &(acadoWorkspace.E[ 1048 ]), &(acadoWorkspace.QE[ 1056 ]) );
acado_setBlockH11( 11, 12, &(acadoWorkspace.E[ 1176 ]), &(acadoWorkspace.QE[ 1184 ]) );
acado_setBlockH11( 11, 12, &(acadoWorkspace.E[ 1312 ]), &(acadoWorkspace.QE[ 1320 ]) );
acado_setBlockH11( 11, 12, &(acadoWorkspace.E[ 1456 ]), &(acadoWorkspace.QE[ 1464 ]) );
acado_setBlockH11( 11, 12, &(acadoWorkspace.E[ 1608 ]), &(acadoWorkspace.QE[ 1616 ]) );

acado_zeroBlockH11( 11, 13 );
acado_setBlockH11( 11, 13, &(acadoWorkspace.E[ 816 ]), &(acadoWorkspace.QE[ 832 ]) );
acado_setBlockH11( 11, 13, &(acadoWorkspace.E[ 928 ]), &(acadoWorkspace.QE[ 944 ]) );
acado_setBlockH11( 11, 13, &(acadoWorkspace.E[ 1048 ]), &(acadoWorkspace.QE[ 1064 ]) );
acado_setBlockH11( 11, 13, &(acadoWorkspace.E[ 1176 ]), &(acadoWorkspace.QE[ 1192 ]) );
acado_setBlockH11( 11, 13, &(acadoWorkspace.E[ 1312 ]), &(acadoWorkspace.QE[ 1328 ]) );
acado_setBlockH11( 11, 13, &(acadoWorkspace.E[ 1456 ]), &(acadoWorkspace.QE[ 1472 ]) );
acado_setBlockH11( 11, 13, &(acadoWorkspace.E[ 1608 ]), &(acadoWorkspace.QE[ 1624 ]) );

acado_zeroBlockH11( 11, 14 );
acado_setBlockH11( 11, 14, &(acadoWorkspace.E[ 928 ]), &(acadoWorkspace.QE[ 952 ]) );
acado_setBlockH11( 11, 14, &(acadoWorkspace.E[ 1048 ]), &(acadoWorkspace.QE[ 1072 ]) );
acado_setBlockH11( 11, 14, &(acadoWorkspace.E[ 1176 ]), &(acadoWorkspace.QE[ 1200 ]) );
acado_setBlockH11( 11, 14, &(acadoWorkspace.E[ 1312 ]), &(acadoWorkspace.QE[ 1336 ]) );
acado_setBlockH11( 11, 14, &(acadoWorkspace.E[ 1456 ]), &(acadoWorkspace.QE[ 1480 ]) );
acado_setBlockH11( 11, 14, &(acadoWorkspace.E[ 1608 ]), &(acadoWorkspace.QE[ 1632 ]) );

acado_zeroBlockH11( 11, 15 );
acado_setBlockH11( 11, 15, &(acadoWorkspace.E[ 1048 ]), &(acadoWorkspace.QE[ 1080 ]) );
acado_setBlockH11( 11, 15, &(acadoWorkspace.E[ 1176 ]), &(acadoWorkspace.QE[ 1208 ]) );
acado_setBlockH11( 11, 15, &(acadoWorkspace.E[ 1312 ]), &(acadoWorkspace.QE[ 1344 ]) );
acado_setBlockH11( 11, 15, &(acadoWorkspace.E[ 1456 ]), &(acadoWorkspace.QE[ 1488 ]) );
acado_setBlockH11( 11, 15, &(acadoWorkspace.E[ 1608 ]), &(acadoWorkspace.QE[ 1640 ]) );

acado_zeroBlockH11( 11, 16 );
acado_setBlockH11( 11, 16, &(acadoWorkspace.E[ 1176 ]), &(acadoWorkspace.QE[ 1216 ]) );
acado_setBlockH11( 11, 16, &(acadoWorkspace.E[ 1312 ]), &(acadoWorkspace.QE[ 1352 ]) );
acado_setBlockH11( 11, 16, &(acadoWorkspace.E[ 1456 ]), &(acadoWorkspace.QE[ 1496 ]) );
acado_setBlockH11( 11, 16, &(acadoWorkspace.E[ 1608 ]), &(acadoWorkspace.QE[ 1648 ]) );

acado_zeroBlockH11( 11, 17 );
acado_setBlockH11( 11, 17, &(acadoWorkspace.E[ 1312 ]), &(acadoWorkspace.QE[ 1360 ]) );
acado_setBlockH11( 11, 17, &(acadoWorkspace.E[ 1456 ]), &(acadoWorkspace.QE[ 1504 ]) );
acado_setBlockH11( 11, 17, &(acadoWorkspace.E[ 1608 ]), &(acadoWorkspace.QE[ 1656 ]) );

acado_zeroBlockH11( 11, 18 );
acado_setBlockH11( 11, 18, &(acadoWorkspace.E[ 1456 ]), &(acadoWorkspace.QE[ 1512 ]) );
acado_setBlockH11( 11, 18, &(acadoWorkspace.E[ 1608 ]), &(acadoWorkspace.QE[ 1664 ]) );

acado_zeroBlockH11( 11, 19 );
acado_setBlockH11( 11, 19, &(acadoWorkspace.E[ 1608 ]), &(acadoWorkspace.QE[ 1672 ]) );

acado_setBlockH11_R1( 12, 12, &(acadoWorkspace.R1[ 48 ]) );
acado_setBlockH11( 12, 12, &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.QE[ 720 ]) );
acado_setBlockH11( 12, 12, &(acadoWorkspace.E[ 824 ]), &(acadoWorkspace.QE[ 824 ]) );
acado_setBlockH11( 12, 12, &(acadoWorkspace.E[ 936 ]), &(acadoWorkspace.QE[ 936 ]) );
acado_setBlockH11( 12, 12, &(acadoWorkspace.E[ 1056 ]), &(acadoWorkspace.QE[ 1056 ]) );
acado_setBlockH11( 12, 12, &(acadoWorkspace.E[ 1184 ]), &(acadoWorkspace.QE[ 1184 ]) );
acado_setBlockH11( 12, 12, &(acadoWorkspace.E[ 1320 ]), &(acadoWorkspace.QE[ 1320 ]) );
acado_setBlockH11( 12, 12, &(acadoWorkspace.E[ 1464 ]), &(acadoWorkspace.QE[ 1464 ]) );
acado_setBlockH11( 12, 12, &(acadoWorkspace.E[ 1616 ]), &(acadoWorkspace.QE[ 1616 ]) );

acado_zeroBlockH11( 12, 13 );
acado_setBlockH11( 12, 13, &(acadoWorkspace.E[ 824 ]), &(acadoWorkspace.QE[ 832 ]) );
acado_setBlockH11( 12, 13, &(acadoWorkspace.E[ 936 ]), &(acadoWorkspace.QE[ 944 ]) );
acado_setBlockH11( 12, 13, &(acadoWorkspace.E[ 1056 ]), &(acadoWorkspace.QE[ 1064 ]) );
acado_setBlockH11( 12, 13, &(acadoWorkspace.E[ 1184 ]), &(acadoWorkspace.QE[ 1192 ]) );
acado_setBlockH11( 12, 13, &(acadoWorkspace.E[ 1320 ]), &(acadoWorkspace.QE[ 1328 ]) );
acado_setBlockH11( 12, 13, &(acadoWorkspace.E[ 1464 ]), &(acadoWorkspace.QE[ 1472 ]) );
acado_setBlockH11( 12, 13, &(acadoWorkspace.E[ 1616 ]), &(acadoWorkspace.QE[ 1624 ]) );

acado_zeroBlockH11( 12, 14 );
acado_setBlockH11( 12, 14, &(acadoWorkspace.E[ 936 ]), &(acadoWorkspace.QE[ 952 ]) );
acado_setBlockH11( 12, 14, &(acadoWorkspace.E[ 1056 ]), &(acadoWorkspace.QE[ 1072 ]) );
acado_setBlockH11( 12, 14, &(acadoWorkspace.E[ 1184 ]), &(acadoWorkspace.QE[ 1200 ]) );
acado_setBlockH11( 12, 14, &(acadoWorkspace.E[ 1320 ]), &(acadoWorkspace.QE[ 1336 ]) );
acado_setBlockH11( 12, 14, &(acadoWorkspace.E[ 1464 ]), &(acadoWorkspace.QE[ 1480 ]) );
acado_setBlockH11( 12, 14, &(acadoWorkspace.E[ 1616 ]), &(acadoWorkspace.QE[ 1632 ]) );

acado_zeroBlockH11( 12, 15 );
acado_setBlockH11( 12, 15, &(acadoWorkspace.E[ 1056 ]), &(acadoWorkspace.QE[ 1080 ]) );
acado_setBlockH11( 12, 15, &(acadoWorkspace.E[ 1184 ]), &(acadoWorkspace.QE[ 1208 ]) );
acado_setBlockH11( 12, 15, &(acadoWorkspace.E[ 1320 ]), &(acadoWorkspace.QE[ 1344 ]) );
acado_setBlockH11( 12, 15, &(acadoWorkspace.E[ 1464 ]), &(acadoWorkspace.QE[ 1488 ]) );
acado_setBlockH11( 12, 15, &(acadoWorkspace.E[ 1616 ]), &(acadoWorkspace.QE[ 1640 ]) );

acado_zeroBlockH11( 12, 16 );
acado_setBlockH11( 12, 16, &(acadoWorkspace.E[ 1184 ]), &(acadoWorkspace.QE[ 1216 ]) );
acado_setBlockH11( 12, 16, &(acadoWorkspace.E[ 1320 ]), &(acadoWorkspace.QE[ 1352 ]) );
acado_setBlockH11( 12, 16, &(acadoWorkspace.E[ 1464 ]), &(acadoWorkspace.QE[ 1496 ]) );
acado_setBlockH11( 12, 16, &(acadoWorkspace.E[ 1616 ]), &(acadoWorkspace.QE[ 1648 ]) );

acado_zeroBlockH11( 12, 17 );
acado_setBlockH11( 12, 17, &(acadoWorkspace.E[ 1320 ]), &(acadoWorkspace.QE[ 1360 ]) );
acado_setBlockH11( 12, 17, &(acadoWorkspace.E[ 1464 ]), &(acadoWorkspace.QE[ 1504 ]) );
acado_setBlockH11( 12, 17, &(acadoWorkspace.E[ 1616 ]), &(acadoWorkspace.QE[ 1656 ]) );

acado_zeroBlockH11( 12, 18 );
acado_setBlockH11( 12, 18, &(acadoWorkspace.E[ 1464 ]), &(acadoWorkspace.QE[ 1512 ]) );
acado_setBlockH11( 12, 18, &(acadoWorkspace.E[ 1616 ]), &(acadoWorkspace.QE[ 1664 ]) );

acado_zeroBlockH11( 12, 19 );
acado_setBlockH11( 12, 19, &(acadoWorkspace.E[ 1616 ]), &(acadoWorkspace.QE[ 1672 ]) );

acado_setBlockH11_R1( 13, 13, &(acadoWorkspace.R1[ 52 ]) );
acado_setBlockH11( 13, 13, &(acadoWorkspace.E[ 832 ]), &(acadoWorkspace.QE[ 832 ]) );
acado_setBlockH11( 13, 13, &(acadoWorkspace.E[ 944 ]), &(acadoWorkspace.QE[ 944 ]) );
acado_setBlockH11( 13, 13, &(acadoWorkspace.E[ 1064 ]), &(acadoWorkspace.QE[ 1064 ]) );
acado_setBlockH11( 13, 13, &(acadoWorkspace.E[ 1192 ]), &(acadoWorkspace.QE[ 1192 ]) );
acado_setBlockH11( 13, 13, &(acadoWorkspace.E[ 1328 ]), &(acadoWorkspace.QE[ 1328 ]) );
acado_setBlockH11( 13, 13, &(acadoWorkspace.E[ 1472 ]), &(acadoWorkspace.QE[ 1472 ]) );
acado_setBlockH11( 13, 13, &(acadoWorkspace.E[ 1624 ]), &(acadoWorkspace.QE[ 1624 ]) );

acado_zeroBlockH11( 13, 14 );
acado_setBlockH11( 13, 14, &(acadoWorkspace.E[ 944 ]), &(acadoWorkspace.QE[ 952 ]) );
acado_setBlockH11( 13, 14, &(acadoWorkspace.E[ 1064 ]), &(acadoWorkspace.QE[ 1072 ]) );
acado_setBlockH11( 13, 14, &(acadoWorkspace.E[ 1192 ]), &(acadoWorkspace.QE[ 1200 ]) );
acado_setBlockH11( 13, 14, &(acadoWorkspace.E[ 1328 ]), &(acadoWorkspace.QE[ 1336 ]) );
acado_setBlockH11( 13, 14, &(acadoWorkspace.E[ 1472 ]), &(acadoWorkspace.QE[ 1480 ]) );
acado_setBlockH11( 13, 14, &(acadoWorkspace.E[ 1624 ]), &(acadoWorkspace.QE[ 1632 ]) );

acado_zeroBlockH11( 13, 15 );
acado_setBlockH11( 13, 15, &(acadoWorkspace.E[ 1064 ]), &(acadoWorkspace.QE[ 1080 ]) );
acado_setBlockH11( 13, 15, &(acadoWorkspace.E[ 1192 ]), &(acadoWorkspace.QE[ 1208 ]) );
acado_setBlockH11( 13, 15, &(acadoWorkspace.E[ 1328 ]), &(acadoWorkspace.QE[ 1344 ]) );
acado_setBlockH11( 13, 15, &(acadoWorkspace.E[ 1472 ]), &(acadoWorkspace.QE[ 1488 ]) );
acado_setBlockH11( 13, 15, &(acadoWorkspace.E[ 1624 ]), &(acadoWorkspace.QE[ 1640 ]) );

acado_zeroBlockH11( 13, 16 );
acado_setBlockH11( 13, 16, &(acadoWorkspace.E[ 1192 ]), &(acadoWorkspace.QE[ 1216 ]) );
acado_setBlockH11( 13, 16, &(acadoWorkspace.E[ 1328 ]), &(acadoWorkspace.QE[ 1352 ]) );
acado_setBlockH11( 13, 16, &(acadoWorkspace.E[ 1472 ]), &(acadoWorkspace.QE[ 1496 ]) );
acado_setBlockH11( 13, 16, &(acadoWorkspace.E[ 1624 ]), &(acadoWorkspace.QE[ 1648 ]) );

acado_zeroBlockH11( 13, 17 );
acado_setBlockH11( 13, 17, &(acadoWorkspace.E[ 1328 ]), &(acadoWorkspace.QE[ 1360 ]) );
acado_setBlockH11( 13, 17, &(acadoWorkspace.E[ 1472 ]), &(acadoWorkspace.QE[ 1504 ]) );
acado_setBlockH11( 13, 17, &(acadoWorkspace.E[ 1624 ]), &(acadoWorkspace.QE[ 1656 ]) );

acado_zeroBlockH11( 13, 18 );
acado_setBlockH11( 13, 18, &(acadoWorkspace.E[ 1472 ]), &(acadoWorkspace.QE[ 1512 ]) );
acado_setBlockH11( 13, 18, &(acadoWorkspace.E[ 1624 ]), &(acadoWorkspace.QE[ 1664 ]) );

acado_zeroBlockH11( 13, 19 );
acado_setBlockH11( 13, 19, &(acadoWorkspace.E[ 1624 ]), &(acadoWorkspace.QE[ 1672 ]) );

acado_setBlockH11_R1( 14, 14, &(acadoWorkspace.R1[ 56 ]) );
acado_setBlockH11( 14, 14, &(acadoWorkspace.E[ 952 ]), &(acadoWorkspace.QE[ 952 ]) );
acado_setBlockH11( 14, 14, &(acadoWorkspace.E[ 1072 ]), &(acadoWorkspace.QE[ 1072 ]) );
acado_setBlockH11( 14, 14, &(acadoWorkspace.E[ 1200 ]), &(acadoWorkspace.QE[ 1200 ]) );
acado_setBlockH11( 14, 14, &(acadoWorkspace.E[ 1336 ]), &(acadoWorkspace.QE[ 1336 ]) );
acado_setBlockH11( 14, 14, &(acadoWorkspace.E[ 1480 ]), &(acadoWorkspace.QE[ 1480 ]) );
acado_setBlockH11( 14, 14, &(acadoWorkspace.E[ 1632 ]), &(acadoWorkspace.QE[ 1632 ]) );

acado_zeroBlockH11( 14, 15 );
acado_setBlockH11( 14, 15, &(acadoWorkspace.E[ 1072 ]), &(acadoWorkspace.QE[ 1080 ]) );
acado_setBlockH11( 14, 15, &(acadoWorkspace.E[ 1200 ]), &(acadoWorkspace.QE[ 1208 ]) );
acado_setBlockH11( 14, 15, &(acadoWorkspace.E[ 1336 ]), &(acadoWorkspace.QE[ 1344 ]) );
acado_setBlockH11( 14, 15, &(acadoWorkspace.E[ 1480 ]), &(acadoWorkspace.QE[ 1488 ]) );
acado_setBlockH11( 14, 15, &(acadoWorkspace.E[ 1632 ]), &(acadoWorkspace.QE[ 1640 ]) );

acado_zeroBlockH11( 14, 16 );
acado_setBlockH11( 14, 16, &(acadoWorkspace.E[ 1200 ]), &(acadoWorkspace.QE[ 1216 ]) );
acado_setBlockH11( 14, 16, &(acadoWorkspace.E[ 1336 ]), &(acadoWorkspace.QE[ 1352 ]) );
acado_setBlockH11( 14, 16, &(acadoWorkspace.E[ 1480 ]), &(acadoWorkspace.QE[ 1496 ]) );
acado_setBlockH11( 14, 16, &(acadoWorkspace.E[ 1632 ]), &(acadoWorkspace.QE[ 1648 ]) );

acado_zeroBlockH11( 14, 17 );
acado_setBlockH11( 14, 17, &(acadoWorkspace.E[ 1336 ]), &(acadoWorkspace.QE[ 1360 ]) );
acado_setBlockH11( 14, 17, &(acadoWorkspace.E[ 1480 ]), &(acadoWorkspace.QE[ 1504 ]) );
acado_setBlockH11( 14, 17, &(acadoWorkspace.E[ 1632 ]), &(acadoWorkspace.QE[ 1656 ]) );

acado_zeroBlockH11( 14, 18 );
acado_setBlockH11( 14, 18, &(acadoWorkspace.E[ 1480 ]), &(acadoWorkspace.QE[ 1512 ]) );
acado_setBlockH11( 14, 18, &(acadoWorkspace.E[ 1632 ]), &(acadoWorkspace.QE[ 1664 ]) );

acado_zeroBlockH11( 14, 19 );
acado_setBlockH11( 14, 19, &(acadoWorkspace.E[ 1632 ]), &(acadoWorkspace.QE[ 1672 ]) );

acado_setBlockH11_R1( 15, 15, &(acadoWorkspace.R1[ 60 ]) );
acado_setBlockH11( 15, 15, &(acadoWorkspace.E[ 1080 ]), &(acadoWorkspace.QE[ 1080 ]) );
acado_setBlockH11( 15, 15, &(acadoWorkspace.E[ 1208 ]), &(acadoWorkspace.QE[ 1208 ]) );
acado_setBlockH11( 15, 15, &(acadoWorkspace.E[ 1344 ]), &(acadoWorkspace.QE[ 1344 ]) );
acado_setBlockH11( 15, 15, &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.QE[ 1488 ]) );
acado_setBlockH11( 15, 15, &(acadoWorkspace.E[ 1640 ]), &(acadoWorkspace.QE[ 1640 ]) );

acado_zeroBlockH11( 15, 16 );
acado_setBlockH11( 15, 16, &(acadoWorkspace.E[ 1208 ]), &(acadoWorkspace.QE[ 1216 ]) );
acado_setBlockH11( 15, 16, &(acadoWorkspace.E[ 1344 ]), &(acadoWorkspace.QE[ 1352 ]) );
acado_setBlockH11( 15, 16, &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.QE[ 1496 ]) );
acado_setBlockH11( 15, 16, &(acadoWorkspace.E[ 1640 ]), &(acadoWorkspace.QE[ 1648 ]) );

acado_zeroBlockH11( 15, 17 );
acado_setBlockH11( 15, 17, &(acadoWorkspace.E[ 1344 ]), &(acadoWorkspace.QE[ 1360 ]) );
acado_setBlockH11( 15, 17, &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.QE[ 1504 ]) );
acado_setBlockH11( 15, 17, &(acadoWorkspace.E[ 1640 ]), &(acadoWorkspace.QE[ 1656 ]) );

acado_zeroBlockH11( 15, 18 );
acado_setBlockH11( 15, 18, &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.QE[ 1512 ]) );
acado_setBlockH11( 15, 18, &(acadoWorkspace.E[ 1640 ]), &(acadoWorkspace.QE[ 1664 ]) );

acado_zeroBlockH11( 15, 19 );
acado_setBlockH11( 15, 19, &(acadoWorkspace.E[ 1640 ]), &(acadoWorkspace.QE[ 1672 ]) );

acado_setBlockH11_R1( 16, 16, &(acadoWorkspace.R1[ 64 ]) );
acado_setBlockH11( 16, 16, &(acadoWorkspace.E[ 1216 ]), &(acadoWorkspace.QE[ 1216 ]) );
acado_setBlockH11( 16, 16, &(acadoWorkspace.E[ 1352 ]), &(acadoWorkspace.QE[ 1352 ]) );
acado_setBlockH11( 16, 16, &(acadoWorkspace.E[ 1496 ]), &(acadoWorkspace.QE[ 1496 ]) );
acado_setBlockH11( 16, 16, &(acadoWorkspace.E[ 1648 ]), &(acadoWorkspace.QE[ 1648 ]) );

acado_zeroBlockH11( 16, 17 );
acado_setBlockH11( 16, 17, &(acadoWorkspace.E[ 1352 ]), &(acadoWorkspace.QE[ 1360 ]) );
acado_setBlockH11( 16, 17, &(acadoWorkspace.E[ 1496 ]), &(acadoWorkspace.QE[ 1504 ]) );
acado_setBlockH11( 16, 17, &(acadoWorkspace.E[ 1648 ]), &(acadoWorkspace.QE[ 1656 ]) );

acado_zeroBlockH11( 16, 18 );
acado_setBlockH11( 16, 18, &(acadoWorkspace.E[ 1496 ]), &(acadoWorkspace.QE[ 1512 ]) );
acado_setBlockH11( 16, 18, &(acadoWorkspace.E[ 1648 ]), &(acadoWorkspace.QE[ 1664 ]) );

acado_zeroBlockH11( 16, 19 );
acado_setBlockH11( 16, 19, &(acadoWorkspace.E[ 1648 ]), &(acadoWorkspace.QE[ 1672 ]) );

acado_setBlockH11_R1( 17, 17, &(acadoWorkspace.R1[ 68 ]) );
acado_setBlockH11( 17, 17, &(acadoWorkspace.E[ 1360 ]), &(acadoWorkspace.QE[ 1360 ]) );
acado_setBlockH11( 17, 17, &(acadoWorkspace.E[ 1504 ]), &(acadoWorkspace.QE[ 1504 ]) );
acado_setBlockH11( 17, 17, &(acadoWorkspace.E[ 1656 ]), &(acadoWorkspace.QE[ 1656 ]) );

acado_zeroBlockH11( 17, 18 );
acado_setBlockH11( 17, 18, &(acadoWorkspace.E[ 1504 ]), &(acadoWorkspace.QE[ 1512 ]) );
acado_setBlockH11( 17, 18, &(acadoWorkspace.E[ 1656 ]), &(acadoWorkspace.QE[ 1664 ]) );

acado_zeroBlockH11( 17, 19 );
acado_setBlockH11( 17, 19, &(acadoWorkspace.E[ 1656 ]), &(acadoWorkspace.QE[ 1672 ]) );

acado_setBlockH11_R1( 18, 18, &(acadoWorkspace.R1[ 72 ]) );
acado_setBlockH11( 18, 18, &(acadoWorkspace.E[ 1512 ]), &(acadoWorkspace.QE[ 1512 ]) );
acado_setBlockH11( 18, 18, &(acadoWorkspace.E[ 1664 ]), &(acadoWorkspace.QE[ 1664 ]) );

acado_zeroBlockH11( 18, 19 );
acado_setBlockH11( 18, 19, &(acadoWorkspace.E[ 1664 ]), &(acadoWorkspace.QE[ 1672 ]) );

acado_setBlockH11_R1( 19, 19, &(acadoWorkspace.R1[ 76 ]) );
acado_setBlockH11( 19, 19, &(acadoWorkspace.E[ 1672 ]), &(acadoWorkspace.QE[ 1672 ]) );


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
acado_copyHTH( 10, 0 );
acado_copyHTH( 10, 1 );
acado_copyHTH( 10, 2 );
acado_copyHTH( 10, 3 );
acado_copyHTH( 10, 4 );
acado_copyHTH( 10, 5 );
acado_copyHTH( 10, 6 );
acado_copyHTH( 10, 7 );
acado_copyHTH( 10, 8 );
acado_copyHTH( 10, 9 );
acado_copyHTH( 11, 0 );
acado_copyHTH( 11, 1 );
acado_copyHTH( 11, 2 );
acado_copyHTH( 11, 3 );
acado_copyHTH( 11, 4 );
acado_copyHTH( 11, 5 );
acado_copyHTH( 11, 6 );
acado_copyHTH( 11, 7 );
acado_copyHTH( 11, 8 );
acado_copyHTH( 11, 9 );
acado_copyHTH( 11, 10 );
acado_copyHTH( 12, 0 );
acado_copyHTH( 12, 1 );
acado_copyHTH( 12, 2 );
acado_copyHTH( 12, 3 );
acado_copyHTH( 12, 4 );
acado_copyHTH( 12, 5 );
acado_copyHTH( 12, 6 );
acado_copyHTH( 12, 7 );
acado_copyHTH( 12, 8 );
acado_copyHTH( 12, 9 );
acado_copyHTH( 12, 10 );
acado_copyHTH( 12, 11 );
acado_copyHTH( 13, 0 );
acado_copyHTH( 13, 1 );
acado_copyHTH( 13, 2 );
acado_copyHTH( 13, 3 );
acado_copyHTH( 13, 4 );
acado_copyHTH( 13, 5 );
acado_copyHTH( 13, 6 );
acado_copyHTH( 13, 7 );
acado_copyHTH( 13, 8 );
acado_copyHTH( 13, 9 );
acado_copyHTH( 13, 10 );
acado_copyHTH( 13, 11 );
acado_copyHTH( 13, 12 );
acado_copyHTH( 14, 0 );
acado_copyHTH( 14, 1 );
acado_copyHTH( 14, 2 );
acado_copyHTH( 14, 3 );
acado_copyHTH( 14, 4 );
acado_copyHTH( 14, 5 );
acado_copyHTH( 14, 6 );
acado_copyHTH( 14, 7 );
acado_copyHTH( 14, 8 );
acado_copyHTH( 14, 9 );
acado_copyHTH( 14, 10 );
acado_copyHTH( 14, 11 );
acado_copyHTH( 14, 12 );
acado_copyHTH( 14, 13 );
acado_copyHTH( 15, 0 );
acado_copyHTH( 15, 1 );
acado_copyHTH( 15, 2 );
acado_copyHTH( 15, 3 );
acado_copyHTH( 15, 4 );
acado_copyHTH( 15, 5 );
acado_copyHTH( 15, 6 );
acado_copyHTH( 15, 7 );
acado_copyHTH( 15, 8 );
acado_copyHTH( 15, 9 );
acado_copyHTH( 15, 10 );
acado_copyHTH( 15, 11 );
acado_copyHTH( 15, 12 );
acado_copyHTH( 15, 13 );
acado_copyHTH( 15, 14 );
acado_copyHTH( 16, 0 );
acado_copyHTH( 16, 1 );
acado_copyHTH( 16, 2 );
acado_copyHTH( 16, 3 );
acado_copyHTH( 16, 4 );
acado_copyHTH( 16, 5 );
acado_copyHTH( 16, 6 );
acado_copyHTH( 16, 7 );
acado_copyHTH( 16, 8 );
acado_copyHTH( 16, 9 );
acado_copyHTH( 16, 10 );
acado_copyHTH( 16, 11 );
acado_copyHTH( 16, 12 );
acado_copyHTH( 16, 13 );
acado_copyHTH( 16, 14 );
acado_copyHTH( 16, 15 );
acado_copyHTH( 17, 0 );
acado_copyHTH( 17, 1 );
acado_copyHTH( 17, 2 );
acado_copyHTH( 17, 3 );
acado_copyHTH( 17, 4 );
acado_copyHTH( 17, 5 );
acado_copyHTH( 17, 6 );
acado_copyHTH( 17, 7 );
acado_copyHTH( 17, 8 );
acado_copyHTH( 17, 9 );
acado_copyHTH( 17, 10 );
acado_copyHTH( 17, 11 );
acado_copyHTH( 17, 12 );
acado_copyHTH( 17, 13 );
acado_copyHTH( 17, 14 );
acado_copyHTH( 17, 15 );
acado_copyHTH( 17, 16 );
acado_copyHTH( 18, 0 );
acado_copyHTH( 18, 1 );
acado_copyHTH( 18, 2 );
acado_copyHTH( 18, 3 );
acado_copyHTH( 18, 4 );
acado_copyHTH( 18, 5 );
acado_copyHTH( 18, 6 );
acado_copyHTH( 18, 7 );
acado_copyHTH( 18, 8 );
acado_copyHTH( 18, 9 );
acado_copyHTH( 18, 10 );
acado_copyHTH( 18, 11 );
acado_copyHTH( 18, 12 );
acado_copyHTH( 18, 13 );
acado_copyHTH( 18, 14 );
acado_copyHTH( 18, 15 );
acado_copyHTH( 18, 16 );
acado_copyHTH( 18, 17 );
acado_copyHTH( 19, 0 );
acado_copyHTH( 19, 1 );
acado_copyHTH( 19, 2 );
acado_copyHTH( 19, 3 );
acado_copyHTH( 19, 4 );
acado_copyHTH( 19, 5 );
acado_copyHTH( 19, 6 );
acado_copyHTH( 19, 7 );
acado_copyHTH( 19, 8 );
acado_copyHTH( 19, 9 );
acado_copyHTH( 19, 10 );
acado_copyHTH( 19, 11 );
acado_copyHTH( 19, 12 );
acado_copyHTH( 19, 13 );
acado_copyHTH( 19, 14 );
acado_copyHTH( 19, 15 );
acado_copyHTH( 19, 16 );
acado_copyHTH( 19, 17 );
acado_copyHTH( 19, 18 );

acado_multQ1d( &(acadoWorkspace.Q1[ 16 ]), acadoWorkspace.d, acadoWorkspace.Qd );
acado_multQ1d( &(acadoWorkspace.Q1[ 32 ]), &(acadoWorkspace.d[ 4 ]), &(acadoWorkspace.Qd[ 4 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 48 ]), &(acadoWorkspace.d[ 8 ]), &(acadoWorkspace.Qd[ 8 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 64 ]), &(acadoWorkspace.d[ 12 ]), &(acadoWorkspace.Qd[ 12 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 80 ]), &(acadoWorkspace.d[ 16 ]), &(acadoWorkspace.Qd[ 16 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 96 ]), &(acadoWorkspace.d[ 20 ]), &(acadoWorkspace.Qd[ 20 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 112 ]), &(acadoWorkspace.d[ 24 ]), &(acadoWorkspace.Qd[ 24 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 128 ]), &(acadoWorkspace.d[ 28 ]), &(acadoWorkspace.Qd[ 28 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.d[ 32 ]), &(acadoWorkspace.Qd[ 32 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 160 ]), &(acadoWorkspace.d[ 36 ]), &(acadoWorkspace.Qd[ 36 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 176 ]), &(acadoWorkspace.d[ 40 ]), &(acadoWorkspace.Qd[ 40 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 192 ]), &(acadoWorkspace.d[ 44 ]), &(acadoWorkspace.Qd[ 44 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 208 ]), &(acadoWorkspace.d[ 48 ]), &(acadoWorkspace.Qd[ 48 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 224 ]), &(acadoWorkspace.d[ 52 ]), &(acadoWorkspace.Qd[ 52 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 240 ]), &(acadoWorkspace.d[ 56 ]), &(acadoWorkspace.Qd[ 56 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 256 ]), &(acadoWorkspace.d[ 60 ]), &(acadoWorkspace.Qd[ 60 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 272 ]), &(acadoWorkspace.d[ 64 ]), &(acadoWorkspace.Qd[ 64 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.d[ 68 ]), &(acadoWorkspace.Qd[ 68 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 304 ]), &(acadoWorkspace.d[ 72 ]), &(acadoWorkspace.Qd[ 72 ]) );
acado_multQN1d( acadoWorkspace.QN1, &(acadoWorkspace.d[ 76 ]), &(acadoWorkspace.Qd[ 76 ]) );

acado_macETSlu( acadoWorkspace.QE, acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 8 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 24 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 48 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 80 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 120 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 168 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 224 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 288 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 360 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 440 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 528 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 624 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 728 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 840 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 960 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 1088 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 1224 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 1368 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 1520 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 16 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 32 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 56 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 88 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 128 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 176 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 232 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 296 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 368 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 448 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 536 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 632 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 736 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 848 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 968 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1096 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1232 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1376 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1528 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 40 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 64 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 96 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 136 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 184 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 240 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 304 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 376 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 456 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 544 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 640 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 744 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 856 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 976 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1104 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1240 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1384 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1536 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 72 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 104 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 144 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 192 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 248 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 312 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 384 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 464 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 552 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 648 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 752 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 864 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 984 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1112 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1248 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1392 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1544 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 112 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 152 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 200 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 256 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 320 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 392 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 472 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 560 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 656 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 760 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 872 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 992 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1120 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1256 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1400 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1552 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 160 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 208 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 264 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 328 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 400 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 480 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 568 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 664 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 768 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 880 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1000 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1128 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1264 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1408 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1560 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 216 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 272 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 336 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 408 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 488 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 576 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 672 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 776 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 888 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1008 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1136 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1272 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1416 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1568 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 280 ]), &(acadoWorkspace.g[ 14 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 344 ]), &(acadoWorkspace.g[ 14 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 416 ]), &(acadoWorkspace.g[ 14 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 496 ]), &(acadoWorkspace.g[ 14 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 584 ]), &(acadoWorkspace.g[ 14 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 680 ]), &(acadoWorkspace.g[ 14 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 784 ]), &(acadoWorkspace.g[ 14 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 896 ]), &(acadoWorkspace.g[ 14 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1016 ]), &(acadoWorkspace.g[ 14 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1144 ]), &(acadoWorkspace.g[ 14 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1280 ]), &(acadoWorkspace.g[ 14 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1424 ]), &(acadoWorkspace.g[ 14 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1576 ]), &(acadoWorkspace.g[ 14 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 352 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 424 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 504 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 592 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 688 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 792 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 904 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1024 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1152 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1288 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1432 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1584 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 432 ]), &(acadoWorkspace.g[ 18 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 512 ]), &(acadoWorkspace.g[ 18 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 600 ]), &(acadoWorkspace.g[ 18 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 696 ]), &(acadoWorkspace.g[ 18 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 800 ]), &(acadoWorkspace.g[ 18 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 912 ]), &(acadoWorkspace.g[ 18 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1032 ]), &(acadoWorkspace.g[ 18 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1160 ]), &(acadoWorkspace.g[ 18 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1296 ]), &(acadoWorkspace.g[ 18 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1440 ]), &(acadoWorkspace.g[ 18 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1592 ]), &(acadoWorkspace.g[ 18 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 520 ]), &(acadoWorkspace.g[ 20 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 608 ]), &(acadoWorkspace.g[ 20 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 704 ]), &(acadoWorkspace.g[ 20 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 808 ]), &(acadoWorkspace.g[ 20 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 920 ]), &(acadoWorkspace.g[ 20 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1040 ]), &(acadoWorkspace.g[ 20 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1168 ]), &(acadoWorkspace.g[ 20 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1304 ]), &(acadoWorkspace.g[ 20 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1448 ]), &(acadoWorkspace.g[ 20 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1600 ]), &(acadoWorkspace.g[ 20 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 616 ]), &(acadoWorkspace.g[ 22 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 712 ]), &(acadoWorkspace.g[ 22 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 816 ]), &(acadoWorkspace.g[ 22 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 928 ]), &(acadoWorkspace.g[ 22 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1048 ]), &(acadoWorkspace.g[ 22 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1176 ]), &(acadoWorkspace.g[ 22 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1312 ]), &(acadoWorkspace.g[ 22 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1456 ]), &(acadoWorkspace.g[ 22 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1608 ]), &(acadoWorkspace.g[ 22 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 720 ]), &(acadoWorkspace.g[ 24 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 824 ]), &(acadoWorkspace.g[ 24 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 936 ]), &(acadoWorkspace.g[ 24 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1056 ]), &(acadoWorkspace.g[ 24 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1184 ]), &(acadoWorkspace.g[ 24 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1320 ]), &(acadoWorkspace.g[ 24 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1464 ]), &(acadoWorkspace.g[ 24 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1616 ]), &(acadoWorkspace.g[ 24 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 832 ]), &(acadoWorkspace.g[ 26 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 944 ]), &(acadoWorkspace.g[ 26 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1064 ]), &(acadoWorkspace.g[ 26 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1192 ]), &(acadoWorkspace.g[ 26 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1328 ]), &(acadoWorkspace.g[ 26 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1472 ]), &(acadoWorkspace.g[ 26 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1624 ]), &(acadoWorkspace.g[ 26 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 952 ]), &(acadoWorkspace.g[ 28 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1072 ]), &(acadoWorkspace.g[ 28 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1200 ]), &(acadoWorkspace.g[ 28 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1336 ]), &(acadoWorkspace.g[ 28 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1480 ]), &(acadoWorkspace.g[ 28 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1632 ]), &(acadoWorkspace.g[ 28 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1080 ]), &(acadoWorkspace.g[ 30 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1208 ]), &(acadoWorkspace.g[ 30 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1344 ]), &(acadoWorkspace.g[ 30 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1488 ]), &(acadoWorkspace.g[ 30 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1640 ]), &(acadoWorkspace.g[ 30 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1216 ]), &(acadoWorkspace.g[ 32 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1352 ]), &(acadoWorkspace.g[ 32 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1496 ]), &(acadoWorkspace.g[ 32 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1648 ]), &(acadoWorkspace.g[ 32 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1360 ]), &(acadoWorkspace.g[ 34 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1504 ]), &(acadoWorkspace.g[ 34 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1656 ]), &(acadoWorkspace.g[ 34 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1512 ]), &(acadoWorkspace.g[ 36 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1664 ]), &(acadoWorkspace.g[ 36 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1672 ]), &(acadoWorkspace.g[ 38 ]) );
acadoWorkspace.lb[0] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[0];
acadoWorkspace.lb[1] = (real_t)-9.0000000000000002e-01 - acadoVariables.u[1];
acadoWorkspace.lb[2] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[2];
acadoWorkspace.lb[3] = (real_t)-9.0000000000000002e-01 - acadoVariables.u[3];
acadoWorkspace.lb[4] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[4];
acadoWorkspace.lb[5] = (real_t)-9.0000000000000002e-01 - acadoVariables.u[5];
acadoWorkspace.lb[6] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[6];
acadoWorkspace.lb[7] = (real_t)-9.0000000000000002e-01 - acadoVariables.u[7];
acadoWorkspace.lb[8] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[8];
acadoWorkspace.lb[9] = (real_t)-9.0000000000000002e-01 - acadoVariables.u[9];
acadoWorkspace.lb[10] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[10];
acadoWorkspace.lb[11] = (real_t)-9.0000000000000002e-01 - acadoVariables.u[11];
acadoWorkspace.lb[12] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[12];
acadoWorkspace.lb[13] = (real_t)-9.0000000000000002e-01 - acadoVariables.u[13];
acadoWorkspace.lb[14] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[14];
acadoWorkspace.lb[15] = (real_t)-9.0000000000000002e-01 - acadoVariables.u[15];
acadoWorkspace.lb[16] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[16];
acadoWorkspace.lb[17] = (real_t)-9.0000000000000002e-01 - acadoVariables.u[17];
acadoWorkspace.lb[18] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[18];
acadoWorkspace.lb[19] = (real_t)-9.0000000000000002e-01 - acadoVariables.u[19];
acadoWorkspace.lb[20] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[20];
acadoWorkspace.lb[21] = (real_t)-9.0000000000000002e-01 - acadoVariables.u[21];
acadoWorkspace.lb[22] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[22];
acadoWorkspace.lb[23] = (real_t)-9.0000000000000002e-01 - acadoVariables.u[23];
acadoWorkspace.lb[24] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[24];
acadoWorkspace.lb[25] = (real_t)-9.0000000000000002e-01 - acadoVariables.u[25];
acadoWorkspace.lb[26] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[26];
acadoWorkspace.lb[27] = (real_t)-9.0000000000000002e-01 - acadoVariables.u[27];
acadoWorkspace.lb[28] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[28];
acadoWorkspace.lb[29] = (real_t)-9.0000000000000002e-01 - acadoVariables.u[29];
acadoWorkspace.lb[30] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[30];
acadoWorkspace.lb[31] = (real_t)-9.0000000000000002e-01 - acadoVariables.u[31];
acadoWorkspace.lb[32] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[32];
acadoWorkspace.lb[33] = (real_t)-9.0000000000000002e-01 - acadoVariables.u[33];
acadoWorkspace.lb[34] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[34];
acadoWorkspace.lb[35] = (real_t)-9.0000000000000002e-01 - acadoVariables.u[35];
acadoWorkspace.lb[36] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[36];
acadoWorkspace.lb[37] = (real_t)-9.0000000000000002e-01 - acadoVariables.u[37];
acadoWorkspace.lb[38] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[38];
acadoWorkspace.lb[39] = (real_t)-9.0000000000000002e-01 - acadoVariables.u[39];
acadoWorkspace.ub[0] = (real_t)5.0000000000000000e-01 - acadoVariables.u[0];
acadoWorkspace.ub[1] = (real_t)9.0000000000000002e-01 - acadoVariables.u[1];
acadoWorkspace.ub[2] = (real_t)5.0000000000000000e-01 - acadoVariables.u[2];
acadoWorkspace.ub[3] = (real_t)9.0000000000000002e-01 - acadoVariables.u[3];
acadoWorkspace.ub[4] = (real_t)5.0000000000000000e-01 - acadoVariables.u[4];
acadoWorkspace.ub[5] = (real_t)9.0000000000000002e-01 - acadoVariables.u[5];
acadoWorkspace.ub[6] = (real_t)5.0000000000000000e-01 - acadoVariables.u[6];
acadoWorkspace.ub[7] = (real_t)9.0000000000000002e-01 - acadoVariables.u[7];
acadoWorkspace.ub[8] = (real_t)5.0000000000000000e-01 - acadoVariables.u[8];
acadoWorkspace.ub[9] = (real_t)9.0000000000000002e-01 - acadoVariables.u[9];
acadoWorkspace.ub[10] = (real_t)5.0000000000000000e-01 - acadoVariables.u[10];
acadoWorkspace.ub[11] = (real_t)9.0000000000000002e-01 - acadoVariables.u[11];
acadoWorkspace.ub[12] = (real_t)5.0000000000000000e-01 - acadoVariables.u[12];
acadoWorkspace.ub[13] = (real_t)9.0000000000000002e-01 - acadoVariables.u[13];
acadoWorkspace.ub[14] = (real_t)5.0000000000000000e-01 - acadoVariables.u[14];
acadoWorkspace.ub[15] = (real_t)9.0000000000000002e-01 - acadoVariables.u[15];
acadoWorkspace.ub[16] = (real_t)5.0000000000000000e-01 - acadoVariables.u[16];
acadoWorkspace.ub[17] = (real_t)9.0000000000000002e-01 - acadoVariables.u[17];
acadoWorkspace.ub[18] = (real_t)5.0000000000000000e-01 - acadoVariables.u[18];
acadoWorkspace.ub[19] = (real_t)9.0000000000000002e-01 - acadoVariables.u[19];
acadoWorkspace.ub[20] = (real_t)5.0000000000000000e-01 - acadoVariables.u[20];
acadoWorkspace.ub[21] = (real_t)9.0000000000000002e-01 - acadoVariables.u[21];
acadoWorkspace.ub[22] = (real_t)5.0000000000000000e-01 - acadoVariables.u[22];
acadoWorkspace.ub[23] = (real_t)9.0000000000000002e-01 - acadoVariables.u[23];
acadoWorkspace.ub[24] = (real_t)5.0000000000000000e-01 - acadoVariables.u[24];
acadoWorkspace.ub[25] = (real_t)9.0000000000000002e-01 - acadoVariables.u[25];
acadoWorkspace.ub[26] = (real_t)5.0000000000000000e-01 - acadoVariables.u[26];
acadoWorkspace.ub[27] = (real_t)9.0000000000000002e-01 - acadoVariables.u[27];
acadoWorkspace.ub[28] = (real_t)5.0000000000000000e-01 - acadoVariables.u[28];
acadoWorkspace.ub[29] = (real_t)9.0000000000000002e-01 - acadoVariables.u[29];
acadoWorkspace.ub[30] = (real_t)5.0000000000000000e-01 - acadoVariables.u[30];
acadoWorkspace.ub[31] = (real_t)9.0000000000000002e-01 - acadoVariables.u[31];
acadoWorkspace.ub[32] = (real_t)5.0000000000000000e-01 - acadoVariables.u[32];
acadoWorkspace.ub[33] = (real_t)9.0000000000000002e-01 - acadoVariables.u[33];
acadoWorkspace.ub[34] = (real_t)5.0000000000000000e-01 - acadoVariables.u[34];
acadoWorkspace.ub[35] = (real_t)9.0000000000000002e-01 - acadoVariables.u[35];
acadoWorkspace.ub[36] = (real_t)5.0000000000000000e-01 - acadoVariables.u[36];
acadoWorkspace.ub[37] = (real_t)9.0000000000000002e-01 - acadoVariables.u[37];
acadoWorkspace.ub[38] = (real_t)5.0000000000000000e-01 - acadoVariables.u[38];
acadoWorkspace.ub[39] = (real_t)9.0000000000000002e-01 - acadoVariables.u[39];

}

void acado_condenseFdb(  )
{
int lRun1;
acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
acadoWorkspace.Dx0[1] = acadoVariables.x0[1] - acadoVariables.x[1];
acadoWorkspace.Dx0[2] = acadoVariables.x0[2] - acadoVariables.x[2];
acadoWorkspace.Dx0[3] = acadoVariables.x0[3] - acadoVariables.x[3];

for (lRun1 = 0; lRun1 < 140; ++lRun1)
acadoWorkspace.Dy[lRun1] -= acadoVariables.y[lRun1];

acadoWorkspace.DyN[0] -= acadoVariables.yN[0];
acadoWorkspace.DyN[1] -= acadoVariables.yN[1];
acadoWorkspace.DyN[2] -= acadoVariables.yN[2];
acadoWorkspace.DyN[3] -= acadoVariables.yN[3];

acado_multRDy( acadoWorkspace.R2, acadoWorkspace.Dy, acadoWorkspace.g );
acado_multRDy( &(acadoWorkspace.R2[ 14 ]), &(acadoWorkspace.Dy[ 7 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 28 ]), &(acadoWorkspace.Dy[ 14 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 42 ]), &(acadoWorkspace.Dy[ 21 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 56 ]), &(acadoWorkspace.Dy[ 28 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 70 ]), &(acadoWorkspace.Dy[ 35 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 84 ]), &(acadoWorkspace.Dy[ 42 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 98 ]), &(acadoWorkspace.Dy[ 49 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 112 ]), &(acadoWorkspace.Dy[ 56 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 126 ]), &(acadoWorkspace.Dy[ 63 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 140 ]), &(acadoWorkspace.Dy[ 70 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 154 ]), &(acadoWorkspace.Dy[ 77 ]), &(acadoWorkspace.g[ 22 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 168 ]), &(acadoWorkspace.Dy[ 84 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 182 ]), &(acadoWorkspace.Dy[ 91 ]), &(acadoWorkspace.g[ 26 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 196 ]), &(acadoWorkspace.Dy[ 98 ]), &(acadoWorkspace.g[ 28 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 210 ]), &(acadoWorkspace.Dy[ 105 ]), &(acadoWorkspace.g[ 30 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 224 ]), &(acadoWorkspace.Dy[ 112 ]), &(acadoWorkspace.g[ 32 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 238 ]), &(acadoWorkspace.Dy[ 119 ]), &(acadoWorkspace.g[ 34 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 252 ]), &(acadoWorkspace.Dy[ 126 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 266 ]), &(acadoWorkspace.Dy[ 133 ]), &(acadoWorkspace.g[ 38 ]) );

acado_multQDy( acadoWorkspace.Q2, acadoWorkspace.Dy, acadoWorkspace.QDy );
acado_multQDy( &(acadoWorkspace.Q2[ 28 ]), &(acadoWorkspace.Dy[ 7 ]), &(acadoWorkspace.QDy[ 4 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 56 ]), &(acadoWorkspace.Dy[ 14 ]), &(acadoWorkspace.QDy[ 8 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 84 ]), &(acadoWorkspace.Dy[ 21 ]), &(acadoWorkspace.QDy[ 12 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 112 ]), &(acadoWorkspace.Dy[ 28 ]), &(acadoWorkspace.QDy[ 16 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 140 ]), &(acadoWorkspace.Dy[ 35 ]), &(acadoWorkspace.QDy[ 20 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 168 ]), &(acadoWorkspace.Dy[ 42 ]), &(acadoWorkspace.QDy[ 24 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 196 ]), &(acadoWorkspace.Dy[ 49 ]), &(acadoWorkspace.QDy[ 28 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 224 ]), &(acadoWorkspace.Dy[ 56 ]), &(acadoWorkspace.QDy[ 32 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 252 ]), &(acadoWorkspace.Dy[ 63 ]), &(acadoWorkspace.QDy[ 36 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 280 ]), &(acadoWorkspace.Dy[ 70 ]), &(acadoWorkspace.QDy[ 40 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 308 ]), &(acadoWorkspace.Dy[ 77 ]), &(acadoWorkspace.QDy[ 44 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 336 ]), &(acadoWorkspace.Dy[ 84 ]), &(acadoWorkspace.QDy[ 48 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 364 ]), &(acadoWorkspace.Dy[ 91 ]), &(acadoWorkspace.QDy[ 52 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 392 ]), &(acadoWorkspace.Dy[ 98 ]), &(acadoWorkspace.QDy[ 56 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 420 ]), &(acadoWorkspace.Dy[ 105 ]), &(acadoWorkspace.QDy[ 60 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 448 ]), &(acadoWorkspace.Dy[ 112 ]), &(acadoWorkspace.QDy[ 64 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 476 ]), &(acadoWorkspace.Dy[ 119 ]), &(acadoWorkspace.QDy[ 68 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 504 ]), &(acadoWorkspace.Dy[ 126 ]), &(acadoWorkspace.QDy[ 72 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 532 ]), &(acadoWorkspace.Dy[ 133 ]), &(acadoWorkspace.QDy[ 76 ]) );

acadoWorkspace.QDy[80] = + acadoWorkspace.QN2[0]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[1]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[2]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[3]*acadoWorkspace.DyN[3];
acadoWorkspace.QDy[81] = + acadoWorkspace.QN2[4]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[5]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[6]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[7]*acadoWorkspace.DyN[3];
acadoWorkspace.QDy[82] = + acadoWorkspace.QN2[8]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[9]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[10]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[11]*acadoWorkspace.DyN[3];
acadoWorkspace.QDy[83] = + acadoWorkspace.QN2[12]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[13]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[14]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[15]*acadoWorkspace.DyN[3];

acadoWorkspace.QDy[4] += acadoWorkspace.Qd[0];
acadoWorkspace.QDy[5] += acadoWorkspace.Qd[1];
acadoWorkspace.QDy[6] += acadoWorkspace.Qd[2];
acadoWorkspace.QDy[7] += acadoWorkspace.Qd[3];
acadoWorkspace.QDy[8] += acadoWorkspace.Qd[4];
acadoWorkspace.QDy[9] += acadoWorkspace.Qd[5];
acadoWorkspace.QDy[10] += acadoWorkspace.Qd[6];
acadoWorkspace.QDy[11] += acadoWorkspace.Qd[7];
acadoWorkspace.QDy[12] += acadoWorkspace.Qd[8];
acadoWorkspace.QDy[13] += acadoWorkspace.Qd[9];
acadoWorkspace.QDy[14] += acadoWorkspace.Qd[10];
acadoWorkspace.QDy[15] += acadoWorkspace.Qd[11];
acadoWorkspace.QDy[16] += acadoWorkspace.Qd[12];
acadoWorkspace.QDy[17] += acadoWorkspace.Qd[13];
acadoWorkspace.QDy[18] += acadoWorkspace.Qd[14];
acadoWorkspace.QDy[19] += acadoWorkspace.Qd[15];
acadoWorkspace.QDy[20] += acadoWorkspace.Qd[16];
acadoWorkspace.QDy[21] += acadoWorkspace.Qd[17];
acadoWorkspace.QDy[22] += acadoWorkspace.Qd[18];
acadoWorkspace.QDy[23] += acadoWorkspace.Qd[19];
acadoWorkspace.QDy[24] += acadoWorkspace.Qd[20];
acadoWorkspace.QDy[25] += acadoWorkspace.Qd[21];
acadoWorkspace.QDy[26] += acadoWorkspace.Qd[22];
acadoWorkspace.QDy[27] += acadoWorkspace.Qd[23];
acadoWorkspace.QDy[28] += acadoWorkspace.Qd[24];
acadoWorkspace.QDy[29] += acadoWorkspace.Qd[25];
acadoWorkspace.QDy[30] += acadoWorkspace.Qd[26];
acadoWorkspace.QDy[31] += acadoWorkspace.Qd[27];
acadoWorkspace.QDy[32] += acadoWorkspace.Qd[28];
acadoWorkspace.QDy[33] += acadoWorkspace.Qd[29];
acadoWorkspace.QDy[34] += acadoWorkspace.Qd[30];
acadoWorkspace.QDy[35] += acadoWorkspace.Qd[31];
acadoWorkspace.QDy[36] += acadoWorkspace.Qd[32];
acadoWorkspace.QDy[37] += acadoWorkspace.Qd[33];
acadoWorkspace.QDy[38] += acadoWorkspace.Qd[34];
acadoWorkspace.QDy[39] += acadoWorkspace.Qd[35];
acadoWorkspace.QDy[40] += acadoWorkspace.Qd[36];
acadoWorkspace.QDy[41] += acadoWorkspace.Qd[37];
acadoWorkspace.QDy[42] += acadoWorkspace.Qd[38];
acadoWorkspace.QDy[43] += acadoWorkspace.Qd[39];
acadoWorkspace.QDy[44] += acadoWorkspace.Qd[40];
acadoWorkspace.QDy[45] += acadoWorkspace.Qd[41];
acadoWorkspace.QDy[46] += acadoWorkspace.Qd[42];
acadoWorkspace.QDy[47] += acadoWorkspace.Qd[43];
acadoWorkspace.QDy[48] += acadoWorkspace.Qd[44];
acadoWorkspace.QDy[49] += acadoWorkspace.Qd[45];
acadoWorkspace.QDy[50] += acadoWorkspace.Qd[46];
acadoWorkspace.QDy[51] += acadoWorkspace.Qd[47];
acadoWorkspace.QDy[52] += acadoWorkspace.Qd[48];
acadoWorkspace.QDy[53] += acadoWorkspace.Qd[49];
acadoWorkspace.QDy[54] += acadoWorkspace.Qd[50];
acadoWorkspace.QDy[55] += acadoWorkspace.Qd[51];
acadoWorkspace.QDy[56] += acadoWorkspace.Qd[52];
acadoWorkspace.QDy[57] += acadoWorkspace.Qd[53];
acadoWorkspace.QDy[58] += acadoWorkspace.Qd[54];
acadoWorkspace.QDy[59] += acadoWorkspace.Qd[55];
acadoWorkspace.QDy[60] += acadoWorkspace.Qd[56];
acadoWorkspace.QDy[61] += acadoWorkspace.Qd[57];
acadoWorkspace.QDy[62] += acadoWorkspace.Qd[58];
acadoWorkspace.QDy[63] += acadoWorkspace.Qd[59];
acadoWorkspace.QDy[64] += acadoWorkspace.Qd[60];
acadoWorkspace.QDy[65] += acadoWorkspace.Qd[61];
acadoWorkspace.QDy[66] += acadoWorkspace.Qd[62];
acadoWorkspace.QDy[67] += acadoWorkspace.Qd[63];
acadoWorkspace.QDy[68] += acadoWorkspace.Qd[64];
acadoWorkspace.QDy[69] += acadoWorkspace.Qd[65];
acadoWorkspace.QDy[70] += acadoWorkspace.Qd[66];
acadoWorkspace.QDy[71] += acadoWorkspace.Qd[67];
acadoWorkspace.QDy[72] += acadoWorkspace.Qd[68];
acadoWorkspace.QDy[73] += acadoWorkspace.Qd[69];
acadoWorkspace.QDy[74] += acadoWorkspace.Qd[70];
acadoWorkspace.QDy[75] += acadoWorkspace.Qd[71];
acadoWorkspace.QDy[76] += acadoWorkspace.Qd[72];
acadoWorkspace.QDy[77] += acadoWorkspace.Qd[73];
acadoWorkspace.QDy[78] += acadoWorkspace.Qd[74];
acadoWorkspace.QDy[79] += acadoWorkspace.Qd[75];
acadoWorkspace.QDy[80] += acadoWorkspace.Qd[76];
acadoWorkspace.QDy[81] += acadoWorkspace.Qd[77];
acadoWorkspace.QDy[82] += acadoWorkspace.Qd[78];
acadoWorkspace.QDy[83] += acadoWorkspace.Qd[79];

acado_multEQDy( acadoWorkspace.E, &(acadoWorkspace.QDy[ 4 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 8 ]), &(acadoWorkspace.QDy[ 8 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 24 ]), &(acadoWorkspace.QDy[ 12 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QDy[ 16 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 80 ]), &(acadoWorkspace.QDy[ 20 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QDy[ 24 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.QDy[ 28 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 224 ]), &(acadoWorkspace.QDy[ 32 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QDy[ 36 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QDy[ 40 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 440 ]), &(acadoWorkspace.QDy[ 44 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.QDy[ 48 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.QDy[ 52 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 728 ]), &(acadoWorkspace.QDy[ 56 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 840 ]), &(acadoWorkspace.QDy[ 60 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QDy[ 64 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 1088 ]), &(acadoWorkspace.QDy[ 68 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 1224 ]), &(acadoWorkspace.QDy[ 72 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 1368 ]), &(acadoWorkspace.QDy[ 76 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 1520 ]), &(acadoWorkspace.QDy[ 80 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 16 ]), &(acadoWorkspace.QDy[ 8 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 32 ]), &(acadoWorkspace.QDy[ 12 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 56 ]), &(acadoWorkspace.QDy[ 16 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 88 ]), &(acadoWorkspace.QDy[ 20 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 128 ]), &(acadoWorkspace.QDy[ 24 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 176 ]), &(acadoWorkspace.QDy[ 28 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 232 ]), &(acadoWorkspace.QDy[ 32 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 296 ]), &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 368 ]), &(acadoWorkspace.QDy[ 40 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 448 ]), &(acadoWorkspace.QDy[ 44 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 536 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 632 ]), &(acadoWorkspace.QDy[ 52 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 736 ]), &(acadoWorkspace.QDy[ 56 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 848 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 968 ]), &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1096 ]), &(acadoWorkspace.QDy[ 68 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1232 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1376 ]), &(acadoWorkspace.QDy[ 76 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1528 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 40 ]), &(acadoWorkspace.QDy[ 12 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 64 ]), &(acadoWorkspace.QDy[ 16 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QDy[ 20 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 136 ]), &(acadoWorkspace.QDy[ 24 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 184 ]), &(acadoWorkspace.QDy[ 28 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QDy[ 32 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 304 ]), &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 376 ]), &(acadoWorkspace.QDy[ 40 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 456 ]), &(acadoWorkspace.QDy[ 44 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 544 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 640 ]), &(acadoWorkspace.QDy[ 52 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 744 ]), &(acadoWorkspace.QDy[ 56 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 856 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 976 ]), &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QDy[ 68 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1240 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1384 ]), &(acadoWorkspace.QDy[ 76 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.QDy[ 16 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 104 ]), &(acadoWorkspace.QDy[ 20 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QDy[ 24 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QDy[ 28 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 248 ]), &(acadoWorkspace.QDy[ 32 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 312 ]), &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.QDy[ 40 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 464 ]), &(acadoWorkspace.QDy[ 44 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 648 ]), &(acadoWorkspace.QDy[ 52 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 752 ]), &(acadoWorkspace.QDy[ 56 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 984 ]), &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1112 ]), &(acadoWorkspace.QDy[ 68 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1248 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1392 ]), &(acadoWorkspace.QDy[ 76 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1544 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 112 ]), &(acadoWorkspace.QDy[ 20 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 152 ]), &(acadoWorkspace.QDy[ 24 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 200 ]), &(acadoWorkspace.QDy[ 28 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 256 ]), &(acadoWorkspace.QDy[ 32 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 320 ]), &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 392 ]), &(acadoWorkspace.QDy[ 40 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 472 ]), &(acadoWorkspace.QDy[ 44 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 560 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 656 ]), &(acadoWorkspace.QDy[ 52 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 760 ]), &(acadoWorkspace.QDy[ 56 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 872 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 992 ]), &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1120 ]), &(acadoWorkspace.QDy[ 68 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1256 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1400 ]), &(acadoWorkspace.QDy[ 76 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1552 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 160 ]), &(acadoWorkspace.QDy[ 24 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 208 ]), &(acadoWorkspace.QDy[ 28 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 264 ]), &(acadoWorkspace.QDy[ 32 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 328 ]), &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 400 ]), &(acadoWorkspace.QDy[ 40 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QDy[ 44 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 568 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 664 ]), &(acadoWorkspace.QDy[ 52 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 768 ]), &(acadoWorkspace.QDy[ 56 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 880 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1000 ]), &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1128 ]), &(acadoWorkspace.QDy[ 68 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1264 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1408 ]), &(acadoWorkspace.QDy[ 76 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1560 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.QDy[ 28 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 272 ]), &(acadoWorkspace.QDy[ 32 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 408 ]), &(acadoWorkspace.QDy[ 40 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 488 ]), &(acadoWorkspace.QDy[ 44 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.QDy[ 52 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 776 ]), &(acadoWorkspace.QDy[ 56 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 888 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1008 ]), &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1136 ]), &(acadoWorkspace.QDy[ 68 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1272 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1416 ]), &(acadoWorkspace.QDy[ 76 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1568 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 280 ]), &(acadoWorkspace.QDy[ 32 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 344 ]), &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 416 ]), &(acadoWorkspace.QDy[ 40 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 496 ]), &(acadoWorkspace.QDy[ 44 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 584 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 680 ]), &(acadoWorkspace.QDy[ 52 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 784 ]), &(acadoWorkspace.QDy[ 56 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 896 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1016 ]), &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1144 ]), &(acadoWorkspace.QDy[ 68 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1280 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1424 ]), &(acadoWorkspace.QDy[ 76 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1576 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 352 ]), &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 424 ]), &(acadoWorkspace.QDy[ 40 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 504 ]), &(acadoWorkspace.QDy[ 44 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 592 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 688 ]), &(acadoWorkspace.QDy[ 52 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 792 ]), &(acadoWorkspace.QDy[ 56 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 904 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1024 ]), &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1152 ]), &(acadoWorkspace.QDy[ 68 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1288 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1432 ]), &(acadoWorkspace.QDy[ 76 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1584 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QDy[ 40 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 512 ]), &(acadoWorkspace.QDy[ 44 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 600 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 696 ]), &(acadoWorkspace.QDy[ 52 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 800 ]), &(acadoWorkspace.QDy[ 56 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 912 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1032 ]), &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1160 ]), &(acadoWorkspace.QDy[ 68 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QDy[ 76 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1592 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 520 ]), &(acadoWorkspace.QDy[ 44 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 608 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 704 ]), &(acadoWorkspace.QDy[ 52 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 808 ]), &(acadoWorkspace.QDy[ 56 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 920 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1040 ]), &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1168 ]), &(acadoWorkspace.QDy[ 68 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1304 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1448 ]), &(acadoWorkspace.QDy[ 76 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1600 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 616 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 22 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 712 ]), &(acadoWorkspace.QDy[ 52 ]), &(acadoWorkspace.g[ 22 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 816 ]), &(acadoWorkspace.QDy[ 56 ]), &(acadoWorkspace.g[ 22 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 928 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 22 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1048 ]), &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.g[ 22 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1176 ]), &(acadoWorkspace.QDy[ 68 ]), &(acadoWorkspace.g[ 22 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1312 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 22 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1456 ]), &(acadoWorkspace.QDy[ 76 ]), &(acadoWorkspace.g[ 22 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1608 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 22 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.QDy[ 52 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 824 ]), &(acadoWorkspace.QDy[ 56 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 936 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1056 ]), &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1184 ]), &(acadoWorkspace.QDy[ 68 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1320 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1464 ]), &(acadoWorkspace.QDy[ 76 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1616 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 832 ]), &(acadoWorkspace.QDy[ 56 ]), &(acadoWorkspace.g[ 26 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 944 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 26 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1064 ]), &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.g[ 26 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1192 ]), &(acadoWorkspace.QDy[ 68 ]), &(acadoWorkspace.g[ 26 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1328 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 26 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1472 ]), &(acadoWorkspace.QDy[ 76 ]), &(acadoWorkspace.g[ 26 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1624 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 26 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 952 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 28 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1072 ]), &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.g[ 28 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1200 ]), &(acadoWorkspace.QDy[ 68 ]), &(acadoWorkspace.g[ 28 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1336 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 28 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1480 ]), &(acadoWorkspace.QDy[ 76 ]), &(acadoWorkspace.g[ 28 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1632 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 28 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1080 ]), &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.g[ 30 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1208 ]), &(acadoWorkspace.QDy[ 68 ]), &(acadoWorkspace.g[ 30 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1344 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 30 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.QDy[ 76 ]), &(acadoWorkspace.g[ 30 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1640 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 30 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1216 ]), &(acadoWorkspace.QDy[ 68 ]), &(acadoWorkspace.g[ 32 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1352 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 32 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1496 ]), &(acadoWorkspace.QDy[ 76 ]), &(acadoWorkspace.g[ 32 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1648 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 32 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1360 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 34 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1504 ]), &(acadoWorkspace.QDy[ 76 ]), &(acadoWorkspace.g[ 34 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1656 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 34 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1512 ]), &(acadoWorkspace.QDy[ 76 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1664 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1672 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 38 ]) );

acadoWorkspace.g[0] += + acadoWorkspace.H10[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[2]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[3]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[1] += + acadoWorkspace.H10[4]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[5]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[6]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[7]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[2] += + acadoWorkspace.H10[8]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[9]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[10]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[11]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[3] += + acadoWorkspace.H10[12]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[13]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[14]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[15]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[4] += + acadoWorkspace.H10[16]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[17]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[18]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[19]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[5] += + acadoWorkspace.H10[20]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[21]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[22]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[23]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[6] += + acadoWorkspace.H10[24]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[25]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[26]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[27]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[7] += + acadoWorkspace.H10[28]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[29]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[30]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[31]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[8] += + acadoWorkspace.H10[32]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[33]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[34]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[35]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[9] += + acadoWorkspace.H10[36]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[37]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[38]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[39]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[10] += + acadoWorkspace.H10[40]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[41]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[42]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[43]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[11] += + acadoWorkspace.H10[44]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[45]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[46]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[47]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[12] += + acadoWorkspace.H10[48]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[49]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[50]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[51]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[13] += + acadoWorkspace.H10[52]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[53]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[54]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[55]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[14] += + acadoWorkspace.H10[56]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[57]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[58]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[59]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[15] += + acadoWorkspace.H10[60]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[61]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[62]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[63]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[16] += + acadoWorkspace.H10[64]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[65]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[66]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[67]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[17] += + acadoWorkspace.H10[68]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[69]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[70]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[71]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[18] += + acadoWorkspace.H10[72]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[73]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[74]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[75]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[19] += + acadoWorkspace.H10[76]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[77]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[78]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[79]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[20] += + acadoWorkspace.H10[80]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[81]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[82]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[83]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[21] += + acadoWorkspace.H10[84]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[85]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[86]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[87]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[22] += + acadoWorkspace.H10[88]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[89]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[90]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[91]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[23] += + acadoWorkspace.H10[92]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[93]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[94]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[95]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[24] += + acadoWorkspace.H10[96]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[97]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[98]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[99]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[25] += + acadoWorkspace.H10[100]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[101]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[102]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[103]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[26] += + acadoWorkspace.H10[104]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[105]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[106]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[107]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[27] += + acadoWorkspace.H10[108]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[109]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[110]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[111]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[28] += + acadoWorkspace.H10[112]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[113]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[114]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[115]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[29] += + acadoWorkspace.H10[116]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[117]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[118]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[119]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[30] += + acadoWorkspace.H10[120]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[121]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[122]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[123]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[31] += + acadoWorkspace.H10[124]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[125]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[126]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[127]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[32] += + acadoWorkspace.H10[128]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[129]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[130]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[131]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[33] += + acadoWorkspace.H10[132]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[133]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[134]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[135]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[34] += + acadoWorkspace.H10[136]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[137]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[138]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[139]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[35] += + acadoWorkspace.H10[140]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[141]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[142]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[143]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[36] += + acadoWorkspace.H10[144]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[145]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[146]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[147]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[37] += + acadoWorkspace.H10[148]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[149]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[150]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[151]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[38] += + acadoWorkspace.H10[152]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[153]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[154]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[155]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[39] += + acadoWorkspace.H10[156]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[157]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[158]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[159]*acadoWorkspace.Dx0[3];

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
acadoVariables.u[20] += acadoWorkspace.x[20];
acadoVariables.u[21] += acadoWorkspace.x[21];
acadoVariables.u[22] += acadoWorkspace.x[22];
acadoVariables.u[23] += acadoWorkspace.x[23];
acadoVariables.u[24] += acadoWorkspace.x[24];
acadoVariables.u[25] += acadoWorkspace.x[25];
acadoVariables.u[26] += acadoWorkspace.x[26];
acadoVariables.u[27] += acadoWorkspace.x[27];
acadoVariables.u[28] += acadoWorkspace.x[28];
acadoVariables.u[29] += acadoWorkspace.x[29];
acadoVariables.u[30] += acadoWorkspace.x[30];
acadoVariables.u[31] += acadoWorkspace.x[31];
acadoVariables.u[32] += acadoWorkspace.x[32];
acadoVariables.u[33] += acadoWorkspace.x[33];
acadoVariables.u[34] += acadoWorkspace.x[34];
acadoVariables.u[35] += acadoWorkspace.x[35];
acadoVariables.u[36] += acadoWorkspace.x[36];
acadoVariables.u[37] += acadoWorkspace.x[37];
acadoVariables.u[38] += acadoWorkspace.x[38];
acadoVariables.u[39] += acadoWorkspace.x[39];

acadoVariables.x[0] += acadoWorkspace.Dx0[0];
acadoVariables.x[1] += acadoWorkspace.Dx0[1];
acadoVariables.x[2] += acadoWorkspace.Dx0[2];
acadoVariables.x[3] += acadoWorkspace.Dx0[3];

acadoVariables.x[4] += + acadoWorkspace.evGx[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[2]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[3]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[0];
acadoVariables.x[5] += + acadoWorkspace.evGx[4]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[5]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[6]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[7]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[1];
acadoVariables.x[6] += + acadoWorkspace.evGx[8]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[9]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[10]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[11]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[2];
acadoVariables.x[7] += + acadoWorkspace.evGx[12]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[13]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[14]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[15]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[3];
acadoVariables.x[8] += + acadoWorkspace.evGx[16]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[17]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[18]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[19]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[4];
acadoVariables.x[9] += + acadoWorkspace.evGx[20]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[21]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[22]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[23]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[5];
acadoVariables.x[10] += + acadoWorkspace.evGx[24]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[25]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[26]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[27]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[6];
acadoVariables.x[11] += + acadoWorkspace.evGx[28]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[29]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[30]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[31]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[7];
acadoVariables.x[12] += + acadoWorkspace.evGx[32]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[33]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[34]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[35]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[8];
acadoVariables.x[13] += + acadoWorkspace.evGx[36]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[37]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[38]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[39]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[9];
acadoVariables.x[14] += + acadoWorkspace.evGx[40]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[41]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[42]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[43]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[10];
acadoVariables.x[15] += + acadoWorkspace.evGx[44]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[45]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[46]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[47]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[11];
acadoVariables.x[16] += + acadoWorkspace.evGx[48]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[49]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[50]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[51]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[12];
acadoVariables.x[17] += + acadoWorkspace.evGx[52]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[53]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[54]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[55]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[13];
acadoVariables.x[18] += + acadoWorkspace.evGx[56]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[57]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[58]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[59]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[14];
acadoVariables.x[19] += + acadoWorkspace.evGx[60]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[61]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[62]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[63]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[15];
acadoVariables.x[20] += + acadoWorkspace.evGx[64]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[65]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[66]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[67]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[16];
acadoVariables.x[21] += + acadoWorkspace.evGx[68]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[69]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[70]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[71]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[17];
acadoVariables.x[22] += + acadoWorkspace.evGx[72]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[73]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[74]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[75]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[18];
acadoVariables.x[23] += + acadoWorkspace.evGx[76]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[77]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[78]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[79]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[19];
acadoVariables.x[24] += + acadoWorkspace.evGx[80]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[81]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[82]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[83]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[20];
acadoVariables.x[25] += + acadoWorkspace.evGx[84]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[85]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[86]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[87]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[21];
acadoVariables.x[26] += + acadoWorkspace.evGx[88]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[89]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[90]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[91]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[22];
acadoVariables.x[27] += + acadoWorkspace.evGx[92]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[93]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[94]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[95]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[23];
acadoVariables.x[28] += + acadoWorkspace.evGx[96]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[97]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[98]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[99]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[24];
acadoVariables.x[29] += + acadoWorkspace.evGx[100]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[101]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[102]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[103]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[25];
acadoVariables.x[30] += + acadoWorkspace.evGx[104]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[105]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[106]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[107]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[26];
acadoVariables.x[31] += + acadoWorkspace.evGx[108]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[109]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[110]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[111]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[27];
acadoVariables.x[32] += + acadoWorkspace.evGx[112]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[113]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[114]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[115]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[28];
acadoVariables.x[33] += + acadoWorkspace.evGx[116]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[117]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[118]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[119]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[29];
acadoVariables.x[34] += + acadoWorkspace.evGx[120]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[121]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[122]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[123]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[30];
acadoVariables.x[35] += + acadoWorkspace.evGx[124]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[125]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[126]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[127]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[31];
acadoVariables.x[36] += + acadoWorkspace.evGx[128]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[129]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[130]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[131]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[32];
acadoVariables.x[37] += + acadoWorkspace.evGx[132]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[133]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[134]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[135]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[33];
acadoVariables.x[38] += + acadoWorkspace.evGx[136]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[137]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[138]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[139]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[34];
acadoVariables.x[39] += + acadoWorkspace.evGx[140]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[141]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[142]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[143]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[35];
acadoVariables.x[40] += + acadoWorkspace.evGx[144]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[145]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[146]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[147]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[36];
acadoVariables.x[41] += + acadoWorkspace.evGx[148]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[149]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[150]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[151]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[37];
acadoVariables.x[42] += + acadoWorkspace.evGx[152]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[153]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[154]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[155]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[38];
acadoVariables.x[43] += + acadoWorkspace.evGx[156]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[157]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[158]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[159]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[39];
acadoVariables.x[44] += + acadoWorkspace.evGx[160]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[161]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[162]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[163]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[40];
acadoVariables.x[45] += + acadoWorkspace.evGx[164]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[165]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[166]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[167]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[41];
acadoVariables.x[46] += + acadoWorkspace.evGx[168]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[169]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[170]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[171]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[42];
acadoVariables.x[47] += + acadoWorkspace.evGx[172]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[173]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[174]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[175]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[43];
acadoVariables.x[48] += + acadoWorkspace.evGx[176]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[177]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[178]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[179]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[44];
acadoVariables.x[49] += + acadoWorkspace.evGx[180]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[181]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[182]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[183]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[45];
acadoVariables.x[50] += + acadoWorkspace.evGx[184]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[185]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[186]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[187]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[46];
acadoVariables.x[51] += + acadoWorkspace.evGx[188]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[189]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[190]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[191]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[47];
acadoVariables.x[52] += + acadoWorkspace.evGx[192]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[193]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[194]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[195]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[48];
acadoVariables.x[53] += + acadoWorkspace.evGx[196]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[197]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[198]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[199]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[49];
acadoVariables.x[54] += + acadoWorkspace.evGx[200]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[201]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[202]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[203]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[50];
acadoVariables.x[55] += + acadoWorkspace.evGx[204]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[205]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[206]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[207]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[51];
acadoVariables.x[56] += + acadoWorkspace.evGx[208]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[209]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[210]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[211]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[52];
acadoVariables.x[57] += + acadoWorkspace.evGx[212]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[213]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[214]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[215]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[53];
acadoVariables.x[58] += + acadoWorkspace.evGx[216]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[217]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[218]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[219]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[54];
acadoVariables.x[59] += + acadoWorkspace.evGx[220]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[221]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[222]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[223]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[55];
acadoVariables.x[60] += + acadoWorkspace.evGx[224]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[225]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[226]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[227]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[56];
acadoVariables.x[61] += + acadoWorkspace.evGx[228]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[229]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[230]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[231]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[57];
acadoVariables.x[62] += + acadoWorkspace.evGx[232]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[233]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[234]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[235]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[58];
acadoVariables.x[63] += + acadoWorkspace.evGx[236]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[237]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[238]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[239]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[59];
acadoVariables.x[64] += + acadoWorkspace.evGx[240]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[241]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[242]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[243]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[60];
acadoVariables.x[65] += + acadoWorkspace.evGx[244]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[245]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[246]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[247]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[61];
acadoVariables.x[66] += + acadoWorkspace.evGx[248]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[249]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[250]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[251]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[62];
acadoVariables.x[67] += + acadoWorkspace.evGx[252]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[253]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[254]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[255]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[63];
acadoVariables.x[68] += + acadoWorkspace.evGx[256]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[257]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[258]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[259]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[64];
acadoVariables.x[69] += + acadoWorkspace.evGx[260]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[261]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[262]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[263]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[65];
acadoVariables.x[70] += + acadoWorkspace.evGx[264]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[265]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[266]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[267]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[66];
acadoVariables.x[71] += + acadoWorkspace.evGx[268]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[269]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[270]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[271]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[67];
acadoVariables.x[72] += + acadoWorkspace.evGx[272]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[273]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[274]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[275]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[68];
acadoVariables.x[73] += + acadoWorkspace.evGx[276]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[277]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[278]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[279]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[69];
acadoVariables.x[74] += + acadoWorkspace.evGx[280]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[281]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[282]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[283]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[70];
acadoVariables.x[75] += + acadoWorkspace.evGx[284]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[285]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[286]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[287]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[71];
acadoVariables.x[76] += + acadoWorkspace.evGx[288]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[289]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[290]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[291]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[72];
acadoVariables.x[77] += + acadoWorkspace.evGx[292]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[293]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[294]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[295]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[73];
acadoVariables.x[78] += + acadoWorkspace.evGx[296]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[297]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[298]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[299]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[74];
acadoVariables.x[79] += + acadoWorkspace.evGx[300]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[301]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[302]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[303]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[75];
acadoVariables.x[80] += + acadoWorkspace.evGx[304]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[305]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[306]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[307]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[76];
acadoVariables.x[81] += + acadoWorkspace.evGx[308]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[309]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[310]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[311]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[77];
acadoVariables.x[82] += + acadoWorkspace.evGx[312]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[313]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[314]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[315]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[78];
acadoVariables.x[83] += + acadoWorkspace.evGx[316]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[317]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[318]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[319]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[79];

acado_multEDu( acadoWorkspace.E, acadoWorkspace.x, &(acadoVariables.x[ 4 ]) );
acado_multEDu( &(acadoWorkspace.E[ 8 ]), acadoWorkspace.x, &(acadoVariables.x[ 8 ]) );
acado_multEDu( &(acadoWorkspace.E[ 16 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 8 ]) );
acado_multEDu( &(acadoWorkspace.E[ 24 ]), acadoWorkspace.x, &(acadoVariables.x[ 12 ]) );
acado_multEDu( &(acadoWorkspace.E[ 32 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 12 ]) );
acado_multEDu( &(acadoWorkspace.E[ 40 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 12 ]) );
acado_multEDu( &(acadoWorkspace.E[ 48 ]), acadoWorkspace.x, &(acadoVariables.x[ 16 ]) );
acado_multEDu( &(acadoWorkspace.E[ 56 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 16 ]) );
acado_multEDu( &(acadoWorkspace.E[ 64 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 16 ]) );
acado_multEDu( &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 16 ]) );
acado_multEDu( &(acadoWorkspace.E[ 80 ]), acadoWorkspace.x, &(acadoVariables.x[ 20 ]) );
acado_multEDu( &(acadoWorkspace.E[ 88 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 20 ]) );
acado_multEDu( &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 20 ]) );
acado_multEDu( &(acadoWorkspace.E[ 104 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 20 ]) );
acado_multEDu( &(acadoWorkspace.E[ 112 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 20 ]) );
acado_multEDu( &(acadoWorkspace.E[ 120 ]), acadoWorkspace.x, &(acadoVariables.x[ 24 ]) );
acado_multEDu( &(acadoWorkspace.E[ 128 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 24 ]) );
acado_multEDu( &(acadoWorkspace.E[ 136 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 24 ]) );
acado_multEDu( &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 24 ]) );
acado_multEDu( &(acadoWorkspace.E[ 152 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 24 ]) );
acado_multEDu( &(acadoWorkspace.E[ 160 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 24 ]) );
acado_multEDu( &(acadoWorkspace.E[ 168 ]), acadoWorkspace.x, &(acadoVariables.x[ 28 ]) );
acado_multEDu( &(acadoWorkspace.E[ 176 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 28 ]) );
acado_multEDu( &(acadoWorkspace.E[ 184 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 28 ]) );
acado_multEDu( &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 28 ]) );
acado_multEDu( &(acadoWorkspace.E[ 200 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 28 ]) );
acado_multEDu( &(acadoWorkspace.E[ 208 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 28 ]) );
acado_multEDu( &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 28 ]) );
acado_multEDu( &(acadoWorkspace.E[ 224 ]), acadoWorkspace.x, &(acadoVariables.x[ 32 ]) );
acado_multEDu( &(acadoWorkspace.E[ 232 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 32 ]) );
acado_multEDu( &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 32 ]) );
acado_multEDu( &(acadoWorkspace.E[ 248 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 32 ]) );
acado_multEDu( &(acadoWorkspace.E[ 256 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 32 ]) );
acado_multEDu( &(acadoWorkspace.E[ 264 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 32 ]) );
acado_multEDu( &(acadoWorkspace.E[ 272 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 32 ]) );
acado_multEDu( &(acadoWorkspace.E[ 280 ]), &(acadoWorkspace.x[ 14 ]), &(acadoVariables.x[ 32 ]) );
acado_multEDu( &(acadoWorkspace.E[ 288 ]), acadoWorkspace.x, &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 296 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 304 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 312 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 320 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 328 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 344 ]), &(acadoWorkspace.x[ 14 ]), &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 352 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 360 ]), acadoWorkspace.x, &(acadoVariables.x[ 40 ]) );
acado_multEDu( &(acadoWorkspace.E[ 368 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 40 ]) );
acado_multEDu( &(acadoWorkspace.E[ 376 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 40 ]) );
acado_multEDu( &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 40 ]) );
acado_multEDu( &(acadoWorkspace.E[ 392 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 40 ]) );
acado_multEDu( &(acadoWorkspace.E[ 400 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 40 ]) );
acado_multEDu( &(acadoWorkspace.E[ 408 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 40 ]) );
acado_multEDu( &(acadoWorkspace.E[ 416 ]), &(acadoWorkspace.x[ 14 ]), &(acadoVariables.x[ 40 ]) );
acado_multEDu( &(acadoWorkspace.E[ 424 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 40 ]) );
acado_multEDu( &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.x[ 18 ]), &(acadoVariables.x[ 40 ]) );
acado_multEDu( &(acadoWorkspace.E[ 440 ]), acadoWorkspace.x, &(acadoVariables.x[ 44 ]) );
acado_multEDu( &(acadoWorkspace.E[ 448 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 44 ]) );
acado_multEDu( &(acadoWorkspace.E[ 456 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 44 ]) );
acado_multEDu( &(acadoWorkspace.E[ 464 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 44 ]) );
acado_multEDu( &(acadoWorkspace.E[ 472 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 44 ]) );
acado_multEDu( &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 44 ]) );
acado_multEDu( &(acadoWorkspace.E[ 488 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 44 ]) );
acado_multEDu( &(acadoWorkspace.E[ 496 ]), &(acadoWorkspace.x[ 14 ]), &(acadoVariables.x[ 44 ]) );
acado_multEDu( &(acadoWorkspace.E[ 504 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 44 ]) );
acado_multEDu( &(acadoWorkspace.E[ 512 ]), &(acadoWorkspace.x[ 18 ]), &(acadoVariables.x[ 44 ]) );
acado_multEDu( &(acadoWorkspace.E[ 520 ]), &(acadoWorkspace.x[ 20 ]), &(acadoVariables.x[ 44 ]) );
acado_multEDu( &(acadoWorkspace.E[ 528 ]), acadoWorkspace.x, &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 536 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 544 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 560 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 568 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 584 ]), &(acadoWorkspace.x[ 14 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 592 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 600 ]), &(acadoWorkspace.x[ 18 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 608 ]), &(acadoWorkspace.x[ 20 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 616 ]), &(acadoWorkspace.x[ 22 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 624 ]), acadoWorkspace.x, &(acadoVariables.x[ 52 ]) );
acado_multEDu( &(acadoWorkspace.E[ 632 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 52 ]) );
acado_multEDu( &(acadoWorkspace.E[ 640 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 52 ]) );
acado_multEDu( &(acadoWorkspace.E[ 648 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 52 ]) );
acado_multEDu( &(acadoWorkspace.E[ 656 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 52 ]) );
acado_multEDu( &(acadoWorkspace.E[ 664 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 52 ]) );
acado_multEDu( &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 52 ]) );
acado_multEDu( &(acadoWorkspace.E[ 680 ]), &(acadoWorkspace.x[ 14 ]), &(acadoVariables.x[ 52 ]) );
acado_multEDu( &(acadoWorkspace.E[ 688 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 52 ]) );
acado_multEDu( &(acadoWorkspace.E[ 696 ]), &(acadoWorkspace.x[ 18 ]), &(acadoVariables.x[ 52 ]) );
acado_multEDu( &(acadoWorkspace.E[ 704 ]), &(acadoWorkspace.x[ 20 ]), &(acadoVariables.x[ 52 ]) );
acado_multEDu( &(acadoWorkspace.E[ 712 ]), &(acadoWorkspace.x[ 22 ]), &(acadoVariables.x[ 52 ]) );
acado_multEDu( &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.x[ 24 ]), &(acadoVariables.x[ 52 ]) );
acado_multEDu( &(acadoWorkspace.E[ 728 ]), acadoWorkspace.x, &(acadoVariables.x[ 56 ]) );
acado_multEDu( &(acadoWorkspace.E[ 736 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 56 ]) );
acado_multEDu( &(acadoWorkspace.E[ 744 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 56 ]) );
acado_multEDu( &(acadoWorkspace.E[ 752 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 56 ]) );
acado_multEDu( &(acadoWorkspace.E[ 760 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 56 ]) );
acado_multEDu( &(acadoWorkspace.E[ 768 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 56 ]) );
acado_multEDu( &(acadoWorkspace.E[ 776 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 56 ]) );
acado_multEDu( &(acadoWorkspace.E[ 784 ]), &(acadoWorkspace.x[ 14 ]), &(acadoVariables.x[ 56 ]) );
acado_multEDu( &(acadoWorkspace.E[ 792 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 56 ]) );
acado_multEDu( &(acadoWorkspace.E[ 800 ]), &(acadoWorkspace.x[ 18 ]), &(acadoVariables.x[ 56 ]) );
acado_multEDu( &(acadoWorkspace.E[ 808 ]), &(acadoWorkspace.x[ 20 ]), &(acadoVariables.x[ 56 ]) );
acado_multEDu( &(acadoWorkspace.E[ 816 ]), &(acadoWorkspace.x[ 22 ]), &(acadoVariables.x[ 56 ]) );
acado_multEDu( &(acadoWorkspace.E[ 824 ]), &(acadoWorkspace.x[ 24 ]), &(acadoVariables.x[ 56 ]) );
acado_multEDu( &(acadoWorkspace.E[ 832 ]), &(acadoWorkspace.x[ 26 ]), &(acadoVariables.x[ 56 ]) );
acado_multEDu( &(acadoWorkspace.E[ 840 ]), acadoWorkspace.x, &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 848 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 856 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 872 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 880 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 888 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 896 ]), &(acadoWorkspace.x[ 14 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 904 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 912 ]), &(acadoWorkspace.x[ 18 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 920 ]), &(acadoWorkspace.x[ 20 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 928 ]), &(acadoWorkspace.x[ 22 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 936 ]), &(acadoWorkspace.x[ 24 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 944 ]), &(acadoWorkspace.x[ 26 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 952 ]), &(acadoWorkspace.x[ 28 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 960 ]), acadoWorkspace.x, &(acadoVariables.x[ 64 ]) );
acado_multEDu( &(acadoWorkspace.E[ 968 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 64 ]) );
acado_multEDu( &(acadoWorkspace.E[ 976 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 64 ]) );
acado_multEDu( &(acadoWorkspace.E[ 984 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 64 ]) );
acado_multEDu( &(acadoWorkspace.E[ 992 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 64 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1000 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 64 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1008 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 64 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1016 ]), &(acadoWorkspace.x[ 14 ]), &(acadoVariables.x[ 64 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1024 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 64 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1032 ]), &(acadoWorkspace.x[ 18 ]), &(acadoVariables.x[ 64 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1040 ]), &(acadoWorkspace.x[ 20 ]), &(acadoVariables.x[ 64 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1048 ]), &(acadoWorkspace.x[ 22 ]), &(acadoVariables.x[ 64 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1056 ]), &(acadoWorkspace.x[ 24 ]), &(acadoVariables.x[ 64 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1064 ]), &(acadoWorkspace.x[ 26 ]), &(acadoVariables.x[ 64 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1072 ]), &(acadoWorkspace.x[ 28 ]), &(acadoVariables.x[ 64 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1080 ]), &(acadoWorkspace.x[ 30 ]), &(acadoVariables.x[ 64 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1088 ]), acadoWorkspace.x, &(acadoVariables.x[ 68 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1096 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 68 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 68 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1112 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 68 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1120 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 68 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1128 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 68 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1136 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 68 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1144 ]), &(acadoWorkspace.x[ 14 ]), &(acadoVariables.x[ 68 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1152 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 68 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1160 ]), &(acadoWorkspace.x[ 18 ]), &(acadoVariables.x[ 68 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1168 ]), &(acadoWorkspace.x[ 20 ]), &(acadoVariables.x[ 68 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1176 ]), &(acadoWorkspace.x[ 22 ]), &(acadoVariables.x[ 68 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1184 ]), &(acadoWorkspace.x[ 24 ]), &(acadoVariables.x[ 68 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1192 ]), &(acadoWorkspace.x[ 26 ]), &(acadoVariables.x[ 68 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1200 ]), &(acadoWorkspace.x[ 28 ]), &(acadoVariables.x[ 68 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1208 ]), &(acadoWorkspace.x[ 30 ]), &(acadoVariables.x[ 68 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1216 ]), &(acadoWorkspace.x[ 32 ]), &(acadoVariables.x[ 68 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1224 ]), acadoWorkspace.x, &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1232 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1240 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1248 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1256 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1264 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1272 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1280 ]), &(acadoWorkspace.x[ 14 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1288 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.x[ 18 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1304 ]), &(acadoWorkspace.x[ 20 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1312 ]), &(acadoWorkspace.x[ 22 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1320 ]), &(acadoWorkspace.x[ 24 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1328 ]), &(acadoWorkspace.x[ 26 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1336 ]), &(acadoWorkspace.x[ 28 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1344 ]), &(acadoWorkspace.x[ 30 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1352 ]), &(acadoWorkspace.x[ 32 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1360 ]), &(acadoWorkspace.x[ 34 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1368 ]), acadoWorkspace.x, &(acadoVariables.x[ 76 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1376 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 76 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1384 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 76 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1392 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 76 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1400 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 76 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1408 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 76 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1416 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 76 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1424 ]), &(acadoWorkspace.x[ 14 ]), &(acadoVariables.x[ 76 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1432 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 76 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.x[ 18 ]), &(acadoVariables.x[ 76 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1448 ]), &(acadoWorkspace.x[ 20 ]), &(acadoVariables.x[ 76 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1456 ]), &(acadoWorkspace.x[ 22 ]), &(acadoVariables.x[ 76 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1464 ]), &(acadoWorkspace.x[ 24 ]), &(acadoVariables.x[ 76 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1472 ]), &(acadoWorkspace.x[ 26 ]), &(acadoVariables.x[ 76 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1480 ]), &(acadoWorkspace.x[ 28 ]), &(acadoVariables.x[ 76 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.x[ 30 ]), &(acadoVariables.x[ 76 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1496 ]), &(acadoWorkspace.x[ 32 ]), &(acadoVariables.x[ 76 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1504 ]), &(acadoWorkspace.x[ 34 ]), &(acadoVariables.x[ 76 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1512 ]), &(acadoWorkspace.x[ 36 ]), &(acadoVariables.x[ 76 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1520 ]), acadoWorkspace.x, &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1528 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1544 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1552 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1560 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1568 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1576 ]), &(acadoWorkspace.x[ 14 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1584 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1592 ]), &(acadoWorkspace.x[ 18 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1600 ]), &(acadoWorkspace.x[ 20 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1608 ]), &(acadoWorkspace.x[ 22 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1616 ]), &(acadoWorkspace.x[ 24 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1624 ]), &(acadoWorkspace.x[ 26 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1632 ]), &(acadoWorkspace.x[ 28 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1640 ]), &(acadoWorkspace.x[ 30 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1648 ]), &(acadoWorkspace.x[ 32 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1656 ]), &(acadoWorkspace.x[ 34 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1664 ]), &(acadoWorkspace.x[ 36 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1672 ]), &(acadoWorkspace.x[ 38 ]), &(acadoVariables.x[ 80 ]) );
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
return ret;
}

void acado_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 20; ++index)
{
state[0] = acadoVariables.x[index * 4];
state[1] = acadoVariables.x[index * 4 + 1];
state[2] = acadoVariables.x[index * 4 + 2];
state[3] = acadoVariables.x[index * 4 + 3];
state[28] = acadoVariables.u[index * 2];
state[29] = acadoVariables.u[index * 2 + 1];
state[30] = acadoVariables.od[index * 32];
state[31] = acadoVariables.od[index * 32 + 1];
state[32] = acadoVariables.od[index * 32 + 2];
state[33] = acadoVariables.od[index * 32 + 3];
state[34] = acadoVariables.od[index * 32 + 4];
state[35] = acadoVariables.od[index * 32 + 5];
state[36] = acadoVariables.od[index * 32 + 6];
state[37] = acadoVariables.od[index * 32 + 7];
state[38] = acadoVariables.od[index * 32 + 8];
state[39] = acadoVariables.od[index * 32 + 9];
state[40] = acadoVariables.od[index * 32 + 10];
state[41] = acadoVariables.od[index * 32 + 11];
state[42] = acadoVariables.od[index * 32 + 12];
state[43] = acadoVariables.od[index * 32 + 13];
state[44] = acadoVariables.od[index * 32 + 14];
state[45] = acadoVariables.od[index * 32 + 15];
state[46] = acadoVariables.od[index * 32 + 16];
state[47] = acadoVariables.od[index * 32 + 17];
state[48] = acadoVariables.od[index * 32 + 18];
state[49] = acadoVariables.od[index * 32 + 19];
state[50] = acadoVariables.od[index * 32 + 20];
state[51] = acadoVariables.od[index * 32 + 21];
state[52] = acadoVariables.od[index * 32 + 22];
state[53] = acadoVariables.od[index * 32 + 23];
state[54] = acadoVariables.od[index * 32 + 24];
state[55] = acadoVariables.od[index * 32 + 25];
state[56] = acadoVariables.od[index * 32 + 26];
state[57] = acadoVariables.od[index * 32 + 27];
state[58] = acadoVariables.od[index * 32 + 28];
state[59] = acadoVariables.od[index * 32 + 29];
state[60] = acadoVariables.od[index * 32 + 30];
state[61] = acadoVariables.od[index * 32 + 31];

acado_integrate(state, index == 0);

acadoVariables.x[index * 4 + 4] = state[0];
acadoVariables.x[index * 4 + 5] = state[1];
acadoVariables.x[index * 4 + 6] = state[2];
acadoVariables.x[index * 4 + 7] = state[3];
}
}

void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 20; ++index)
{
acadoVariables.x[index * 4] = acadoVariables.x[index * 4 + 4];
acadoVariables.x[index * 4 + 1] = acadoVariables.x[index * 4 + 5];
acadoVariables.x[index * 4 + 2] = acadoVariables.x[index * 4 + 6];
acadoVariables.x[index * 4 + 3] = acadoVariables.x[index * 4 + 7];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[80] = xEnd[0];
acadoVariables.x[81] = xEnd[1];
acadoVariables.x[82] = xEnd[2];
acadoVariables.x[83] = xEnd[3];
}
else if (strategy == 2) 
{
state[0] = acadoVariables.x[80];
state[1] = acadoVariables.x[81];
state[2] = acadoVariables.x[82];
state[3] = acadoVariables.x[83];
if (uEnd != 0)
{
state[28] = uEnd[0];
state[29] = uEnd[1];
}
else
{
state[28] = acadoVariables.u[38];
state[29] = acadoVariables.u[39];
}
state[30] = acadoVariables.od[640];
state[31] = acadoVariables.od[641];
state[32] = acadoVariables.od[642];
state[33] = acadoVariables.od[643];
state[34] = acadoVariables.od[644];
state[35] = acadoVariables.od[645];
state[36] = acadoVariables.od[646];
state[37] = acadoVariables.od[647];
state[38] = acadoVariables.od[648];
state[39] = acadoVariables.od[649];
state[40] = acadoVariables.od[650];
state[41] = acadoVariables.od[651];
state[42] = acadoVariables.od[652];
state[43] = acadoVariables.od[653];
state[44] = acadoVariables.od[654];
state[45] = acadoVariables.od[655];
state[46] = acadoVariables.od[656];
state[47] = acadoVariables.od[657];
state[48] = acadoVariables.od[658];
state[49] = acadoVariables.od[659];
state[50] = acadoVariables.od[660];
state[51] = acadoVariables.od[661];
state[52] = acadoVariables.od[662];
state[53] = acadoVariables.od[663];
state[54] = acadoVariables.od[664];
state[55] = acadoVariables.od[665];
state[56] = acadoVariables.od[666];
state[57] = acadoVariables.od[667];
state[58] = acadoVariables.od[668];
state[59] = acadoVariables.od[669];
state[60] = acadoVariables.od[670];
state[61] = acadoVariables.od[671];

acado_integrate(state, 1);

acadoVariables.x[80] = state[0];
acadoVariables.x[81] = state[1];
acadoVariables.x[82] = state[2];
acadoVariables.x[83] = state[3];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 19; ++index)
{
acadoVariables.u[index * 2] = acadoVariables.u[index * 2 + 2];
acadoVariables.u[index * 2 + 1] = acadoVariables.u[index * 2 + 3];
}

if (uEnd != 0)
{
acadoVariables.u[38] = uEnd[0];
acadoVariables.u[39] = uEnd[1];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19] + acadoWorkspace.g[20]*acadoWorkspace.x[20] + acadoWorkspace.g[21]*acadoWorkspace.x[21] + acadoWorkspace.g[22]*acadoWorkspace.x[22] + acadoWorkspace.g[23]*acadoWorkspace.x[23] + acadoWorkspace.g[24]*acadoWorkspace.x[24] + acadoWorkspace.g[25]*acadoWorkspace.x[25] + acadoWorkspace.g[26]*acadoWorkspace.x[26] + acadoWorkspace.g[27]*acadoWorkspace.x[27] + acadoWorkspace.g[28]*acadoWorkspace.x[28] + acadoWorkspace.g[29]*acadoWorkspace.x[29] + acadoWorkspace.g[30]*acadoWorkspace.x[30] + acadoWorkspace.g[31]*acadoWorkspace.x[31] + acadoWorkspace.g[32]*acadoWorkspace.x[32] + acadoWorkspace.g[33]*acadoWorkspace.x[33] + acadoWorkspace.g[34]*acadoWorkspace.x[34] + acadoWorkspace.g[35]*acadoWorkspace.x[35] + acadoWorkspace.g[36]*acadoWorkspace.x[36] + acadoWorkspace.g[37]*acadoWorkspace.x[37] + acadoWorkspace.g[38]*acadoWorkspace.x[38] + acadoWorkspace.g[39]*acadoWorkspace.x[39];
kkt = fabs( kkt );
for (index = 0; index < 40; ++index)
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
/** Row vector of size: 7 */
real_t tmpDy[ 7 ];

/** Row vector of size: 4 */
real_t tmpDyN[ 4 ];

for (lRun1 = 0; lRun1 < 20; ++lRun1)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1 * 4];
acadoWorkspace.objValueIn[1] = acadoVariables.x[lRun1 * 4 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[lRun1 * 4 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[lRun1 * 4 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.u[lRun1 * 2];
acadoWorkspace.objValueIn[5] = acadoVariables.u[lRun1 * 2 + 1];
acadoWorkspace.objValueIn[6] = acadoVariables.od[lRun1 * 32];
acadoWorkspace.objValueIn[7] = acadoVariables.od[lRun1 * 32 + 1];
acadoWorkspace.objValueIn[8] = acadoVariables.od[lRun1 * 32 + 2];
acadoWorkspace.objValueIn[9] = acadoVariables.od[lRun1 * 32 + 3];
acadoWorkspace.objValueIn[10] = acadoVariables.od[lRun1 * 32 + 4];
acadoWorkspace.objValueIn[11] = acadoVariables.od[lRun1 * 32 + 5];
acadoWorkspace.objValueIn[12] = acadoVariables.od[lRun1 * 32 + 6];
acadoWorkspace.objValueIn[13] = acadoVariables.od[lRun1 * 32 + 7];
acadoWorkspace.objValueIn[14] = acadoVariables.od[lRun1 * 32 + 8];
acadoWorkspace.objValueIn[15] = acadoVariables.od[lRun1 * 32 + 9];
acadoWorkspace.objValueIn[16] = acadoVariables.od[lRun1 * 32 + 10];
acadoWorkspace.objValueIn[17] = acadoVariables.od[lRun1 * 32 + 11];
acadoWorkspace.objValueIn[18] = acadoVariables.od[lRun1 * 32 + 12];
acadoWorkspace.objValueIn[19] = acadoVariables.od[lRun1 * 32 + 13];
acadoWorkspace.objValueIn[20] = acadoVariables.od[lRun1 * 32 + 14];
acadoWorkspace.objValueIn[21] = acadoVariables.od[lRun1 * 32 + 15];
acadoWorkspace.objValueIn[22] = acadoVariables.od[lRun1 * 32 + 16];
acadoWorkspace.objValueIn[23] = acadoVariables.od[lRun1 * 32 + 17];
acadoWorkspace.objValueIn[24] = acadoVariables.od[lRun1 * 32 + 18];
acadoWorkspace.objValueIn[25] = acadoVariables.od[lRun1 * 32 + 19];
acadoWorkspace.objValueIn[26] = acadoVariables.od[lRun1 * 32 + 20];
acadoWorkspace.objValueIn[27] = acadoVariables.od[lRun1 * 32 + 21];
acadoWorkspace.objValueIn[28] = acadoVariables.od[lRun1 * 32 + 22];
acadoWorkspace.objValueIn[29] = acadoVariables.od[lRun1 * 32 + 23];
acadoWorkspace.objValueIn[30] = acadoVariables.od[lRun1 * 32 + 24];
acadoWorkspace.objValueIn[31] = acadoVariables.od[lRun1 * 32 + 25];
acadoWorkspace.objValueIn[32] = acadoVariables.od[lRun1 * 32 + 26];
acadoWorkspace.objValueIn[33] = acadoVariables.od[lRun1 * 32 + 27];
acadoWorkspace.objValueIn[34] = acadoVariables.od[lRun1 * 32 + 28];
acadoWorkspace.objValueIn[35] = acadoVariables.od[lRun1 * 32 + 29];
acadoWorkspace.objValueIn[36] = acadoVariables.od[lRun1 * 32 + 30];
acadoWorkspace.objValueIn[37] = acadoVariables.od[lRun1 * 32 + 31];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[lRun1 * 7] = acadoWorkspace.objValueOut[0] - acadoVariables.y[lRun1 * 7];
acadoWorkspace.Dy[lRun1 * 7 + 1] = acadoWorkspace.objValueOut[1] - acadoVariables.y[lRun1 * 7 + 1];
acadoWorkspace.Dy[lRun1 * 7 + 2] = acadoWorkspace.objValueOut[2] - acadoVariables.y[lRun1 * 7 + 2];
acadoWorkspace.Dy[lRun1 * 7 + 3] = acadoWorkspace.objValueOut[3] - acadoVariables.y[lRun1 * 7 + 3];
acadoWorkspace.Dy[lRun1 * 7 + 4] = acadoWorkspace.objValueOut[4] - acadoVariables.y[lRun1 * 7 + 4];
acadoWorkspace.Dy[lRun1 * 7 + 5] = acadoWorkspace.objValueOut[5] - acadoVariables.y[lRun1 * 7 + 5];
acadoWorkspace.Dy[lRun1 * 7 + 6] = acadoWorkspace.objValueOut[6] - acadoVariables.y[lRun1 * 7 + 6];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[80];
acadoWorkspace.objValueIn[1] = acadoVariables.x[81];
acadoWorkspace.objValueIn[2] = acadoVariables.x[82];
acadoWorkspace.objValueIn[3] = acadoVariables.x[83];
acadoWorkspace.objValueIn[4] = acadoVariables.od[640];
acadoWorkspace.objValueIn[5] = acadoVariables.od[641];
acadoWorkspace.objValueIn[6] = acadoVariables.od[642];
acadoWorkspace.objValueIn[7] = acadoVariables.od[643];
acadoWorkspace.objValueIn[8] = acadoVariables.od[644];
acadoWorkspace.objValueIn[9] = acadoVariables.od[645];
acadoWorkspace.objValueIn[10] = acadoVariables.od[646];
acadoWorkspace.objValueIn[11] = acadoVariables.od[647];
acadoWorkspace.objValueIn[12] = acadoVariables.od[648];
acadoWorkspace.objValueIn[13] = acadoVariables.od[649];
acadoWorkspace.objValueIn[14] = acadoVariables.od[650];
acadoWorkspace.objValueIn[15] = acadoVariables.od[651];
acadoWorkspace.objValueIn[16] = acadoVariables.od[652];
acadoWorkspace.objValueIn[17] = acadoVariables.od[653];
acadoWorkspace.objValueIn[18] = acadoVariables.od[654];
acadoWorkspace.objValueIn[19] = acadoVariables.od[655];
acadoWorkspace.objValueIn[20] = acadoVariables.od[656];
acadoWorkspace.objValueIn[21] = acadoVariables.od[657];
acadoWorkspace.objValueIn[22] = acadoVariables.od[658];
acadoWorkspace.objValueIn[23] = acadoVariables.od[659];
acadoWorkspace.objValueIn[24] = acadoVariables.od[660];
acadoWorkspace.objValueIn[25] = acadoVariables.od[661];
acadoWorkspace.objValueIn[26] = acadoVariables.od[662];
acadoWorkspace.objValueIn[27] = acadoVariables.od[663];
acadoWorkspace.objValueIn[28] = acadoVariables.od[664];
acadoWorkspace.objValueIn[29] = acadoVariables.od[665];
acadoWorkspace.objValueIn[30] = acadoVariables.od[666];
acadoWorkspace.objValueIn[31] = acadoVariables.od[667];
acadoWorkspace.objValueIn[32] = acadoVariables.od[668];
acadoWorkspace.objValueIn[33] = acadoVariables.od[669];
acadoWorkspace.objValueIn[34] = acadoVariables.od[670];
acadoWorkspace.objValueIn[35] = acadoVariables.od[671];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1] - acadoVariables.yN[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2] - acadoVariables.yN[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3] - acadoVariables.yN[3];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 20; ++lRun1)
{
tmpDy[0] = + acadoWorkspace.Dy[lRun1 * 7]*acadoVariables.W[lRun1 * 49] + acadoWorkspace.Dy[lRun1 * 7 + 1]*acadoVariables.W[lRun1 * 49 + 7] + acadoWorkspace.Dy[lRun1 * 7 + 2]*acadoVariables.W[lRun1 * 49 + 14] + acadoWorkspace.Dy[lRun1 * 7 + 3]*acadoVariables.W[lRun1 * 49 + 21] + acadoWorkspace.Dy[lRun1 * 7 + 4]*acadoVariables.W[lRun1 * 49 + 28] + acadoWorkspace.Dy[lRun1 * 7 + 5]*acadoVariables.W[lRun1 * 49 + 35] + acadoWorkspace.Dy[lRun1 * 7 + 6]*acadoVariables.W[lRun1 * 49 + 42];
tmpDy[1] = + acadoWorkspace.Dy[lRun1 * 7]*acadoVariables.W[lRun1 * 49 + 1] + acadoWorkspace.Dy[lRun1 * 7 + 1]*acadoVariables.W[lRun1 * 49 + 8] + acadoWorkspace.Dy[lRun1 * 7 + 2]*acadoVariables.W[lRun1 * 49 + 15] + acadoWorkspace.Dy[lRun1 * 7 + 3]*acadoVariables.W[lRun1 * 49 + 22] + acadoWorkspace.Dy[lRun1 * 7 + 4]*acadoVariables.W[lRun1 * 49 + 29] + acadoWorkspace.Dy[lRun1 * 7 + 5]*acadoVariables.W[lRun1 * 49 + 36] + acadoWorkspace.Dy[lRun1 * 7 + 6]*acadoVariables.W[lRun1 * 49 + 43];
tmpDy[2] = + acadoWorkspace.Dy[lRun1 * 7]*acadoVariables.W[lRun1 * 49 + 2] + acadoWorkspace.Dy[lRun1 * 7 + 1]*acadoVariables.W[lRun1 * 49 + 9] + acadoWorkspace.Dy[lRun1 * 7 + 2]*acadoVariables.W[lRun1 * 49 + 16] + acadoWorkspace.Dy[lRun1 * 7 + 3]*acadoVariables.W[lRun1 * 49 + 23] + acadoWorkspace.Dy[lRun1 * 7 + 4]*acadoVariables.W[lRun1 * 49 + 30] + acadoWorkspace.Dy[lRun1 * 7 + 5]*acadoVariables.W[lRun1 * 49 + 37] + acadoWorkspace.Dy[lRun1 * 7 + 6]*acadoVariables.W[lRun1 * 49 + 44];
tmpDy[3] = + acadoWorkspace.Dy[lRun1 * 7]*acadoVariables.W[lRun1 * 49 + 3] + acadoWorkspace.Dy[lRun1 * 7 + 1]*acadoVariables.W[lRun1 * 49 + 10] + acadoWorkspace.Dy[lRun1 * 7 + 2]*acadoVariables.W[lRun1 * 49 + 17] + acadoWorkspace.Dy[lRun1 * 7 + 3]*acadoVariables.W[lRun1 * 49 + 24] + acadoWorkspace.Dy[lRun1 * 7 + 4]*acadoVariables.W[lRun1 * 49 + 31] + acadoWorkspace.Dy[lRun1 * 7 + 5]*acadoVariables.W[lRun1 * 49 + 38] + acadoWorkspace.Dy[lRun1 * 7 + 6]*acadoVariables.W[lRun1 * 49 + 45];
tmpDy[4] = + acadoWorkspace.Dy[lRun1 * 7]*acadoVariables.W[lRun1 * 49 + 4] + acadoWorkspace.Dy[lRun1 * 7 + 1]*acadoVariables.W[lRun1 * 49 + 11] + acadoWorkspace.Dy[lRun1 * 7 + 2]*acadoVariables.W[lRun1 * 49 + 18] + acadoWorkspace.Dy[lRun1 * 7 + 3]*acadoVariables.W[lRun1 * 49 + 25] + acadoWorkspace.Dy[lRun1 * 7 + 4]*acadoVariables.W[lRun1 * 49 + 32] + acadoWorkspace.Dy[lRun1 * 7 + 5]*acadoVariables.W[lRun1 * 49 + 39] + acadoWorkspace.Dy[lRun1 * 7 + 6]*acadoVariables.W[lRun1 * 49 + 46];
tmpDy[5] = + acadoWorkspace.Dy[lRun1 * 7]*acadoVariables.W[lRun1 * 49 + 5] + acadoWorkspace.Dy[lRun1 * 7 + 1]*acadoVariables.W[lRun1 * 49 + 12] + acadoWorkspace.Dy[lRun1 * 7 + 2]*acadoVariables.W[lRun1 * 49 + 19] + acadoWorkspace.Dy[lRun1 * 7 + 3]*acadoVariables.W[lRun1 * 49 + 26] + acadoWorkspace.Dy[lRun1 * 7 + 4]*acadoVariables.W[lRun1 * 49 + 33] + acadoWorkspace.Dy[lRun1 * 7 + 5]*acadoVariables.W[lRun1 * 49 + 40] + acadoWorkspace.Dy[lRun1 * 7 + 6]*acadoVariables.W[lRun1 * 49 + 47];
tmpDy[6] = + acadoWorkspace.Dy[lRun1 * 7]*acadoVariables.W[lRun1 * 49 + 6] + acadoWorkspace.Dy[lRun1 * 7 + 1]*acadoVariables.W[lRun1 * 49 + 13] + acadoWorkspace.Dy[lRun1 * 7 + 2]*acadoVariables.W[lRun1 * 49 + 20] + acadoWorkspace.Dy[lRun1 * 7 + 3]*acadoVariables.W[lRun1 * 49 + 27] + acadoWorkspace.Dy[lRun1 * 7 + 4]*acadoVariables.W[lRun1 * 49 + 34] + acadoWorkspace.Dy[lRun1 * 7 + 5]*acadoVariables.W[lRun1 * 49 + 41] + acadoWorkspace.Dy[lRun1 * 7 + 6]*acadoVariables.W[lRun1 * 49 + 48];
objVal += + acadoWorkspace.Dy[lRun1 * 7]*tmpDy[0] + acadoWorkspace.Dy[lRun1 * 7 + 1]*tmpDy[1] + acadoWorkspace.Dy[lRun1 * 7 + 2]*tmpDy[2] + acadoWorkspace.Dy[lRun1 * 7 + 3]*tmpDy[3] + acadoWorkspace.Dy[lRun1 * 7 + 4]*tmpDy[4] + acadoWorkspace.Dy[lRun1 * 7 + 5]*tmpDy[5] + acadoWorkspace.Dy[lRun1 * 7 + 6]*tmpDy[6];
}

tmpDyN[0] = + acadoWorkspace.DyN[0]*acadoVariables.WN[0];
tmpDyN[1] = + acadoWorkspace.DyN[1]*acadoVariables.WN[5];
tmpDyN[2] = + acadoWorkspace.DyN[2]*acadoVariables.WN[10];
tmpDyN[3] = + acadoWorkspace.DyN[3]*acadoVariables.WN[15];
objVal += + acadoWorkspace.DyN[0]*tmpDyN[0] + acadoWorkspace.DyN[1]*tmpDyN[1] + acadoWorkspace.DyN[2]*tmpDyN[2] + acadoWorkspace.DyN[3]*tmpDyN[3];

objVal *= 0.5;
return objVal;
}

