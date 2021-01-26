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
for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
acadoWorkspace.state[0] = acadoVariables.x[lRun1 * 5];
acadoWorkspace.state[1] = acadoVariables.x[lRun1 * 5 + 1];
acadoWorkspace.state[2] = acadoVariables.x[lRun1 * 5 + 2];
acadoWorkspace.state[3] = acadoVariables.x[lRun1 * 5 + 3];
acadoWorkspace.state[4] = acadoVariables.x[lRun1 * 5 + 4];

acadoWorkspace.state[40] = acadoVariables.u[lRun1 * 2];
acadoWorkspace.state[41] = acadoVariables.u[lRun1 * 2 + 1];

ret = acado_integrate(acadoWorkspace.state, 1);

acadoWorkspace.d[lRun1 * 5] = acadoWorkspace.state[0] - acadoVariables.x[lRun1 * 5 + 5];
acadoWorkspace.d[lRun1 * 5 + 1] = acadoWorkspace.state[1] - acadoVariables.x[lRun1 * 5 + 6];
acadoWorkspace.d[lRun1 * 5 + 2] = acadoWorkspace.state[2] - acadoVariables.x[lRun1 * 5 + 7];
acadoWorkspace.d[lRun1 * 5 + 3] = acadoWorkspace.state[3] - acadoVariables.x[lRun1 * 5 + 8];
acadoWorkspace.d[lRun1 * 5 + 4] = acadoWorkspace.state[4] - acadoVariables.x[lRun1 * 5 + 9];

acadoWorkspace.evGx[lRun1 * 25] = acadoWorkspace.state[5];
acadoWorkspace.evGx[lRun1 * 25 + 1] = acadoWorkspace.state[6];
acadoWorkspace.evGx[lRun1 * 25 + 2] = acadoWorkspace.state[7];
acadoWorkspace.evGx[lRun1 * 25 + 3] = acadoWorkspace.state[8];
acadoWorkspace.evGx[lRun1 * 25 + 4] = acadoWorkspace.state[9];
acadoWorkspace.evGx[lRun1 * 25 + 5] = acadoWorkspace.state[10];
acadoWorkspace.evGx[lRun1 * 25 + 6] = acadoWorkspace.state[11];
acadoWorkspace.evGx[lRun1 * 25 + 7] = acadoWorkspace.state[12];
acadoWorkspace.evGx[lRun1 * 25 + 8] = acadoWorkspace.state[13];
acadoWorkspace.evGx[lRun1 * 25 + 9] = acadoWorkspace.state[14];
acadoWorkspace.evGx[lRun1 * 25 + 10] = acadoWorkspace.state[15];
acadoWorkspace.evGx[lRun1 * 25 + 11] = acadoWorkspace.state[16];
acadoWorkspace.evGx[lRun1 * 25 + 12] = acadoWorkspace.state[17];
acadoWorkspace.evGx[lRun1 * 25 + 13] = acadoWorkspace.state[18];
acadoWorkspace.evGx[lRun1 * 25 + 14] = acadoWorkspace.state[19];
acadoWorkspace.evGx[lRun1 * 25 + 15] = acadoWorkspace.state[20];
acadoWorkspace.evGx[lRun1 * 25 + 16] = acadoWorkspace.state[21];
acadoWorkspace.evGx[lRun1 * 25 + 17] = acadoWorkspace.state[22];
acadoWorkspace.evGx[lRun1 * 25 + 18] = acadoWorkspace.state[23];
acadoWorkspace.evGx[lRun1 * 25 + 19] = acadoWorkspace.state[24];
acadoWorkspace.evGx[lRun1 * 25 + 20] = acadoWorkspace.state[25];
acadoWorkspace.evGx[lRun1 * 25 + 21] = acadoWorkspace.state[26];
acadoWorkspace.evGx[lRun1 * 25 + 22] = acadoWorkspace.state[27];
acadoWorkspace.evGx[lRun1 * 25 + 23] = acadoWorkspace.state[28];
acadoWorkspace.evGx[lRun1 * 25 + 24] = acadoWorkspace.state[29];

acadoWorkspace.evGu[lRun1 * 10] = acadoWorkspace.state[30];
acadoWorkspace.evGu[lRun1 * 10 + 1] = acadoWorkspace.state[31];
acadoWorkspace.evGu[lRun1 * 10 + 2] = acadoWorkspace.state[32];
acadoWorkspace.evGu[lRun1 * 10 + 3] = acadoWorkspace.state[33];
acadoWorkspace.evGu[lRun1 * 10 + 4] = acadoWorkspace.state[34];
acadoWorkspace.evGu[lRun1 * 10 + 5] = acadoWorkspace.state[35];
acadoWorkspace.evGu[lRun1 * 10 + 6] = acadoWorkspace.state[36];
acadoWorkspace.evGu[lRun1 * 10 + 7] = acadoWorkspace.state[37];
acadoWorkspace.evGu[lRun1 * 10 + 8] = acadoWorkspace.state[38];
acadoWorkspace.evGu[lRun1 * 10 + 9] = acadoWorkspace.state[39];
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
out[3] = xd[3];
out[4] = xd[4];
}

void acado_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
out[4] = xd[4];
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
tmpQ1[0] = + tmpQ2[0];
tmpQ1[1] = + tmpQ2[1];
tmpQ1[2] = + tmpQ2[2];
tmpQ1[3] = + tmpQ2[3];
tmpQ1[4] = + tmpQ2[4];
tmpQ1[5] = + tmpQ2[5];
tmpQ1[6] = + tmpQ2[6];
tmpQ1[7] = + tmpQ2[7];
tmpQ1[8] = + tmpQ2[8];
tmpQ1[9] = + tmpQ2[9];
tmpQ1[10] = + tmpQ2[10];
tmpQ1[11] = + tmpQ2[11];
tmpQ1[12] = + tmpQ2[12];
tmpQ1[13] = + tmpQ2[13];
tmpQ1[14] = + tmpQ2[14];
tmpQ1[15] = + tmpQ2[15];
tmpQ1[16] = + tmpQ2[16];
tmpQ1[17] = + tmpQ2[17];
tmpQ1[18] = + tmpQ2[18];
tmpQ1[19] = + tmpQ2[19];
tmpQ1[20] = + tmpQ2[20];
tmpQ1[21] = + tmpQ2[21];
tmpQ1[22] = + tmpQ2[22];
tmpQ1[23] = + tmpQ2[23];
tmpQ1[24] = + tmpQ2[24];
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
tmpQN2[16] = +tmpObjSEndTerm[16];
tmpQN2[17] = +tmpObjSEndTerm[17];
tmpQN2[18] = +tmpObjSEndTerm[18];
tmpQN2[19] = +tmpObjSEndTerm[19];
tmpQN2[20] = +tmpObjSEndTerm[20];
tmpQN2[21] = +tmpObjSEndTerm[21];
tmpQN2[22] = +tmpObjSEndTerm[22];
tmpQN2[23] = +tmpObjSEndTerm[23];
tmpQN2[24] = +tmpObjSEndTerm[24];
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
tmpQN1[16] = + tmpQN2[16];
tmpQN1[17] = + tmpQN2[17];
tmpQN1[18] = + tmpQN2[18];
tmpQN1[19] = + tmpQN2[19];
tmpQN1[20] = + tmpQN2[20];
tmpQN1[21] = + tmpQN2[21];
tmpQN1[22] = + tmpQN2[22];
tmpQN1[23] = + tmpQN2[23];
tmpQN1[24] = + tmpQN2[24];
}

void acado_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 30; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj * 5];
acadoWorkspace.objValueIn[1] = acadoVariables.x[runObj * 5 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[runObj * 5 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[runObj * 5 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[runObj * 5 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.u[runObj * 2];
acadoWorkspace.objValueIn[6] = acadoVariables.u[runObj * 2 + 1];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[runObj * 5] = acadoWorkspace.objValueOut[0];
acadoWorkspace.Dy[runObj * 5 + 1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.Dy[runObj * 5 + 2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.Dy[runObj * 5 + 3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.Dy[runObj * 5 + 4] = acadoWorkspace.objValueOut[4];

acado_setObjQ1Q2( &(acadoVariables.W[ runObj * 25 ]), &(acadoWorkspace.Q1[ runObj * 25 ]), &(acadoWorkspace.Q2[ runObj * 25 ]) );

}
acadoWorkspace.objValueIn[0] = acadoVariables.x[150];
acadoWorkspace.objValueIn[1] = acadoVariables.x[151];
acadoWorkspace.objValueIn[2] = acadoVariables.x[152];
acadoWorkspace.objValueIn[3] = acadoVariables.x[153];
acadoWorkspace.objValueIn[4] = acadoVariables.x[154];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.DyN[4] = acadoWorkspace.objValueOut[4];

acado_setObjQN1QN2( acadoVariables.WN, acadoWorkspace.QN1, acadoWorkspace.QN2 );

}

void acado_multGxd( real_t* const dOld, real_t* const Gx1, real_t* const dNew )
{
dNew[0] += + Gx1[0]*dOld[0] + Gx1[1]*dOld[1] + Gx1[2]*dOld[2] + Gx1[3]*dOld[3] + Gx1[4]*dOld[4];
dNew[1] += + Gx1[5]*dOld[0] + Gx1[6]*dOld[1] + Gx1[7]*dOld[2] + Gx1[8]*dOld[3] + Gx1[9]*dOld[4];
dNew[2] += + Gx1[10]*dOld[0] + Gx1[11]*dOld[1] + Gx1[12]*dOld[2] + Gx1[13]*dOld[3] + Gx1[14]*dOld[4];
dNew[3] += + Gx1[15]*dOld[0] + Gx1[16]*dOld[1] + Gx1[17]*dOld[2] + Gx1[18]*dOld[3] + Gx1[19]*dOld[4];
dNew[4] += + Gx1[20]*dOld[0] + Gx1[21]*dOld[1] + Gx1[22]*dOld[2] + Gx1[23]*dOld[3] + Gx1[24]*dOld[4];
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
Gx2[16] = Gx1[16];
Gx2[17] = Gx1[17];
Gx2[18] = Gx1[18];
Gx2[19] = Gx1[19];
Gx2[20] = Gx1[20];
Gx2[21] = Gx1[21];
Gx2[22] = Gx1[22];
Gx2[23] = Gx1[23];
Gx2[24] = Gx1[24];
}

void acado_multGxGx( real_t* const Gx1, real_t* const Gx2, real_t* const Gx3 )
{
Gx3[0] = + Gx1[0]*Gx2[0] + Gx1[1]*Gx2[5] + Gx1[2]*Gx2[10] + Gx1[3]*Gx2[15] + Gx1[4]*Gx2[20];
Gx3[1] = + Gx1[0]*Gx2[1] + Gx1[1]*Gx2[6] + Gx1[2]*Gx2[11] + Gx1[3]*Gx2[16] + Gx1[4]*Gx2[21];
Gx3[2] = + Gx1[0]*Gx2[2] + Gx1[1]*Gx2[7] + Gx1[2]*Gx2[12] + Gx1[3]*Gx2[17] + Gx1[4]*Gx2[22];
Gx3[3] = + Gx1[0]*Gx2[3] + Gx1[1]*Gx2[8] + Gx1[2]*Gx2[13] + Gx1[3]*Gx2[18] + Gx1[4]*Gx2[23];
Gx3[4] = + Gx1[0]*Gx2[4] + Gx1[1]*Gx2[9] + Gx1[2]*Gx2[14] + Gx1[3]*Gx2[19] + Gx1[4]*Gx2[24];
Gx3[5] = + Gx1[5]*Gx2[0] + Gx1[6]*Gx2[5] + Gx1[7]*Gx2[10] + Gx1[8]*Gx2[15] + Gx1[9]*Gx2[20];
Gx3[6] = + Gx1[5]*Gx2[1] + Gx1[6]*Gx2[6] + Gx1[7]*Gx2[11] + Gx1[8]*Gx2[16] + Gx1[9]*Gx2[21];
Gx3[7] = + Gx1[5]*Gx2[2] + Gx1[6]*Gx2[7] + Gx1[7]*Gx2[12] + Gx1[8]*Gx2[17] + Gx1[9]*Gx2[22];
Gx3[8] = + Gx1[5]*Gx2[3] + Gx1[6]*Gx2[8] + Gx1[7]*Gx2[13] + Gx1[8]*Gx2[18] + Gx1[9]*Gx2[23];
Gx3[9] = + Gx1[5]*Gx2[4] + Gx1[6]*Gx2[9] + Gx1[7]*Gx2[14] + Gx1[8]*Gx2[19] + Gx1[9]*Gx2[24];
Gx3[10] = + Gx1[10]*Gx2[0] + Gx1[11]*Gx2[5] + Gx1[12]*Gx2[10] + Gx1[13]*Gx2[15] + Gx1[14]*Gx2[20];
Gx3[11] = + Gx1[10]*Gx2[1] + Gx1[11]*Gx2[6] + Gx1[12]*Gx2[11] + Gx1[13]*Gx2[16] + Gx1[14]*Gx2[21];
Gx3[12] = + Gx1[10]*Gx2[2] + Gx1[11]*Gx2[7] + Gx1[12]*Gx2[12] + Gx1[13]*Gx2[17] + Gx1[14]*Gx2[22];
Gx3[13] = + Gx1[10]*Gx2[3] + Gx1[11]*Gx2[8] + Gx1[12]*Gx2[13] + Gx1[13]*Gx2[18] + Gx1[14]*Gx2[23];
Gx3[14] = + Gx1[10]*Gx2[4] + Gx1[11]*Gx2[9] + Gx1[12]*Gx2[14] + Gx1[13]*Gx2[19] + Gx1[14]*Gx2[24];
Gx3[15] = + Gx1[15]*Gx2[0] + Gx1[16]*Gx2[5] + Gx1[17]*Gx2[10] + Gx1[18]*Gx2[15] + Gx1[19]*Gx2[20];
Gx3[16] = + Gx1[15]*Gx2[1] + Gx1[16]*Gx2[6] + Gx1[17]*Gx2[11] + Gx1[18]*Gx2[16] + Gx1[19]*Gx2[21];
Gx3[17] = + Gx1[15]*Gx2[2] + Gx1[16]*Gx2[7] + Gx1[17]*Gx2[12] + Gx1[18]*Gx2[17] + Gx1[19]*Gx2[22];
Gx3[18] = + Gx1[15]*Gx2[3] + Gx1[16]*Gx2[8] + Gx1[17]*Gx2[13] + Gx1[18]*Gx2[18] + Gx1[19]*Gx2[23];
Gx3[19] = + Gx1[15]*Gx2[4] + Gx1[16]*Gx2[9] + Gx1[17]*Gx2[14] + Gx1[18]*Gx2[19] + Gx1[19]*Gx2[24];
Gx3[20] = + Gx1[20]*Gx2[0] + Gx1[21]*Gx2[5] + Gx1[22]*Gx2[10] + Gx1[23]*Gx2[15] + Gx1[24]*Gx2[20];
Gx3[21] = + Gx1[20]*Gx2[1] + Gx1[21]*Gx2[6] + Gx1[22]*Gx2[11] + Gx1[23]*Gx2[16] + Gx1[24]*Gx2[21];
Gx3[22] = + Gx1[20]*Gx2[2] + Gx1[21]*Gx2[7] + Gx1[22]*Gx2[12] + Gx1[23]*Gx2[17] + Gx1[24]*Gx2[22];
Gx3[23] = + Gx1[20]*Gx2[3] + Gx1[21]*Gx2[8] + Gx1[22]*Gx2[13] + Gx1[23]*Gx2[18] + Gx1[24]*Gx2[23];
Gx3[24] = + Gx1[20]*Gx2[4] + Gx1[21]*Gx2[9] + Gx1[22]*Gx2[14] + Gx1[23]*Gx2[19] + Gx1[24]*Gx2[24];
}

void acado_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[2] + Gx1[2]*Gu1[4] + Gx1[3]*Gu1[6] + Gx1[4]*Gu1[8];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[1]*Gu1[3] + Gx1[2]*Gu1[5] + Gx1[3]*Gu1[7] + Gx1[4]*Gu1[9];
Gu2[2] = + Gx1[5]*Gu1[0] + Gx1[6]*Gu1[2] + Gx1[7]*Gu1[4] + Gx1[8]*Gu1[6] + Gx1[9]*Gu1[8];
Gu2[3] = + Gx1[5]*Gu1[1] + Gx1[6]*Gu1[3] + Gx1[7]*Gu1[5] + Gx1[8]*Gu1[7] + Gx1[9]*Gu1[9];
Gu2[4] = + Gx1[10]*Gu1[0] + Gx1[11]*Gu1[2] + Gx1[12]*Gu1[4] + Gx1[13]*Gu1[6] + Gx1[14]*Gu1[8];
Gu2[5] = + Gx1[10]*Gu1[1] + Gx1[11]*Gu1[3] + Gx1[12]*Gu1[5] + Gx1[13]*Gu1[7] + Gx1[14]*Gu1[9];
Gu2[6] = + Gx1[15]*Gu1[0] + Gx1[16]*Gu1[2] + Gx1[17]*Gu1[4] + Gx1[18]*Gu1[6] + Gx1[19]*Gu1[8];
Gu2[7] = + Gx1[15]*Gu1[1] + Gx1[16]*Gu1[3] + Gx1[17]*Gu1[5] + Gx1[18]*Gu1[7] + Gx1[19]*Gu1[9];
Gu2[8] = + Gx1[20]*Gu1[0] + Gx1[21]*Gu1[2] + Gx1[22]*Gu1[4] + Gx1[23]*Gu1[6] + Gx1[24]*Gu1[8];
Gu2[9] = + Gx1[20]*Gu1[1] + Gx1[21]*Gu1[3] + Gx1[22]*Gu1[5] + Gx1[23]*Gu1[7] + Gx1[24]*Gu1[9];
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
Gu2[8] = Gu1[8];
Gu2[9] = Gu1[9];
}

void acado_setBlockH11( int iRow, int iCol, real_t* const Gu1, real_t* const Gu2 )
{
acadoWorkspace.H[(iRow * 120) + (iCol * 2)] += + Gu1[0]*Gu2[0] + Gu1[2]*Gu2[2] + Gu1[4]*Gu2[4] + Gu1[6]*Gu2[6] + Gu1[8]*Gu2[8];
acadoWorkspace.H[(iRow * 120) + (iCol * 2 + 1)] += + Gu1[0]*Gu2[1] + Gu1[2]*Gu2[3] + Gu1[4]*Gu2[5] + Gu1[6]*Gu2[7] + Gu1[8]*Gu2[9];
acadoWorkspace.H[(iRow * 120 + 60) + (iCol * 2)] += + Gu1[1]*Gu2[0] + Gu1[3]*Gu2[2] + Gu1[5]*Gu2[4] + Gu1[7]*Gu2[6] + Gu1[9]*Gu2[8];
acadoWorkspace.H[(iRow * 120 + 60) + (iCol * 2 + 1)] += + Gu1[1]*Gu2[1] + Gu1[3]*Gu2[3] + Gu1[5]*Gu2[5] + Gu1[7]*Gu2[7] + Gu1[9]*Gu2[9];
}

void acado_setBlockH11_R1( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 120) + (iCol * 2)] = + (real_t)1.0000000000000000e-03;
acadoWorkspace.H[(iRow * 120) + (iCol * 2 + 1)] = 0.0;
acadoWorkspace.H[(iRow * 120 + 60) + (iCol * 2)] = 0.0;
acadoWorkspace.H[(iRow * 120 + 60) + (iCol * 2 + 1)] = + (real_t)1.0000000000000000e-03;
}

void acado_zeroBlockH11( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 120) + (iCol * 2)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 120) + (iCol * 2 + 1)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 120 + 60) + (iCol * 2)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 120 + 60) + (iCol * 2 + 1)] = 0.0000000000000000e+00;
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 120) + (iCol * 2)] = acadoWorkspace.H[(iCol * 120) + (iRow * 2)];
acadoWorkspace.H[(iRow * 120) + (iCol * 2 + 1)] = acadoWorkspace.H[(iCol * 120 + 60) + (iRow * 2)];
acadoWorkspace.H[(iRow * 120 + 60) + (iCol * 2)] = acadoWorkspace.H[(iCol * 120) + (iRow * 2 + 1)];
acadoWorkspace.H[(iRow * 120 + 60) + (iCol * 2 + 1)] = acadoWorkspace.H[(iCol * 120 + 60) + (iRow * 2 + 1)];
}

void acado_multQ1d( real_t* const Gx1, real_t* const dOld, real_t* const dNew )
{
dNew[0] = + Gx1[0]*dOld[0] + Gx1[1]*dOld[1] + Gx1[2]*dOld[2] + Gx1[3]*dOld[3] + Gx1[4]*dOld[4];
dNew[1] = + Gx1[5]*dOld[0] + Gx1[6]*dOld[1] + Gx1[7]*dOld[2] + Gx1[8]*dOld[3] + Gx1[9]*dOld[4];
dNew[2] = + Gx1[10]*dOld[0] + Gx1[11]*dOld[1] + Gx1[12]*dOld[2] + Gx1[13]*dOld[3] + Gx1[14]*dOld[4];
dNew[3] = + Gx1[15]*dOld[0] + Gx1[16]*dOld[1] + Gx1[17]*dOld[2] + Gx1[18]*dOld[3] + Gx1[19]*dOld[4];
dNew[4] = + Gx1[20]*dOld[0] + Gx1[21]*dOld[1] + Gx1[22]*dOld[2] + Gx1[23]*dOld[3] + Gx1[24]*dOld[4];
}

void acado_multQN1d( real_t* const QN1, real_t* const dOld, real_t* const dNew )
{
dNew[0] = + acadoWorkspace.QN1[0]*dOld[0] + acadoWorkspace.QN1[1]*dOld[1] + acadoWorkspace.QN1[2]*dOld[2] + acadoWorkspace.QN1[3]*dOld[3] + acadoWorkspace.QN1[4]*dOld[4];
dNew[1] = + acadoWorkspace.QN1[5]*dOld[0] + acadoWorkspace.QN1[6]*dOld[1] + acadoWorkspace.QN1[7]*dOld[2] + acadoWorkspace.QN1[8]*dOld[3] + acadoWorkspace.QN1[9]*dOld[4];
dNew[2] = + acadoWorkspace.QN1[10]*dOld[0] + acadoWorkspace.QN1[11]*dOld[1] + acadoWorkspace.QN1[12]*dOld[2] + acadoWorkspace.QN1[13]*dOld[3] + acadoWorkspace.QN1[14]*dOld[4];
dNew[3] = + acadoWorkspace.QN1[15]*dOld[0] + acadoWorkspace.QN1[16]*dOld[1] + acadoWorkspace.QN1[17]*dOld[2] + acadoWorkspace.QN1[18]*dOld[3] + acadoWorkspace.QN1[19]*dOld[4];
dNew[4] = + acadoWorkspace.QN1[20]*dOld[0] + acadoWorkspace.QN1[21]*dOld[1] + acadoWorkspace.QN1[22]*dOld[2] + acadoWorkspace.QN1[23]*dOld[3] + acadoWorkspace.QN1[24]*dOld[4];
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
QDy1[0] = + Q2[0]*Dy1[0] + Q2[1]*Dy1[1] + Q2[2]*Dy1[2] + Q2[3]*Dy1[3] + Q2[4]*Dy1[4];
QDy1[1] = + Q2[5]*Dy1[0] + Q2[6]*Dy1[1] + Q2[7]*Dy1[2] + Q2[8]*Dy1[3] + Q2[9]*Dy1[4];
QDy1[2] = + Q2[10]*Dy1[0] + Q2[11]*Dy1[1] + Q2[12]*Dy1[2] + Q2[13]*Dy1[3] + Q2[14]*Dy1[4];
QDy1[3] = + Q2[15]*Dy1[0] + Q2[16]*Dy1[1] + Q2[17]*Dy1[2] + Q2[18]*Dy1[3] + Q2[19]*Dy1[4];
QDy1[4] = + Q2[20]*Dy1[0] + Q2[21]*Dy1[1] + Q2[22]*Dy1[2] + Q2[23]*Dy1[3] + Q2[24]*Dy1[4];
}

void acado_multEQDy( real_t* const E1, real_t* const QDy1, real_t* const U1 )
{
U1[0] += + E1[0]*QDy1[0] + E1[2]*QDy1[1] + E1[4]*QDy1[2] + E1[6]*QDy1[3] + E1[8]*QDy1[4];
U1[1] += + E1[1]*QDy1[0] + E1[3]*QDy1[1] + E1[5]*QDy1[2] + E1[7]*QDy1[3] + E1[9]*QDy1[4];
}

void acado_multQETGx( real_t* const E1, real_t* const Gx1, real_t* const H101 )
{
H101[0] += + E1[0]*Gx1[0] + E1[2]*Gx1[5] + E1[4]*Gx1[10] + E1[6]*Gx1[15] + E1[8]*Gx1[20];
H101[1] += + E1[0]*Gx1[1] + E1[2]*Gx1[6] + E1[4]*Gx1[11] + E1[6]*Gx1[16] + E1[8]*Gx1[21];
H101[2] += + E1[0]*Gx1[2] + E1[2]*Gx1[7] + E1[4]*Gx1[12] + E1[6]*Gx1[17] + E1[8]*Gx1[22];
H101[3] += + E1[0]*Gx1[3] + E1[2]*Gx1[8] + E1[4]*Gx1[13] + E1[6]*Gx1[18] + E1[8]*Gx1[23];
H101[4] += + E1[0]*Gx1[4] + E1[2]*Gx1[9] + E1[4]*Gx1[14] + E1[6]*Gx1[19] + E1[8]*Gx1[24];
H101[5] += + E1[1]*Gx1[0] + E1[3]*Gx1[5] + E1[5]*Gx1[10] + E1[7]*Gx1[15] + E1[9]*Gx1[20];
H101[6] += + E1[1]*Gx1[1] + E1[3]*Gx1[6] + E1[5]*Gx1[11] + E1[7]*Gx1[16] + E1[9]*Gx1[21];
H101[7] += + E1[1]*Gx1[2] + E1[3]*Gx1[7] + E1[5]*Gx1[12] + E1[7]*Gx1[17] + E1[9]*Gx1[22];
H101[8] += + E1[1]*Gx1[3] + E1[3]*Gx1[8] + E1[5]*Gx1[13] + E1[7]*Gx1[18] + E1[9]*Gx1[23];
H101[9] += + E1[1]*Gx1[4] + E1[3]*Gx1[9] + E1[5]*Gx1[14] + E1[7]*Gx1[19] + E1[9]*Gx1[24];
}

void acado_zeroBlockH10( real_t* const H101 )
{
{ int lCopy; for (lCopy = 0; lCopy < 10; lCopy++) H101[ lCopy ] = 0; }
}

void acado_multEDu( real_t* const E1, real_t* const U1, real_t* const dNew )
{
dNew[0] += + E1[0]*U1[0] + E1[1]*U1[1];
dNew[1] += + E1[2]*U1[0] + E1[3]*U1[1];
dNew[2] += + E1[4]*U1[0] + E1[5]*U1[1];
dNew[3] += + E1[6]*U1[0] + E1[7]*U1[1];
dNew[4] += + E1[8]*U1[0] + E1[9]*U1[1];
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
int lRun1;
int lRun2;
int lRun3;
int lRun4;
int lRun5;
/** Row vector of size: 30 */
static const int xBoundIndices[ 30 ] = 
{ 9, 14, 19, 24, 29, 34, 39, 44, 49, 54, 59, 64, 69, 74, 79, 84, 89, 94, 99, 104, 109, 114, 119, 124, 129, 134, 139, 144, 149, 154 };
acado_moveGuE( acadoWorkspace.evGu, acadoWorkspace.E );
for (lRun1 = 1; lRun1 < 30; ++lRun1)
{
acado_moveGxT( &(acadoWorkspace.evGx[ lRun1 * 25 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ lRun1 * 5-5 ]), &(acadoWorkspace.evGx[ lRun1 * 25 ]), &(acadoWorkspace.d[ lRun1 * 5 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ lRun1 * 25-25 ]), &(acadoWorkspace.evGx[ lRun1 * 25 ]) );
for (lRun2 = 0; lRun2 < lRun1; ++lRun2)
{
lRun4 = (((lRun1) * (lRun1-1)) / (2)) + (lRun2);
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ lRun4 * 10 ]), &(acadoWorkspace.E[ lRun3 * 10 ]) );
}
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_moveGuE( &(acadoWorkspace.evGu[ lRun1 * 10 ]), &(acadoWorkspace.E[ lRun3 * 10 ]) );
}

for (lRun1 = 0; lRun1 < 29; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_multGxGu( &(acadoWorkspace.Q1[ lRun1 * 25 + 25 ]), &(acadoWorkspace.E[ lRun3 * 10 ]), &(acadoWorkspace.QE[ lRun3 * 10 ]) );
}
}

for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ lRun3 * 10 ]), &(acadoWorkspace.QE[ lRun3 * 10 ]) );
}

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
acado_zeroBlockH10( &(acadoWorkspace.H10[ lRun1 * 10 ]) );
for (lRun2 = lRun1; lRun2 < 30; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
acado_multQETGx( &(acadoWorkspace.QE[ lRun3 * 10 ]), &(acadoWorkspace.evGx[ lRun2 * 25 ]), &(acadoWorkspace.H10[ lRun1 * 10 ]) );
}
}

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
acado_setBlockH11_R1( lRun1, lRun1 );
lRun2 = lRun1;
for (lRun3 = lRun1; lRun3 < 30; ++lRun3)
{
lRun4 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun1);
lRun5 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun2);
acado_setBlockH11( lRun1, lRun2, &(acadoWorkspace.E[ lRun4 * 10 ]), &(acadoWorkspace.QE[ lRun5 * 10 ]) );
}
for (lRun2 = lRun1 + 1; lRun2 < 30; ++lRun2)
{
acado_zeroBlockH11( lRun1, lRun2 );
for (lRun3 = lRun2; lRun3 < 30; ++lRun3)
{
lRun4 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun1);
lRun5 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun2);
acado_setBlockH11( lRun1, lRun2, &(acadoWorkspace.E[ lRun4 * 10 ]), &(acadoWorkspace.QE[ lRun5 * 10 ]) );
}
}
}

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1; ++lRun2)
{
acado_copyHTH( lRun1, lRun2 );
}
}

acado_multQ1d( &(acadoWorkspace.Q1[ 25 ]), acadoWorkspace.d, acadoWorkspace.Qd );
acado_multQ1d( &(acadoWorkspace.Q1[ 50 ]), &(acadoWorkspace.d[ 5 ]), &(acadoWorkspace.Qd[ 5 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 75 ]), &(acadoWorkspace.d[ 10 ]), &(acadoWorkspace.Qd[ 10 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 100 ]), &(acadoWorkspace.d[ 15 ]), &(acadoWorkspace.Qd[ 15 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 125 ]), &(acadoWorkspace.d[ 20 ]), &(acadoWorkspace.Qd[ 20 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 150 ]), &(acadoWorkspace.d[ 25 ]), &(acadoWorkspace.Qd[ 25 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 175 ]), &(acadoWorkspace.d[ 30 ]), &(acadoWorkspace.Qd[ 30 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 200 ]), &(acadoWorkspace.d[ 35 ]), &(acadoWorkspace.Qd[ 35 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 225 ]), &(acadoWorkspace.d[ 40 ]), &(acadoWorkspace.Qd[ 40 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 250 ]), &(acadoWorkspace.d[ 45 ]), &(acadoWorkspace.Qd[ 45 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 275 ]), &(acadoWorkspace.d[ 50 ]), &(acadoWorkspace.Qd[ 50 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 300 ]), &(acadoWorkspace.d[ 55 ]), &(acadoWorkspace.Qd[ 55 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 325 ]), &(acadoWorkspace.d[ 60 ]), &(acadoWorkspace.Qd[ 60 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 350 ]), &(acadoWorkspace.d[ 65 ]), &(acadoWorkspace.Qd[ 65 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 375 ]), &(acadoWorkspace.d[ 70 ]), &(acadoWorkspace.Qd[ 70 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 400 ]), &(acadoWorkspace.d[ 75 ]), &(acadoWorkspace.Qd[ 75 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 425 ]), &(acadoWorkspace.d[ 80 ]), &(acadoWorkspace.Qd[ 80 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 450 ]), &(acadoWorkspace.d[ 85 ]), &(acadoWorkspace.Qd[ 85 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 475 ]), &(acadoWorkspace.d[ 90 ]), &(acadoWorkspace.Qd[ 90 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 500 ]), &(acadoWorkspace.d[ 95 ]), &(acadoWorkspace.Qd[ 95 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 525 ]), &(acadoWorkspace.d[ 100 ]), &(acadoWorkspace.Qd[ 100 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 550 ]), &(acadoWorkspace.d[ 105 ]), &(acadoWorkspace.Qd[ 105 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 575 ]), &(acadoWorkspace.d[ 110 ]), &(acadoWorkspace.Qd[ 110 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 600 ]), &(acadoWorkspace.d[ 115 ]), &(acadoWorkspace.Qd[ 115 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 625 ]), &(acadoWorkspace.d[ 120 ]), &(acadoWorkspace.Qd[ 120 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 650 ]), &(acadoWorkspace.d[ 125 ]), &(acadoWorkspace.Qd[ 125 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 675 ]), &(acadoWorkspace.d[ 130 ]), &(acadoWorkspace.Qd[ 130 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 700 ]), &(acadoWorkspace.d[ 135 ]), &(acadoWorkspace.Qd[ 135 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 725 ]), &(acadoWorkspace.d[ 140 ]), &(acadoWorkspace.Qd[ 140 ]) );
acado_multQN1d( acadoWorkspace.QN1, &(acadoWorkspace.d[ 145 ]), &(acadoWorkspace.Qd[ 145 ]) );

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
for (lRun2 = lRun1; lRun2 < 30; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
acado_macETSlu( &(acadoWorkspace.QE[ lRun3 * 10 ]), &(acadoWorkspace.g[ lRun1 * 2 ]) );
}
}
for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
lRun3 = xBoundIndices[ lRun1 ] - 5;
lRun4 = ((lRun3) / (5)) + (1);
for (lRun2 = 0; lRun2 < lRun4; ++lRun2)
{
lRun5 = (((((lRun4) * (lRun4-1)) / (2)) + (lRun2)) * (5)) + ((lRun3) % (5));
acadoWorkspace.A[(lRun1 * 60) + (lRun2 * 2)] = acadoWorkspace.E[lRun5 * 2];
acadoWorkspace.A[(lRun1 * 60) + (lRun2 * 2 + 1)] = acadoWorkspace.E[lRun5 * 2 + 1];
}
}

}

void acado_condenseFdb(  )
{
int lRun1;
int lRun2;
int lRun3;
real_t tmp;

acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
acadoWorkspace.Dx0[1] = acadoVariables.x0[1] - acadoVariables.x[1];
acadoWorkspace.Dx0[2] = acadoVariables.x0[2] - acadoVariables.x[2];
acadoWorkspace.Dx0[3] = acadoVariables.x0[3] - acadoVariables.x[3];
acadoWorkspace.Dx0[4] = acadoVariables.x0[4] - acadoVariables.x[4];

for (lRun2 = 0; lRun2 < 150; ++lRun2)
acadoWorkspace.Dy[lRun2] -= acadoVariables.y[lRun2];

acadoWorkspace.DyN[0] -= acadoVariables.yN[0];
acadoWorkspace.DyN[1] -= acadoVariables.yN[1];
acadoWorkspace.DyN[2] -= acadoVariables.yN[2];
acadoWorkspace.DyN[3] -= acadoVariables.yN[3];
acadoWorkspace.DyN[4] -= acadoVariables.yN[4];

acado_multRDy( acadoWorkspace.Dy, acadoWorkspace.g );
acado_multRDy( &(acadoWorkspace.Dy[ 5 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 10 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 15 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 20 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 25 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 30 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 35 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 40 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 45 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 50 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 55 ]), &(acadoWorkspace.g[ 22 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 60 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 65 ]), &(acadoWorkspace.g[ 26 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 70 ]), &(acadoWorkspace.g[ 28 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 75 ]), &(acadoWorkspace.g[ 30 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 80 ]), &(acadoWorkspace.g[ 32 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 85 ]), &(acadoWorkspace.g[ 34 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 90 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 95 ]), &(acadoWorkspace.g[ 38 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 100 ]), &(acadoWorkspace.g[ 40 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 105 ]), &(acadoWorkspace.g[ 42 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 110 ]), &(acadoWorkspace.g[ 44 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 115 ]), &(acadoWorkspace.g[ 46 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 120 ]), &(acadoWorkspace.g[ 48 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 125 ]), &(acadoWorkspace.g[ 50 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 130 ]), &(acadoWorkspace.g[ 52 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 135 ]), &(acadoWorkspace.g[ 54 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 140 ]), &(acadoWorkspace.g[ 56 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 145 ]), &(acadoWorkspace.g[ 58 ]) );

acado_multQDy( acadoWorkspace.Q2, acadoWorkspace.Dy, acadoWorkspace.QDy );
acado_multQDy( &(acadoWorkspace.Q2[ 25 ]), &(acadoWorkspace.Dy[ 5 ]), &(acadoWorkspace.QDy[ 5 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 50 ]), &(acadoWorkspace.Dy[ 10 ]), &(acadoWorkspace.QDy[ 10 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 75 ]), &(acadoWorkspace.Dy[ 15 ]), &(acadoWorkspace.QDy[ 15 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 100 ]), &(acadoWorkspace.Dy[ 20 ]), &(acadoWorkspace.QDy[ 20 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 125 ]), &(acadoWorkspace.Dy[ 25 ]), &(acadoWorkspace.QDy[ 25 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 150 ]), &(acadoWorkspace.Dy[ 30 ]), &(acadoWorkspace.QDy[ 30 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 175 ]), &(acadoWorkspace.Dy[ 35 ]), &(acadoWorkspace.QDy[ 35 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 200 ]), &(acadoWorkspace.Dy[ 40 ]), &(acadoWorkspace.QDy[ 40 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 225 ]), &(acadoWorkspace.Dy[ 45 ]), &(acadoWorkspace.QDy[ 45 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 250 ]), &(acadoWorkspace.Dy[ 50 ]), &(acadoWorkspace.QDy[ 50 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 275 ]), &(acadoWorkspace.Dy[ 55 ]), &(acadoWorkspace.QDy[ 55 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 300 ]), &(acadoWorkspace.Dy[ 60 ]), &(acadoWorkspace.QDy[ 60 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 325 ]), &(acadoWorkspace.Dy[ 65 ]), &(acadoWorkspace.QDy[ 65 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 350 ]), &(acadoWorkspace.Dy[ 70 ]), &(acadoWorkspace.QDy[ 70 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 375 ]), &(acadoWorkspace.Dy[ 75 ]), &(acadoWorkspace.QDy[ 75 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 400 ]), &(acadoWorkspace.Dy[ 80 ]), &(acadoWorkspace.QDy[ 80 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 425 ]), &(acadoWorkspace.Dy[ 85 ]), &(acadoWorkspace.QDy[ 85 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 450 ]), &(acadoWorkspace.Dy[ 90 ]), &(acadoWorkspace.QDy[ 90 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 475 ]), &(acadoWorkspace.Dy[ 95 ]), &(acadoWorkspace.QDy[ 95 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 500 ]), &(acadoWorkspace.Dy[ 100 ]), &(acadoWorkspace.QDy[ 100 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 525 ]), &(acadoWorkspace.Dy[ 105 ]), &(acadoWorkspace.QDy[ 105 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 550 ]), &(acadoWorkspace.Dy[ 110 ]), &(acadoWorkspace.QDy[ 110 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 575 ]), &(acadoWorkspace.Dy[ 115 ]), &(acadoWorkspace.QDy[ 115 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 600 ]), &(acadoWorkspace.Dy[ 120 ]), &(acadoWorkspace.QDy[ 120 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 625 ]), &(acadoWorkspace.Dy[ 125 ]), &(acadoWorkspace.QDy[ 125 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 650 ]), &(acadoWorkspace.Dy[ 130 ]), &(acadoWorkspace.QDy[ 130 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 675 ]), &(acadoWorkspace.Dy[ 135 ]), &(acadoWorkspace.QDy[ 135 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 700 ]), &(acadoWorkspace.Dy[ 140 ]), &(acadoWorkspace.QDy[ 140 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 725 ]), &(acadoWorkspace.Dy[ 145 ]), &(acadoWorkspace.QDy[ 145 ]) );

acadoWorkspace.QDy[150] = + acadoWorkspace.QN2[0]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[1]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[2]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[3]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[4]*acadoWorkspace.DyN[4];
acadoWorkspace.QDy[151] = + acadoWorkspace.QN2[5]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[6]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[7]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[8]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[9]*acadoWorkspace.DyN[4];
acadoWorkspace.QDy[152] = + acadoWorkspace.QN2[10]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[11]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[12]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[13]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[14]*acadoWorkspace.DyN[4];
acadoWorkspace.QDy[153] = + acadoWorkspace.QN2[15]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[16]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[17]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[18]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[19]*acadoWorkspace.DyN[4];
acadoWorkspace.QDy[154] = + acadoWorkspace.QN2[20]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[21]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[22]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[23]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[24]*acadoWorkspace.DyN[4];

for (lRun2 = 0; lRun2 < 150; ++lRun2)
acadoWorkspace.QDy[lRun2 + 5] += acadoWorkspace.Qd[lRun2];


for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
for (lRun2 = lRun1; lRun2 < 30; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
acado_multEQDy( &(acadoWorkspace.E[ lRun3 * 10 ]), &(acadoWorkspace.QDy[ lRun2 * 5 + 5 ]), &(acadoWorkspace.g[ lRun1 * 2 ]) );
}
}

acadoWorkspace.g[0] += + acadoWorkspace.H10[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[2]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[3]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[4]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[1] += + acadoWorkspace.H10[5]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[6]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[7]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[8]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[9]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[2] += + acadoWorkspace.H10[10]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[11]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[12]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[13]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[14]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[3] += + acadoWorkspace.H10[15]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[16]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[17]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[18]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[19]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[4] += + acadoWorkspace.H10[20]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[21]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[22]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[23]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[24]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[5] += + acadoWorkspace.H10[25]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[26]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[27]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[28]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[29]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[6] += + acadoWorkspace.H10[30]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[31]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[32]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[33]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[34]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[7] += + acadoWorkspace.H10[35]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[36]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[37]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[38]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[39]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[8] += + acadoWorkspace.H10[40]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[41]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[42]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[43]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[44]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[9] += + acadoWorkspace.H10[45]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[46]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[47]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[48]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[49]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[10] += + acadoWorkspace.H10[50]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[51]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[52]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[53]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[54]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[11] += + acadoWorkspace.H10[55]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[56]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[57]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[58]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[59]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[12] += + acadoWorkspace.H10[60]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[61]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[62]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[63]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[64]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[13] += + acadoWorkspace.H10[65]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[66]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[67]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[68]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[69]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[14] += + acadoWorkspace.H10[70]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[71]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[72]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[73]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[74]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[15] += + acadoWorkspace.H10[75]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[76]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[77]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[78]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[79]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[16] += + acadoWorkspace.H10[80]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[81]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[82]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[83]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[84]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[17] += + acadoWorkspace.H10[85]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[86]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[87]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[88]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[89]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[18] += + acadoWorkspace.H10[90]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[91]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[92]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[93]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[94]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[19] += + acadoWorkspace.H10[95]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[96]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[97]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[98]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[99]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[20] += + acadoWorkspace.H10[100]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[101]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[102]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[103]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[104]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[21] += + acadoWorkspace.H10[105]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[106]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[107]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[108]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[109]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[22] += + acadoWorkspace.H10[110]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[111]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[112]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[113]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[114]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[23] += + acadoWorkspace.H10[115]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[116]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[117]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[118]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[119]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[24] += + acadoWorkspace.H10[120]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[121]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[122]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[123]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[124]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[25] += + acadoWorkspace.H10[125]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[126]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[127]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[128]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[129]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[26] += + acadoWorkspace.H10[130]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[131]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[132]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[133]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[134]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[27] += + acadoWorkspace.H10[135]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[136]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[137]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[138]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[139]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[28] += + acadoWorkspace.H10[140]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[141]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[142]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[143]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[144]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[29] += + acadoWorkspace.H10[145]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[146]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[147]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[148]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[149]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[30] += + acadoWorkspace.H10[150]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[151]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[152]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[153]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[154]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[31] += + acadoWorkspace.H10[155]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[156]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[157]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[158]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[159]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[32] += + acadoWorkspace.H10[160]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[161]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[162]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[163]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[164]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[33] += + acadoWorkspace.H10[165]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[166]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[167]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[168]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[169]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[34] += + acadoWorkspace.H10[170]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[171]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[172]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[173]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[174]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[35] += + acadoWorkspace.H10[175]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[176]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[177]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[178]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[179]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[36] += + acadoWorkspace.H10[180]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[181]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[182]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[183]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[184]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[37] += + acadoWorkspace.H10[185]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[186]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[187]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[188]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[189]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[38] += + acadoWorkspace.H10[190]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[191]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[192]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[193]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[194]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[39] += + acadoWorkspace.H10[195]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[196]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[197]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[198]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[199]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[40] += + acadoWorkspace.H10[200]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[201]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[202]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[203]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[204]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[41] += + acadoWorkspace.H10[205]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[206]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[207]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[208]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[209]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[42] += + acadoWorkspace.H10[210]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[211]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[212]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[213]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[214]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[43] += + acadoWorkspace.H10[215]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[216]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[217]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[218]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[219]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[44] += + acadoWorkspace.H10[220]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[221]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[222]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[223]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[224]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[45] += + acadoWorkspace.H10[225]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[226]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[227]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[228]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[229]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[46] += + acadoWorkspace.H10[230]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[231]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[232]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[233]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[234]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[47] += + acadoWorkspace.H10[235]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[236]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[237]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[238]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[239]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[48] += + acadoWorkspace.H10[240]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[241]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[242]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[243]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[244]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[49] += + acadoWorkspace.H10[245]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[246]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[247]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[248]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[249]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[50] += + acadoWorkspace.H10[250]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[251]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[252]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[253]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[254]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[51] += + acadoWorkspace.H10[255]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[256]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[257]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[258]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[259]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[52] += + acadoWorkspace.H10[260]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[261]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[262]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[263]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[264]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[53] += + acadoWorkspace.H10[265]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[266]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[267]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[268]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[269]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[54] += + acadoWorkspace.H10[270]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[271]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[272]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[273]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[274]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[55] += + acadoWorkspace.H10[275]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[276]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[277]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[278]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[279]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[56] += + acadoWorkspace.H10[280]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[281]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[282]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[283]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[284]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[57] += + acadoWorkspace.H10[285]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[286]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[287]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[288]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[289]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[58] += + acadoWorkspace.H10[290]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[291]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[292]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[293]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[294]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[59] += + acadoWorkspace.H10[295]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[296]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[297]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[298]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[299]*acadoWorkspace.Dx0[4];

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
acadoWorkspace.lb[20] = acadoVariables.lbValues[20] - acadoVariables.u[20];
acadoWorkspace.lb[21] = acadoVariables.lbValues[21] - acadoVariables.u[21];
acadoWorkspace.lb[22] = acadoVariables.lbValues[22] - acadoVariables.u[22];
acadoWorkspace.lb[23] = acadoVariables.lbValues[23] - acadoVariables.u[23];
acadoWorkspace.lb[24] = acadoVariables.lbValues[24] - acadoVariables.u[24];
acadoWorkspace.lb[25] = acadoVariables.lbValues[25] - acadoVariables.u[25];
acadoWorkspace.lb[26] = acadoVariables.lbValues[26] - acadoVariables.u[26];
acadoWorkspace.lb[27] = acadoVariables.lbValues[27] - acadoVariables.u[27];
acadoWorkspace.lb[28] = acadoVariables.lbValues[28] - acadoVariables.u[28];
acadoWorkspace.lb[29] = acadoVariables.lbValues[29] - acadoVariables.u[29];
acadoWorkspace.lb[30] = acadoVariables.lbValues[30] - acadoVariables.u[30];
acadoWorkspace.lb[31] = acadoVariables.lbValues[31] - acadoVariables.u[31];
acadoWorkspace.lb[32] = acadoVariables.lbValues[32] - acadoVariables.u[32];
acadoWorkspace.lb[33] = acadoVariables.lbValues[33] - acadoVariables.u[33];
acadoWorkspace.lb[34] = acadoVariables.lbValues[34] - acadoVariables.u[34];
acadoWorkspace.lb[35] = acadoVariables.lbValues[35] - acadoVariables.u[35];
acadoWorkspace.lb[36] = acadoVariables.lbValues[36] - acadoVariables.u[36];
acadoWorkspace.lb[37] = acadoVariables.lbValues[37] - acadoVariables.u[37];
acadoWorkspace.lb[38] = acadoVariables.lbValues[38] - acadoVariables.u[38];
acadoWorkspace.lb[39] = acadoVariables.lbValues[39] - acadoVariables.u[39];
acadoWorkspace.lb[40] = acadoVariables.lbValues[40] - acadoVariables.u[40];
acadoWorkspace.lb[41] = acadoVariables.lbValues[41] - acadoVariables.u[41];
acadoWorkspace.lb[42] = acadoVariables.lbValues[42] - acadoVariables.u[42];
acadoWorkspace.lb[43] = acadoVariables.lbValues[43] - acadoVariables.u[43];
acadoWorkspace.lb[44] = acadoVariables.lbValues[44] - acadoVariables.u[44];
acadoWorkspace.lb[45] = acadoVariables.lbValues[45] - acadoVariables.u[45];
acadoWorkspace.lb[46] = acadoVariables.lbValues[46] - acadoVariables.u[46];
acadoWorkspace.lb[47] = acadoVariables.lbValues[47] - acadoVariables.u[47];
acadoWorkspace.lb[48] = acadoVariables.lbValues[48] - acadoVariables.u[48];
acadoWorkspace.lb[49] = acadoVariables.lbValues[49] - acadoVariables.u[49];
acadoWorkspace.lb[50] = acadoVariables.lbValues[50] - acadoVariables.u[50];
acadoWorkspace.lb[51] = acadoVariables.lbValues[51] - acadoVariables.u[51];
acadoWorkspace.lb[52] = acadoVariables.lbValues[52] - acadoVariables.u[52];
acadoWorkspace.lb[53] = acadoVariables.lbValues[53] - acadoVariables.u[53];
acadoWorkspace.lb[54] = acadoVariables.lbValues[54] - acadoVariables.u[54];
acadoWorkspace.lb[55] = acadoVariables.lbValues[55] - acadoVariables.u[55];
acadoWorkspace.lb[56] = acadoVariables.lbValues[56] - acadoVariables.u[56];
acadoWorkspace.lb[57] = acadoVariables.lbValues[57] - acadoVariables.u[57];
acadoWorkspace.lb[58] = acadoVariables.lbValues[58] - acadoVariables.u[58];
acadoWorkspace.lb[59] = acadoVariables.lbValues[59] - acadoVariables.u[59];
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
acadoWorkspace.ub[20] = acadoVariables.ubValues[20] - acadoVariables.u[20];
acadoWorkspace.ub[21] = acadoVariables.ubValues[21] - acadoVariables.u[21];
acadoWorkspace.ub[22] = acadoVariables.ubValues[22] - acadoVariables.u[22];
acadoWorkspace.ub[23] = acadoVariables.ubValues[23] - acadoVariables.u[23];
acadoWorkspace.ub[24] = acadoVariables.ubValues[24] - acadoVariables.u[24];
acadoWorkspace.ub[25] = acadoVariables.ubValues[25] - acadoVariables.u[25];
acadoWorkspace.ub[26] = acadoVariables.ubValues[26] - acadoVariables.u[26];
acadoWorkspace.ub[27] = acadoVariables.ubValues[27] - acadoVariables.u[27];
acadoWorkspace.ub[28] = acadoVariables.ubValues[28] - acadoVariables.u[28];
acadoWorkspace.ub[29] = acadoVariables.ubValues[29] - acadoVariables.u[29];
acadoWorkspace.ub[30] = acadoVariables.ubValues[30] - acadoVariables.u[30];
acadoWorkspace.ub[31] = acadoVariables.ubValues[31] - acadoVariables.u[31];
acadoWorkspace.ub[32] = acadoVariables.ubValues[32] - acadoVariables.u[32];
acadoWorkspace.ub[33] = acadoVariables.ubValues[33] - acadoVariables.u[33];
acadoWorkspace.ub[34] = acadoVariables.ubValues[34] - acadoVariables.u[34];
acadoWorkspace.ub[35] = acadoVariables.ubValues[35] - acadoVariables.u[35];
acadoWorkspace.ub[36] = acadoVariables.ubValues[36] - acadoVariables.u[36];
acadoWorkspace.ub[37] = acadoVariables.ubValues[37] - acadoVariables.u[37];
acadoWorkspace.ub[38] = acadoVariables.ubValues[38] - acadoVariables.u[38];
acadoWorkspace.ub[39] = acadoVariables.ubValues[39] - acadoVariables.u[39];
acadoWorkspace.ub[40] = acadoVariables.ubValues[40] - acadoVariables.u[40];
acadoWorkspace.ub[41] = acadoVariables.ubValues[41] - acadoVariables.u[41];
acadoWorkspace.ub[42] = acadoVariables.ubValues[42] - acadoVariables.u[42];
acadoWorkspace.ub[43] = acadoVariables.ubValues[43] - acadoVariables.u[43];
acadoWorkspace.ub[44] = acadoVariables.ubValues[44] - acadoVariables.u[44];
acadoWorkspace.ub[45] = acadoVariables.ubValues[45] - acadoVariables.u[45];
acadoWorkspace.ub[46] = acadoVariables.ubValues[46] - acadoVariables.u[46];
acadoWorkspace.ub[47] = acadoVariables.ubValues[47] - acadoVariables.u[47];
acadoWorkspace.ub[48] = acadoVariables.ubValues[48] - acadoVariables.u[48];
acadoWorkspace.ub[49] = acadoVariables.ubValues[49] - acadoVariables.u[49];
acadoWorkspace.ub[50] = acadoVariables.ubValues[50] - acadoVariables.u[50];
acadoWorkspace.ub[51] = acadoVariables.ubValues[51] - acadoVariables.u[51];
acadoWorkspace.ub[52] = acadoVariables.ubValues[52] - acadoVariables.u[52];
acadoWorkspace.ub[53] = acadoVariables.ubValues[53] - acadoVariables.u[53];
acadoWorkspace.ub[54] = acadoVariables.ubValues[54] - acadoVariables.u[54];
acadoWorkspace.ub[55] = acadoVariables.ubValues[55] - acadoVariables.u[55];
acadoWorkspace.ub[56] = acadoVariables.ubValues[56] - acadoVariables.u[56];
acadoWorkspace.ub[57] = acadoVariables.ubValues[57] - acadoVariables.u[57];
acadoWorkspace.ub[58] = acadoVariables.ubValues[58] - acadoVariables.u[58];
acadoWorkspace.ub[59] = acadoVariables.ubValues[59] - acadoVariables.u[59];

tmp = + acadoWorkspace.evGx[20]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[21]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[22]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[23]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[24]*acadoWorkspace.Dx0[4] + acadoVariables.x[9];
tmp += acadoWorkspace.d[4];
acadoWorkspace.lbA[0] = acadoVariables.lbAValues[0] - tmp;
acadoWorkspace.ubA[0] = acadoVariables.ubAValues[0] - tmp;
tmp = + acadoWorkspace.evGx[45]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[46]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[47]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[48]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[49]*acadoWorkspace.Dx0[4] + acadoVariables.x[14];
tmp += acadoWorkspace.d[9];
acadoWorkspace.lbA[1] = acadoVariables.lbAValues[1] - tmp;
acadoWorkspace.ubA[1] = acadoVariables.ubAValues[1] - tmp;
tmp = + acadoWorkspace.evGx[70]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[71]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[72]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[73]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[74]*acadoWorkspace.Dx0[4] + acadoVariables.x[19];
tmp += acadoWorkspace.d[14];
acadoWorkspace.lbA[2] = acadoVariables.lbAValues[2] - tmp;
acadoWorkspace.ubA[2] = acadoVariables.ubAValues[2] - tmp;
tmp = + acadoWorkspace.evGx[95]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[96]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[97]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[98]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[99]*acadoWorkspace.Dx0[4] + acadoVariables.x[24];
tmp += acadoWorkspace.d[19];
acadoWorkspace.lbA[3] = acadoVariables.lbAValues[3] - tmp;
acadoWorkspace.ubA[3] = acadoVariables.ubAValues[3] - tmp;
tmp = + acadoWorkspace.evGx[120]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[121]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[122]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[123]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[124]*acadoWorkspace.Dx0[4] + acadoVariables.x[29];
tmp += acadoWorkspace.d[24];
acadoWorkspace.lbA[4] = acadoVariables.lbAValues[4] - tmp;
acadoWorkspace.ubA[4] = acadoVariables.ubAValues[4] - tmp;
tmp = + acadoWorkspace.evGx[145]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[146]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[147]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[148]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[149]*acadoWorkspace.Dx0[4] + acadoVariables.x[34];
tmp += acadoWorkspace.d[29];
acadoWorkspace.lbA[5] = acadoVariables.lbAValues[5] - tmp;
acadoWorkspace.ubA[5] = acadoVariables.ubAValues[5] - tmp;
tmp = + acadoWorkspace.evGx[170]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[171]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[172]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[173]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[174]*acadoWorkspace.Dx0[4] + acadoVariables.x[39];
tmp += acadoWorkspace.d[34];
acadoWorkspace.lbA[6] = acadoVariables.lbAValues[6] - tmp;
acadoWorkspace.ubA[6] = acadoVariables.ubAValues[6] - tmp;
tmp = + acadoWorkspace.evGx[195]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[196]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[197]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[198]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[199]*acadoWorkspace.Dx0[4] + acadoVariables.x[44];
tmp += acadoWorkspace.d[39];
acadoWorkspace.lbA[7] = acadoVariables.lbAValues[7] - tmp;
acadoWorkspace.ubA[7] = acadoVariables.ubAValues[7] - tmp;
tmp = + acadoWorkspace.evGx[220]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[221]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[222]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[223]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[224]*acadoWorkspace.Dx0[4] + acadoVariables.x[49];
tmp += acadoWorkspace.d[44];
acadoWorkspace.lbA[8] = acadoVariables.lbAValues[8] - tmp;
acadoWorkspace.ubA[8] = acadoVariables.ubAValues[8] - tmp;
tmp = + acadoWorkspace.evGx[245]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[246]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[247]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[248]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[249]*acadoWorkspace.Dx0[4] + acadoVariables.x[54];
tmp += acadoWorkspace.d[49];
acadoWorkspace.lbA[9] = acadoVariables.lbAValues[9] - tmp;
acadoWorkspace.ubA[9] = acadoVariables.ubAValues[9] - tmp;
tmp = + acadoWorkspace.evGx[270]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[271]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[272]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[273]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[274]*acadoWorkspace.Dx0[4] + acadoVariables.x[59];
tmp += acadoWorkspace.d[54];
acadoWorkspace.lbA[10] = acadoVariables.lbAValues[10] - tmp;
acadoWorkspace.ubA[10] = acadoVariables.ubAValues[10] - tmp;
tmp = + acadoWorkspace.evGx[295]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[296]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[297]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[298]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[299]*acadoWorkspace.Dx0[4] + acadoVariables.x[64];
tmp += acadoWorkspace.d[59];
acadoWorkspace.lbA[11] = acadoVariables.lbAValues[11] - tmp;
acadoWorkspace.ubA[11] = acadoVariables.ubAValues[11] - tmp;
tmp = + acadoWorkspace.evGx[320]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[321]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[322]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[323]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[324]*acadoWorkspace.Dx0[4] + acadoVariables.x[69];
tmp += acadoWorkspace.d[64];
acadoWorkspace.lbA[12] = acadoVariables.lbAValues[12] - tmp;
acadoWorkspace.ubA[12] = acadoVariables.ubAValues[12] - tmp;
tmp = + acadoWorkspace.evGx[345]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[346]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[347]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[348]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[349]*acadoWorkspace.Dx0[4] + acadoVariables.x[74];
tmp += acadoWorkspace.d[69];
acadoWorkspace.lbA[13] = acadoVariables.lbAValues[13] - tmp;
acadoWorkspace.ubA[13] = acadoVariables.ubAValues[13] - tmp;
tmp = + acadoWorkspace.evGx[370]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[371]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[372]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[373]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[374]*acadoWorkspace.Dx0[4] + acadoVariables.x[79];
tmp += acadoWorkspace.d[74];
acadoWorkspace.lbA[14] = acadoVariables.lbAValues[14] - tmp;
acadoWorkspace.ubA[14] = acadoVariables.ubAValues[14] - tmp;
tmp = + acadoWorkspace.evGx[395]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[396]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[397]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[398]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[399]*acadoWorkspace.Dx0[4] + acadoVariables.x[84];
tmp += acadoWorkspace.d[79];
acadoWorkspace.lbA[15] = acadoVariables.lbAValues[15] - tmp;
acadoWorkspace.ubA[15] = acadoVariables.ubAValues[15] - tmp;
tmp = + acadoWorkspace.evGx[420]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[421]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[422]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[423]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[424]*acadoWorkspace.Dx0[4] + acadoVariables.x[89];
tmp += acadoWorkspace.d[84];
acadoWorkspace.lbA[16] = acadoVariables.lbAValues[16] - tmp;
acadoWorkspace.ubA[16] = acadoVariables.ubAValues[16] - tmp;
tmp = + acadoWorkspace.evGx[445]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[446]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[447]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[448]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[449]*acadoWorkspace.Dx0[4] + acadoVariables.x[94];
tmp += acadoWorkspace.d[89];
acadoWorkspace.lbA[17] = acadoVariables.lbAValues[17] - tmp;
acadoWorkspace.ubA[17] = acadoVariables.ubAValues[17] - tmp;
tmp = + acadoWorkspace.evGx[470]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[471]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[472]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[473]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[474]*acadoWorkspace.Dx0[4] + acadoVariables.x[99];
tmp += acadoWorkspace.d[94];
acadoWorkspace.lbA[18] = acadoVariables.lbAValues[18] - tmp;
acadoWorkspace.ubA[18] = acadoVariables.ubAValues[18] - tmp;
tmp = + acadoWorkspace.evGx[495]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[496]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[497]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[498]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[499]*acadoWorkspace.Dx0[4] + acadoVariables.x[104];
tmp += acadoWorkspace.d[99];
acadoWorkspace.lbA[19] = acadoVariables.lbAValues[19] - tmp;
acadoWorkspace.ubA[19] = acadoVariables.ubAValues[19] - tmp;
tmp = + acadoWorkspace.evGx[520]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[521]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[522]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[523]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[524]*acadoWorkspace.Dx0[4] + acadoVariables.x[109];
tmp += acadoWorkspace.d[104];
acadoWorkspace.lbA[20] = acadoVariables.lbAValues[20] - tmp;
acadoWorkspace.ubA[20] = acadoVariables.ubAValues[20] - tmp;
tmp = + acadoWorkspace.evGx[545]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[546]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[547]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[548]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[549]*acadoWorkspace.Dx0[4] + acadoVariables.x[114];
tmp += acadoWorkspace.d[109];
acadoWorkspace.lbA[21] = acadoVariables.lbAValues[21] - tmp;
acadoWorkspace.ubA[21] = acadoVariables.ubAValues[21] - tmp;
tmp = + acadoWorkspace.evGx[570]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[571]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[572]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[573]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[574]*acadoWorkspace.Dx0[4] + acadoVariables.x[119];
tmp += acadoWorkspace.d[114];
acadoWorkspace.lbA[22] = acadoVariables.lbAValues[22] - tmp;
acadoWorkspace.ubA[22] = acadoVariables.ubAValues[22] - tmp;
tmp = + acadoWorkspace.evGx[595]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[596]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[597]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[598]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[599]*acadoWorkspace.Dx0[4] + acadoVariables.x[124];
tmp += acadoWorkspace.d[119];
acadoWorkspace.lbA[23] = acadoVariables.lbAValues[23] - tmp;
acadoWorkspace.ubA[23] = acadoVariables.ubAValues[23] - tmp;
tmp = + acadoWorkspace.evGx[620]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[621]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[622]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[623]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[624]*acadoWorkspace.Dx0[4] + acadoVariables.x[129];
tmp += acadoWorkspace.d[124];
acadoWorkspace.lbA[24] = acadoVariables.lbAValues[24] - tmp;
acadoWorkspace.ubA[24] = acadoVariables.ubAValues[24] - tmp;
tmp = + acadoWorkspace.evGx[645]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[646]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[647]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[648]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[649]*acadoWorkspace.Dx0[4] + acadoVariables.x[134];
tmp += acadoWorkspace.d[129];
acadoWorkspace.lbA[25] = acadoVariables.lbAValues[25] - tmp;
acadoWorkspace.ubA[25] = acadoVariables.ubAValues[25] - tmp;
tmp = + acadoWorkspace.evGx[670]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[671]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[672]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[673]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[674]*acadoWorkspace.Dx0[4] + acadoVariables.x[139];
tmp += acadoWorkspace.d[134];
acadoWorkspace.lbA[26] = acadoVariables.lbAValues[26] - tmp;
acadoWorkspace.ubA[26] = acadoVariables.ubAValues[26] - tmp;
tmp = + acadoWorkspace.evGx[695]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[696]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[697]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[698]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[699]*acadoWorkspace.Dx0[4] + acadoVariables.x[144];
tmp += acadoWorkspace.d[139];
acadoWorkspace.lbA[27] = acadoVariables.lbAValues[27] - tmp;
acadoWorkspace.ubA[27] = acadoVariables.ubAValues[27] - tmp;
tmp = + acadoWorkspace.evGx[720]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[721]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[722]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[723]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[724]*acadoWorkspace.Dx0[4] + acadoVariables.x[149];
tmp += acadoWorkspace.d[144];
acadoWorkspace.lbA[28] = acadoVariables.lbAValues[28] - tmp;
acadoWorkspace.ubA[28] = acadoVariables.ubAValues[28] - tmp;
tmp = + acadoWorkspace.evGx[745]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[746]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[747]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[748]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[749]*acadoWorkspace.Dx0[4] + acadoVariables.x[154];
tmp += acadoWorkspace.d[149];
acadoWorkspace.lbA[29] = acadoVariables.lbAValues[29] - tmp;
acadoWorkspace.ubA[29] = acadoVariables.ubAValues[29] - tmp;

}

void acado_expand(  )
{
int lRun1;
int lRun2;
int lRun3;
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
acadoVariables.u[40] += acadoWorkspace.x[40];
acadoVariables.u[41] += acadoWorkspace.x[41];
acadoVariables.u[42] += acadoWorkspace.x[42];
acadoVariables.u[43] += acadoWorkspace.x[43];
acadoVariables.u[44] += acadoWorkspace.x[44];
acadoVariables.u[45] += acadoWorkspace.x[45];
acadoVariables.u[46] += acadoWorkspace.x[46];
acadoVariables.u[47] += acadoWorkspace.x[47];
acadoVariables.u[48] += acadoWorkspace.x[48];
acadoVariables.u[49] += acadoWorkspace.x[49];
acadoVariables.u[50] += acadoWorkspace.x[50];
acadoVariables.u[51] += acadoWorkspace.x[51];
acadoVariables.u[52] += acadoWorkspace.x[52];
acadoVariables.u[53] += acadoWorkspace.x[53];
acadoVariables.u[54] += acadoWorkspace.x[54];
acadoVariables.u[55] += acadoWorkspace.x[55];
acadoVariables.u[56] += acadoWorkspace.x[56];
acadoVariables.u[57] += acadoWorkspace.x[57];
acadoVariables.u[58] += acadoWorkspace.x[58];
acadoVariables.u[59] += acadoWorkspace.x[59];

acadoVariables.x[0] += acadoWorkspace.Dx0[0];
acadoVariables.x[1] += acadoWorkspace.Dx0[1];
acadoVariables.x[2] += acadoWorkspace.Dx0[2];
acadoVariables.x[3] += acadoWorkspace.Dx0[3];
acadoVariables.x[4] += acadoWorkspace.Dx0[4];

acadoVariables.x[5] += + acadoWorkspace.evGx[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[2]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[3]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[4]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[0];
acadoVariables.x[6] += + acadoWorkspace.evGx[5]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[6]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[7]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[8]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[9]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[1];
acadoVariables.x[7] += + acadoWorkspace.evGx[10]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[11]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[12]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[13]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[14]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[2];
acadoVariables.x[8] += + acadoWorkspace.evGx[15]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[16]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[17]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[18]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[19]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[3];
acadoVariables.x[9] += + acadoWorkspace.evGx[20]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[21]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[22]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[23]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[24]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[4];
acadoVariables.x[10] += + acadoWorkspace.evGx[25]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[26]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[27]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[28]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[29]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[5];
acadoVariables.x[11] += + acadoWorkspace.evGx[30]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[31]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[32]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[33]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[34]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[6];
acadoVariables.x[12] += + acadoWorkspace.evGx[35]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[36]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[37]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[38]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[39]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[7];
acadoVariables.x[13] += + acadoWorkspace.evGx[40]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[41]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[42]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[43]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[44]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[8];
acadoVariables.x[14] += + acadoWorkspace.evGx[45]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[46]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[47]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[48]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[49]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[9];
acadoVariables.x[15] += + acadoWorkspace.evGx[50]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[51]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[52]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[53]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[54]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[10];
acadoVariables.x[16] += + acadoWorkspace.evGx[55]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[56]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[57]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[58]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[59]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[11];
acadoVariables.x[17] += + acadoWorkspace.evGx[60]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[61]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[62]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[63]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[64]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[12];
acadoVariables.x[18] += + acadoWorkspace.evGx[65]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[66]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[67]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[68]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[69]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[13];
acadoVariables.x[19] += + acadoWorkspace.evGx[70]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[71]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[72]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[73]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[74]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[14];
acadoVariables.x[20] += + acadoWorkspace.evGx[75]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[76]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[77]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[78]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[79]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[15];
acadoVariables.x[21] += + acadoWorkspace.evGx[80]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[81]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[82]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[83]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[84]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[16];
acadoVariables.x[22] += + acadoWorkspace.evGx[85]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[86]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[87]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[88]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[89]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[17];
acadoVariables.x[23] += + acadoWorkspace.evGx[90]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[91]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[92]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[93]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[94]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[18];
acadoVariables.x[24] += + acadoWorkspace.evGx[95]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[96]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[97]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[98]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[99]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[19];
acadoVariables.x[25] += + acadoWorkspace.evGx[100]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[101]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[102]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[103]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[104]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[20];
acadoVariables.x[26] += + acadoWorkspace.evGx[105]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[106]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[107]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[108]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[109]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[21];
acadoVariables.x[27] += + acadoWorkspace.evGx[110]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[111]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[112]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[113]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[114]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[22];
acadoVariables.x[28] += + acadoWorkspace.evGx[115]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[116]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[117]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[118]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[119]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[23];
acadoVariables.x[29] += + acadoWorkspace.evGx[120]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[121]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[122]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[123]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[124]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[24];
acadoVariables.x[30] += + acadoWorkspace.evGx[125]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[126]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[127]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[128]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[129]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[25];
acadoVariables.x[31] += + acadoWorkspace.evGx[130]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[131]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[132]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[133]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[134]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[26];
acadoVariables.x[32] += + acadoWorkspace.evGx[135]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[136]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[137]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[138]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[139]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[27];
acadoVariables.x[33] += + acadoWorkspace.evGx[140]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[141]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[142]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[143]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[144]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[28];
acadoVariables.x[34] += + acadoWorkspace.evGx[145]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[146]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[147]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[148]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[149]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[29];
acadoVariables.x[35] += + acadoWorkspace.evGx[150]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[151]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[152]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[153]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[154]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[30];
acadoVariables.x[36] += + acadoWorkspace.evGx[155]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[156]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[157]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[158]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[159]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[31];
acadoVariables.x[37] += + acadoWorkspace.evGx[160]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[161]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[162]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[163]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[164]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[32];
acadoVariables.x[38] += + acadoWorkspace.evGx[165]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[166]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[167]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[168]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[169]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[33];
acadoVariables.x[39] += + acadoWorkspace.evGx[170]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[171]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[172]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[173]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[174]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[34];
acadoVariables.x[40] += + acadoWorkspace.evGx[175]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[176]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[177]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[178]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[179]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[35];
acadoVariables.x[41] += + acadoWorkspace.evGx[180]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[181]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[182]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[183]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[184]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[36];
acadoVariables.x[42] += + acadoWorkspace.evGx[185]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[186]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[187]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[188]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[189]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[37];
acadoVariables.x[43] += + acadoWorkspace.evGx[190]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[191]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[192]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[193]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[194]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[38];
acadoVariables.x[44] += + acadoWorkspace.evGx[195]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[196]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[197]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[198]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[199]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[39];
acadoVariables.x[45] += + acadoWorkspace.evGx[200]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[201]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[202]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[203]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[204]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[40];
acadoVariables.x[46] += + acadoWorkspace.evGx[205]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[206]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[207]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[208]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[209]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[41];
acadoVariables.x[47] += + acadoWorkspace.evGx[210]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[211]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[212]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[213]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[214]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[42];
acadoVariables.x[48] += + acadoWorkspace.evGx[215]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[216]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[217]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[218]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[219]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[43];
acadoVariables.x[49] += + acadoWorkspace.evGx[220]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[221]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[222]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[223]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[224]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[44];
acadoVariables.x[50] += + acadoWorkspace.evGx[225]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[226]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[227]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[228]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[229]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[45];
acadoVariables.x[51] += + acadoWorkspace.evGx[230]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[231]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[232]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[233]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[234]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[46];
acadoVariables.x[52] += + acadoWorkspace.evGx[235]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[236]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[237]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[238]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[239]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[47];
acadoVariables.x[53] += + acadoWorkspace.evGx[240]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[241]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[242]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[243]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[244]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[48];
acadoVariables.x[54] += + acadoWorkspace.evGx[245]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[246]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[247]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[248]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[249]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[49];
acadoVariables.x[55] += + acadoWorkspace.evGx[250]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[251]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[252]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[253]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[254]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[50];
acadoVariables.x[56] += + acadoWorkspace.evGx[255]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[256]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[257]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[258]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[259]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[51];
acadoVariables.x[57] += + acadoWorkspace.evGx[260]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[261]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[262]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[263]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[264]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[52];
acadoVariables.x[58] += + acadoWorkspace.evGx[265]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[266]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[267]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[268]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[269]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[53];
acadoVariables.x[59] += + acadoWorkspace.evGx[270]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[271]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[272]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[273]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[274]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[54];
acadoVariables.x[60] += + acadoWorkspace.evGx[275]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[276]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[277]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[278]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[279]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[55];
acadoVariables.x[61] += + acadoWorkspace.evGx[280]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[281]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[282]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[283]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[284]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[56];
acadoVariables.x[62] += + acadoWorkspace.evGx[285]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[286]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[287]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[288]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[289]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[57];
acadoVariables.x[63] += + acadoWorkspace.evGx[290]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[291]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[292]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[293]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[294]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[58];
acadoVariables.x[64] += + acadoWorkspace.evGx[295]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[296]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[297]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[298]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[299]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[59];
acadoVariables.x[65] += + acadoWorkspace.evGx[300]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[301]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[302]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[303]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[304]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[60];
acadoVariables.x[66] += + acadoWorkspace.evGx[305]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[306]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[307]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[308]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[309]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[61];
acadoVariables.x[67] += + acadoWorkspace.evGx[310]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[311]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[312]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[313]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[314]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[62];
acadoVariables.x[68] += + acadoWorkspace.evGx[315]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[316]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[317]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[318]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[319]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[63];
acadoVariables.x[69] += + acadoWorkspace.evGx[320]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[321]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[322]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[323]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[324]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[64];
acadoVariables.x[70] += + acadoWorkspace.evGx[325]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[326]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[327]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[328]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[329]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[65];
acadoVariables.x[71] += + acadoWorkspace.evGx[330]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[331]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[332]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[333]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[334]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[66];
acadoVariables.x[72] += + acadoWorkspace.evGx[335]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[336]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[337]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[338]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[339]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[67];
acadoVariables.x[73] += + acadoWorkspace.evGx[340]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[341]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[342]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[343]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[344]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[68];
acadoVariables.x[74] += + acadoWorkspace.evGx[345]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[346]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[347]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[348]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[349]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[69];
acadoVariables.x[75] += + acadoWorkspace.evGx[350]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[351]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[352]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[353]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[354]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[70];
acadoVariables.x[76] += + acadoWorkspace.evGx[355]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[356]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[357]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[358]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[359]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[71];
acadoVariables.x[77] += + acadoWorkspace.evGx[360]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[361]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[362]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[363]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[364]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[72];
acadoVariables.x[78] += + acadoWorkspace.evGx[365]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[366]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[367]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[368]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[369]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[73];
acadoVariables.x[79] += + acadoWorkspace.evGx[370]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[371]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[372]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[373]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[374]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[74];
acadoVariables.x[80] += + acadoWorkspace.evGx[375]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[376]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[377]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[378]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[379]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[75];
acadoVariables.x[81] += + acadoWorkspace.evGx[380]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[381]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[382]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[383]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[384]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[76];
acadoVariables.x[82] += + acadoWorkspace.evGx[385]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[386]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[387]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[388]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[389]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[77];
acadoVariables.x[83] += + acadoWorkspace.evGx[390]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[391]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[392]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[393]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[394]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[78];
acadoVariables.x[84] += + acadoWorkspace.evGx[395]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[396]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[397]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[398]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[399]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[79];
acadoVariables.x[85] += + acadoWorkspace.evGx[400]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[401]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[402]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[403]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[404]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[80];
acadoVariables.x[86] += + acadoWorkspace.evGx[405]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[406]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[407]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[408]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[409]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[81];
acadoVariables.x[87] += + acadoWorkspace.evGx[410]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[411]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[412]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[413]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[414]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[82];
acadoVariables.x[88] += + acadoWorkspace.evGx[415]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[416]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[417]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[418]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[419]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[83];
acadoVariables.x[89] += + acadoWorkspace.evGx[420]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[421]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[422]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[423]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[424]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[84];
acadoVariables.x[90] += + acadoWorkspace.evGx[425]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[426]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[427]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[428]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[429]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[85];
acadoVariables.x[91] += + acadoWorkspace.evGx[430]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[431]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[432]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[433]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[434]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[86];
acadoVariables.x[92] += + acadoWorkspace.evGx[435]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[436]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[437]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[438]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[439]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[87];
acadoVariables.x[93] += + acadoWorkspace.evGx[440]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[441]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[442]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[443]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[444]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[88];
acadoVariables.x[94] += + acadoWorkspace.evGx[445]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[446]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[447]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[448]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[449]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[89];
acadoVariables.x[95] += + acadoWorkspace.evGx[450]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[451]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[452]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[453]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[454]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[90];
acadoVariables.x[96] += + acadoWorkspace.evGx[455]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[456]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[457]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[458]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[459]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[91];
acadoVariables.x[97] += + acadoWorkspace.evGx[460]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[461]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[462]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[463]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[464]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[92];
acadoVariables.x[98] += + acadoWorkspace.evGx[465]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[466]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[467]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[468]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[469]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[93];
acadoVariables.x[99] += + acadoWorkspace.evGx[470]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[471]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[472]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[473]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[474]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[94];
acadoVariables.x[100] += + acadoWorkspace.evGx[475]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[476]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[477]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[478]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[479]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[95];
acadoVariables.x[101] += + acadoWorkspace.evGx[480]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[481]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[482]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[483]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[484]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[96];
acadoVariables.x[102] += + acadoWorkspace.evGx[485]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[486]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[487]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[488]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[489]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[97];
acadoVariables.x[103] += + acadoWorkspace.evGx[490]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[491]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[492]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[493]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[494]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[98];
acadoVariables.x[104] += + acadoWorkspace.evGx[495]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[496]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[497]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[498]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[499]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[99];
acadoVariables.x[105] += + acadoWorkspace.evGx[500]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[501]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[502]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[503]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[504]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[100];
acadoVariables.x[106] += + acadoWorkspace.evGx[505]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[506]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[507]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[508]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[509]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[101];
acadoVariables.x[107] += + acadoWorkspace.evGx[510]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[511]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[512]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[513]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[514]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[102];
acadoVariables.x[108] += + acadoWorkspace.evGx[515]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[516]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[517]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[518]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[519]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[103];
acadoVariables.x[109] += + acadoWorkspace.evGx[520]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[521]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[522]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[523]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[524]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[104];
acadoVariables.x[110] += + acadoWorkspace.evGx[525]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[526]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[527]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[528]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[529]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[105];
acadoVariables.x[111] += + acadoWorkspace.evGx[530]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[531]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[532]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[533]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[534]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[106];
acadoVariables.x[112] += + acadoWorkspace.evGx[535]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[536]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[537]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[538]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[539]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[107];
acadoVariables.x[113] += + acadoWorkspace.evGx[540]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[541]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[542]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[543]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[544]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[108];
acadoVariables.x[114] += + acadoWorkspace.evGx[545]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[546]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[547]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[548]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[549]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[109];
acadoVariables.x[115] += + acadoWorkspace.evGx[550]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[551]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[552]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[553]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[554]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[110];
acadoVariables.x[116] += + acadoWorkspace.evGx[555]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[556]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[557]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[558]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[559]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[111];
acadoVariables.x[117] += + acadoWorkspace.evGx[560]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[561]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[562]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[563]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[564]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[112];
acadoVariables.x[118] += + acadoWorkspace.evGx[565]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[566]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[567]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[568]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[569]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[113];
acadoVariables.x[119] += + acadoWorkspace.evGx[570]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[571]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[572]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[573]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[574]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[114];
acadoVariables.x[120] += + acadoWorkspace.evGx[575]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[576]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[577]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[578]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[579]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[115];
acadoVariables.x[121] += + acadoWorkspace.evGx[580]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[581]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[582]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[583]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[584]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[116];
acadoVariables.x[122] += + acadoWorkspace.evGx[585]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[586]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[587]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[588]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[589]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[117];
acadoVariables.x[123] += + acadoWorkspace.evGx[590]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[591]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[592]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[593]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[594]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[118];
acadoVariables.x[124] += + acadoWorkspace.evGx[595]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[596]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[597]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[598]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[599]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[119];
acadoVariables.x[125] += + acadoWorkspace.evGx[600]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[601]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[602]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[603]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[604]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[120];
acadoVariables.x[126] += + acadoWorkspace.evGx[605]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[606]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[607]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[608]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[609]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[121];
acadoVariables.x[127] += + acadoWorkspace.evGx[610]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[611]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[612]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[613]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[614]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[122];
acadoVariables.x[128] += + acadoWorkspace.evGx[615]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[616]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[617]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[618]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[619]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[123];
acadoVariables.x[129] += + acadoWorkspace.evGx[620]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[621]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[622]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[623]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[624]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[124];
acadoVariables.x[130] += + acadoWorkspace.evGx[625]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[626]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[627]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[628]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[629]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[125];
acadoVariables.x[131] += + acadoWorkspace.evGx[630]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[631]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[632]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[633]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[634]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[126];
acadoVariables.x[132] += + acadoWorkspace.evGx[635]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[636]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[637]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[638]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[639]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[127];
acadoVariables.x[133] += + acadoWorkspace.evGx[640]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[641]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[642]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[643]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[644]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[128];
acadoVariables.x[134] += + acadoWorkspace.evGx[645]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[646]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[647]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[648]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[649]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[129];
acadoVariables.x[135] += + acadoWorkspace.evGx[650]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[651]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[652]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[653]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[654]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[130];
acadoVariables.x[136] += + acadoWorkspace.evGx[655]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[656]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[657]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[658]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[659]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[131];
acadoVariables.x[137] += + acadoWorkspace.evGx[660]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[661]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[662]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[663]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[664]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[132];
acadoVariables.x[138] += + acadoWorkspace.evGx[665]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[666]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[667]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[668]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[669]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[133];
acadoVariables.x[139] += + acadoWorkspace.evGx[670]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[671]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[672]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[673]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[674]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[134];
acadoVariables.x[140] += + acadoWorkspace.evGx[675]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[676]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[677]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[678]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[679]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[135];
acadoVariables.x[141] += + acadoWorkspace.evGx[680]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[681]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[682]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[683]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[684]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[136];
acadoVariables.x[142] += + acadoWorkspace.evGx[685]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[686]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[687]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[688]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[689]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[137];
acadoVariables.x[143] += + acadoWorkspace.evGx[690]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[691]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[692]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[693]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[694]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[138];
acadoVariables.x[144] += + acadoWorkspace.evGx[695]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[696]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[697]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[698]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[699]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[139];
acadoVariables.x[145] += + acadoWorkspace.evGx[700]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[701]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[702]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[703]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[704]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[140];
acadoVariables.x[146] += + acadoWorkspace.evGx[705]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[706]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[707]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[708]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[709]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[141];
acadoVariables.x[147] += + acadoWorkspace.evGx[710]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[711]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[712]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[713]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[714]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[142];
acadoVariables.x[148] += + acadoWorkspace.evGx[715]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[716]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[717]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[718]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[719]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[143];
acadoVariables.x[149] += + acadoWorkspace.evGx[720]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[721]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[722]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[723]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[724]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[144];
acadoVariables.x[150] += + acadoWorkspace.evGx[725]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[726]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[727]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[728]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[729]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[145];
acadoVariables.x[151] += + acadoWorkspace.evGx[730]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[731]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[732]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[733]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[734]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[146];
acadoVariables.x[152] += + acadoWorkspace.evGx[735]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[736]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[737]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[738]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[739]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[147];
acadoVariables.x[153] += + acadoWorkspace.evGx[740]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[741]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[742]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[743]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[744]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[148];
acadoVariables.x[154] += + acadoWorkspace.evGx[745]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[746]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[747]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[748]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[749]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[149];

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_multEDu( &(acadoWorkspace.E[ lRun3 * 10 ]), &(acadoWorkspace.x[ lRun2 * 2 ]), &(acadoVariables.x[ lRun1 * 5 + 5 ]) );
}
}
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
acadoVariables.lbValues[0] = -3.0000000000000000e+00;
acadoVariables.lbValues[1] = -1.0000000000000000e+12;
acadoVariables.lbValues[2] = -3.0000000000000000e+00;
acadoVariables.lbValues[3] = -1.0000000000000000e+12;
acadoVariables.lbValues[4] = -3.0000000000000000e+00;
acadoVariables.lbValues[5] = -1.0000000000000000e+12;
acadoVariables.lbValues[6] = -3.0000000000000000e+00;
acadoVariables.lbValues[7] = -1.0000000000000000e+12;
acadoVariables.lbValues[8] = -3.0000000000000000e+00;
acadoVariables.lbValues[9] = -1.0000000000000000e+12;
acadoVariables.lbValues[10] = -3.0000000000000000e+00;
acadoVariables.lbValues[11] = -1.0000000000000000e+12;
acadoVariables.lbValues[12] = -3.0000000000000000e+00;
acadoVariables.lbValues[13] = -1.0000000000000000e+12;
acadoVariables.lbValues[14] = -3.0000000000000000e+00;
acadoVariables.lbValues[15] = -1.0000000000000000e+12;
acadoVariables.lbValues[16] = -3.0000000000000000e+00;
acadoVariables.lbValues[17] = -1.0000000000000000e+12;
acadoVariables.lbValues[18] = -3.0000000000000000e+00;
acadoVariables.lbValues[19] = -1.0000000000000000e+12;
acadoVariables.lbValues[20] = -3.0000000000000000e+00;
acadoVariables.lbValues[21] = -1.0000000000000000e+12;
acadoVariables.lbValues[22] = -3.0000000000000000e+00;
acadoVariables.lbValues[23] = -1.0000000000000000e+12;
acadoVariables.lbValues[24] = -3.0000000000000000e+00;
acadoVariables.lbValues[25] = -1.0000000000000000e+12;
acadoVariables.lbValues[26] = -3.0000000000000000e+00;
acadoVariables.lbValues[27] = -1.0000000000000000e+12;
acadoVariables.lbValues[28] = -3.0000000000000000e+00;
acadoVariables.lbValues[29] = -1.0000000000000000e+12;
acadoVariables.lbValues[30] = -3.0000000000000000e+00;
acadoVariables.lbValues[31] = -1.0000000000000000e+12;
acadoVariables.lbValues[32] = -3.0000000000000000e+00;
acadoVariables.lbValues[33] = -1.0000000000000000e+12;
acadoVariables.lbValues[34] = -3.0000000000000000e+00;
acadoVariables.lbValues[35] = -1.0000000000000000e+12;
acadoVariables.lbValues[36] = -3.0000000000000000e+00;
acadoVariables.lbValues[37] = -1.0000000000000000e+12;
acadoVariables.lbValues[38] = -3.0000000000000000e+00;
acadoVariables.lbValues[39] = -1.0000000000000000e+12;
acadoVariables.lbValues[40] = -3.0000000000000000e+00;
acadoVariables.lbValues[41] = -1.0000000000000000e+12;
acadoVariables.lbValues[42] = -3.0000000000000000e+00;
acadoVariables.lbValues[43] = -1.0000000000000000e+12;
acadoVariables.lbValues[44] = -3.0000000000000000e+00;
acadoVariables.lbValues[45] = -1.0000000000000000e+12;
acadoVariables.lbValues[46] = -3.0000000000000000e+00;
acadoVariables.lbValues[47] = -1.0000000000000000e+12;
acadoVariables.lbValues[48] = -3.0000000000000000e+00;
acadoVariables.lbValues[49] = -1.0000000000000000e+12;
acadoVariables.lbValues[50] = -3.0000000000000000e+00;
acadoVariables.lbValues[51] = -1.0000000000000000e+12;
acadoVariables.lbValues[52] = -3.0000000000000000e+00;
acadoVariables.lbValues[53] = -1.0000000000000000e+12;
acadoVariables.lbValues[54] = -3.0000000000000000e+00;
acadoVariables.lbValues[55] = -1.0000000000000000e+12;
acadoVariables.lbValues[56] = -3.0000000000000000e+00;
acadoVariables.lbValues[57] = -1.0000000000000000e+12;
acadoVariables.lbValues[58] = -3.0000000000000000e+00;
acadoVariables.lbValues[59] = -1.0000000000000000e+12;
acadoVariables.ubValues[0] = 3.0000000000000000e+00;
acadoVariables.ubValues[1] = 1.0000000000000000e+12;
acadoVariables.ubValues[2] = 3.0000000000000000e+00;
acadoVariables.ubValues[3] = 1.0000000000000000e+12;
acadoVariables.ubValues[4] = 3.0000000000000000e+00;
acadoVariables.ubValues[5] = 1.0000000000000000e+12;
acadoVariables.ubValues[6] = 3.0000000000000000e+00;
acadoVariables.ubValues[7] = 1.0000000000000000e+12;
acadoVariables.ubValues[8] = 3.0000000000000000e+00;
acadoVariables.ubValues[9] = 1.0000000000000000e+12;
acadoVariables.ubValues[10] = 3.0000000000000000e+00;
acadoVariables.ubValues[11] = 1.0000000000000000e+12;
acadoVariables.ubValues[12] = 3.0000000000000000e+00;
acadoVariables.ubValues[13] = 1.0000000000000000e+12;
acadoVariables.ubValues[14] = 3.0000000000000000e+00;
acadoVariables.ubValues[15] = 1.0000000000000000e+12;
acadoVariables.ubValues[16] = 3.0000000000000000e+00;
acadoVariables.ubValues[17] = 1.0000000000000000e+12;
acadoVariables.ubValues[18] = 3.0000000000000000e+00;
acadoVariables.ubValues[19] = 1.0000000000000000e+12;
acadoVariables.ubValues[20] = 3.0000000000000000e+00;
acadoVariables.ubValues[21] = 1.0000000000000000e+12;
acadoVariables.ubValues[22] = 3.0000000000000000e+00;
acadoVariables.ubValues[23] = 1.0000000000000000e+12;
acadoVariables.ubValues[24] = 3.0000000000000000e+00;
acadoVariables.ubValues[25] = 1.0000000000000000e+12;
acadoVariables.ubValues[26] = 3.0000000000000000e+00;
acadoVariables.ubValues[27] = 1.0000000000000000e+12;
acadoVariables.ubValues[28] = 3.0000000000000000e+00;
acadoVariables.ubValues[29] = 1.0000000000000000e+12;
acadoVariables.ubValues[30] = 3.0000000000000000e+00;
acadoVariables.ubValues[31] = 1.0000000000000000e+12;
acadoVariables.ubValues[32] = 3.0000000000000000e+00;
acadoVariables.ubValues[33] = 1.0000000000000000e+12;
acadoVariables.ubValues[34] = 3.0000000000000000e+00;
acadoVariables.ubValues[35] = 1.0000000000000000e+12;
acadoVariables.ubValues[36] = 3.0000000000000000e+00;
acadoVariables.ubValues[37] = 1.0000000000000000e+12;
acadoVariables.ubValues[38] = 3.0000000000000000e+00;
acadoVariables.ubValues[39] = 1.0000000000000000e+12;
acadoVariables.ubValues[40] = 3.0000000000000000e+00;
acadoVariables.ubValues[41] = 1.0000000000000000e+12;
acadoVariables.ubValues[42] = 3.0000000000000000e+00;
acadoVariables.ubValues[43] = 1.0000000000000000e+12;
acadoVariables.ubValues[44] = 3.0000000000000000e+00;
acadoVariables.ubValues[45] = 1.0000000000000000e+12;
acadoVariables.ubValues[46] = 3.0000000000000000e+00;
acadoVariables.ubValues[47] = 1.0000000000000000e+12;
acadoVariables.ubValues[48] = 3.0000000000000000e+00;
acadoVariables.ubValues[49] = 1.0000000000000000e+12;
acadoVariables.ubValues[50] = 3.0000000000000000e+00;
acadoVariables.ubValues[51] = 1.0000000000000000e+12;
acadoVariables.ubValues[52] = 3.0000000000000000e+00;
acadoVariables.ubValues[53] = 1.0000000000000000e+12;
acadoVariables.ubValues[54] = 3.0000000000000000e+00;
acadoVariables.ubValues[55] = 1.0000000000000000e+12;
acadoVariables.ubValues[56] = 3.0000000000000000e+00;
acadoVariables.ubValues[57] = 1.0000000000000000e+12;
acadoVariables.ubValues[58] = 3.0000000000000000e+00;
acadoVariables.ubValues[59] = 1.0000000000000000e+12;
acadoVariables.lbAValues[0] = -3.1415926535897931e+00;
acadoVariables.lbAValues[1] = -3.1415926535897931e+00;
acadoVariables.lbAValues[2] = -3.1415926535897931e+00;
acadoVariables.lbAValues[3] = -3.1415926535897931e+00;
acadoVariables.lbAValues[4] = -3.1415926535897931e+00;
acadoVariables.lbAValues[5] = -3.1415926535897931e+00;
acadoVariables.lbAValues[6] = -3.1415926535897931e+00;
acadoVariables.lbAValues[7] = -3.1415926535897931e+00;
acadoVariables.lbAValues[8] = -3.1415926535897931e+00;
acadoVariables.lbAValues[9] = -3.1415926535897931e+00;
acadoVariables.lbAValues[10] = -3.1415926535897931e+00;
acadoVariables.lbAValues[11] = -3.1415926535897931e+00;
acadoVariables.lbAValues[12] = -3.1415926535897931e+00;
acadoVariables.lbAValues[13] = -3.1415926535897931e+00;
acadoVariables.lbAValues[14] = -3.1415926535897931e+00;
acadoVariables.lbAValues[15] = -3.1415926535897931e+00;
acadoVariables.lbAValues[16] = -3.1415926535897931e+00;
acadoVariables.lbAValues[17] = -3.1415926535897931e+00;
acadoVariables.lbAValues[18] = -3.1415926535897931e+00;
acadoVariables.lbAValues[19] = -3.1415926535897931e+00;
acadoVariables.lbAValues[20] = -3.1415926535897931e+00;
acadoVariables.lbAValues[21] = -3.1415926535897931e+00;
acadoVariables.lbAValues[22] = -3.1415926535897931e+00;
acadoVariables.lbAValues[23] = -3.1415926535897931e+00;
acadoVariables.lbAValues[24] = -3.1415926535897931e+00;
acadoVariables.lbAValues[25] = -3.1415926535897931e+00;
acadoVariables.lbAValues[26] = -3.1415926535897931e+00;
acadoVariables.lbAValues[27] = -3.1415926535897931e+00;
acadoVariables.lbAValues[28] = -3.1415926535897931e+00;
acadoVariables.lbAValues[29] = -3.1415926535897931e+00;
acadoVariables.ubAValues[0] = 3.1415926535897931e+00;
acadoVariables.ubAValues[1] = 3.1415926535897931e+00;
acadoVariables.ubAValues[2] = 3.1415926535897931e+00;
acadoVariables.ubAValues[3] = 3.1415926535897931e+00;
acadoVariables.ubAValues[4] = 3.1415926535897931e+00;
acadoVariables.ubAValues[5] = 3.1415926535897931e+00;
acadoVariables.ubAValues[6] = 3.1415926535897931e+00;
acadoVariables.ubAValues[7] = 3.1415926535897931e+00;
acadoVariables.ubAValues[8] = 3.1415926535897931e+00;
acadoVariables.ubAValues[9] = 3.1415926535897931e+00;
acadoVariables.ubAValues[10] = 3.1415926535897931e+00;
acadoVariables.ubAValues[11] = 3.1415926535897931e+00;
acadoVariables.ubAValues[12] = 3.1415926535897931e+00;
acadoVariables.ubAValues[13] = 3.1415926535897931e+00;
acadoVariables.ubAValues[14] = 3.1415926535897931e+00;
acadoVariables.ubAValues[15] = 3.1415926535897931e+00;
acadoVariables.ubAValues[16] = 3.1415926535897931e+00;
acadoVariables.ubAValues[17] = 3.1415926535897931e+00;
acadoVariables.ubAValues[18] = 3.1415926535897931e+00;
acadoVariables.ubAValues[19] = 3.1415926535897931e+00;
acadoVariables.ubAValues[20] = 3.1415926535897931e+00;
acadoVariables.ubAValues[21] = 3.1415926535897931e+00;
acadoVariables.ubAValues[22] = 3.1415926535897931e+00;
acadoVariables.ubAValues[23] = 3.1415926535897931e+00;
acadoVariables.ubAValues[24] = 3.1415926535897931e+00;
acadoVariables.ubAValues[25] = 3.1415926535897931e+00;
acadoVariables.ubAValues[26] = 3.1415926535897931e+00;
acadoVariables.ubAValues[27] = 3.1415926535897931e+00;
acadoVariables.ubAValues[28] = 3.1415926535897931e+00;
acadoVariables.ubAValues[29] = 3.1415926535897931e+00;
return ret;
}

void acado_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 30; ++index)
{
acadoWorkspace.state[0] = acadoVariables.x[index * 5];
acadoWorkspace.state[1] = acadoVariables.x[index * 5 + 1];
acadoWorkspace.state[2] = acadoVariables.x[index * 5 + 2];
acadoWorkspace.state[3] = acadoVariables.x[index * 5 + 3];
acadoWorkspace.state[4] = acadoVariables.x[index * 5 + 4];
acadoWorkspace.state[40] = acadoVariables.u[index * 2];
acadoWorkspace.state[41] = acadoVariables.u[index * 2 + 1];

acado_integrate(acadoWorkspace.state, index == 0);

acadoVariables.x[index * 5 + 5] = acadoWorkspace.state[0];
acadoVariables.x[index * 5 + 6] = acadoWorkspace.state[1];
acadoVariables.x[index * 5 + 7] = acadoWorkspace.state[2];
acadoVariables.x[index * 5 + 8] = acadoWorkspace.state[3];
acadoVariables.x[index * 5 + 9] = acadoWorkspace.state[4];
}
}

void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 30; ++index)
{
acadoVariables.x[index * 5] = acadoVariables.x[index * 5 + 5];
acadoVariables.x[index * 5 + 1] = acadoVariables.x[index * 5 + 6];
acadoVariables.x[index * 5 + 2] = acadoVariables.x[index * 5 + 7];
acadoVariables.x[index * 5 + 3] = acadoVariables.x[index * 5 + 8];
acadoVariables.x[index * 5 + 4] = acadoVariables.x[index * 5 + 9];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[150] = xEnd[0];
acadoVariables.x[151] = xEnd[1];
acadoVariables.x[152] = xEnd[2];
acadoVariables.x[153] = xEnd[3];
acadoVariables.x[154] = xEnd[4];
}
else if (strategy == 2) 
{
acadoWorkspace.state[0] = acadoVariables.x[150];
acadoWorkspace.state[1] = acadoVariables.x[151];
acadoWorkspace.state[2] = acadoVariables.x[152];
acadoWorkspace.state[3] = acadoVariables.x[153];
acadoWorkspace.state[4] = acadoVariables.x[154];
if (uEnd != 0)
{
acadoWorkspace.state[40] = uEnd[0];
acadoWorkspace.state[41] = uEnd[1];
}
else
{
acadoWorkspace.state[40] = acadoVariables.u[58];
acadoWorkspace.state[41] = acadoVariables.u[59];
}

acado_integrate(acadoWorkspace.state, 1);

acadoVariables.x[150] = acadoWorkspace.state[0];
acadoVariables.x[151] = acadoWorkspace.state[1];
acadoVariables.x[152] = acadoWorkspace.state[2];
acadoVariables.x[153] = acadoWorkspace.state[3];
acadoVariables.x[154] = acadoWorkspace.state[4];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 29; ++index)
{
acadoVariables.u[index * 2] = acadoVariables.u[index * 2 + 2];
acadoVariables.u[index * 2 + 1] = acadoVariables.u[index * 2 + 3];
}

if (uEnd != 0)
{
acadoVariables.u[58] = uEnd[0];
acadoVariables.u[59] = uEnd[1];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19] + acadoWorkspace.g[20]*acadoWorkspace.x[20] + acadoWorkspace.g[21]*acadoWorkspace.x[21] + acadoWorkspace.g[22]*acadoWorkspace.x[22] + acadoWorkspace.g[23]*acadoWorkspace.x[23] + acadoWorkspace.g[24]*acadoWorkspace.x[24] + acadoWorkspace.g[25]*acadoWorkspace.x[25] + acadoWorkspace.g[26]*acadoWorkspace.x[26] + acadoWorkspace.g[27]*acadoWorkspace.x[27] + acadoWorkspace.g[28]*acadoWorkspace.x[28] + acadoWorkspace.g[29]*acadoWorkspace.x[29] + acadoWorkspace.g[30]*acadoWorkspace.x[30] + acadoWorkspace.g[31]*acadoWorkspace.x[31] + acadoWorkspace.g[32]*acadoWorkspace.x[32] + acadoWorkspace.g[33]*acadoWorkspace.x[33] + acadoWorkspace.g[34]*acadoWorkspace.x[34] + acadoWorkspace.g[35]*acadoWorkspace.x[35] + acadoWorkspace.g[36]*acadoWorkspace.x[36] + acadoWorkspace.g[37]*acadoWorkspace.x[37] + acadoWorkspace.g[38]*acadoWorkspace.x[38] + acadoWorkspace.g[39]*acadoWorkspace.x[39] + acadoWorkspace.g[40]*acadoWorkspace.x[40] + acadoWorkspace.g[41]*acadoWorkspace.x[41] + acadoWorkspace.g[42]*acadoWorkspace.x[42] + acadoWorkspace.g[43]*acadoWorkspace.x[43] + acadoWorkspace.g[44]*acadoWorkspace.x[44] + acadoWorkspace.g[45]*acadoWorkspace.x[45] + acadoWorkspace.g[46]*acadoWorkspace.x[46] + acadoWorkspace.g[47]*acadoWorkspace.x[47] + acadoWorkspace.g[48]*acadoWorkspace.x[48] + acadoWorkspace.g[49]*acadoWorkspace.x[49] + acadoWorkspace.g[50]*acadoWorkspace.x[50] + acadoWorkspace.g[51]*acadoWorkspace.x[51] + acadoWorkspace.g[52]*acadoWorkspace.x[52] + acadoWorkspace.g[53]*acadoWorkspace.x[53] + acadoWorkspace.g[54]*acadoWorkspace.x[54] + acadoWorkspace.g[55]*acadoWorkspace.x[55] + acadoWorkspace.g[56]*acadoWorkspace.x[56] + acadoWorkspace.g[57]*acadoWorkspace.x[57] + acadoWorkspace.g[58]*acadoWorkspace.x[58] + acadoWorkspace.g[59]*acadoWorkspace.x[59];
kkt = fabs( kkt );
for (index = 0; index < 60; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
for (index = 0; index < 30; ++index)
{
prd = acadoWorkspace.y[index + 60];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lbA[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ubA[index] * prd);
}
return kkt;
}

real_t acado_getObjective(  )
{
real_t objVal;

int lRun1;
/** Row vector of size: 5 */
real_t tmpDy[ 5 ];

/** Row vector of size: 5 */
real_t tmpDyN[ 5 ];

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1 * 5];
acadoWorkspace.objValueIn[1] = acadoVariables.x[lRun1 * 5 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[lRun1 * 5 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[lRun1 * 5 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[lRun1 * 5 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.u[lRun1 * 2];
acadoWorkspace.objValueIn[6] = acadoVariables.u[lRun1 * 2 + 1];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[lRun1 * 5] = acadoWorkspace.objValueOut[0] - acadoVariables.y[lRun1 * 5];
acadoWorkspace.Dy[lRun1 * 5 + 1] = acadoWorkspace.objValueOut[1] - acadoVariables.y[lRun1 * 5 + 1];
acadoWorkspace.Dy[lRun1 * 5 + 2] = acadoWorkspace.objValueOut[2] - acadoVariables.y[lRun1 * 5 + 2];
acadoWorkspace.Dy[lRun1 * 5 + 3] = acadoWorkspace.objValueOut[3] - acadoVariables.y[lRun1 * 5 + 3];
acadoWorkspace.Dy[lRun1 * 5 + 4] = acadoWorkspace.objValueOut[4] - acadoVariables.y[lRun1 * 5 + 4];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[150];
acadoWorkspace.objValueIn[1] = acadoVariables.x[151];
acadoWorkspace.objValueIn[2] = acadoVariables.x[152];
acadoWorkspace.objValueIn[3] = acadoVariables.x[153];
acadoWorkspace.objValueIn[4] = acadoVariables.x[154];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1] - acadoVariables.yN[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2] - acadoVariables.yN[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3] - acadoVariables.yN[3];
acadoWorkspace.DyN[4] = acadoWorkspace.objValueOut[4] - acadoVariables.yN[4];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
tmpDy[0] = + acadoWorkspace.Dy[lRun1 * 5]*acadoVariables.W[lRun1 * 25] + acadoWorkspace.Dy[lRun1 * 5 + 1]*acadoVariables.W[lRun1 * 25 + 5] + acadoWorkspace.Dy[lRun1 * 5 + 2]*acadoVariables.W[lRun1 * 25 + 10] + acadoWorkspace.Dy[lRun1 * 5 + 3]*acadoVariables.W[lRun1 * 25 + 15] + acadoWorkspace.Dy[lRun1 * 5 + 4]*acadoVariables.W[lRun1 * 25 + 20];
tmpDy[1] = + acadoWorkspace.Dy[lRun1 * 5]*acadoVariables.W[lRun1 * 25 + 1] + acadoWorkspace.Dy[lRun1 * 5 + 1]*acadoVariables.W[lRun1 * 25 + 6] + acadoWorkspace.Dy[lRun1 * 5 + 2]*acadoVariables.W[lRun1 * 25 + 11] + acadoWorkspace.Dy[lRun1 * 5 + 3]*acadoVariables.W[lRun1 * 25 + 16] + acadoWorkspace.Dy[lRun1 * 5 + 4]*acadoVariables.W[lRun1 * 25 + 21];
tmpDy[2] = + acadoWorkspace.Dy[lRun1 * 5]*acadoVariables.W[lRun1 * 25 + 2] + acadoWorkspace.Dy[lRun1 * 5 + 1]*acadoVariables.W[lRun1 * 25 + 7] + acadoWorkspace.Dy[lRun1 * 5 + 2]*acadoVariables.W[lRun1 * 25 + 12] + acadoWorkspace.Dy[lRun1 * 5 + 3]*acadoVariables.W[lRun1 * 25 + 17] + acadoWorkspace.Dy[lRun1 * 5 + 4]*acadoVariables.W[lRun1 * 25 + 22];
tmpDy[3] = + acadoWorkspace.Dy[lRun1 * 5]*acadoVariables.W[lRun1 * 25 + 3] + acadoWorkspace.Dy[lRun1 * 5 + 1]*acadoVariables.W[lRun1 * 25 + 8] + acadoWorkspace.Dy[lRun1 * 5 + 2]*acadoVariables.W[lRun1 * 25 + 13] + acadoWorkspace.Dy[lRun1 * 5 + 3]*acadoVariables.W[lRun1 * 25 + 18] + acadoWorkspace.Dy[lRun1 * 5 + 4]*acadoVariables.W[lRun1 * 25 + 23];
tmpDy[4] = + acadoWorkspace.Dy[lRun1 * 5]*acadoVariables.W[lRun1 * 25 + 4] + acadoWorkspace.Dy[lRun1 * 5 + 1]*acadoVariables.W[lRun1 * 25 + 9] + acadoWorkspace.Dy[lRun1 * 5 + 2]*acadoVariables.W[lRun1 * 25 + 14] + acadoWorkspace.Dy[lRun1 * 5 + 3]*acadoVariables.W[lRun1 * 25 + 19] + acadoWorkspace.Dy[lRun1 * 5 + 4]*acadoVariables.W[lRun1 * 25 + 24];
objVal += + acadoWorkspace.Dy[lRun1 * 5]*tmpDy[0] + acadoWorkspace.Dy[lRun1 * 5 + 1]*tmpDy[1] + acadoWorkspace.Dy[lRun1 * 5 + 2]*tmpDy[2] + acadoWorkspace.Dy[lRun1 * 5 + 3]*tmpDy[3] + acadoWorkspace.Dy[lRun1 * 5 + 4]*tmpDy[4];
}

tmpDyN[0] = + acadoWorkspace.DyN[0]*acadoVariables.WN[0];
tmpDyN[1] = + acadoWorkspace.DyN[1]*acadoVariables.WN[6];
tmpDyN[2] = + acadoWorkspace.DyN[2]*acadoVariables.WN[12];
tmpDyN[3] = + acadoWorkspace.DyN[3]*acadoVariables.WN[18];
tmpDyN[4] = + acadoWorkspace.DyN[4]*acadoVariables.WN[24];
objVal += + acadoWorkspace.DyN[0]*tmpDyN[0] + acadoWorkspace.DyN[1]*tmpDyN[1] + acadoWorkspace.DyN[2]*tmpDyN[2] + acadoWorkspace.DyN[3]*tmpDyN[3] + acadoWorkspace.DyN[4]*tmpDyN[4];

objVal *= 0.5;
return objVal;
}

