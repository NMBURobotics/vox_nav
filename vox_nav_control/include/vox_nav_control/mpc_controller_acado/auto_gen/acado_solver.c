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
for (lRun1 = 0; lRun1 < 16; ++lRun1)
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
const real_t* od = in + 6;
/* Vector of auxiliary variables; number of elements: 120. */
real_t* a = acadoWorkspace.objAuxVar;

/* Compute intermediate quantities: */
a[0] = (pow((xd[0]-od[0]),2));
a[1] = ((od[2])*(od[2]));
a[2] = (pow((xd[1]-od[1]),2));
a[3] = ((od[3])*(od[3]));
a[4] = (exp(((real_t)(1.0000000000000000e+00)/(pow((((a[0]/a[1])+(a[2]/a[3]))-(real_t)(1.0000000000000000e+00)),4)))));
a[5] = (pow((xd[0]-od[4]),2));
a[6] = ((od[6])*(od[6]));
a[7] = (pow((xd[1]-od[5]),2));
a[8] = ((od[7])*(od[7]));
a[9] = (exp(((real_t)(1.0000000000000000e+00)/(pow((((a[5]/a[6])+(a[7]/a[8]))-(real_t)(1.0000000000000000e+00)),4)))));
a[10] = (pow((xd[0]-od[8]),2));
a[11] = ((od[10])*(od[10]));
a[12] = (pow((xd[1]-od[9]),2));
a[13] = ((od[11])*(od[11]));
a[14] = (exp(((real_t)(1.0000000000000000e+00)/(pow((((a[10]/a[11])+(a[12]/a[13]))-(real_t)(1.0000000000000000e+00)),4)))));
a[15] = (pow((xd[0]-od[12]),2));
a[16] = ((od[14])*(od[14]));
a[17] = (pow((xd[1]-od[13]),2));
a[18] = ((od[15])*(od[15]));
a[19] = (exp(((real_t)(1.0000000000000000e+00)/(pow((((a[15]/a[16])+(a[17]/a[18]))-(real_t)(1.0000000000000000e+00)),4)))));
a[20] = (pow((xd[0]-od[16]),2));
a[21] = ((od[18])*(od[18]));
a[22] = (pow((xd[1]-od[17]),2));
a[23] = ((od[19])*(od[19]));
a[24] = (exp(((real_t)(1.0000000000000000e+00)/(pow((((a[20]/a[21])+(a[22]/a[23]))-(real_t)(1.0000000000000000e+00)),4)))));
a[25] = (pow((xd[0]-od[20]),2));
a[26] = ((od[22])*(od[22]));
a[27] = (pow((xd[1]-od[21]),2));
a[28] = ((od[23])*(od[23]));
a[29] = (exp(((real_t)(1.0000000000000000e+00)/(pow((((a[25]/a[26])+(a[27]/a[28]))-(real_t)(1.0000000000000000e+00)),4)))));
a[30] = (pow((xd[0]-od[24]),2));
a[31] = ((od[26])*(od[26]));
a[32] = (pow((xd[1]-od[25]),2));
a[33] = ((od[27])*(od[27]));
a[34] = (exp(((real_t)(1.0000000000000000e+00)/(pow((((a[30]/a[31])+(a[32]/a[33]))-(real_t)(1.0000000000000000e+00)),4)))));
a[35] = (pow((xd[0]-od[28]),2));
a[36] = ((od[30])*(od[30]));
a[37] = (pow((xd[1]-od[29]),2));
a[38] = ((od[31])*(od[31]));
a[39] = (exp(((real_t)(1.0000000000000000e+00)/(pow((((a[35]/a[36])+(a[37]/a[38]))-(real_t)(1.0000000000000000e+00)),4)))));
a[40] = ((real_t)(4.0000000000000000e+00)*(pow((((a[0]/a[1])+(a[2]/a[3]))-(real_t)(1.0000000000000000e+00)),3)));
a[41] = ((real_t)(2.0000000000000000e+00)*(xd[0]-od[0]));
a[42] = ((real_t)(1.0000000000000000e+00)/a[1]);
a[43] = ((real_t)(1.0000000000000000e+00)/(pow((((a[0]/a[1])+(a[2]/a[3]))-(real_t)(1.0000000000000000e+00)),4)));
a[44] = (a[43]*a[43]);
a[45] = (exp(((real_t)(1.0000000000000000e+00)/(pow((((a[0]/a[1])+(a[2]/a[3]))-(real_t)(1.0000000000000000e+00)),4)))));
a[46] = (((real_t)(0.0000000000000000e+00)-((a[40]*(a[41]*a[42]))*a[44]))*a[45]);
a[47] = ((real_t)(4.0000000000000000e+00)*(pow((((a[5]/a[6])+(a[7]/a[8]))-(real_t)(1.0000000000000000e+00)),3)));
a[48] = ((real_t)(2.0000000000000000e+00)*(xd[0]-od[4]));
a[49] = ((real_t)(1.0000000000000000e+00)/a[6]);
a[50] = ((real_t)(1.0000000000000000e+00)/(pow((((a[5]/a[6])+(a[7]/a[8]))-(real_t)(1.0000000000000000e+00)),4)));
a[51] = (a[50]*a[50]);
a[52] = (exp(((real_t)(1.0000000000000000e+00)/(pow((((a[5]/a[6])+(a[7]/a[8]))-(real_t)(1.0000000000000000e+00)),4)))));
a[53] = (((real_t)(0.0000000000000000e+00)-((a[47]*(a[48]*a[49]))*a[51]))*a[52]);
a[54] = ((real_t)(4.0000000000000000e+00)*(pow((((a[10]/a[11])+(a[12]/a[13]))-(real_t)(1.0000000000000000e+00)),3)));
a[55] = ((real_t)(2.0000000000000000e+00)*(xd[0]-od[8]));
a[56] = ((real_t)(1.0000000000000000e+00)/a[11]);
a[57] = ((real_t)(1.0000000000000000e+00)/(pow((((a[10]/a[11])+(a[12]/a[13]))-(real_t)(1.0000000000000000e+00)),4)));
a[58] = (a[57]*a[57]);
a[59] = (exp(((real_t)(1.0000000000000000e+00)/(pow((((a[10]/a[11])+(a[12]/a[13]))-(real_t)(1.0000000000000000e+00)),4)))));
a[60] = (((real_t)(0.0000000000000000e+00)-((a[54]*(a[55]*a[56]))*a[58]))*a[59]);
a[61] = ((real_t)(4.0000000000000000e+00)*(pow((((a[15]/a[16])+(a[17]/a[18]))-(real_t)(1.0000000000000000e+00)),3)));
a[62] = ((real_t)(2.0000000000000000e+00)*(xd[0]-od[12]));
a[63] = ((real_t)(1.0000000000000000e+00)/a[16]);
a[64] = ((real_t)(1.0000000000000000e+00)/(pow((((a[15]/a[16])+(a[17]/a[18]))-(real_t)(1.0000000000000000e+00)),4)));
a[65] = (a[64]*a[64]);
a[66] = (exp(((real_t)(1.0000000000000000e+00)/(pow((((a[15]/a[16])+(a[17]/a[18]))-(real_t)(1.0000000000000000e+00)),4)))));
a[67] = (((real_t)(0.0000000000000000e+00)-((a[61]*(a[62]*a[63]))*a[65]))*a[66]);
a[68] = ((real_t)(4.0000000000000000e+00)*(pow((((a[20]/a[21])+(a[22]/a[23]))-(real_t)(1.0000000000000000e+00)),3)));
a[69] = ((real_t)(2.0000000000000000e+00)*(xd[0]-od[16]));
a[70] = ((real_t)(1.0000000000000000e+00)/a[21]);
a[71] = ((real_t)(1.0000000000000000e+00)/(pow((((a[20]/a[21])+(a[22]/a[23]))-(real_t)(1.0000000000000000e+00)),4)));
a[72] = (a[71]*a[71]);
a[73] = (exp(((real_t)(1.0000000000000000e+00)/(pow((((a[20]/a[21])+(a[22]/a[23]))-(real_t)(1.0000000000000000e+00)),4)))));
a[74] = (((real_t)(0.0000000000000000e+00)-((a[68]*(a[69]*a[70]))*a[72]))*a[73]);
a[75] = ((real_t)(4.0000000000000000e+00)*(pow((((a[25]/a[26])+(a[27]/a[28]))-(real_t)(1.0000000000000000e+00)),3)));
a[76] = ((real_t)(2.0000000000000000e+00)*(xd[0]-od[20]));
a[77] = ((real_t)(1.0000000000000000e+00)/a[26]);
a[78] = ((real_t)(1.0000000000000000e+00)/(pow((((a[25]/a[26])+(a[27]/a[28]))-(real_t)(1.0000000000000000e+00)),4)));
a[79] = (a[78]*a[78]);
a[80] = (exp(((real_t)(1.0000000000000000e+00)/(pow((((a[25]/a[26])+(a[27]/a[28]))-(real_t)(1.0000000000000000e+00)),4)))));
a[81] = (((real_t)(0.0000000000000000e+00)-((a[75]*(a[76]*a[77]))*a[79]))*a[80]);
a[82] = ((real_t)(4.0000000000000000e+00)*(pow((((a[30]/a[31])+(a[32]/a[33]))-(real_t)(1.0000000000000000e+00)),3)));
a[83] = ((real_t)(2.0000000000000000e+00)*(xd[0]-od[24]));
a[84] = ((real_t)(1.0000000000000000e+00)/a[31]);
a[85] = ((real_t)(1.0000000000000000e+00)/(pow((((a[30]/a[31])+(a[32]/a[33]))-(real_t)(1.0000000000000000e+00)),4)));
a[86] = (a[85]*a[85]);
a[87] = (exp(((real_t)(1.0000000000000000e+00)/(pow((((a[30]/a[31])+(a[32]/a[33]))-(real_t)(1.0000000000000000e+00)),4)))));
a[88] = (((real_t)(0.0000000000000000e+00)-((a[82]*(a[83]*a[84]))*a[86]))*a[87]);
a[89] = ((real_t)(4.0000000000000000e+00)*(pow((((a[35]/a[36])+(a[37]/a[38]))-(real_t)(1.0000000000000000e+00)),3)));
a[90] = ((real_t)(2.0000000000000000e+00)*(xd[0]-od[28]));
a[91] = ((real_t)(1.0000000000000000e+00)/a[36]);
a[92] = ((real_t)(1.0000000000000000e+00)/(pow((((a[35]/a[36])+(a[37]/a[38]))-(real_t)(1.0000000000000000e+00)),4)));
a[93] = (a[92]*a[92]);
a[94] = (exp(((real_t)(1.0000000000000000e+00)/(pow((((a[35]/a[36])+(a[37]/a[38]))-(real_t)(1.0000000000000000e+00)),4)))));
a[95] = (((real_t)(0.0000000000000000e+00)-((a[89]*(a[90]*a[91]))*a[93]))*a[94]);
a[96] = ((real_t)(2.0000000000000000e+00)*(xd[1]-od[1]));
a[97] = ((real_t)(1.0000000000000000e+00)/a[3]);
a[98] = (((real_t)(0.0000000000000000e+00)-((a[40]*(a[96]*a[97]))*a[44]))*a[45]);
a[99] = ((real_t)(2.0000000000000000e+00)*(xd[1]-od[5]));
a[100] = ((real_t)(1.0000000000000000e+00)/a[8]);
a[101] = (((real_t)(0.0000000000000000e+00)-((a[47]*(a[99]*a[100]))*a[51]))*a[52]);
a[102] = ((real_t)(2.0000000000000000e+00)*(xd[1]-od[9]));
a[103] = ((real_t)(1.0000000000000000e+00)/a[13]);
a[104] = (((real_t)(0.0000000000000000e+00)-((a[54]*(a[102]*a[103]))*a[58]))*a[59]);
a[105] = ((real_t)(2.0000000000000000e+00)*(xd[1]-od[13]));
a[106] = ((real_t)(1.0000000000000000e+00)/a[18]);
a[107] = (((real_t)(0.0000000000000000e+00)-((a[61]*(a[105]*a[106]))*a[65]))*a[66]);
a[108] = ((real_t)(2.0000000000000000e+00)*(xd[1]-od[17]));
a[109] = ((real_t)(1.0000000000000000e+00)/a[23]);
a[110] = (((real_t)(0.0000000000000000e+00)-((a[68]*(a[108]*a[109]))*a[72]))*a[73]);
a[111] = ((real_t)(2.0000000000000000e+00)*(xd[1]-od[21]));
a[112] = ((real_t)(1.0000000000000000e+00)/a[28]);
a[113] = (((real_t)(0.0000000000000000e+00)-((a[75]*(a[111]*a[112]))*a[79]))*a[80]);
a[114] = ((real_t)(2.0000000000000000e+00)*(xd[1]-od[25]));
a[115] = ((real_t)(1.0000000000000000e+00)/a[33]);
a[116] = (((real_t)(0.0000000000000000e+00)-((a[82]*(a[114]*a[115]))*a[86]))*a[87]);
a[117] = ((real_t)(2.0000000000000000e+00)*(xd[1]-od[29]));
a[118] = ((real_t)(1.0000000000000000e+00)/a[38]);
a[119] = (((real_t)(0.0000000000000000e+00)-((a[89]*(a[117]*a[118]))*a[93]))*a[94]);

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
out[4] = u[0];
out[5] = u[1];
out[6] = ((((((((xd[4]+a[4])+a[9])+a[14])+a[19])+a[24])+a[29])+a[34])+a[39]);
out[7] = (real_t)(1.0000000000000000e+00);
out[8] = (real_t)(0.0000000000000000e+00);
out[9] = (real_t)(0.0000000000000000e+00);
out[10] = (real_t)(0.0000000000000000e+00);
out[11] = (real_t)(0.0000000000000000e+00);
out[12] = (real_t)(1.0000000000000000e+00);
out[13] = (real_t)(0.0000000000000000e+00);
out[14] = (real_t)(0.0000000000000000e+00);
out[15] = (real_t)(0.0000000000000000e+00);
out[16] = (real_t)(0.0000000000000000e+00);
out[17] = (real_t)(1.0000000000000000e+00);
out[18] = (real_t)(0.0000000000000000e+00);
out[19] = (real_t)(0.0000000000000000e+00);
out[20] = (real_t)(0.0000000000000000e+00);
out[21] = (real_t)(0.0000000000000000e+00);
out[22] = (real_t)(1.0000000000000000e+00);
out[23] = (real_t)(0.0000000000000000e+00);
out[24] = (real_t)(0.0000000000000000e+00);
out[25] = (real_t)(0.0000000000000000e+00);
out[26] = (real_t)(0.0000000000000000e+00);
out[27] = (real_t)(0.0000000000000000e+00);
out[28] = (real_t)(0.0000000000000000e+00);
out[29] = (real_t)(0.0000000000000000e+00);
out[30] = (real_t)(0.0000000000000000e+00);
out[31] = (((((((a[46]+a[53])+a[60])+a[67])+a[74])+a[81])+a[88])+a[95]);
out[32] = (((((((a[98]+a[101])+a[104])+a[107])+a[110])+a[113])+a[116])+a[119]);
out[33] = (real_t)(0.0000000000000000e+00);
out[34] = (real_t)(0.0000000000000000e+00);
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

void acado_setObjQ1Q2( real_t* const tmpFx, real_t* const tmpObjS, real_t* const tmpQ1, real_t* const tmpQ2 )
{
tmpQ2[0] = + tmpFx[0]*tmpObjS[0] + tmpFx[4]*tmpObjS[7] + tmpFx[8]*tmpObjS[14] + tmpFx[12]*tmpObjS[21] + tmpFx[16]*tmpObjS[28] + tmpFx[20]*tmpObjS[35] + tmpFx[24]*tmpObjS[42];
tmpQ2[1] = + tmpFx[0]*tmpObjS[1] + tmpFx[4]*tmpObjS[8] + tmpFx[8]*tmpObjS[15] + tmpFx[12]*tmpObjS[22] + tmpFx[16]*tmpObjS[29] + tmpFx[20]*tmpObjS[36] + tmpFx[24]*tmpObjS[43];
tmpQ2[2] = + tmpFx[0]*tmpObjS[2] + tmpFx[4]*tmpObjS[9] + tmpFx[8]*tmpObjS[16] + tmpFx[12]*tmpObjS[23] + tmpFx[16]*tmpObjS[30] + tmpFx[20]*tmpObjS[37] + tmpFx[24]*tmpObjS[44];
tmpQ2[3] = + tmpFx[0]*tmpObjS[3] + tmpFx[4]*tmpObjS[10] + tmpFx[8]*tmpObjS[17] + tmpFx[12]*tmpObjS[24] + tmpFx[16]*tmpObjS[31] + tmpFx[20]*tmpObjS[38] + tmpFx[24]*tmpObjS[45];
tmpQ2[4] = + tmpFx[0]*tmpObjS[4] + tmpFx[4]*tmpObjS[11] + tmpFx[8]*tmpObjS[18] + tmpFx[12]*tmpObjS[25] + tmpFx[16]*tmpObjS[32] + tmpFx[20]*tmpObjS[39] + tmpFx[24]*tmpObjS[46];
tmpQ2[5] = + tmpFx[0]*tmpObjS[5] + tmpFx[4]*tmpObjS[12] + tmpFx[8]*tmpObjS[19] + tmpFx[12]*tmpObjS[26] + tmpFx[16]*tmpObjS[33] + tmpFx[20]*tmpObjS[40] + tmpFx[24]*tmpObjS[47];
tmpQ2[6] = + tmpFx[0]*tmpObjS[6] + tmpFx[4]*tmpObjS[13] + tmpFx[8]*tmpObjS[20] + tmpFx[12]*tmpObjS[27] + tmpFx[16]*tmpObjS[34] + tmpFx[20]*tmpObjS[41] + tmpFx[24]*tmpObjS[48];
tmpQ2[7] = + tmpFx[1]*tmpObjS[0] + tmpFx[5]*tmpObjS[7] + tmpFx[9]*tmpObjS[14] + tmpFx[13]*tmpObjS[21] + tmpFx[17]*tmpObjS[28] + tmpFx[21]*tmpObjS[35] + tmpFx[25]*tmpObjS[42];
tmpQ2[8] = + tmpFx[1]*tmpObjS[1] + tmpFx[5]*tmpObjS[8] + tmpFx[9]*tmpObjS[15] + tmpFx[13]*tmpObjS[22] + tmpFx[17]*tmpObjS[29] + tmpFx[21]*tmpObjS[36] + tmpFx[25]*tmpObjS[43];
tmpQ2[9] = + tmpFx[1]*tmpObjS[2] + tmpFx[5]*tmpObjS[9] + tmpFx[9]*tmpObjS[16] + tmpFx[13]*tmpObjS[23] + tmpFx[17]*tmpObjS[30] + tmpFx[21]*tmpObjS[37] + tmpFx[25]*tmpObjS[44];
tmpQ2[10] = + tmpFx[1]*tmpObjS[3] + tmpFx[5]*tmpObjS[10] + tmpFx[9]*tmpObjS[17] + tmpFx[13]*tmpObjS[24] + tmpFx[17]*tmpObjS[31] + tmpFx[21]*tmpObjS[38] + tmpFx[25]*tmpObjS[45];
tmpQ2[11] = + tmpFx[1]*tmpObjS[4] + tmpFx[5]*tmpObjS[11] + tmpFx[9]*tmpObjS[18] + tmpFx[13]*tmpObjS[25] + tmpFx[17]*tmpObjS[32] + tmpFx[21]*tmpObjS[39] + tmpFx[25]*tmpObjS[46];
tmpQ2[12] = + tmpFx[1]*tmpObjS[5] + tmpFx[5]*tmpObjS[12] + tmpFx[9]*tmpObjS[19] + tmpFx[13]*tmpObjS[26] + tmpFx[17]*tmpObjS[33] + tmpFx[21]*tmpObjS[40] + tmpFx[25]*tmpObjS[47];
tmpQ2[13] = + tmpFx[1]*tmpObjS[6] + tmpFx[5]*tmpObjS[13] + tmpFx[9]*tmpObjS[20] + tmpFx[13]*tmpObjS[27] + tmpFx[17]*tmpObjS[34] + tmpFx[21]*tmpObjS[41] + tmpFx[25]*tmpObjS[48];
tmpQ2[14] = + tmpFx[2]*tmpObjS[0] + tmpFx[6]*tmpObjS[7] + tmpFx[10]*tmpObjS[14] + tmpFx[14]*tmpObjS[21] + tmpFx[18]*tmpObjS[28] + tmpFx[22]*tmpObjS[35] + tmpFx[26]*tmpObjS[42];
tmpQ2[15] = + tmpFx[2]*tmpObjS[1] + tmpFx[6]*tmpObjS[8] + tmpFx[10]*tmpObjS[15] + tmpFx[14]*tmpObjS[22] + tmpFx[18]*tmpObjS[29] + tmpFx[22]*tmpObjS[36] + tmpFx[26]*tmpObjS[43];
tmpQ2[16] = + tmpFx[2]*tmpObjS[2] + tmpFx[6]*tmpObjS[9] + tmpFx[10]*tmpObjS[16] + tmpFx[14]*tmpObjS[23] + tmpFx[18]*tmpObjS[30] + tmpFx[22]*tmpObjS[37] + tmpFx[26]*tmpObjS[44];
tmpQ2[17] = + tmpFx[2]*tmpObjS[3] + tmpFx[6]*tmpObjS[10] + tmpFx[10]*tmpObjS[17] + tmpFx[14]*tmpObjS[24] + tmpFx[18]*tmpObjS[31] + tmpFx[22]*tmpObjS[38] + tmpFx[26]*tmpObjS[45];
tmpQ2[18] = + tmpFx[2]*tmpObjS[4] + tmpFx[6]*tmpObjS[11] + tmpFx[10]*tmpObjS[18] + tmpFx[14]*tmpObjS[25] + tmpFx[18]*tmpObjS[32] + tmpFx[22]*tmpObjS[39] + tmpFx[26]*tmpObjS[46];
tmpQ2[19] = + tmpFx[2]*tmpObjS[5] + tmpFx[6]*tmpObjS[12] + tmpFx[10]*tmpObjS[19] + tmpFx[14]*tmpObjS[26] + tmpFx[18]*tmpObjS[33] + tmpFx[22]*tmpObjS[40] + tmpFx[26]*tmpObjS[47];
tmpQ2[20] = + tmpFx[2]*tmpObjS[6] + tmpFx[6]*tmpObjS[13] + tmpFx[10]*tmpObjS[20] + tmpFx[14]*tmpObjS[27] + tmpFx[18]*tmpObjS[34] + tmpFx[22]*tmpObjS[41] + tmpFx[26]*tmpObjS[48];
tmpQ2[21] = + tmpFx[3]*tmpObjS[0] + tmpFx[7]*tmpObjS[7] + tmpFx[11]*tmpObjS[14] + tmpFx[15]*tmpObjS[21] + tmpFx[19]*tmpObjS[28] + tmpFx[23]*tmpObjS[35] + tmpFx[27]*tmpObjS[42];
tmpQ2[22] = + tmpFx[3]*tmpObjS[1] + tmpFx[7]*tmpObjS[8] + tmpFx[11]*tmpObjS[15] + tmpFx[15]*tmpObjS[22] + tmpFx[19]*tmpObjS[29] + tmpFx[23]*tmpObjS[36] + tmpFx[27]*tmpObjS[43];
tmpQ2[23] = + tmpFx[3]*tmpObjS[2] + tmpFx[7]*tmpObjS[9] + tmpFx[11]*tmpObjS[16] + tmpFx[15]*tmpObjS[23] + tmpFx[19]*tmpObjS[30] + tmpFx[23]*tmpObjS[37] + tmpFx[27]*tmpObjS[44];
tmpQ2[24] = + tmpFx[3]*tmpObjS[3] + tmpFx[7]*tmpObjS[10] + tmpFx[11]*tmpObjS[17] + tmpFx[15]*tmpObjS[24] + tmpFx[19]*tmpObjS[31] + tmpFx[23]*tmpObjS[38] + tmpFx[27]*tmpObjS[45];
tmpQ2[25] = + tmpFx[3]*tmpObjS[4] + tmpFx[7]*tmpObjS[11] + tmpFx[11]*tmpObjS[18] + tmpFx[15]*tmpObjS[25] + tmpFx[19]*tmpObjS[32] + tmpFx[23]*tmpObjS[39] + tmpFx[27]*tmpObjS[46];
tmpQ2[26] = + tmpFx[3]*tmpObjS[5] + tmpFx[7]*tmpObjS[12] + tmpFx[11]*tmpObjS[19] + tmpFx[15]*tmpObjS[26] + tmpFx[19]*tmpObjS[33] + tmpFx[23]*tmpObjS[40] + tmpFx[27]*tmpObjS[47];
tmpQ2[27] = + tmpFx[3]*tmpObjS[6] + tmpFx[7]*tmpObjS[13] + tmpFx[11]*tmpObjS[20] + tmpFx[15]*tmpObjS[27] + tmpFx[19]*tmpObjS[34] + tmpFx[23]*tmpObjS[41] + tmpFx[27]*tmpObjS[48];
tmpQ1[0] = + tmpQ2[0]*tmpFx[0] + tmpQ2[1]*tmpFx[4] + tmpQ2[2]*tmpFx[8] + tmpQ2[3]*tmpFx[12] + tmpQ2[4]*tmpFx[16] + tmpQ2[5]*tmpFx[20] + tmpQ2[6]*tmpFx[24];
tmpQ1[1] = + tmpQ2[0]*tmpFx[1] + tmpQ2[1]*tmpFx[5] + tmpQ2[2]*tmpFx[9] + tmpQ2[3]*tmpFx[13] + tmpQ2[4]*tmpFx[17] + tmpQ2[5]*tmpFx[21] + tmpQ2[6]*tmpFx[25];
tmpQ1[2] = + tmpQ2[0]*tmpFx[2] + tmpQ2[1]*tmpFx[6] + tmpQ2[2]*tmpFx[10] + tmpQ2[3]*tmpFx[14] + tmpQ2[4]*tmpFx[18] + tmpQ2[5]*tmpFx[22] + tmpQ2[6]*tmpFx[26];
tmpQ1[3] = + tmpQ2[0]*tmpFx[3] + tmpQ2[1]*tmpFx[7] + tmpQ2[2]*tmpFx[11] + tmpQ2[3]*tmpFx[15] + tmpQ2[4]*tmpFx[19] + tmpQ2[5]*tmpFx[23] + tmpQ2[6]*tmpFx[27];
tmpQ1[4] = + tmpQ2[7]*tmpFx[0] + tmpQ2[8]*tmpFx[4] + tmpQ2[9]*tmpFx[8] + tmpQ2[10]*tmpFx[12] + tmpQ2[11]*tmpFx[16] + tmpQ2[12]*tmpFx[20] + tmpQ2[13]*tmpFx[24];
tmpQ1[5] = + tmpQ2[7]*tmpFx[1] + tmpQ2[8]*tmpFx[5] + tmpQ2[9]*tmpFx[9] + tmpQ2[10]*tmpFx[13] + tmpQ2[11]*tmpFx[17] + tmpQ2[12]*tmpFx[21] + tmpQ2[13]*tmpFx[25];
tmpQ1[6] = + tmpQ2[7]*tmpFx[2] + tmpQ2[8]*tmpFx[6] + tmpQ2[9]*tmpFx[10] + tmpQ2[10]*tmpFx[14] + tmpQ2[11]*tmpFx[18] + tmpQ2[12]*tmpFx[22] + tmpQ2[13]*tmpFx[26];
tmpQ1[7] = + tmpQ2[7]*tmpFx[3] + tmpQ2[8]*tmpFx[7] + tmpQ2[9]*tmpFx[11] + tmpQ2[10]*tmpFx[15] + tmpQ2[11]*tmpFx[19] + tmpQ2[12]*tmpFx[23] + tmpQ2[13]*tmpFx[27];
tmpQ1[8] = + tmpQ2[14]*tmpFx[0] + tmpQ2[15]*tmpFx[4] + tmpQ2[16]*tmpFx[8] + tmpQ2[17]*tmpFx[12] + tmpQ2[18]*tmpFx[16] + tmpQ2[19]*tmpFx[20] + tmpQ2[20]*tmpFx[24];
tmpQ1[9] = + tmpQ2[14]*tmpFx[1] + tmpQ2[15]*tmpFx[5] + tmpQ2[16]*tmpFx[9] + tmpQ2[17]*tmpFx[13] + tmpQ2[18]*tmpFx[17] + tmpQ2[19]*tmpFx[21] + tmpQ2[20]*tmpFx[25];
tmpQ1[10] = + tmpQ2[14]*tmpFx[2] + tmpQ2[15]*tmpFx[6] + tmpQ2[16]*tmpFx[10] + tmpQ2[17]*tmpFx[14] + tmpQ2[18]*tmpFx[18] + tmpQ2[19]*tmpFx[22] + tmpQ2[20]*tmpFx[26];
tmpQ1[11] = + tmpQ2[14]*tmpFx[3] + tmpQ2[15]*tmpFx[7] + tmpQ2[16]*tmpFx[11] + tmpQ2[17]*tmpFx[15] + tmpQ2[18]*tmpFx[19] + tmpQ2[19]*tmpFx[23] + tmpQ2[20]*tmpFx[27];
tmpQ1[12] = + tmpQ2[21]*tmpFx[0] + tmpQ2[22]*tmpFx[4] + tmpQ2[23]*tmpFx[8] + tmpQ2[24]*tmpFx[12] + tmpQ2[25]*tmpFx[16] + tmpQ2[26]*tmpFx[20] + tmpQ2[27]*tmpFx[24];
tmpQ1[13] = + tmpQ2[21]*tmpFx[1] + tmpQ2[22]*tmpFx[5] + tmpQ2[23]*tmpFx[9] + tmpQ2[24]*tmpFx[13] + tmpQ2[25]*tmpFx[17] + tmpQ2[26]*tmpFx[21] + tmpQ2[27]*tmpFx[25];
tmpQ1[14] = + tmpQ2[21]*tmpFx[2] + tmpQ2[22]*tmpFx[6] + tmpQ2[23]*tmpFx[10] + tmpQ2[24]*tmpFx[14] + tmpQ2[25]*tmpFx[18] + tmpQ2[26]*tmpFx[22] + tmpQ2[27]*tmpFx[26];
tmpQ1[15] = + tmpQ2[21]*tmpFx[3] + tmpQ2[22]*tmpFx[7] + tmpQ2[23]*tmpFx[11] + tmpQ2[24]*tmpFx[15] + tmpQ2[25]*tmpFx[19] + tmpQ2[26]*tmpFx[23] + tmpQ2[27]*tmpFx[27];
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
for (runObj = 0; runObj < 16; ++runObj)
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

acado_setObjQ1Q2( &(acadoWorkspace.objValueOut[ 7 ]), &(acadoVariables.W[ runObj * 49 ]), &(acadoWorkspace.Q1[ runObj * 16 ]), &(acadoWorkspace.Q2[ runObj * 28 ]) );

acado_setObjR1R2( &(acadoVariables.W[ runObj * 49 ]), &(acadoWorkspace.R1[ runObj * 4 ]), &(acadoWorkspace.R2[ runObj * 14 ]) );

}
acadoWorkspace.objValueIn[0] = acadoVariables.x[64];
acadoWorkspace.objValueIn[1] = acadoVariables.x[65];
acadoWorkspace.objValueIn[2] = acadoVariables.x[66];
acadoWorkspace.objValueIn[3] = acadoVariables.x[67];
acadoWorkspace.objValueIn[4] = acadoVariables.od[512];
acadoWorkspace.objValueIn[5] = acadoVariables.od[513];
acadoWorkspace.objValueIn[6] = acadoVariables.od[514];
acadoWorkspace.objValueIn[7] = acadoVariables.od[515];
acadoWorkspace.objValueIn[8] = acadoVariables.od[516];
acadoWorkspace.objValueIn[9] = acadoVariables.od[517];
acadoWorkspace.objValueIn[10] = acadoVariables.od[518];
acadoWorkspace.objValueIn[11] = acadoVariables.od[519];
acadoWorkspace.objValueIn[12] = acadoVariables.od[520];
acadoWorkspace.objValueIn[13] = acadoVariables.od[521];
acadoWorkspace.objValueIn[14] = acadoVariables.od[522];
acadoWorkspace.objValueIn[15] = acadoVariables.od[523];
acadoWorkspace.objValueIn[16] = acadoVariables.od[524];
acadoWorkspace.objValueIn[17] = acadoVariables.od[525];
acadoWorkspace.objValueIn[18] = acadoVariables.od[526];
acadoWorkspace.objValueIn[19] = acadoVariables.od[527];
acadoWorkspace.objValueIn[20] = acadoVariables.od[528];
acadoWorkspace.objValueIn[21] = acadoVariables.od[529];
acadoWorkspace.objValueIn[22] = acadoVariables.od[530];
acadoWorkspace.objValueIn[23] = acadoVariables.od[531];
acadoWorkspace.objValueIn[24] = acadoVariables.od[532];
acadoWorkspace.objValueIn[25] = acadoVariables.od[533];
acadoWorkspace.objValueIn[26] = acadoVariables.od[534];
acadoWorkspace.objValueIn[27] = acadoVariables.od[535];
acadoWorkspace.objValueIn[28] = acadoVariables.od[536];
acadoWorkspace.objValueIn[29] = acadoVariables.od[537];
acadoWorkspace.objValueIn[30] = acadoVariables.od[538];
acadoWorkspace.objValueIn[31] = acadoVariables.od[539];
acadoWorkspace.objValueIn[32] = acadoVariables.od[540];
acadoWorkspace.objValueIn[33] = acadoVariables.od[541];
acadoWorkspace.objValueIn[34] = acadoVariables.od[542];
acadoWorkspace.objValueIn[35] = acadoVariables.od[543];
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
acadoWorkspace.H[(iRow * 64) + (iCol * 2)] += + Gu1[0]*Gu2[0] + Gu1[2]*Gu2[2] + Gu1[4]*Gu2[4] + Gu1[6]*Gu2[6];
acadoWorkspace.H[(iRow * 64) + (iCol * 2 + 1)] += + Gu1[0]*Gu2[1] + Gu1[2]*Gu2[3] + Gu1[4]*Gu2[5] + Gu1[6]*Gu2[7];
acadoWorkspace.H[(iRow * 64 + 32) + (iCol * 2)] += + Gu1[1]*Gu2[0] + Gu1[3]*Gu2[2] + Gu1[5]*Gu2[4] + Gu1[7]*Gu2[6];
acadoWorkspace.H[(iRow * 64 + 32) + (iCol * 2 + 1)] += + Gu1[1]*Gu2[1] + Gu1[3]*Gu2[3] + Gu1[5]*Gu2[5] + Gu1[7]*Gu2[7];
}

void acado_setBlockH11_R1( int iRow, int iCol, real_t* const R11 )
{
acadoWorkspace.H[(iRow * 64) + (iCol * 2)] = R11[0];
acadoWorkspace.H[(iRow * 64) + (iCol * 2 + 1)] = R11[1];
acadoWorkspace.H[(iRow * 64 + 32) + (iCol * 2)] = R11[2];
acadoWorkspace.H[(iRow * 64 + 32) + (iCol * 2 + 1)] = R11[3];
}

void acado_zeroBlockH11( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 64) + (iCol * 2)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 64) + (iCol * 2 + 1)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 64 + 32) + (iCol * 2)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 64 + 32) + (iCol * 2 + 1)] = 0.0000000000000000e+00;
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 64) + (iCol * 2)] = acadoWorkspace.H[(iCol * 64) + (iRow * 2)];
acadoWorkspace.H[(iRow * 64) + (iCol * 2 + 1)] = acadoWorkspace.H[(iCol * 64 + 32) + (iRow * 2)];
acadoWorkspace.H[(iRow * 64 + 32) + (iCol * 2)] = acadoWorkspace.H[(iCol * 64) + (iRow * 2 + 1)];
acadoWorkspace.H[(iRow * 64 + 32) + (iCol * 2 + 1)] = acadoWorkspace.H[(iCol * 64 + 32) + (iRow * 2 + 1)];
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

void acado_multHxC( real_t* const Hx, real_t* const Gx, real_t* const A01 )
{
A01[0] = + Hx[0]*Gx[0] + Hx[1]*Gx[4] + Hx[2]*Gx[8] + Hx[3]*Gx[12];
A01[1] = + Hx[0]*Gx[1] + Hx[1]*Gx[5] + Hx[2]*Gx[9] + Hx[3]*Gx[13];
A01[2] = + Hx[0]*Gx[2] + Hx[1]*Gx[6] + Hx[2]*Gx[10] + Hx[3]*Gx[14];
A01[3] = + Hx[0]*Gx[3] + Hx[1]*Gx[7] + Hx[2]*Gx[11] + Hx[3]*Gx[15];
A01[4] = + Hx[4]*Gx[0] + Hx[5]*Gx[4] + Hx[6]*Gx[8] + Hx[7]*Gx[12];
A01[5] = + Hx[4]*Gx[1] + Hx[5]*Gx[5] + Hx[6]*Gx[9] + Hx[7]*Gx[13];
A01[6] = + Hx[4]*Gx[2] + Hx[5]*Gx[6] + Hx[6]*Gx[10] + Hx[7]*Gx[14];
A01[7] = + Hx[4]*Gx[3] + Hx[5]*Gx[7] + Hx[6]*Gx[11] + Hx[7]*Gx[15];
A01[8] = + Hx[8]*Gx[0] + Hx[9]*Gx[4] + Hx[10]*Gx[8] + Hx[11]*Gx[12];
A01[9] = + Hx[8]*Gx[1] + Hx[9]*Gx[5] + Hx[10]*Gx[9] + Hx[11]*Gx[13];
A01[10] = + Hx[8]*Gx[2] + Hx[9]*Gx[6] + Hx[10]*Gx[10] + Hx[11]*Gx[14];
A01[11] = + Hx[8]*Gx[3] + Hx[9]*Gx[7] + Hx[10]*Gx[11] + Hx[11]*Gx[15];
A01[12] = + Hx[12]*Gx[0] + Hx[13]*Gx[4] + Hx[14]*Gx[8] + Hx[15]*Gx[12];
A01[13] = + Hx[12]*Gx[1] + Hx[13]*Gx[5] + Hx[14]*Gx[9] + Hx[15]*Gx[13];
A01[14] = + Hx[12]*Gx[2] + Hx[13]*Gx[6] + Hx[14]*Gx[10] + Hx[15]*Gx[14];
A01[15] = + Hx[12]*Gx[3] + Hx[13]*Gx[7] + Hx[14]*Gx[11] + Hx[15]*Gx[15];
A01[16] = + Hx[16]*Gx[0] + Hx[17]*Gx[4] + Hx[18]*Gx[8] + Hx[19]*Gx[12];
A01[17] = + Hx[16]*Gx[1] + Hx[17]*Gx[5] + Hx[18]*Gx[9] + Hx[19]*Gx[13];
A01[18] = + Hx[16]*Gx[2] + Hx[17]*Gx[6] + Hx[18]*Gx[10] + Hx[19]*Gx[14];
A01[19] = + Hx[16]*Gx[3] + Hx[17]*Gx[7] + Hx[18]*Gx[11] + Hx[19]*Gx[15];
A01[20] = + Hx[20]*Gx[0] + Hx[21]*Gx[4] + Hx[22]*Gx[8] + Hx[23]*Gx[12];
A01[21] = + Hx[20]*Gx[1] + Hx[21]*Gx[5] + Hx[22]*Gx[9] + Hx[23]*Gx[13];
A01[22] = + Hx[20]*Gx[2] + Hx[21]*Gx[6] + Hx[22]*Gx[10] + Hx[23]*Gx[14];
A01[23] = + Hx[20]*Gx[3] + Hx[21]*Gx[7] + Hx[22]*Gx[11] + Hx[23]*Gx[15];
A01[24] = + Hx[24]*Gx[0] + Hx[25]*Gx[4] + Hx[26]*Gx[8] + Hx[27]*Gx[12];
A01[25] = + Hx[24]*Gx[1] + Hx[25]*Gx[5] + Hx[26]*Gx[9] + Hx[27]*Gx[13];
A01[26] = + Hx[24]*Gx[2] + Hx[25]*Gx[6] + Hx[26]*Gx[10] + Hx[27]*Gx[14];
A01[27] = + Hx[24]*Gx[3] + Hx[25]*Gx[7] + Hx[26]*Gx[11] + Hx[27]*Gx[15];
A01[28] = + Hx[28]*Gx[0] + Hx[29]*Gx[4] + Hx[30]*Gx[8] + Hx[31]*Gx[12];
A01[29] = + Hx[28]*Gx[1] + Hx[29]*Gx[5] + Hx[30]*Gx[9] + Hx[31]*Gx[13];
A01[30] = + Hx[28]*Gx[2] + Hx[29]*Gx[6] + Hx[30]*Gx[10] + Hx[31]*Gx[14];
A01[31] = + Hx[28]*Gx[3] + Hx[29]*Gx[7] + Hx[30]*Gx[11] + Hx[31]*Gx[15];
}

void acado_multHxE( real_t* const Hx, real_t* const E, int row, int col )
{
acadoWorkspace.A[(row * 256) + (col * 2)] = + Hx[0]*E[0] + Hx[1]*E[2] + Hx[2]*E[4] + Hx[3]*E[6];
acadoWorkspace.A[(row * 256) + (col * 2 + 1)] = + Hx[0]*E[1] + Hx[1]*E[3] + Hx[2]*E[5] + Hx[3]*E[7];
acadoWorkspace.A[(row * 256 + 32) + (col * 2)] = + Hx[4]*E[0] + Hx[5]*E[2] + Hx[6]*E[4] + Hx[7]*E[6];
acadoWorkspace.A[(row * 256 + 32) + (col * 2 + 1)] = + Hx[4]*E[1] + Hx[5]*E[3] + Hx[6]*E[5] + Hx[7]*E[7];
acadoWorkspace.A[(row * 256 + 64) + (col * 2)] = + Hx[8]*E[0] + Hx[9]*E[2] + Hx[10]*E[4] + Hx[11]*E[6];
acadoWorkspace.A[(row * 256 + 64) + (col * 2 + 1)] = + Hx[8]*E[1] + Hx[9]*E[3] + Hx[10]*E[5] + Hx[11]*E[7];
acadoWorkspace.A[(row * 256 + 96) + (col * 2)] = + Hx[12]*E[0] + Hx[13]*E[2] + Hx[14]*E[4] + Hx[15]*E[6];
acadoWorkspace.A[(row * 256 + 96) + (col * 2 + 1)] = + Hx[12]*E[1] + Hx[13]*E[3] + Hx[14]*E[5] + Hx[15]*E[7];
acadoWorkspace.A[(row * 256 + 128) + (col * 2)] = + Hx[16]*E[0] + Hx[17]*E[2] + Hx[18]*E[4] + Hx[19]*E[6];
acadoWorkspace.A[(row * 256 + 128) + (col * 2 + 1)] = + Hx[16]*E[1] + Hx[17]*E[3] + Hx[18]*E[5] + Hx[19]*E[7];
acadoWorkspace.A[(row * 256 + 160) + (col * 2)] = + Hx[20]*E[0] + Hx[21]*E[2] + Hx[22]*E[4] + Hx[23]*E[6];
acadoWorkspace.A[(row * 256 + 160) + (col * 2 + 1)] = + Hx[20]*E[1] + Hx[21]*E[3] + Hx[22]*E[5] + Hx[23]*E[7];
acadoWorkspace.A[(row * 256 + 192) + (col * 2)] = + Hx[24]*E[0] + Hx[25]*E[2] + Hx[26]*E[4] + Hx[27]*E[6];
acadoWorkspace.A[(row * 256 + 192) + (col * 2 + 1)] = + Hx[24]*E[1] + Hx[25]*E[3] + Hx[26]*E[5] + Hx[27]*E[7];
acadoWorkspace.A[(row * 256 + 224) + (col * 2)] = + Hx[28]*E[0] + Hx[29]*E[2] + Hx[30]*E[4] + Hx[31]*E[6];
acadoWorkspace.A[(row * 256 + 224) + (col * 2 + 1)] = + Hx[28]*E[1] + Hx[29]*E[3] + Hx[30]*E[5] + Hx[31]*E[7];
}

void acado_macHxd( real_t* const Hx, real_t* const tmpd, real_t* const lbA, real_t* const ubA )
{
acadoWorkspace.evHxd[0] = + Hx[0]*tmpd[0] + Hx[1]*tmpd[1] + Hx[2]*tmpd[2] + Hx[3]*tmpd[3];
acadoWorkspace.evHxd[1] = + Hx[4]*tmpd[0] + Hx[5]*tmpd[1] + Hx[6]*tmpd[2] + Hx[7]*tmpd[3];
acadoWorkspace.evHxd[2] = + Hx[8]*tmpd[0] + Hx[9]*tmpd[1] + Hx[10]*tmpd[2] + Hx[11]*tmpd[3];
acadoWorkspace.evHxd[3] = + Hx[12]*tmpd[0] + Hx[13]*tmpd[1] + Hx[14]*tmpd[2] + Hx[15]*tmpd[3];
acadoWorkspace.evHxd[4] = + Hx[16]*tmpd[0] + Hx[17]*tmpd[1] + Hx[18]*tmpd[2] + Hx[19]*tmpd[3];
acadoWorkspace.evHxd[5] = + Hx[20]*tmpd[0] + Hx[21]*tmpd[1] + Hx[22]*tmpd[2] + Hx[23]*tmpd[3];
acadoWorkspace.evHxd[6] = + Hx[24]*tmpd[0] + Hx[25]*tmpd[1] + Hx[26]*tmpd[2] + Hx[27]*tmpd[3];
acadoWorkspace.evHxd[7] = + Hx[28]*tmpd[0] + Hx[29]*tmpd[1] + Hx[30]*tmpd[2] + Hx[31]*tmpd[3];
lbA[0] -= acadoWorkspace.evHxd[0];
lbA[1] -= acadoWorkspace.evHxd[1];
lbA[2] -= acadoWorkspace.evHxd[2];
lbA[3] -= acadoWorkspace.evHxd[3];
lbA[4] -= acadoWorkspace.evHxd[4];
lbA[5] -= acadoWorkspace.evHxd[5];
lbA[6] -= acadoWorkspace.evHxd[6];
lbA[7] -= acadoWorkspace.evHxd[7];
ubA[0] -= acadoWorkspace.evHxd[0];
ubA[1] -= acadoWorkspace.evHxd[1];
ubA[2] -= acadoWorkspace.evHxd[2];
ubA[3] -= acadoWorkspace.evHxd[3];
ubA[4] -= acadoWorkspace.evHxd[4];
ubA[5] -= acadoWorkspace.evHxd[5];
ubA[6] -= acadoWorkspace.evHxd[6];
ubA[7] -= acadoWorkspace.evHxd[7];
}

void acado_evaluatePathConstraints(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* od = in + 6;
/* Vector of auxiliary variables; number of elements: 112. */
real_t* a = acadoWorkspace.conAuxVar;

/* Compute intermediate quantities: */
a[0] = (pow((xd[0]-od[0]),2));
a[1] = ((od[2])*(od[2]));
a[2] = (pow((xd[1]-od[1]),2));
a[3] = ((od[3])*(od[3]));
a[4] = (pow((xd[0]-od[4]),2));
a[5] = ((od[6])*(od[6]));
a[6] = (pow((xd[1]-od[5]),2));
a[7] = ((od[7])*(od[7]));
a[8] = (pow((xd[0]-od[8]),2));
a[9] = ((od[10])*(od[10]));
a[10] = (pow((xd[1]-od[9]),2));
a[11] = ((od[11])*(od[11]));
a[12] = (pow((xd[0]-od[12]),2));
a[13] = ((od[14])*(od[14]));
a[14] = (pow((xd[1]-od[13]),2));
a[15] = ((od[15])*(od[15]));
a[16] = (pow((xd[0]-od[16]),2));
a[17] = ((od[18])*(od[18]));
a[18] = (pow((xd[1]-od[17]),2));
a[19] = ((od[19])*(od[19]));
a[20] = (pow((xd[0]-od[20]),2));
a[21] = ((od[22])*(od[22]));
a[22] = (pow((xd[1]-od[21]),2));
a[23] = ((od[23])*(od[23]));
a[24] = (pow((xd[0]-od[24]),2));
a[25] = ((od[26])*(od[26]));
a[26] = (pow((xd[1]-od[25]),2));
a[27] = ((od[27])*(od[27]));
a[28] = (pow((xd[0]-od[28]),2));
a[29] = ((od[30])*(od[30]));
a[30] = (pow((xd[1]-od[29]),2));
a[31] = ((od[31])*(od[31]));
a[32] = ((real_t)(2.0000000000000000e+00)*(xd[0]-od[0]));
a[33] = ((real_t)(1.0000000000000000e+00)/a[1]);
a[34] = (a[32]*a[33]);
a[35] = ((real_t)(2.0000000000000000e+00)*(xd[1]-od[1]));
a[36] = ((real_t)(1.0000000000000000e+00)/a[3]);
a[37] = (a[35]*a[36]);
a[38] = (real_t)(0.0000000000000000e+00);
a[39] = (real_t)(0.0000000000000000e+00);
a[40] = ((real_t)(2.0000000000000000e+00)*(xd[0]-od[4]));
a[41] = ((real_t)(1.0000000000000000e+00)/a[5]);
a[42] = (a[40]*a[41]);
a[43] = ((real_t)(2.0000000000000000e+00)*(xd[1]-od[5]));
a[44] = ((real_t)(1.0000000000000000e+00)/a[7]);
a[45] = (a[43]*a[44]);
a[46] = (real_t)(0.0000000000000000e+00);
a[47] = (real_t)(0.0000000000000000e+00);
a[48] = ((real_t)(2.0000000000000000e+00)*(xd[0]-od[8]));
a[49] = ((real_t)(1.0000000000000000e+00)/a[9]);
a[50] = (a[48]*a[49]);
a[51] = ((real_t)(2.0000000000000000e+00)*(xd[1]-od[9]));
a[52] = ((real_t)(1.0000000000000000e+00)/a[11]);
a[53] = (a[51]*a[52]);
a[54] = (real_t)(0.0000000000000000e+00);
a[55] = (real_t)(0.0000000000000000e+00);
a[56] = ((real_t)(2.0000000000000000e+00)*(xd[0]-od[12]));
a[57] = ((real_t)(1.0000000000000000e+00)/a[13]);
a[58] = (a[56]*a[57]);
a[59] = ((real_t)(2.0000000000000000e+00)*(xd[1]-od[13]));
a[60] = ((real_t)(1.0000000000000000e+00)/a[15]);
a[61] = (a[59]*a[60]);
a[62] = (real_t)(0.0000000000000000e+00);
a[63] = (real_t)(0.0000000000000000e+00);
a[64] = ((real_t)(2.0000000000000000e+00)*(xd[0]-od[16]));
a[65] = ((real_t)(1.0000000000000000e+00)/a[17]);
a[66] = (a[64]*a[65]);
a[67] = ((real_t)(2.0000000000000000e+00)*(xd[1]-od[17]));
a[68] = ((real_t)(1.0000000000000000e+00)/a[19]);
a[69] = (a[67]*a[68]);
a[70] = (real_t)(0.0000000000000000e+00);
a[71] = (real_t)(0.0000000000000000e+00);
a[72] = ((real_t)(2.0000000000000000e+00)*(xd[0]-od[20]));
a[73] = ((real_t)(1.0000000000000000e+00)/a[21]);
a[74] = (a[72]*a[73]);
a[75] = ((real_t)(2.0000000000000000e+00)*(xd[1]-od[21]));
a[76] = ((real_t)(1.0000000000000000e+00)/a[23]);
a[77] = (a[75]*a[76]);
a[78] = (real_t)(0.0000000000000000e+00);
a[79] = (real_t)(0.0000000000000000e+00);
a[80] = ((real_t)(2.0000000000000000e+00)*(xd[0]-od[24]));
a[81] = ((real_t)(1.0000000000000000e+00)/a[25]);
a[82] = (a[80]*a[81]);
a[83] = ((real_t)(2.0000000000000000e+00)*(xd[1]-od[25]));
a[84] = ((real_t)(1.0000000000000000e+00)/a[27]);
a[85] = (a[83]*a[84]);
a[86] = (real_t)(0.0000000000000000e+00);
a[87] = (real_t)(0.0000000000000000e+00);
a[88] = ((real_t)(2.0000000000000000e+00)*(xd[0]-od[28]));
a[89] = ((real_t)(1.0000000000000000e+00)/a[29]);
a[90] = (a[88]*a[89]);
a[91] = ((real_t)(2.0000000000000000e+00)*(xd[1]-od[29]));
a[92] = ((real_t)(1.0000000000000000e+00)/a[31]);
a[93] = (a[91]*a[92]);
a[94] = (real_t)(0.0000000000000000e+00);
a[95] = (real_t)(0.0000000000000000e+00);
a[96] = (real_t)(0.0000000000000000e+00);
a[97] = (real_t)(0.0000000000000000e+00);
a[98] = (real_t)(0.0000000000000000e+00);
a[99] = (real_t)(0.0000000000000000e+00);
a[100] = (real_t)(0.0000000000000000e+00);
a[101] = (real_t)(0.0000000000000000e+00);
a[102] = (real_t)(0.0000000000000000e+00);
a[103] = (real_t)(0.0000000000000000e+00);
a[104] = (real_t)(0.0000000000000000e+00);
a[105] = (real_t)(0.0000000000000000e+00);
a[106] = (real_t)(0.0000000000000000e+00);
a[107] = (real_t)(0.0000000000000000e+00);
a[108] = (real_t)(0.0000000000000000e+00);
a[109] = (real_t)(0.0000000000000000e+00);
a[110] = (real_t)(0.0000000000000000e+00);
a[111] = (real_t)(0.0000000000000000e+00);

/* Compute outputs: */
out[0] = (((a[0]/a[1])+(a[2]/a[3]))-(real_t)(1.9399999999999999e+00));
out[1] = (((a[4]/a[5])+(a[6]/a[7]))-(real_t)(1.9399999999999999e+00));
out[2] = (((a[8]/a[9])+(a[10]/a[11]))-(real_t)(1.9399999999999999e+00));
out[3] = (((a[12]/a[13])+(a[14]/a[15]))-(real_t)(1.9399999999999999e+00));
out[4] = (((a[16]/a[17])+(a[18]/a[19]))-(real_t)(1.9399999999999999e+00));
out[5] = (((a[20]/a[21])+(a[22]/a[23]))-(real_t)(1.9399999999999999e+00));
out[6] = (((a[24]/a[25])+(a[26]/a[27]))-(real_t)(1.9399999999999999e+00));
out[7] = (((a[28]/a[29])+(a[30]/a[31]))-(real_t)(1.9399999999999999e+00));
out[8] = a[34];
out[9] = a[37];
out[10] = a[38];
out[11] = a[39];
out[12] = a[42];
out[13] = a[45];
out[14] = a[46];
out[15] = a[47];
out[16] = a[50];
out[17] = a[53];
out[18] = a[54];
out[19] = a[55];
out[20] = a[58];
out[21] = a[61];
out[22] = a[62];
out[23] = a[63];
out[24] = a[66];
out[25] = a[69];
out[26] = a[70];
out[27] = a[71];
out[28] = a[74];
out[29] = a[77];
out[30] = a[78];
out[31] = a[79];
out[32] = a[82];
out[33] = a[85];
out[34] = a[86];
out[35] = a[87];
out[36] = a[90];
out[37] = a[93];
out[38] = a[94];
out[39] = a[95];
out[40] = a[96];
out[41] = a[97];
out[42] = a[98];
out[43] = a[99];
out[44] = a[100];
out[45] = a[101];
out[46] = a[102];
out[47] = a[103];
out[48] = a[104];
out[49] = a[105];
out[50] = a[106];
out[51] = a[107];
out[52] = a[108];
out[53] = a[109];
out[54] = a[110];
out[55] = a[111];
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
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 960 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 968 ]), &(acadoWorkspace.QE[ 968 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 976 ]), &(acadoWorkspace.QE[ 976 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 984 ]), &(acadoWorkspace.QE[ 984 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 992 ]), &(acadoWorkspace.QE[ 992 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 1000 ]), &(acadoWorkspace.QE[ 1000 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 1008 ]), &(acadoWorkspace.QE[ 1008 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 1016 ]), &(acadoWorkspace.QE[ 1016 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 1024 ]), &(acadoWorkspace.QE[ 1024 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 1032 ]), &(acadoWorkspace.QE[ 1032 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 1040 ]), &(acadoWorkspace.QE[ 1040 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 1048 ]), &(acadoWorkspace.QE[ 1048 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 1056 ]), &(acadoWorkspace.QE[ 1056 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 1064 ]), &(acadoWorkspace.QE[ 1064 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 1072 ]), &(acadoWorkspace.QE[ 1072 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 1080 ]), &(acadoWorkspace.QE[ 1080 ]) );

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
acado_zeroBlockH10( &(acadoWorkspace.H10[ 64 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 352 ]), &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.H10[ 64 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 424 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.H10[ 64 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 504 ]), &(acadoWorkspace.evGx[ 160 ]), &(acadoWorkspace.H10[ 64 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 592 ]), &(acadoWorkspace.evGx[ 176 ]), &(acadoWorkspace.H10[ 64 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 688 ]), &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.H10[ 64 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 792 ]), &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.H10[ 64 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 904 ]), &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.H10[ 64 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1024 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.H10[ 64 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 72 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 432 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.H10[ 72 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 512 ]), &(acadoWorkspace.evGx[ 160 ]), &(acadoWorkspace.H10[ 72 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 600 ]), &(acadoWorkspace.evGx[ 176 ]), &(acadoWorkspace.H10[ 72 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 696 ]), &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.H10[ 72 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 800 ]), &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.H10[ 72 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 912 ]), &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.H10[ 72 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1032 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.H10[ 72 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 80 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 520 ]), &(acadoWorkspace.evGx[ 160 ]), &(acadoWorkspace.H10[ 80 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 608 ]), &(acadoWorkspace.evGx[ 176 ]), &(acadoWorkspace.H10[ 80 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 704 ]), &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.H10[ 80 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 808 ]), &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.H10[ 80 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 920 ]), &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.H10[ 80 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1040 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.H10[ 80 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 88 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 616 ]), &(acadoWorkspace.evGx[ 176 ]), &(acadoWorkspace.H10[ 88 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 712 ]), &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.H10[ 88 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 816 ]), &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.H10[ 88 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 928 ]), &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.H10[ 88 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1048 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.H10[ 88 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 720 ]), &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 824 ]), &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 936 ]), &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1056 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 104 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 832 ]), &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.H10[ 104 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 944 ]), &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.H10[ 104 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1064 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.H10[ 104 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 112 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 952 ]), &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.H10[ 112 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1072 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.H10[ 112 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 120 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1080 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.H10[ 120 ]) );

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

acado_zeroBlockH11( 0, 8 );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 352 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 424 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 440 ]), &(acadoWorkspace.QE[ 504 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.QE[ 592 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.QE[ 688 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 728 ]), &(acadoWorkspace.QE[ 792 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 840 ]), &(acadoWorkspace.QE[ 904 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 1024 ]) );

acado_zeroBlockH11( 0, 9 );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 432 ]) );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 440 ]), &(acadoWorkspace.QE[ 512 ]) );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.QE[ 600 ]) );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.QE[ 696 ]) );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 728 ]), &(acadoWorkspace.QE[ 800 ]) );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 840 ]), &(acadoWorkspace.QE[ 912 ]) );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 1032 ]) );

acado_zeroBlockH11( 0, 10 );
acado_setBlockH11( 0, 10, &(acadoWorkspace.E[ 440 ]), &(acadoWorkspace.QE[ 520 ]) );
acado_setBlockH11( 0, 10, &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.QE[ 608 ]) );
acado_setBlockH11( 0, 10, &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.QE[ 704 ]) );
acado_setBlockH11( 0, 10, &(acadoWorkspace.E[ 728 ]), &(acadoWorkspace.QE[ 808 ]) );
acado_setBlockH11( 0, 10, &(acadoWorkspace.E[ 840 ]), &(acadoWorkspace.QE[ 920 ]) );
acado_setBlockH11( 0, 10, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 1040 ]) );

acado_zeroBlockH11( 0, 11 );
acado_setBlockH11( 0, 11, &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.QE[ 616 ]) );
acado_setBlockH11( 0, 11, &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.QE[ 712 ]) );
acado_setBlockH11( 0, 11, &(acadoWorkspace.E[ 728 ]), &(acadoWorkspace.QE[ 816 ]) );
acado_setBlockH11( 0, 11, &(acadoWorkspace.E[ 840 ]), &(acadoWorkspace.QE[ 928 ]) );
acado_setBlockH11( 0, 11, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 1048 ]) );

acado_zeroBlockH11( 0, 12 );
acado_setBlockH11( 0, 12, &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.QE[ 720 ]) );
acado_setBlockH11( 0, 12, &(acadoWorkspace.E[ 728 ]), &(acadoWorkspace.QE[ 824 ]) );
acado_setBlockH11( 0, 12, &(acadoWorkspace.E[ 840 ]), &(acadoWorkspace.QE[ 936 ]) );
acado_setBlockH11( 0, 12, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 1056 ]) );

acado_zeroBlockH11( 0, 13 );
acado_setBlockH11( 0, 13, &(acadoWorkspace.E[ 728 ]), &(acadoWorkspace.QE[ 832 ]) );
acado_setBlockH11( 0, 13, &(acadoWorkspace.E[ 840 ]), &(acadoWorkspace.QE[ 944 ]) );
acado_setBlockH11( 0, 13, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 1064 ]) );

acado_zeroBlockH11( 0, 14 );
acado_setBlockH11( 0, 14, &(acadoWorkspace.E[ 840 ]), &(acadoWorkspace.QE[ 952 ]) );
acado_setBlockH11( 0, 14, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 1072 ]) );

acado_zeroBlockH11( 0, 15 );
acado_setBlockH11( 0, 15, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 1080 ]) );

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

acado_zeroBlockH11( 1, 8 );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 296 ]), &(acadoWorkspace.QE[ 352 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 368 ]), &(acadoWorkspace.QE[ 424 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 448 ]), &(acadoWorkspace.QE[ 504 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 536 ]), &(acadoWorkspace.QE[ 592 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 632 ]), &(acadoWorkspace.QE[ 688 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 736 ]), &(acadoWorkspace.QE[ 792 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 848 ]), &(acadoWorkspace.QE[ 904 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 968 ]), &(acadoWorkspace.QE[ 1024 ]) );

acado_zeroBlockH11( 1, 9 );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 368 ]), &(acadoWorkspace.QE[ 432 ]) );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 448 ]), &(acadoWorkspace.QE[ 512 ]) );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 536 ]), &(acadoWorkspace.QE[ 600 ]) );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 632 ]), &(acadoWorkspace.QE[ 696 ]) );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 736 ]), &(acadoWorkspace.QE[ 800 ]) );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 848 ]), &(acadoWorkspace.QE[ 912 ]) );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 968 ]), &(acadoWorkspace.QE[ 1032 ]) );

acado_zeroBlockH11( 1, 10 );
acado_setBlockH11( 1, 10, &(acadoWorkspace.E[ 448 ]), &(acadoWorkspace.QE[ 520 ]) );
acado_setBlockH11( 1, 10, &(acadoWorkspace.E[ 536 ]), &(acadoWorkspace.QE[ 608 ]) );
acado_setBlockH11( 1, 10, &(acadoWorkspace.E[ 632 ]), &(acadoWorkspace.QE[ 704 ]) );
acado_setBlockH11( 1, 10, &(acadoWorkspace.E[ 736 ]), &(acadoWorkspace.QE[ 808 ]) );
acado_setBlockH11( 1, 10, &(acadoWorkspace.E[ 848 ]), &(acadoWorkspace.QE[ 920 ]) );
acado_setBlockH11( 1, 10, &(acadoWorkspace.E[ 968 ]), &(acadoWorkspace.QE[ 1040 ]) );

acado_zeroBlockH11( 1, 11 );
acado_setBlockH11( 1, 11, &(acadoWorkspace.E[ 536 ]), &(acadoWorkspace.QE[ 616 ]) );
acado_setBlockH11( 1, 11, &(acadoWorkspace.E[ 632 ]), &(acadoWorkspace.QE[ 712 ]) );
acado_setBlockH11( 1, 11, &(acadoWorkspace.E[ 736 ]), &(acadoWorkspace.QE[ 816 ]) );
acado_setBlockH11( 1, 11, &(acadoWorkspace.E[ 848 ]), &(acadoWorkspace.QE[ 928 ]) );
acado_setBlockH11( 1, 11, &(acadoWorkspace.E[ 968 ]), &(acadoWorkspace.QE[ 1048 ]) );

acado_zeroBlockH11( 1, 12 );
acado_setBlockH11( 1, 12, &(acadoWorkspace.E[ 632 ]), &(acadoWorkspace.QE[ 720 ]) );
acado_setBlockH11( 1, 12, &(acadoWorkspace.E[ 736 ]), &(acadoWorkspace.QE[ 824 ]) );
acado_setBlockH11( 1, 12, &(acadoWorkspace.E[ 848 ]), &(acadoWorkspace.QE[ 936 ]) );
acado_setBlockH11( 1, 12, &(acadoWorkspace.E[ 968 ]), &(acadoWorkspace.QE[ 1056 ]) );

acado_zeroBlockH11( 1, 13 );
acado_setBlockH11( 1, 13, &(acadoWorkspace.E[ 736 ]), &(acadoWorkspace.QE[ 832 ]) );
acado_setBlockH11( 1, 13, &(acadoWorkspace.E[ 848 ]), &(acadoWorkspace.QE[ 944 ]) );
acado_setBlockH11( 1, 13, &(acadoWorkspace.E[ 968 ]), &(acadoWorkspace.QE[ 1064 ]) );

acado_zeroBlockH11( 1, 14 );
acado_setBlockH11( 1, 14, &(acadoWorkspace.E[ 848 ]), &(acadoWorkspace.QE[ 952 ]) );
acado_setBlockH11( 1, 14, &(acadoWorkspace.E[ 968 ]), &(acadoWorkspace.QE[ 1072 ]) );

acado_zeroBlockH11( 1, 15 );
acado_setBlockH11( 1, 15, &(acadoWorkspace.E[ 968 ]), &(acadoWorkspace.QE[ 1080 ]) );

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

acado_zeroBlockH11( 2, 8 );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 304 ]), &(acadoWorkspace.QE[ 352 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 376 ]), &(acadoWorkspace.QE[ 424 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 456 ]), &(acadoWorkspace.QE[ 504 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 544 ]), &(acadoWorkspace.QE[ 592 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 640 ]), &(acadoWorkspace.QE[ 688 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 744 ]), &(acadoWorkspace.QE[ 792 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 856 ]), &(acadoWorkspace.QE[ 904 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 976 ]), &(acadoWorkspace.QE[ 1024 ]) );

acado_zeroBlockH11( 2, 9 );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 376 ]), &(acadoWorkspace.QE[ 432 ]) );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 456 ]), &(acadoWorkspace.QE[ 512 ]) );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 544 ]), &(acadoWorkspace.QE[ 600 ]) );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 640 ]), &(acadoWorkspace.QE[ 696 ]) );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 744 ]), &(acadoWorkspace.QE[ 800 ]) );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 856 ]), &(acadoWorkspace.QE[ 912 ]) );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 976 ]), &(acadoWorkspace.QE[ 1032 ]) );

acado_zeroBlockH11( 2, 10 );
acado_setBlockH11( 2, 10, &(acadoWorkspace.E[ 456 ]), &(acadoWorkspace.QE[ 520 ]) );
acado_setBlockH11( 2, 10, &(acadoWorkspace.E[ 544 ]), &(acadoWorkspace.QE[ 608 ]) );
acado_setBlockH11( 2, 10, &(acadoWorkspace.E[ 640 ]), &(acadoWorkspace.QE[ 704 ]) );
acado_setBlockH11( 2, 10, &(acadoWorkspace.E[ 744 ]), &(acadoWorkspace.QE[ 808 ]) );
acado_setBlockH11( 2, 10, &(acadoWorkspace.E[ 856 ]), &(acadoWorkspace.QE[ 920 ]) );
acado_setBlockH11( 2, 10, &(acadoWorkspace.E[ 976 ]), &(acadoWorkspace.QE[ 1040 ]) );

acado_zeroBlockH11( 2, 11 );
acado_setBlockH11( 2, 11, &(acadoWorkspace.E[ 544 ]), &(acadoWorkspace.QE[ 616 ]) );
acado_setBlockH11( 2, 11, &(acadoWorkspace.E[ 640 ]), &(acadoWorkspace.QE[ 712 ]) );
acado_setBlockH11( 2, 11, &(acadoWorkspace.E[ 744 ]), &(acadoWorkspace.QE[ 816 ]) );
acado_setBlockH11( 2, 11, &(acadoWorkspace.E[ 856 ]), &(acadoWorkspace.QE[ 928 ]) );
acado_setBlockH11( 2, 11, &(acadoWorkspace.E[ 976 ]), &(acadoWorkspace.QE[ 1048 ]) );

acado_zeroBlockH11( 2, 12 );
acado_setBlockH11( 2, 12, &(acadoWorkspace.E[ 640 ]), &(acadoWorkspace.QE[ 720 ]) );
acado_setBlockH11( 2, 12, &(acadoWorkspace.E[ 744 ]), &(acadoWorkspace.QE[ 824 ]) );
acado_setBlockH11( 2, 12, &(acadoWorkspace.E[ 856 ]), &(acadoWorkspace.QE[ 936 ]) );
acado_setBlockH11( 2, 12, &(acadoWorkspace.E[ 976 ]), &(acadoWorkspace.QE[ 1056 ]) );

acado_zeroBlockH11( 2, 13 );
acado_setBlockH11( 2, 13, &(acadoWorkspace.E[ 744 ]), &(acadoWorkspace.QE[ 832 ]) );
acado_setBlockH11( 2, 13, &(acadoWorkspace.E[ 856 ]), &(acadoWorkspace.QE[ 944 ]) );
acado_setBlockH11( 2, 13, &(acadoWorkspace.E[ 976 ]), &(acadoWorkspace.QE[ 1064 ]) );

acado_zeroBlockH11( 2, 14 );
acado_setBlockH11( 2, 14, &(acadoWorkspace.E[ 856 ]), &(acadoWorkspace.QE[ 952 ]) );
acado_setBlockH11( 2, 14, &(acadoWorkspace.E[ 976 ]), &(acadoWorkspace.QE[ 1072 ]) );

acado_zeroBlockH11( 2, 15 );
acado_setBlockH11( 2, 15, &(acadoWorkspace.E[ 976 ]), &(acadoWorkspace.QE[ 1080 ]) );

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

acado_zeroBlockH11( 3, 8 );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 312 ]), &(acadoWorkspace.QE[ 352 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.QE[ 424 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 464 ]), &(acadoWorkspace.QE[ 504 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.QE[ 592 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 648 ]), &(acadoWorkspace.QE[ 688 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 752 ]), &(acadoWorkspace.QE[ 792 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.QE[ 904 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 984 ]), &(acadoWorkspace.QE[ 1024 ]) );

acado_zeroBlockH11( 3, 9 );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.QE[ 432 ]) );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 464 ]), &(acadoWorkspace.QE[ 512 ]) );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.QE[ 600 ]) );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 648 ]), &(acadoWorkspace.QE[ 696 ]) );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 752 ]), &(acadoWorkspace.QE[ 800 ]) );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.QE[ 912 ]) );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 984 ]), &(acadoWorkspace.QE[ 1032 ]) );

acado_zeroBlockH11( 3, 10 );
acado_setBlockH11( 3, 10, &(acadoWorkspace.E[ 464 ]), &(acadoWorkspace.QE[ 520 ]) );
acado_setBlockH11( 3, 10, &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.QE[ 608 ]) );
acado_setBlockH11( 3, 10, &(acadoWorkspace.E[ 648 ]), &(acadoWorkspace.QE[ 704 ]) );
acado_setBlockH11( 3, 10, &(acadoWorkspace.E[ 752 ]), &(acadoWorkspace.QE[ 808 ]) );
acado_setBlockH11( 3, 10, &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.QE[ 920 ]) );
acado_setBlockH11( 3, 10, &(acadoWorkspace.E[ 984 ]), &(acadoWorkspace.QE[ 1040 ]) );

acado_zeroBlockH11( 3, 11 );
acado_setBlockH11( 3, 11, &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.QE[ 616 ]) );
acado_setBlockH11( 3, 11, &(acadoWorkspace.E[ 648 ]), &(acadoWorkspace.QE[ 712 ]) );
acado_setBlockH11( 3, 11, &(acadoWorkspace.E[ 752 ]), &(acadoWorkspace.QE[ 816 ]) );
acado_setBlockH11( 3, 11, &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.QE[ 928 ]) );
acado_setBlockH11( 3, 11, &(acadoWorkspace.E[ 984 ]), &(acadoWorkspace.QE[ 1048 ]) );

acado_zeroBlockH11( 3, 12 );
acado_setBlockH11( 3, 12, &(acadoWorkspace.E[ 648 ]), &(acadoWorkspace.QE[ 720 ]) );
acado_setBlockH11( 3, 12, &(acadoWorkspace.E[ 752 ]), &(acadoWorkspace.QE[ 824 ]) );
acado_setBlockH11( 3, 12, &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.QE[ 936 ]) );
acado_setBlockH11( 3, 12, &(acadoWorkspace.E[ 984 ]), &(acadoWorkspace.QE[ 1056 ]) );

acado_zeroBlockH11( 3, 13 );
acado_setBlockH11( 3, 13, &(acadoWorkspace.E[ 752 ]), &(acadoWorkspace.QE[ 832 ]) );
acado_setBlockH11( 3, 13, &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.QE[ 944 ]) );
acado_setBlockH11( 3, 13, &(acadoWorkspace.E[ 984 ]), &(acadoWorkspace.QE[ 1064 ]) );

acado_zeroBlockH11( 3, 14 );
acado_setBlockH11( 3, 14, &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.QE[ 952 ]) );
acado_setBlockH11( 3, 14, &(acadoWorkspace.E[ 984 ]), &(acadoWorkspace.QE[ 1072 ]) );

acado_zeroBlockH11( 3, 15 );
acado_setBlockH11( 3, 15, &(acadoWorkspace.E[ 984 ]), &(acadoWorkspace.QE[ 1080 ]) );

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

acado_zeroBlockH11( 4, 8 );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 320 ]), &(acadoWorkspace.QE[ 352 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 392 ]), &(acadoWorkspace.QE[ 424 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 472 ]), &(acadoWorkspace.QE[ 504 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 560 ]), &(acadoWorkspace.QE[ 592 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 656 ]), &(acadoWorkspace.QE[ 688 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 760 ]), &(acadoWorkspace.QE[ 792 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 872 ]), &(acadoWorkspace.QE[ 904 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 992 ]), &(acadoWorkspace.QE[ 1024 ]) );

acado_zeroBlockH11( 4, 9 );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 392 ]), &(acadoWorkspace.QE[ 432 ]) );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 472 ]), &(acadoWorkspace.QE[ 512 ]) );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 560 ]), &(acadoWorkspace.QE[ 600 ]) );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 656 ]), &(acadoWorkspace.QE[ 696 ]) );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 760 ]), &(acadoWorkspace.QE[ 800 ]) );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 872 ]), &(acadoWorkspace.QE[ 912 ]) );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 992 ]), &(acadoWorkspace.QE[ 1032 ]) );

acado_zeroBlockH11( 4, 10 );
acado_setBlockH11( 4, 10, &(acadoWorkspace.E[ 472 ]), &(acadoWorkspace.QE[ 520 ]) );
acado_setBlockH11( 4, 10, &(acadoWorkspace.E[ 560 ]), &(acadoWorkspace.QE[ 608 ]) );
acado_setBlockH11( 4, 10, &(acadoWorkspace.E[ 656 ]), &(acadoWorkspace.QE[ 704 ]) );
acado_setBlockH11( 4, 10, &(acadoWorkspace.E[ 760 ]), &(acadoWorkspace.QE[ 808 ]) );
acado_setBlockH11( 4, 10, &(acadoWorkspace.E[ 872 ]), &(acadoWorkspace.QE[ 920 ]) );
acado_setBlockH11( 4, 10, &(acadoWorkspace.E[ 992 ]), &(acadoWorkspace.QE[ 1040 ]) );

acado_zeroBlockH11( 4, 11 );
acado_setBlockH11( 4, 11, &(acadoWorkspace.E[ 560 ]), &(acadoWorkspace.QE[ 616 ]) );
acado_setBlockH11( 4, 11, &(acadoWorkspace.E[ 656 ]), &(acadoWorkspace.QE[ 712 ]) );
acado_setBlockH11( 4, 11, &(acadoWorkspace.E[ 760 ]), &(acadoWorkspace.QE[ 816 ]) );
acado_setBlockH11( 4, 11, &(acadoWorkspace.E[ 872 ]), &(acadoWorkspace.QE[ 928 ]) );
acado_setBlockH11( 4, 11, &(acadoWorkspace.E[ 992 ]), &(acadoWorkspace.QE[ 1048 ]) );

acado_zeroBlockH11( 4, 12 );
acado_setBlockH11( 4, 12, &(acadoWorkspace.E[ 656 ]), &(acadoWorkspace.QE[ 720 ]) );
acado_setBlockH11( 4, 12, &(acadoWorkspace.E[ 760 ]), &(acadoWorkspace.QE[ 824 ]) );
acado_setBlockH11( 4, 12, &(acadoWorkspace.E[ 872 ]), &(acadoWorkspace.QE[ 936 ]) );
acado_setBlockH11( 4, 12, &(acadoWorkspace.E[ 992 ]), &(acadoWorkspace.QE[ 1056 ]) );

acado_zeroBlockH11( 4, 13 );
acado_setBlockH11( 4, 13, &(acadoWorkspace.E[ 760 ]), &(acadoWorkspace.QE[ 832 ]) );
acado_setBlockH11( 4, 13, &(acadoWorkspace.E[ 872 ]), &(acadoWorkspace.QE[ 944 ]) );
acado_setBlockH11( 4, 13, &(acadoWorkspace.E[ 992 ]), &(acadoWorkspace.QE[ 1064 ]) );

acado_zeroBlockH11( 4, 14 );
acado_setBlockH11( 4, 14, &(acadoWorkspace.E[ 872 ]), &(acadoWorkspace.QE[ 952 ]) );
acado_setBlockH11( 4, 14, &(acadoWorkspace.E[ 992 ]), &(acadoWorkspace.QE[ 1072 ]) );

acado_zeroBlockH11( 4, 15 );
acado_setBlockH11( 4, 15, &(acadoWorkspace.E[ 992 ]), &(acadoWorkspace.QE[ 1080 ]) );

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

acado_zeroBlockH11( 5, 8 );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 328 ]), &(acadoWorkspace.QE[ 352 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 400 ]), &(acadoWorkspace.QE[ 424 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 504 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 568 ]), &(acadoWorkspace.QE[ 592 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 664 ]), &(acadoWorkspace.QE[ 688 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 768 ]), &(acadoWorkspace.QE[ 792 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 880 ]), &(acadoWorkspace.QE[ 904 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 1000 ]), &(acadoWorkspace.QE[ 1024 ]) );

acado_zeroBlockH11( 5, 9 );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 400 ]), &(acadoWorkspace.QE[ 432 ]) );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 512 ]) );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 568 ]), &(acadoWorkspace.QE[ 600 ]) );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 664 ]), &(acadoWorkspace.QE[ 696 ]) );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 768 ]), &(acadoWorkspace.QE[ 800 ]) );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 880 ]), &(acadoWorkspace.QE[ 912 ]) );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 1000 ]), &(acadoWorkspace.QE[ 1032 ]) );

acado_zeroBlockH11( 5, 10 );
acado_setBlockH11( 5, 10, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 520 ]) );
acado_setBlockH11( 5, 10, &(acadoWorkspace.E[ 568 ]), &(acadoWorkspace.QE[ 608 ]) );
acado_setBlockH11( 5, 10, &(acadoWorkspace.E[ 664 ]), &(acadoWorkspace.QE[ 704 ]) );
acado_setBlockH11( 5, 10, &(acadoWorkspace.E[ 768 ]), &(acadoWorkspace.QE[ 808 ]) );
acado_setBlockH11( 5, 10, &(acadoWorkspace.E[ 880 ]), &(acadoWorkspace.QE[ 920 ]) );
acado_setBlockH11( 5, 10, &(acadoWorkspace.E[ 1000 ]), &(acadoWorkspace.QE[ 1040 ]) );

acado_zeroBlockH11( 5, 11 );
acado_setBlockH11( 5, 11, &(acadoWorkspace.E[ 568 ]), &(acadoWorkspace.QE[ 616 ]) );
acado_setBlockH11( 5, 11, &(acadoWorkspace.E[ 664 ]), &(acadoWorkspace.QE[ 712 ]) );
acado_setBlockH11( 5, 11, &(acadoWorkspace.E[ 768 ]), &(acadoWorkspace.QE[ 816 ]) );
acado_setBlockH11( 5, 11, &(acadoWorkspace.E[ 880 ]), &(acadoWorkspace.QE[ 928 ]) );
acado_setBlockH11( 5, 11, &(acadoWorkspace.E[ 1000 ]), &(acadoWorkspace.QE[ 1048 ]) );

acado_zeroBlockH11( 5, 12 );
acado_setBlockH11( 5, 12, &(acadoWorkspace.E[ 664 ]), &(acadoWorkspace.QE[ 720 ]) );
acado_setBlockH11( 5, 12, &(acadoWorkspace.E[ 768 ]), &(acadoWorkspace.QE[ 824 ]) );
acado_setBlockH11( 5, 12, &(acadoWorkspace.E[ 880 ]), &(acadoWorkspace.QE[ 936 ]) );
acado_setBlockH11( 5, 12, &(acadoWorkspace.E[ 1000 ]), &(acadoWorkspace.QE[ 1056 ]) );

acado_zeroBlockH11( 5, 13 );
acado_setBlockH11( 5, 13, &(acadoWorkspace.E[ 768 ]), &(acadoWorkspace.QE[ 832 ]) );
acado_setBlockH11( 5, 13, &(acadoWorkspace.E[ 880 ]), &(acadoWorkspace.QE[ 944 ]) );
acado_setBlockH11( 5, 13, &(acadoWorkspace.E[ 1000 ]), &(acadoWorkspace.QE[ 1064 ]) );

acado_zeroBlockH11( 5, 14 );
acado_setBlockH11( 5, 14, &(acadoWorkspace.E[ 880 ]), &(acadoWorkspace.QE[ 952 ]) );
acado_setBlockH11( 5, 14, &(acadoWorkspace.E[ 1000 ]), &(acadoWorkspace.QE[ 1072 ]) );

acado_zeroBlockH11( 5, 15 );
acado_setBlockH11( 5, 15, &(acadoWorkspace.E[ 1000 ]), &(acadoWorkspace.QE[ 1080 ]) );

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

acado_zeroBlockH11( 6, 8 );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QE[ 352 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 408 ]), &(acadoWorkspace.QE[ 424 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 488 ]), &(acadoWorkspace.QE[ 504 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 592 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.QE[ 688 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 776 ]), &(acadoWorkspace.QE[ 792 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 888 ]), &(acadoWorkspace.QE[ 904 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 1008 ]), &(acadoWorkspace.QE[ 1024 ]) );

acado_zeroBlockH11( 6, 9 );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 408 ]), &(acadoWorkspace.QE[ 432 ]) );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 488 ]), &(acadoWorkspace.QE[ 512 ]) );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 600 ]) );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.QE[ 696 ]) );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 776 ]), &(acadoWorkspace.QE[ 800 ]) );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 888 ]), &(acadoWorkspace.QE[ 912 ]) );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 1008 ]), &(acadoWorkspace.QE[ 1032 ]) );

acado_zeroBlockH11( 6, 10 );
acado_setBlockH11( 6, 10, &(acadoWorkspace.E[ 488 ]), &(acadoWorkspace.QE[ 520 ]) );
acado_setBlockH11( 6, 10, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 608 ]) );
acado_setBlockH11( 6, 10, &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.QE[ 704 ]) );
acado_setBlockH11( 6, 10, &(acadoWorkspace.E[ 776 ]), &(acadoWorkspace.QE[ 808 ]) );
acado_setBlockH11( 6, 10, &(acadoWorkspace.E[ 888 ]), &(acadoWorkspace.QE[ 920 ]) );
acado_setBlockH11( 6, 10, &(acadoWorkspace.E[ 1008 ]), &(acadoWorkspace.QE[ 1040 ]) );

acado_zeroBlockH11( 6, 11 );
acado_setBlockH11( 6, 11, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 616 ]) );
acado_setBlockH11( 6, 11, &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.QE[ 712 ]) );
acado_setBlockH11( 6, 11, &(acadoWorkspace.E[ 776 ]), &(acadoWorkspace.QE[ 816 ]) );
acado_setBlockH11( 6, 11, &(acadoWorkspace.E[ 888 ]), &(acadoWorkspace.QE[ 928 ]) );
acado_setBlockH11( 6, 11, &(acadoWorkspace.E[ 1008 ]), &(acadoWorkspace.QE[ 1048 ]) );

acado_zeroBlockH11( 6, 12 );
acado_setBlockH11( 6, 12, &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.QE[ 720 ]) );
acado_setBlockH11( 6, 12, &(acadoWorkspace.E[ 776 ]), &(acadoWorkspace.QE[ 824 ]) );
acado_setBlockH11( 6, 12, &(acadoWorkspace.E[ 888 ]), &(acadoWorkspace.QE[ 936 ]) );
acado_setBlockH11( 6, 12, &(acadoWorkspace.E[ 1008 ]), &(acadoWorkspace.QE[ 1056 ]) );

acado_zeroBlockH11( 6, 13 );
acado_setBlockH11( 6, 13, &(acadoWorkspace.E[ 776 ]), &(acadoWorkspace.QE[ 832 ]) );
acado_setBlockH11( 6, 13, &(acadoWorkspace.E[ 888 ]), &(acadoWorkspace.QE[ 944 ]) );
acado_setBlockH11( 6, 13, &(acadoWorkspace.E[ 1008 ]), &(acadoWorkspace.QE[ 1064 ]) );

acado_zeroBlockH11( 6, 14 );
acado_setBlockH11( 6, 14, &(acadoWorkspace.E[ 888 ]), &(acadoWorkspace.QE[ 952 ]) );
acado_setBlockH11( 6, 14, &(acadoWorkspace.E[ 1008 ]), &(acadoWorkspace.QE[ 1072 ]) );

acado_zeroBlockH11( 6, 15 );
acado_setBlockH11( 6, 15, &(acadoWorkspace.E[ 1008 ]), &(acadoWorkspace.QE[ 1080 ]) );

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

acado_zeroBlockH11( 7, 8 );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 344 ]), &(acadoWorkspace.QE[ 352 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 416 ]), &(acadoWorkspace.QE[ 424 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 496 ]), &(acadoWorkspace.QE[ 504 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 584 ]), &(acadoWorkspace.QE[ 592 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 680 ]), &(acadoWorkspace.QE[ 688 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 784 ]), &(acadoWorkspace.QE[ 792 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 896 ]), &(acadoWorkspace.QE[ 904 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 1016 ]), &(acadoWorkspace.QE[ 1024 ]) );

acado_zeroBlockH11( 7, 9 );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 416 ]), &(acadoWorkspace.QE[ 432 ]) );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 496 ]), &(acadoWorkspace.QE[ 512 ]) );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 584 ]), &(acadoWorkspace.QE[ 600 ]) );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 680 ]), &(acadoWorkspace.QE[ 696 ]) );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 784 ]), &(acadoWorkspace.QE[ 800 ]) );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 896 ]), &(acadoWorkspace.QE[ 912 ]) );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 1016 ]), &(acadoWorkspace.QE[ 1032 ]) );

acado_zeroBlockH11( 7, 10 );
acado_setBlockH11( 7, 10, &(acadoWorkspace.E[ 496 ]), &(acadoWorkspace.QE[ 520 ]) );
acado_setBlockH11( 7, 10, &(acadoWorkspace.E[ 584 ]), &(acadoWorkspace.QE[ 608 ]) );
acado_setBlockH11( 7, 10, &(acadoWorkspace.E[ 680 ]), &(acadoWorkspace.QE[ 704 ]) );
acado_setBlockH11( 7, 10, &(acadoWorkspace.E[ 784 ]), &(acadoWorkspace.QE[ 808 ]) );
acado_setBlockH11( 7, 10, &(acadoWorkspace.E[ 896 ]), &(acadoWorkspace.QE[ 920 ]) );
acado_setBlockH11( 7, 10, &(acadoWorkspace.E[ 1016 ]), &(acadoWorkspace.QE[ 1040 ]) );

acado_zeroBlockH11( 7, 11 );
acado_setBlockH11( 7, 11, &(acadoWorkspace.E[ 584 ]), &(acadoWorkspace.QE[ 616 ]) );
acado_setBlockH11( 7, 11, &(acadoWorkspace.E[ 680 ]), &(acadoWorkspace.QE[ 712 ]) );
acado_setBlockH11( 7, 11, &(acadoWorkspace.E[ 784 ]), &(acadoWorkspace.QE[ 816 ]) );
acado_setBlockH11( 7, 11, &(acadoWorkspace.E[ 896 ]), &(acadoWorkspace.QE[ 928 ]) );
acado_setBlockH11( 7, 11, &(acadoWorkspace.E[ 1016 ]), &(acadoWorkspace.QE[ 1048 ]) );

acado_zeroBlockH11( 7, 12 );
acado_setBlockH11( 7, 12, &(acadoWorkspace.E[ 680 ]), &(acadoWorkspace.QE[ 720 ]) );
acado_setBlockH11( 7, 12, &(acadoWorkspace.E[ 784 ]), &(acadoWorkspace.QE[ 824 ]) );
acado_setBlockH11( 7, 12, &(acadoWorkspace.E[ 896 ]), &(acadoWorkspace.QE[ 936 ]) );
acado_setBlockH11( 7, 12, &(acadoWorkspace.E[ 1016 ]), &(acadoWorkspace.QE[ 1056 ]) );

acado_zeroBlockH11( 7, 13 );
acado_setBlockH11( 7, 13, &(acadoWorkspace.E[ 784 ]), &(acadoWorkspace.QE[ 832 ]) );
acado_setBlockH11( 7, 13, &(acadoWorkspace.E[ 896 ]), &(acadoWorkspace.QE[ 944 ]) );
acado_setBlockH11( 7, 13, &(acadoWorkspace.E[ 1016 ]), &(acadoWorkspace.QE[ 1064 ]) );

acado_zeroBlockH11( 7, 14 );
acado_setBlockH11( 7, 14, &(acadoWorkspace.E[ 896 ]), &(acadoWorkspace.QE[ 952 ]) );
acado_setBlockH11( 7, 14, &(acadoWorkspace.E[ 1016 ]), &(acadoWorkspace.QE[ 1072 ]) );

acado_zeroBlockH11( 7, 15 );
acado_setBlockH11( 7, 15, &(acadoWorkspace.E[ 1016 ]), &(acadoWorkspace.QE[ 1080 ]) );

acado_setBlockH11_R1( 8, 8, &(acadoWorkspace.R1[ 32 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 352 ]), &(acadoWorkspace.QE[ 352 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 424 ]), &(acadoWorkspace.QE[ 424 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 504 ]), &(acadoWorkspace.QE[ 504 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 592 ]), &(acadoWorkspace.QE[ 592 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 688 ]), &(acadoWorkspace.QE[ 688 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 792 ]), &(acadoWorkspace.QE[ 792 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 904 ]), &(acadoWorkspace.QE[ 904 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 1024 ]), &(acadoWorkspace.QE[ 1024 ]) );

acado_zeroBlockH11( 8, 9 );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 424 ]), &(acadoWorkspace.QE[ 432 ]) );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 504 ]), &(acadoWorkspace.QE[ 512 ]) );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 592 ]), &(acadoWorkspace.QE[ 600 ]) );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 688 ]), &(acadoWorkspace.QE[ 696 ]) );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 792 ]), &(acadoWorkspace.QE[ 800 ]) );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 904 ]), &(acadoWorkspace.QE[ 912 ]) );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 1024 ]), &(acadoWorkspace.QE[ 1032 ]) );

acado_zeroBlockH11( 8, 10 );
acado_setBlockH11( 8, 10, &(acadoWorkspace.E[ 504 ]), &(acadoWorkspace.QE[ 520 ]) );
acado_setBlockH11( 8, 10, &(acadoWorkspace.E[ 592 ]), &(acadoWorkspace.QE[ 608 ]) );
acado_setBlockH11( 8, 10, &(acadoWorkspace.E[ 688 ]), &(acadoWorkspace.QE[ 704 ]) );
acado_setBlockH11( 8, 10, &(acadoWorkspace.E[ 792 ]), &(acadoWorkspace.QE[ 808 ]) );
acado_setBlockH11( 8, 10, &(acadoWorkspace.E[ 904 ]), &(acadoWorkspace.QE[ 920 ]) );
acado_setBlockH11( 8, 10, &(acadoWorkspace.E[ 1024 ]), &(acadoWorkspace.QE[ 1040 ]) );

acado_zeroBlockH11( 8, 11 );
acado_setBlockH11( 8, 11, &(acadoWorkspace.E[ 592 ]), &(acadoWorkspace.QE[ 616 ]) );
acado_setBlockH11( 8, 11, &(acadoWorkspace.E[ 688 ]), &(acadoWorkspace.QE[ 712 ]) );
acado_setBlockH11( 8, 11, &(acadoWorkspace.E[ 792 ]), &(acadoWorkspace.QE[ 816 ]) );
acado_setBlockH11( 8, 11, &(acadoWorkspace.E[ 904 ]), &(acadoWorkspace.QE[ 928 ]) );
acado_setBlockH11( 8, 11, &(acadoWorkspace.E[ 1024 ]), &(acadoWorkspace.QE[ 1048 ]) );

acado_zeroBlockH11( 8, 12 );
acado_setBlockH11( 8, 12, &(acadoWorkspace.E[ 688 ]), &(acadoWorkspace.QE[ 720 ]) );
acado_setBlockH11( 8, 12, &(acadoWorkspace.E[ 792 ]), &(acadoWorkspace.QE[ 824 ]) );
acado_setBlockH11( 8, 12, &(acadoWorkspace.E[ 904 ]), &(acadoWorkspace.QE[ 936 ]) );
acado_setBlockH11( 8, 12, &(acadoWorkspace.E[ 1024 ]), &(acadoWorkspace.QE[ 1056 ]) );

acado_zeroBlockH11( 8, 13 );
acado_setBlockH11( 8, 13, &(acadoWorkspace.E[ 792 ]), &(acadoWorkspace.QE[ 832 ]) );
acado_setBlockH11( 8, 13, &(acadoWorkspace.E[ 904 ]), &(acadoWorkspace.QE[ 944 ]) );
acado_setBlockH11( 8, 13, &(acadoWorkspace.E[ 1024 ]), &(acadoWorkspace.QE[ 1064 ]) );

acado_zeroBlockH11( 8, 14 );
acado_setBlockH11( 8, 14, &(acadoWorkspace.E[ 904 ]), &(acadoWorkspace.QE[ 952 ]) );
acado_setBlockH11( 8, 14, &(acadoWorkspace.E[ 1024 ]), &(acadoWorkspace.QE[ 1072 ]) );

acado_zeroBlockH11( 8, 15 );
acado_setBlockH11( 8, 15, &(acadoWorkspace.E[ 1024 ]), &(acadoWorkspace.QE[ 1080 ]) );

acado_setBlockH11_R1( 9, 9, &(acadoWorkspace.R1[ 36 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QE[ 432 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 512 ]), &(acadoWorkspace.QE[ 512 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 600 ]), &(acadoWorkspace.QE[ 600 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 696 ]), &(acadoWorkspace.QE[ 696 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 800 ]), &(acadoWorkspace.QE[ 800 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 912 ]), &(acadoWorkspace.QE[ 912 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 1032 ]), &(acadoWorkspace.QE[ 1032 ]) );

acado_zeroBlockH11( 9, 10 );
acado_setBlockH11( 9, 10, &(acadoWorkspace.E[ 512 ]), &(acadoWorkspace.QE[ 520 ]) );
acado_setBlockH11( 9, 10, &(acadoWorkspace.E[ 600 ]), &(acadoWorkspace.QE[ 608 ]) );
acado_setBlockH11( 9, 10, &(acadoWorkspace.E[ 696 ]), &(acadoWorkspace.QE[ 704 ]) );
acado_setBlockH11( 9, 10, &(acadoWorkspace.E[ 800 ]), &(acadoWorkspace.QE[ 808 ]) );
acado_setBlockH11( 9, 10, &(acadoWorkspace.E[ 912 ]), &(acadoWorkspace.QE[ 920 ]) );
acado_setBlockH11( 9, 10, &(acadoWorkspace.E[ 1032 ]), &(acadoWorkspace.QE[ 1040 ]) );

acado_zeroBlockH11( 9, 11 );
acado_setBlockH11( 9, 11, &(acadoWorkspace.E[ 600 ]), &(acadoWorkspace.QE[ 616 ]) );
acado_setBlockH11( 9, 11, &(acadoWorkspace.E[ 696 ]), &(acadoWorkspace.QE[ 712 ]) );
acado_setBlockH11( 9, 11, &(acadoWorkspace.E[ 800 ]), &(acadoWorkspace.QE[ 816 ]) );
acado_setBlockH11( 9, 11, &(acadoWorkspace.E[ 912 ]), &(acadoWorkspace.QE[ 928 ]) );
acado_setBlockH11( 9, 11, &(acadoWorkspace.E[ 1032 ]), &(acadoWorkspace.QE[ 1048 ]) );

acado_zeroBlockH11( 9, 12 );
acado_setBlockH11( 9, 12, &(acadoWorkspace.E[ 696 ]), &(acadoWorkspace.QE[ 720 ]) );
acado_setBlockH11( 9, 12, &(acadoWorkspace.E[ 800 ]), &(acadoWorkspace.QE[ 824 ]) );
acado_setBlockH11( 9, 12, &(acadoWorkspace.E[ 912 ]), &(acadoWorkspace.QE[ 936 ]) );
acado_setBlockH11( 9, 12, &(acadoWorkspace.E[ 1032 ]), &(acadoWorkspace.QE[ 1056 ]) );

acado_zeroBlockH11( 9, 13 );
acado_setBlockH11( 9, 13, &(acadoWorkspace.E[ 800 ]), &(acadoWorkspace.QE[ 832 ]) );
acado_setBlockH11( 9, 13, &(acadoWorkspace.E[ 912 ]), &(acadoWorkspace.QE[ 944 ]) );
acado_setBlockH11( 9, 13, &(acadoWorkspace.E[ 1032 ]), &(acadoWorkspace.QE[ 1064 ]) );

acado_zeroBlockH11( 9, 14 );
acado_setBlockH11( 9, 14, &(acadoWorkspace.E[ 912 ]), &(acadoWorkspace.QE[ 952 ]) );
acado_setBlockH11( 9, 14, &(acadoWorkspace.E[ 1032 ]), &(acadoWorkspace.QE[ 1072 ]) );

acado_zeroBlockH11( 9, 15 );
acado_setBlockH11( 9, 15, &(acadoWorkspace.E[ 1032 ]), &(acadoWorkspace.QE[ 1080 ]) );

acado_setBlockH11_R1( 10, 10, &(acadoWorkspace.R1[ 40 ]) );
acado_setBlockH11( 10, 10, &(acadoWorkspace.E[ 520 ]), &(acadoWorkspace.QE[ 520 ]) );
acado_setBlockH11( 10, 10, &(acadoWorkspace.E[ 608 ]), &(acadoWorkspace.QE[ 608 ]) );
acado_setBlockH11( 10, 10, &(acadoWorkspace.E[ 704 ]), &(acadoWorkspace.QE[ 704 ]) );
acado_setBlockH11( 10, 10, &(acadoWorkspace.E[ 808 ]), &(acadoWorkspace.QE[ 808 ]) );
acado_setBlockH11( 10, 10, &(acadoWorkspace.E[ 920 ]), &(acadoWorkspace.QE[ 920 ]) );
acado_setBlockH11( 10, 10, &(acadoWorkspace.E[ 1040 ]), &(acadoWorkspace.QE[ 1040 ]) );

acado_zeroBlockH11( 10, 11 );
acado_setBlockH11( 10, 11, &(acadoWorkspace.E[ 608 ]), &(acadoWorkspace.QE[ 616 ]) );
acado_setBlockH11( 10, 11, &(acadoWorkspace.E[ 704 ]), &(acadoWorkspace.QE[ 712 ]) );
acado_setBlockH11( 10, 11, &(acadoWorkspace.E[ 808 ]), &(acadoWorkspace.QE[ 816 ]) );
acado_setBlockH11( 10, 11, &(acadoWorkspace.E[ 920 ]), &(acadoWorkspace.QE[ 928 ]) );
acado_setBlockH11( 10, 11, &(acadoWorkspace.E[ 1040 ]), &(acadoWorkspace.QE[ 1048 ]) );

acado_zeroBlockH11( 10, 12 );
acado_setBlockH11( 10, 12, &(acadoWorkspace.E[ 704 ]), &(acadoWorkspace.QE[ 720 ]) );
acado_setBlockH11( 10, 12, &(acadoWorkspace.E[ 808 ]), &(acadoWorkspace.QE[ 824 ]) );
acado_setBlockH11( 10, 12, &(acadoWorkspace.E[ 920 ]), &(acadoWorkspace.QE[ 936 ]) );
acado_setBlockH11( 10, 12, &(acadoWorkspace.E[ 1040 ]), &(acadoWorkspace.QE[ 1056 ]) );

acado_zeroBlockH11( 10, 13 );
acado_setBlockH11( 10, 13, &(acadoWorkspace.E[ 808 ]), &(acadoWorkspace.QE[ 832 ]) );
acado_setBlockH11( 10, 13, &(acadoWorkspace.E[ 920 ]), &(acadoWorkspace.QE[ 944 ]) );
acado_setBlockH11( 10, 13, &(acadoWorkspace.E[ 1040 ]), &(acadoWorkspace.QE[ 1064 ]) );

acado_zeroBlockH11( 10, 14 );
acado_setBlockH11( 10, 14, &(acadoWorkspace.E[ 920 ]), &(acadoWorkspace.QE[ 952 ]) );
acado_setBlockH11( 10, 14, &(acadoWorkspace.E[ 1040 ]), &(acadoWorkspace.QE[ 1072 ]) );

acado_zeroBlockH11( 10, 15 );
acado_setBlockH11( 10, 15, &(acadoWorkspace.E[ 1040 ]), &(acadoWorkspace.QE[ 1080 ]) );

acado_setBlockH11_R1( 11, 11, &(acadoWorkspace.R1[ 44 ]) );
acado_setBlockH11( 11, 11, &(acadoWorkspace.E[ 616 ]), &(acadoWorkspace.QE[ 616 ]) );
acado_setBlockH11( 11, 11, &(acadoWorkspace.E[ 712 ]), &(acadoWorkspace.QE[ 712 ]) );
acado_setBlockH11( 11, 11, &(acadoWorkspace.E[ 816 ]), &(acadoWorkspace.QE[ 816 ]) );
acado_setBlockH11( 11, 11, &(acadoWorkspace.E[ 928 ]), &(acadoWorkspace.QE[ 928 ]) );
acado_setBlockH11( 11, 11, &(acadoWorkspace.E[ 1048 ]), &(acadoWorkspace.QE[ 1048 ]) );

acado_zeroBlockH11( 11, 12 );
acado_setBlockH11( 11, 12, &(acadoWorkspace.E[ 712 ]), &(acadoWorkspace.QE[ 720 ]) );
acado_setBlockH11( 11, 12, &(acadoWorkspace.E[ 816 ]), &(acadoWorkspace.QE[ 824 ]) );
acado_setBlockH11( 11, 12, &(acadoWorkspace.E[ 928 ]), &(acadoWorkspace.QE[ 936 ]) );
acado_setBlockH11( 11, 12, &(acadoWorkspace.E[ 1048 ]), &(acadoWorkspace.QE[ 1056 ]) );

acado_zeroBlockH11( 11, 13 );
acado_setBlockH11( 11, 13, &(acadoWorkspace.E[ 816 ]), &(acadoWorkspace.QE[ 832 ]) );
acado_setBlockH11( 11, 13, &(acadoWorkspace.E[ 928 ]), &(acadoWorkspace.QE[ 944 ]) );
acado_setBlockH11( 11, 13, &(acadoWorkspace.E[ 1048 ]), &(acadoWorkspace.QE[ 1064 ]) );

acado_zeroBlockH11( 11, 14 );
acado_setBlockH11( 11, 14, &(acadoWorkspace.E[ 928 ]), &(acadoWorkspace.QE[ 952 ]) );
acado_setBlockH11( 11, 14, &(acadoWorkspace.E[ 1048 ]), &(acadoWorkspace.QE[ 1072 ]) );

acado_zeroBlockH11( 11, 15 );
acado_setBlockH11( 11, 15, &(acadoWorkspace.E[ 1048 ]), &(acadoWorkspace.QE[ 1080 ]) );

acado_setBlockH11_R1( 12, 12, &(acadoWorkspace.R1[ 48 ]) );
acado_setBlockH11( 12, 12, &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.QE[ 720 ]) );
acado_setBlockH11( 12, 12, &(acadoWorkspace.E[ 824 ]), &(acadoWorkspace.QE[ 824 ]) );
acado_setBlockH11( 12, 12, &(acadoWorkspace.E[ 936 ]), &(acadoWorkspace.QE[ 936 ]) );
acado_setBlockH11( 12, 12, &(acadoWorkspace.E[ 1056 ]), &(acadoWorkspace.QE[ 1056 ]) );

acado_zeroBlockH11( 12, 13 );
acado_setBlockH11( 12, 13, &(acadoWorkspace.E[ 824 ]), &(acadoWorkspace.QE[ 832 ]) );
acado_setBlockH11( 12, 13, &(acadoWorkspace.E[ 936 ]), &(acadoWorkspace.QE[ 944 ]) );
acado_setBlockH11( 12, 13, &(acadoWorkspace.E[ 1056 ]), &(acadoWorkspace.QE[ 1064 ]) );

acado_zeroBlockH11( 12, 14 );
acado_setBlockH11( 12, 14, &(acadoWorkspace.E[ 936 ]), &(acadoWorkspace.QE[ 952 ]) );
acado_setBlockH11( 12, 14, &(acadoWorkspace.E[ 1056 ]), &(acadoWorkspace.QE[ 1072 ]) );

acado_zeroBlockH11( 12, 15 );
acado_setBlockH11( 12, 15, &(acadoWorkspace.E[ 1056 ]), &(acadoWorkspace.QE[ 1080 ]) );

acado_setBlockH11_R1( 13, 13, &(acadoWorkspace.R1[ 52 ]) );
acado_setBlockH11( 13, 13, &(acadoWorkspace.E[ 832 ]), &(acadoWorkspace.QE[ 832 ]) );
acado_setBlockH11( 13, 13, &(acadoWorkspace.E[ 944 ]), &(acadoWorkspace.QE[ 944 ]) );
acado_setBlockH11( 13, 13, &(acadoWorkspace.E[ 1064 ]), &(acadoWorkspace.QE[ 1064 ]) );

acado_zeroBlockH11( 13, 14 );
acado_setBlockH11( 13, 14, &(acadoWorkspace.E[ 944 ]), &(acadoWorkspace.QE[ 952 ]) );
acado_setBlockH11( 13, 14, &(acadoWorkspace.E[ 1064 ]), &(acadoWorkspace.QE[ 1072 ]) );

acado_zeroBlockH11( 13, 15 );
acado_setBlockH11( 13, 15, &(acadoWorkspace.E[ 1064 ]), &(acadoWorkspace.QE[ 1080 ]) );

acado_setBlockH11_R1( 14, 14, &(acadoWorkspace.R1[ 56 ]) );
acado_setBlockH11( 14, 14, &(acadoWorkspace.E[ 952 ]), &(acadoWorkspace.QE[ 952 ]) );
acado_setBlockH11( 14, 14, &(acadoWorkspace.E[ 1072 ]), &(acadoWorkspace.QE[ 1072 ]) );

acado_zeroBlockH11( 14, 15 );
acado_setBlockH11( 14, 15, &(acadoWorkspace.E[ 1072 ]), &(acadoWorkspace.QE[ 1080 ]) );

acado_setBlockH11_R1( 15, 15, &(acadoWorkspace.R1[ 60 ]) );
acado_setBlockH11( 15, 15, &(acadoWorkspace.E[ 1080 ]), &(acadoWorkspace.QE[ 1080 ]) );


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
acado_multQN1d( acadoWorkspace.QN1, &(acadoWorkspace.d[ 60 ]), &(acadoWorkspace.Qd[ 60 ]) );

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
acado_macETSlu( &(acadoWorkspace.QE[ 280 ]), &(acadoWorkspace.g[ 14 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 344 ]), &(acadoWorkspace.g[ 14 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 416 ]), &(acadoWorkspace.g[ 14 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 496 ]), &(acadoWorkspace.g[ 14 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 584 ]), &(acadoWorkspace.g[ 14 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 680 ]), &(acadoWorkspace.g[ 14 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 784 ]), &(acadoWorkspace.g[ 14 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 896 ]), &(acadoWorkspace.g[ 14 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1016 ]), &(acadoWorkspace.g[ 14 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 352 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 424 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 504 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 592 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 688 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 792 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 904 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1024 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 432 ]), &(acadoWorkspace.g[ 18 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 512 ]), &(acadoWorkspace.g[ 18 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 600 ]), &(acadoWorkspace.g[ 18 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 696 ]), &(acadoWorkspace.g[ 18 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 800 ]), &(acadoWorkspace.g[ 18 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 912 ]), &(acadoWorkspace.g[ 18 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1032 ]), &(acadoWorkspace.g[ 18 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 520 ]), &(acadoWorkspace.g[ 20 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 608 ]), &(acadoWorkspace.g[ 20 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 704 ]), &(acadoWorkspace.g[ 20 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 808 ]), &(acadoWorkspace.g[ 20 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 920 ]), &(acadoWorkspace.g[ 20 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1040 ]), &(acadoWorkspace.g[ 20 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 616 ]), &(acadoWorkspace.g[ 22 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 712 ]), &(acadoWorkspace.g[ 22 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 816 ]), &(acadoWorkspace.g[ 22 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 928 ]), &(acadoWorkspace.g[ 22 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1048 ]), &(acadoWorkspace.g[ 22 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 720 ]), &(acadoWorkspace.g[ 24 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 824 ]), &(acadoWorkspace.g[ 24 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 936 ]), &(acadoWorkspace.g[ 24 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1056 ]), &(acadoWorkspace.g[ 24 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 832 ]), &(acadoWorkspace.g[ 26 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 944 ]), &(acadoWorkspace.g[ 26 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1064 ]), &(acadoWorkspace.g[ 26 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 952 ]), &(acadoWorkspace.g[ 28 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1072 ]), &(acadoWorkspace.g[ 28 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1080 ]), &(acadoWorkspace.g[ 30 ]) );
acadoWorkspace.lb[0] = (real_t)-1.5000000000000000e+00 - acadoVariables.u[0];
acadoWorkspace.lb[1] = (real_t)-4.0000000000000002e-01 - acadoVariables.u[1];
acadoWorkspace.lb[2] = (real_t)-1.5000000000000000e+00 - acadoVariables.u[2];
acadoWorkspace.lb[3] = (real_t)-4.0000000000000002e-01 - acadoVariables.u[3];
acadoWorkspace.lb[4] = (real_t)-1.5000000000000000e+00 - acadoVariables.u[4];
acadoWorkspace.lb[5] = (real_t)-4.0000000000000002e-01 - acadoVariables.u[5];
acadoWorkspace.lb[6] = (real_t)-1.5000000000000000e+00 - acadoVariables.u[6];
acadoWorkspace.lb[7] = (real_t)-4.0000000000000002e-01 - acadoVariables.u[7];
acadoWorkspace.lb[8] = (real_t)-1.5000000000000000e+00 - acadoVariables.u[8];
acadoWorkspace.lb[9] = (real_t)-4.0000000000000002e-01 - acadoVariables.u[9];
acadoWorkspace.lb[10] = (real_t)-1.5000000000000000e+00 - acadoVariables.u[10];
acadoWorkspace.lb[11] = (real_t)-4.0000000000000002e-01 - acadoVariables.u[11];
acadoWorkspace.lb[12] = (real_t)-1.5000000000000000e+00 - acadoVariables.u[12];
acadoWorkspace.lb[13] = (real_t)-4.0000000000000002e-01 - acadoVariables.u[13];
acadoWorkspace.lb[14] = (real_t)-1.5000000000000000e+00 - acadoVariables.u[14];
acadoWorkspace.lb[15] = (real_t)-4.0000000000000002e-01 - acadoVariables.u[15];
acadoWorkspace.lb[16] = (real_t)-1.5000000000000000e+00 - acadoVariables.u[16];
acadoWorkspace.lb[17] = (real_t)-4.0000000000000002e-01 - acadoVariables.u[17];
acadoWorkspace.lb[18] = (real_t)-1.5000000000000000e+00 - acadoVariables.u[18];
acadoWorkspace.lb[19] = (real_t)-4.0000000000000002e-01 - acadoVariables.u[19];
acadoWorkspace.lb[20] = (real_t)-1.5000000000000000e+00 - acadoVariables.u[20];
acadoWorkspace.lb[21] = (real_t)-4.0000000000000002e-01 - acadoVariables.u[21];
acadoWorkspace.lb[22] = (real_t)-1.5000000000000000e+00 - acadoVariables.u[22];
acadoWorkspace.lb[23] = (real_t)-4.0000000000000002e-01 - acadoVariables.u[23];
acadoWorkspace.lb[24] = (real_t)-1.5000000000000000e+00 - acadoVariables.u[24];
acadoWorkspace.lb[25] = (real_t)-4.0000000000000002e-01 - acadoVariables.u[25];
acadoWorkspace.lb[26] = (real_t)-1.5000000000000000e+00 - acadoVariables.u[26];
acadoWorkspace.lb[27] = (real_t)-4.0000000000000002e-01 - acadoVariables.u[27];
acadoWorkspace.lb[28] = (real_t)-1.5000000000000000e+00 - acadoVariables.u[28];
acadoWorkspace.lb[29] = (real_t)-4.0000000000000002e-01 - acadoVariables.u[29];
acadoWorkspace.lb[30] = (real_t)-1.5000000000000000e+00 - acadoVariables.u[30];
acadoWorkspace.lb[31] = (real_t)-4.0000000000000002e-01 - acadoVariables.u[31];
acadoWorkspace.ub[0] = (real_t)1.0000000000000000e+00 - acadoVariables.u[0];
acadoWorkspace.ub[1] = (real_t)4.0000000000000002e-01 - acadoVariables.u[1];
acadoWorkspace.ub[2] = (real_t)1.0000000000000000e+00 - acadoVariables.u[2];
acadoWorkspace.ub[3] = (real_t)4.0000000000000002e-01 - acadoVariables.u[3];
acadoWorkspace.ub[4] = (real_t)1.0000000000000000e+00 - acadoVariables.u[4];
acadoWorkspace.ub[5] = (real_t)4.0000000000000002e-01 - acadoVariables.u[5];
acadoWorkspace.ub[6] = (real_t)1.0000000000000000e+00 - acadoVariables.u[6];
acadoWorkspace.ub[7] = (real_t)4.0000000000000002e-01 - acadoVariables.u[7];
acadoWorkspace.ub[8] = (real_t)1.0000000000000000e+00 - acadoVariables.u[8];
acadoWorkspace.ub[9] = (real_t)4.0000000000000002e-01 - acadoVariables.u[9];
acadoWorkspace.ub[10] = (real_t)1.0000000000000000e+00 - acadoVariables.u[10];
acadoWorkspace.ub[11] = (real_t)4.0000000000000002e-01 - acadoVariables.u[11];
acadoWorkspace.ub[12] = (real_t)1.0000000000000000e+00 - acadoVariables.u[12];
acadoWorkspace.ub[13] = (real_t)4.0000000000000002e-01 - acadoVariables.u[13];
acadoWorkspace.ub[14] = (real_t)1.0000000000000000e+00 - acadoVariables.u[14];
acadoWorkspace.ub[15] = (real_t)4.0000000000000002e-01 - acadoVariables.u[15];
acadoWorkspace.ub[16] = (real_t)1.0000000000000000e+00 - acadoVariables.u[16];
acadoWorkspace.ub[17] = (real_t)4.0000000000000002e-01 - acadoVariables.u[17];
acadoWorkspace.ub[18] = (real_t)1.0000000000000000e+00 - acadoVariables.u[18];
acadoWorkspace.ub[19] = (real_t)4.0000000000000002e-01 - acadoVariables.u[19];
acadoWorkspace.ub[20] = (real_t)1.0000000000000000e+00 - acadoVariables.u[20];
acadoWorkspace.ub[21] = (real_t)4.0000000000000002e-01 - acadoVariables.u[21];
acadoWorkspace.ub[22] = (real_t)1.0000000000000000e+00 - acadoVariables.u[22];
acadoWorkspace.ub[23] = (real_t)4.0000000000000002e-01 - acadoVariables.u[23];
acadoWorkspace.ub[24] = (real_t)1.0000000000000000e+00 - acadoVariables.u[24];
acadoWorkspace.ub[25] = (real_t)4.0000000000000002e-01 - acadoVariables.u[25];
acadoWorkspace.ub[26] = (real_t)1.0000000000000000e+00 - acadoVariables.u[26];
acadoWorkspace.ub[27] = (real_t)4.0000000000000002e-01 - acadoVariables.u[27];
acadoWorkspace.ub[28] = (real_t)1.0000000000000000e+00 - acadoVariables.u[28];
acadoWorkspace.ub[29] = (real_t)4.0000000000000002e-01 - acadoVariables.u[29];
acadoWorkspace.ub[30] = (real_t)1.0000000000000000e+00 - acadoVariables.u[30];
acadoWorkspace.ub[31] = (real_t)4.0000000000000002e-01 - acadoVariables.u[31];

for (lRun1 = 0; lRun1 < 16; ++lRun1)
{
acadoWorkspace.conValueIn[0] = acadoVariables.x[lRun1 * 4];
acadoWorkspace.conValueIn[1] = acadoVariables.x[lRun1 * 4 + 1];
acadoWorkspace.conValueIn[2] = acadoVariables.x[lRun1 * 4 + 2];
acadoWorkspace.conValueIn[3] = acadoVariables.x[lRun1 * 4 + 3];
acadoWorkspace.conValueIn[4] = acadoVariables.u[lRun1 * 2];
acadoWorkspace.conValueIn[5] = acadoVariables.u[lRun1 * 2 + 1];
acadoWorkspace.conValueIn[6] = acadoVariables.od[lRun1 * 32];
acadoWorkspace.conValueIn[7] = acadoVariables.od[lRun1 * 32 + 1];
acadoWorkspace.conValueIn[8] = acadoVariables.od[lRun1 * 32 + 2];
acadoWorkspace.conValueIn[9] = acadoVariables.od[lRun1 * 32 + 3];
acadoWorkspace.conValueIn[10] = acadoVariables.od[lRun1 * 32 + 4];
acadoWorkspace.conValueIn[11] = acadoVariables.od[lRun1 * 32 + 5];
acadoWorkspace.conValueIn[12] = acadoVariables.od[lRun1 * 32 + 6];
acadoWorkspace.conValueIn[13] = acadoVariables.od[lRun1 * 32 + 7];
acadoWorkspace.conValueIn[14] = acadoVariables.od[lRun1 * 32 + 8];
acadoWorkspace.conValueIn[15] = acadoVariables.od[lRun1 * 32 + 9];
acadoWorkspace.conValueIn[16] = acadoVariables.od[lRun1 * 32 + 10];
acadoWorkspace.conValueIn[17] = acadoVariables.od[lRun1 * 32 + 11];
acadoWorkspace.conValueIn[18] = acadoVariables.od[lRun1 * 32 + 12];
acadoWorkspace.conValueIn[19] = acadoVariables.od[lRun1 * 32 + 13];
acadoWorkspace.conValueIn[20] = acadoVariables.od[lRun1 * 32 + 14];
acadoWorkspace.conValueIn[21] = acadoVariables.od[lRun1 * 32 + 15];
acadoWorkspace.conValueIn[22] = acadoVariables.od[lRun1 * 32 + 16];
acadoWorkspace.conValueIn[23] = acadoVariables.od[lRun1 * 32 + 17];
acadoWorkspace.conValueIn[24] = acadoVariables.od[lRun1 * 32 + 18];
acadoWorkspace.conValueIn[25] = acadoVariables.od[lRun1 * 32 + 19];
acadoWorkspace.conValueIn[26] = acadoVariables.od[lRun1 * 32 + 20];
acadoWorkspace.conValueIn[27] = acadoVariables.od[lRun1 * 32 + 21];
acadoWorkspace.conValueIn[28] = acadoVariables.od[lRun1 * 32 + 22];
acadoWorkspace.conValueIn[29] = acadoVariables.od[lRun1 * 32 + 23];
acadoWorkspace.conValueIn[30] = acadoVariables.od[lRun1 * 32 + 24];
acadoWorkspace.conValueIn[31] = acadoVariables.od[lRun1 * 32 + 25];
acadoWorkspace.conValueIn[32] = acadoVariables.od[lRun1 * 32 + 26];
acadoWorkspace.conValueIn[33] = acadoVariables.od[lRun1 * 32 + 27];
acadoWorkspace.conValueIn[34] = acadoVariables.od[lRun1 * 32 + 28];
acadoWorkspace.conValueIn[35] = acadoVariables.od[lRun1 * 32 + 29];
acadoWorkspace.conValueIn[36] = acadoVariables.od[lRun1 * 32 + 30];
acadoWorkspace.conValueIn[37] = acadoVariables.od[lRun1 * 32 + 31];
acado_evaluatePathConstraints( acadoWorkspace.conValueIn, acadoWorkspace.conValueOut );
acadoWorkspace.evH[lRun1 * 8] = acadoWorkspace.conValueOut[0];
acadoWorkspace.evH[lRun1 * 8 + 1] = acadoWorkspace.conValueOut[1];
acadoWorkspace.evH[lRun1 * 8 + 2] = acadoWorkspace.conValueOut[2];
acadoWorkspace.evH[lRun1 * 8 + 3] = acadoWorkspace.conValueOut[3];
acadoWorkspace.evH[lRun1 * 8 + 4] = acadoWorkspace.conValueOut[4];
acadoWorkspace.evH[lRun1 * 8 + 5] = acadoWorkspace.conValueOut[5];
acadoWorkspace.evH[lRun1 * 8 + 6] = acadoWorkspace.conValueOut[6];
acadoWorkspace.evH[lRun1 * 8 + 7] = acadoWorkspace.conValueOut[7];

acadoWorkspace.evHx[lRun1 * 32] = acadoWorkspace.conValueOut[8];
acadoWorkspace.evHx[lRun1 * 32 + 1] = acadoWorkspace.conValueOut[9];
acadoWorkspace.evHx[lRun1 * 32 + 2] = acadoWorkspace.conValueOut[10];
acadoWorkspace.evHx[lRun1 * 32 + 3] = acadoWorkspace.conValueOut[11];
acadoWorkspace.evHx[lRun1 * 32 + 4] = acadoWorkspace.conValueOut[12];
acadoWorkspace.evHx[lRun1 * 32 + 5] = acadoWorkspace.conValueOut[13];
acadoWorkspace.evHx[lRun1 * 32 + 6] = acadoWorkspace.conValueOut[14];
acadoWorkspace.evHx[lRun1 * 32 + 7] = acadoWorkspace.conValueOut[15];
acadoWorkspace.evHx[lRun1 * 32 + 8] = acadoWorkspace.conValueOut[16];
acadoWorkspace.evHx[lRun1 * 32 + 9] = acadoWorkspace.conValueOut[17];
acadoWorkspace.evHx[lRun1 * 32 + 10] = acadoWorkspace.conValueOut[18];
acadoWorkspace.evHx[lRun1 * 32 + 11] = acadoWorkspace.conValueOut[19];
acadoWorkspace.evHx[lRun1 * 32 + 12] = acadoWorkspace.conValueOut[20];
acadoWorkspace.evHx[lRun1 * 32 + 13] = acadoWorkspace.conValueOut[21];
acadoWorkspace.evHx[lRun1 * 32 + 14] = acadoWorkspace.conValueOut[22];
acadoWorkspace.evHx[lRun1 * 32 + 15] = acadoWorkspace.conValueOut[23];
acadoWorkspace.evHx[lRun1 * 32 + 16] = acadoWorkspace.conValueOut[24];
acadoWorkspace.evHx[lRun1 * 32 + 17] = acadoWorkspace.conValueOut[25];
acadoWorkspace.evHx[lRun1 * 32 + 18] = acadoWorkspace.conValueOut[26];
acadoWorkspace.evHx[lRun1 * 32 + 19] = acadoWorkspace.conValueOut[27];
acadoWorkspace.evHx[lRun1 * 32 + 20] = acadoWorkspace.conValueOut[28];
acadoWorkspace.evHx[lRun1 * 32 + 21] = acadoWorkspace.conValueOut[29];
acadoWorkspace.evHx[lRun1 * 32 + 22] = acadoWorkspace.conValueOut[30];
acadoWorkspace.evHx[lRun1 * 32 + 23] = acadoWorkspace.conValueOut[31];
acadoWorkspace.evHx[lRun1 * 32 + 24] = acadoWorkspace.conValueOut[32];
acadoWorkspace.evHx[lRun1 * 32 + 25] = acadoWorkspace.conValueOut[33];
acadoWorkspace.evHx[lRun1 * 32 + 26] = acadoWorkspace.conValueOut[34];
acadoWorkspace.evHx[lRun1 * 32 + 27] = acadoWorkspace.conValueOut[35];
acadoWorkspace.evHx[lRun1 * 32 + 28] = acadoWorkspace.conValueOut[36];
acadoWorkspace.evHx[lRun1 * 32 + 29] = acadoWorkspace.conValueOut[37];
acadoWorkspace.evHx[lRun1 * 32 + 30] = acadoWorkspace.conValueOut[38];
acadoWorkspace.evHx[lRun1 * 32 + 31] = acadoWorkspace.conValueOut[39];
acadoWorkspace.evHu[lRun1 * 16] = acadoWorkspace.conValueOut[40];
acadoWorkspace.evHu[lRun1 * 16 + 1] = acadoWorkspace.conValueOut[41];
acadoWorkspace.evHu[lRun1 * 16 + 2] = acadoWorkspace.conValueOut[42];
acadoWorkspace.evHu[lRun1 * 16 + 3] = acadoWorkspace.conValueOut[43];
acadoWorkspace.evHu[lRun1 * 16 + 4] = acadoWorkspace.conValueOut[44];
acadoWorkspace.evHu[lRun1 * 16 + 5] = acadoWorkspace.conValueOut[45];
acadoWorkspace.evHu[lRun1 * 16 + 6] = acadoWorkspace.conValueOut[46];
acadoWorkspace.evHu[lRun1 * 16 + 7] = acadoWorkspace.conValueOut[47];
acadoWorkspace.evHu[lRun1 * 16 + 8] = acadoWorkspace.conValueOut[48];
acadoWorkspace.evHu[lRun1 * 16 + 9] = acadoWorkspace.conValueOut[49];
acadoWorkspace.evHu[lRun1 * 16 + 10] = acadoWorkspace.conValueOut[50];
acadoWorkspace.evHu[lRun1 * 16 + 11] = acadoWorkspace.conValueOut[51];
acadoWorkspace.evHu[lRun1 * 16 + 12] = acadoWorkspace.conValueOut[52];
acadoWorkspace.evHu[lRun1 * 16 + 13] = acadoWorkspace.conValueOut[53];
acadoWorkspace.evHu[lRun1 * 16 + 14] = acadoWorkspace.conValueOut[54];
acadoWorkspace.evHu[lRun1 * 16 + 15] = acadoWorkspace.conValueOut[55];
}

acadoWorkspace.A01[0] = acadoWorkspace.evHx[0];
acadoWorkspace.A01[1] = acadoWorkspace.evHx[1];
acadoWorkspace.A01[2] = acadoWorkspace.evHx[2];
acadoWorkspace.A01[3] = acadoWorkspace.evHx[3];
acadoWorkspace.A01[4] = acadoWorkspace.evHx[4];
acadoWorkspace.A01[5] = acadoWorkspace.evHx[5];
acadoWorkspace.A01[6] = acadoWorkspace.evHx[6];
acadoWorkspace.A01[7] = acadoWorkspace.evHx[7];
acadoWorkspace.A01[8] = acadoWorkspace.evHx[8];
acadoWorkspace.A01[9] = acadoWorkspace.evHx[9];
acadoWorkspace.A01[10] = acadoWorkspace.evHx[10];
acadoWorkspace.A01[11] = acadoWorkspace.evHx[11];
acadoWorkspace.A01[12] = acadoWorkspace.evHx[12];
acadoWorkspace.A01[13] = acadoWorkspace.evHx[13];
acadoWorkspace.A01[14] = acadoWorkspace.evHx[14];
acadoWorkspace.A01[15] = acadoWorkspace.evHx[15];
acadoWorkspace.A01[16] = acadoWorkspace.evHx[16];
acadoWorkspace.A01[17] = acadoWorkspace.evHx[17];
acadoWorkspace.A01[18] = acadoWorkspace.evHx[18];
acadoWorkspace.A01[19] = acadoWorkspace.evHx[19];
acadoWorkspace.A01[20] = acadoWorkspace.evHx[20];
acadoWorkspace.A01[21] = acadoWorkspace.evHx[21];
acadoWorkspace.A01[22] = acadoWorkspace.evHx[22];
acadoWorkspace.A01[23] = acadoWorkspace.evHx[23];
acadoWorkspace.A01[24] = acadoWorkspace.evHx[24];
acadoWorkspace.A01[25] = acadoWorkspace.evHx[25];
acadoWorkspace.A01[26] = acadoWorkspace.evHx[26];
acadoWorkspace.A01[27] = acadoWorkspace.evHx[27];
acadoWorkspace.A01[28] = acadoWorkspace.evHx[28];
acadoWorkspace.A01[29] = acadoWorkspace.evHx[29];
acadoWorkspace.A01[30] = acadoWorkspace.evHx[30];
acadoWorkspace.A01[31] = acadoWorkspace.evHx[31];

acado_multHxC( &(acadoWorkspace.evHx[ 32 ]), acadoWorkspace.evGx, &(acadoWorkspace.A01[ 32 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 64 ]), &(acadoWorkspace.evGx[ 16 ]), &(acadoWorkspace.A01[ 64 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 96 ]), &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.A01[ 96 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 128 ]), &(acadoWorkspace.evGx[ 48 ]), &(acadoWorkspace.A01[ 128 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 160 ]), &(acadoWorkspace.evGx[ 64 ]), &(acadoWorkspace.A01[ 160 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 192 ]), &(acadoWorkspace.evGx[ 80 ]), &(acadoWorkspace.A01[ 192 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 224 ]), &(acadoWorkspace.evGx[ 96 ]), &(acadoWorkspace.A01[ 224 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 256 ]), &(acadoWorkspace.evGx[ 112 ]), &(acadoWorkspace.A01[ 256 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 288 ]), &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.A01[ 288 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 320 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.A01[ 320 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 352 ]), &(acadoWorkspace.evGx[ 160 ]), &(acadoWorkspace.A01[ 352 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 384 ]), &(acadoWorkspace.evGx[ 176 ]), &(acadoWorkspace.A01[ 384 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 416 ]), &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.A01[ 416 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 448 ]), &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.A01[ 448 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 480 ]), &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.A01[ 480 ]) );

acado_multHxE( &(acadoWorkspace.evHx[ 32 ]), acadoWorkspace.E, 1, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 64 ]), &(acadoWorkspace.E[ 8 ]), 2, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 64 ]), &(acadoWorkspace.E[ 16 ]), 2, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 96 ]), &(acadoWorkspace.E[ 24 ]), 3, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 96 ]), &(acadoWorkspace.E[ 32 ]), 3, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 96 ]), &(acadoWorkspace.E[ 40 ]), 3, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 128 ]), &(acadoWorkspace.E[ 48 ]), 4, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 128 ]), &(acadoWorkspace.E[ 56 ]), 4, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 128 ]), &(acadoWorkspace.E[ 64 ]), 4, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 128 ]), &(acadoWorkspace.E[ 72 ]), 4, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 160 ]), &(acadoWorkspace.E[ 80 ]), 5, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 160 ]), &(acadoWorkspace.E[ 88 ]), 5, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 160 ]), &(acadoWorkspace.E[ 96 ]), 5, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 160 ]), &(acadoWorkspace.E[ 104 ]), 5, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 160 ]), &(acadoWorkspace.E[ 112 ]), 5, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 192 ]), &(acadoWorkspace.E[ 120 ]), 6, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 192 ]), &(acadoWorkspace.E[ 128 ]), 6, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 192 ]), &(acadoWorkspace.E[ 136 ]), 6, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 192 ]), &(acadoWorkspace.E[ 144 ]), 6, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 192 ]), &(acadoWorkspace.E[ 152 ]), 6, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 192 ]), &(acadoWorkspace.E[ 160 ]), 6, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 224 ]), &(acadoWorkspace.E[ 168 ]), 7, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 224 ]), &(acadoWorkspace.E[ 176 ]), 7, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 224 ]), &(acadoWorkspace.E[ 184 ]), 7, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 224 ]), &(acadoWorkspace.E[ 192 ]), 7, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 224 ]), &(acadoWorkspace.E[ 200 ]), 7, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 224 ]), &(acadoWorkspace.E[ 208 ]), 7, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 224 ]), &(acadoWorkspace.E[ 216 ]), 7, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 256 ]), &(acadoWorkspace.E[ 224 ]), 8, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 256 ]), &(acadoWorkspace.E[ 232 ]), 8, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 256 ]), &(acadoWorkspace.E[ 240 ]), 8, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 256 ]), &(acadoWorkspace.E[ 248 ]), 8, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 256 ]), &(acadoWorkspace.E[ 256 ]), 8, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 256 ]), &(acadoWorkspace.E[ 264 ]), 8, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 256 ]), &(acadoWorkspace.E[ 272 ]), 8, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 256 ]), &(acadoWorkspace.E[ 280 ]), 8, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 288 ]), &(acadoWorkspace.E[ 288 ]), 9, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 288 ]), &(acadoWorkspace.E[ 296 ]), 9, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 288 ]), &(acadoWorkspace.E[ 304 ]), 9, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 288 ]), &(acadoWorkspace.E[ 312 ]), 9, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 288 ]), &(acadoWorkspace.E[ 320 ]), 9, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 288 ]), &(acadoWorkspace.E[ 328 ]), 9, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 288 ]), &(acadoWorkspace.E[ 336 ]), 9, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 288 ]), &(acadoWorkspace.E[ 344 ]), 9, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 288 ]), &(acadoWorkspace.E[ 352 ]), 9, 8 );
acado_multHxE( &(acadoWorkspace.evHx[ 320 ]), &(acadoWorkspace.E[ 360 ]), 10, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 320 ]), &(acadoWorkspace.E[ 368 ]), 10, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 320 ]), &(acadoWorkspace.E[ 376 ]), 10, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 320 ]), &(acadoWorkspace.E[ 384 ]), 10, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 320 ]), &(acadoWorkspace.E[ 392 ]), 10, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 320 ]), &(acadoWorkspace.E[ 400 ]), 10, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 320 ]), &(acadoWorkspace.E[ 408 ]), 10, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 320 ]), &(acadoWorkspace.E[ 416 ]), 10, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 320 ]), &(acadoWorkspace.E[ 424 ]), 10, 8 );
acado_multHxE( &(acadoWorkspace.evHx[ 320 ]), &(acadoWorkspace.E[ 432 ]), 10, 9 );
acado_multHxE( &(acadoWorkspace.evHx[ 352 ]), &(acadoWorkspace.E[ 440 ]), 11, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 352 ]), &(acadoWorkspace.E[ 448 ]), 11, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 352 ]), &(acadoWorkspace.E[ 456 ]), 11, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 352 ]), &(acadoWorkspace.E[ 464 ]), 11, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 352 ]), &(acadoWorkspace.E[ 472 ]), 11, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 352 ]), &(acadoWorkspace.E[ 480 ]), 11, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 352 ]), &(acadoWorkspace.E[ 488 ]), 11, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 352 ]), &(acadoWorkspace.E[ 496 ]), 11, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 352 ]), &(acadoWorkspace.E[ 504 ]), 11, 8 );
acado_multHxE( &(acadoWorkspace.evHx[ 352 ]), &(acadoWorkspace.E[ 512 ]), 11, 9 );
acado_multHxE( &(acadoWorkspace.evHx[ 352 ]), &(acadoWorkspace.E[ 520 ]), 11, 10 );
acado_multHxE( &(acadoWorkspace.evHx[ 384 ]), &(acadoWorkspace.E[ 528 ]), 12, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 384 ]), &(acadoWorkspace.E[ 536 ]), 12, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 384 ]), &(acadoWorkspace.E[ 544 ]), 12, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 384 ]), &(acadoWorkspace.E[ 552 ]), 12, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 384 ]), &(acadoWorkspace.E[ 560 ]), 12, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 384 ]), &(acadoWorkspace.E[ 568 ]), 12, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 384 ]), &(acadoWorkspace.E[ 576 ]), 12, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 384 ]), &(acadoWorkspace.E[ 584 ]), 12, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 384 ]), &(acadoWorkspace.E[ 592 ]), 12, 8 );
acado_multHxE( &(acadoWorkspace.evHx[ 384 ]), &(acadoWorkspace.E[ 600 ]), 12, 9 );
acado_multHxE( &(acadoWorkspace.evHx[ 384 ]), &(acadoWorkspace.E[ 608 ]), 12, 10 );
acado_multHxE( &(acadoWorkspace.evHx[ 384 ]), &(acadoWorkspace.E[ 616 ]), 12, 11 );
acado_multHxE( &(acadoWorkspace.evHx[ 416 ]), &(acadoWorkspace.E[ 624 ]), 13, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 416 ]), &(acadoWorkspace.E[ 632 ]), 13, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 416 ]), &(acadoWorkspace.E[ 640 ]), 13, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 416 ]), &(acadoWorkspace.E[ 648 ]), 13, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 416 ]), &(acadoWorkspace.E[ 656 ]), 13, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 416 ]), &(acadoWorkspace.E[ 664 ]), 13, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 416 ]), &(acadoWorkspace.E[ 672 ]), 13, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 416 ]), &(acadoWorkspace.E[ 680 ]), 13, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 416 ]), &(acadoWorkspace.E[ 688 ]), 13, 8 );
acado_multHxE( &(acadoWorkspace.evHx[ 416 ]), &(acadoWorkspace.E[ 696 ]), 13, 9 );
acado_multHxE( &(acadoWorkspace.evHx[ 416 ]), &(acadoWorkspace.E[ 704 ]), 13, 10 );
acado_multHxE( &(acadoWorkspace.evHx[ 416 ]), &(acadoWorkspace.E[ 712 ]), 13, 11 );
acado_multHxE( &(acadoWorkspace.evHx[ 416 ]), &(acadoWorkspace.E[ 720 ]), 13, 12 );
acado_multHxE( &(acadoWorkspace.evHx[ 448 ]), &(acadoWorkspace.E[ 728 ]), 14, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 448 ]), &(acadoWorkspace.E[ 736 ]), 14, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 448 ]), &(acadoWorkspace.E[ 744 ]), 14, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 448 ]), &(acadoWorkspace.E[ 752 ]), 14, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 448 ]), &(acadoWorkspace.E[ 760 ]), 14, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 448 ]), &(acadoWorkspace.E[ 768 ]), 14, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 448 ]), &(acadoWorkspace.E[ 776 ]), 14, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 448 ]), &(acadoWorkspace.E[ 784 ]), 14, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 448 ]), &(acadoWorkspace.E[ 792 ]), 14, 8 );
acado_multHxE( &(acadoWorkspace.evHx[ 448 ]), &(acadoWorkspace.E[ 800 ]), 14, 9 );
acado_multHxE( &(acadoWorkspace.evHx[ 448 ]), &(acadoWorkspace.E[ 808 ]), 14, 10 );
acado_multHxE( &(acadoWorkspace.evHx[ 448 ]), &(acadoWorkspace.E[ 816 ]), 14, 11 );
acado_multHxE( &(acadoWorkspace.evHx[ 448 ]), &(acadoWorkspace.E[ 824 ]), 14, 12 );
acado_multHxE( &(acadoWorkspace.evHx[ 448 ]), &(acadoWorkspace.E[ 832 ]), 14, 13 );
acado_multHxE( &(acadoWorkspace.evHx[ 480 ]), &(acadoWorkspace.E[ 840 ]), 15, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 480 ]), &(acadoWorkspace.E[ 848 ]), 15, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 480 ]), &(acadoWorkspace.E[ 856 ]), 15, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 480 ]), &(acadoWorkspace.E[ 864 ]), 15, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 480 ]), &(acadoWorkspace.E[ 872 ]), 15, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 480 ]), &(acadoWorkspace.E[ 880 ]), 15, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 480 ]), &(acadoWorkspace.E[ 888 ]), 15, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 480 ]), &(acadoWorkspace.E[ 896 ]), 15, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 480 ]), &(acadoWorkspace.E[ 904 ]), 15, 8 );
acado_multHxE( &(acadoWorkspace.evHx[ 480 ]), &(acadoWorkspace.E[ 912 ]), 15, 9 );
acado_multHxE( &(acadoWorkspace.evHx[ 480 ]), &(acadoWorkspace.E[ 920 ]), 15, 10 );
acado_multHxE( &(acadoWorkspace.evHx[ 480 ]), &(acadoWorkspace.E[ 928 ]), 15, 11 );
acado_multHxE( &(acadoWorkspace.evHx[ 480 ]), &(acadoWorkspace.E[ 936 ]), 15, 12 );
acado_multHxE( &(acadoWorkspace.evHx[ 480 ]), &(acadoWorkspace.E[ 944 ]), 15, 13 );
acado_multHxE( &(acadoWorkspace.evHx[ 480 ]), &(acadoWorkspace.E[ 952 ]), 15, 14 );

acadoWorkspace.A[0] = acadoWorkspace.evHu[0];
acadoWorkspace.A[1] = acadoWorkspace.evHu[1];
acadoWorkspace.A[32] = acadoWorkspace.evHu[2];
acadoWorkspace.A[33] = acadoWorkspace.evHu[3];
acadoWorkspace.A[64] = acadoWorkspace.evHu[4];
acadoWorkspace.A[65] = acadoWorkspace.evHu[5];
acadoWorkspace.A[96] = acadoWorkspace.evHu[6];
acadoWorkspace.A[97] = acadoWorkspace.evHu[7];
acadoWorkspace.A[128] = acadoWorkspace.evHu[8];
acadoWorkspace.A[129] = acadoWorkspace.evHu[9];
acadoWorkspace.A[160] = acadoWorkspace.evHu[10];
acadoWorkspace.A[161] = acadoWorkspace.evHu[11];
acadoWorkspace.A[192] = acadoWorkspace.evHu[12];
acadoWorkspace.A[193] = acadoWorkspace.evHu[13];
acadoWorkspace.A[224] = acadoWorkspace.evHu[14];
acadoWorkspace.A[225] = acadoWorkspace.evHu[15];
acadoWorkspace.A[258] = acadoWorkspace.evHu[16];
acadoWorkspace.A[259] = acadoWorkspace.evHu[17];
acadoWorkspace.A[290] = acadoWorkspace.evHu[18];
acadoWorkspace.A[291] = acadoWorkspace.evHu[19];
acadoWorkspace.A[322] = acadoWorkspace.evHu[20];
acadoWorkspace.A[323] = acadoWorkspace.evHu[21];
acadoWorkspace.A[354] = acadoWorkspace.evHu[22];
acadoWorkspace.A[355] = acadoWorkspace.evHu[23];
acadoWorkspace.A[386] = acadoWorkspace.evHu[24];
acadoWorkspace.A[387] = acadoWorkspace.evHu[25];
acadoWorkspace.A[418] = acadoWorkspace.evHu[26];
acadoWorkspace.A[419] = acadoWorkspace.evHu[27];
acadoWorkspace.A[450] = acadoWorkspace.evHu[28];
acadoWorkspace.A[451] = acadoWorkspace.evHu[29];
acadoWorkspace.A[482] = acadoWorkspace.evHu[30];
acadoWorkspace.A[483] = acadoWorkspace.evHu[31];
acadoWorkspace.A[516] = acadoWorkspace.evHu[32];
acadoWorkspace.A[517] = acadoWorkspace.evHu[33];
acadoWorkspace.A[548] = acadoWorkspace.evHu[34];
acadoWorkspace.A[549] = acadoWorkspace.evHu[35];
acadoWorkspace.A[580] = acadoWorkspace.evHu[36];
acadoWorkspace.A[581] = acadoWorkspace.evHu[37];
acadoWorkspace.A[612] = acadoWorkspace.evHu[38];
acadoWorkspace.A[613] = acadoWorkspace.evHu[39];
acadoWorkspace.A[644] = acadoWorkspace.evHu[40];
acadoWorkspace.A[645] = acadoWorkspace.evHu[41];
acadoWorkspace.A[676] = acadoWorkspace.evHu[42];
acadoWorkspace.A[677] = acadoWorkspace.evHu[43];
acadoWorkspace.A[708] = acadoWorkspace.evHu[44];
acadoWorkspace.A[709] = acadoWorkspace.evHu[45];
acadoWorkspace.A[740] = acadoWorkspace.evHu[46];
acadoWorkspace.A[741] = acadoWorkspace.evHu[47];
acadoWorkspace.A[774] = acadoWorkspace.evHu[48];
acadoWorkspace.A[775] = acadoWorkspace.evHu[49];
acadoWorkspace.A[806] = acadoWorkspace.evHu[50];
acadoWorkspace.A[807] = acadoWorkspace.evHu[51];
acadoWorkspace.A[838] = acadoWorkspace.evHu[52];
acadoWorkspace.A[839] = acadoWorkspace.evHu[53];
acadoWorkspace.A[870] = acadoWorkspace.evHu[54];
acadoWorkspace.A[871] = acadoWorkspace.evHu[55];
acadoWorkspace.A[902] = acadoWorkspace.evHu[56];
acadoWorkspace.A[903] = acadoWorkspace.evHu[57];
acadoWorkspace.A[934] = acadoWorkspace.evHu[58];
acadoWorkspace.A[935] = acadoWorkspace.evHu[59];
acadoWorkspace.A[966] = acadoWorkspace.evHu[60];
acadoWorkspace.A[967] = acadoWorkspace.evHu[61];
acadoWorkspace.A[998] = acadoWorkspace.evHu[62];
acadoWorkspace.A[999] = acadoWorkspace.evHu[63];
acadoWorkspace.A[1032] = acadoWorkspace.evHu[64];
acadoWorkspace.A[1033] = acadoWorkspace.evHu[65];
acadoWorkspace.A[1064] = acadoWorkspace.evHu[66];
acadoWorkspace.A[1065] = acadoWorkspace.evHu[67];
acadoWorkspace.A[1096] = acadoWorkspace.evHu[68];
acadoWorkspace.A[1097] = acadoWorkspace.evHu[69];
acadoWorkspace.A[1128] = acadoWorkspace.evHu[70];
acadoWorkspace.A[1129] = acadoWorkspace.evHu[71];
acadoWorkspace.A[1160] = acadoWorkspace.evHu[72];
acadoWorkspace.A[1161] = acadoWorkspace.evHu[73];
acadoWorkspace.A[1192] = acadoWorkspace.evHu[74];
acadoWorkspace.A[1193] = acadoWorkspace.evHu[75];
acadoWorkspace.A[1224] = acadoWorkspace.evHu[76];
acadoWorkspace.A[1225] = acadoWorkspace.evHu[77];
acadoWorkspace.A[1256] = acadoWorkspace.evHu[78];
acadoWorkspace.A[1257] = acadoWorkspace.evHu[79];
acadoWorkspace.A[1290] = acadoWorkspace.evHu[80];
acadoWorkspace.A[1291] = acadoWorkspace.evHu[81];
acadoWorkspace.A[1322] = acadoWorkspace.evHu[82];
acadoWorkspace.A[1323] = acadoWorkspace.evHu[83];
acadoWorkspace.A[1354] = acadoWorkspace.evHu[84];
acadoWorkspace.A[1355] = acadoWorkspace.evHu[85];
acadoWorkspace.A[1386] = acadoWorkspace.evHu[86];
acadoWorkspace.A[1387] = acadoWorkspace.evHu[87];
acadoWorkspace.A[1418] = acadoWorkspace.evHu[88];
acadoWorkspace.A[1419] = acadoWorkspace.evHu[89];
acadoWorkspace.A[1450] = acadoWorkspace.evHu[90];
acadoWorkspace.A[1451] = acadoWorkspace.evHu[91];
acadoWorkspace.A[1482] = acadoWorkspace.evHu[92];
acadoWorkspace.A[1483] = acadoWorkspace.evHu[93];
acadoWorkspace.A[1514] = acadoWorkspace.evHu[94];
acadoWorkspace.A[1515] = acadoWorkspace.evHu[95];
acadoWorkspace.A[1548] = acadoWorkspace.evHu[96];
acadoWorkspace.A[1549] = acadoWorkspace.evHu[97];
acadoWorkspace.A[1580] = acadoWorkspace.evHu[98];
acadoWorkspace.A[1581] = acadoWorkspace.evHu[99];
acadoWorkspace.A[1612] = acadoWorkspace.evHu[100];
acadoWorkspace.A[1613] = acadoWorkspace.evHu[101];
acadoWorkspace.A[1644] = acadoWorkspace.evHu[102];
acadoWorkspace.A[1645] = acadoWorkspace.evHu[103];
acadoWorkspace.A[1676] = acadoWorkspace.evHu[104];
acadoWorkspace.A[1677] = acadoWorkspace.evHu[105];
acadoWorkspace.A[1708] = acadoWorkspace.evHu[106];
acadoWorkspace.A[1709] = acadoWorkspace.evHu[107];
acadoWorkspace.A[1740] = acadoWorkspace.evHu[108];
acadoWorkspace.A[1741] = acadoWorkspace.evHu[109];
acadoWorkspace.A[1772] = acadoWorkspace.evHu[110];
acadoWorkspace.A[1773] = acadoWorkspace.evHu[111];
acadoWorkspace.A[1806] = acadoWorkspace.evHu[112];
acadoWorkspace.A[1807] = acadoWorkspace.evHu[113];
acadoWorkspace.A[1838] = acadoWorkspace.evHu[114];
acadoWorkspace.A[1839] = acadoWorkspace.evHu[115];
acadoWorkspace.A[1870] = acadoWorkspace.evHu[116];
acadoWorkspace.A[1871] = acadoWorkspace.evHu[117];
acadoWorkspace.A[1902] = acadoWorkspace.evHu[118];
acadoWorkspace.A[1903] = acadoWorkspace.evHu[119];
acadoWorkspace.A[1934] = acadoWorkspace.evHu[120];
acadoWorkspace.A[1935] = acadoWorkspace.evHu[121];
acadoWorkspace.A[1966] = acadoWorkspace.evHu[122];
acadoWorkspace.A[1967] = acadoWorkspace.evHu[123];
acadoWorkspace.A[1998] = acadoWorkspace.evHu[124];
acadoWorkspace.A[1999] = acadoWorkspace.evHu[125];
acadoWorkspace.A[2030] = acadoWorkspace.evHu[126];
acadoWorkspace.A[2031] = acadoWorkspace.evHu[127];
acadoWorkspace.A[2064] = acadoWorkspace.evHu[128];
acadoWorkspace.A[2065] = acadoWorkspace.evHu[129];
acadoWorkspace.A[2096] = acadoWorkspace.evHu[130];
acadoWorkspace.A[2097] = acadoWorkspace.evHu[131];
acadoWorkspace.A[2128] = acadoWorkspace.evHu[132];
acadoWorkspace.A[2129] = acadoWorkspace.evHu[133];
acadoWorkspace.A[2160] = acadoWorkspace.evHu[134];
acadoWorkspace.A[2161] = acadoWorkspace.evHu[135];
acadoWorkspace.A[2192] = acadoWorkspace.evHu[136];
acadoWorkspace.A[2193] = acadoWorkspace.evHu[137];
acadoWorkspace.A[2224] = acadoWorkspace.evHu[138];
acadoWorkspace.A[2225] = acadoWorkspace.evHu[139];
acadoWorkspace.A[2256] = acadoWorkspace.evHu[140];
acadoWorkspace.A[2257] = acadoWorkspace.evHu[141];
acadoWorkspace.A[2288] = acadoWorkspace.evHu[142];
acadoWorkspace.A[2289] = acadoWorkspace.evHu[143];
acadoWorkspace.A[2322] = acadoWorkspace.evHu[144];
acadoWorkspace.A[2323] = acadoWorkspace.evHu[145];
acadoWorkspace.A[2354] = acadoWorkspace.evHu[146];
acadoWorkspace.A[2355] = acadoWorkspace.evHu[147];
acadoWorkspace.A[2386] = acadoWorkspace.evHu[148];
acadoWorkspace.A[2387] = acadoWorkspace.evHu[149];
acadoWorkspace.A[2418] = acadoWorkspace.evHu[150];
acadoWorkspace.A[2419] = acadoWorkspace.evHu[151];
acadoWorkspace.A[2450] = acadoWorkspace.evHu[152];
acadoWorkspace.A[2451] = acadoWorkspace.evHu[153];
acadoWorkspace.A[2482] = acadoWorkspace.evHu[154];
acadoWorkspace.A[2483] = acadoWorkspace.evHu[155];
acadoWorkspace.A[2514] = acadoWorkspace.evHu[156];
acadoWorkspace.A[2515] = acadoWorkspace.evHu[157];
acadoWorkspace.A[2546] = acadoWorkspace.evHu[158];
acadoWorkspace.A[2547] = acadoWorkspace.evHu[159];
acadoWorkspace.A[2580] = acadoWorkspace.evHu[160];
acadoWorkspace.A[2581] = acadoWorkspace.evHu[161];
acadoWorkspace.A[2612] = acadoWorkspace.evHu[162];
acadoWorkspace.A[2613] = acadoWorkspace.evHu[163];
acadoWorkspace.A[2644] = acadoWorkspace.evHu[164];
acadoWorkspace.A[2645] = acadoWorkspace.evHu[165];
acadoWorkspace.A[2676] = acadoWorkspace.evHu[166];
acadoWorkspace.A[2677] = acadoWorkspace.evHu[167];
acadoWorkspace.A[2708] = acadoWorkspace.evHu[168];
acadoWorkspace.A[2709] = acadoWorkspace.evHu[169];
acadoWorkspace.A[2740] = acadoWorkspace.evHu[170];
acadoWorkspace.A[2741] = acadoWorkspace.evHu[171];
acadoWorkspace.A[2772] = acadoWorkspace.evHu[172];
acadoWorkspace.A[2773] = acadoWorkspace.evHu[173];
acadoWorkspace.A[2804] = acadoWorkspace.evHu[174];
acadoWorkspace.A[2805] = acadoWorkspace.evHu[175];
acadoWorkspace.A[2838] = acadoWorkspace.evHu[176];
acadoWorkspace.A[2839] = acadoWorkspace.evHu[177];
acadoWorkspace.A[2870] = acadoWorkspace.evHu[178];
acadoWorkspace.A[2871] = acadoWorkspace.evHu[179];
acadoWorkspace.A[2902] = acadoWorkspace.evHu[180];
acadoWorkspace.A[2903] = acadoWorkspace.evHu[181];
acadoWorkspace.A[2934] = acadoWorkspace.evHu[182];
acadoWorkspace.A[2935] = acadoWorkspace.evHu[183];
acadoWorkspace.A[2966] = acadoWorkspace.evHu[184];
acadoWorkspace.A[2967] = acadoWorkspace.evHu[185];
acadoWorkspace.A[2998] = acadoWorkspace.evHu[186];
acadoWorkspace.A[2999] = acadoWorkspace.evHu[187];
acadoWorkspace.A[3030] = acadoWorkspace.evHu[188];
acadoWorkspace.A[3031] = acadoWorkspace.evHu[189];
acadoWorkspace.A[3062] = acadoWorkspace.evHu[190];
acadoWorkspace.A[3063] = acadoWorkspace.evHu[191];
acadoWorkspace.A[3096] = acadoWorkspace.evHu[192];
acadoWorkspace.A[3097] = acadoWorkspace.evHu[193];
acadoWorkspace.A[3128] = acadoWorkspace.evHu[194];
acadoWorkspace.A[3129] = acadoWorkspace.evHu[195];
acadoWorkspace.A[3160] = acadoWorkspace.evHu[196];
acadoWorkspace.A[3161] = acadoWorkspace.evHu[197];
acadoWorkspace.A[3192] = acadoWorkspace.evHu[198];
acadoWorkspace.A[3193] = acadoWorkspace.evHu[199];
acadoWorkspace.A[3224] = acadoWorkspace.evHu[200];
acadoWorkspace.A[3225] = acadoWorkspace.evHu[201];
acadoWorkspace.A[3256] = acadoWorkspace.evHu[202];
acadoWorkspace.A[3257] = acadoWorkspace.evHu[203];
acadoWorkspace.A[3288] = acadoWorkspace.evHu[204];
acadoWorkspace.A[3289] = acadoWorkspace.evHu[205];
acadoWorkspace.A[3320] = acadoWorkspace.evHu[206];
acadoWorkspace.A[3321] = acadoWorkspace.evHu[207];
acadoWorkspace.A[3354] = acadoWorkspace.evHu[208];
acadoWorkspace.A[3355] = acadoWorkspace.evHu[209];
acadoWorkspace.A[3386] = acadoWorkspace.evHu[210];
acadoWorkspace.A[3387] = acadoWorkspace.evHu[211];
acadoWorkspace.A[3418] = acadoWorkspace.evHu[212];
acadoWorkspace.A[3419] = acadoWorkspace.evHu[213];
acadoWorkspace.A[3450] = acadoWorkspace.evHu[214];
acadoWorkspace.A[3451] = acadoWorkspace.evHu[215];
acadoWorkspace.A[3482] = acadoWorkspace.evHu[216];
acadoWorkspace.A[3483] = acadoWorkspace.evHu[217];
acadoWorkspace.A[3514] = acadoWorkspace.evHu[218];
acadoWorkspace.A[3515] = acadoWorkspace.evHu[219];
acadoWorkspace.A[3546] = acadoWorkspace.evHu[220];
acadoWorkspace.A[3547] = acadoWorkspace.evHu[221];
acadoWorkspace.A[3578] = acadoWorkspace.evHu[222];
acadoWorkspace.A[3579] = acadoWorkspace.evHu[223];
acadoWorkspace.A[3612] = acadoWorkspace.evHu[224];
acadoWorkspace.A[3613] = acadoWorkspace.evHu[225];
acadoWorkspace.A[3644] = acadoWorkspace.evHu[226];
acadoWorkspace.A[3645] = acadoWorkspace.evHu[227];
acadoWorkspace.A[3676] = acadoWorkspace.evHu[228];
acadoWorkspace.A[3677] = acadoWorkspace.evHu[229];
acadoWorkspace.A[3708] = acadoWorkspace.evHu[230];
acadoWorkspace.A[3709] = acadoWorkspace.evHu[231];
acadoWorkspace.A[3740] = acadoWorkspace.evHu[232];
acadoWorkspace.A[3741] = acadoWorkspace.evHu[233];
acadoWorkspace.A[3772] = acadoWorkspace.evHu[234];
acadoWorkspace.A[3773] = acadoWorkspace.evHu[235];
acadoWorkspace.A[3804] = acadoWorkspace.evHu[236];
acadoWorkspace.A[3805] = acadoWorkspace.evHu[237];
acadoWorkspace.A[3836] = acadoWorkspace.evHu[238];
acadoWorkspace.A[3837] = acadoWorkspace.evHu[239];
acadoWorkspace.A[3870] = acadoWorkspace.evHu[240];
acadoWorkspace.A[3871] = acadoWorkspace.evHu[241];
acadoWorkspace.A[3902] = acadoWorkspace.evHu[242];
acadoWorkspace.A[3903] = acadoWorkspace.evHu[243];
acadoWorkspace.A[3934] = acadoWorkspace.evHu[244];
acadoWorkspace.A[3935] = acadoWorkspace.evHu[245];
acadoWorkspace.A[3966] = acadoWorkspace.evHu[246];
acadoWorkspace.A[3967] = acadoWorkspace.evHu[247];
acadoWorkspace.A[3998] = acadoWorkspace.evHu[248];
acadoWorkspace.A[3999] = acadoWorkspace.evHu[249];
acadoWorkspace.A[4030] = acadoWorkspace.evHu[250];
acadoWorkspace.A[4031] = acadoWorkspace.evHu[251];
acadoWorkspace.A[4062] = acadoWorkspace.evHu[252];
acadoWorkspace.A[4063] = acadoWorkspace.evHu[253];
acadoWorkspace.A[4094] = acadoWorkspace.evHu[254];
acadoWorkspace.A[4095] = acadoWorkspace.evHu[255];
acadoWorkspace.lbA[0] = - acadoWorkspace.evH[0];
acadoWorkspace.lbA[1] = - acadoWorkspace.evH[1];
acadoWorkspace.lbA[2] = - acadoWorkspace.evH[2];
acadoWorkspace.lbA[3] = - acadoWorkspace.evH[3];
acadoWorkspace.lbA[4] = - acadoWorkspace.evH[4];
acadoWorkspace.lbA[5] = - acadoWorkspace.evH[5];
acadoWorkspace.lbA[6] = - acadoWorkspace.evH[6];
acadoWorkspace.lbA[7] = - acadoWorkspace.evH[7];
acadoWorkspace.lbA[8] = - acadoWorkspace.evH[8];
acadoWorkspace.lbA[9] = - acadoWorkspace.evH[9];
acadoWorkspace.lbA[10] = - acadoWorkspace.evH[10];
acadoWorkspace.lbA[11] = - acadoWorkspace.evH[11];
acadoWorkspace.lbA[12] = - acadoWorkspace.evH[12];
acadoWorkspace.lbA[13] = - acadoWorkspace.evH[13];
acadoWorkspace.lbA[14] = - acadoWorkspace.evH[14];
acadoWorkspace.lbA[15] = - acadoWorkspace.evH[15];
acadoWorkspace.lbA[16] = - acadoWorkspace.evH[16];
acadoWorkspace.lbA[17] = - acadoWorkspace.evH[17];
acadoWorkspace.lbA[18] = - acadoWorkspace.evH[18];
acadoWorkspace.lbA[19] = - acadoWorkspace.evH[19];
acadoWorkspace.lbA[20] = - acadoWorkspace.evH[20];
acadoWorkspace.lbA[21] = - acadoWorkspace.evH[21];
acadoWorkspace.lbA[22] = - acadoWorkspace.evH[22];
acadoWorkspace.lbA[23] = - acadoWorkspace.evH[23];
acadoWorkspace.lbA[24] = - acadoWorkspace.evH[24];
acadoWorkspace.lbA[25] = - acadoWorkspace.evH[25];
acadoWorkspace.lbA[26] = - acadoWorkspace.evH[26];
acadoWorkspace.lbA[27] = - acadoWorkspace.evH[27];
acadoWorkspace.lbA[28] = - acadoWorkspace.evH[28];
acadoWorkspace.lbA[29] = - acadoWorkspace.evH[29];
acadoWorkspace.lbA[30] = - acadoWorkspace.evH[30];
acadoWorkspace.lbA[31] = - acadoWorkspace.evH[31];
acadoWorkspace.lbA[32] = - acadoWorkspace.evH[32];
acadoWorkspace.lbA[33] = - acadoWorkspace.evH[33];
acadoWorkspace.lbA[34] = - acadoWorkspace.evH[34];
acadoWorkspace.lbA[35] = - acadoWorkspace.evH[35];
acadoWorkspace.lbA[36] = - acadoWorkspace.evH[36];
acadoWorkspace.lbA[37] = - acadoWorkspace.evH[37];
acadoWorkspace.lbA[38] = - acadoWorkspace.evH[38];
acadoWorkspace.lbA[39] = - acadoWorkspace.evH[39];
acadoWorkspace.lbA[40] = - acadoWorkspace.evH[40];
acadoWorkspace.lbA[41] = - acadoWorkspace.evH[41];
acadoWorkspace.lbA[42] = - acadoWorkspace.evH[42];
acadoWorkspace.lbA[43] = - acadoWorkspace.evH[43];
acadoWorkspace.lbA[44] = - acadoWorkspace.evH[44];
acadoWorkspace.lbA[45] = - acadoWorkspace.evH[45];
acadoWorkspace.lbA[46] = - acadoWorkspace.evH[46];
acadoWorkspace.lbA[47] = - acadoWorkspace.evH[47];
acadoWorkspace.lbA[48] = - acadoWorkspace.evH[48];
acadoWorkspace.lbA[49] = - acadoWorkspace.evH[49];
acadoWorkspace.lbA[50] = - acadoWorkspace.evH[50];
acadoWorkspace.lbA[51] = - acadoWorkspace.evH[51];
acadoWorkspace.lbA[52] = - acadoWorkspace.evH[52];
acadoWorkspace.lbA[53] = - acadoWorkspace.evH[53];
acadoWorkspace.lbA[54] = - acadoWorkspace.evH[54];
acadoWorkspace.lbA[55] = - acadoWorkspace.evH[55];
acadoWorkspace.lbA[56] = - acadoWorkspace.evH[56];
acadoWorkspace.lbA[57] = - acadoWorkspace.evH[57];
acadoWorkspace.lbA[58] = - acadoWorkspace.evH[58];
acadoWorkspace.lbA[59] = - acadoWorkspace.evH[59];
acadoWorkspace.lbA[60] = - acadoWorkspace.evH[60];
acadoWorkspace.lbA[61] = - acadoWorkspace.evH[61];
acadoWorkspace.lbA[62] = - acadoWorkspace.evH[62];
acadoWorkspace.lbA[63] = - acadoWorkspace.evH[63];
acadoWorkspace.lbA[64] = - acadoWorkspace.evH[64];
acadoWorkspace.lbA[65] = - acadoWorkspace.evH[65];
acadoWorkspace.lbA[66] = - acadoWorkspace.evH[66];
acadoWorkspace.lbA[67] = - acadoWorkspace.evH[67];
acadoWorkspace.lbA[68] = - acadoWorkspace.evH[68];
acadoWorkspace.lbA[69] = - acadoWorkspace.evH[69];
acadoWorkspace.lbA[70] = - acadoWorkspace.evH[70];
acadoWorkspace.lbA[71] = - acadoWorkspace.evH[71];
acadoWorkspace.lbA[72] = - acadoWorkspace.evH[72];
acadoWorkspace.lbA[73] = - acadoWorkspace.evH[73];
acadoWorkspace.lbA[74] = - acadoWorkspace.evH[74];
acadoWorkspace.lbA[75] = - acadoWorkspace.evH[75];
acadoWorkspace.lbA[76] = - acadoWorkspace.evH[76];
acadoWorkspace.lbA[77] = - acadoWorkspace.evH[77];
acadoWorkspace.lbA[78] = - acadoWorkspace.evH[78];
acadoWorkspace.lbA[79] = - acadoWorkspace.evH[79];
acadoWorkspace.lbA[80] = - acadoWorkspace.evH[80];
acadoWorkspace.lbA[81] = - acadoWorkspace.evH[81];
acadoWorkspace.lbA[82] = - acadoWorkspace.evH[82];
acadoWorkspace.lbA[83] = - acadoWorkspace.evH[83];
acadoWorkspace.lbA[84] = - acadoWorkspace.evH[84];
acadoWorkspace.lbA[85] = - acadoWorkspace.evH[85];
acadoWorkspace.lbA[86] = - acadoWorkspace.evH[86];
acadoWorkspace.lbA[87] = - acadoWorkspace.evH[87];
acadoWorkspace.lbA[88] = - acadoWorkspace.evH[88];
acadoWorkspace.lbA[89] = - acadoWorkspace.evH[89];
acadoWorkspace.lbA[90] = - acadoWorkspace.evH[90];
acadoWorkspace.lbA[91] = - acadoWorkspace.evH[91];
acadoWorkspace.lbA[92] = - acadoWorkspace.evH[92];
acadoWorkspace.lbA[93] = - acadoWorkspace.evH[93];
acadoWorkspace.lbA[94] = - acadoWorkspace.evH[94];
acadoWorkspace.lbA[95] = - acadoWorkspace.evH[95];
acadoWorkspace.lbA[96] = - acadoWorkspace.evH[96];
acadoWorkspace.lbA[97] = - acadoWorkspace.evH[97];
acadoWorkspace.lbA[98] = - acadoWorkspace.evH[98];
acadoWorkspace.lbA[99] = - acadoWorkspace.evH[99];
acadoWorkspace.lbA[100] = - acadoWorkspace.evH[100];
acadoWorkspace.lbA[101] = - acadoWorkspace.evH[101];
acadoWorkspace.lbA[102] = - acadoWorkspace.evH[102];
acadoWorkspace.lbA[103] = - acadoWorkspace.evH[103];
acadoWorkspace.lbA[104] = - acadoWorkspace.evH[104];
acadoWorkspace.lbA[105] = - acadoWorkspace.evH[105];
acadoWorkspace.lbA[106] = - acadoWorkspace.evH[106];
acadoWorkspace.lbA[107] = - acadoWorkspace.evH[107];
acadoWorkspace.lbA[108] = - acadoWorkspace.evH[108];
acadoWorkspace.lbA[109] = - acadoWorkspace.evH[109];
acadoWorkspace.lbA[110] = - acadoWorkspace.evH[110];
acadoWorkspace.lbA[111] = - acadoWorkspace.evH[111];
acadoWorkspace.lbA[112] = - acadoWorkspace.evH[112];
acadoWorkspace.lbA[113] = - acadoWorkspace.evH[113];
acadoWorkspace.lbA[114] = - acadoWorkspace.evH[114];
acadoWorkspace.lbA[115] = - acadoWorkspace.evH[115];
acadoWorkspace.lbA[116] = - acadoWorkspace.evH[116];
acadoWorkspace.lbA[117] = - acadoWorkspace.evH[117];
acadoWorkspace.lbA[118] = - acadoWorkspace.evH[118];
acadoWorkspace.lbA[119] = - acadoWorkspace.evH[119];
acadoWorkspace.lbA[120] = - acadoWorkspace.evH[120];
acadoWorkspace.lbA[121] = - acadoWorkspace.evH[121];
acadoWorkspace.lbA[122] = - acadoWorkspace.evH[122];
acadoWorkspace.lbA[123] = - acadoWorkspace.evH[123];
acadoWorkspace.lbA[124] = - acadoWorkspace.evH[124];
acadoWorkspace.lbA[125] = - acadoWorkspace.evH[125];
acadoWorkspace.lbA[126] = - acadoWorkspace.evH[126];
acadoWorkspace.lbA[127] = - acadoWorkspace.evH[127];

acadoWorkspace.ubA[0] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[0];
acadoWorkspace.ubA[1] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[1];
acadoWorkspace.ubA[2] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[2];
acadoWorkspace.ubA[3] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[3];
acadoWorkspace.ubA[4] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[4];
acadoWorkspace.ubA[5] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[5];
acadoWorkspace.ubA[6] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[6];
acadoWorkspace.ubA[7] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[7];
acadoWorkspace.ubA[8] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[8];
acadoWorkspace.ubA[9] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[9];
acadoWorkspace.ubA[10] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[10];
acadoWorkspace.ubA[11] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[11];
acadoWorkspace.ubA[12] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[12];
acadoWorkspace.ubA[13] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[13];
acadoWorkspace.ubA[14] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[14];
acadoWorkspace.ubA[15] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[15];
acadoWorkspace.ubA[16] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[16];
acadoWorkspace.ubA[17] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[17];
acadoWorkspace.ubA[18] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[18];
acadoWorkspace.ubA[19] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[19];
acadoWorkspace.ubA[20] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[20];
acadoWorkspace.ubA[21] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[21];
acadoWorkspace.ubA[22] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[22];
acadoWorkspace.ubA[23] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[23];
acadoWorkspace.ubA[24] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[24];
acadoWorkspace.ubA[25] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[25];
acadoWorkspace.ubA[26] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[26];
acadoWorkspace.ubA[27] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[27];
acadoWorkspace.ubA[28] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[28];
acadoWorkspace.ubA[29] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[29];
acadoWorkspace.ubA[30] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[30];
acadoWorkspace.ubA[31] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[31];
acadoWorkspace.ubA[32] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[32];
acadoWorkspace.ubA[33] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[33];
acadoWorkspace.ubA[34] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[34];
acadoWorkspace.ubA[35] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[35];
acadoWorkspace.ubA[36] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[36];
acadoWorkspace.ubA[37] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[37];
acadoWorkspace.ubA[38] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[38];
acadoWorkspace.ubA[39] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[39];
acadoWorkspace.ubA[40] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[40];
acadoWorkspace.ubA[41] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[41];
acadoWorkspace.ubA[42] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[42];
acadoWorkspace.ubA[43] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[43];
acadoWorkspace.ubA[44] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[44];
acadoWorkspace.ubA[45] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[45];
acadoWorkspace.ubA[46] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[46];
acadoWorkspace.ubA[47] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[47];
acadoWorkspace.ubA[48] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[48];
acadoWorkspace.ubA[49] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[49];
acadoWorkspace.ubA[50] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[50];
acadoWorkspace.ubA[51] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[51];
acadoWorkspace.ubA[52] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[52];
acadoWorkspace.ubA[53] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[53];
acadoWorkspace.ubA[54] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[54];
acadoWorkspace.ubA[55] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[55];
acadoWorkspace.ubA[56] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[56];
acadoWorkspace.ubA[57] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[57];
acadoWorkspace.ubA[58] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[58];
acadoWorkspace.ubA[59] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[59];
acadoWorkspace.ubA[60] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[60];
acadoWorkspace.ubA[61] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[61];
acadoWorkspace.ubA[62] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[62];
acadoWorkspace.ubA[63] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[63];
acadoWorkspace.ubA[64] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[64];
acadoWorkspace.ubA[65] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[65];
acadoWorkspace.ubA[66] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[66];
acadoWorkspace.ubA[67] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[67];
acadoWorkspace.ubA[68] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[68];
acadoWorkspace.ubA[69] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[69];
acadoWorkspace.ubA[70] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[70];
acadoWorkspace.ubA[71] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[71];
acadoWorkspace.ubA[72] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[72];
acadoWorkspace.ubA[73] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[73];
acadoWorkspace.ubA[74] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[74];
acadoWorkspace.ubA[75] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[75];
acadoWorkspace.ubA[76] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[76];
acadoWorkspace.ubA[77] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[77];
acadoWorkspace.ubA[78] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[78];
acadoWorkspace.ubA[79] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[79];
acadoWorkspace.ubA[80] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[80];
acadoWorkspace.ubA[81] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[81];
acadoWorkspace.ubA[82] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[82];
acadoWorkspace.ubA[83] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[83];
acadoWorkspace.ubA[84] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[84];
acadoWorkspace.ubA[85] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[85];
acadoWorkspace.ubA[86] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[86];
acadoWorkspace.ubA[87] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[87];
acadoWorkspace.ubA[88] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[88];
acadoWorkspace.ubA[89] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[89];
acadoWorkspace.ubA[90] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[90];
acadoWorkspace.ubA[91] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[91];
acadoWorkspace.ubA[92] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[92];
acadoWorkspace.ubA[93] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[93];
acadoWorkspace.ubA[94] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[94];
acadoWorkspace.ubA[95] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[95];
acadoWorkspace.ubA[96] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[96];
acadoWorkspace.ubA[97] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[97];
acadoWorkspace.ubA[98] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[98];
acadoWorkspace.ubA[99] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[99];
acadoWorkspace.ubA[100] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[100];
acadoWorkspace.ubA[101] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[101];
acadoWorkspace.ubA[102] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[102];
acadoWorkspace.ubA[103] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[103];
acadoWorkspace.ubA[104] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[104];
acadoWorkspace.ubA[105] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[105];
acadoWorkspace.ubA[106] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[106];
acadoWorkspace.ubA[107] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[107];
acadoWorkspace.ubA[108] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[108];
acadoWorkspace.ubA[109] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[109];
acadoWorkspace.ubA[110] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[110];
acadoWorkspace.ubA[111] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[111];
acadoWorkspace.ubA[112] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[112];
acadoWorkspace.ubA[113] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[113];
acadoWorkspace.ubA[114] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[114];
acadoWorkspace.ubA[115] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[115];
acadoWorkspace.ubA[116] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[116];
acadoWorkspace.ubA[117] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[117];
acadoWorkspace.ubA[118] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[118];
acadoWorkspace.ubA[119] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[119];
acadoWorkspace.ubA[120] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[120];
acadoWorkspace.ubA[121] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[121];
acadoWorkspace.ubA[122] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[122];
acadoWorkspace.ubA[123] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[123];
acadoWorkspace.ubA[124] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[124];
acadoWorkspace.ubA[125] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[125];
acadoWorkspace.ubA[126] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[126];
acadoWorkspace.ubA[127] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[127];

acado_macHxd( &(acadoWorkspace.evHx[ 32 ]), acadoWorkspace.d, &(acadoWorkspace.lbA[ 8 ]), &(acadoWorkspace.ubA[ 8 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 64 ]), &(acadoWorkspace.d[ 4 ]), &(acadoWorkspace.lbA[ 16 ]), &(acadoWorkspace.ubA[ 16 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 96 ]), &(acadoWorkspace.d[ 8 ]), &(acadoWorkspace.lbA[ 24 ]), &(acadoWorkspace.ubA[ 24 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 128 ]), &(acadoWorkspace.d[ 12 ]), &(acadoWorkspace.lbA[ 32 ]), &(acadoWorkspace.ubA[ 32 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 160 ]), &(acadoWorkspace.d[ 16 ]), &(acadoWorkspace.lbA[ 40 ]), &(acadoWorkspace.ubA[ 40 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 192 ]), &(acadoWorkspace.d[ 20 ]), &(acadoWorkspace.lbA[ 48 ]), &(acadoWorkspace.ubA[ 48 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 224 ]), &(acadoWorkspace.d[ 24 ]), &(acadoWorkspace.lbA[ 56 ]), &(acadoWorkspace.ubA[ 56 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 256 ]), &(acadoWorkspace.d[ 28 ]), &(acadoWorkspace.lbA[ 64 ]), &(acadoWorkspace.ubA[ 64 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 288 ]), &(acadoWorkspace.d[ 32 ]), &(acadoWorkspace.lbA[ 72 ]), &(acadoWorkspace.ubA[ 72 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 320 ]), &(acadoWorkspace.d[ 36 ]), &(acadoWorkspace.lbA[ 80 ]), &(acadoWorkspace.ubA[ 80 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 352 ]), &(acadoWorkspace.d[ 40 ]), &(acadoWorkspace.lbA[ 88 ]), &(acadoWorkspace.ubA[ 88 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 384 ]), &(acadoWorkspace.d[ 44 ]), &(acadoWorkspace.lbA[ 96 ]), &(acadoWorkspace.ubA[ 96 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 416 ]), &(acadoWorkspace.d[ 48 ]), &(acadoWorkspace.lbA[ 104 ]), &(acadoWorkspace.ubA[ 104 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 448 ]), &(acadoWorkspace.d[ 52 ]), &(acadoWorkspace.lbA[ 112 ]), &(acadoWorkspace.ubA[ 112 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 480 ]), &(acadoWorkspace.d[ 56 ]), &(acadoWorkspace.lbA[ 120 ]), &(acadoWorkspace.ubA[ 120 ]) );

}

void acado_condenseFdb(  )
{
int lRun1;
acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
acadoWorkspace.Dx0[1] = acadoVariables.x0[1] - acadoVariables.x[1];
acadoWorkspace.Dx0[2] = acadoVariables.x0[2] - acadoVariables.x[2];
acadoWorkspace.Dx0[3] = acadoVariables.x0[3] - acadoVariables.x[3];

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
acadoWorkspace.Dy[30] -= acadoVariables.y[30];
acadoWorkspace.Dy[31] -= acadoVariables.y[31];
acadoWorkspace.Dy[32] -= acadoVariables.y[32];
acadoWorkspace.Dy[33] -= acadoVariables.y[33];
acadoWorkspace.Dy[34] -= acadoVariables.y[34];
acadoWorkspace.Dy[35] -= acadoVariables.y[35];
acadoWorkspace.Dy[36] -= acadoVariables.y[36];
acadoWorkspace.Dy[37] -= acadoVariables.y[37];
acadoWorkspace.Dy[38] -= acadoVariables.y[38];
acadoWorkspace.Dy[39] -= acadoVariables.y[39];
acadoWorkspace.Dy[40] -= acadoVariables.y[40];
acadoWorkspace.Dy[41] -= acadoVariables.y[41];
acadoWorkspace.Dy[42] -= acadoVariables.y[42];
acadoWorkspace.Dy[43] -= acadoVariables.y[43];
acadoWorkspace.Dy[44] -= acadoVariables.y[44];
acadoWorkspace.Dy[45] -= acadoVariables.y[45];
acadoWorkspace.Dy[46] -= acadoVariables.y[46];
acadoWorkspace.Dy[47] -= acadoVariables.y[47];
acadoWorkspace.Dy[48] -= acadoVariables.y[48];
acadoWorkspace.Dy[49] -= acadoVariables.y[49];
acadoWorkspace.Dy[50] -= acadoVariables.y[50];
acadoWorkspace.Dy[51] -= acadoVariables.y[51];
acadoWorkspace.Dy[52] -= acadoVariables.y[52];
acadoWorkspace.Dy[53] -= acadoVariables.y[53];
acadoWorkspace.Dy[54] -= acadoVariables.y[54];
acadoWorkspace.Dy[55] -= acadoVariables.y[55];
acadoWorkspace.Dy[56] -= acadoVariables.y[56];
acadoWorkspace.Dy[57] -= acadoVariables.y[57];
acadoWorkspace.Dy[58] -= acadoVariables.y[58];
acadoWorkspace.Dy[59] -= acadoVariables.y[59];
acadoWorkspace.Dy[60] -= acadoVariables.y[60];
acadoWorkspace.Dy[61] -= acadoVariables.y[61];
acadoWorkspace.Dy[62] -= acadoVariables.y[62];
acadoWorkspace.Dy[63] -= acadoVariables.y[63];
acadoWorkspace.Dy[64] -= acadoVariables.y[64];
acadoWorkspace.Dy[65] -= acadoVariables.y[65];
acadoWorkspace.Dy[66] -= acadoVariables.y[66];
acadoWorkspace.Dy[67] -= acadoVariables.y[67];
acadoWorkspace.Dy[68] -= acadoVariables.y[68];
acadoWorkspace.Dy[69] -= acadoVariables.y[69];
acadoWorkspace.Dy[70] -= acadoVariables.y[70];
acadoWorkspace.Dy[71] -= acadoVariables.y[71];
acadoWorkspace.Dy[72] -= acadoVariables.y[72];
acadoWorkspace.Dy[73] -= acadoVariables.y[73];
acadoWorkspace.Dy[74] -= acadoVariables.y[74];
acadoWorkspace.Dy[75] -= acadoVariables.y[75];
acadoWorkspace.Dy[76] -= acadoVariables.y[76];
acadoWorkspace.Dy[77] -= acadoVariables.y[77];
acadoWorkspace.Dy[78] -= acadoVariables.y[78];
acadoWorkspace.Dy[79] -= acadoVariables.y[79];
acadoWorkspace.Dy[80] -= acadoVariables.y[80];
acadoWorkspace.Dy[81] -= acadoVariables.y[81];
acadoWorkspace.Dy[82] -= acadoVariables.y[82];
acadoWorkspace.Dy[83] -= acadoVariables.y[83];
acadoWorkspace.Dy[84] -= acadoVariables.y[84];
acadoWorkspace.Dy[85] -= acadoVariables.y[85];
acadoWorkspace.Dy[86] -= acadoVariables.y[86];
acadoWorkspace.Dy[87] -= acadoVariables.y[87];
acadoWorkspace.Dy[88] -= acadoVariables.y[88];
acadoWorkspace.Dy[89] -= acadoVariables.y[89];
acadoWorkspace.Dy[90] -= acadoVariables.y[90];
acadoWorkspace.Dy[91] -= acadoVariables.y[91];
acadoWorkspace.Dy[92] -= acadoVariables.y[92];
acadoWorkspace.Dy[93] -= acadoVariables.y[93];
acadoWorkspace.Dy[94] -= acadoVariables.y[94];
acadoWorkspace.Dy[95] -= acadoVariables.y[95];
acadoWorkspace.Dy[96] -= acadoVariables.y[96];
acadoWorkspace.Dy[97] -= acadoVariables.y[97];
acadoWorkspace.Dy[98] -= acadoVariables.y[98];
acadoWorkspace.Dy[99] -= acadoVariables.y[99];
acadoWorkspace.Dy[100] -= acadoVariables.y[100];
acadoWorkspace.Dy[101] -= acadoVariables.y[101];
acadoWorkspace.Dy[102] -= acadoVariables.y[102];
acadoWorkspace.Dy[103] -= acadoVariables.y[103];
acadoWorkspace.Dy[104] -= acadoVariables.y[104];
acadoWorkspace.Dy[105] -= acadoVariables.y[105];
acadoWorkspace.Dy[106] -= acadoVariables.y[106];
acadoWorkspace.Dy[107] -= acadoVariables.y[107];
acadoWorkspace.Dy[108] -= acadoVariables.y[108];
acadoWorkspace.Dy[109] -= acadoVariables.y[109];
acadoWorkspace.Dy[110] -= acadoVariables.y[110];
acadoWorkspace.Dy[111] -= acadoVariables.y[111];
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

acadoWorkspace.QDy[64] = + acadoWorkspace.QN2[0]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[1]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[2]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[3]*acadoWorkspace.DyN[3];
acadoWorkspace.QDy[65] = + acadoWorkspace.QN2[4]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[5]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[6]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[7]*acadoWorkspace.DyN[3];
acadoWorkspace.QDy[66] = + acadoWorkspace.QN2[8]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[9]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[10]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[11]*acadoWorkspace.DyN[3];
acadoWorkspace.QDy[67] = + acadoWorkspace.QN2[12]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[13]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[14]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[15]*acadoWorkspace.DyN[3];

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
acado_multEQDy( &(acadoWorkspace.E[ 280 ]), &(acadoWorkspace.QDy[ 32 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 344 ]), &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 416 ]), &(acadoWorkspace.QDy[ 40 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 496 ]), &(acadoWorkspace.QDy[ 44 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 584 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 680 ]), &(acadoWorkspace.QDy[ 52 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 784 ]), &(acadoWorkspace.QDy[ 56 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 896 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1016 ]), &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 352 ]), &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 424 ]), &(acadoWorkspace.QDy[ 40 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 504 ]), &(acadoWorkspace.QDy[ 44 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 592 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 688 ]), &(acadoWorkspace.QDy[ 52 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 792 ]), &(acadoWorkspace.QDy[ 56 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 904 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1024 ]), &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QDy[ 40 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 512 ]), &(acadoWorkspace.QDy[ 44 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 600 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 696 ]), &(acadoWorkspace.QDy[ 52 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 800 ]), &(acadoWorkspace.QDy[ 56 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 912 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1032 ]), &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 520 ]), &(acadoWorkspace.QDy[ 44 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 608 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 704 ]), &(acadoWorkspace.QDy[ 52 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 808 ]), &(acadoWorkspace.QDy[ 56 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 920 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1040 ]), &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 616 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 22 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 712 ]), &(acadoWorkspace.QDy[ 52 ]), &(acadoWorkspace.g[ 22 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 816 ]), &(acadoWorkspace.QDy[ 56 ]), &(acadoWorkspace.g[ 22 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 928 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 22 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1048 ]), &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.g[ 22 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.QDy[ 52 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 824 ]), &(acadoWorkspace.QDy[ 56 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 936 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1056 ]), &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 832 ]), &(acadoWorkspace.QDy[ 56 ]), &(acadoWorkspace.g[ 26 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 944 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 26 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1064 ]), &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.g[ 26 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 952 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 28 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1072 ]), &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.g[ 28 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1080 ]), &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.g[ 30 ]) );

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

acadoWorkspace.pacA01Dx0[0] = + acadoWorkspace.A01[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[2]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[3]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[1] = + acadoWorkspace.A01[4]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[5]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[6]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[7]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[2] = + acadoWorkspace.A01[8]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[9]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[10]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[11]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[3] = + acadoWorkspace.A01[12]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[13]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[14]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[15]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[4] = + acadoWorkspace.A01[16]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[17]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[18]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[19]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[5] = + acadoWorkspace.A01[20]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[21]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[22]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[23]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[6] = + acadoWorkspace.A01[24]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[25]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[26]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[27]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[7] = + acadoWorkspace.A01[28]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[29]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[30]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[31]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[8] = + acadoWorkspace.A01[32]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[33]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[34]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[35]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[9] = + acadoWorkspace.A01[36]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[37]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[38]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[39]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[10] = + acadoWorkspace.A01[40]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[41]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[42]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[43]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[11] = + acadoWorkspace.A01[44]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[45]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[46]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[47]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[12] = + acadoWorkspace.A01[48]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[49]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[50]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[51]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[13] = + acadoWorkspace.A01[52]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[53]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[54]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[55]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[14] = + acadoWorkspace.A01[56]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[57]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[58]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[59]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[15] = + acadoWorkspace.A01[60]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[61]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[62]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[63]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[16] = + acadoWorkspace.A01[64]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[65]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[66]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[67]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[17] = + acadoWorkspace.A01[68]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[69]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[70]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[71]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[18] = + acadoWorkspace.A01[72]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[73]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[74]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[75]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[19] = + acadoWorkspace.A01[76]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[77]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[78]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[79]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[20] = + acadoWorkspace.A01[80]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[81]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[82]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[83]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[21] = + acadoWorkspace.A01[84]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[85]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[86]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[87]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[22] = + acadoWorkspace.A01[88]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[89]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[90]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[91]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[23] = + acadoWorkspace.A01[92]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[93]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[94]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[95]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[24] = + acadoWorkspace.A01[96]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[97]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[98]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[99]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[25] = + acadoWorkspace.A01[100]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[101]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[102]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[103]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[26] = + acadoWorkspace.A01[104]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[105]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[106]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[107]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[27] = + acadoWorkspace.A01[108]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[109]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[110]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[111]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[28] = + acadoWorkspace.A01[112]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[113]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[114]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[115]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[29] = + acadoWorkspace.A01[116]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[117]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[118]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[119]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[30] = + acadoWorkspace.A01[120]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[121]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[122]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[123]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[31] = + acadoWorkspace.A01[124]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[125]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[126]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[127]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[32] = + acadoWorkspace.A01[128]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[129]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[130]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[131]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[33] = + acadoWorkspace.A01[132]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[133]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[134]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[135]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[34] = + acadoWorkspace.A01[136]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[137]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[138]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[139]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[35] = + acadoWorkspace.A01[140]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[141]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[142]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[143]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[36] = + acadoWorkspace.A01[144]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[145]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[146]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[147]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[37] = + acadoWorkspace.A01[148]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[149]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[150]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[151]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[38] = + acadoWorkspace.A01[152]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[153]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[154]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[155]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[39] = + acadoWorkspace.A01[156]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[157]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[158]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[159]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[40] = + acadoWorkspace.A01[160]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[161]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[162]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[163]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[41] = + acadoWorkspace.A01[164]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[165]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[166]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[167]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[42] = + acadoWorkspace.A01[168]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[169]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[170]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[171]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[43] = + acadoWorkspace.A01[172]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[173]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[174]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[175]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[44] = + acadoWorkspace.A01[176]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[177]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[178]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[179]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[45] = + acadoWorkspace.A01[180]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[181]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[182]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[183]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[46] = + acadoWorkspace.A01[184]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[185]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[186]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[187]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[47] = + acadoWorkspace.A01[188]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[189]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[190]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[191]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[48] = + acadoWorkspace.A01[192]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[193]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[194]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[195]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[49] = + acadoWorkspace.A01[196]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[197]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[198]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[199]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[50] = + acadoWorkspace.A01[200]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[201]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[202]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[203]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[51] = + acadoWorkspace.A01[204]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[205]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[206]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[207]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[52] = + acadoWorkspace.A01[208]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[209]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[210]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[211]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[53] = + acadoWorkspace.A01[212]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[213]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[214]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[215]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[54] = + acadoWorkspace.A01[216]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[217]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[218]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[219]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[55] = + acadoWorkspace.A01[220]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[221]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[222]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[223]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[56] = + acadoWorkspace.A01[224]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[225]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[226]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[227]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[57] = + acadoWorkspace.A01[228]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[229]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[230]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[231]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[58] = + acadoWorkspace.A01[232]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[233]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[234]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[235]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[59] = + acadoWorkspace.A01[236]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[237]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[238]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[239]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[60] = + acadoWorkspace.A01[240]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[241]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[242]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[243]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[61] = + acadoWorkspace.A01[244]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[245]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[246]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[247]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[62] = + acadoWorkspace.A01[248]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[249]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[250]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[251]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[63] = + acadoWorkspace.A01[252]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[253]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[254]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[255]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[64] = + acadoWorkspace.A01[256]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[257]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[258]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[259]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[65] = + acadoWorkspace.A01[260]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[261]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[262]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[263]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[66] = + acadoWorkspace.A01[264]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[265]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[266]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[267]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[67] = + acadoWorkspace.A01[268]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[269]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[270]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[271]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[68] = + acadoWorkspace.A01[272]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[273]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[274]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[275]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[69] = + acadoWorkspace.A01[276]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[277]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[278]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[279]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[70] = + acadoWorkspace.A01[280]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[281]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[282]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[283]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[71] = + acadoWorkspace.A01[284]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[285]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[286]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[287]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[72] = + acadoWorkspace.A01[288]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[289]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[290]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[291]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[73] = + acadoWorkspace.A01[292]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[293]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[294]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[295]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[74] = + acadoWorkspace.A01[296]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[297]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[298]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[299]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[75] = + acadoWorkspace.A01[300]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[301]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[302]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[303]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[76] = + acadoWorkspace.A01[304]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[305]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[306]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[307]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[77] = + acadoWorkspace.A01[308]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[309]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[310]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[311]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[78] = + acadoWorkspace.A01[312]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[313]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[314]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[315]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[79] = + acadoWorkspace.A01[316]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[317]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[318]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[319]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[80] = + acadoWorkspace.A01[320]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[321]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[322]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[323]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[81] = + acadoWorkspace.A01[324]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[325]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[326]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[327]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[82] = + acadoWorkspace.A01[328]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[329]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[330]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[331]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[83] = + acadoWorkspace.A01[332]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[333]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[334]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[335]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[84] = + acadoWorkspace.A01[336]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[337]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[338]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[339]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[85] = + acadoWorkspace.A01[340]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[341]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[342]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[343]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[86] = + acadoWorkspace.A01[344]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[345]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[346]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[347]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[87] = + acadoWorkspace.A01[348]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[349]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[350]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[351]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[88] = + acadoWorkspace.A01[352]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[353]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[354]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[355]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[89] = + acadoWorkspace.A01[356]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[357]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[358]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[359]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[90] = + acadoWorkspace.A01[360]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[361]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[362]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[363]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[91] = + acadoWorkspace.A01[364]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[365]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[366]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[367]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[92] = + acadoWorkspace.A01[368]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[369]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[370]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[371]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[93] = + acadoWorkspace.A01[372]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[373]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[374]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[375]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[94] = + acadoWorkspace.A01[376]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[377]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[378]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[379]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[95] = + acadoWorkspace.A01[380]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[381]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[382]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[383]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[96] = + acadoWorkspace.A01[384]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[385]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[386]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[387]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[97] = + acadoWorkspace.A01[388]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[389]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[390]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[391]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[98] = + acadoWorkspace.A01[392]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[393]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[394]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[395]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[99] = + acadoWorkspace.A01[396]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[397]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[398]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[399]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[100] = + acadoWorkspace.A01[400]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[401]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[402]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[403]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[101] = + acadoWorkspace.A01[404]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[405]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[406]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[407]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[102] = + acadoWorkspace.A01[408]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[409]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[410]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[411]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[103] = + acadoWorkspace.A01[412]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[413]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[414]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[415]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[104] = + acadoWorkspace.A01[416]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[417]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[418]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[419]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[105] = + acadoWorkspace.A01[420]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[421]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[422]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[423]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[106] = + acadoWorkspace.A01[424]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[425]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[426]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[427]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[107] = + acadoWorkspace.A01[428]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[429]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[430]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[431]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[108] = + acadoWorkspace.A01[432]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[433]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[434]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[435]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[109] = + acadoWorkspace.A01[436]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[437]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[438]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[439]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[110] = + acadoWorkspace.A01[440]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[441]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[442]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[443]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[111] = + acadoWorkspace.A01[444]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[445]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[446]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[447]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[112] = + acadoWorkspace.A01[448]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[449]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[450]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[451]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[113] = + acadoWorkspace.A01[452]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[453]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[454]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[455]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[114] = + acadoWorkspace.A01[456]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[457]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[458]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[459]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[115] = + acadoWorkspace.A01[460]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[461]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[462]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[463]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[116] = + acadoWorkspace.A01[464]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[465]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[466]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[467]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[117] = + acadoWorkspace.A01[468]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[469]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[470]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[471]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[118] = + acadoWorkspace.A01[472]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[473]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[474]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[475]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[119] = + acadoWorkspace.A01[476]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[477]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[478]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[479]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[120] = + acadoWorkspace.A01[480]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[481]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[482]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[483]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[121] = + acadoWorkspace.A01[484]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[485]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[486]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[487]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[122] = + acadoWorkspace.A01[488]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[489]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[490]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[491]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[123] = + acadoWorkspace.A01[492]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[493]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[494]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[495]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[124] = + acadoWorkspace.A01[496]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[497]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[498]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[499]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[125] = + acadoWorkspace.A01[500]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[501]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[502]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[503]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[126] = + acadoWorkspace.A01[504]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[505]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[506]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[507]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[127] = + acadoWorkspace.A01[508]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[509]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[510]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[511]*acadoWorkspace.Dx0[3];
for (lRun1 = 0; lRun1 < 128; ++lRun1)
acadoWorkspace.lbA[lRun1] -= acadoWorkspace.pacA01Dx0[lRun1];


for (lRun1 = 0; lRun1 < 128; ++lRun1)
acadoWorkspace.ubA[lRun1] -= acadoWorkspace.pacA01Dx0[lRun1];


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
for (index = 0; index < 16; ++index)
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
for (index = 0; index < 16; ++index)
{
acadoVariables.x[index * 4] = acadoVariables.x[index * 4 + 4];
acadoVariables.x[index * 4 + 1] = acadoVariables.x[index * 4 + 5];
acadoVariables.x[index * 4 + 2] = acadoVariables.x[index * 4 + 6];
acadoVariables.x[index * 4 + 3] = acadoVariables.x[index * 4 + 7];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[64] = xEnd[0];
acadoVariables.x[65] = xEnd[1];
acadoVariables.x[66] = xEnd[2];
acadoVariables.x[67] = xEnd[3];
}
else if (strategy == 2) 
{
state[0] = acadoVariables.x[64];
state[1] = acadoVariables.x[65];
state[2] = acadoVariables.x[66];
state[3] = acadoVariables.x[67];
if (uEnd != 0)
{
state[28] = uEnd[0];
state[29] = uEnd[1];
}
else
{
state[28] = acadoVariables.u[30];
state[29] = acadoVariables.u[31];
}
state[30] = acadoVariables.od[512];
state[31] = acadoVariables.od[513];
state[32] = acadoVariables.od[514];
state[33] = acadoVariables.od[515];
state[34] = acadoVariables.od[516];
state[35] = acadoVariables.od[517];
state[36] = acadoVariables.od[518];
state[37] = acadoVariables.od[519];
state[38] = acadoVariables.od[520];
state[39] = acadoVariables.od[521];
state[40] = acadoVariables.od[522];
state[41] = acadoVariables.od[523];
state[42] = acadoVariables.od[524];
state[43] = acadoVariables.od[525];
state[44] = acadoVariables.od[526];
state[45] = acadoVariables.od[527];
state[46] = acadoVariables.od[528];
state[47] = acadoVariables.od[529];
state[48] = acadoVariables.od[530];
state[49] = acadoVariables.od[531];
state[50] = acadoVariables.od[532];
state[51] = acadoVariables.od[533];
state[52] = acadoVariables.od[534];
state[53] = acadoVariables.od[535];
state[54] = acadoVariables.od[536];
state[55] = acadoVariables.od[537];
state[56] = acadoVariables.od[538];
state[57] = acadoVariables.od[539];
state[58] = acadoVariables.od[540];
state[59] = acadoVariables.od[541];
state[60] = acadoVariables.od[542];
state[61] = acadoVariables.od[543];

acado_integrate(state, 1);

acadoVariables.x[64] = state[0];
acadoVariables.x[65] = state[1];
acadoVariables.x[66] = state[2];
acadoVariables.x[67] = state[3];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 15; ++index)
{
acadoVariables.u[index * 2] = acadoVariables.u[index * 2 + 2];
acadoVariables.u[index * 2 + 1] = acadoVariables.u[index * 2 + 3];
}

if (uEnd != 0)
{
acadoVariables.u[30] = uEnd[0];
acadoVariables.u[31] = uEnd[1];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19] + acadoWorkspace.g[20]*acadoWorkspace.x[20] + acadoWorkspace.g[21]*acadoWorkspace.x[21] + acadoWorkspace.g[22]*acadoWorkspace.x[22] + acadoWorkspace.g[23]*acadoWorkspace.x[23] + acadoWorkspace.g[24]*acadoWorkspace.x[24] + acadoWorkspace.g[25]*acadoWorkspace.x[25] + acadoWorkspace.g[26]*acadoWorkspace.x[26] + acadoWorkspace.g[27]*acadoWorkspace.x[27] + acadoWorkspace.g[28]*acadoWorkspace.x[28] + acadoWorkspace.g[29]*acadoWorkspace.x[29] + acadoWorkspace.g[30]*acadoWorkspace.x[30] + acadoWorkspace.g[31]*acadoWorkspace.x[31];
kkt = fabs( kkt );
for (index = 0; index < 32; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
for (index = 0; index < 128; ++index)
{
prd = acadoWorkspace.y[index + 32];
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
/** Row vector of size: 7 */
real_t tmpDy[ 7 ];

/** Row vector of size: 4 */
real_t tmpDyN[ 4 ];

for (lRun1 = 0; lRun1 < 16; ++lRun1)
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
acadoWorkspace.objValueIn[0] = acadoVariables.x[64];
acadoWorkspace.objValueIn[1] = acadoVariables.x[65];
acadoWorkspace.objValueIn[2] = acadoVariables.x[66];
acadoWorkspace.objValueIn[3] = acadoVariables.x[67];
acadoWorkspace.objValueIn[4] = acadoVariables.od[512];
acadoWorkspace.objValueIn[5] = acadoVariables.od[513];
acadoWorkspace.objValueIn[6] = acadoVariables.od[514];
acadoWorkspace.objValueIn[7] = acadoVariables.od[515];
acadoWorkspace.objValueIn[8] = acadoVariables.od[516];
acadoWorkspace.objValueIn[9] = acadoVariables.od[517];
acadoWorkspace.objValueIn[10] = acadoVariables.od[518];
acadoWorkspace.objValueIn[11] = acadoVariables.od[519];
acadoWorkspace.objValueIn[12] = acadoVariables.od[520];
acadoWorkspace.objValueIn[13] = acadoVariables.od[521];
acadoWorkspace.objValueIn[14] = acadoVariables.od[522];
acadoWorkspace.objValueIn[15] = acadoVariables.od[523];
acadoWorkspace.objValueIn[16] = acadoVariables.od[524];
acadoWorkspace.objValueIn[17] = acadoVariables.od[525];
acadoWorkspace.objValueIn[18] = acadoVariables.od[526];
acadoWorkspace.objValueIn[19] = acadoVariables.od[527];
acadoWorkspace.objValueIn[20] = acadoVariables.od[528];
acadoWorkspace.objValueIn[21] = acadoVariables.od[529];
acadoWorkspace.objValueIn[22] = acadoVariables.od[530];
acadoWorkspace.objValueIn[23] = acadoVariables.od[531];
acadoWorkspace.objValueIn[24] = acadoVariables.od[532];
acadoWorkspace.objValueIn[25] = acadoVariables.od[533];
acadoWorkspace.objValueIn[26] = acadoVariables.od[534];
acadoWorkspace.objValueIn[27] = acadoVariables.od[535];
acadoWorkspace.objValueIn[28] = acadoVariables.od[536];
acadoWorkspace.objValueIn[29] = acadoVariables.od[537];
acadoWorkspace.objValueIn[30] = acadoVariables.od[538];
acadoWorkspace.objValueIn[31] = acadoVariables.od[539];
acadoWorkspace.objValueIn[32] = acadoVariables.od[540];
acadoWorkspace.objValueIn[33] = acadoVariables.od[541];
acadoWorkspace.objValueIn[34] = acadoVariables.od[542];
acadoWorkspace.objValueIn[35] = acadoVariables.od[543];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1] - acadoVariables.yN[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2] - acadoVariables.yN[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3] - acadoVariables.yN[3];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 16; ++lRun1)
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

