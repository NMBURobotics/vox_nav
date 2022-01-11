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


void acado_rhs(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 4;
/* Vector of auxiliary variables; number of elements: 5. */
real_t* a = acadoWorkspace.rhs_aux;

/* Compute intermediate quantities: */
a[0] = (tan(u[1]));
a[1] = (atan(((real_t)(5.0000000000000000e-01)*a[0])));
a[2] = (cos((xd[2]+a[1])));
a[3] = (sin((xd[2]+a[1])));
a[4] = (sin(a[1]));

/* Compute outputs: */
out[0] = (xd[3]*a[2]);
out[1] = (xd[3]*a[3]);
out[2] = ((xd[3]/(real_t)(6.6000000000000003e-01))*a[4]);
out[3] = u[0];
}



void acado_diffs(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 4;
/* Vector of auxiliary variables; number of elements: 17. */
real_t* a = acadoWorkspace.rhs_aux;

/* Compute intermediate quantities: */
a[0] = (tan(u[1]));
a[1] = (atan(((real_t)(5.0000000000000000e-01)*a[0])));
a[2] = ((real_t)(-1.0000000000000000e+00)*(sin((xd[2]+a[1]))));
a[3] = (cos((xd[2]+a[1])));
a[4] = ((real_t)(1.0000000000000000e+00)/(pow((cos(u[1])),2)));
a[5] = ((real_t)(1.0000000000000000e+00)/((real_t)(1.0000000000000000e+00)+(pow(((real_t)(5.0000000000000000e-01)*a[0]),2))));
a[6] = (((real_t)(5.0000000000000000e-01)*a[4])*a[5]);
a[7] = (a[6]*a[2]);
a[8] = (cos((xd[2]+a[1])));
a[9] = (sin((xd[2]+a[1])));
a[10] = (((real_t)(5.0000000000000000e-01)*a[4])*a[5]);
a[11] = (a[10]*a[8]);
a[12] = ((real_t)(1.0000000000000000e+00)/(real_t)(6.6000000000000003e-01));
a[13] = (sin(a[1]));
a[14] = (((real_t)(5.0000000000000000e-01)*a[4])*a[5]);
a[15] = (cos(a[1]));
a[16] = (a[14]*a[15]);

/* Compute outputs: */
out[0] = (real_t)(0.0000000000000000e+00);
out[1] = (real_t)(0.0000000000000000e+00);
out[2] = (xd[3]*a[2]);
out[3] = a[3];
out[4] = (real_t)(0.0000000000000000e+00);
out[5] = (xd[3]*a[7]);
out[6] = (real_t)(0.0000000000000000e+00);
out[7] = (real_t)(0.0000000000000000e+00);
out[8] = (xd[3]*a[8]);
out[9] = a[9];
out[10] = (real_t)(0.0000000000000000e+00);
out[11] = (xd[3]*a[11]);
out[12] = (real_t)(0.0000000000000000e+00);
out[13] = (real_t)(0.0000000000000000e+00);
out[14] = (real_t)(0.0000000000000000e+00);
out[15] = (a[12]*a[13]);
out[16] = (real_t)(0.0000000000000000e+00);
out[17] = ((xd[3]/(real_t)(6.6000000000000003e-01))*a[16]);
out[18] = (real_t)(0.0000000000000000e+00);
out[19] = (real_t)(0.0000000000000000e+00);
out[20] = (real_t)(0.0000000000000000e+00);
out[21] = (real_t)(0.0000000000000000e+00);
out[22] = (real_t)(1.0000000000000000e+00);
out[23] = (real_t)(0.0000000000000000e+00);
}



void acado_solve_dim8_triangular( real_t* const A, real_t* const b )
{

b[7] = b[7]/A[63];
b[6] -= + A[55]*b[7];
b[6] = b[6]/A[54];
b[5] -= + A[47]*b[7];
b[5] -= + A[46]*b[6];
b[5] = b[5]/A[45];
b[4] -= + A[39]*b[7];
b[4] -= + A[38]*b[6];
b[4] -= + A[37]*b[5];
b[4] = b[4]/A[36];
b[3] -= + A[31]*b[7];
b[3] -= + A[30]*b[6];
b[3] -= + A[29]*b[5];
b[3] -= + A[28]*b[4];
b[3] = b[3]/A[27];
b[2] -= + A[23]*b[7];
b[2] -= + A[22]*b[6];
b[2] -= + A[21]*b[5];
b[2] -= + A[20]*b[4];
b[2] -= + A[19]*b[3];
b[2] = b[2]/A[18];
b[1] -= + A[15]*b[7];
b[1] -= + A[14]*b[6];
b[1] -= + A[13]*b[5];
b[1] -= + A[12]*b[4];
b[1] -= + A[11]*b[3];
b[1] -= + A[10]*b[2];
b[1] = b[1]/A[9];
b[0] -= + A[7]*b[7];
b[0] -= + A[6]*b[6];
b[0] -= + A[5]*b[5];
b[0] -= + A[4]*b[4];
b[0] -= + A[3]*b[3];
b[0] -= + A[2]*b[2];
b[0] -= + A[1]*b[1];
b[0] = b[0]/A[0];
}

real_t acado_solve_dim8_system( real_t* const A, real_t* const b, int* const rk_perm )
{
real_t det;

int i;
int j;
int k;

int indexMax;

int intSwap;

real_t valueMax;

real_t temp;

for (i = 0; i < 8; ++i)
{
rk_perm[i] = i;
}
det = 1.0000000000000000e+00;
for( i=0; i < (7); i++ ) {
	indexMax = i;
	valueMax = fabs(A[i*8+i]);
	for( j=(i+1); j < 8; j++ ) {
		temp = fabs(A[j*8+i]);
		if( temp > valueMax ) {
			indexMax = j;
			valueMax = temp;
		}
	}
	if( indexMax > i ) {
for (k = 0; k < 8; ++k)
{
	acadoWorkspace.rk_dim8_swap = A[i*8+k];
	A[i*8+k] = A[indexMax*8+k];
	A[indexMax*8+k] = acadoWorkspace.rk_dim8_swap;
}
	acadoWorkspace.rk_dim8_swap = b[i];
	b[i] = b[indexMax];
	b[indexMax] = acadoWorkspace.rk_dim8_swap;
	intSwap = rk_perm[i];
	rk_perm[i] = rk_perm[indexMax];
	rk_perm[indexMax] = intSwap;
	}
	det *= A[i*8+i];
	for( j=i+1; j < 8; j++ ) {
		A[j*8+i] = -A[j*8+i]/A[i*8+i];
		for( k=i+1; k < 8; k++ ) {
			A[j*8+k] += A[j*8+i] * A[i*8+k];
		}
		b[j] += A[j*8+i] * b[i];
	}
}
det *= A[63];
det = fabs(det);
acado_solve_dim8_triangular( A, b );
return det;
}

void acado_solve_dim8_system_reuse( real_t* const A, real_t* const b, int* const rk_perm )
{

acadoWorkspace.rk_dim8_bPerm[0] = b[rk_perm[0]];
acadoWorkspace.rk_dim8_bPerm[1] = b[rk_perm[1]];
acadoWorkspace.rk_dim8_bPerm[2] = b[rk_perm[2]];
acadoWorkspace.rk_dim8_bPerm[3] = b[rk_perm[3]];
acadoWorkspace.rk_dim8_bPerm[4] = b[rk_perm[4]];
acadoWorkspace.rk_dim8_bPerm[5] = b[rk_perm[5]];
acadoWorkspace.rk_dim8_bPerm[6] = b[rk_perm[6]];
acadoWorkspace.rk_dim8_bPerm[7] = b[rk_perm[7]];
acadoWorkspace.rk_dim8_bPerm[1] += A[8]*acadoWorkspace.rk_dim8_bPerm[0];

acadoWorkspace.rk_dim8_bPerm[2] += A[16]*acadoWorkspace.rk_dim8_bPerm[0];
acadoWorkspace.rk_dim8_bPerm[2] += A[17]*acadoWorkspace.rk_dim8_bPerm[1];

acadoWorkspace.rk_dim8_bPerm[3] += A[24]*acadoWorkspace.rk_dim8_bPerm[0];
acadoWorkspace.rk_dim8_bPerm[3] += A[25]*acadoWorkspace.rk_dim8_bPerm[1];
acadoWorkspace.rk_dim8_bPerm[3] += A[26]*acadoWorkspace.rk_dim8_bPerm[2];

acadoWorkspace.rk_dim8_bPerm[4] += A[32]*acadoWorkspace.rk_dim8_bPerm[0];
acadoWorkspace.rk_dim8_bPerm[4] += A[33]*acadoWorkspace.rk_dim8_bPerm[1];
acadoWorkspace.rk_dim8_bPerm[4] += A[34]*acadoWorkspace.rk_dim8_bPerm[2];
acadoWorkspace.rk_dim8_bPerm[4] += A[35]*acadoWorkspace.rk_dim8_bPerm[3];

acadoWorkspace.rk_dim8_bPerm[5] += A[40]*acadoWorkspace.rk_dim8_bPerm[0];
acadoWorkspace.rk_dim8_bPerm[5] += A[41]*acadoWorkspace.rk_dim8_bPerm[1];
acadoWorkspace.rk_dim8_bPerm[5] += A[42]*acadoWorkspace.rk_dim8_bPerm[2];
acadoWorkspace.rk_dim8_bPerm[5] += A[43]*acadoWorkspace.rk_dim8_bPerm[3];
acadoWorkspace.rk_dim8_bPerm[5] += A[44]*acadoWorkspace.rk_dim8_bPerm[4];

acadoWorkspace.rk_dim8_bPerm[6] += A[48]*acadoWorkspace.rk_dim8_bPerm[0];
acadoWorkspace.rk_dim8_bPerm[6] += A[49]*acadoWorkspace.rk_dim8_bPerm[1];
acadoWorkspace.rk_dim8_bPerm[6] += A[50]*acadoWorkspace.rk_dim8_bPerm[2];
acadoWorkspace.rk_dim8_bPerm[6] += A[51]*acadoWorkspace.rk_dim8_bPerm[3];
acadoWorkspace.rk_dim8_bPerm[6] += A[52]*acadoWorkspace.rk_dim8_bPerm[4];
acadoWorkspace.rk_dim8_bPerm[6] += A[53]*acadoWorkspace.rk_dim8_bPerm[5];

acadoWorkspace.rk_dim8_bPerm[7] += A[56]*acadoWorkspace.rk_dim8_bPerm[0];
acadoWorkspace.rk_dim8_bPerm[7] += A[57]*acadoWorkspace.rk_dim8_bPerm[1];
acadoWorkspace.rk_dim8_bPerm[7] += A[58]*acadoWorkspace.rk_dim8_bPerm[2];
acadoWorkspace.rk_dim8_bPerm[7] += A[59]*acadoWorkspace.rk_dim8_bPerm[3];
acadoWorkspace.rk_dim8_bPerm[7] += A[60]*acadoWorkspace.rk_dim8_bPerm[4];
acadoWorkspace.rk_dim8_bPerm[7] += A[61]*acadoWorkspace.rk_dim8_bPerm[5];
acadoWorkspace.rk_dim8_bPerm[7] += A[62]*acadoWorkspace.rk_dim8_bPerm[6];


acado_solve_dim8_triangular( A, acadoWorkspace.rk_dim8_bPerm );
b[0] = acadoWorkspace.rk_dim8_bPerm[0];
b[1] = acadoWorkspace.rk_dim8_bPerm[1];
b[2] = acadoWorkspace.rk_dim8_bPerm[2];
b[3] = acadoWorkspace.rk_dim8_bPerm[3];
b[4] = acadoWorkspace.rk_dim8_bPerm[4];
b[5] = acadoWorkspace.rk_dim8_bPerm[5];
b[6] = acadoWorkspace.rk_dim8_bPerm[6];
b[7] = acadoWorkspace.rk_dim8_bPerm[7];
}



/** Matrix of size: 2 x 2 (row major format) */
static const real_t acado_Ah_mat[ 4 ] = 
{ 1.0416666666666668e-02, -2.0833333333333333e-03, 
1.8750000000000003e-02, 6.2500000000000003e-03 };


/* Fixed step size:0.025 */
int acado_integrate( real_t* const rk_eta, int resetIntegrator )
{
int error;

int i;
int j;
int k;
int run;
int run1;
int tmp_index1;
int tmp_index2;

real_t det;

acadoWorkspace.rk_ttt = 0.0000000000000000e+00;
acadoWorkspace.rk_xxx[4] = rk_eta[28];
acadoWorkspace.rk_xxx[5] = rk_eta[29];

for (run = 0; run < 4; ++run)
{
if( run > 0 ) {
for (i = 0; i < 4; ++i)
{
acadoWorkspace.rk_diffsPrev2[i * 6] = rk_eta[i * 4 + 4];
acadoWorkspace.rk_diffsPrev2[i * 6 + 1] = rk_eta[i * 4 + 5];
acadoWorkspace.rk_diffsPrev2[i * 6 + 2] = rk_eta[i * 4 + 6];
acadoWorkspace.rk_diffsPrev2[i * 6 + 3] = rk_eta[i * 4 + 7];
acadoWorkspace.rk_diffsPrev2[i * 6 + 4] = rk_eta[i * 2 + 20];
acadoWorkspace.rk_diffsPrev2[i * 6 + 5] = rk_eta[i * 2 + 21];
}
}
if( resetIntegrator ) {
for (i = 0; i < 1; ++i)
{
for (run1 = 0; run1 < 2; ++run1)
{
for (j = 0; j < 4; ++j)
{
acadoWorkspace.rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_kkk[tmp_index1 * 2];
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_kkk[tmp_index1 * 2 + 1];
}
acado_diffs( acadoWorkspace.rk_xxx, &(acadoWorkspace.rk_diffsTemp2[ run1 * 24 ]) );
for (j = 0; j < 4; ++j)
{
tmp_index1 = (run1 * 4) + (j);
acadoWorkspace.rk_A[tmp_index1 * 8] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 24) + (j * 6)];
acadoWorkspace.rk_A[tmp_index1 * 8 + 1] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 24) + (j * 6 + 1)];
acadoWorkspace.rk_A[tmp_index1 * 8 + 2] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 24) + (j * 6 + 2)];
acadoWorkspace.rk_A[tmp_index1 * 8 + 3] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 24) + (j * 6 + 3)];
if( 0 == run1 ) acadoWorkspace.rk_A[(tmp_index1 * 8) + (j)] -= 1.0000000000000000e+00;
acadoWorkspace.rk_A[tmp_index1 * 8 + 4] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 24) + (j * 6)];
acadoWorkspace.rk_A[tmp_index1 * 8 + 5] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 24) + (j * 6 + 1)];
acadoWorkspace.rk_A[tmp_index1 * 8 + 6] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 24) + (j * 6 + 2)];
acadoWorkspace.rk_A[tmp_index1 * 8 + 7] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 24) + (j * 6 + 3)];
if( 1 == run1 ) acadoWorkspace.rk_A[(tmp_index1 * 8) + (j + 4)] -= 1.0000000000000000e+00;
}
acado_rhs( acadoWorkspace.rk_xxx, acadoWorkspace.rk_rhsTemp );
acadoWorkspace.rk_b[run1 * 4] = acadoWorkspace.rk_kkk[run1] - acadoWorkspace.rk_rhsTemp[0];
acadoWorkspace.rk_b[run1 * 4 + 1] = acadoWorkspace.rk_kkk[run1 + 2] - acadoWorkspace.rk_rhsTemp[1];
acadoWorkspace.rk_b[run1 * 4 + 2] = acadoWorkspace.rk_kkk[run1 + 4] - acadoWorkspace.rk_rhsTemp[2];
acadoWorkspace.rk_b[run1 * 4 + 3] = acadoWorkspace.rk_kkk[run1 + 6] - acadoWorkspace.rk_rhsTemp[3];
}
det = acado_solve_dim8_system( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim8_perm );
for (j = 0; j < 2; ++j)
{
acadoWorkspace.rk_kkk[j] += acadoWorkspace.rk_b[j * 4];
acadoWorkspace.rk_kkk[j + 2] += acadoWorkspace.rk_b[j * 4 + 1];
acadoWorkspace.rk_kkk[j + 4] += acadoWorkspace.rk_b[j * 4 + 2];
acadoWorkspace.rk_kkk[j + 6] += acadoWorkspace.rk_b[j * 4 + 3];
}
}
}
for (i = 0; i < 5; ++i)
{
for (run1 = 0; run1 < 2; ++run1)
{
for (j = 0; j < 4; ++j)
{
acadoWorkspace.rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_kkk[tmp_index1 * 2];
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_kkk[tmp_index1 * 2 + 1];
}
acado_rhs( acadoWorkspace.rk_xxx, acadoWorkspace.rk_rhsTemp );
acadoWorkspace.rk_b[run1 * 4] = acadoWorkspace.rk_kkk[run1] - acadoWorkspace.rk_rhsTemp[0];
acadoWorkspace.rk_b[run1 * 4 + 1] = acadoWorkspace.rk_kkk[run1 + 2] - acadoWorkspace.rk_rhsTemp[1];
acadoWorkspace.rk_b[run1 * 4 + 2] = acadoWorkspace.rk_kkk[run1 + 4] - acadoWorkspace.rk_rhsTemp[2];
acadoWorkspace.rk_b[run1 * 4 + 3] = acadoWorkspace.rk_kkk[run1 + 6] - acadoWorkspace.rk_rhsTemp[3];
}
acado_solve_dim8_system_reuse( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim8_perm );
for (j = 0; j < 2; ++j)
{
acadoWorkspace.rk_kkk[j] += acadoWorkspace.rk_b[j * 4];
acadoWorkspace.rk_kkk[j + 2] += acadoWorkspace.rk_b[j * 4 + 1];
acadoWorkspace.rk_kkk[j + 4] += acadoWorkspace.rk_b[j * 4 + 2];
acadoWorkspace.rk_kkk[j + 6] += acadoWorkspace.rk_b[j * 4 + 3];
}
}
for (run1 = 0; run1 < 2; ++run1)
{
for (j = 0; j < 4; ++j)
{
acadoWorkspace.rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_kkk[tmp_index1 * 2];
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_kkk[tmp_index1 * 2 + 1];
}
acado_diffs( acadoWorkspace.rk_xxx, &(acadoWorkspace.rk_diffsTemp2[ run1 * 24 ]) );
for (j = 0; j < 4; ++j)
{
tmp_index1 = (run1 * 4) + (j);
acadoWorkspace.rk_A[tmp_index1 * 8] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 24) + (j * 6)];
acadoWorkspace.rk_A[tmp_index1 * 8 + 1] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 24) + (j * 6 + 1)];
acadoWorkspace.rk_A[tmp_index1 * 8 + 2] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 24) + (j * 6 + 2)];
acadoWorkspace.rk_A[tmp_index1 * 8 + 3] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 24) + (j * 6 + 3)];
if( 0 == run1 ) acadoWorkspace.rk_A[(tmp_index1 * 8) + (j)] -= 1.0000000000000000e+00;
acadoWorkspace.rk_A[tmp_index1 * 8 + 4] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 24) + (j * 6)];
acadoWorkspace.rk_A[tmp_index1 * 8 + 5] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 24) + (j * 6 + 1)];
acadoWorkspace.rk_A[tmp_index1 * 8 + 6] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 24) + (j * 6 + 2)];
acadoWorkspace.rk_A[tmp_index1 * 8 + 7] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 24) + (j * 6 + 3)];
if( 1 == run1 ) acadoWorkspace.rk_A[(tmp_index1 * 8) + (j + 4)] -= 1.0000000000000000e+00;
}
}
for (run1 = 0; run1 < 4; ++run1)
{
for (i = 0; i < 2; ++i)
{
acadoWorkspace.rk_b[i * 4] = - acadoWorkspace.rk_diffsTemp2[(i * 24) + (run1)];
acadoWorkspace.rk_b[i * 4 + 1] = - acadoWorkspace.rk_diffsTemp2[(i * 24) + (run1 + 6)];
acadoWorkspace.rk_b[i * 4 + 2] = - acadoWorkspace.rk_diffsTemp2[(i * 24) + (run1 + 12)];
acadoWorkspace.rk_b[i * 4 + 3] = - acadoWorkspace.rk_diffsTemp2[(i * 24) + (run1 + 18)];
}
if( 0 == run1 ) {
det = acado_solve_dim8_system( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim8_perm );
}
 else {
acado_solve_dim8_system_reuse( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim8_perm );
}
for (i = 0; i < 2; ++i)
{
acadoWorkspace.rk_diffK[i] = acadoWorkspace.rk_b[i * 4];
acadoWorkspace.rk_diffK[i + 2] = acadoWorkspace.rk_b[i * 4 + 1];
acadoWorkspace.rk_diffK[i + 4] = acadoWorkspace.rk_b[i * 4 + 2];
acadoWorkspace.rk_diffK[i + 6] = acadoWorkspace.rk_b[i * 4 + 3];
}
for (i = 0; i < 4; ++i)
{
acadoWorkspace.rk_diffsNew2[(i * 6) + (run1)] = (i == run1-0);
acadoWorkspace.rk_diffsNew2[(i * 6) + (run1)] += + acadoWorkspace.rk_diffK[i * 2]*(real_t)1.8750000000000003e-02 + acadoWorkspace.rk_diffK[i * 2 + 1]*(real_t)6.2500000000000003e-03;
}
}
for (run1 = 0; run1 < 2; ++run1)
{
for (i = 0; i < 2; ++i)
{
for (j = 0; j < 4; ++j)
{
tmp_index1 = (i * 4) + (j);
tmp_index2 = (run1) + (j * 6);
acadoWorkspace.rk_b[tmp_index1] = - acadoWorkspace.rk_diffsTemp2[(i * 24) + (tmp_index2 + 4)];
}
}
acado_solve_dim8_system_reuse( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim8_perm );
for (i = 0; i < 2; ++i)
{
acadoWorkspace.rk_diffK[i] = acadoWorkspace.rk_b[i * 4];
acadoWorkspace.rk_diffK[i + 2] = acadoWorkspace.rk_b[i * 4 + 1];
acadoWorkspace.rk_diffK[i + 4] = acadoWorkspace.rk_b[i * 4 + 2];
acadoWorkspace.rk_diffK[i + 6] = acadoWorkspace.rk_b[i * 4 + 3];
}
for (i = 0; i < 4; ++i)
{
acadoWorkspace.rk_diffsNew2[(i * 6) + (run1 + 4)] = + acadoWorkspace.rk_diffK[i * 2]*(real_t)1.8750000000000003e-02 + acadoWorkspace.rk_diffK[i * 2 + 1]*(real_t)6.2500000000000003e-03;
}
}
rk_eta[0] += + acadoWorkspace.rk_kkk[0]*(real_t)1.8750000000000003e-02 + acadoWorkspace.rk_kkk[1]*(real_t)6.2500000000000003e-03;
rk_eta[1] += + acadoWorkspace.rk_kkk[2]*(real_t)1.8750000000000003e-02 + acadoWorkspace.rk_kkk[3]*(real_t)6.2500000000000003e-03;
rk_eta[2] += + acadoWorkspace.rk_kkk[4]*(real_t)1.8750000000000003e-02 + acadoWorkspace.rk_kkk[5]*(real_t)6.2500000000000003e-03;
rk_eta[3] += + acadoWorkspace.rk_kkk[6]*(real_t)1.8750000000000003e-02 + acadoWorkspace.rk_kkk[7]*(real_t)6.2500000000000003e-03;
if( run == 0 ) {
for (i = 0; i < 4; ++i)
{
for (j = 0; j < 4; ++j)
{
tmp_index2 = (j) + (i * 4);
rk_eta[tmp_index2 + 4] = acadoWorkspace.rk_diffsNew2[(i * 6) + (j)];
}
for (j = 0; j < 2; ++j)
{
tmp_index2 = (j) + (i * 2);
rk_eta[tmp_index2 + 20] = acadoWorkspace.rk_diffsNew2[(i * 6) + (j + 4)];
}
}
}
else {
for (i = 0; i < 4; ++i)
{
for (j = 0; j < 4; ++j)
{
tmp_index2 = (j) + (i * 4);
rk_eta[tmp_index2 + 4] = + acadoWorkspace.rk_diffsNew2[i * 6]*acadoWorkspace.rk_diffsPrev2[j];
rk_eta[tmp_index2 + 4] += + acadoWorkspace.rk_diffsNew2[i * 6 + 1]*acadoWorkspace.rk_diffsPrev2[j + 6];
rk_eta[tmp_index2 + 4] += + acadoWorkspace.rk_diffsNew2[i * 6 + 2]*acadoWorkspace.rk_diffsPrev2[j + 12];
rk_eta[tmp_index2 + 4] += + acadoWorkspace.rk_diffsNew2[i * 6 + 3]*acadoWorkspace.rk_diffsPrev2[j + 18];
}
for (j = 0; j < 2; ++j)
{
tmp_index2 = (j) + (i * 2);
rk_eta[tmp_index2 + 20] = acadoWorkspace.rk_diffsNew2[(i * 6) + (j + 4)];
rk_eta[tmp_index2 + 20] += + acadoWorkspace.rk_diffsNew2[i * 6]*acadoWorkspace.rk_diffsPrev2[j + 4];
rk_eta[tmp_index2 + 20] += + acadoWorkspace.rk_diffsNew2[i * 6 + 1]*acadoWorkspace.rk_diffsPrev2[j + 10];
rk_eta[tmp_index2 + 20] += + acadoWorkspace.rk_diffsNew2[i * 6 + 2]*acadoWorkspace.rk_diffsPrev2[j + 16];
rk_eta[tmp_index2 + 20] += + acadoWorkspace.rk_diffsNew2[i * 6 + 3]*acadoWorkspace.rk_diffsPrev2[j + 22];
}
}
}
resetIntegrator = 0;
acadoWorkspace.rk_ttt += 2.5000000000000000e-01;
}
for (i = 0; i < 4; ++i)
{
}
if( det < 1e-12 ) {
error = 2;
} else if( det < 1e-6 ) {
error = 1;
} else {
error = 0;
}
return error;
}



