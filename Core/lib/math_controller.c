#include "math_controller.h"
#include "rt_heap.h"
#include "stdlib.h"


/**
* @brief sign function
* @param a variable
* @retval its sign
*/
int sgn(float x)
{
	if(x > 0) return 1;
	else if(x < 0) return -1;
	else return 0;
}

/**
* @brief satuation function
* @param a variable, lower_bound, upper_bound
* @retval bounded variable
*/
float sat(float x, float lower_bound, float upper_bound)
{
	if(lower_bound > upper_bound) return 0;
	if(x > upper_bound) return upper_bound;
	else if(x < lower_bound) return lower_bound;
	else return x;
}

/**
* @brief sgn-like function
* @param a variable, dead 
* @retval processed variable
*/
float sgn_like(float x, float d)
{
	if(fabs(x) >= d) return sgn(x);
	else return x/d;
}

void mat_mul_33(float *src1, float *src2, float *dst)
{
	int i,j,k;
	float dst_tmp[9];
	memset(dst_tmp, 0, 9*sizeof(float));
	for(i=0;i<3;++i) {
		for(j=0;j<3;++j) {
			for(k=0;k<3;++k) {
				dst_tmp[i*3+j] += src1[i*3+k] * src2[k*3+j];
			}
		}
	}
	memcpy(dst, dst_tmp, 9*sizeof(float));
}

void mat_mul_31(float *src1, float *src2, float *dst)
{
	int i,j,k;
	float dst_tmp[3];
	memset(dst_tmp, 0, 3*sizeof(float));
	for(i=0;i<3;++i) {
		for(j=0;j<3;++j) {
			dst_tmp[i] += src1[i*3+k] * src2[j];
		}
	}
	memcpy(dst, dst_tmp, 3*sizeof(float));
}

void mat_scale_33(float *src, float scale, float *dst)
{
	int i,j;
	float dst_tmp[9];
	memset(dst_tmp, 0, 9*sizeof(float));

	for(i=0;i<3;++i) {
		for(j=0;j<3;++j) {
				dst_tmp[i*3+j] = src[i*3+j] * scale;
		}
	}
	memcpy(dst, dst_tmp, 9*sizeof(float));

}

void mat_scale_31(float *src, float scale, float *dst)
{
	int i;
	float dst_tmp[3];
	memset(dst_tmp, 0, 3*sizeof(float));

	for(i=0;i<3;++i) {
			dst_tmp[i] = src[i] * scale;
	}
	memcpy(dst, dst_tmp, 3*sizeof(float));
}

void mat_add_33(float *src1, float *src2, float *dst)
{
	int i,j;
	float dst_tmp[9];
	for(i=0;i<3;++i) {
		for(j=0;j<3;++j) {
			dst_tmp[i*3+j] = src1[i*3+j] + src2[i*3+j];
		}
	}
	memcpy(dst, dst_tmp, 9*sizeof(float));
}

void mat_add_31(float *src1, float *src2, float *dst)
{
	int i,j;
	float dst_tmp[3];
	for(i=0;i<3;++i) {
		dst_tmp[i] = src1[i] + src2[i];
	}
	memcpy(dst, dst_tmp, 3*sizeof(float));
}



void mat_sub_33(float *src1, float *src2, float *dst)
{
	int i,j;
	float dst_tmp[9];
	for(i=0;i<3;++i) {
		for(j=0;j<3;++j) {
			dst_tmp[i*3+j] = src1[i*3+j] - src2[i*3+j];
		}
	}
	memcpy(dst, dst_tmp, 9*sizeof(float));
}

void TransitionDiscrete33(float *A, float *F, float dt)
{
	float I[9] = {1,0,0,0,1,0,0,0,1};
	float At[9];
	float At2[9];
	
	mat_scale_33(A, dt, At);
	mat_mul_33(At, At, At2);
	mat_scale_33(At2, 0.5, At2);
	memset(F, 0, 9*sizeof(float));
	
	mat_add_33(F, I, F);
	mat_add_33(F, At, F);
	mat_add_33(F, At2, F);
}





void GainDiscrete31(float *B, float *G, float dt)
{
	mat_scale_31(B, dt, G);
}

void matexp3(matf32_t *src, matf32_t *dst)
{
	matf32_t taylor0;
	matf32_t taylor1;
	matf32_t taylor2;
	
	float taylor0_data[9] = {1,0,0,0,1,0,0,0,1};
	float taylor1_data[9];
	float taylor2_data[9];
	
	memset(taylor1_data, 0, 9);
	memset(taylor2_data, 0, 9);
	
	arm_mat_init_f32(&taylor0, 3, 3, taylor0_data);
	arm_mat_init_f32(&taylor1, 3, 3, taylor1_data);
	arm_mat_init_f32(&taylor2, 3, 3, taylor2_data);
	
	arm_mat_cmplx_mult_f32(src, src, &taylor2);
	arm_mat_cmplx_mult_f32(src, &taylor0, &taylor1);
	
	memset(dst->pData, 0, 9);
	
	arm_mat_add_f32(dst, &taylor0, dst);
	arm_mat_add_f32(dst, &taylor1, dst);
	arm_mat_add_f32(dst, &taylor2, dst);
	
}

