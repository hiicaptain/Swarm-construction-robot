#ifndef _MATH_CONTROLLER_
#define _MATH_CONTROLLER_

#include "main.h"
#include "math.h"
#include "arm_math.h"
#include "arm_const_structs.h"

typedef arm_matrix_instance_f32 matf32_t;

//typedef arm_mat_cmplx_mult_f32 matf32_mult;




int sgn(float x);
float sat(float x, float lower_bound, float upper_bound);
float sgn_like(float x, float d);

void TransitionDiscrete33(float *A, float *F, float dt);
void GainDiscrete31(float *B, float *G, float dt);

void mat_mul_33(float *src1, float *src2, float *dst);
void mat_mul_31(float *src1, float *src2, float *dst);
void mat_scale_33(float *src, float scale, float *dst);
void mat_scale_31(float *src, float scale, float *dst);
void mat_add_33(float *src1, float *src2, float *dst);
void mat_add_31(float *src1, float *src2, float *dst);
void mat_sub_33(float *src1, float *src2, float *dst);


#endif

