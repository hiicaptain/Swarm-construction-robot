#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include "math_controller.h"

typedef struct 
{
	float x1;
	float x2;
	float x;
	float r;
	float h;
	float aim;
	float dt;
}td_t;

typedef struct
{
	float ref;
	float fdb;
	float err;
	float err_last;
	float kp;
	float ki;
	float kd;
  float err_sum;
  float err_max;
	float output;
	float output_max;
}pid_t;

typedef struct
{
	td_t td;
	pid_t pos_loop;
	pid_t spd_loop;
	float nonlinear_torque;
	float output;
	float output_max;
}cascade_pid_t;

typedef struct
{
	float x1_ref;
	float x1_fdb;
	float x2_ref;
	float x2_fdb;
	float x1_err;
	float x2_err;
	float k1;
	float k2;
	float alpha1;
	float alpha2;
	float alpha3;
	float gamma1;
	float gamma2;
	float gamma3;
	float output;
	float output_max;
	float dt;
	td_t x1_td;
	td_t x2_td;
	unsigned char enable;
}mrac_2d_t;


typedef struct
{
	float v_hat;
	float v_hat_last;
	
	float tl_hat;
	float tl_hat_last;
	
	float v_err;
	
	float b;
	float f;
	float kv;
	float kt;
	float dt;
}tob_t;

typedef struct
{
	float theta;
	float theta_last;
	float delta_theta;
	float theta_inf;
	float omiga;
	int cylinder;
}encoder_t;

typedef struct
{
	td_t td;
	tob_t ob;
	float x1_err;
	float x2_err;
	float kp;
	float kd;
	float output;
	float output_max;
	
}flpd2d_t;


typedef struct
{
	tob_t ob;
	pid_t pid;
	float output;
	float output_max;
}flpid_t;


typedef struct
{
	tob_t ob;
	cascade_pid_t cpid;
	float output;
	float output_max;
}flcpid2d_t;


typedef struct
{
	td_t planner;
	
	float xb_ref;
	float xb_fdb;
	float phi_err;
	
	float theta1_ref;
	float theta2_ref;
	float theta1;
	float theta2;
	
	flpd2d_t theta1_ctrl;
	flpd2d_t theta2_ctrl;
	
	float theta1_offset;
	float theta2_offset;
	float length;
}bounce_wheel_t;


#define DEFAULT_TD_T \
{0,0,0,0,0,0,0}

#define DEFAULT_PID_T \
{0,0,0,0,0,0,0,0,0,0,0}   

#define MOTOR1_SET_POS     0.73f
#define MOTOR1_RESET_POS  -0.3f

#define MOTOR2_SET_POS     0.72f
#define MOTOR2_RESET_POS   0.0128f

#define BOUNCE_WHEEL_SET_POS 1
#define BOUNCE_WHEEL_RESET_POS 2


void td_init(td_t *ptd, float r, float h, float dt);
void pid_init(pid_t *pid, float kp, float ki, float kd, float outmax, float err_max);
void td_function(td_t *ptd);
void pid_calc(pid_t *pid, float ref, float fdb);
void encoder_process(encoder_t *encoder, float theta_now, float dt);
void ramp(float* pvalue, float ref, float step);

void cascade_pid_init(cascade_pid_t *cpid, float kp1, float ki1, float kd1, float err_max1, float output_max1,
											float kp2, float ki2, float kd2, float err_max2, float output_max2,
											float r, float h, float dt);
void cascade_pid_calc(cascade_pid_t *cpid, float ref, float x1_fdb, float x2_fdbm, uint8_t is_enable);
void cascade_pid_calc_test(cascade_pid_t *cascade_pid, float ref, float x1_fdb, float x2_fdb, uint8_t type);
void mrac_2d_calc(mrac_2d_t *mrac, float ref, float x_fdb, float v_fdb);
											
void flpid_init(flpid_t *pc, float kp, float ki, float kd, float err_max, float output_max,
										float dt, float J, float gain);
void flpid_calc(flpid_t *pc, float x_ref, float x_fdb, uint8_t is_enable);

void flpd2d_init(flpd2d_t *pc, float kp, float kd, float r, float h, float dt, float J, float gain, float output_max);
void flpd2d_calc(flpd2d_t *pc, float x1_ref, float x1_fdb, float x2_fdb, uint8_t is_enable);

void flcpid2d_init(flcpid2d_t *pc, float kp1, float ki1, float kd1, float err_max1, float output_max1,
									 float kp2, float ki2, float kd2, float err_max2, float output_max2,
										float r, float h, float dt, float J, float gain);
void flcpid2d_calc(flcpid2d_t *pc, float x1_ref, float x1_fdb, float x2_fdb, uint8_t is_enable);

void BounceWheelInit(bounce_wheel_t *pw);
void BounceWheelControl(bounce_wheel_t *pw, uint8_t state, float r_set, float r_reset, 
												float theta1_m, float theta2_m, float omiga1_m, float omiga2_m, uint8_t is_enable);

void tob_calc(tob_t *tob, float fdb, float output);
void tob_init(tob_t *tob, float dt, float J, float gain);

#endif

