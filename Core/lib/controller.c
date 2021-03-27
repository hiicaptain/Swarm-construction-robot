#include "controller.h"
#include "math.h"
/**
* @brief TD struct init
* @param TD struct, initial value
* @retval None
*/
void td_init(td_t *ptd, float r, float h, float dt)
{
	ptd->h = h;
	ptd->r = r;
  ptd->dt = dt;
}

/**
* @brief TD algorithm
* @param TD struct
* @retval None
*/
void td_function(td_t *ptd)
{
	float d,d0,y,a0,a=0;
	ptd->x = ptd->x1 - ptd->aim;
	d = ptd->r*ptd->h;
	d0 = ptd->h*d;
	y = ptd->x + ptd->h*ptd->x2;
	a0 = sqrt(d*d + 8*ptd->r*fabs(y));
	if(fabs(y) > d0)
		a = ptd->x2 + (a0 - d)*sgn(y)/2;
	else
		a = ptd->x2 + y/ptd->h;
	if(fabs(a) > d)
		y = -ptd->r*sgn(a);
	else
		y = -ptd->r*a/d;
	ptd->x1 += ptd->dt*ptd->x2;
	ptd->x2 += ptd->dt*y;
}

void pid_init(pid_t *pid, float kp, float ki, float kd, float outmax, float err_max)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->output_max = outmax;
	pid->err_max = err_max;
}

/**
* @brief PID calculate 
* @param pid_struct, ref, fdb
* @retval None
*/
void pid_calc(pid_t *pid, float ref, float fdb)
{
	
	pid->ref = ref;
	pid->fdb = fdb;
	pid->err_last = pid->err;
	pid->err = pid->ref - pid->fdb; 
	pid->err_sum += pid->err;       
	pid->err_sum = sat(pid->err_sum, -pid->err_max, pid->err_max);
	pid->output = pid->kp * pid->err + pid->ki * pid->err_sum + pid->kd * (pid->err - pid->err_last);
	pid->output = sat(pid->output, -pid->output_max, pid->output_max);
}

void cascade_pid_init(cascade_pid_t *cpid, float kp1, float ki1, float kd1, float err_max1, float output_max1,
											float kp2, float ki2, float kd2, float err_max2, float output_max2,
											float r, float h, float dt)
{
	td_init(&cpid->td, r, h, dt);
	pid_init(&cpid->pos_loop, kp1, ki1, kd1, output_max1, err_max1);
	pid_init(&cpid->spd_loop, kp2, ki2, kd2, output_max2, err_max2);
	cpid->output_max = output_max2;
}

void cascade_pid_calc(cascade_pid_t *cpid, float ref, float x1_fdb, float x2_fdb, uint8_t is_enable)
{
	if(is_enable)
	{
		cpid->td.aim = ref;
		td_function(&cpid->td);
		pid_calc(&cpid->pos_loop, cpid->td.x1, x1_fdb);
		pid_calc(&cpid->spd_loop, cpid->pos_loop.output + cpid->td.x2, x2_fdb);
		cpid->output = cpid->spd_loop.output + cpid->nonlinear_torque;
		cpid->output = sat(cpid->output, -cpid->output_max, cpid->output_max);
	}
	else
	{
		cpid->td.aim = ref;
		cpid->td.x1 = x1_fdb;
		cpid->td.x2 = x2_fdb;
		cpid->output = 0;
	}
}

void cascade_pid_calc_test(cascade_pid_t *cascade_pid, float ref, float x1_fdb, float x2_fdb, uint8_t type)
{
	cascade_pid->td.aim = ref;
	td_function(&cascade_pid->td);
	if(type == 1)
	{
		pid_calc(&cascade_pid->pos_loop, ref, x1_fdb);
		pid_calc(&cascade_pid->spd_loop, cascade_pid->pos_loop.output + cascade_pid->td.x2, x2_fdb);
		cascade_pid->output = cascade_pid->spd_loop.output + cascade_pid->nonlinear_torque;
		cascade_pid->output = sat(cascade_pid->output, -cascade_pid->output_max, cascade_pid->output_max);
	}
	else
	{
		pid_calc(&cascade_pid->pos_loop, cascade_pid->td.x1, x1_fdb);
		pid_calc(&cascade_pid->spd_loop, cascade_pid->pos_loop.output + cascade_pid->td.x2, x2_fdb);
		cascade_pid->output = cascade_pid->spd_loop.output + cascade_pid->nonlinear_torque;
		cascade_pid->output = sat(cascade_pid->output, -cascade_pid->output_max, cascade_pid->output_max);
	}
		
}

void encoder_process(encoder_t *encoder, float theta_now, float dt)
{
	encoder->theta_last = encoder->theta;
	encoder->theta = theta_now;
	encoder->delta_theta = encoder->theta - encoder->theta_last;
	
	if(encoder->delta_theta < -PI && encoder->delta_theta > -PI * 2.0f) {
		encoder->delta_theta += PI * 2.0f;
		encoder->cylinder++;
	}
		
	else if(encoder->delta_theta > PI && encoder->delta_theta < PI * 2.0f) {
		encoder->delta_theta -= PI * 2.0f;
		encoder->cylinder--;
	}
		
	encoder->theta_inf += encoder->delta_theta;
	encoder->omiga = encoder->delta_theta / dt;
}

void ramp(float* pvalue, float ref, float step)
{
  uint8_t type;
	if(*pvalue < ref) type = 0;
	else if(*pvalue > ref) type = 1;
	if(!type)
	{
		if(*pvalue >= ref) *pvalue = ref;
		else 
		{
			*pvalue += step;
			if(*pvalue >= ref) *pvalue = ref;
		}
	}
	else
	{
		if(*pvalue <= ref) *pvalue = ref;
		else 
		{
			*pvalue -= step;
			if(*pvalue <= ref) *pvalue = ref;
		}
	}
}


/**
* @brief MRAC calculate 
* @param mrac struct, ref, all fdb
* @retval None
*/
void mrac_2d_calc(mrac_2d_t *mrac, float ref, float x1_fdb, float x2_fdb)
{
	float cosx, sinx;
	cosx = (float)cos(x1_fdb*PI/180.0f);
	sinx = (float)sin(x1_fdb*PI/180.0f);
	
	if(mrac->enable == 1)
	{
		//update fdb
		mrac->x1_fdb = x1_fdb;
		mrac->x2_fdb = x2_fdb;
		//update x_td
		mrac->x1_td.aim = ref;
		td_function(&mrac->x1_td);
		mrac->x1_ref = mrac->x1_td.x1;
		mrac->x1_err = mrac->x1_ref - mrac->x1_fdb;
		
		//x2_ref = k1 * x1_err + dot_x1_ref  feedforward
		mrac->x2_ref = mrac->k1 * mrac->x1_err + mrac->x1_td.x2;
		mrac->x2_err = mrac->x2_ref - mrac->x2_fdb;
		mrac->x2_td.aim = mrac->x2_ref;
		td_function(&mrac->x2_td);
		
		//u = alpha1 * dot_v_ref + alpha2 * cos(theta) + alpha3 * sin(theta) + k2 * v_err
		mrac->output = mrac->k2 * mrac->x2_err + \
									 mrac->alpha1 * mrac->x2_td.x2 + \
									 mrac->alpha2 * cosx + \
									 mrac->alpha3 * sinx;
		mrac->output = sat(mrac->output, -mrac->output_max, mrac->output_max);
		
		//alpha(i)(k+1) = alpha(i)(k) + dt * gamma(i) * v_err * TBD
		mrac->alpha1 += mrac->dt * mrac->gamma1 * mrac->x2_err * mrac->x2_td.x2;
		mrac->alpha2 += mrac->dt * mrac->gamma2 * mrac->x2_err * cosx;
		mrac->alpha3 += mrac->dt * mrac->gamma3 * mrac->x2_err * sinx;
	}
	else
	{
		mrac->x1_td.aim = x1_fdb;
		mrac->x1_td.x1 = x1_fdb;
		mrac->x1_td.x2 = 0;
		mrac->x1_ref = x1_fdb;
		mrac->x1_err = 0;
		
		mrac->x2_td.aim = x2_fdb;
		mrac->x2_td.x1 = x2_fdb;
		mrac->x2_td.x2 = 0;
		mrac->x2_ref = x2_fdb;
		mrac->x2_err = 0;
		
		mrac->output = 0;
	}
}


void tob_init(tob_t *tob, float dt, float J, float gain)
{
	tob->dt = dt;
	tob->b = 1/J;
	tob->kv = gain;
	tob->kt = -gain*gain/4/tob->b;
}

/**
* @brief torque observer calculate 
* @param mrac struct, ref, all fdb
* @retval None
*/
void tob_calc(tob_t *tob, float fdb, float output)
{
	tob->kt = -tob->kv * tob->kv/4/tob->b;

	tob->v_err = fdb - tob->v_hat;
	
	tob->v_hat = tob->v_hat_last + tob->dt * tob->b * (output - tob->tl_hat) + tob->kv * tob->v_err * tob->dt;
	tob->tl_hat = tob->tl_hat_last + tob->kt * tob->v_err * tob->dt;
	
	tob->v_hat_last = tob->v_hat;
	tob->tl_hat_last = tob->tl_hat;
	
}

void flpd2d_init(flpd2d_t *pc, float kp, float kd, float r, float h, float dt, float J, float gain, float output_max)
{
	td_init(&pc->td, r, h, dt);
	tob_init(&pc->ob, dt, J, gain);
	
	pc->kp = kp;
	pc->kd = kd;
	pc->output_max = output_max;
}

/**
* @brief controller 2d with feedback linearization 
* @param flc2d struct, ref, all fdb
* @retval None
*/
void flpd2d_calc(flpd2d_t *pc, float x1_ref, float x1_fdb, float x2_fdb, uint8_t is_enable)
{
	if(is_enable)
	{
		pc->td.aim = x1_ref;
		td_function(&pc->td);
		pc->x1_err = pc->td.x1 - x1_fdb;
		pc->x2_err = pc->td.x2 - x2_fdb;
		
		tob_calc(&pc->ob, x2_fdb, pc->output);
		pc->output = pc->kp * pc->x1_err + pc->kd * pc->x2_err + pc->ob.tl_hat;
		pc->output = sat(pc->output, -pc->output_max, pc->output_max);
	}
	else
	{
		pc->td.aim = x1_fdb;
		pc->td.x1 = x1_fdb;
		pc->td.x2 = x1_fdb;
		pc->output = 0;
		tob_calc(&pc->ob, x2_fdb, pc->output);
	}
}

void flpid_init(flpid_t *pc, float kp, float ki, float kd, float err_max, float output_max,
										float dt, float J, float gain)
{
	pid_init(&pc->pid, kp, ki, kd, output_max, err_max);
	tob_init(&pc->ob, dt, J, gain);
	pc->output_max = output_max;
}


void flpid_calc(flpid_t *pc, float x_ref, float x_fdb, uint8_t is_enable)
{
	if(is_enable)
	{
		tob_calc(&pc->ob, x_fdb, pc->output);
		pid_calc(&pc->pid, x_ref, x_fdb);
		pc->output = pc->pid.output + pc->ob.tl_hat;
		pc->output = sat(pc->output, -pc->output_max, pc->output_max);
	}
	else
	{
		pc->output = 0;
		tob_calc(&pc->ob, x_fdb, pc->output);
		pc->pid.ref = x_fdb;
		pc->pid.err_sum = 0;
	}
}

void flcpid2d_init(flcpid2d_t *pc, float kp1, float ki1, float kd1, float err_max1, float output_max1,
									 float kp2, float ki2, float kd2, float err_max2, float output_max2,
										float r, float h, float dt, float J, float gain)
{
	cascade_pid_init(&pc->cpid, kp1, ki1, kd1, err_max1, output_max1,
															kp2, ki2, kd2, err_max2, output_max2,
															r, h, dt);
	tob_init(&pc->ob, dt, J, gain);
	pc->output_max = output_max2;
}


void flcpid2d_calc(flcpid2d_t *pc, float x1_ref, float x1_fdb, float x2_fdb, uint8_t is_enable)
{
	tob_calc(&pc->ob, x2_fdb, pc->output);
	cascade_pid_calc(&pc->cpid, x1_ref, x1_fdb, x2_fdb, is_enable);
	pc->output = pc->cpid.output + pc->ob.tl_hat;
	pc->output = sat(pc->output, -pc->output_max, pc->output_max);
}


void BounceWheelInit(bounce_wheel_t *pw)
{
	td_init(&pw->planner, 50, 0.001, 0.001);
	flpd2d_init(&pw->theta1_ctrl, 1, 1, 2000, 0.001, 0.001, 1, 1, 30);
	flpd2d_init(&pw->theta2_ctrl, 1, 1, 2000, 0.001, 0.001, 1, 1, 30);
	pw->length = 1;
}

void BounceWheelControl(bounce_wheel_t *pw, uint8_t state, float r_set, float r_reset, 
												float theta1_m, float theta2_m, float omiga1_m, float omiga2_m, uint8_t is_enable)
{
	if(state == 1)
	{
		pw->xb_ref = BOUNCE_WHEEL_SET_POS;
		pw->planner.r = r_set;
	}
	else
	{
		pw->xb_fdb = BOUNCE_WHEEL_RESET_POS;
		pw->planner.r = r_reset;
	}
	
	pw->theta1 = theta1_m - pw->theta1_offset;
	pw->theta2 = theta2_m - pw->theta2_offset;
	pw->phi_err = - pw->theta1 - pw->theta2/2 - PI/2;
	pw->xb_fdb = 2 * pw->length * cos(pw->theta2);
	
	td_function(&pw->planner);
	pw->xb_ref = pw->planner.x1 * cos(pw->phi_err);
	pw->theta1_ref =     asin(pw->xb_ref / 2 / pw->length);
	pw->theta2_ref = 2 * acos(pw->xb_ref / 2 / pw->length);
	
	flpd2d_calc(&pw->theta1_ctrl, pw->theta1_ref, pw->theta1, omiga1_m, is_enable);
	flpd2d_calc(&pw->theta2_ctrl, pw->theta2_ref, pw->theta2, omiga2_m, is_enable);
	
}






