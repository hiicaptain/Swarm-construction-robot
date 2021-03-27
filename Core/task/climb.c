#include "climb.h"
#include "controller.h"
#include "can_motor.h"

#define ROUTE_LENGTH 6000
#define SPACING_DEAD_REGION 10
uint8_t support_link_init_status = 0;

climb_state_t climb_status = READY;

float position_offset;
float position_now;
float position_set;
float position_planned;
float position_target;
flcpid2d_t supportlink_ctrl;
extern encoder_t motor_climb_encoder;

void SupportLinkInit(void)
{
	position_offset = motor_climb_encoder.theta_inf;
	position_set = position_offset - ROUTE_LENGTH;
	position_now = motor_climb_encoder.theta_inf - position_offset;
	flcpid2d_init(&supportlink_ctrl, 10, 0, 0, 0, 2000, 30, 0, 0, 0, 10000, 10000, 0.001, 0.001, 1, 200);
//	position_target = position_now;
}

void SelectMode(void)
{
	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == GPIO_PIN_SET)
	{
		climb_status = CALIB;
		position_target += 0.2f;
	}
	else
	{
		if(climb_status == CLIMBING && fabs(position_now - position_set) < 10)
		{
			climb_status = PUTDOWN;
		}	
		else if(climb_status != CLIMBING && climb_status != PUTDOWN)
		{
			climb_status = READY;
		}
	}
}

void SelfCalibration(void)
{
	position_target += 0.2f;
	flcpid2d_calc(&supportlink_ctrl, position_target, motor_climb_encoder.theta_inf, motor_climb_encoder.omiga, 1);
	if(supportlink_ctrl.cpid.pos_loop.err > 5)
	{
		support_link_init_status = 1;
		position_offset = motor_climb_encoder.theta_inf - SPACING_DEAD_REGION;
		position_now = motor_climb_encoder.theta_inf - position_offset;
		position_set = position_offset - ROUTE_LENGTH;
		supportlink_ctrl.output = 0;
	}
}

void SupportLinkControl(uint8_t status)
{
	position_now = motor_climb_encoder.theta_inf - position_offset;
//	ramp(&position_planned, position_target, 1.0);
//	flcpid2d_calc(&supportlink_ctrl, position_planned, position_now, motor_climb_encoder.omiga, 1);
	
	if(status == 1)
	{
		ramp(&position_planned, position_set, 0.5);
		flcpid2d_calc(&supportlink_ctrl, position_planned, position_now, motor_climb_encoder.omiga, 1);
	}
	else if(status == 0)
	{
		ramp(&position_planned, position_offset, 0.5);
		flcpid2d_calc(&supportlink_ctrl, position_planned, position_now, motor_climb_encoder.omiga, 1);
	}
	else if(status == 2)
	{
		position_planned = position_offset;
		flcpid2d_calc(&supportlink_ctrl, position_target, motor_climb_encoder.theta_inf, motor_climb_encoder.omiga, 1);
	}
	else
	{
		position_planned = position_offset;
		flcpid2d_calc(&supportlink_ctrl, position_target, motor_climb_encoder.theta_inf, motor_climb_encoder.omiga, 1);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_2)
	{
		position_offset = motor_climb_encoder.theta_inf;
		position_target = motor_climb_encoder.theta_inf;
	}
}
