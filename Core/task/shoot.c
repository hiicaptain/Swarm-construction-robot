#include "shoot.h"
#include "robot.h"
#include "can_motor.h"

/**
* @brief 机器人发射机构参数更新
* @param None
* @retval None
*/
//void ShootParamChange(void)
//{
//	static float reference = 0;
//	if(infantry.gimbal_state == Stop)
//	{
//		pluck_motor.cascade_pid.enable = 0;
//		reference = pluck_motor.angle_inf;
//		pid_2d_calc(&pluck_motor.cascade_pid, pluck_motor.angle_inf, pluck_motor.angle_inf, pluck_motor.angular_spd);
//		return;
//	}
//	if(infantry.gimbal_control_info.shoot_mode != 0)
//	{
//		pluck_motor.cascade_pid.enable = 1;
//		if(infantry.gimbal_control_info.shoot_add_num  == 1 && TIM1->CCR1 > 1200)
//			reference = pluck_motor.angle_inf + infantry.gimbal_control_info.shoot_freq * 45 * 36;
//		else if(infantry.gimbal_control_info.shoot_add_num  == 2 && TIM1->CCR1 > 1200)
//			reference = pluck_motor.angle_inf - infantry.gimbal_control_info.shoot_freq * 45 * 36;
//		pid_2d_calc(&pluck_motor.cascade_pid, reference, pluck_motor.angle_inf, pluck_motor.angular_spd);//注意速度与角度的尺度问题
//	}
//	else
//	{
//		reference = pluck_motor.angle_inf;
//		pluck_motor.cascade_pid.enable = 1;
//		pid_2d_calc(&pluck_motor.cascade_pid, pluck_motor.angle_inf, pluck_motor.angle_inf, pluck_motor.angular_spd);
//	}
//}

//void ShootDataSend(void)
//{
//	static int Frictionspeed = 0;
//	if(infantry.gimbal_control_info.shoot_mode == 1)
//	{
//		Frictionspeed = infantry.gimbal_control_info.friction_wheel_speed;
//		TIM1->CCR1 = TIM1->CCR4 = 1000 + (infantry.gimbal_control_info.friction_wheel_speed-1000)
//			*RampCalc(&FrictionLeftWheel);
//		RampInit(&FrictionRightWheel,8000);
//	}
//	else if(infantry.gimbal_control_info.shoot_mode == 0)
//	{
//		RampInit(&FrictionLeftWheel,8000);
//		TIM1->CCR1 = TIM1->CCR4 = Frictionspeed - (Frictionspeed-1000)*RampCalc(&FrictionRightWheel);
//		if(TIM1->CCR1 < 1000)
//			TIM1->CCR1 = TIM1->CCR4 = 1000;
//	}
//}
