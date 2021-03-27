#include "robot.h"
#include "can_motor.h"
#include "supervise.h"
#include "mpu6050.h"
#include "controller.h"

//struct Robot_t infantry;
//struct ramp_t FrictionLeftWheel;
//struct ramp_t FrictionRightWheel;
//int16_t pitch_offset, yaw_offset;

//mrac_2d_t pitch_mrac = {.k1 = 40.0f, 
//												.k2 = 100.0f, 
//												.alpha1 = 1,
//												.alpha2 = 3000.0f,
//												.alpha3 = 1200.0f,
//												.gamma1 = 0,
//												.gamma2 = 0,
//												.gamma3 = 0,
//												.output_max = 30000,
//												.dt = 0.001f,
//												.x1_td.r = 8000,
//												.x1_td.h = 0.001f,
//												.x1_td.dt = 0.001f,
//												.x2_td.r = 20000,
//												.x2_td.h = 0.001f,
//												.x2_td.dt = 0.001f,
//												.integrator.ki = 15.0f,
//												.integrator.sum_err_max = 50000,
//												.enable = 0};

//mrac_2d_t yaw_mrac = 	 {.k1 = 40.0f, //50
//												.k2 = 100.0f, //100
//												.alpha1 = 0,
//												.gamma1 = 0,
//												.gamma2 = 0,
//												.gamma3 = 0,
//												.output_max = 30000,
//												.dt = 0.001f,
//												.x1_td.r = 10000,
//												.x1_td.h = 0.001f,
//												.x1_td.dt = 0.001f,
//												.x2_td.r = 20000,
//												.x2_td.h = 0.001f,
//												.x2_td.dt = 0.001f,
//												.integrator.ki = 20.0f,
//												.integrator.sum_err_max = 50000,
//												.enable = 0};											

void RobotParamInit(void)
{
//	GetGimbalOffset();
//	pitch_motor.encoder_offset = pitch_offset;
//	yaw_motor.encoder_offset   = yaw_offset;
}

void GimbalStateChange(void)
{
//	if(Is_Error(1 << CHASSIS_INDEX) || infantry.gimbal_control_info.gimbal_mode == 0)
//		infantry.gimbal_state = Stop;
//	else if(infantry.gimbal_control_info.gimbal_mode == 1)
//		infantry.gimbal_state = Follow_Encoder;
//	else if(infantry.gimbal_control_info.gimbal_mode == 2)
//		infantry.gimbal_state = Follow_Gyro;
}

/**
* @brief 斜坡初始化函数
* @param 斜坡结构体，斜坡时间（ms）
* @retval None
*/
void RampInit(struct ramp_t *ramp, int scale)
{
  ramp->count = 0;     //清空斜坡函数
  ramp->scale = scale; //设置斜坡周期
  ramp->out = 0;       //清空斜坡输出
}

/**
* @brief 斜坡计算函数
* @param 斜坡结构体
* @retval 0~1之间的浮点数
*/
float RampCalc(struct ramp_t *ramp)
{
  if(ramp->scale <= 0)
    return 0;
  ramp->count++;                                  //斜坡计数累加
  if(ramp->count >= ramp->scale)
    ramp->count = ramp->scale;                    //溢出保护
  
  ramp->out = ramp->count / ((float)ramp->scale); //计算百分比，反馈浮点数
  return ramp->out;
}
