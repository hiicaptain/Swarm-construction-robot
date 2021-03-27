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
* @brief б�³�ʼ������
* @param б�½ṹ�壬б��ʱ�䣨ms��
* @retval None
*/
void RampInit(struct ramp_t *ramp, int scale)
{
  ramp->count = 0;     //���б�º���
  ramp->scale = scale; //����б������
  ramp->out = 0;       //���б�����
}

/**
* @brief б�¼��㺯��
* @param б�½ṹ��
* @retval 0~1֮��ĸ�����
*/
float RampCalc(struct ramp_t *ramp)
{
  if(ramp->scale <= 0)
    return 0;
  ramp->count++;                                  //б�¼����ۼ�
  if(ramp->count >= ramp->scale)
    ramp->count = ramp->scale;                    //�������
  
  ramp->out = ramp->count / ((float)ramp->scale); //����ٷֱȣ�����������
  return ramp->out;
}
