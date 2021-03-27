/**
  ******************************************************************************
  * �ļ���        ��IMU.c
	* �ļ�����      ���ٷ�������BMI088��̬����
	* ����ʱ��      ��2019.12.18
	* ����          ��������
	*-----------------------------------------------------------------------------
	* ����޸�ʱ��  ��2019.12.18
	* �޸���        ��������
  ******************************************************************************
	* 1.���������STM32F427IIH6��������̻���λKeil 5������FreeRTOS���п���
	* 2.������ֻ������Robomaster������
	* 3.������δ�������ֹ˽��ת������ֹ˽��ʹ��
	* 4.�����������������ע�ͣ�����ANSI�����ʽ��
	* 5.���������ս���Ȩ���������ҵ��ѧ�����ڣ��Ϲ���ӥս��Critical HIT����
	* 
	* Copyright (c) ��������ҵ��ѧ�����ڣ��Ϲ���ӥս��Critical HIT ��Ȩ����
	******************************************************************************
  */

#include "IMU.h"
#include "math.h"
#include "string.h"

#define RAD2DEG		57.29578f		  /* ����ת�� */

IMU_Data_t IMU_Data;

/**
* @brief BMI088��������̬����
* @param None
* @retval None
*/
void IMUHandle(void)
{
	IMU_Data.z = atan2(IMU_Data.quat_fp32[0]*IMU_Data.quat_fp32[3]-IMU_Data.quat_fp32[1]*IMU_Data.quat_fp32[2],
	                      IMU_Data.quat_fp32[0]*IMU_Data.quat_fp32[0]+IMU_Data.quat_fp32[2]*IMU_Data.quat_fp32[2]-0.5f)*RAD2DEG;
	IMU_Data.y = asin(2*(IMU_Data.quat_fp32[0]*IMU_Data.quat_fp32[1]+IMU_Data.quat_fp32[2]*IMU_Data.quat_fp32[3]))*RAD2DEG;
	IMU_Data.x = atan2(IMU_Data.quat_fp32[0]*IMU_Data.quat_fp32[2]-IMU_Data.quat_fp32[1]*IMU_Data.quat_fp32[3],
	                       IMU_Data.quat_fp32[0]*IMU_Data.quat_fp32[0]+IMU_Data.quat_fp32[3]*IMU_Data.quat_fp32[3]-0.5f)*RAD2DEG;
	
	IMU_Data.last_yaw = IMU_Data.yaw;
	IMU_Data.yaw = IMU_Data.z * 22.7556f;
	if(IMU_Data.yaw - IMU_Data.last_yaw > 4096)
		IMU_Data.yaw_count--;
	else if(IMU_Data.yaw - IMU_Data.last_yaw < -4096)
		IMU_Data.yaw_count++;
	IMU_Data.encoder_yaw = IMU_Data.yaw + IMU_Data.yaw_count * 8192;
	IMU_Data.real_yaw = IMU_Data.z + 360 * IMU_Data.yaw_count;
	
	IMU_Data.last_pitch = IMU_Data.pitch;
	IMU_Data.pitch = IMU_Data.x * 22.7556f;
	if(IMU_Data.pitch - IMU_Data.last_pitch > 4096)
		IMU_Data.pitch_count--;
	else if(IMU_Data.pitch - IMU_Data.last_pitch < -4096)
		IMU_Data.pitch_count++;
	IMU_Data.encoder_pitch = IMU_Data.pitch + IMU_Data.pitch_count * 8192;
	
	IMU_Data.last_roll = IMU_Data.roll;
	IMU_Data.roll = IMU_Data.y * 22.7556f;
	if(IMU_Data.roll - IMU_Data.last_roll > 4096)
		IMU_Data.roll_count--;
	else if(IMU_Data.roll - IMU_Data.last_roll < -4096)
		IMU_Data.roll_count++;
	IMU_Data.encoder_roll = IMU_Data.roll + IMU_Data.roll_count * 8192;
}

/**
* @brief BMI088������CANͨ�Ž����жϻص�����
* @param CANx��CANͨ����
* @retval None
* @TODO 
*/
void CanIMUReceiveCallback(CAN_RxHeaderTypeDef RxHeader, uint8_t *CanData)
{
	switch(RxHeader.StdId)
	{
		#ifdef IMU_Debug
		case RM_IMU_PARAM_ID:                               //��ͨ������ATָ�����ú������Ǻ������ȡ��������Ϣ������������궨��debug��ȡ
			IMU_Data.accel_rangle = CanData[0] & 0x0F;
			IMU_Data.gyro_rangle = (CanData[0] & 0xF0)>>4;
			IMU_Data.sensor_control_temperature = CanData[2];
			IMU_Data.imu_sensor_rotation = CanData[3] & 0x1F;         
			IMU_Data.ahrs_rotation_sequence = (CanData[3] & 0xE0) >> 5;         
			IMU_Data.quat_euler = CanData[4] & 0x01;	
			IMU_Data.gyro_sen = GYRO_2000_SEN;
			IMU_Data.accel_sen = ACCEL_3G_SEN;
			break;
		#endif
		case RM_IMU_QUAT_ID:                                   //��Ԫ����ȡ
			#ifdef QUAT
			memcpy(IMU_Data.quat,CanData,8);
			IMU_Data.quat_fp32[0] = IMU_Data.quat[0] * 0.0001f;
			IMU_Data.quat_fp32[1] = IMU_Data.quat[1] * 0.0001f;
			IMU_Data.quat_fp32[2] = IMU_Data.quat[2] * 0.0001f;
			IMU_Data.quat_fp32[3] = IMU_Data.quat[3] * 0.0001f;
			IMUHandle();
			#endif
		
			#ifdef EULER_ANGLE
			memcpy(IMU_Data.euler_angle,CanData,6);              //ŷ���Ƕ�ȡ
			IMU_Data.euler_angle_fp32[0] = IMU_Data.euler_angle[0] * 0.0001f;
			IMU_Data.euler_angle_fp32[1] = IMU_Data.euler_angle[1] * 0.0001f;
			IMU_Data.euler_angle_fp32[2] = IMU_Data.euler_angle[2] * 0.0001f;
			#endif
			break;
		case RM_IMU_GYRO_ID:                                   //����������
			memcpy(IMU_Data.gyro_int16, CanData,6);
			IMU_Data.gyro_fp32[0] = IMU_Data.gyro_int16[0]*GYRO_2000_SEN;
			IMU_Data.gyro_fp32[1] = IMU_Data.gyro_int16[1]*GYRO_2000_SEN;
			IMU_Data.gyro_fp32[2] = IMU_Data.gyro_int16[2]*GYRO_2000_SEN;
			IMU_Data.sensor_temperature = (int16_t)(CanData[6]<<3|CanData[7]>>5);
			if(IMU_Data.sensor_temperature > 1023)
				IMU_Data.sensor_temperature -= 2048;
			break;
		case RM_IMU_ACCEL_ID:                                  //���ٶȼ�����
			memcpy(IMU_Data.accel_int16, CanData,6); 
			IMU_Data.accel_fp32[0] = IMU_Data.accel_int16[0]*ACCEL_3G_SEN;
			IMU_Data.accel_fp32[1] = IMU_Data.accel_int16[1]*ACCEL_3G_SEN;
			IMU_Data.accel_fp32[2] = IMU_Data.accel_int16[2]*ACCEL_3G_SEN;
			break;
		case RM_IMU_MAG_ID:                                    //����������
			memcpy(IMU_Data.mag_int16,CanData,6);
			break;
		default: ;
	}
}
