/**
  ******************************************************************************
  * 文件名        ：IMU.c
	* 文件描述      ：官方陀螺仪BMI088姿态解析
	* 创建时间      ：2019.12.18
	* 作者          ：刘文熠
	*-----------------------------------------------------------------------------
	* 最近修改时间  ：2019.12.18
	* 修改人        ：刘文熠
  ******************************************************************************
	* 1.本代码基于STM32F427IIH6开发，编程环境位Keil 5，基于FreeRTOS进行开发
	* 2.本代码只适用于Robomaster机器人
	* 3.本代码未经允许禁止私自转发、禁止私自使用
	* 4.本代码包含大量中文注释，请以ANSI编码格式打开
	* 5.本代码最终解释权归哈尔滨工业大学（深圳）南工骁鹰战队Critical HIT所有
	* 
	* Copyright (c) 哈尔滨工业大学（深圳）南工骁鹰战队Critical HIT 版权所有
	******************************************************************************
  */

#include "IMU.h"
#include "math.h"
#include "string.h"

#define RAD2DEG		57.29578f		  /* 弧度转度 */

IMU_Data_t IMU_Data;

/**
* @brief BMI088陀螺仪姿态解析
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
* @brief BMI088陀螺仪CAN通信接收中断回调函数
* @param CANx（CAN通道）
* @retval None
* @TODO 
*/
void CanIMUReceiveCallback(CAN_RxHeaderTypeDef RxHeader, uint8_t *CanData)
{
	switch(RxHeader.StdId)
	{
		#ifdef IMU_Debug
		case RM_IMU_PARAM_ID:                               //已通过串口AT指令配置后陀螺仪后，无需读取其配置信息，若出现问题宏定义debug读取
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
		case RM_IMU_QUAT_ID:                                   //四元数读取
			#ifdef QUAT
			memcpy(IMU_Data.quat,CanData,8);
			IMU_Data.quat_fp32[0] = IMU_Data.quat[0] * 0.0001f;
			IMU_Data.quat_fp32[1] = IMU_Data.quat[1] * 0.0001f;
			IMU_Data.quat_fp32[2] = IMU_Data.quat[2] * 0.0001f;
			IMU_Data.quat_fp32[3] = IMU_Data.quat[3] * 0.0001f;
			IMUHandle();
			#endif
		
			#ifdef EULER_ANGLE
			memcpy(IMU_Data.euler_angle,CanData,6);              //欧拉角读取
			IMU_Data.euler_angle_fp32[0] = IMU_Data.euler_angle[0] * 0.0001f;
			IMU_Data.euler_angle_fp32[1] = IMU_Data.euler_angle[1] * 0.0001f;
			IMU_Data.euler_angle_fp32[2] = IMU_Data.euler_angle[2] * 0.0001f;
			#endif
			break;
		case RM_IMU_GYRO_ID:                                   //陀螺仪数据
			memcpy(IMU_Data.gyro_int16, CanData,6);
			IMU_Data.gyro_fp32[0] = IMU_Data.gyro_int16[0]*GYRO_2000_SEN;
			IMU_Data.gyro_fp32[1] = IMU_Data.gyro_int16[1]*GYRO_2000_SEN;
			IMU_Data.gyro_fp32[2] = IMU_Data.gyro_int16[2]*GYRO_2000_SEN;
			IMU_Data.sensor_temperature = (int16_t)(CanData[6]<<3|CanData[7]>>5);
			if(IMU_Data.sensor_temperature > 1023)
				IMU_Data.sensor_temperature -= 2048;
			break;
		case RM_IMU_ACCEL_ID:                                  //加速度计数据
			memcpy(IMU_Data.accel_int16, CanData,6); 
			IMU_Data.accel_fp32[0] = IMU_Data.accel_int16[0]*ACCEL_3G_SEN;
			IMU_Data.accel_fp32[1] = IMU_Data.accel_int16[1]*ACCEL_3G_SEN;
			IMU_Data.accel_fp32[2] = IMU_Data.accel_int16[2]*ACCEL_3G_SEN;
			break;
		case RM_IMU_MAG_ID:                                    //磁力计数据
			memcpy(IMU_Data.mag_int16,CanData,6);
			break;
		default: ;
	}
}
