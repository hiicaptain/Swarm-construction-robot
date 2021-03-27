/**
  ******************************************************************************
  * 文件名        ：IMU.h
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

#ifndef _IMU_H_
#define _IMU_H_

#include "stm32f4xx.h"

#define    RM_IMU_QUAT_ID   0x401 
#define    RM_IMU_GYRO_ID   0x402 
#define    RM_IMU_ACCEL_ID  0x403 
#define    RM_IMU_MAG_ID    0x404 
#define    RM_IMU_PARAM_ID  0x405

#define  ACCEL_3G_SEN 0.0008974358974f
#define  GYRO_2000_SEN 0.061035156f

/* 注释掉的代码由于测试用，配置好之后检测，若无错误则可完全注释 */
#define IMU_Debug
#define QUAT          //四元数读取（优先配置四元数）
//#define EULER_ANGLE   //欧拉角读取（存在万向节死锁的问题）

typedef struct 
{
	#ifdef IMU_Debug
	uint8_t quat_euler:1;     
	uint8_t gyro_rangle:3;     
	uint8_t accel_rangle:2;     
	uint8_t imu_sensor_rotation:5;     
	uint8_t ahrs_rotation_sequence:3;     
	uint16_t sensor_time; 
	int16_t sensor_control_temperature;     
	float gyro_sen;     
	float accel_sen;
	#endif
	
	int16_t quat[4];            //四元数原始数据
	float quat_fp32[4];         //四元数处理后的数据
	int16_t euler_angle[3];     //欧拉角原始数据
	float euler_angle_fp32[3];  //欧拉角处理后的数据
	int16_t gyro_int16[3];      //陀螺仪原始数据
	int16_t accel_int16[3];     //加速度计原始数据
	int16_t mag_int16[3];       //磁力计原始数据
	float gyro_fp32[3];         //角速度处理后的道具
	float accel_fp32[3];        //加速度处理后的数据
	uint16_t sensor_temperature;//温度
	
	float x;
	float y;
	float z;
	float real_yaw;
	int yaw;                    //IMU_Data结构体的欧拉角数据
	int last_yaw;
	int encoder_yaw;
	int yaw_count;
	int pitch;
	int last_pitch;
	int encoder_pitch;
	int pitch_count;
	int roll;
	int last_roll;
	int encoder_roll;
	int roll_count;
}IMU_Data_t;

extern IMU_Data_t IMU_Data;

void IMUHandle(void);
void CanIMUReceiveCallback(CAN_RxHeaderTypeDef RxHeader, uint8_t *CanData);

#endif
