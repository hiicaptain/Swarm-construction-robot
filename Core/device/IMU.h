/**
  ******************************************************************************
  * �ļ���        ��IMU.h
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

/* ע�͵��Ĵ������ڲ����ã����ú�֮���⣬���޴��������ȫע�� */
#define IMU_Debug
#define QUAT          //��Ԫ����ȡ������������Ԫ����
//#define EULER_ANGLE   //ŷ���Ƕ�ȡ��������������������⣩

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
	
	int16_t quat[4];            //��Ԫ��ԭʼ����
	float quat_fp32[4];         //��Ԫ������������
	int16_t euler_angle[3];     //ŷ����ԭʼ����
	float euler_angle_fp32[3];  //ŷ���Ǵ���������
	int16_t gyro_int16[3];      //������ԭʼ����
	int16_t accel_int16[3];     //���ٶȼ�ԭʼ����
	int16_t mag_int16[3];       //������ԭʼ����
	float gyro_fp32[3];         //���ٶȴ����ĵ���
	float accel_fp32[3];        //���ٶȴ���������
	uint16_t sensor_temperature;//�¶�
	
	float x;
	float y;
	float z;
	float real_yaw;
	int yaw;                    //IMU_Data�ṹ���ŷ��������
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
