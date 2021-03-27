/**
  ******************************************************************************
  * �ļ���        ��Kalman.h
	* ����ʱ��      ��2019.11.24
	* ����          ��������
	*-----------------------------------------------------------------------------
	* ����޸�ʱ��  ��2019.11.24
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

#ifndef _KALMAN_H_
#define _KALMAN_H_

typedef struct 
{
	float dt;            //�������˲�������ʱ��
	float angle;
	float angle_dot;     //�Ƕȼ����ٶ�
	float P[2][2];
	float Pdot[4];
	float Q_angle;
	float Q_gyro;        //�Ƕ����Ŷȣ����ٶ����Ŷ�
	float R_angle;
	float C_0;
	float q_bias;
	float angle_err;
	float PCt_0;
	float PCt_1;
	float E;
	float K_0;
	float K_1;
	float t_0;
	float t_1;
}KALMAN_t;

extern KALMAN_t Kalman_yaw;
extern KALMAN_t Kalman_pitch;
extern KALMAN_t Kalman_roll;

void KalmanInit(KALMAN_t *Kalman);
float Kalman_Filter(float angle_m, float gyro_m, KALMAN_t *Kalman);

#endif 
