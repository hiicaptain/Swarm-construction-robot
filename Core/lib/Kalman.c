/**
  ******************************************************************************
  * �ļ���        ��Kalman.c
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

#include "Kalman.h"

KALMAN_t Kalman_yaw;
KALMAN_t Kalman_pitch;
KALMAN_t Kalman_roll;

/**
* @brief �������ṹ���ʼ������
* @param �������ṹ��
* @retval None
*/
void KalmanInit(KALMAN_t *Kalman)
{
	Kalman->dt = 0.001f;
	Kalman->Q_angle = 0.001f;
	Kalman->Q_gyro = 0.005f;
	Kalman->R_angle = 0.5f;
	Kalman->C_0 = 1.0f;
}

/**
* @brief ������һ���˲�
* @param �Ƕ�angle_x�ͽ��ٶ�gyro_y
* @retval ���ŽǶ�
*/
float Kalman_Filter(float angle_m, float gyro_m, KALMAN_t *Kalman)
{
	Kalman->angle += (gyro_m - Kalman->q_bias) * Kalman->dt;
	
	Kalman->Pdot[0] = Kalman->Q_angle - Kalman->P[0][1] - Kalman->P[1][0];
	Kalman->Pdot[1] = -Kalman->P[1][1];
	Kalman->Pdot[2] = -Kalman->P[1][1];
	Kalman->Pdot[3] = Kalman->Q_gyro;
	Kalman->P[0][0] += Kalman->Pdot[0] * Kalman->dt;
	Kalman->P[0][1] += Kalman->Pdot[1] * Kalman->dt;
	Kalman->P[1][0] += Kalman->Pdot[2] * Kalman->dt;
	Kalman->P[1][1] += Kalman->Pdot[3] * Kalman->dt;
	
	Kalman->angle_err = angle_m - Kalman->angle;
	
	Kalman->PCt_0 = Kalman->C_0 * Kalman->P[0][0];
	Kalman->PCt_1 = Kalman->C_0 * Kalman->P[1][0];
	Kalman->E = Kalman->R_angle + Kalman->C_0 * Kalman->PCt_0;
	Kalman->K_0 = Kalman->PCt_0 / Kalman->E;
	Kalman->K_1 = Kalman->PCt_1 / Kalman->E;
	Kalman->t_0 = Kalman->PCt_0;
	Kalman->t_1 = Kalman->C_0 * Kalman->P[0][1];
	Kalman->P[0][0] -= Kalman->K_0 * Kalman->t_0;
	Kalman->P[0][1] -= Kalman->K_0 * Kalman->t_1;
	Kalman->P[1][0] -= Kalman->K_1 * Kalman->t_0;
	Kalman->P[1][1] -= Kalman->K_1 * Kalman->t_1;
	Kalman->angle += Kalman->K_0 * Kalman->angle_err;    //���ŽǶ�
	Kalman->q_bias += Kalman->K_1 * Kalman->angle_err;
	Kalman->angle_dot = gyro_m - Kalman->q_bias;         //���Ž��ٶ�
	
	return Kalman->angle;
}
