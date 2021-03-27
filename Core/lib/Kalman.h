/**
  ******************************************************************************
  * 文件名        ：Kalman.h
	* 创建时间      ：2019.11.24
	* 作者          ：刘文熠
	*-----------------------------------------------------------------------------
	* 最近修改时间  ：2019.11.24
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

#ifndef _KALMAN_H_
#define _KALMAN_H_

typedef struct 
{
	float dt;            //卡尔曼滤波器采样时间
	float angle;
	float angle_dot;     //角度及角速度
	float P[2][2];
	float Pdot[4];
	float Q_angle;
	float Q_gyro;        //角度置信度，角速度置信度
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
