#ifndef CAN_MOTOR_H
#define CAN_MOTOR_H

#include "can.h"
#include "controller.h"

#define CAN_MOTOR5_ID 0x205
#define CAN_MOTOR6_ID 0x206
#define CAN_MOTOR7_ID 0x207

#define yaw_motor 	can1_motor_5
#define pitch_motor can1_motor_6
#define pluck_motor can1_motor_7

struct CAN_Motor
{
	cascade_pid_t cascade_pid;
	float angle;
	float angle_inf;
	float angular_spd;
  int   round;              //���ת����Ȧ��	
	int   encoder_inf;        //���㴦���ĵ��ת��λ��
	short encoder_offset;
  short encoder;            //����ı���������ֵ
  short last_encoder;       //����ϴεı���������ֵ
  short angular_spd_rpm;    //���������ת��/rpm
};

extern struct CAN_Motor can1_motor_5;
extern struct CAN_Motor can1_motor_6;
extern struct CAN_Motor can1_motor_7;

HAL_StatusTypeDef CanFilterInit(CAN_HandleTypeDef* hcan);
void CanTransmit_1234(CAN_HandleTypeDef *hcanx, int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq);
void CanTransmit_5678(CAN_HandleTypeDef *hcanx, int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq);
void MotorParamInit(struct CAN_Motor *motor,float speedP,float speedI,float speedD,int speedOutmax,
	    int speederrormax,float positionP,float positionI,float positionD,int positionOutmax,int positionerrormax);


void MotorMsgUnpack(encoder_t *encoder, uint8_t *buffer, uint8_t type);

#endif
