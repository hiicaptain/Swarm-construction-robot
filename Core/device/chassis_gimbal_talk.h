#ifndef CHASSIS_GIMBAL_TALK_H
#define CHASSIS_GIMBAL_TALK_H

#include "stm32f4xx.h"

extern uint8_t chassis2gimbal_data[100];
void ChassisGimbalTalkCallback(UART_HandleTypeDef *huart);
void ChassisGimbalTalk(uint8_t* data, uint16_t len);

#endif
