#ifndef _HC_SR04_H_
#define _HC_SR04_H_

#include "main.h"
#include "usart.h"
//typedef struct 
//{
//	float distance
//}hcsr04_t;

void HCSR04Unpack(uint8_t *buf);
uint8_t HCSR04GetDistance(void);
uint8_t HCSR04Receive(UART_HandleTypeDef *huart, uint8_t *buffer);
uint8_t HCSR04Transmit(UART_HandleTypeDef *huart, uint8_t *buffer, uint16_t len);

#endif
