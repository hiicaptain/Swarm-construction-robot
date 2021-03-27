#ifndef LA_MOTOR_H
#define LA_MOTOR_H

#include "usart.h"

typedef struct
{
	// ctrl command
	uint8_t  id;             // default 1 1-254
	uint8_t  bandrate;       // default 921600  0-19200 1-115200 2- 57600 3-921600
	uint16_t current_max;    // default 1500  300-1500
	uint16_t position_ref;   // 0-2000
	uint16_t temprature_max; // default 800 [temprature_reset+5, 80]*10
	uint16_t temprature_reset; // default 600 [20, temprature_max-5]*10
	
	uint16_t position_fdb;
	uint16_t current_fdb;
	uint16_t force_fdb;
	uint8_t temprature_fdb;
	uint8_t error_code;
}lamotor_t;

void LAMotorReadTemprature(void);
uint8_t LAMotorReceive(UART_HandleTypeDef *huart, uint8_t *buffer);
uint8_t LAMotorTransmit(UART_HandleTypeDef *huart, uint8_t *buffer, uint16_t len);
uint8_t LAMotorWriteBaudrate(uint8_t id, uint8_t type);
uint8_t LAMotorSetPosition(uint8_t id, uint16_t ref);
uint8_t LAMotorGetState(uint8_t id);
uint8_t LAMotorWriteId(uint8_t id);
uint8_t LAMotorParaFixed(uint8_t id);
uint8_t LAMotorUnpack(uint8_t *buf, uint16_t len);
uint8_t LAMotorReset(uint8_t id);
uint8_t LamCheckSum(uint8_t *buf, uint8_t len, uint8_t sum);
uint8_t LamCalcSum(uint8_t *buf, uint8_t len);

#endif

