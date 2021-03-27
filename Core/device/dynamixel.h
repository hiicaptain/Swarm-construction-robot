#ifndef _DYNAMIXEL_H_
#define _DYNAMIXEL_H_

#include "main.h"

typedef struct
{
	uint16_t model_number;
	uint8_t firmware;
	uint8_t id;
	uint8_t baudrate;
	uint8_t delay_time;
	uint16_t cw_angle_limit;
	uint16_t ccw_angle_limit;
	uint8_t temp_limit;
	uint8_t min_voltage;
	uint8_t max_voltage;
	uint8_t null1;
	uint16_t max_torque;
	uint8_t status_level;
	uint8_t alarm_led;
	uint8_t shutdown;
	uint8_t null2;
	uint16_t multiterm_offset;
	uint8_t resolution_divider;
	uint8_t null3;
	uint8_t torque_enable;
	uint8_t led_status;
	uint8_t kd;
	uint8_t ki;
	uint8_t kp;
	uint8_t null4;
	uint16_t goal_position;
	uint16_t moving_speed;
	uint16_t torque_limit;
	uint16_t present_position;
	uint16_t present_speed;
	uint16_t present_load;
	uint8_t present_voltage;
	uint8_t present_temperature;
}dynamexiel_data_t;



typedef union
{
	dynamexiel_data_t structure;
	uint8_t           data[44];
}dynamixel_t;



uint8_t DynamixelTransmit(uint8_t len);

void DynamixelReceive(void);
void DynamixelEnable(uint8_t id);
void DynamixelSetMultiTermMode(uint8_t id);
void DynamixelSetSingleTermMode(uint8_t id);
void DynamixelSetSpeedMode(uint8_t id);
void DynamixelMsgUnpack(uint8_t *buf, uint8_t len);
void DynamixelRamUnpack(uint8_t *buf, uint8_t len);
void DynamixelReadState(uint8_t id);
void DynamixelReadTemperature(uint8_t id);
void DynamixelReadPosition(void);
void DynamixelWriteSpeed(uint8_t id, uint16_t spd);
void DynamixelWritePosition(uint8_t id, uint16_t pos);
void DynamixelWritePositionAndSpeed(uint8_t id, uint16_t pos, uint16_t spd);
void DynamixelChangeID(uint8_t id, uint8_t new_id);
void DynamixelReadFirmware(uint8_t id);
uint8_t DymCheckSum(uint8_t *buf, uint8_t len, uint8_t sum);
uint8_t DymCalcSum(uint8_t *buf, uint8_t len);
void DynamixelReadMsg(uint8_t id);
void DynamixelReset(uint8_t id);

#endif

