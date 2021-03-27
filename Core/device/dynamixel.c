#include "dynamixel.h"
#include "usart.h"
#include "string.h"

uint8_t dynamixel_rx_buffer[UART4_RX_SIZE];
uint8_t dynamixel_tx_buffer[UART4_RX_SIZE] = {0xFF, 0xFF};

const uint16_t servo1_set_encoder = 1425;
const uint16_t servo1_reset_encoder = 278;

const uint16_t servo2_set_encoder = 2231;
const uint16_t servo2_reset_encoder = 2894;

extern uint8_t uart4_dma_rx_buffer[UART4_RX_SIZE];
extern uint8_t uart4_dma_tx_buffer[UART4_RX_SIZE];

dynamixel_t servo1;
dynamixel_t servo2;

uint8_t DynamixelTransmit(uint8_t len)
{
	HAL_HalfDuplex_EnableTransmitter(&huart4);
	memcpy(uart4_dma_tx_buffer, dynamixel_tx_buffer, len);
	HAL_StatusTypeDef res = HAL_UART_Transmit_IT(&huart4, uart4_dma_tx_buffer, len);
	if(res != HAL_OK)
		return 1;
	return 0;
}

void DynamixelReceive(void)
{
	
}

void DynamixelEnable(uint8_t id)
{
	dynamixel_tx_buffer[0] = 0xFF;
	dynamixel_tx_buffer[1] = 0xFF;
	dynamixel_tx_buffer[2] = id;
	dynamixel_tx_buffer[3] = 0x04;
	dynamixel_tx_buffer[5] = 0x18;
	dynamixel_tx_buffer[4] = 0x03; // write
	dynamixel_tx_buffer[6] = 0x00; //   3: position mode 0-4095   4:  extended mode -512r-512r
	dynamixel_tx_buffer[7] = DymCalcSum(dynamixel_tx_buffer, 8);
	uint8_t res =  DynamixelTransmit(8);
}

void DynamixelReset(uint8_t id)
{
	dynamixel_tx_buffer[0] = 0xFF;
	dynamixel_tx_buffer[1] = 0xFF;
	dynamixel_tx_buffer[2] = id;
	dynamixel_tx_buffer[3] = 0x02;
	dynamixel_tx_buffer[4] = 0x06; 
	dynamixel_tx_buffer[5] = DymCalcSum(dynamixel_tx_buffer, 6);
	uint8_t res =  DynamixelTransmit(6);
}

void DynamixelSetMultiTermMode(uint8_t id)
{
	dynamixel_tx_buffer[0] = 0xFF;
	dynamixel_tx_buffer[1] = 0xFF;
	dynamixel_tx_buffer[2] = id;
	dynamixel_tx_buffer[3] = 0x07;
	dynamixel_tx_buffer[4] = 0x03; // write
	dynamixel_tx_buffer[5] = 0x06;
	dynamixel_tx_buffer[6] = (uint8_t)4095; 
	dynamixel_tx_buffer[7] = (uint8_t)(4095 >> 8);
	dynamixel_tx_buffer[8] = (uint8_t)4095; 
	dynamixel_tx_buffer[9] = (uint8_t)(4095 >> 8);
	dynamixel_tx_buffer[10] = DymCalcSum(dynamixel_tx_buffer, 11);
	
	uint8_t res =  DynamixelTransmit(11);
}

void DynamixelSetSingleTermMode(uint8_t id)
{
	dynamixel_tx_buffer[0] = 0xFF;
	dynamixel_tx_buffer[1] = 0xFF;
	dynamixel_tx_buffer[2] = id;
	dynamixel_tx_buffer[3] = 0x07;
	dynamixel_tx_buffer[4] = 0x03; // write
	dynamixel_tx_buffer[5] = 0x06;
	dynamixel_tx_buffer[6] = (uint8_t)0; 
	dynamixel_tx_buffer[7] = (uint8_t)(0 >> 8);
	dynamixel_tx_buffer[8] = (uint8_t)4095; 
	dynamixel_tx_buffer[9] = (uint8_t)(4095 >> 8);
	dynamixel_tx_buffer[10] = DymCalcSum(dynamixel_tx_buffer, 11);
	
	uint8_t res =  DynamixelTransmit(11);
}

void DynamixelSetSpeedMode(uint8_t id)
{
	dynamixel_tx_buffer[0] = 0xFF;
	dynamixel_tx_buffer[1] = 0xFF;
	dynamixel_tx_buffer[2] = id;
	dynamixel_tx_buffer[3] = 0x07;
	dynamixel_tx_buffer[4] = 0x03; // write
	dynamixel_tx_buffer[5] = 0x06;
	dynamixel_tx_buffer[6] = (uint8_t)0; 
	dynamixel_tx_buffer[7] = (uint8_t)(0 >> 8);
	dynamixel_tx_buffer[8] = (uint8_t)0; 
	dynamixel_tx_buffer[9] = (uint8_t)(0 >> 8);
	dynamixel_tx_buffer[10] = DymCalcSum(dynamixel_tx_buffer, 11);
	
	uint8_t res =  DynamixelTransmit(11);
}

void DynamixelReadTemperature(uint8_t id)
{
	dynamixel_tx_buffer[0] = 0xFF;
	dynamixel_tx_buffer[1] = 0xFF;
	dynamixel_tx_buffer[2] = id;
	dynamixel_tx_buffer[3] = 0x04;
	dynamixel_tx_buffer[4] = 0x02;
	dynamixel_tx_buffer[5] = 0x2B;
	dynamixel_tx_buffer[6] = 0x01;
	dynamixel_tx_buffer[7] = DymCalcSum(dynamixel_tx_buffer, 8);
	uint8_t res =  DynamixelTransmit(8);
}

void DynamixelChangeID(uint8_t id, uint8_t new_id)
{
	dynamixel_tx_buffer[0] = 0xFF;
	dynamixel_tx_buffer[1] = 0xFF;
	dynamixel_tx_buffer[2] = id;
	dynamixel_tx_buffer[3] = 0x04;
	dynamixel_tx_buffer[4] = 0x03;
	dynamixel_tx_buffer[5] = 0x03;
	dynamixel_tx_buffer[6] = new_id;
	dynamixel_tx_buffer[7] = DymCalcSum(dynamixel_tx_buffer, 8);
	uint8_t res =  DynamixelTransmit(8);
}

void DynamixelWritePosition(uint8_t id, uint16_t pos)
{
	dynamixel_tx_buffer[0] = 0xFF;
	dynamixel_tx_buffer[1] = 0xFF;
	dynamixel_tx_buffer[2] = id;
	dynamixel_tx_buffer[3] = 0x05;
	dynamixel_tx_buffer[4] = 0x03; //write
	dynamixel_tx_buffer[5] = 0x1E;
	dynamixel_tx_buffer[6] = (uint8_t)pos;
	dynamixel_tx_buffer[7] = (uint8_t)(pos>>8);
	dynamixel_tx_buffer[8] = DymCalcSum(dynamixel_tx_buffer, 9);
	uint8_t res =  DynamixelTransmit(9);
}

void DynamixelWriteSpeed(uint8_t id, uint16_t spd)
{
	dynamixel_tx_buffer[0] = 0xFF;
	dynamixel_tx_buffer[1] = 0xFF;
	dynamixel_tx_buffer[2] = id;
	dynamixel_tx_buffer[3] = 0x05;
	dynamixel_tx_buffer[4] = 0x03; //write
	dynamixel_tx_buffer[5] = 0x20;
	dynamixel_tx_buffer[6] = (uint8_t)spd;
	dynamixel_tx_buffer[7] = (uint8_t)(spd>>8);
	dynamixel_tx_buffer[8] = DymCalcSum(dynamixel_tx_buffer, 9);
	uint8_t res =  DynamixelTransmit(9);
}

void DynamixelWritePositionAndSpeed(uint8_t id, uint16_t pos, uint16_t spd)
{
	dynamixel_tx_buffer[0] = 0xFF;
	dynamixel_tx_buffer[1] = 0xFF;
	dynamixel_tx_buffer[2] = id;
	dynamixel_tx_buffer[3] = 0x07;
	dynamixel_tx_buffer[4] = 0x03; // write
	dynamixel_tx_buffer[5] = 0x1E; // global addr
	dynamixel_tx_buffer[6] = (uint8_t)pos;
	dynamixel_tx_buffer[7] = (uint8_t)(pos>>8);
	dynamixel_tx_buffer[8] = (uint8_t)spd;
	dynamixel_tx_buffer[9] = (uint8_t)(spd>>8);
	dynamixel_tx_buffer[10] = DymCalcSum(dynamixel_tx_buffer, 11);
	uint8_t res =  DynamixelTransmit(11);
}

void DynamixelReadPosition(void)
{
	dynamixel_tx_buffer[0] = 0xFF;
	dynamixel_tx_buffer[1] = 0xFF;
	dynamixel_tx_buffer[2] = 0xFE;
	dynamixel_tx_buffer[3] = 0x09;
	dynamixel_tx_buffer[4] = 0x92; // bulk read
	dynamixel_tx_buffer[5] = 0x00;
	dynamixel_tx_buffer[6] = 0x02;
	dynamixel_tx_buffer[7] = 0x01;
	dynamixel_tx_buffer[8] = 0x1E;
	dynamixel_tx_buffer[9] = 0x02;
	dynamixel_tx_buffer[10] = 0x02;
	dynamixel_tx_buffer[11] = 0x24;
	dynamixel_tx_buffer[12] = DymCalcSum(dynamixel_tx_buffer, 13);
	uint8_t res =  DynamixelTransmit(13);
}

void DynamixelReadMsg(uint8_t id)
{
	dynamixel_tx_buffer[0] = 0xFF;
	dynamixel_tx_buffer[1] = 0xFF;
	dynamixel_tx_buffer[2] = id;
	dynamixel_tx_buffer[3] = 0x04;
	dynamixel_tx_buffer[4] = 0x02;
	dynamixel_tx_buffer[5] = 0x00;
	dynamixel_tx_buffer[6] = 0x2C;
	dynamixel_tx_buffer[7] = DymCalcSum(dynamixel_tx_buffer, 8);
	uint8_t res =  DynamixelTransmit(8);
}

void DynamixelReadRam(uint8_t id)
{
	dynamixel_tx_buffer[0] = 0xFF;
	dynamixel_tx_buffer[1] = 0xFF;
	dynamixel_tx_buffer[2] = id;
	dynamixel_tx_buffer[3] = 0x04;
	dynamixel_tx_buffer[4] = 0x02;
	dynamixel_tx_buffer[5] = 0x18;
	dynamixel_tx_buffer[6] = 0x14;
	dynamixel_tx_buffer[7] = DymCalcSum(dynamixel_tx_buffer, 8);
	uint8_t res =  DynamixelTransmit(8);
}

void DynamixelReadFirmware(uint8_t id)
{
	dynamixel_tx_buffer[0] = 0xFF;
	dynamixel_tx_buffer[1] = 0xFF;
	dynamixel_tx_buffer[2] = id;
	dynamixel_tx_buffer[3] = 0x04;
	dynamixel_tx_buffer[4] = 0x02;
	dynamixel_tx_buffer[5] = 0x00;
	dynamixel_tx_buffer[6] = 0x03;
	dynamixel_tx_buffer[7] = DymCalcSum(dynamixel_tx_buffer, 8);
	uint8_t res =  DynamixelTransmit(8);
}


void DynamixelMsgUnpack(uint8_t *buf, uint8_t len)
{
	if(buf[0] == 0xFF && buf[1] == 0xFF && DymCheckSum(buf, len, buf[len-1])) // header and crc check
	{
		if(buf[3] == len - 4)  //length check
		{
			uint8_t id = buf[2];
			switch(id)
			{
				case 0x01:
					memcpy(servo1.data, &buf[5], 44);
					break;
				
				case 0x02:
					memcpy(servo2.data, &buf[5], 44);
					break;
				
				default:
					break;
			}
		}
	}
}


uint8_t DymCheckSum(uint8_t *buf, uint8_t len, uint8_t sum)
{
	uint8_t i;
	uint8_t calc_sum = 0;
	for(i = 2; i < len - 1; ++i)
	{
		calc_sum += buf[i];
	}
	calc_sum = ~calc_sum;
	return (calc_sum == sum);
}

uint8_t DymCalcSum(uint8_t *buf, uint8_t len)
{
	uint8_t i;
	uint8_t calc_sum = 0;
	for(i = 2; i < len - 1; ++i)
	{
		calc_sum += buf[i];
	}
	calc_sum = ~calc_sum;
	return calc_sum;
}
