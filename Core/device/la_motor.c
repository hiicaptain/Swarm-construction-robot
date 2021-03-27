#include "la_motor.h"
#include "string.h"

extern DMA_HandleTypeDef hdma_uart7_rx;
extern DMA_HandleTypeDef hdma_uart7_tx;
extern uint8_t uart7_dma_rx_buffer[UART7_RX_SIZE];
extern uint8_t uart7_dma_tx_buffer[UART7_RX_SIZE];

uint8_t lamotor_rx_buffer[UART7_RX_SIZE];
uint8_t lamotor_tx_buffer[UART7_RX_SIZE];

lamotor_t lamotor1,lamotor2;


uint8_t LAMotorReceive(UART_HandleTypeDef *huart, uint8_t *buffer)
{
	uint32_t tmp = __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE);
	if(tmp != RESET)
	{
		__HAL_UART_CLEAR_IDLEFLAG(huart);
		uint32_t clear;
		clear = huart->Instance->SR;
		clear = huart->Instance->DR;
		HAL_UART_DMAStop(huart);
		
		uint32_t rx_len = UART7_RX_SIZE - __HAL_DMA_GET_COUNTER(&hdma_uart7_rx);
		
		memcpy(buffer, uart7_dma_rx_buffer, rx_len);
		HAL_UART_Receive_DMA(huart, uart7_dma_rx_buffer, UART7_RX_SIZE);
		return rx_len;
	}
	else
		return 0;
}

uint8_t LAMotorTransmit(UART_HandleTypeDef *huart, uint8_t *buffer, uint16_t len)
{
	if(__HAL_DMA_GET_COUNTER(&hdma_uart7_tx) == 0)
	{
		memcpy(uart7_dma_tx_buffer, buffer, len);
		HAL_UART_Transmit_DMA(huart, uart7_dma_tx_buffer, len);
		return 0;
	}
	return 1;
}

uint8_t LAMotorWriteBaudrate(uint8_t id, uint8_t type)  //0-19200 1-115200 2- 57600 3-921600
{
	lamotor_tx_buffer[0] = 0x55;
	lamotor_tx_buffer[1] = 0xAA;  //header
	lamotor_tx_buffer[2] = id;
	lamotor_tx_buffer[3] = 0x02;  //write 
	lamotor_tx_buffer[4] = 0x01;
	lamotor_tx_buffer[5] = 0x62;
	lamotor_tx_buffer[6] = 0x02;
	lamotor_tx_buffer[7] = 0;
	int i;
	for(i = 2; i < 7; ++i)
	{
		lamotor_tx_buffer[7] += lamotor_tx_buffer[i];//
	}
}

uint8_t LAMotorWriteId(uint8_t id) 
{
	lamotor_tx_buffer[0] = 0x55;
	lamotor_tx_buffer[1] = 0xAA;  //header
	lamotor_tx_buffer[2] = id;
	lamotor_tx_buffer[3] = 0x02;  //write 
	lamotor_tx_buffer[4] = 0x01;
	lamotor_tx_buffer[5] = 0x62;
	lamotor_tx_buffer[6] = 0x02;
	lamotor_tx_buffer[7] = 0;
	int i;
	for(i = 2; i < 7; ++i)
	{
		lamotor_tx_buffer[7] += lamotor_tx_buffer[i];
	}
}

uint8_t LAMotorReset(uint8_t id) 
{
	lamotor_tx_buffer[0] = 0x55;
	lamotor_tx_buffer[1] = 0xAA;  //header
	lamotor_tx_buffer[2] = 0x03;
	lamotor_tx_buffer[3] = id;  //write 
	lamotor_tx_buffer[4] = 0x04;
	lamotor_tx_buffer[5] = 0x00;
	lamotor_tx_buffer[6] = 0x1E;
	lamotor_tx_buffer[7] = 0;
	int i;
	for(i = 2; i < 7; ++i)
	{
		lamotor_tx_buffer[7] += lamotor_tx_buffer[i];
	}
	return LAMotorTransmit(&huart7, lamotor_tx_buffer, 8);
}

uint8_t LAMotorSetPosition(uint8_t id, uint16_t ref)  //0-2000
{
	lamotor_tx_buffer[0] = 0x55;
	lamotor_tx_buffer[1] = 0xAA;  //header
	lamotor_tx_buffer[2] = 0x04;
	lamotor_tx_buffer[3] = id;  //write 
	lamotor_tx_buffer[4] = 0x21;
	lamotor_tx_buffer[5] = 0x37;
	lamotor_tx_buffer[6] = (uint8_t)ref;
	lamotor_tx_buffer[7] = (uint8_t)(ref >> 8);
	lamotor_tx_buffer[8] = 0;
//	int i;
//	for(i = 2; i < 8; ++i)
//	{
//		lamotor_tx_buffer[8] += lamotor_tx_buffer[i];
//	}
	
	lamotor_tx_buffer[8] = LamCalcSum(lamotor_tx_buffer, 9);
	return LAMotorTransmit(&huart7, lamotor_tx_buffer, 9);
}

uint8_t LAMotorGetState(uint8_t id)  //0-2000
{
	lamotor_tx_buffer[0] = 0x55;
	lamotor_tx_buffer[1] = 0xAA;  //header
	lamotor_tx_buffer[2] = 0x03;
	lamotor_tx_buffer[3] = id;  //write 
	lamotor_tx_buffer[4] = 0x04;
	lamotor_tx_buffer[5] = 0x00;
	lamotor_tx_buffer[6] = 0x22;
	lamotor_tx_buffer[7] = 0;
	int i;
	for(i = 2; i < 7; ++i)
	{
		lamotor_tx_buffer[7] += lamotor_tx_buffer[i];
	}
	return LAMotorTransmit(&huart7, lamotor_tx_buffer, 8);
}

uint8_t LAMotorParaFixed(uint8_t id)
{
	lamotor_tx_buffer[0] = 0x55;
	lamotor_tx_buffer[1] = 0xAA;  //header
	lamotor_tx_buffer[2] = 0x03;
	lamotor_tx_buffer[3] = id;  //write 
	lamotor_tx_buffer[4] = 0x04;
	lamotor_tx_buffer[5] = 0x00;
	lamotor_tx_buffer[6] = 0x20;
	lamotor_tx_buffer[7] = 0;
	int i;
	for(i = 2; i < 7; ++i)
	{
		lamotor_tx_buffer[7] += lamotor_tx_buffer[i];
	}
	return LAMotorTransmit(&huart7, lamotor_tx_buffer, 8);
}

//uint8_t LAMotorReadTemprature(void)
//{

//	lamotor_tx_buffer[0] = 0x55;
//	lamotor_tx_buffer[1] = 0xAA;
//	lamotor_tx_buffer[2] = 0x03;
//	lamotor_tx_buffer[3] = 0x01;
//	lamotor_tx_buffer[4] = 0x01;
//	lamotor_tx_buffer[5] = 0x62;
//	lamotor_tx_buffer[6] = 0x02;
//	lamotor_tx_buffer[7] = 0;
//	int i;
//	for(i = 2; i < 7; ++i)
//	{
//		lamotor_tx_buffer[7] += lamotor_tx_buffer[i];
//	}
//	return 8;
//}

uint8_t LAMotorUnpack(uint8_t *buf, uint16_t len)
{
	int i = 0;
	if(*(buf) != 0xAA || *(buf+1) != 0x55)
	{
		return 1;
	}
	else
	{
		uint8_t id = *(buf+3);
		uint8_t sum = *(buf+len-1);
		uint8_t check_sum = 0;
		lamotor_t* ptr;
//		if(LamCheckSum(buf, len, sum) == 1)
//		{
			if(id == 1)
			{
				ptr = &lamotor1; 
			}
			ptr->position_fdb = (uint16_t)((buf[10] << 8) | buf[9]);
			ptr->temprature_fdb = buf[11];
			ptr->current_fdb = (uint16_t)((buf[13] << 8) | buf[12]);
			ptr->force_fdb = (uint16_t)((buf[16] << 8) | buf[14]);
			ptr->error_code = buf[15];
			return 1;
//		}
//		else
//			return 0;
	}

	return 0;
}

uint8_t LamCheckSum(uint8_t *buf, uint8_t len, uint8_t sum)
{
	uint8_t i;
	uint8_t calc_sum = 0;
	for(i = 2; i < len - 1; ++i)
	{
		calc_sum += buf[i];
	}
	return (calc_sum == sum);
}


uint8_t LamCalcSum(uint8_t *buf, uint8_t len)
{
	uint8_t i;
	uint8_t calc_sum = 0;
	for(i = 2; i < len - 1; ++i)
	{
		calc_sum += buf[i];
	}
	return calc_sum;
}

