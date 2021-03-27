#include "chassis_gimbal_talk.h"
#include "DataHandle.h"
#include "supervise.h"

uint8_t chassis2gimbal_data[100];

void ChassisGimbalTalkCallback(UART_HandleTypeDef *huart)
{
	uint16_t len;
	if(__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))
	{
		__HAL_UART_CLEAR_IDLEFLAG(huart);
		(void)UART7->SR;
		(void)UART7->DR;
		__HAL_DMA_CLEAR_FLAG(huart, DMA_FLAG_DMEIF3_7);
		HAL_UART_DMAStop(huart);
		len = 500 - __HAL_DMA_GET_COUNTER(huart->hdmarx);
		ChassisGimbalTalk(chassis2gimbal_data, len);
		LostCounterFeed(CHASSIS_INDEX);
		HAL_UART_Receive_DMA(huart, chassis2gimbal_data, 100);
	}
}

void ChassisGimbalTalk(uint8_t* data, uint16_t len)
{
	if(VerifyData(data))
	{
			Header *header_ptr = (Header*)data;
			ContainerHandler(data[HEADER_LEN], data[HEADER_LEN+1], &data[HEADER_LEN+2], header_ptr->length);
	}
}
