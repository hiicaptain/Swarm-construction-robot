#include "hc_sr04.h"
#include "string.h"

uint8_t hcsr04_rx_buffer[UART8_RX_SIZE];
uint8_t hcsr04_tx_buffer[UART8_RX_SIZE] = {0xFD, 0xCC, 0x00, 0xDF};

extern uint8_t uart8_dma_rx_buffer[UART8_RX_SIZE];
extern uint8_t uart8_dma_tx_buffer[UART8_RX_SIZE];

extern DMA_HandleTypeDef hdma_uart8_tx;
extern DMA_HandleTypeDef hdma_uart8_rx;

float distance = 0; // m

void HCSR04Unpack(uint8_t *buf)
{
	if(buf[0] == 0xFD && buf[3] == 0xDF)
	{
		distance = buf[1] + 0.01f * buf[2];
	}
}

uint8_t HCSR04GetDistance(void)
{
	uint8_t command[4] = {0xFD, 0xCC, 0x00, 0xDF};
	return HCSR04Transmit(&huart8, command, 4);
}

uint8_t HCSR04Receive(UART_HandleTypeDef *huart, uint8_t *buffer)
{
	uint32_t tmp = __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE);
	if(tmp != RESET)
	{
		__HAL_UART_CLEAR_IDLEFLAG(huart);
		uint32_t clear;
		clear = huart->Instance->SR;
		clear = huart->Instance->DR;
		HAL_UART_DMAStop(huart);
		
		uint32_t rx_len = UART8_RX_SIZE - __HAL_DMA_GET_COUNTER(&hdma_uart8_rx);
		
		memcpy(buffer, uart8_dma_rx_buffer, rx_len);
		HAL_UART_Receive_DMA(huart, uart8_dma_rx_buffer, UART8_RX_SIZE);
		return rx_len;
	}
	else
		return 0;
}

uint8_t HCSR04Transmit(UART_HandleTypeDef *huart, uint8_t *buffer, uint16_t len)
{
	if(__HAL_DMA_GET_COUNTER(&hdma_uart8_tx) == 0)
	{
		memcpy(uart8_dma_tx_buffer, buffer, len);
		HAL_UART_Transmit_DMA(huart, uart8_dma_tx_buffer, len);
		return 0;
	}
	return 1;
}


