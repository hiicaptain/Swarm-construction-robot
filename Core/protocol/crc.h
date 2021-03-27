#ifndef CRC_H
#define CRC_H

#include "stm32f4xx.h"

uint16_t CRC16Update(uint16_t crc, uint8_t ch);
uint32_t CRC32Update(uint32_t crc, uint8_t ch);
uint16_t CRC16Calc(const uint8_t *data_ptr, size_t length);
uint32_t CRC32Calc(const uint8_t *data_ptr, size_t length);
int CRCHeadCheck(uint8_t *data_ptr, size_t length);
int CRCTailCheck(uint8_t *data_ptr, size_t length);

#endif
