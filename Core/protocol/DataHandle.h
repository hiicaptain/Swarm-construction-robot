#ifndef DATAHANDLE_H
#define DATAHANDLE_H

#include "crc.h"
#include "protocol.h"
#include "robot.h"

int VerifyData(uint8_t* data);
void ContainerHandler(uint8_t cmd_set, uint8_t cmd_id, uint8_t* data, uint16_t len);
uint16_t DataPack(uint8_t cmd_set, uint8_t cmd_id, uint8_t *p_data, uint16_t len, uint8_t *tx_buf);
void RecvDataCallback(uint8_t *data, uint16_t len);

#endif
