#ifndef CLIMB_H
#define CLIMB_H

#include "stm32f4xx.h"

typedef enum
{
	READY = 0,
	CLIMBING = 1,
	PUTDOWN = 2,
	RETRIEVING = 3,
	CALIB = 4,
}climb_state_t;

void SelectMode(void);
void SelfCalibration(void);
void SupportLinkInit(void);
void SupportLinkControl(uint8_t status);

#endif
