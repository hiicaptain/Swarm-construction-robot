#ifndef CHASSIS_H
#define CHASSIS_H

#include "stm32f4xx.h"

#define WHEEL_RADIUS   0.0425f // m 
#define WHEEL_BASELINE 0.173f  // m

#define CLIMBING_SPEED 0.075

void ChassisInit(void);
void SpeedControl(float v, float w, uint8_t is_enable);

void WheelControl(float target);

#endif
