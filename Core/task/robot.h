#ifndef ROBOT_H
#define ROBOT_H

#include "protocol.h"

struct Robot_t
{
	enum work_state_e
	{
		Stop,
		Follow_Encoder,
		Follow_Gyro
	} Gripper_state;
	cmd_Gripper_control Gripper_control_info;
	cmd_Gripper_info Gripper_info;
};

struct ramp_t
{
  int count;
  int scale;
  float   out;
  void  (*init)(struct ramp_t *ramp, int scale);
  float (*calc)(struct ramp_t *ramp);
};

void RampInit(struct ramp_t *ramp, int scale);
float RampCalc(struct ramp_t *ramp);

extern struct Robot_t infantry;
extern struct ramp_t FrictionLeftWheel;
extern struct ramp_t FrictionRightWheel;
extern int16_t pitch_offset, yaw_offset;

void RobotParamInit(void);
void GripperStateChange(void);

#endif
