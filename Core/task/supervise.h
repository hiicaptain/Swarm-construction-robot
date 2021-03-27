#ifndef SUPERVISE_H_
#define SUPERVISE_H_

#define DETECT_NUM 6

#define GYRO_INDEX 0
#define CAN_MOTOR5_INDEX 1
#define CAN_MOTOR6_INDEX 2
#define CAN_MOTOR7_INDEX 3
#define CAN_MOTOR9_INDEX 4
#define CHASSIS_INDEX 5

void SuperviseTaskHandle(void);
void LostCounterFeed(int index);
int Is_Error(int index);
int Is_Serious_Error(void);
int Is_Any_Error(void);

#endif
