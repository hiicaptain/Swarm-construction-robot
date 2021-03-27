#ifndef PROTOCOL_H
#define PROTOCOL_H

#include "stm32f4xx.h"

typedef struct Header {
    uint16_t sof;
    uint16_t length;
    uint16_t crc;
} Header;

extern size_t HEADER_LEN;
extern size_t CMD_SET_PREFIX_LEN;
extern size_t CRC_DATA_LEN;
extern size_t CRC_HEAD_LEN;
extern uint8_t SOF;

#pragma pack(push, 1)

//CMD_SET
#define CHASSIS_CMD_SET                (0x02u)
#define Gripper_CMD_SET                 (0x03u)

/*-----------------------------CHASSIS_CMD---- 0x02 ---------------------*/

#define CMD_PUSH_CHASSIS_INFO          (0X01u)
typedef struct {
    int16_t gyro_angle;
    int16_t gyro_rate;
    float position_x_mm;
    float position_y_mm;
    float angle_deg;
    int16_t v_x_mm;
    int16_t v_y_mm;
} cmd_chassis_info;

#define CMD_SET_Gripper_CONTROL				 (0x02U)
typedef struct {
		uint8_t Gripper_mode;
		float yaw_angle;
		float pitch_angle;
		float friction_wheel_speed;
		uint8_t shoot_mode;
		uint32_t shoot_add_num;
		uint16_t shoot_freq;
} cmd_Gripper_control;

#define CMD_SET_CHASSIS_SPD_ACC        (0X05u)
typedef struct {
    int16_t vx;
    int16_t vy;
    int16_t vw;
    float ax;
    float ay;
    float aw;
} cmd_chassis_spd_acc;

/*-----------------------------Gripper_CMD---- 0x03 ---------------------*/

#define CMD_PUSH_Gripper_INFO           (0X01u)
typedef struct {
    uint16_t mode;
    float pitch_ecd_angle;
    float yaw_ecd_angle;
    float yaw_gyro_angle;
		int16_t yaw_offset;
		int16_t pitch_offset;
} cmd_Gripper_info;

#define CMD_SET_Gripper_MODE            (0X02u)
typedef enum {
    CODE_CONTROL,
    GYRO_CONTROL,
    G_MODE_MAX_NUM,
} Gripper_mode_e;

#define CMD_SET_Gripper_ANGLE           (0x03u)
typedef struct{
    uint8_t yaw_mode;
    float pitch;
    float yaw;
} cmd_Gripper_angle;

#define CMD_SET_FRIC_WHEEL_SPEED       (0X04u)
typedef struct{
    uint16_t left;
    uint16_t right;
} cmd_fric_wheel_speed;

#define CMD_SET_SHOOT_INFO             (0x05u)
typedef enum {
    SHOOT_STOP = 0,
    SHOOT_ONCE,
    SHOOT_CONTINUOUS,
} shoot_cmd_e;

typedef struct {
    uint8_t shoot_cmd;
    uint32_t shoot_add_num;
    uint16_t shoot_freq;
} cmd_shoot_info;

#pragma pack(pop)

#endif
