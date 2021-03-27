#include "chassis.h"
#include "robot.h"
#include "can_motor.h"
#include "math.h"
#include "controller.h"

pid_t motor_left_pid, motor_right_pid;
encoder_t motor_left_encoder, motor_right_encoder, motor_climb_encoder;

flpid_t wheel_left, wheel_right;

tob_t motor_left_tob;

const uint16_t m2006_ratio = 36;

void ChassisInit(void)
{
	pid_init(&motor_left_pid, 40, 0.1, 0, 10000, 200);
	pid_init(&motor_right_pid, 40, 0.1, 0, 10000, 200);
	
	tob_init(&motor_left_tob, 0.001, 1, 200);
	
	flpid_init(&wheel_left, 40, 0.1, 0, 2000, 10000, 0.001, 1, 200);
	flpid_init(&wheel_right, 40, 0.1, 0, 2000, 10000, 0.001, 1, 200);
}

void SpeedControl(float v, float w, uint8_t is_enable) // m/s
{
	// speed planning
	float wd_wheel_left = v / WHEEL_RADIUS;
	float wd_wheel_right = -v / WHEEL_RADIUS;
	float wr_wheel_left = -w * 0.5f * WHEEL_BASELINE / WHEEL_RADIUS;
	float wr_wheel_right = -w * 0.5f * WHEEL_BASELINE / WHEEL_RADIUS;
	
	// loop control
//	pid_calc(&motor_left_pid, wd_wheel_left + wr_wheel_left, motor_left_encoder.omiga);
//	pid_calc(&motor_right_pid, wd_wheel_right + wr_wheel_right, motor_right_encoder.omiga);
	
	flpid_calc(&wheel_left, (wd_wheel_left + wr_wheel_left) * m2006_ratio, motor_left_encoder.omiga, is_enable);
	flpid_calc(&wheel_right, (wd_wheel_right + wr_wheel_right) * m2006_ratio, motor_right_encoder.omiga, is_enable);
	

}

float test_current;
flpd2d_t wheel;
flcpid2d_t wheel_cpid;
uint8_t is_enable = 0;
void WheelControl(float target)
{
	
//	tob_calc(&motor_left_tob, motor_left_encoder.omiga, test_current);
//	pid_calc(&motor_left_pid, target, motor_left_encoder.omiga);
//	test_current = motor_left_pid.output + motor_left_tob.tl_hat;
//	test_current = sat(test_current, -motor_left_pid.output_max, motor_left_pid.output_max);
//	
	
//	flpd2d_calc(&wheel, target, motor_left_encoder.theta_inf, motor_left_encoder.omiga, is_enable);
	flcpid2d_calc(&wheel_cpid, target, motor_left_encoder.theta_inf, motor_left_encoder.omiga, is_enable);
	CanTransmit_1234(&hcan1, (int16_t)(wheel_cpid.output), 0, 0, 0);
}

