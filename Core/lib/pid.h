#ifndef PID_H_
#define PID_H_

struct PID_t
{
	float KP;
	float KI;
	float KD;
	float error[3];
	float error_sum;
	float fdb;
	float ref;
	float output;
	int outputMax;
	int ErrorMax;
};

#define DEFAULT_PID \
{0.0f,0.0f,0.0f,{0,0,0},0,0,0,0,0, \
}

void PID_Calc(struct PID_t *pid);

#endif
