#include "headfile.h"
#ifndef _PID_H
#define _PID_H
int32 left_motor_pid(void);
int32 right_motor_pid(void);
float error_pid(float error);
void pram_init(void);
void speedDifference(float left_v,float right_v);
void control(void);
typedef struct Speed_pram
{
	float KP;
	float KI;
	int16 left_v;
	int16 right_v;
}Speed_pram;


typedef struct Steer_pram
{
	float KP;
	float KD;
	float near_quan;
	float far_quan;
}Steer_pram;
#endif