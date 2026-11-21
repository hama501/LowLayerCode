#ifndef VELOCITY_CTRL_H
#define VELOCITY_CTRL_H

#include "motor_config.h"

#define INTEGRAL_LIMIT	  	150.0f

extern volatile float now_vel[MOTOR_MAX];
extern float target_vel[MOTOR_MAX];
extern float integral[MOTOR_MAX];
extern float error_now[MOTOR_MAX];
extern float error_last[MOTOR_MAX];
extern float error_delta[MOTOR_MAX];
extern float p_gain[MOTOR_MAX];
extern float i_gain[MOTOR_MAX];
extern float d_gain[MOTOR_MAX];
extern bool pid_first[MOTOR_MAX];

void Vel_Command(int i, float value);
void vel_PID_control(int i);

#endif
