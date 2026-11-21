#include "velocity_ctrl.h"
#include "current_ctrl.h"
#include "can_bus.h"
#include <math.h>

volatile float now_vel[MOTOR_MAX];
float target_vel[MOTOR_MAX];
float integral[MOTOR_MAX];
float error_now[MOTOR_MAX];
float error_last[MOTOR_MAX];
float error_delta[MOTOR_MAX];
float p_gain[MOTOR_MAX] = MOTOR_P_GAIN;
float i_gain[MOTOR_MAX] = MOTOR_I_GAIN;
float d_gain[MOTOR_MAX] = MOTOR_D_GAIN;
bool pid_first[MOTOR_MAX];

void Vel_Command(int i, float value)
{
	if (!isfinite(value)) value = 0.0f;
	if (value > motor_vel_max[i])  value = motor_vel_max[i];
	if (value < -motor_vel_max[i]) value = -motor_vel_max[i];
	target_vel[i] = value;
}

void vel_PID_control(int i)
{
	if (!Motor_Control_Ready(i))
	{
		current[i]     = 0.0f;
		integral[i]    = 0.0f;
		error_now[i]   = 0.0f;
		error_last[i]  = 0.0f;
		pid_first[i]   = true;
		return;
	}

	error_now[i] = target_vel[i] - now_vel[i];
	if (pid_first[i])
	{
		error_last[i] = error_now[i];
		pid_first[i]  = false;
	}
	error_delta[i] = error_now[i] - error_last[i];

	float integral_limit = INTEGRAL_LIMIT;
	if (i_gain[i] > 0.0f) integral_limit = current_max[i] / i_gain[i];

	if (integral[i] > integral_limit)  integral[i] = integral_limit;
	if (integral[i] < -integral_limit) integral[i] = -integral_limit;

	current[i] = (p_gain[i] * error_now[i]) + (i_gain[i] * integral[i]) + (d_gain[i] * error_delta[i]);

	bool saturating = (current[i] > current_max[i] && error_now[i] > 0.0f) || (current[i] < -current_max[i] && error_now[i] < 0.0f);
	if (!saturating)
	{
		integral[i] += error_now[i] * DT;
	}
	error_last[i] = error_now[i];

	if (current[i] > current_max[i])  current[i] = current_max[i];
	if (current[i] < -current_max[i]) current[i] = -current_max[i];
}
