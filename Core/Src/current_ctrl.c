#include "current_ctrl.h"
#include <math.h>

float current[MOTOR_MAX];
float current_max[MOTOR_MAX] = MOTOR_CURRENT_MAX;

void Current_Command(int i, float value)
{
	if (!isfinite(value)) value = 0.0f;
	if (value > motor_current_max[i])  value = motor_current_max[i];
	if (value < -motor_current_max[i]) value = -motor_current_max[i];
	current[i] = value;
}
