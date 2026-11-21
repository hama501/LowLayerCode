#include "grip_ctrl.h"
#include "current_ctrl.h"
#include "velocity_ctrl.h"
#include "position_ctrl.h"
#include "can_bus.h"
#include <math.h>

GripState_t grip_state[MOTOR_MAX];
volatile uint8_t grip_mode_cmd[MOTOR_MAX];
float grip_open_target_x[MOTOR_MAX] = {GRIP_OPEN_TARGET_X, GRIP_OPEN_TARGET_X, GRIP_OPEN_TARGET_X, GRIP_OPEN_TARGET_X};
uint32_t squeeze_start_tick[MOTOR_MAX];
float squeeze_start_current[MOTOR_MAX];
uint32_t hold_start_tick[MOTOR_MAX];
uint8_t grip_fault[MOTOR_MAX];
float current_alarm_limit = CURRENT_ALARM_LIMIT;
static float grip_stable_ref_x[MOTOR_MAX];
static uint32_t grip_stable_start_tick[MOTOR_MAX];

void Grip_Command(int i, uint8_t value)
{
	grip_mode_cmd[i] = value;
}

void Grip_Open_Command(int i, float value)
{
	if (!isfinite(value)) return;
	grip_open_target_x[i] = value;
}

float Grip_Feedback_Flag(int i)
{
	float flag = 0.0f;
	if (grip_state[i] == GRIP_HOLD || grip_state[i] == GRIP_APPROACH)
	{
		flag += 1.0f;
	}
#if CURRENT_ALARM_MOTOR >= 0
	if (fabsf(fb_current[CURRENT_ALARM_MOTOR]) >= current_alarm_limit)
	{
		flag += 0.1f;
	}
#endif
	return flag;
}

static void Grip_Reset_Axis_Control(int i)
{
	integral[i]    = 0.0f;
	error_now[i]   = 0.0f;
	error_last[i]  = 0.0f;
	target_vel[i]  = 0.0f;
	is_stopped[i]  = false;
	pid_first[i]   = true;
}

static void Grip_Enter_Approach(int i)
{
	Grip_Reset_Axis_Control(i);
	target_x[i] = GRIP_CLOSE_TARGET_X;
	current_max[i] = APPROACH_CURRENT_MAX;
	grip_fault[i] = 0;
	grip_stable_ref_x[i] = now_x[i];
	grip_stable_start_tick[i] = HAL_GetTick();
	grip_state[i] = GRIP_APPROACH;
}

static void Grip_Enter_Squeeze(int i)
{
	Grip_Reset_Axis_Control(i);
	squeeze_start_tick[i] = HAL_GetTick();
	squeeze_start_current[i] = current[i];
	grip_state[i] = GRIP_SQUEEZE;
}

static void Grip_Enter_Release(int i)
{
	Grip_Reset_Axis_Control(i);
	target_x[i] = grip_open_target_x[i];
	current_max[i] = APPROACH_CURRENT_MAX;
	grip_fault[i] = 0;
	grip_state[i] = GRIP_RELEASE;
}

static void Grip_Enter_Off(int i)
{
	Grip_Reset_Axis_Control(i);
	current_max[i] = APPROACH_CURRENT_MAX;
	grip_state[i] = GRIP_OFF;
}

void Grip_Axis_Update(int i)
{
	if (grip_mode_cmd[i] == 1 && (grip_state[i] == GRIP_OFF || grip_state[i] == GRIP_RELEASE))
	{
		Grip_Enter_Approach(i);
	}
	else if (grip_mode_cmd[i] == 0 &&
	         (grip_state[i] == GRIP_APPROACH || grip_state[i] == GRIP_SQUEEZE || grip_state[i] == GRIP_HOLD))
	{
		Grip_Enter_Release(i);
	}

	switch (grip_state[i])
	{
	case GRIP_OFF:
	case GRIP_APPROACH:
	case GRIP_RELEASE:
	{
		Pos_P_control(i);
		vel_PID_control(i);

		if (grip_state[i] == GRIP_APPROACH)
		{
			if (!Motor_Control_Ready(i) ||
			    fabsf(now_x[i] - grip_stable_ref_x[i]) > GRIP_HOLD_STABLE_EPS)
			{
				grip_stable_ref_x[i] = now_x[i];
				grip_stable_start_tick[i] = HAL_GetTick();
			}
			else if (HAL_GetTick() - grip_stable_start_tick[i] >= GRIP_HOLD_STABLE_TIME_MS)
			{
				Grip_Enter_Squeeze(i);
			}
		}
		else if (grip_state[i] == GRIP_RELEASE && is_stopped[i])
		{
			Grip_Enter_Off(i);
		}
		break;
	}

	case GRIP_SQUEEZE:
	{
		float squeeze_target_current = -SQUEEZE_CURRENT_MAX;
		uint32_t elapsed = HAL_GetTick() - squeeze_start_tick[i];
		if (elapsed >= SQUEEZE_RAMP_TIME_MS)
		{
			current[i] = squeeze_target_current;
			hold_start_tick[i] = HAL_GetTick();
			grip_state[i] = GRIP_HOLD;
		}
		else
		{
			float ratio = (float)elapsed / (float)SQUEEZE_RAMP_TIME_MS;
			current[i] = squeeze_start_current[i]
			           + (squeeze_target_current - squeeze_start_current[i]) * ratio;
		}
		break;
	}

	case GRIP_HOLD:
	{
		current[i] = -HOLD_CURRENT;

		if (fb_temperature[i] >= TEMP_WARNING_C)
		{
			grip_fault[i] = 1;
		}
		if (grip_fault[i])
		{
			current[i] = -HOLD_CURRENT * 0.3f;
		}
		break;
	}

	default:
		Grip_Enter_Off(i);
		break;
	}
}
