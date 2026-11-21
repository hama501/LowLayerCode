#include "motor_config.h"
#include "can_send.h"
#include "current_ctrl.h"
#include "velocity_ctrl.h"
#include "position_ctrl.h"
#include "grip_ctrl.h"
#include "io_ctrl.h"

volatile ModeState_t mode[MOTOR_MAX];

const float motor_direction[MOTOR_MAX]   = MOTOR_DIRECTION;
const float motor_gear_ratio[MOTOR_MAX]  = MOTOR_GEAR_RATIO;
const float motor_gear_length[MOTOR_MAX] = MOTOR_GEAR_LENGTH;
const float motor_current_max[MOTOR_MAX] = MOTOR_CURRENT_MAX;
const float motor_vel_max[MOTOR_MAX]     = MOTOR_VEL_MAX;
const uint8_t motor_vesc_rpm[MOTOR_MAX]  = MOTOR_VESC_RPM;

volatile int8_t motor_config_error = -1;	//設定ミスのあった軸の番号(-1: なし、9: リミットスイッチの設定)

void ResetTask(int i)
{
	integral[i] = 0;
	error_now[i] = 0;
	error_last[i] = 0;
	current[i] = 0;
	pid_first[i] = true;
	is_stopped[i] = false;
}

//設定ミスがないか確認する(ミスがあればfalse)
static bool Motor_Config_Valid(int i)
{
	if ((mode[i] == POSITION || mode[i] == GRIP) && !Motor_Has_Encoder(i)) return false;

	for (int j = 0; j < i; j++)
	{
		bool same_bus_type = (motor_driver[i] == DRIVER_VESC) == (motor_driver[j] == DRIVER_VESC);
		if (same_bus_type && motor_can2_id[i] == motor_can2_id[j]) return false;
		if (Motor_Encoder_Tim(i) != NULL && Motor_Encoder_Tim(i) == Motor_Encoder_Tim(j)) return false;
	}
	return true;
}

void Motor_Mode_Init(void)
{
	for (int i = 0; i < MOTOR_NUM; i++)
	{
		if (i < CURRENT_MOTOR_NUM)
		{
			mode[i] = CURRENT;
		}
		else if (i < CURRENT_MOTOR_NUM + VELOCITY_MOTOR_NUM)
		{
			mode[i] = VELOCITY;
		}
		else if (i < CURRENT_MOTOR_NUM + VELOCITY_MOTOR_NUM + POSITION_MOTOR_NUM)
		{
			mode[i] = POSITION;
		}
		else
		{
			mode[i] = GRIP;
		}
		ResetTask(i);
		target_vel[i]  = 0.0f;
		target_x[i]    = 0.0f;
		current_max[i] = motor_current_max[i];
		if (mode[i] == GRIP)
		{
			target_x[i]      = grip_open_target_x[i];
			current_max[i]   = APPROACH_CURRENT_MAX;
			grip_state[i]    = GRIP_OFF;
			grip_mode_cmd[i] = 0;
		}
	}

	for (int i = 0; i < MOTOR_NUM; i++)
	{
		if (!Motor_Config_Valid(i))
		{
			motor_config_error = (int8_t)i;
			Error_Handler();
		}
	}

	if (!Limit_Switch_Config_Valid())
	{
		motor_config_error = 9;
		Error_Handler();
	}
}
