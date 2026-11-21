#ifndef GRIP_CTRL_H
#define GRIP_CTRL_H

#include "motor_config.h"

#define GRIP_CLOSE_TARGET_X			0.0f
#define GRIP_OPEN_TARGET_X			3.0f
#define APPROACH_CURRENT_MAX		4000.0f
#define SQUEEZE_CURRENT_MAX			3000.0f
#define SQUEEZE_RAMP_TIME_MS		250u
#define HOLD_CURRENT				4500.0f
#define TEMP_WARNING_C				70.0f
#define GRIP_HOLD_STABLE_TIME_MS	100u
#define GRIP_HOLD_STABLE_EPS		0.5f

#define CURRENT_ALARM_MOTOR			-1
#define CURRENT_ALARM_LIMIT			8800.0f

typedef enum {
    GRIP_OFF = 0,
    GRIP_APPROACH,
    GRIP_SQUEEZE,
    GRIP_HOLD,
    GRIP_RELEASE
} GripState_t;

extern GripState_t grip_state[MOTOR_MAX];
extern volatile uint8_t grip_mode_cmd[MOTOR_MAX];
extern float grip_open_target_x[MOTOR_MAX];
extern uint32_t squeeze_start_tick[MOTOR_MAX];
extern float squeeze_start_current[MOTOR_MAX];
extern uint32_t hold_start_tick[MOTOR_MAX];
extern uint8_t grip_fault[MOTOR_MAX];
extern float current_alarm_limit;

void Grip_Command(int i, uint8_t value);
void Grip_Open_Command(int i, float value);
float Grip_Feedback_Flag(int i);
void Grip_Axis_Update(int i);

#endif
