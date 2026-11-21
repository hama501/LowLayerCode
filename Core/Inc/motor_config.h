#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

#include "main.h"
#include "robot_config.h"
#include <stdbool.h>

#define MOTOR_MAX			4
#define MOTOR_NUM			(CURRENT_MOTOR_NUM + VELOCITY_MOTOR_NUM + POSITION_MOTOR_NUM + GRIP_MOTOR_NUM)

#if MOTOR_NUM <= 0
#error "MOTOR_NUM must be at least 1"
#endif
#if MOTOR_NUM > MOTOR_MAX
#error "MOTOR_NUM exceeds MOTOR_MAX"
#endif

#define DT					0.001f	//制御周期[s]

typedef enum {
    CURRENT = 0,
    VELOCITY,
    POSITION,
    GRIP,
	BRAKE,
} ModeState_t;

extern volatile ModeState_t mode[MOTOR_MAX];

extern const float motor_direction[MOTOR_MAX];
extern const float motor_gear_ratio[MOTOR_MAX];
extern const float motor_gear_length[MOTOR_MAX];
extern const float motor_current_max[MOTOR_MAX];
extern const float motor_vel_max[MOTOR_MAX];
extern const uint8_t motor_vesc_rpm[MOTOR_MAX];

extern volatile int8_t motor_config_error;

void ResetTask(int i);
void Motor_Mode_Init(void);

#endif
