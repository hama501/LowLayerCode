#ifndef IO_CTRL_H
#define IO_CTRL_H

#include "motor_config.h"

#define SERVO_MAX			4
#define LIMIT_SW_MAX		4

#if SERVO_MAX_NUM < 0 || SERVO_MAX_NUM > SERVO_MAX
#error "SERVO_MAX_NUM must be 0 to 4"
#endif
#if LIMIT_SW_MAX_NUM < 0 || LIMIT_SW_MAX_NUM > LIMIT_SW_MAX
#error "LIMIT_SW_MAX_NUM must be 0 to 4"
#endif
#if SERVO_NUM > SERVO_MAX_NUM
#error "Too many connectors are set to IO_SERVO (more than SERVO_MAX_NUM)"
#endif
#if LIMIT_SW_NUM > LIMIT_SW_MAX_NUM
#error "Too many connectors are set to IO_LIMIT_SW (more than LIMIT_SW_MAX_NUM)"
#endif
#if GPIO1_USE == IO_SERVO && EX2_USE == IO_SERVO
#error "GPIO_1 (PB8) and EX-2 (PA15) both use TIM2_CH1. Only one of them can be IO_SERVO"
#endif
#if GPIO1_USE < IO_UNUSED || GPIO1_USE > IO_LIMIT_SW || GPIO2_USE < IO_UNUSED || GPIO2_USE > IO_LIMIT_SW || EX2_USE < IO_UNUSED || EX2_USE > IO_LIMIT_SW || EX3_USE < IO_UNUSED || EX3_USE > IO_LIMIT_SW
#error "GPIO1_USE / GPIO2_USE / EX2_USE / EX3_USE must be IO_UNUSED, IO_SERVO or IO_LIMIT_SW"
#endif

extern int duty[SERVO_MAX];
extern float servo_mode[SERVO_MAX];
extern float servo_target_angle[SERVO_MAX];
extern uint8_t Limit_Switch_state[LIMIT_SW_MAX];

void Servo_Init(void);
void Servo_Angle_Command(int i, float value);
void Servo_Mode_Command(int i, float value);
void send_servo_data(void);

bool Limit_Switch_Config_Valid(void);
void Read_LimitSwitch(void);
void Limit_Switch_Apply(void);
uint8_t Limit_Switch_Bits(void);

#endif
