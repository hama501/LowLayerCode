#ifndef POSITION_CTRL_H
#define POSITION_CTRL_H

#include "motor_config.h"

#define MOTOR_ENCODER_COUNT		8192.0f	//C610/C620の1回転あたりのカウント

extern float now_x[MOTOR_MAX];
extern float target_x[MOTOR_MAX];
extern float diff_x;
extern float pos_p_gain[MOTOR_MAX];
extern bool is_stopped[MOTOR_MAX];

extern volatile int32_t now_count[MOTOR_MAX];
extern int32_t last_count[MOTOR_MAX];
extern float now_angle[MOTOR_MAX];
extern float plas_angle[MOTOR_MAX];

float wrap(float x, float min, float max);
TIM_HandleTypeDef *Motor_Encoder_Tim(int i);
bool Motor_Has_Encoder(int i);
void Encoder_Init(void);
void Encoder_Update(void);
bool Motor_Encoder_Synced(int i);
void Motor_Encoder_Update(int i, uint16_t count);
void Motor_Encoder_Unsync(int i);
void Origin_Reset_Request(int i);
void Origin_Reset_Process(void);
void Pos_Command(int i, float value);
void Pos_P_control(int i);

#endif
