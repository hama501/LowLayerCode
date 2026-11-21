#include "position_ctrl.h"
#include "velocity_ctrl.h"
#include "can_send.h"
#include <math.h>

float now_x[MOTOR_MAX];
float target_x[MOTOR_MAX];
float diff_x;
float pos_p_gain[MOTOR_MAX] = MOTOR_POS_P_GAIN;
bool is_stopped[MOTOR_MAX];

static const float pos_stop[MOTOR_MAX]    = MOTOR_POS_STOP;
static const float pos_restart[MOTOR_MAX] = MOTOR_POS_RESTART;
static const float pos_min_vel[MOTOR_MAX] = MOTOR_POS_MIN_VEL;

volatile int32_t now_count[MOTOR_MAX];	//エンコーダーの今のカウント
int32_t last_count[MOTOR_MAX];			//前回のカウント
float now_angle[MOTOR_MAX];				//出力軸の角度[deg]
float plas_angle[MOTOR_MAX];			//1回の更新で進んだ角度[deg]

static volatile bool encoder_synced[MOTOR_MAX];
static volatile bool origin_reset_request[MOTOR_MAX];

//VESCの軸だけ、robot_config.hで指定したエンコーダータイマーを使う
#if MOTOR_NUM > 0 && MOTOR0_DRIVER == DRIVER_VESC
#define MOTOR0_ENC_PTR		MOTOR0_ENCODER
#else
#define MOTOR0_ENC_PTR		ENCODER_NONE
#endif
#if MOTOR_NUM > 1 && MOTOR1_DRIVER == DRIVER_VESC
#define MOTOR1_ENC_PTR		MOTOR1_ENCODER
#else
#define MOTOR1_ENC_PTR		ENCODER_NONE
#endif
#if MOTOR_NUM > 2 && MOTOR2_DRIVER == DRIVER_VESC
#define MOTOR2_ENC_PTR		MOTOR2_ENCODER
#else
#define MOTOR2_ENC_PTR		ENCODER_NONE
#endif
#if MOTOR_NUM > 3 && MOTOR3_DRIVER == DRIVER_VESC
#define MOTOR3_ENC_PTR		MOTOR3_ENCODER
#else
#define MOTOR3_ENC_PTR		ENCODER_NONE
#endif

static TIM_HandleTypeDef *const motor_encoder[MOTOR_MAX] = {MOTOR0_ENC_PTR, MOTOR1_ENC_PTR, MOTOR2_ENC_PTR, MOTOR3_ENC_PTR};

float wrap(float x, float min, float max)
{
	float range = max - min;
	float r = fmodf(x - min, range);
	if (r < 0) r += range;
	return r + min;
}

TIM_HandleTypeDef *Motor_Encoder_Tim(int i)
{
	return motor_encoder[i];
}

//角度・位置を出せる軸か(VESCはエンコーダーあり、C610/C620は常にあり)
bool Motor_Has_Encoder(int i)
{
	if (motor_driver[i] == DRIVER_VESC) return motor_encoder[i] != NULL;
	return true;
}

//出力軸の回転量[回転]を角度と位置に足す
static void Motor_Angle_Add(int i, float rev)
{
	plas_angle[i] = motor_direction[i] * rev * 360.0f;
	now_angle[i] += plas_angle[i];
	now_x[i]     += motor_direction[i] * rev * motor_gear_length[i];
}

//エンコーダータイマーのカウントを開始する
void Encoder_Init(void)
{
	for (int i = 0; i < MOTOR_NUM; i++)
	{
		if (motor_encoder[i] == NULL) continue;
		HAL_TIM_Encoder_Start(motor_encoder[i], TIM_CHANNEL_ALL);
		now_count[i]  = (int32_t)__HAL_TIM_GET_COUNTER(motor_encoder[i]);
		last_count[i] = now_count[i];
	}
}

//VESCの軸: エンコーダータイマーから角度・位置・速度を出す(1msごと)
void Encoder_Update(void)
{
	for (int i = 0; i < MOTOR_NUM; i++)
	{
		TIM_HandleTypeDef *tim = motor_encoder[i];
		if (tim == NULL) continue;

		int32_t count_max = (int32_t)__HAL_TIM_GET_AUTORELOAD(tim) + 1;
		now_count[i] = (int32_t)__HAL_TIM_GET_COUNTER(tim);

		int32_t diff_count = now_count[i] - last_count[i];
		if (diff_count >  count_max / 2) diff_count -= count_max;
		if (diff_count < -count_max / 2) diff_count += count_max;
		last_count[i] = now_count[i];

		float rev = (float)diff_count / (float)count_max;
		Motor_Angle_Add(i, rev);

		float vel = motor_direction[i] * rev * motor_gear_length[i] / DT;
		now_vel[i] += ENCODER_VEL_FILTER * (vel - now_vel[i]);
	}
}

bool Motor_Encoder_Synced(int i)
{
	return encoder_synced[i];
}

//C610/C620の軸: CAN2で受けた角度カウントから角度・位置を出す
void Motor_Encoder_Update(int i, uint16_t count)
{
	if (!encoder_synced[i])
	{
		now_count[i]  = count;
		last_count[i] = count;
		encoder_synced[i] = true;
		return;
	}

	now_count[i] = count;
	float diff = (float)now_count[i] - (float)last_count[i];
	if (diff >  MOTOR_ENCODER_COUNT / 2) diff -= MOTOR_ENCODER_COUNT;
	if (diff < -MOTOR_ENCODER_COUNT / 2) diff += MOTOR_ENCODER_COUNT;
	last_count[i] = now_count[i];

	Motor_Angle_Add(i, diff / MOTOR_ENCODER_COUNT / motor_gear_ratio[i]);
}

void Motor_Encoder_Unsync(int i)
{
	encoder_synced[i] = false;
}

void Origin_Reset_Request(int i)
{
	origin_reset_request[i] = true;
}

//原点リセット: 位置と角度を0にする(POSITIONの軸は目標も0にして急に動かないようにする)
void Origin_Reset_Process(void)
{
	for (int i = 0; i < MOTOR_NUM; i++)
	{
		if (!origin_reset_request[i]) continue;
		origin_reset_request[i] = false;

		if (motor_encoder[i] != NULL)
		{
			now_count[i]  = (int32_t)__HAL_TIM_GET_COUNTER(motor_encoder[i]);
			last_count[i] = now_count[i];
		}
		now_angle[i]  = 0.0f;
		plas_angle[i] = 0.0f;
		now_x[i]      = 0.0f;
		if (mode[i] == POSITION)
		{
			target_x[i]   = 0.0f;
			is_stopped[i] = false;
		}
	}
}

void Pos_Command(int i, float value)
{
	if (!isfinite(value)) return;
	target_x[i] = value;
}

void Pos_P_control(int i)
{
	diff_x = target_x[i] - now_x[i];

	if (fabsf(diff_x) < pos_stop[i] && !is_stopped[i])
	{
	    is_stopped[i] = true;
	}
	if (fabsf(diff_x) > pos_restart[i])
	{
	    is_stopped[i] = false;
	}

	if (is_stopped[i])
	{
	    target_vel[i] = 0;
	    return;
	}

	target_vel[i] = pos_p_gain[i] * diff_x;

	if      (target_vel[i] > 0.0f && target_vel[i] <  pos_min_vel[i]) target_vel[i] =  pos_min_vel[i];
	else if (target_vel[i] < 0.0f && target_vel[i] > -pos_min_vel[i]) target_vel[i] = -pos_min_vel[i];

	if (target_vel[i] > motor_vel_max[i])  target_vel[i] = motor_vel_max[i];
	if (target_vel[i] < -motor_vel_max[i]) target_vel[i] = -motor_vel_max[i];
}
