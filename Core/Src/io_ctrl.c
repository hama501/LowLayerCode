#include "io_ctrl.h"
#include "current_ctrl.h"
#include "velocity_ctrl.h"
#include "position_ctrl.h"
#include "can_send.h"
#include <math.h>

//==================== コネクター ====================

//サーボに使うコネクターだけタイマーを参照する(CubeMXで未設定でもビルドできる)
#if GPIO1_USE == IO_SERVO
#define GPIO1_TIM_PTR		&htim2
#else
#define GPIO1_TIM_PTR		NULL
#endif
#if GPIO2_USE == IO_SERVO
#define GPIO2_TIM_PTR		&htim2
#else
#define GPIO2_TIM_PTR		NULL
#endif
#if EX2_USE == IO_SERVO
#define EX2_TIM_PTR			&htim2
#else
#define EX2_TIM_PTR			NULL
#endif
#if EX3_USE == IO_SERVO
#define EX3_TIM_PTR			&htim2
#else
#define EX3_TIM_PTR			NULL
#endif

#define IO_CONNECTOR_NUM	4

typedef struct
{
	GPIO_TypeDef *port;
	uint16_t pin;
	uint8_t use;
	TIM_HandleTypeDef *tim;
	uint32_t channel;
	uint8_t alternate;
} IoConnector_t;

static const IoConnector_t io_connector[IO_CONNECTOR_NUM] = {
	{GPIOB, GPIO_PIN_8,  GPIO1_USE, GPIO1_TIM_PTR, TIM_CHANNEL_1, GPIO_AF1_TIM2},
	{GPIOB, GPIO_PIN_9,  GPIO2_USE, GPIO2_TIM_PTR, TIM_CHANNEL_2, GPIO_AF1_TIM2},
	{GPIOA, GPIO_PIN_15, EX2_USE,   EX2_TIM_PTR,   TIM_CHANNEL_1, GPIO_AF1_TIM2},
	{GPIOB, GPIO_PIN_10, EX3_USE,   EX3_TIM_PTR,   TIM_CHANNEL_3, GPIO_AF1_TIM2},
};

//==================== サーボ ====================

static TIM_HandleTypeDef *servo_tim[SERVO_MAX];
static uint32_t servo_channel[SERVO_MAX];
static GPIO_TypeDef *limit_sw_port[LIMIT_SW_MAX];
static uint16_t limit_sw_pin[LIMIT_SW_MAX];
static const int servo_pulse_min[SERVO_MAX] = SERVO_PULSE_MIN;
static const int servo_pulse_max[SERVO_MAX] = SERVO_PULSE_MAX;
static const float servo_angle_max[SERVO_MAX] = SERVO_ANGLE_MAX;

int duty[SERVO_MAX];								//出しているパルス幅[µs]
float servo_mode[SERVO_MAX] = SERVO_INIT_MODE;
float servo_target_angle[SERVO_MAX] = SERVO_INIT_ANGLE;

//コネクターごとにピンをサーボ(タイマーのPWM出力)かリミットスイッチ(入力)にして、サーボ・スイッチの番号を割り当てる
static void IO_Connector_Init(void)
{
	int servo = 0;
	int limit_sw = 0;

	for (int c = 0; c < IO_CONNECTOR_NUM; c++)
	{
		GPIO_InitTypeDef GPIO_InitStruct = {0};
		GPIO_InitStruct.Pin = io_connector[c].pin;

		if (io_connector[c].use == IO_SERVO)
		{
			GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
			GPIO_InitStruct.Pull = GPIO_NOPULL;
			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
			GPIO_InitStruct.Alternate = io_connector[c].alternate;

			TIM_OC_InitTypeDef sConfigOC = {0};
			sConfigOC.OCMode     = TIM_OCMODE_PWM1;
			sConfigOC.Pulse      = 0;
			sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
			sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
			HAL_TIM_PWM_ConfigChannel(io_connector[c].tim, &sConfigOC, io_connector[c].channel);

			servo_tim[servo]     = io_connector[c].tim;
			servo_channel[servo] = io_connector[c].channel;
			servo++;
		}
		else if (io_connector[c].use == IO_LIMIT_SW)
		{
			GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
			GPIO_InitStruct.Pull = GPIO_PULLUP;
			limit_sw_port[limit_sw] = io_connector[c].port;
			limit_sw_pin[limit_sw]  = io_connector[c].pin;
			limit_sw++;
		}
		else
		{
			GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
			GPIO_InitStruct.Pull = GPIO_NOPULL;
		}
		HAL_GPIO_Init(io_connector[c].port, &GPIO_InitStruct);
	}
}

//ピンを設定し、PWM出力を開始して初期角度を出す
void Servo_Init(void)
{
	IO_Connector_Init();
	for (int i = 0; i < SERVO_NUM; i++)
	{
		HAL_TIM_PWM_Start(servo_tim[i], servo_channel[i]);
	}
	send_servo_data();
}

void Servo_Angle_Command(int i, float value)
{
	if (i < 0 || i >= SERVO_NUM || !isfinite(value)) return;
	if (value < 0.0f) value = 0.0f;
	if (value > servo_angle_max[i]) value = servo_angle_max[i];
	servo_target_angle[i] = value;
}

void Servo_Mode_Command(int i, float value)
{
	if (i < 0 || i >= SERVO_NUM || !isfinite(value)) return;
	servo_mode[i] = value;
}

//角度をパルス幅にしてPWMに出す
void send_servo_data(void)
{
	for (int i = 0; i < SERVO_NUM; i++)
	{
		float ratio = servo_target_angle[i] / servo_angle_max[i];
		int range = servo_pulse_max[i] - servo_pulse_min[i];

		if (servo_mode[i] == 0) duty[i] = (int)(servo_pulse_max[i] - ratio * range);
		else					duty[i] = (int)(servo_pulse_min[i] + ratio * range);
		__HAL_TIM_SET_COMPARE(servo_tim[i], servo_channel[i], duty[i]);
	}
}

//==================== リミットスイッチ ====================

static const GPIO_PinState limit_sw_active[LIMIT_SW_MAX] = LIMIT_SW_ACTIVE;
static const int8_t limit_sw_motor[LIMIT_SW_MAX] = LIMIT_SW_MOTOR;
static const int8_t limit_sw_dir[LIMIT_SW_MAX] = LIMIT_SW_DIR;
static const uint8_t limit_sw_origin[LIMIT_SW_MAX] = LIMIT_SW_ORIGIN;

uint8_t Limit_Switch_state[LIMIT_SW_MAX];			//1: 押されている(チャタリング除去後)
static uint16_t limit_sw_stable_ms[LIMIT_SW_MAX];

//止めるモーターの番号が正しいか確認する
bool Limit_Switch_Config_Valid(void)
{
	for (int s = 0; s < LIMIT_SW_NUM; s++)
	{
		if (limit_sw_motor[s] >= MOTOR_NUM) return false;
		if (limit_sw_dir[s] != 1 && limit_sw_dir[s] != -1) return false;
	}
	return true;
}

//スイッチを読み、LIMIT_SW_DEBOUNCE_MSの間同じなら状態を確定する。押された瞬間に原点リセット(設定時)
void Read_LimitSwitch(void)
{
	for (int s = 0; s < LIMIT_SW_NUM; s++)
	{
		uint8_t raw = (HAL_GPIO_ReadPin(limit_sw_port[s], limit_sw_pin[s]) == limit_sw_active[s]) ? 1 : 0;

		if (raw == Limit_Switch_state[s])
		{
			limit_sw_stable_ms[s] = 0;
			continue;
		}
		if (++limit_sw_stable_ms[s] < LIMIT_SW_DEBOUNCE_MS) continue;

		limit_sw_stable_ms[s] = 0;
		Limit_Switch_state[s] = raw;

		int m = limit_sw_motor[s];
		if (raw && m >= 0 && limit_sw_origin[s]) Origin_Reset_Request(m);
	}
}

//押されているスイッチの向きへは動かさない(電流・RPM指令を0にして積分も止める)
void Limit_Switch_Apply(void)
{
	for (int s = 0; s < LIMIT_SW_NUM; s++)
	{
		int m = limit_sw_motor[s];
		if (!Limit_Switch_state[s] || m < 0) continue;

		float dir = (float)limit_sw_dir[s];
		if (current[m] * dir > 0.0f)
		{
			current[m]  = 0.0f;
			integral[m] = 0.0f;
		}
		if (Motor_Use_Vesc_Rpm(m) && target_vel[m] * dir > 0.0f)
		{
			target_vel[m] = 0.0f;
		}
	}
}

//スイッチの状態をビットにまとめる(ビットs = スイッチs)
uint8_t Limit_Switch_Bits(void)
{
	uint8_t bits = 0;
	for (int s = 0; s < LIMIT_SW_NUM; s++)
	{
		if (Limit_Switch_state[s]) bits |= (uint8_t)(1u << s);
	}
	return bits;
}
