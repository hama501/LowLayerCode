#include "can_send.h"
#include "can_bus.h"
#include "current_ctrl.h"
#include "velocity_ctrl.h"
#include "position_ctrl.h"
#include "grip_ctrl.h"
#include "io_ctrl.h"
#include <math.h>
#include <string.h>

const uint8_t motor_driver[MOTOR_MAX]    = {MOTOR0_DRIVER, MOTOR1_DRIVER, MOTOR2_DRIVER, MOTOR3_DRIVER};
const uint32_t motor_can2_id[MOTOR_MAX]  = {MOTOR0_CAN2_ID, MOTOR1_CAN2_ID, MOTOR2_CAN2_ID, MOTOR3_CAN2_ID};
const uint32_t motor_cmd_rx_id[MOTOR_MAX] = {MOTOR0_CMD_RX_ID, MOTOR1_CMD_RX_ID, MOTOR2_CMD_RX_ID, MOTOR3_CMD_RX_ID};
const uint32_t motor_fb_tx_id[MOTOR_MAX]  = {MOTOR0_FB_TX_ID, MOTOR1_FB_TX_ID, MOTOR2_FB_TX_ID, MOTOR3_FB_TX_ID};

bool Motor_Use_Vesc_Rpm(int i)
{
	return motor_driver[i] == DRIVER_VESC && motor_vesc_rpm[i] && (mode[i] == VELOCITY || mode[i] == POSITION);
}

static void send_vesc_data(uint8_t packet, uint8_t vesc_id, int32_t value)
{
    uint8_t TxData[4];
    CAN_TxHeaderTypeDef TxHeader = {0};

    TxData[0] = (uint8_t)(((uint32_t)value >> 24) & 0xFF);
    TxData[1] = (uint8_t)(((uint32_t)value >> 16) & 0xFF);
    TxData[2] = (uint8_t)(((uint32_t)value >> 8) & 0xFF);
    TxData[3] = (uint8_t)((uint32_t)value & 0xFF);

    TxHeader.DLC = 4;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_EXT;
    TxHeader.ExtId = ((uint32_t)packet << 8) | vesc_id;

    CAN_Send(&hcan2, &TxHeader, TxData);
}

static void send_vesc_command(int i)
{
	uint8_t vesc_id = (uint8_t)motor_can2_id[i];

	if (Motor_Use_Vesc_Rpm(i))
	{
		float erpm = target_vel[i] * motor_direction[i] / motor_gear_length[i] * 60.0f * motor_gear_ratio[i] * MOTOR_POLE_PAIRS;
		send_vesc_data(VESC_PACKET_SET_RPM, vesc_id, (int32_t)erpm);
	}
	else
	{
		send_vesc_data(VESC_PACKET_SET_CURRENT, vesc_id, (int32_t)(current[i] * motor_direction[i]));
	}
}

static void send_c6x0_frame(uint32_t std_id, const uint8_t *TxData)
{
	CAN_TxHeaderTypeDef TxHeader = {0};

	TxHeader.DLC    = 8;
	TxHeader.RTR    = CAN_RTR_DATA;
	TxHeader.IDE    = CAN_ID_STD;
	TxHeader.StdId  = std_id;

	CAN_Send(&hcan2, &TxHeader, TxData);
}

static void send_c6x0_current_data(void)
{
	uint8_t TxLow[8]  = {0};
	uint8_t TxHigh[8] = {0};
	bool use_low  = false;
	bool use_high = false;

	for (int i = 0; i < MOTOR_NUM; i++)
	{
		if (motor_driver[i] == DRIVER_VESC) continue;

		float limit = (motor_driver[i] == DRIVER_C620) ? C620_CURRENT_MAX : C610_CURRENT_MAX;
		float scale = (motor_driver[i] == DRIVER_C620) ? (C620_CURRENT_RAW_MAX / C620_CURRENT_MAX) : 1.0f;

		float value = current[i] * motor_direction[i];
		if (value > limit)  value = limit;
		if (value < -limit) value = -limit;
		int16_t out = (int16_t)(value * scale);

		uint32_t slot = motor_can2_id[i] - C6X0_RX_ID_MIN;
		uint8_t *buf;
		if (slot < 4)
		{
			buf = TxLow;
			use_low = true;
		}
		else
		{
			buf = TxHigh;
			use_high = true;
			slot -= 4;
		}
		buf[2 * slot]     = (uint8_t)((out >> 8) & 0xFF);
		buf[2 * slot + 1] = (uint8_t)(out & 0xFF);
	}

	if (use_low)  send_c6x0_frame(C6X0_TX_ID_LOW, TxLow);
	if (use_high) send_c6x0_frame(C6X0_TX_ID_HIGH, TxHigh);
}

void send_motor_current_data(void)
{
	for (int i = 0; i < MOTOR_NUM; i++)
	{
		if (motor_driver[i] == DRIVER_VESC)
		{
			send_vesc_command(i);
		}
	}
	send_c6x0_current_data();
}

//制御モードに合わせたフィードバックを送る(中身はcan_send.hのMOTORn_FB_TX_IDの説明)
void send_feedback_data(int i)
{
	uint8_t TxData[8] = {0};
	CAN_TxHeaderTypeDef TxHeader = {0};
	uint32_t std_id = motor_fb_tx_id[i];

	if (std_id > 0x7FF) return;

	float value[2];
	switch (mode[i])
	{
	case CURRENT:
		value[0] = fb_current[i];
		value[1] = now_vel[i];
		break;
	case VELOCITY:
		value[0] = now_vel[i];
		value[1] = fb_current[i];
		break;
	case POSITION:
		value[0] = now_x[i];
		value[1] = now_vel[i];
		break;
	case GRIP:
		value[0] = Grip_Feedback_Flag(i);
		value[1] = fb_current[i];
		break;
	default:
		return;
	}
	memcpy(TxData, value, sizeof(value));

	TxHeader.DLC    = 8;
	TxHeader.RTR    = CAN_RTR_DATA;
	TxHeader.IDE    = CAN_ID_STD;
	TxHeader.StdId  = std_id;

	CAN_Send(&hcan1, &TxHeader, TxData);
}

//全モーターの制御モードをまとめて送る(値の意味はcan_send.hのMODE_TX_IDの説明)
void send_mode_data(void)
{
	uint8_t TxData[8] = {0};
	CAN_TxHeaderTypeDef TxHeader = {0};

	for (int i = 0; i < MOTOR_NUM; i++)
	{
		switch (mode[i])
		{
		case CURRENT:
			TxData[i] = 1;
			break;
		case VELOCITY:
			TxData[i] = 2;
			break;
		case POSITION:
			TxData[i] = 3;
			break;
		case GRIP:
			if (grip_state[i] == GRIP_APPROACH)
			{
				TxData[i] = 6;
			}
			else if (grip_state[i] == GRIP_SQUEEZE || grip_state[i] == GRIP_HOLD)
			{
				TxData[i] = 5;
			}
			else
			{
				TxData[i] = 4;
			}
			break;
		default:
			TxData[i] = 0;
			break;
		}
	}

	TxHeader.DLC    = MOTOR_MAX;
	TxHeader.RTR    = CAN_RTR_DATA;
	TxHeader.IDE    = CAN_ID_STD;
	TxHeader.StdId  = MODE_TX_ID;

	CAN_Send(&hcan1, &TxHeader, TxData);
}

//リミットスイッチがどれか押されている間だけ、状態をまとめて送る
void send_limit_switch_data(void)
{
#if LIMIT_SW_NUM > 0
	uint8_t TxData[8] = {0};
	CAN_TxHeaderTypeDef TxHeader = {0};

	TxData[0] = Limit_Switch_Bits();
	if (TxData[0] == 0) return;

	TxHeader.DLC    = 1;
	TxHeader.RTR    = CAN_RTR_DATA;
	TxHeader.IDE    = CAN_ID_STD;
	TxHeader.StdId  = LIMIT_SW_TX_ID;

	CAN_Send(&hcan1, &TxHeader, TxData);
#endif
}
