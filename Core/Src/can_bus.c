#include "can_bus.h"
#include "can_send.h"
#include "current_ctrl.h"
#include "velocity_ctrl.h"
#include "position_ctrl.h"
#include "grip_ctrl.h"
#include "io_ctrl.h"
#include <math.h>
#include <string.h>

volatile float vesc_erpm[MOTOR_MAX];
volatile float fb_current[MOTOR_MAX];
volatile float fb_temperature[MOTOR_MAX];

CanBus_t can_bus1;
CanBus_t can_bus2;

static uint16_t rx_frame_count[MOTOR_MAX];
static uint32_t last_feedback_tick[MOTOR_MAX];

__attribute__((unused)) static void CAN1_ID_Duplicate_Check(uint32_t id)
{
	switch (id)
	{
	case MOTOR0_CMD_RX_ID:
	case MOTOR1_CMD_RX_ID:
	case MOTOR2_CMD_RX_ID:
	case MOTOR3_CMD_RX_ID:
	case MOTOR0_FB_TX_ID:
	case MOTOR1_FB_TX_ID:
	case MOTOR2_FB_TX_ID:
	case MOTOR3_FB_TX_ID:
	case CMD_RESET_ID:
	case CMD_CURRENT_ALARM_ID:
	case CMD_GRIP_OPEN_ID:
	case CMD_ORIGIN_RESET_ID:
	case SERVO0_ANGLE_RX_ID:
	case SERVO0_MODE_RX_ID:
	case SERVO1_ANGLE_RX_ID:
	case SERVO1_MODE_RX_ID:
	case SERVO2_ANGLE_RX_ID:
	case SERVO2_MODE_RX_ID:
	case SERVO3_ANGLE_RX_ID:
	case SERVO3_MODE_RX_ID:
	case LIMIT_SW_TX_ID:
	case MODE_TX_ID:
		break;
	default:
		break;
	}
}

bool Motor_Control_Ready(int i)
{
	return rx_frame_count[i] >= STARTUP_FRAMES;
}

void Frame_Count_Increace(int i)
{
	last_feedback_tick[i] = HAL_GetTick();
	if (rx_frame_count[i] < STARTUP_FRAMES) rx_frame_count[i]++;
}

void Frame_Count_Reset(void)
{
	for (int i = 0; i < MOTOR_NUM; i++)
	{
		if (rx_frame_count[i] > 0 && (HAL_GetTick() - last_feedback_tick[i]) > CAN_FEEDBACK_TIMEOUT_MS)
		{
			rx_frame_count[i] = 0;
			now_vel[i] = 0.0f;
			Motor_Encoder_Unsync(i);
		}
	}
}

static CanBus_t *CAN_GetBus(CAN_HandleTypeDef *hcan)
{
	if (hcan == &hcan1) return &can_bus1;
	if (hcan == &hcan2) return &can_bus2;
	return NULL;
}

static uint32_t CAN_GetFrameId(const CAN_TxHeaderTypeDef *header)
{
	return (header->IDE == CAN_ID_STD) ? header->StdId : header->ExtId;
}

static uint32_t CAN_MailboxIndex(uint32_t mailbox)
{
	if (mailbox == CAN_TX_MAILBOX0) return 0;
	if (mailbox == CAN_TX_MAILBOX1) return 1;
	return 2;
}

void CAN_Flush(CAN_HandleTypeDef *hcan)
{
	CanBus_t *bus = CAN_GetBus(hcan);
	if (bus == NULL) return;

	uint32_t primask = __get_PRIMASK();
	__disable_irq();

	while (bus->count > 0 && HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0)
	{
		CanTxFrame_t *frame = &bus->buf[bus->tail];
		uint32_t mailbox;

		if (HAL_CAN_AddTxMessage(hcan, &frame->header, frame->data, &mailbox) != HAL_OK) break;

		bus->mailbox_tick[CAN_MailboxIndex(mailbox)] = HAL_GetTick();
		bus->tail = (bus->tail + 1) % CAN_TXQ_SIZE;
		bus->count--;
	}

	__set_PRIMASK(primask);
}

bool CAN_Send(CAN_HandleTypeDef *hcan, const CAN_TxHeaderTypeDef *header, const uint8_t *data)
{
	CanBus_t *bus = CAN_GetBus(hcan);
	if (bus == NULL || header->DLC > 8) return false;

	uint32_t id = CAN_GetFrameId(header);
	bool queued = false;

	uint32_t primask = __get_PRIMASK();
	__disable_irq();

	uint8_t pos = bus->tail;
	for (uint8_t n = 0; n < bus->count; n++)
	{
		CanTxFrame_t *frame = &bus->buf[pos];
		if (frame->header.IDE == header->IDE && CAN_GetFrameId(&frame->header) == id)
		{
			frame->header = *header;
			memcpy(frame->data, data, header->DLC);
			queued = true;
			break;
		}
		pos = (pos + 1) % CAN_TXQ_SIZE;
	}

	if (!queued)
	{
		if (bus->count < CAN_TXQ_SIZE)
		{
			CanTxFrame_t *frame = &bus->buf[bus->head];
			frame->header = *header;
			memcpy(frame->data, data, header->DLC);
			bus->head = (bus->head + 1) % CAN_TXQ_SIZE;
			bus->count++;
			queued = true;
		}
		else
		{
			bus->tx_drop++;
		}
	}

	__set_PRIMASK(primask);

	CAN_Flush(hcan);
	return queued;
}

uint32_t CAN_Check_Tx_Timeout(CAN_HandleTypeDef *hcan)
{
	CanBus_t *bus = CAN_GetBus(hcan);
	if (bus == NULL) return 0;

	const uint32_t mailboxes[3] = {CAN_TX_MAILBOX0, CAN_TX_MAILBOX1, CAN_TX_MAILBOX2};
	uint32_t now = HAL_GetTick();
	uint32_t aborted = 0;

	for (int i = 0; i < 3; i++)
	{
		if (HAL_CAN_IsTxMessagePending(hcan, mailboxes[i]) &&
		    (now - bus->mailbox_tick[i]) > CAN_TX_TIMEOUT_MS)
		{
			HAL_CAN_AbortTxRequest(hcan, mailboxes[i]);
			bus->tx_abort++;
			aborted++;
		}
	}
	return aborted;
}

void CAN_Update_Status(CAN_HandleTypeDef *hcan)
{
	CanBus_t *bus = CAN_GetBus(hcan);
	if (bus == NULL) return;

	uint32_t esr = hcan->Instance->ESR;
	bus->tec     = (uint8_t)((esr & CAN_ESR_TEC) >> CAN_ESR_TEC_Pos);
	bus->rec     = (uint8_t)((esr & CAN_ESR_REC) >> CAN_ESR_REC_Pos);
	bus->bus_off = (esr & CAN_ESR_BOFF) != 0;
}

void CAN_Safe_Task(CAN_HandleTypeDef *hcan)
{
	CanBus_t *bus = CAN_GetBus(hcan);
	if (bus == NULL) return;

	uint32_t aborted = CAN_Check_Tx_Timeout(hcan);
	if (aborted > 0)
	{
		uint32_t streak = bus->stuck_streak + aborted;
		bus->stuck_streak = (streak > 0xFFFFu) ? 0xFFFFu : (uint16_t)streak;
		if (bus->stuck_streak >= CAN_TX_STUCK_THRESHOLD) bus->need_recover = 1;
	}
	else if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0)
	{
		bus->stuck_streak = 0;
		if (!(hcan->Instance->ESR & CAN_ESR_BOFF)) bus->need_recover = 0;
	}

	CAN_Update_Status(hcan);
	CAN_Flush(hcan);
}

static void CAN_Recover(CAN_HandleTypeDef *hcan, CanBus_t *bus)
{
	bus->last_esr = hcan->Instance->ESR;

	HAL_CAN_AbortTxRequest(hcan, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
	HAL_CAN_Stop(hcan);
	if (HAL_CAN_Start(hcan) != HAL_OK)
	{
		return;
	}
	HAL_CAN_ActivateNotification(hcan, CAN_NOTIFY_FLAGS);

	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	bus->head  = 0;
	bus->tail  = 0;
	bus->count = 0;
	if (hcan == &hcan2)
	{
		for (int i = 0; i < MOTOR_NUM; i++)
		{
			rx_frame_count[i] = 0;
			Motor_Encoder_Unsync(i);
		}
	}
	__set_PRIMASK(primask);

	bus->stuck_streak = 0;
	bus->boff_active  = 0;
	bus->need_recover = 0;
	bus->recover_cnt++;
}

void CAN_Monitor(CAN_HandleTypeDef *hcan)
{
	CanBus_t *bus = CAN_GetBus(hcan);
	if (bus == NULL) return;

	uint32_t now = HAL_GetTick();

	if (hcan->Instance->ESR & CAN_ESR_BOFF)
	{
		if (!bus->boff_active)
		{
			bus->boff_active = 1;
			bus->boff_since_tick = now;
		}
		else if ((now - bus->boff_since_tick) >= CAN_BOFF_FORCE_RECOVER_MS)
		{
			bus->need_recover = 1;
		}
	}
	else
	{
		bus->boff_active = 0;
	}

	if (bus->need_recover && (now - bus->last_recover_tick) >= CAN_RECOVER_MIN_INTERVAL_MS)
	{
		bus->last_recover_tick = now;
		CAN_Recover(hcan, bus);
	}
}

static void CAN1_Configure_And_Start(void)
{
	CAN_FilterTypeDef can1_filter;
	can1_filter.FilterIdHigh         = 0 << 5; //highとlowでor条件で通す
	can1_filter.FilterIdLow          = 0 << 5; //論理積の結果がidと一緒なら通す
	can1_filter.FilterMaskIdHigh     = 0 << 5;
	can1_filter.FilterMaskIdLow      = 0 << 5; //filtermaskで論理積
	can1_filter.FilterScale          = CAN_FILTERSCALE_32BIT;
	can1_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	can1_filter.FilterBank           = 0;
	can1_filter.FilterMode           = CAN_FILTERMODE_IDMASK;
	can1_filter.SlaveStartFilterBank = 14;
	can1_filter.FilterActivation     = ENABLE;
	if (HAL_CAN_ConfigFilter(&hcan1, &can1_filter) != HAL_OK) Error_Handler();
	if (HAL_CAN_Start(&hcan1) != HAL_OK) Error_Handler();
	if (HAL_CAN_ActivateNotification(&hcan1, CAN_NOTIFY_FLAGS) != HAL_OK)
	{
		Error_Handler();
	}
}

static void CAN2_Configure_And_Start(void)
{
	CAN_FilterTypeDef can2_filter;
	can2_filter.FilterIdHigh         = 0 << 5;
	can2_filter.FilterIdLow          = 0 << 5;
	can2_filter.FilterMaskIdHigh     = 0 << 5;
	can2_filter.FilterMaskIdLow      = 0 << 5;
	can2_filter.FilterScale          = CAN_FILTERSCALE_32BIT;
	can2_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	can2_filter.FilterBank           = 14;
	can2_filter.FilterMode           = CAN_FILTERMODE_IDMASK;
	can2_filter.SlaveStartFilterBank = 14;
	can2_filter.FilterActivation     = ENABLE;
	if (HAL_CAN_ConfigFilter(&hcan2, &can2_filter) != HAL_OK) Error_Handler();
	if (HAL_CAN_Start(&hcan2) != HAL_OK) Error_Handler();
	if (HAL_CAN_ActivateNotification(&hcan2, CAN_NOTIFY_FLAGS) != HAL_OK)
	{
		Error_Handler();
	}
}

void CAN_Init(void)
{
	memset(&can_bus1, 0, sizeof(can_bus1));
	memset(&can_bus2, 0, sizeof(can_bus2));
	CAN1_Configure_And_Start();
	CAN2_Configure_And_Start();
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
	CAN_Flush(hcan);
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
	CAN_Flush(hcan);
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
	CAN_Flush(hcan);
}

void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *hcan)
{
	CAN_Flush(hcan);
}

void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *hcan)
{
	CAN_Flush(hcan);
}

void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *hcan)
{
	CAN_Flush(hcan);
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
	CanBus_t *bus = CAN_GetBus(hcan);
	if (bus != NULL)
	{
		uint32_t err = HAL_CAN_GetError(hcan);
		bus->last_error = err;
		bus->error_count++;
		if (err & HAL_CAN_ERROR_BOF)     bus->busoff_count++;
		if (err & HAL_CAN_ERROR_RX_FOV0) bus->rx_overrun++;
	}
	HAL_CAN_ResetError(hcan);
	CAN_Flush(hcan);
}

static void Parse_Vesc_Temperature(int i, uint8_t *RxData)
{
	int16_t temp_fet = (int16_t)(((uint16_t)RxData[0] << 8) | RxData[1]);
	fb_temperature[i] = temp_fet / 10.0f;
}

static void Parse_Vesc_Status(int i, uint8_t *RxData)
{
	int32_t erpm = (int32_t)(((uint32_t)RxData[0] << 24) | ((uint32_t)RxData[1] << 16) |
	                         ((uint32_t)RxData[2] << 8)  |  (uint32_t)RxData[3]);
	int16_t cur  = (int16_t)(((uint16_t)RxData[4] << 8) | RxData[5]);

	vesc_erpm[i]  = (float)erpm;
	fb_current[i] = motor_direction[i] * cur * 100.0f;

	if (Motor_Encoder_Tim(i) == NULL)
	{
		float motor_rpm = vesc_erpm[i] / MOTOR_POLE_PAIRS;
		now_vel[i] = motor_direction[i] * motor_rpm / 60.0f / motor_gear_ratio[i] * motor_gear_length[i];
	}
}

static void Parse_C6x0_Feedback(int i, uint8_t *RxData)
{
	uint16_t count = (uint16_t)(((uint16_t)RxData[0] << 8) | RxData[1]);
	int16_t rpm    = (int16_t)(((uint16_t)RxData[2] << 8) | RxData[3]);
	int16_t cur    = (int16_t)(((uint16_t)RxData[4] << 8) | RxData[5]);
	float scale    = (motor_driver[i] == DRIVER_C620) ? (C620_CURRENT_MAX / C620_CURRENT_RAW_MAX) : 1.0f;

	fb_current[i]     = motor_direction[i] * cur * scale;
	fb_temperature[i] = RxData[6];

	bool first = !Motor_Encoder_Synced(i);
	Motor_Encoder_Update(i, count);

	if (first) now_vel[i] = 0.0f;
	else       now_vel[i] = motor_direction[i] * (float)rpm / motor_gear_ratio[i] / 60.0f * motor_gear_length[i];
}

static void Motor_Command(int i, CAN_RxHeaderTypeDef *RxHeader, uint8_t *RxData)
{
	if (mode[i] == GRIP)
	{
		if (RxHeader->DLC < 1) return;
		Grip_Command(i, RxData[0]);
		return;
	}

	if (RxHeader->DLC < 4) return;

	float value;
	memcpy(&value, RxData, sizeof(float));

	switch (mode[i])
	{
	case CURRENT:
		Current_Command(i, value);
		break;
	case VELOCITY:
		Vel_Command(i, value);
		break;
	case POSITION:
		Pos_Command(i, value);
		break;
	default:
		break;
	}
}

static int Command_Target(uint8_t dlc, uint8_t *RxData, uint8_t pos)
{
	if (dlc <= pos || RxData[pos] == 0xFF) return -1;
	if (RxData[pos] >= MOTOR_NUM) return -2;
	return RxData[pos];
}

static bool Command_Match(int target, int i)
{
	if (target == -1) return true;
	return target == i;
}

static void System_Command(uint32_t id, uint8_t dlc, uint8_t *RxData)
{
	float value = 0.0f;
	bool has_value = (dlc >= 4);
	if (has_value)
	{
		memcpy(&value, RxData, sizeof(float));
		if (!isfinite(value)) has_value = false;
	}

	switch (id)
	{
	case CMD_RESET_ID:
		for (int i = 0; i < MOTOR_NUM; i++) ResetTask(i);
		break;

	case CMD_CURRENT_ALARM_ID:
		if (has_value) current_alarm_limit = value;
		break;

	case CMD_GRIP_OPEN_ID:
	{
		if (!has_value) break;
		int target = Command_Target(dlc, RxData, 4);
		for (int i = 0; i < MOTOR_NUM; i++)
		{
			if (mode[i] == GRIP && Command_Match(target, i)) Grip_Open_Command(i, value);
		}
		break;
	}

	case CMD_ORIGIN_RESET_ID:
	{
		if (dlc < 1 || RxData[0] != 1) break;
		int target = Command_Target(dlc, RxData, 1);
		for (int i = 0; i < MOTOR_NUM; i++)
		{
			if (Command_Match(target, i)) Origin_Reset_Request(i);
		}
		break;
	}

	default:
		break;
	}
}

static const uint32_t servo_angle_rx_id[SERVO_MAX] = {SERVO0_ANGLE_RX_ID, SERVO1_ANGLE_RX_ID, SERVO2_ANGLE_RX_ID, SERVO3_ANGLE_RX_ID};
static const uint32_t servo_mode_rx_id[SERVO_MAX]  = {SERVO0_MODE_RX_ID, SERVO1_MODE_RX_ID, SERVO2_MODE_RX_ID, SERVO3_MODE_RX_ID};

static bool Servo_Rx_Command(uint32_t id, uint8_t dlc, uint8_t *RxData)
{
	for (int i = 0; i < SERVO_NUM; i++)
	{
		if (id != servo_angle_rx_id[i] && id != servo_mode_rx_id[i]) continue;
		if (dlc >= 4)
		{
			float value;
			memcpy(&value, RxData, sizeof(float));
			if (id == servo_angle_rx_id[i]) Servo_Angle_Command(i, value);
			else                            Servo_Mode_Command(i, value);
		}
		return true;
	}
	return false;
}

static void CAN1_Rx_Process(CAN_RxHeaderTypeDef *RxHeader, uint8_t *RxData)
{
	if (RxHeader->IDE != CAN_ID_STD) return;
	uint32_t id = RxHeader->StdId;

	for (int i = 0; i < MOTOR_NUM; i++)
	{
		if (motor_cmd_rx_id[i] != id) continue;
		Motor_Command(i, RxHeader, RxData);
		return;
	}

	if (Servo_Rx_Command(id, (uint8_t)RxHeader->DLC, RxData)) return;

	System_Command(id, (uint8_t)RxHeader->DLC, RxData);
}

static int Find_Motor_Index(bool vesc, uint32_t id)
{
	for (int i = 0; i < MOTOR_NUM; i++)
	{
		if ((motor_driver[i] == DRIVER_VESC) == vesc && motor_can2_id[i] == id) return i;
	}
	return -1;
}

static void CAN2_Rx_Process(CAN_RxHeaderTypeDef *RxHeader, uint8_t *RxData)
{
	if (RxHeader->IDE == CAN_ID_STD)
	{
		int i = Find_Motor_Index(false, RxHeader->StdId);
		if (i < 0 || RxHeader->DLC < 8) return;
		Parse_C6x0_Feedback(i, RxData);
		Frame_Count_Increace(i);
		return;
	}

	uint32_t id = RxHeader->ExtId;

	int i = Find_Motor_Index(true, id & 0xFF);
	if (i < 0) return;

	uint8_t packet = (id >> 8) & 0xFF;
	if (packet == CAN_PACKET_STATUS && RxHeader->DLC >= 8)
	{
		Parse_Vesc_Status(i, RxData);
		Frame_Count_Increace(i);
	}
	else if (packet == CAN_PACKET_STATUS_ANGLE && RxHeader->DLC >= 8)
	{
		Parse_Vesc_Temperature(i, RxData);
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef RxHeader;
	uint8_t RxData[8];

	while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0)
	{
		if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) break;

		if (hcan == &hcan1)
		{
			CAN1_Rx_Process(&RxHeader, RxData);
		}
		else if (hcan == &hcan2)
		{
			CAN2_Rx_Process(&RxHeader, RxData);
		}
	}
}
