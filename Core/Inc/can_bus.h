#ifndef CAN_BUS_H
#define CAN_BUS_H

#include "motor_config.h"

#define STARTUP_FRAMES			5u
#define CAN_FEEDBACK_TIMEOUT_MS	100u

#define CAN_TXQ_SIZE			16u
#define CAN_TX_TIMEOUT_MS		5u

#define CAN_BOFF_FORCE_RECOVER_MS	20u
#define CAN_TX_STUCK_THRESHOLD		20u
#define CAN_RECOVER_MIN_INTERVAL_MS	20u

#define CAN_NOTIFY_FLAGS		(CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_OVERRUN | \
								 CAN_IT_TX_MAILBOX_EMPTY | \
								 CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE | CAN_IT_BUSOFF | CAN_IT_ERROR)

typedef struct {
	CAN_TxHeaderTypeDef header;
	uint8_t data[8];
} CanTxFrame_t;

typedef struct {
	CanTxFrame_t buf[16];
	volatile uint8_t head;
	volatile uint8_t tail;
	volatile uint8_t count;
	uint32_t mailbox_tick[3];
	volatile uint32_t tx_drop;
	volatile uint32_t tx_abort;
	volatile uint32_t rx_overrun;
	volatile uint32_t error_count;
	volatile uint32_t busoff_count;
	volatile uint32_t last_error;
	volatile uint8_t tec;
	volatile uint8_t rec;
	volatile bool bus_off;
	volatile uint16_t stuck_streak;
	volatile uint8_t need_recover;
	volatile uint32_t recover_cnt;
	volatile uint32_t last_esr;
	uint8_t boff_active;
	uint32_t boff_since_tick;
	uint32_t last_recover_tick;
} CanBus_t;

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern CanBus_t can_bus1;
extern CanBus_t can_bus2;

extern volatile float vesc_erpm[MOTOR_MAX];
extern volatile float fb_current[MOTOR_MAX];
extern volatile float fb_temperature[MOTOR_MAX];

void CAN_Init(void);
void CAN_Flush(CAN_HandleTypeDef *hcan);
bool CAN_Send(CAN_HandleTypeDef *hcan, const CAN_TxHeaderTypeDef *header, const uint8_t *data);
uint32_t CAN_Check_Tx_Timeout(CAN_HandleTypeDef *hcan);
void CAN_Update_Status(CAN_HandleTypeDef *hcan);
void CAN_Safe_Task(CAN_HandleTypeDef *hcan);
void CAN_Monitor(CAN_HandleTypeDef *hcan);

bool Motor_Control_Ready(int i);
void Frame_Count_Increace(int i);
void Frame_Count_Reset(void);

#endif
