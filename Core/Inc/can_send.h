#ifndef CAN_SEND_H
#define CAN_SEND_H

#include "motor_config.h"

//----------ID系-----------//

//CAN2: モーターごとのID(インデックス0〜3)。ドライバーごとに用意しておき、robot_config.hのMOTORn_DRIVERで使うIDが選ばれる
//VESCのとき: VESC Toolで設定したコントローラーID(0x000〜0x0FF)。指令は拡張ID (パケット番号<<8)|ID で送信、STATUSも同じIDで受信
#define MOTOR0_VESC_ID			0x014
#define MOTOR1_VESC_ID			0x015
#define MOTOR2_VESC_ID			0x016
#define MOTOR3_VESC_ID			0x017

//C610のとき: ESCのID(LED点滅回数)に対応するフィードバックID(0x201〜0x208)。電流指令は0x200または0x1FFにまとめて送信
#define MOTOR0_C610_ID			0x201
#define MOTOR1_C610_ID			0x202
#define MOTOR2_C610_ID			0x203
#define MOTOR3_C610_ID			0x204

//C620のとき: C610と同じ決まり(0x201〜0x208)
#define MOTOR0_C620_ID			0x201
#define MOTOR1_C620_ID			0x202
#define MOTOR2_C620_ID			0x203
#define MOTOR3_C620_ID			0x204

//MOTORn_DRIVERに合わせて上のIDから選ぶ(ここは変えない)
#define CAN2_SELECT_ID(drv, vesc, c610, c620)	((drv) == DRIVER_VESC ? (vesc) : ((drv) == DRIVER_C610 ? (c610) : (c620)))
#define MOTOR0_CAN2_ID			CAN2_SELECT_ID(MOTOR0_DRIVER, MOTOR0_VESC_ID, MOTOR0_C610_ID, MOTOR0_C620_ID)
#define MOTOR1_CAN2_ID			CAN2_SELECT_ID(MOTOR1_DRIVER, MOTOR1_VESC_ID, MOTOR1_C610_ID, MOTOR1_C620_ID)
#define MOTOR2_CAN2_ID			CAN2_SELECT_ID(MOTOR2_DRIVER, MOTOR2_VESC_ID, MOTOR2_C610_ID, MOTOR2_C620_ID)
#define MOTOR3_CAN2_ID			CAN2_SELECT_ID(MOTOR3_DRIVER, MOTOR3_VESC_ID, MOTOR3_C610_ID, MOTOR3_C620_ID)

//CAN1: PCとの通信(標準ID)
//モーターごとの指令の受信ID。CURRENTは電流[mA]、VELOCITYは速度、POSITIONは位置をfloatで受信。GRIPは1バイトで0=開放,1=把持
#define MOTOR0_CMD_RX_ID		0x100
#define MOTOR1_CMD_RX_ID		0x101
#define MOTOR2_CMD_RX_ID		0x102
#define MOTOR3_CMD_RX_ID		0x103

//モーターごとのフィードバックの送信ID。中身は制御モードで決まる(float 2個、DLC=8)
//  CURRENT : 0〜3バイト目 電流[mA]   / 4〜7バイト目 速度
//  VELOCITY: 0〜3バイト目 速度       / 4〜7バイト目 電流[mA]
//  POSITION: 0〜3バイト目 位置       / 4〜7バイト目 速度
//  GRIP    : 0〜3バイト目 把持フラグ / 4〜7バイト目 電流[mA]
//  把持フラグ: APPROACH/HOLD中は1.0、CURRENT_ALARM_MOTORの電流がしきい値以上なら+0.1
#define MOTOR0_FB_TX_ID			0x01E
#define MOTOR1_FB_TX_ID			0x01F
#define MOTOR2_FB_TX_ID			0x020
#define MOTOR3_FB_TX_ID			0x021

#define FB_TX_PERIOD_MS			1	//フィードバックの送信周期[ms]

#define LIMIT_SW_TX_ID			0x070	//リミットスイッチの状態の送信ID(1バイト、ビット0〜3がスイッチ0〜3、1=押されている)。どれかが押されている間だけ送る

//全体への指令の受信ID
#define CMD_RESET_ID			0x000	//全モーターの積分・偏差・電流をリセット(データなし)
#define CMD_CURRENT_ALARM_ID	0x043	//把持フラグ+0.1にする電流しきい値[mA](float)
#define CMD_GRIP_OPEN_ID		0x047	//GRIPの開き位置(float)。次の開放から使われる。5バイト目でモーター番号を指定(省略または0xFFなら全GRIP軸)
#define CMD_ORIGIN_RESET_ID		0x140	//1バイト目が1で現在位置を0にする。2バイト目でモーター番号を指定(省略または0xFFなら全モーター)

//サーボごとの指令の受信ID(SERVO_NUMの個数だけ使われる)
#define SERVO0_ANGLE_RX_ID		0x160	//サーボ0の目標角度[deg](float)
#define SERVO0_MODE_RX_ID		0x063	//サーボ0の回転の向き(float、0か1)
#define SERVO1_ANGLE_RX_ID		0x161	//サーボ1の目標角度[deg](float)
#define SERVO1_MODE_RX_ID		0x067	//サーボ1の回転の向き(float、0か1)
#define SERVO2_ANGLE_RX_ID		0x068	//サーボ2の目標角度[deg](float)
#define SERVO2_MODE_RX_ID		0x069	//サーボ2の回転の向き(float、0か1)
#define SERVO3_ANGLE_RX_ID		0x06A	//サーボ3の目標角度[deg](float)
#define SERVO3_MODE_RX_ID		0x06B	//サーボ3の回転の向き(float、0か1)

//VESCのパケット番号(拡張IDの上位バイト)
#define VESC_PACKET_SET_CURRENT			0x01	//電流指令(mA, int32)
#define VESC_PACKET_SET_RPM				0x03	//ERPM指令(int32)
#define CAN_PACKET_STATUS				0x09	//STATUS_1: ERPM、電流、デューティ
#define CAN_PACKET_STATUS_ANGLE			0x10	//STATUS_4: FET温度、モーター温度など

//C610(M2006) / C620(M3508)のID
#define C6X0_TX_ID_LOW			0x200	//電流指令の送信ID。ESC ID 1〜4 (0x201〜0x204) の分をまとめて送る
#define C6X0_TX_ID_HIGH			0x1FF	//電流指令の送信ID。ESC ID 5〜8 (0x205〜0x208) の分をまとめて送る
#define C6X0_RX_ID_MIN			0x201	//フィードバックの受信ID (ESC ID 1)
#define C6X0_RX_ID_MAX			0x208	//フィードバックの受信ID (ESC ID 8)
#define C610_CURRENT_MAX		10000.0f	//C610の電流指令の上限[mA](-10000〜10000 = -10A〜10A)
#define C620_CURRENT_MAX		20000.0f	//C620の電流指令の上限[mA](-16384〜16384 = -20A〜20A)
#define C620_CURRENT_RAW_MAX	16384.0f

#define CAN2_ID_VALID(drv, id)	((drv) == DRIVER_VESC ? ((id) <= 0xFF) : ((id) >= C6X0_RX_ID_MIN && (id) <= C6X0_RX_ID_MAX))

#if !CAN2_ID_VALID(MOTOR0_DRIVER, MOTOR0_CAN2_ID)
#error "MOTOR0_CAN2_ID is out of range for MOTOR0_DRIVER"
#endif
#if !CAN2_ID_VALID(MOTOR1_DRIVER, MOTOR1_CAN2_ID)
#error "MOTOR1_CAN2_ID is out of range for MOTOR1_DRIVER"
#endif
#if !CAN2_ID_VALID(MOTOR2_DRIVER, MOTOR2_CAN2_ID)
#error "MOTOR2_CAN2_ID is out of range for MOTOR2_DRIVER"
#endif
#if !CAN2_ID_VALID(MOTOR3_DRIVER, MOTOR3_CAN2_ID)
#error "MOTOR3_CAN2_ID is out of range for MOTOR3_DRIVER"
#endif

extern const uint8_t motor_driver[MOTOR_MAX];
extern const uint32_t motor_can2_id[MOTOR_MAX];
extern const uint32_t motor_cmd_rx_id[MOTOR_MAX];
extern const uint32_t motor_fb_tx_id[MOTOR_MAX];

bool Motor_Use_Vesc_Rpm(int i);
void send_motor_current_data(void);
void send_feedback_data(int i);
void send_limit_switch_data(void);

#endif
