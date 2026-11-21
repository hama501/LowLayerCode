#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include "main.h"

//============================================================
// 設定はこのファイルにまとめてある (IDはcan_send.h)
//============================================================

//タイマーの宣言(エンコーダー・サーボで&htimXとして使う。CubeMXで有効にしたものだけ実際に使える)
extern TIM_HandleTypeDef htim1, htim2, htim3, htim4, htim5, htim8;

//==================== モーター ====================

//モードごとの使う個数(合計4個まで)
#define CURRENT_MOTOR_NUM	0	//電流制御
#define VELOCITY_MOTOR_NUM	0	//速度制御
#define POSITION_MOTOR_NUM	1	//位置制御
#define GRIP_MOTOR_NUM		0	//把持
//インデックスは上から順番に割り当てられる

//ドライバーの種類
#define DRIVER_VESC			0
#define DRIVER_C610			1	//M2006
#define DRIVER_C620			2	//M3508

//インデックスごとのドライバー
#define MOTOR0_DRIVER		DRIVER_C610
#define MOTOR1_DRIVER		DRIVER_VESC
#define MOTOR2_DRIVER		DRIVER_VESC
#define MOTOR3_DRIVER		DRIVER_VESC

//モーター→出力軸の減速比。ドライバーごとに用意しておき、MOTORn_DRIVERで使う値が選ばれる
//VESCのとき
#define MOTOR0_VESC_GEAR_RATIO	1.0f
#define MOTOR1_VESC_GEAR_RATIO	1.0f
#define MOTOR2_VESC_GEAR_RATIO	1.0f
#define MOTOR3_VESC_GEAR_RATIO	1.0f
//C610のとき(M2006 = 36)
#define MOTOR0_C610_GEAR_RATIO	36.0f
#define MOTOR1_C610_GEAR_RATIO	36.0f
#define MOTOR2_C610_GEAR_RATIO	36.0f
#define MOTOR3_C610_GEAR_RATIO	36.0f
//C620のとき(M3508 = 19、正確には3591/187)
#define MOTOR0_C620_GEAR_RATIO	19.0f
#define MOTOR1_C620_GEAR_RATIO	19.0f
#define MOTOR2_C620_GEAR_RATIO	19.0f
#define MOTOR3_C620_GEAR_RATIO	19.0f

//MOTORn_DRIVERに合わせて上の値から選ぶ(ここは変えない)
#define DRIVER_SELECT(drv, vesc, c610, c620)	((drv) == DRIVER_VESC ? (vesc) : ((drv) == DRIVER_C610 ? (c610) : (c620)))
#define MOTOR_GEAR_RATIO	{DRIVER_SELECT(MOTOR0_DRIVER, MOTOR0_VESC_GEAR_RATIO, MOTOR0_C610_GEAR_RATIO, MOTOR0_C620_GEAR_RATIO), DRIVER_SELECT(MOTOR1_DRIVER, MOTOR1_VESC_GEAR_RATIO, MOTOR1_C610_GEAR_RATIO, MOTOR1_C620_GEAR_RATIO), DRIVER_SELECT(MOTOR2_DRIVER, MOTOR2_VESC_GEAR_RATIO, MOTOR2_C610_GEAR_RATIO, MOTOR2_C620_GEAR_RATIO), DRIVER_SELECT(MOTOR3_DRIVER, MOTOR3_VESC_GEAR_RATIO, MOTOR3_C610_GEAR_RATIO, MOTOR3_C620_GEAR_RATIO)}

//VESCのときに角度・位置・速度を出すエンコーダー(&htim1など。なければENCODER_NONE)
//C610/C620はCAN2のフィードバックから計算するので、ここの設定は使われない
#define ENCODER_NONE		NULL
#define MOTOR0_ENCODER		&htim1
#define MOTOR1_ENCODER		ENCODER_NONE
#define MOTOR2_ENCODER		ENCODER_NONE
#define MOTOR3_ENCODER		ENCODER_NONE

#define ENCODER_VEL_FILTER	0.2f	//エンコーダーから出す速度のなめらかさ(0〜1、1でフィルターなし)

//モーターごとの設定 {軸0, 軸1, 軸2, 軸3}
#define MOTOR_DIRECTION		{1.0f, 1.0f, 1.0f, 1.0f}	//回転方向(1 / -1)
#define MOTOR_GEAR_LENGTH	{120.95131716f, 120.95131716f, 120.95131716f, 120.95131716f}	//出力軸1回転で進む量(mmで書けばmm、2πならrad)
#define MOTOR_CURRENT_MAX	{10000.0f, 10000.0f, 10000.0f, 10000.0f}	//電流上限[mA]
#define MOTOR_VEL_MAX		{1000.0f, 300.0f, 300.0f, 300.0f}	//速度上限
#define MOTOR_P_GAIN		{101.209938f, 300.0f, 300.0f, 300.0f}	//速度PID P
#define MOTOR_I_GAIN		{1935.172f, 30.0f, 30.0f, 30.0f}		//速度PID I
#define MOTOR_D_GAIN		{0.0f, 0.0f, 0.0f, 0.0f}			//速度PID D
#define MOTOR_POS_P_GAIN	{19.1203747f, 5.0f, 5.0f, 5.0f}			//位置P
#define MOTOR_POS_STOP		{0.05f, 0.1f, 0.1f, 0.1f}			//位置誤差がこれ未満で停止
#define MOTOR_POS_RESTART	{0.05f, 0.5f, 0.5f, 0.5f}			//位置誤差がこれを超えたら再び動く
#define MOTOR_POS_MIN_VEL	{0.0f, 0.0f, 0.0f, 0.0f}			//位置制御の最低速度(0で無効)
#define MOTOR_VESC_RPM		{0, 0, 0, 0}	//1: VESC内部の速度制御(SET_RPM)を使う(VESCの速度・位置モードのみ)

#define MOTOR_POLE_PAIRS	7.0f	//VESCモーターの極対数

//==================== サーボ・リミットスイッチのピン ====================

//基板のコネクターはサーボとリミットスイッチで同じピンを使うので、コネクターごとにどちらに使うかを選ぶ
//サーボのときは電源ジャンパーを5V、リミットスイッチのときは3.3Vにする
#define IO_UNUSED			0	//使わない
#define IO_SERVO			1	//サーボ
#define IO_LIMIT_SW			2	//リミットスイッチ

#define GPIO1_USE			IO_SERVO	//GPIO_1 (PB8、TIM2_CH1)
#define GPIO2_USE			IO_SERVO	//GPIO_2 (PB9、TIM2_CH2)
#define EX2_USE				IO_UNUSED	//EX-2 (PA15、TIM2_CH1)
#define EX3_USE				IO_UNUSED	//EX-3 (PB10、TIM2_CH3)
//サーボの番号・リミットスイッチの番号は、上のコネクターから順に0, 1, 2…と割り当てられる
//GPIO_1とEX-2は同じTIM2_CH1なので、両方をIO_SERVOにはできない
//EX-1 (PA9) はエンコーダー(TIM1_CH2)に使っているので選べない

#define SERVO_MAX_NUM		2	//サーボの最大数(0〜4)。これより多くIO_SERVOを選ぶとビルドエラー
#define LIMIT_SW_MAX_NUM	0	//リミットスイッチの最大数(0〜4)。これより多くIO_LIMIT_SWを選ぶとビルドエラー

//使う個数(上の選び方から自動で数える。ここは変えない)
#define SERVO_NUM			((GPIO1_USE == IO_SERVO) + (GPIO2_USE == IO_SERVO) + (EX2_USE == IO_SERVO) + (EX3_USE == IO_SERVO))
#define LIMIT_SW_NUM		((GPIO1_USE == IO_LIMIT_SW) + (GPIO2_USE == IO_LIMIT_SW) + (EX2_USE == IO_LIMIT_SW) + (EX3_USE == IO_LIMIT_SW))

//==================== サーボ ====================

//TIM2は1カウント=1µs、周期20ms(Prescaler 83、Counter Period 19999)に設定済み

//サーボごとの設定 {サーボ0, サーボ1, サーボ2, サーボ3}
#define SERVO_PULSE_MIN		{1000, 1000, 1000, 1000}	//パルス幅の最小[µs]
#define SERVO_PULSE_MAX		{2000, 2000, 2000, 2000}	//パルス幅の最大[µs]
#define SERVO_ANGLE_MAX		{180.0f, 180.0f, 180.0f, 180.0f}	//最大角度[deg]
#define SERVO_INIT_ANGLE	{0.0f, 0.0f, 0.0f, 0.0f}	//起動時の角度[deg]
#define SERVO_INIT_MODE		{0.0f, 0.0f, 0.0f, 0.0f}	//起動時の向き(0: 角度が増えるとパルスが減る / 1: 増える)

//==================== リミットスイッチ ====================

#define LIMIT_SW_DEBOUNCE_MS	5	//この時間同じ状態が続いたら確定(チャタリング対策)

//スイッチごとの設定 {スイッチ0, スイッチ1, スイッチ2, スイッチ3}
#define LIMIT_SW_ACTIVE		{GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET}	//押されたときのピンの状態
#define LIMIT_SW_MOTOR		{-1, -1, -1, -1}	//止めるモーターの番号(-1: 止めない、状態を送るだけ)
#define LIMIT_SW_DIR		{1, 1, 1, 1}		//押されている間に止める向き(1: +方向、-1: -方向)
#define LIMIT_SW_ORIGIN		{0, 0, 0, 0}		//1: 押された瞬間にそのモーターの位置を0にする(原点出し)

#endif
